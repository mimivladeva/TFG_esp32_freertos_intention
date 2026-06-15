/****************************************************
 * ESP32 + FreeRTOS
 * Arnés háptico – versión corregida
 *
 * Prioridades (NO CAMBIADAS):
 * 5 -> Seguridad (Task_SafetyMonitor)   MÁXIMA
 * 4 -> Lectura sensor (Task_SensorRead)
 * 3 -> Filtrado (Task_Filtering)
 * 2 -> Detección de intención (Task_IntentionDetection)
 * 1 -> Comunicación con Jetson (Task_CommUART)
 * 0 -> Debug / diagnóstico (Task_Debug)
 *
 * Periodos / frecuencias (NO CAMBIADOS):
 * - SafetyMonitor: 5 ms
 * - SensorRead:    1 ms
 * - Filtering:     driven by data
 * - Intention:     5 ms
 * - CommUART:      50 ms
 ****************************************************/

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <math.h>


#define RXD2 16
#define TXD2 17


/* ================= SEGURIDAD ================= */

static const float SAFETY_FORCE_N = 25.0f; // Umbral de seguridad (N) 
static const uint32_t SAFETY_PERIOD_MS = 5; // 5ms -> 200 Hz (seguridad rápida)
static const int ESTOP_PIN = 26;  // CAMBIA si el hardware usa otro pin

typedef enum { // Estado del sistema (compartido)
  SYS_OK = 0,
  SYS_ESTOP
} SystemState_t;

volatile SystemState_t g_systemState = SYS_OK; // Lectura rápida desde otras tareas

/* 
 * Flag de evento para que UART mande ESTOP una sola vez.
 * Lo activa SafetyMonitor cuando detecta E-STOP.
 * Lo consume Task_CommUART.
 */
volatile bool g_estopCommandPending = false;

/* ================= ESTRUCTURAS ================= */

typedef struct { 
  float Fx, Fy, Fz;
  float Tx, Ty, Tz;
  uint32_t timestamp;
} SensorRaw_t; 

typedef struct {
  float Fx, Fy, Fz;
  float Tx, Ty, Tz;
  uint32_t timestamp;
} SensorFiltered_t;

typedef enum {
  INTENTION_STOP = 0,
  INTENTION_SLOW, //Fx negativo moderado 
  INTENTION_NORMAL,
  INTENTION_FAST,        //  NUEVO
  INTENTION_TURN_RIGHT,
  INTENTION_TURN_LEFT
} IntentionState_t;

typedef struct {
  IntentionState_t state;
  uint32_t timestamp;
} IntentionMsg_t;

/* ================= COLAS ================= */

QueueHandle_t Queue_SensorRaw;       // SensorRead → Filtering
QueueHandle_t Queue_SensorFiltered;  // Filtering → Safety / Intention
QueueHandle_t Queue_Intention;       // Intention → Comm / Debug

/* ================= TEST ================= */

typedef struct { // Datos TEST brutos de sensor F/T 6DOF 
  float Fx, Fy, Fz;
  float Tx, Ty, Tz;
  uint32_t duration_ms;
  const char* label;
} TestEvent_t;

TestEvent_t testSequence[] = {
  {  1.5, 0,0,0,0, 0.0, 3000, "NORMAL"     },
  { -5.0, 0,0,0,0, 0.0, 4000, "SLOW"       },
  //{-40.0, 0,0,0,0, 0.0, 1000, "ESTOP"      },
    {  1.5, 0,0,0,0, 0.8, 4000, "TURN_RIGHT" },
  {  1.5, 0,0,0,0, 0.0, 4000, "NORMAL"     },
   //{  1.5, 0,0,0,0, 0.0, 4000, "NORMAL"     },
   
   {  6.0, 0,0,0,0, 0.0, 4000, "FAST" },
   {-12.0, 0,0,0,0, 0.0, 4000, "STOP"       },
  {  1.5, 0,0,0,0, 0.0, 4000, "NORMAL"     },
 {  1.5, 0,0,0,0,-0.8, 4000, "TURN_LEFT"  },
  // {  1.5, 0,0,0,0, 0.0, 7000, "NORMAL"     },
  

};

const int TEST_LENGTH = sizeof(testSequence) / sizeof(TestEvent_t);

/* ================= UTILIDADES ================= */

template<typename T>
void flushTypedQueue(QueueHandle_t q) {
  T dummy;
  while (xQueueReceive(q, &dummy, 0) == pdTRUE) {}
}

/* ================= TAREAS ================= */

/* ---------- PRIORIDAD 4: SensorRead ---------- */
void Task_SensorRead(void *pvParameters) {
  SensorRaw_t sample;
  int idx = 0;
  uint32_t start = millis();

  while (1) {
    /* 
     * Si el sistema está en E-STOP:
     * - NO avanzar array
     * - NO publicar nuevas muestras
     * - NO generar nada después del ESTOP
     */
    if (g_systemState == SYS_ESTOP) {
      vTaskDelay(pdMS_TO_TICKS(1)); // mismo periodo
      continue;
    }

    TestEvent_t ev = testSequence[idx];

    sample.Fx = ev.Fx;
    sample.Fy = ev.Fy;
    sample.Fz = ev.Fz;
    sample.Tx = ev.Tx;
    sample.Ty = ev.Ty;
    sample.Tz = ev.Tz;
    sample.timestamp = micros();

    /* Última muestra válida, no FIFO */
    xQueueOverwrite(Queue_SensorRaw, &sample);

    if (millis() - start > ev.duration_ms) {
      idx = (idx + 1) % TEST_LENGTH;
      start = millis();
    }

    vTaskDelay(pdMS_TO_TICKS(1)); // 1 kHz
  }
}

/* ---------- PRIORIDAD 3: Filtering ---------- */
void Task_Filtering(void *pvParameters) {
  SensorRaw_t in;
  SensorFiltered_t out;

  const float alpha = 0.15f;

  float fFx = 0.0f, fFy = 0.0f, fFz = 0.0f;
  float fTx = 0.0f, fTy = 0.0f, fTz = 0.0f;
  bool initialized = false;

  while (1) {
    if (xQueueReceive(Queue_SensorRaw, &in, portMAX_DELAY) == pdTRUE) {

      /* Si ya entró en E-STOP, no seguir propagando pipeline */
      if (g_systemState == SYS_ESTOP) {
        continue;
      }

      if (!initialized) {
        fFx = in.Fx; fFy = in.Fy; fFz = in.Fz;
        fTx = in.Tx; fTy = in.Ty; fTz = in.Tz;
        initialized = true;
      } else {
        fFx = alpha * in.Fx + (1.0f - alpha) * fFx;
        fFy = alpha * in.Fy + (1.0f - alpha) * fFy;
        fFz = alpha * in.Fz + (1.0f - alpha) * fFz;

        fTx = alpha * in.Tx + (1.0f - alpha) * fTx;
        fTy = alpha * in.Ty + (1.0f - alpha) * fTy;
        fTz = alpha * in.Tz + (1.0f - alpha) * fTz;
      }

      out.Fx = fFx; out.Fy = fFy; out.Fz = fFz;
      out.Tx = fTx; out.Ty = fTy; out.Tz = fTz;
      out.timestamp = in.timestamp;

      /* Último valor, sin FIFO */
      xQueueOverwrite(Queue_SensorFiltered, &out);
    }
  }
}

/* ---------- PRIORIDAD 5: SafetyMonitor ---------- */
void Task_SafetyMonitor(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(SAFETY_PERIOD_MS);

  SensorFiltered_t f;
  bool hasSample = false;

  while (1) {

    /* Si ya estamos en E-STOP, mantener latch hardware y no salir */
    if (g_systemState == SYS_ESTOP) {
      digitalWrite(ESTOP_PIN, HIGH);
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
      continue;
    }

    if (xQueueReceive(Queue_SensorFiltered, &f, 0) == pdTRUE) {
      hasSample = true;
    }

    if (hasSample) {
      const float Fmag = sqrtf(f.Fx*f.Fx + f.Fy*f.Fy + f.Fz*f.Fz);

      if (Fmag >= SAFETY_FORCE_N) {
        /* 1) Latch global inmediato */
        g_systemState = SYS_ESTOP;

        /* 2) Activar salida física */
        digitalWrite(ESTOP_PIN, HIGH);

        /* 3) Invalidar completamente el pipeline */
        flushTypedQueue<SensorRaw_t>(Queue_SensorRaw);
        flushTypedQueue<SensorFiltered_t>(Queue_SensorFiltered);
        flushTypedQueue<IntentionMsg_t>(Queue_Intention);

        /* 4) Solicitar a UART que mande CMD: ESTOP una sola vez */
        g_estopCommandPending = true;

        /* NO imprimir CMD aquí */
      }
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/* ---------- PRIORIDAD 2: IntentionDetection ---------- */
void Task_IntentionDetection(void *pvParameters) {
  SensorFiltered_t f;
  IntentionMsg_t msg;

  const float FX_STOP_ENTER = -10.0f;
  const float FX_STOP_EXIT  = -6.0f;
  const float FX_SLOW_ENTER = -4.0f;
  const float FX_SLOW_EXIT  = -2.0f;
  const float TZ_REORIENT   =  0.5f;
  const float FX_FAST_ENTER = 3.0f; // nuevo
  const float FX_FAST_EXIT  = 1.5f;

  IntentionState_t currentState = INTENTION_NORMAL;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(5); // 200 Hz

  while (1) {

    /* En E-STOP no se genera ninguna intención nueva */
    if (g_systemState == SYS_ESTOP) {
      vTaskDelayUntil(&xLastWakeTime, xPeriod);
      continue;
    }

    if (xQueueReceive(Queue_SensorFiltered, &f, 0) == pdTRUE) {

      /* doble protección por si el estado cambió justo aquí */
      if (g_systemState == SYS_ESTOP) {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
        continue;
      }

      if (f.Fx <= FX_STOP_ENTER) {
        currentState = INTENTION_STOP;
      }
      else if (currentState == INTENTION_STOP && f.Fx > FX_STOP_EXIT) {
        currentState = INTENTION_SLOW;
      }
      else if (f.Fx >= FX_FAST_ENTER) {          //  NUEVO
        currentState = INTENTION_FAST;
      }
      else if (f.Tz >= TZ_REORIENT) {
        currentState = INTENTION_TURN_RIGHT;
      }
      else if (f.Tz <= -TZ_REORIENT) {
        currentState = INTENTION_TURN_LEFT;
      }
      else {
        switch (currentState) {
          case INTENTION_SLOW:
            if (f.Fx > FX_SLOW_EXIT) {
              currentState = INTENTION_NORMAL;
            }
            break;

            case INTENTION_FAST:
            if (f.Fx < FX_FAST_EXIT) {
              currentState = INTENTION_NORMAL;
            }
            break;

          case INTENTION_TURN_LEFT:
          case INTENTION_TURN_RIGHT:
            if (fabsf(f.Tz) < TZ_REORIENT) {
              currentState = INTENTION_NORMAL;
            }
            break;

          case INTENTION_STOP:
          case INTENTION_NORMAL:
          default:
            if (f.Fx <= FX_SLOW_ENTER) {
              currentState = INTENTION_SLOW;
            } else {
              currentState = INTENTION_NORMAL;
            }
            break;
        }
      }

      msg.state = currentState;
      msg.timestamp = micros();

      xQueueOverwrite(Queue_Intention, &msg);
    }

    vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }
}

/* ---------- PRIORIDAD 1: CommUART ---------- */
void Task_CommUART(void *pvParameters) {
  IntentionMsg_t msg;
  IntentionState_t lastSent = (IntentionState_t)-1;
  bool estopAlreadySent = false;

  while (1) {

    /*
     * E-STOP domina absolutamente:
     * - vaciar intención residual
     * - mandar ESTOP una sola vez
     * - no dejar pasar ningún otro comando
     */
    if (g_systemState == SYS_ESTOP) {
      flushTypedQueue<IntentionMsg_t>(Queue_Intention);

      if (g_estopCommandPending && !estopAlreadySent) {
       // Serial.println("CMD: ESTOP");
        estopAlreadySent = true;
        g_estopCommandPending = false;
      }

      vTaskDelay(pdMS_TO_TICKS(50)); // NO CAMBIADO
      continue;
    }

    /* Si en el futuro implementas reset, esto rearma la UART */
    estopAlreadySent = false;

    if (xQueueReceive(Queue_Intention, &msg, 0) == pdTRUE) {

      if (msg.state != lastSent) {
        switch (msg.state) {
          case INTENTION_STOP:
            Serial.println("CMD:STOP");
            break;

          case INTENTION_SLOW:
            Serial.println("CMD:SLOW");
            break;

          case INTENTION_FAST:
            Serial.println("CMD:FAST");
            break;

          case INTENTION_NORMAL:
            Serial.println("CMD:NORMAL");
            break;

          case INTENTION_TURN_LEFT:
            Serial.println("CMD:TURN_LEFT");
            break;

          case INTENTION_TURN_RIGHT:
            Serial.println("CMD:TURN_RIGHT");
            break;
        }

        lastSent = msg.state;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}

/* ---------- PRIORIDAD 0: Debug ---------- */
void Task_Debug(void *pvParameters) {
  while (1) {
    vTaskDelay(portMAX_DELAY);
  }
}

/* ================= SETUP ================= */

void setup() {
  Serial.begin(115200); //pc
  //Serial.begin(115200, SERIAL_8N1, RXD2, TXD2);  // 🔴 UART REAL

  delay(500);  // 🔴 IMPORTANTE

  pinMode(ESTOP_PIN, OUTPUT);
  //digitalWrite(ESTOP_PIN, HIGH);  // FORZADO
  digitalWrite(ESTOP_PIN, LOW);

  /* Colas de último valor, no FIFO acumulativo */
  Queue_SensorRaw      = xQueueCreate(1, sizeof(SensorRaw_t));
  Queue_SensorFiltered = xQueueCreate(1, sizeof(SensorFiltered_t));
  Queue_Intention      = xQueueCreate(1, sizeof(IntentionMsg_t));

  if (Queue_SensorRaw == NULL || Queue_SensorFiltered == NULL || Queue_Intention == NULL) {
    while (1) { delay(1000); }
  }

  xTaskCreatePinnedToCore(Task_SafetyMonitor,      "Task_SafetyMonitor",      4096, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(Task_SensorRead,         "Task_SensorRead",         4096, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(Task_Filtering,          "Task_Filtering",          4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(Task_IntentionDetection, "Task_IntentionDetection", 4096, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(Task_CommUART,           "Task_CommUART",           4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(Task_Debug,              "Task_Debug",              2048, NULL, 0, NULL, 0);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
