# ESP32 FreeRTOS Intention Detection Firmware

## Español

Este repositorio contiene el firmware desarrollado para la ESP32 dentro del proyecto TFG.

El sistema simula lecturas de un sensor de fuerza/par, filtra las señales mediante un filtro EMA, detecta estados de intención del usuario, supervisa condiciones de parada de emergencia y envía comandos discretos mediante comunicación UART hacia el sistema ROS 2.

### Características principales

- Sistema embebido basado en ESP32.
- Arquitectura multitarea con FreeRTOS.
- Simulación de señales de fuerza/par.
- Filtrado mediante media móvil exponencial (EMA).
- Detección de intención del usuario.
- Supervisión de parada de emergencia (E-STOP).
- Comunicación UART con ROS 2.

### Comandos generados

La ESP32 envía comandos discretos con el siguiente formato:

```text
CMD:NORMAL
CMD:SLOW
CMD:FAST
CMD:STOP
CMD:TURN_LEFT
CMD:TURN_RIGHT
CMD:ESTOP

# ESP32 FreeRTOS Intention Detection Firmware

This repository contains the ESP32 firmware developed for the TFG project.

The system simulates force/torque sensor readings, filters the signals using an EMA filter, detects user intention states, supervises emergency stop conditions, and sends discrete commands through UART.

## Main features

- ESP32-based embedded system
- FreeRTOS multitask architecture
- Simulated force/torque signal acquisition
- EMA filtering
- Intention detection
- Emergency stop supervision
- UART communication with ROS 2

## Generated commands

The ESP32 sends discrete commands using the following format:

```text
CMD:NORMAL
CMD:SLOW
CMD:FAST
CMD:STOP
CMD:TURN_LEFT
CMD:TURN_RIGHT
CMD:ESTOP
