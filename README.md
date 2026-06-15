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
