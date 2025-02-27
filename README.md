UART to UART Bridge

Overview

This project implements a bidirectional UART bridge for ESP32 using FreeRTOS and the ESP-IDF framework. The bridge allows seamless communication between two UART ports, leveraging FreeRTOS queues for efficient and event-driven data transfer.

Features

✅ Bidirectional communication between two UART interfaces.

✅ Interrupt-driven UART event handling for improved performance.

✅ Queue-based synchronization, avoiding the need for mutexes or semaphores.

✅ Configurable debug logs to monitor data forwarding between ports.

Performance Considerations
Queue processing ensures minimal CPU usage and prevents data loss.

Project Structure
```
uart_to_uart_bridge/
├── main/
│   ├── uart_bridge_isr.c  # Main UART bridge implementation with ISR 
│   ├── uart_bridge_pooling.c  # Optional UART bridge implementation without interupts 
│   ├── CMakeLists.txt      # CMake configuration
├── sdkconfig              # ESP-IDF configuration
├── README.md              # Project documentation
└── .gitignore             # Git ignore file
```

Installation

Prerequisites

ESP-IDF installed (export.sh sourced)

ESP32 board connected via USB

Build & Flash
```
idf.py set-target esp32
idf.py build
idf.py flash
idf.py monitor
```
Configuration

You can configure the debug logs by modifying:

#define DEBUG 1  // Set to 0 to disable logging

Usage

The firmware automatically forwards data between UART1 and UART2.

You can monitor UART logs via:
```
idf.py monitor
```
License

This project is released under the MIT License.

Contributions

Feel free to submit pull requests, report issues, or suggest improvements!
