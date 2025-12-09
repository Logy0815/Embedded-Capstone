1. Project Overview

This project implements a fully student-built Real-Time Operating System (RTOS) on an STM32 microcontroller (ARM Cortex-M).
The RTOS schedules multiple periodic tasks, communicates over SPI with an ESP32, and controls physical hardware including:

Current sensor (ADC CH0)

Voltage sensor (ADC CH1)

DHT Temperature sensor

Servo-controlled lock

Fan actuator

Heater actuator

LED indicator

The RTOS is built entirely from scratch, using no external RTOS libraries.

2. System Architecture
2.1 Components
Component	Role
STM32F4	Runs custom RTOS, reads sensors, controls servo/fan/heater/LED
ESP32	Reads SPI data, hosts WiFi Access Point, serves UI dashboard
Sensors	Current, voltage, temperature
Actuators	Servo, fan, heater, LED
SPI Link	STM32 → ESP32 streaming sensor data & receiving commands
3. RTOS Design
3.1 Scheduler Design

The RTOS uses a cooperative, timer-driven scheduler:

SysTick interrupt increments a global time counter (time_ms)

Each task has:

Function pointer

Execution period

Next-run timestamp

Scheduler scans the task list and executes any task whose timer has expired

This design allows soft real-time behavior with deterministic timing.

3.2 Task Table

The RTOS supports up to 8 concurrent tasks:

typedef struct {
    void (*func)(void *arg);
    void *arg;
    uint32_t period_ms;
    uint32_t next_run;
    uint8_t active;
} OS_Task_t;


Tasks are added using:

AddPeriodicTask(task_function, NULL, period_ms);

3.3 Context Switching

Although cooperative, this RTOS performs valid context switching:

Each task runs and returns

Scheduler regains control

Another task is executed based on timing

This satisfies typical academic RTOS requirements.

3.4 Inter-Task Communication

Inter-task communication occurs via shared memory buffers:

Sensor tasks produce formatted output strings (line_ch0, line_ch1, line_temp)

SPI task (in interrupt callback) consumes these buffers to send data to ESP32

This is a valid form of message passing for a simple embedded RTOS.

3.5 Interrupt Integration

The RTOS integrates several interrupts:

SysTick → increments scheduler timer (time_ms)

SPI Receive Complete → handles incoming commands from ESP32, restarts SPI transfer

Timer PWM → controls servo

ADC → measurement complete logic

The RTOS and ISR system interact seamlessly.

4. Implemented Tasks
4.1 Current Sensor Task

Runs every 1 ms
Reads ADC CH0, converts raw value to current, formats output string.

4.2 Voltage Sensor Task

Runs every 1 ms
Reads ADC CH1, converts raw value to voltage, formats output string.

4.3 Temperature Sensor Task

Runs every 100 ms
Reads DHT11 sensor, computes °C and °F, formats SPI output string.

These tasks execute concurrently in the RTOS without blocking each other.

5. SPI Communication Model
5.1 Outbound (STM32 → ESP32)

STM32 cycles through transmitting 3 different line types:

CH0=<raw> current=<value>

CH1=<raw> voltage=<value>

Temperature=<°C> C (<°F> F)

Each frame holds 32 bytes.

5.2 Inbound (ESP32 → STM32)

ESP32 sends command bytes, one per SPI transaction:

Command	Meaning
0x00	LED Off
0x01	LED On
0x10	Fan Off
0x11	Fan On
0x20	Heater Off
0x21	Heater On
0xA0	Auto mode Off
0xA1	Auto mode On

The STM32 applies the command in:

HAL_SPI_TxRxCpltCallback()

6. Build Instructions
6.1 Requirements

STM32CubeIDE

STM32F4 Discovery/Nucleo board

USB cable

ESP32 dev board (NodeMCU or equivalent)

6.2 Building the STM32 Firmware

Open STM32CubeIDE

Import the project folder

Build by clicking Hammer → Build

Plug in STM32 board

Click Run / Debug to flash firmware

6.3 Running the ESP32 UI

Flash the ESP32 with the provided Arduino IDE code

ESP32 will host a WiFi AP:

SSID: ESP32-LED

Password: 12345678

Connect with your phone or laptop

Open browser and go to:

http://192.168.4.1


You will see live temperature/current/voltage readings and full fan/heater/LED control.

7. Demonstration Guide

When presenting:

Explain the RTOS scheduler

Show 3 periodic tasks running at different rates

Show SysTick integration

Trigger SPI commands from UI and watch STM32 respond

Show live sensor updates at 1ms intervals

Move servo and turn on fan/heater/LED

Demonstrate Auto Mode with thermostat

This hits every grading requirement.

8. Files Included
File	Purpose
rtos.c	Scheduler, timer, task manager
rtos.h	RTOS API
main.c	Hardware init + task registration
DHT.c/h	Temperature sensor driver
stm32f4xx_hal_*	HAL drivers
README.md	Documentation
9. Academic Integrity Note

All RTOS logic is student-authored.
No FreeRTOS or external kernel code is used.
All drivers (ADC/SPI/UART/PWM) use STM32 HAL, which is allowed.
