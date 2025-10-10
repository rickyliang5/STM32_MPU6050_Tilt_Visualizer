# STM32 MPU6050 Tilt Sensor Visualization

Real-time 3D tilt visualization using STM32 microcontroller and MPU6050 gyroscope/accelerometer sensor.

## Hardware
- STM32F4 (or your board)
- MPU6050 6-axis IMU sensor
- USB-to-UART adapter

## Wiring
MPU6050  →  STM32
VCC      →  3.3V
GND      →  GND
SCL      →  PB8
SDA      →  PB9

## Features
- I2C communication with MPU6050
- Real-time roll and pitch angle calculation
- UART data output (115200 baud)
- Web-based 3D visualization

## Setup
1. Open project in STM32CubeIDE
2. Build and flash to STM32
3. Connect via USB
4. Open `visualizer/tilt.html` in Chrome
5. Connect to serial port

## Data Format
CSV format: `roll,pitch`
Example: `15.45,-5.67`

## Technologies
- STM32 HAL
- I2C Protocol
- UART Communication
- Web Serial API
- Three.js for 3D rendering