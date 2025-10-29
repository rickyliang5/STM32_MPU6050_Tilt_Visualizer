
## 🌟 Features

- **Real-time tilt sensing** using MPU6050 accelerometer and gyroscopeee
- **I2C communication** between STM32 and sensor
- **Roll and pitch angle calculation** from accelerometer data
- **UART serial output** at 115200 baud
- **Web-based 3D visualization** using Three.js
- **Web Serial API integration** for direct browser-to-hardware communication
- **Live sensor data display** with visual feedback

## 🛠️ Hardware Requirements

- **STM32 microcontroller** (STM32F4 series recommended, but any STM32 with I2C and UART will work)
- **MPU6050** 6-axis IMU sensor module
- **USB cable** for programming and serial communication
- **Jumper wires** for connections
- **Breadboard** (optional)

## 📐 Wiring Diagram
MPU6050          STM32
┌────────┐      ┌─────────┐
│  VCC   │─────→│  3.3V   │s
│  GND   │─────→│  GND    │
│  SCL   │─────→│  PB8    │  I2C1 SCL)
│  SDA   │─────→│  PB9    │  (I2C1 SDA)
└────────┘      └─────────┘
Note: Your STM32 board may have different I2C pins.
Check your board's pinout and adjust in STM32CubeMX if needed..

### Important Notes:
- ⚠️ **Use 3.3V, NOT 5V!** The MPU6050 is a 3.3V device
- Most MPU6050 modules have built-in pull-up resistors on SCL/SDA
- If your module doesn't have pull-ups, add 4.7kΩ resistors from SCL/SDA to 3.3V

## 🚀 Getting Started

### 1. Hardware Setup

1. Connect MPU6050 to STM32 as shown in wiring diagram
2. Connect STM32 to computer via USB

### 2. Software Setup

#### STM32 Firmware:

1. Open the project in **STM32CubeIDE**
2. Build the project (Ctrl + B)
3. Flash to your STM32 board (Run → Debug or F11)
4. Disconnect from debugger (important for serial port access)

#### Web Visualizer:

1. Start a local web server in the project directory:
```bash
   # Python
   python -m http.server 8000
   
   # Or use VS Code Live Server extension

Open Chrome/Edge browser and go to:

   http://localhost:8000/visualizer/tilt.html

Click "Connect Serial Port"
Select your STM32's COM port
Tilt the sensor and watch the 3D visualization!

📊 Data Format
The STM32 sends data via UART in CSV format:
roll,pitch
15.45,-5.67
20.12,10.34
-5.23,25.89

Baud Rate: 115200
Format: roll,pitch\r\n
Update Rate: 10 Hz (every 100ms)
Units: Degrees

🎮 Usage
Visualizer Controls:

Connect Serial Port - Connect to STM32
Disconnect - Close serial connection
Simulate Data - Test visualization without hardware

What You'll See:

3D board representing your sensor orientation
Red arrow showing forward direction
Grid for spatial reference
Real-time angle values (Roll and Pitch)

Movements:

Tilt left/right → Roll changes
Tilt forward/backward → Pitch changes
Rotate the sensor → Both angles change

🧠 How It Works
1. Sensor Reading (I2C)
c// Read 14 bytes: 6 accel + 2 temp + 6 gyro
HAL_I2C_Mem_Read(&hi2c1, 0xD0, 0x3B, 1, data, 14, 1000);
2. Data Processing
c// Combine two bytes into 16-bit values
int16_t accel_x = (data[0] << 8) | data[1];
int16_t accel_y = (data[2] << 8) | data[3];
int16_t accel_z = (data[4] << 8) | data[5];
3. Angle Calculation
c// Calculate tilt angles from accelerometer
roll = atan2(accel_y, accel_z) * 180 / π
pitch = atan2(-accel_x, √(accel_y² + accel_z²)) * 180 / π
4. Serial Transmission (UART)
c// Send as CSV format
sprintf(msg, "%d.%02d,%d.%02d\r\n", roll/100, roll%100, pitch/100, pitch%100);
HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 100);
5. Web Visualization

Browser connects via Web Serial API
Parses incoming data
Updates Three.js 3D scene in real-time

📁 Project Structure
STM32_MPU6050_Tilt_Visualizer/
├── stm32/                      # STM32 firmware
│   ├── Core/
│   │   ├── Src/
│   │   │   └── main.c          # Main application code
│   │   └── Inc/
│   │       └── main.h
│   ├── Drivers/                # HAL drivers
│   └── *.ioc                   # STM32CubeMX config
├── visualizer/
│   └── tilt.html               # Web-based 3D viewer
├── docs/
│   ├── wiring.png
│   └── screenshots/
├── .gitignore
└── README.md
🔧 Troubleshooting
Sensor Not Detected

✅ Check wiring (especially SCL/SDA)
✅ Verify 3.3V power supply
✅ Check I2C pull-up resistors
✅ Try I2C scanner to detect device

No Serial Port Available

✅ Close STM32CubeIDE debugger
✅ Install ST-Link USB drivers
✅ Check Device Manager for COM port
✅ Try unplugging and replugging USB

Web Serial Not Working

✅ Use Chrome, Edge, or Opera (Firefox/Safari not supported)
✅ Must use localhost or HTTPS (not file://)
✅ Check browser console for errors
✅ Grant serial port permissions

Angles Look Wrong

✅ Sensor should be flat during startup
✅ Check sensor orientation (arrow shows forward)
✅ Verify data format in serial terminal
✅ Try recalibrating by resetting STM32

📚 Technical Details
Communication Protocols
I2C Configuration:

Clock Speed: 100 kHz
Address Mode: 7-bit
MPU6050 Address: 0x68 (0xD0 in 8-bit format)

UART Configuration:

Baud Rate: 115200
Data Bits: 8
Stop Bits: 1
Parity: None
Flow Control: None

MPU6050 Configuration
Accelerometer:

Range: ±2g
Sensitivity: 16,384 LSB/g
Used for: Tilt angle calculation

Gyroscope:

Range: ±250°/s
Sensitivity: 131 LSB/°/s
Used for: Rotation rate monitoring

Update Rate: 10 Hz (100ms between readings)
🎓 Learning Resources
This project demonstrates:

✅ I2C protocol and addressing
✅ Sensor data acquisition and processing
✅ UART serial communication
✅ Byte manipulation and type casting
✅ Fixed-point arithmetic (avoiding floats)
✅ Web Serial API
✅ Real-time 3D graphics with Three.js

🤝 Contributing
Contributions are welcome! Feel free to:

Report bugs
Suggest features
Submit pull requests
Improve documentation

📝 License
MIT License - feel free to use this project for learning and commercial purposes.
🙏 Acknowledgments

STMicroelectronics for HAL libraries
InvenSense for MPU6050 sensor
Three.js for 3D rendering
Web Serial API community

📧 Contact
Ricky Liang

GitHub: @rickyliang5
Email: ricky.liang@sjsu.edu

🎯 Future Enhancements

 Add magnetometer for yaw (compass heading)
 Implement complementary filter for smoother angles
 Add gesture recognition (shake, tap detection)
 OLED display for standalone operation
 Bluetooth/WiFi wireless data transmission
 Data logging to SD card
 Multiple sensor support
 Kalman filter for sensor fusion


⭐ If you found this project helpful, please give it a star!
Made with ❤️ and lots of learning
