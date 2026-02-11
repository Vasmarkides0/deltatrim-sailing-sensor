# DeltaTrim Sailing Sensor

Wireless IMU sensor network for measuring sail orientation on dinghies. ESP32 nodes with MPU-6050 IMUs send roll/pitch data over BLE to a receiver with a web dashboard.

Built during InnoX Hong Kong (selected for incubation, HK$100k seed funding).

## System Architecture

### Hardware Components
- **Sensor Nodes:** ESP32 microcontrollers with MPU-6050 IMUs
- **Receiver Hub:** ESP32 with WiFi access point and embedded web server
- **Communication:** Bluetooth Low Energy (BLE) with sequential node scanning
- **Power:** Battery-powered nodes for wireless deployment on sailing equipment

### Software Stack
- **Firmware:** C++ on Arduino framework
- **Sensor Fusion:** Complementary filter for orientation estimation
- **Protocol:** Custom binary packet format with CRC16 error detection
- **Visualization:** Real-time HTML5/JavaScript dashboard with live graphs

## Technical Approach

### Complementary Filter
Uses a complementary filter to fuse gyroscope and accelerometer data, rather than a full Kalman filter:

```cpp
// High-pass filtered gyroscope integration + low-pass filtered accelerometer
const float alpha = 0.98f;
roll_deg = alpha * (roll_deg + gx_dps * dt) + (1.0f - alpha) * roll_acc;
```

This provides fast response to motion (from gyroscope) while correcting for drift over time (from accelerometer), with low computational overhead.

### Gyro Bias Calibration
On startup, each node calibrates gyro bias using 200 samples to reduce drift:

```cpp
// ±250 dps sensitivity scale factor for MPU-6050
gyro_bias_x_dps = ((float)sum_gx / (float)N) / 131.0f;
```

### BLE Protocol
Custom packet structure with CRC16 checksum for error detection:
- Version byte for protocol compatibility
- Node ID for multi-sensor deployments
- Timestamp for synchronization
- CRC16-CCITT checksum for error detection

### Web Dashboard
The receiver serves an HTML5 interface with:
- Live 2D tilt visualization
- Real-time graphs of roll and pitch
- Multi-node status table

## Project Files

- **`Node.cpp`** - Sensor node firmware with IMU reading and BLE transmission
- **`Receiver.cpp`** - Hub firmware with BLE scanning, web server, and dashboard

## Building and Flashing

### Requirements
- PlatformIO or Arduino IDE
- ESP32 development board (e.g., ESP32-DevKitC)
- MPU-6050 6-axis IMU module
- USB cable for programming

### Dependencies
```ini
NimBLE-Arduino
Wire (included with Arduino core)
WiFi (included with ESP32 core)
WebServer (included with ESP32 core)
```

### Compilation
1. Open `Node.cpp` in your IDE
2. Set `NODE_ID` define to unique value (1-5) for each sensor
3. Flash to ESP32 connected to IMU via I2C (SDA/SCL pins)
4. Flash `Receiver.cpp` to separate ESP32 (no IMU required)

### Usage
1. Power on sensor nodes (they will auto-calibrate - keep stationary for 5 seconds)
2. Power on receiver hub
3. Connect to WiFi network "LEECH_RECEIVER" (password: 12345678)
4. Navigate to `192.168.4.1` in browser
5. View live data from all connected nodes

## Hardware Setup

### IMU Wiring (I2C)
```
MPU-6050 VCC  → ESP32 3.3V
MPU-6050 GND  → ESP32 GND
MPU-6050 SCL  → ESP32 GPIO 22 (default I2C clock)
MPU-6050 SDA  → ESP32 GPIO 21 (default I2C data)
```

### Sensor Mounting
- Attach nodes rigidly to sail leech (trailing edge)
- Align IMU axes with sail plane for accurate roll/pitch measurement
- Ensure secure waterproof housing for marine environment

## Performance Characteristics

- **Update Rate:** ~50 Hz per sensor node
- **BLE Range:** ~10m line-of-sight

## Background

Built during InnoX Hong Kong as part of the DeltaTrim startup. The goal was to turn subjective sailing feel into measurable data for coaching.

Developed in about a month from concept to working prototype.

## Possible Improvements

- Kalman filtering for better accuracy
- SD card logging for post-session analysis
- GPS integration for absolute position tracking
- Mobile app for iOS/Android
- Machine learning integration for technique analysis
- Extended battery life with deep sleep modes

## License

MIT License - See LICENSE file for details

## Contact

Developed by Vasilis Markides  
MEng Mechanical Engineering, UCL  
[GitHub](https://github.com/Vasmarkides0) • [LinkedIn](https://www.linkedin.com/in/vasilis-markides-477911206/)

---
