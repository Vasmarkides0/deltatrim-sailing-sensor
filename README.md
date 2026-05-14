# DeltaTrim Sailing Sensor

Wireless sensor network that measures sail orientation on dinghies in real time. ESP32 nodes with IMUs send roll/pitch data over BLE to a receiver hub with a live web dashboard.

Built during InnoX Hong Kong. Selected for incubation with HK$100k seed funding.

## What it does

Attach sensor nodes to a sail. They measure orientation 50 times per second using accelerometer and gyroscope data, fused with a complementary filter. Data streams over Bluetooth to a receiver hub that hosts a live dashboard — open a browser, see your sail angles in real time.

The goal: turn subjective sailing feel into measurable data for coaching.

## Tech stack

- ESP32 microcontrollers — sensor nodes and receiver hub
- MPU-6050 IMU — 6-axis accelerometer/gyroscope
- BLE — wireless communication between nodes and receiver
- Complementary filter — sensor fusion for orientation estimation
- HTML5/JavaScript — real-time web dashboard
- C++ on Arduino framework — firmware

## Project files

**Node.cpp** — sensor node firmware. Reads IMU data, applies complementary filter, transmits over BLE.

**Receiver.cpp** — hub firmware. Scans for BLE nodes, hosts WiFi access point, serves live dashboard.

## Getting started

**Hardware:** ESP32 dev board, MPU-6050 IMU, USB cable for flashing.

**Wiring:** MPU-6050 to ESP32 via I2C (SDA → GPIO 21, SCL → GPIO 22, VCC → 3.3V, GND → GND).

**Flash:** Open Node.cpp, set NODE_ID (1-5) for each sensor, flash to ESP32. Flash Receiver.cpp to a separate ESP32.

**Run:** Power on nodes (keep still for 5s to calibrate). Power on receiver. Connect to WiFi network "LEECH_RECEIVER" (password: 12345678). Open 192.168.4.1 in a browser.

## Built at

InnoX Hong Kong. Concept to working prototype in one month. Vasilis Markides.

## License

MIT
