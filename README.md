# ESP32S3 BLE Sensor Project

## Overview

This project is designed to collect gyroscope data from an MPU6050 sensor connected to an ESP32S3 via I2C and store the data in SD card via SPI. The ESP32S3 broadcasts the gyroscope data in real-time via BLE. When an Android app connects to the ESP32S3 via BLE and sends a notification, it receives the data and converts it into coordinate form. The ESP32S3 also stores the MPU6050 data in the SD card in coordinate format regardless of whether the Android app's Bluetooth is connected. When the Android app sends the hexadecimal data "50" to the ESP32S3 via BLE, the ESP32S3 sends the most recently stored file to the Android app.

## Features

- Real-time gyroscope data collection and broadcasting via BLE.
- Data storage on SD card with a file size limit of 500KB.
- Automatic file rotation when the file size limit is reached.
- Coordinate conversion and output on Android app upon receiving data.
- File transmission from ESP32S3 to Android app upon receiving a specific BLE command.

## Hardware Requirements

- ESP32S3 Development Board
- MPU6050 Gyroscope Sensor
- SD Card Module (compatible with SPI)
- Jumper Wires
- Breadboard (optional)

## Software Requirements

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html) development framework
- Android Studio for developing the Android app

## Getting Started

1. **Clone the repository:**
   ```bash
   git clone https://github.com/your-username/esp32s3-ble-sensor.git
   cd esp32s3-ble-sensor

2. **Install ESP-IDF:**
    Follow the instructions on the ESP-IDF GitHub to install and set up the development environment.

3. **Configure the project:**
    Modify the   sdkconfig   file to match your hardware configuration, such as setting the correct I2C and SPI pins.
 
4. **Build and Flash:**
    Use the ESP-IDF command to build and flash the firmware to your ESP32S3:
    ```bash
    idf.py build
    idf.py -p (your_serial_port) flash

5. **Android App:**
    Clone the Android app repository and follow the instructions to build and install the app on your Android device.

## Usage

1. Connect the MPU6050 sensor to the ESP32S3 via I2C.
2. Connect the SD card module to the ESP32S3 via SPI.
3. Power on the ESP32S3 and wait for the BLE service to start.
4. Open the Android app and scan for BLE devices.
5. Connect to the ESP32S3 and send notifications to receive real-time gyroscope data.

## Contributing

Contributions are welcome! Feel free to submit issues, pull requests, or suggest new features.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html) for providing the development framework.
- [MPU6050](https://www.invensense.com/products/motion-tracking-dmp/mpu-6050/) for the gyroscope sensor.
- [SD Card Module](https://www.seeedstudio.com/) for the SD card storage.
- [Android App](https://github.com/your-username/android-app) for the Android app.

## Contact

For any questions or issues, please open an issue on this repository or contact the project maintainer directly.
