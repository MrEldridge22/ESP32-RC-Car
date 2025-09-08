# ESP32 RC Car

This project is an ESP32-S3 based remote-controlled (RC) car that utilizes the ESP-NOW communication protocol for wireless control. The project is designed to provide a robust, low-latency, and efficient way to control an RC car using ESP32 microcontrollers.

## Features
- **Platform:** ESP32-S3
- **Wireless Protocol:** ESP-NOW (no WiFi router required)
- **Low-latency communication** for real-time control
- **Expandable**: Easily add sensors or additional features
- **Open source** and customizable

## How It Works
- The ESP32-S3 microcontroller is mounted on the RC car and controls the motors and any additional peripherals.
- Another ESP32 (or compatible device) acts as the remote controller, sending commands to the car using ESP-NOW.
- ESP-NOW enables direct, peer-to-peer communication between ESP32 devices without the need for a WiFi network.

## Project Structure
- `src/` - Main source code for the car's firmware
- `lib/` - Additional libraries (if any)
- `include/` - Header files
- `test/` - Test code
- `platformio.ini` - PlatformIO project configuration

## Getting Started
1. **Hardware Required:**
   - 2x ESP32-S3 development board (1 for the Car and 1 for the Receiver)
   - Adafruit TB6612 1.2A DC/Stepper Motor Driver Breakout Board
   - RC car chassis with motors
   - Power supply (battery)

2. **Software Setup:**
   - Built using VS Code with [PlatformIO](https://platformio.org/)
   - Clone this repository
   - Open the project in PlatformIO-compatible IDE (e.g., VS Code)
   - Connect your ESP32-S3 board
   - Build and upload the firmware

3. **ESP-NOW Pairing:**
   - Ensure both the car and remote controller ESP32 devices are programmed to use ESP-NOW and are paired using their MAC addresses.

## Documentation
- The main firmware logic is in `src/main.cpp`.
- For more details on ESP-NOW, see the [ESP-NOW documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html).

## License
This project is open source and available under the MIT License.
