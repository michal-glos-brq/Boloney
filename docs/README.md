# RC Aircraft Telemetry System

## Project Overview
This project demonstrates a telemetry system for a remotely controlled aircraft using economical sensors and microcontrollers. The main components used are the MPU-6500 and BME280 sensors, coupled with ESP32 microcontrollers. The system facilitates real-time data collection and transmission, enhancing the understanding of aircraft dynamics and environmental conditions during flights.

## Features
- **Real-Time Telemetry Data:** Capture and transmit data on the aircraft's orientation and environmental conditions.
- **Sensor Integration:** Utilizes MPU-6500 for orientation and BME280 for environmental sensing.
- **Communication:** Employs ESP32 microcontrollers to handle data processing and transmission using the ESP-NOW protocol.
- **Data Accuracy:** Implements the Madgwick algorithm to improve the accuracy of orientation data derived from sensors.

## Hardware Requirements
- MPU-6500 Sensor
- BME280 Environmental Sensor
- ESP32 Microcontrollers (Two units: one for the aircraft and one for the ground station)
- Prototyping Board and Connecting Wires
- RC Aircraft

## Software Requirements
- Arduino IDE for programming ESP32 microcontrollers.
- Libraries: `ESP_NOW`, `Wire`, `Adafruit_Sensor`, and `Adafruit_BME280`.

## Installation
1. **Set Up Arduino Environment:**
   - Install the Arduino IDE from [Arduino Website](https://www.arduino.cc/en/software).
   - Ensure the ESP32 board definitions are installed. Instructions available [here](https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/).

2. **Library Installation:**
   - Open Arduino IDE, go to Sketch > Include Library > Manage Libraries.
   - Install `Adafruit_BME280` and other required libraries.

3. **Hardware Setup:**
   - Assemble the sensors and microcontrollers on the prototyping board according to the wiring diagrams provided in the `schematics` folder.

4. **Firmware Flashing:**
   - Connect the ESP32 to your computer.
   - Open the `telemetry_code.ino` file in Arduino IDE.
   - Select the correct port and board from the Tools menu.
   - Upload the code to the ESP32.

## Usage
- Power the RC aircraft and ensure the ground station is active.
- The aircraft will begin transmitting data to the ground station once powered.
- Data received by the ground station can be viewed in real-time through the serial monitor.

## Contributing
As this project is a demonstrator for educational purposes, no further developments are planned. However, contributions or forks for extending functionality or improving existing features are welcome.

## License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.

## Acknowledgements
- Thanks to all contributors who have invested their time in testing and improving this telemetry system.
- Special thanks to the open-source community for providing robust libraries and tools that facilitate this kind of development.