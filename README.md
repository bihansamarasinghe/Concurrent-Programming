---

# Gyroscope and Traffic Light Controller

This C++ code defines a gyroscope module and implements a traffic light controller using an Arduino microcontroller. The code utilizes various components and functionalities, including gyroscope sensors, switches, LED control, and seven-segment displays.

## Table of Contents

- [Overview](#overview)
- [Components](#components)
- [How to Use](#how-to-use)
- [Traffic Light States](#traffic-light-states)
- [Gyroscope Axis Data](#gyroscope-axis-data)
- [Contributing](#contributing)
- [License](#license)

## Overview

The code defines a gyroscope module using the MPU6050 sensor and incorporates functionality to control traffic lights in various states. Additionally, it implements seven-segment display control and responds to user input from switches.

## Components

- Arduino Microcontroller
- MPU6050 Gyroscope Sensor
- LED Lights (Red, Green, Yellow)
- Seven-Segment Display
- Resistors and wiring components

## How to Use

1. Connect the MPU6050 gyroscope sensor and LEDs to the Arduino microcontroller following the wiring instructions in the code.

2. Upload the provided C++ code to the Arduino microcontroller using the Arduino IDE.

3. Run the code on the Arduino microcontroller.

4. Observe the gyroscope axis data displayed on the seven-segment display and the traffic light states controlled by the gyroscope data and user input.

## Traffic Light States

The code defines traffic light states based on the gyroscope axis data:

- **Normal State**: Traffic lights operate based on predefined timing.
- **Flat Orientation**: Traffic light timing is adjusted based on the gyroscope data indicating a flat orientation.
- **Upside Down Orientation**: Traffic light timing is adjusted based on the gyroscope data indicating an upside-down orientation.

## Gyroscope Axis Data

The gyroscope axis data is displayed on the seven-segment display, representing different orientations:

- **F (Forward)**: Gyroscope data indicating a forward orientation.
- **U (Upside Down)**: Gyroscope data indicating an upside-down orientation.
- **L (Left)**: Gyroscope data indicating a left orientation.
- **PR (Rotate Right)**: Gyroscope data indicating a right rotation.
- **PL (Rotate Left)**: Gyroscope data indicating a left rotation.

## Contributing

Contributions and improvements are welcome! Feel free to open issues, suggest enhancements, or submit pull requests.

## License

This project is licensed under the [MIT License](LICENSE).

---
