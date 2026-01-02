AMR + AGV DIFFERENTIAL ROBOT - OMRON PROJECT (Mechatronics Eng. 5th semester AD25)
===================================
The objective of this project was to design a differential-wheeled robot capable of:
- Being operated manually with a mobile app joystick.
- Operating under a configurable AGV mode with station color detection.
- AMR mode with complete custom route configuration.
Following safety protocols regarding velocity and acceleration curves, energy consumption, emergency stops and awareness of nearby objects.
-----------------------------------
The repository includes the following:
- STM32f0Discovery project files (C)
- ESP32 project files (C++)
- Webpage history logs and final used file. (Javascript, HTML, CSS)
- MIT App Inventor project file for mobile app. (Block-based)
-----------------------------------
Main aspects of the project include:
- Direct register-level configuration of GPIOs using STM32 clock trees and peripheral memory maps.
- Timer prescaling and compare/capture configuration for PWM generation, ultrasonic time-of-flight measurement, and interrupt-driven non-blocking task scheduling.
- Implementation of UART communication with custom frame synchronization and desynchronization recovery between a Bluetooth client, STM32, and ESP32.
- Integration of multiple peripheral sensors using the I²C communication protocol on an STM32F0-based platform.
- Implementation of encoder-based odometry with inertial correction using first-order digital low-pass filtering.
- Closed-loop PID motor control using PWM, with acceleration-limited velocity references to mitigate current spikes and mechanical stress.
- AGV operation mode featuring robust line-following algorithms and floor-based station identification logic.
- Autonomous mobile robot (AMR) control supporting dynamically uploaded navigation routes.
- Implementation of visual feedback cues using addressable LED control libraries on the ESP32.
- ADC acquisition and conversion for temperature monitoring and battery-level measurement using voltage dividers.
- Additional audio and visual feedback features using buzzer-driven melodies and programmable LED patterns.
- Real-time WebSocket communication between the ESP32 and a web-based AMR control interface.
- Custom parametric trajectory generation using sinusoidal functions for x–y coordinate computation.
- Route smoothing for user-drawn paths using combined Chaikin subdivision and Gaussian filtering techniques.
- CSS-based layout and styling for an intuitive and user-friendly web interface.
