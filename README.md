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
- Key usage of registers, clock trees, and memory maps for STM32 pin configuration.
- Timer preescaling for precise PWM outputs, ultrasonic sensors and interrupt-based non-blocking task control.
- Implementation of UART serial communication with byte-syncing algorithms for byte transmission between **App Bluetooth Client <-> STM32 <-> ESP32**
- I2C communication protocol for IMU and Color sensor communication with the STM32f0Discovery.
- Implementation of encoder based odometry with IMU correction, with digital low pass RC smoothing filters.
- PWM control with PID motor output generation with acceleration curves on velocity references to reduce sudden current loads.
- AGV mode with precise IR line following algorithms and station floor color detection logic control.
- Robust AMR control that works for any route uploaded.
- Intuitive visual cues configuration with LED strip ESP32 libraries.
- ADC readings and conversions from LM35 temperature sensors and battery-level voltage divider measured voltage.
- Extra buzzer christmas carols melodies and LED configurations.
- Websockets communication between ESP32 and AMR Webpage.
- Custom function computations for sinusoidal parametric equations for x and y coordinates based on period lengths.
- Combined Chaikin-Gaussian smoothing for custom drawn route generation for AMR mode.
- CSS formatting for aesthetic intuitive UI.
