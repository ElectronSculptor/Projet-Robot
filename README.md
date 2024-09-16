
# Autonomous Robot Project

This repository contains the code and documentation for an autonomous robot project developed as part of the *Conception des Systèmes Electroniques 1* course at *École des Mines de Saint-Étienne*. The objective of the project was to design, program, and control a robot that navigates autonomously using a sonar sensor for object detection and distance measurement.

Read the Rapport_Projet_Robot.pdf file to have a better understanding (this file is in French)
## Project Overview

The robot is built using an STM32 Nucleo-L476RG microcontroller. It is equipped with a sonar sensor for navigation and is capable of tracking and maintaining a set distance from an object (20 cm). The system was designed following a finite state machine (FSM) that controls various peripherals, such as motors, sonar, and LEDs.

### Main Features

- **Autonomous Navigation**: The robot can search for and follow an object while maintaining a constant distance of 20 cm.
- **Sonar-based Detection**: Utilizes a LV-MaxSonar-EZ24 sonar sensor for object detection and distance measurement.
- **Motor Control**: PWM signals control the motors for precise movement and turning.
- **State Machine**: Implements a finite state machine (FSM) to manage the robot's behavior.

## Hardware Used

- **STM32 Nucleo-L476RG**: The microcontroller used to run the code and control the robot.
- **Sonar Sensor (LV-MaxSonar-EZ24)**: Used for distance measurement and object detection.
- **Servo Motor (DGServo 9g)**: Controls the angle of the sonar sensor for scanning.
- **Motors**: Controls the movement of the robot via PWM signals.
- **Battery Monitoring**: The robot includes a low-battery warning system using an ADC and LED indicator.

## Functionalities

1. **Battery Monitoring**: Monitors battery levels and turns on an LED if the battery is low.
2. **Movement**: The robot moves forward and can turn left or right based on the sonar input.
3. **Object Tracking**: The sonar scans for an object, aligns the robot to face it, and moves towards it, maintaining a set distance.
4. **Button Control**: The robot starts and stops using a single button.

## System Design

The system is controlled by a finite state machine (FSM) with the following states:

- **Idle**: The robot is waiting to start.
- **Scanning**: The robot scans the environment for an object using the sonar sensor.
- **Calibration**: The robot aligns itself with the detected object.
- **Move**: The robot moves towards the object and keeps a distance of 20 cm.
- **Stop**: The robot stops once it reaches the desired position.

## Code Structure

- **`main.c`**: The main program file controlling the robot.
- **`motor_control.c`**: Functions to control the motors via PWM signals.
- **`sonar_control.c`**: Functions to read from the sonar sensor and calculate distances.
- **`fsm.c`**: Implements the finite state machine for the robot’s operation.

## Getting Started

### Prerequisites

- **STM32CubeIDE**: The project was developed using STM32CubeIDE. Make sure you have it installed and configured with the necessary drivers for the Nucleo board.
- **STM32 Nucleo-L476RG**: Required hardware for running the project.

### Running the Code

1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/robot-project.git
   ```
2. Open the project in STM32CubeIDE.
3. Build the project and flash it to the STM32 Nucleo board.
4. Use the button on the board to start the robot's autonomous operation.

## Future Improvements

- **Sonar Filtering**: Adding a filter to reduce erroneous readings from the sonar.
- **Encoder Usage**: Implementing wheel encoders to improve calibration and reduce navigation errors.

## Contributors

- **Gevorg Ishkhanyan**
- **Ibrahim Hadj-Arab**

