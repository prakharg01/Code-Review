# Code-Review

This repository contains code snippets organized into three different categories for a code review. Please note that some code may be incomplete or not shareable due to proprietary or confidentiality reasons. I have tried keeping a varity rather than depth.

## Sections

### Embedded C, C++

This section includes firmware for communicating with hardware, specifically targeting STM32-based systems. It also contains PCB designs for the boards used in the projects.

The RDB/Pihat for ‘Stoch Series’ Quadruped robots is a 4-layer board that handles communication-related tasks for the various actuators, sensors, and onboardcomputers like Jetson and Raspberry Pi* (4b or 3b+). STM32G474 is the main microcontroller on the board withclock speeds up to 170 Mhz. RDB has various communication interfaces like RS485, CAN-FD, i2c, SPI, Ethernet,and UART with ADC and GPIO functionalities to make the onboard computers more powerful and provide complete support to handle various sensors and actuators. This board is based on the open-source hardware design of the flight controller Pixhawk 4.

The firmware is written on an STM32G4 series microcontroller board for handling the i/o tasks and communicating with an onboard SBC (Raspberry Pi* (4b or 3b+) via SPI protocol). This used VL53L0X tof sensor, B3M servo motors.

**Note** Generally used C, and some places direct baremetal coding for best performance. I have only added code/designs developed by me. You can find a seperate project where I implemented steady state kalman filter on Labview using STM32: https://github.com/prakharg01/SteadyStateKalmanFilter-LabView

### Matlab

The Matlab section contains a potential field local planner that has been custom developed. It also includes simulation results of the Timed Elastic Band (TEB) algorithm for 3D obstacles and unmanned aerial vehicles (UAVs).


### C++

In this section, you will find a PSO-based PD autotuner for tuning impedance control gains on a quadruped leg. The autotuner utilizes Particle Swarm Optimization (PSO) to find optimal control parameters.

**Note** Since this was part of a larger code base, which is not shareable, I have not included Compile files, Cmake files and Unit testing

## Instructions for Reviewers

Thank you for taking the time to review my code snippets. Below are some guidelines to help you navigate through the repository:

1. Each section is organized into its respective folder. Feel free to explore the code and associated files within each folder.
2. If a code snippet is incomplete or not shareable, it will be clearly indicated in the respective folder's README file.
3. You can find more detailed information, explanations, and instructions within the code files themselves.
4. If you have any questions or need further clarification, please don't hesitate to reach out to me.
5. I have not added ROS/ROS2 projects here, have shared a complete repo that I have been maintaining and developing leveraging Moveit2 and ROS2 control for a custom designed manipulator https://github.com/prakharg01/stoch_manipulator_ros

## Contact Information

If you have any questions or would like to discuss any aspect of the code, please feel free to contact me.

- Name: Prakhar Goel
- Email: prakhs00@gmail.com
- LinkedIn: https://www.linkedin.com/in/prakhar-goel-4616501a5/

Thank you again for considering my code for review. I appreciate your time and feedback!
