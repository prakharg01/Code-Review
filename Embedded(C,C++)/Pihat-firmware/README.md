## Pinout


### SPI : SPI3
|Function | GPIO | Pin |
| :---: | :---: | :---: |
|MOSI | PC12 | CN7-3 |
|MISO | PC11 | CN7-2 |
|SCK  | PC10 | CN7-1 |
|NSS  | PA6  | CN7-32 |

Note: Do not forget to connect the ground on the 
SPI master (RPi) and the SPI slave device (STM Nucleo).


### I2C : I2C1
|Function | GPIO | Pin |
| :---: | :---: | :---: |
|SDA | PB9  | CN7-21 |
|SCL | PB8 | CN7-17 |


### I2C : I2C2
|Function | GPIO | Pin |
| :---: | :---: | :---: |
|SDA | PA8  | CN7-21 |
|SCL | PA9 | CN7-17 |


### USART : USART3
|Function | GPIO | Pin |
| :---: | :---: | :---: |
| Rx | PB11 | CN10-18 |
| Tx | PB10 | CN10-25 |
| DE | PB14 | CN10-28 |

### USART : USART1
|Function | GPIO | Pin |
| :---: | :---: | :---: |
| Rx | PC5 | CN10-18 |
| Tx | PB6 | CN10-25 |
| DE | PA12 | CN10-28 |


### GPIO
|Function | GPIO | Pin |
| :---: | :---: | :---: |
|XSHUT TOF_1 | PC4 | CN10-6 |
|XSHUT TOF_2 | PC9 | CN10-4 |
|XSHUT TOF_3 | PC2 | CN10-2 |
|XSHUT TOF_4 | PB3 | CN10-1 |
|XSHUT TOF_5 | PB5 | CN10-3 |
|XSHUT TOF_6 | PB4 | CN10-5 |


## Code Structure


# Firmware Code Structure - Hardware Agnostic

This README file provides an overview of the code structure of the embedded firmware, which is designed to be hardware agnostic, specifically targeting the STM32-based controllers. The code is organized in a modular and scalable manner to facilitate easy integration with different hardware platforms.

## Project Structure

The firmware project follows the following structure:

- `main.c`: This file serves as the entry point of the firmware and handles the scheduling of tasks in a round-robin fashion. It initializes the necessary components, sets up interrupts, and manages the task execution flow.

- `{sensor/actuator}_task.c`: This file contains the implementation of the sensor or hardware for the respective application. Generally is implemented as a FSM,communicating with the `{sensor/actuator}.c` file.
  
- `{sensor/actuator}.c`: This file contains the implementation of the sensor or actuator related functionality. It defines a platform-independent task that communicates with the sensors using HAL-like functions, which are vendor-independent. The sensor task interacts with the hardware through the platform-specific sensor driver functions. This file can be used with any microcontroller vendor irrespective of vendor dependent functionality.

- `{sensor/actuator}_platform.c`: This file provides the platform-specific sensor driver functions, which serve as the lowest-level abstraction. It includes the necessary register-level calls to interact with the hardware, making it platform-dependent. By separating the platform-specific code into separate files, the firmware can easily be adapted to different hardware platforms without affecting the higher-level tasks.


## Advantages of the Code Structure

The hardware-agnostic design of the firmware code offers several advantages:

- **Portability**: By separating the platform-specific code from the higher-level tasks, the firmware can be easily ported to different hardware platforms with minimal modifications.

- **Scalability**: The modular structure allows for easy addition or removal of sensor and actuator tasks, without affecting other parts of the codebase. New tasks can be added by following the same pattern and using the vendor-independent HAL-like functions.

- **Code Reusability**: The vendor-independent HAL-like functions ensure code reusability across different projects targeting STM32-based controllers. These functions abstract away the hardware-specific details and provide a consistent interface for interacting with the sensors and actuators.
