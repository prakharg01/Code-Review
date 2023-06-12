/*
 * register_bank.h
 *
 *  Created on: May 17, 2021
 *      Author: aditya
 */

#ifndef INC_REGISTER_BANK_H_
#define INC_REGISTER_BANK_H_

#include "stdbool.h"



#define MOTOR_DATA_NUM_BYTES 12
/*
 * id: uint8_t
 * status: uint8_t
 * pos: int16_t
 * vel: int16_t
 * current: int16_t
 * temp: uint16_t
 * voltage: uint16_t
 */


// Start address for motor data
#define MOTOR_ADDR 				0

// Start address for IMU data
#define IMU_ADDR 				(MOTOR_ADDR + 12*MOTOR_DATA_NUM_BYTES)

// Start address for ToF data
#define TOF_ADDR 				(IMU_ADDR + 20)

// Size of the sensor register bank
#define SENSOR_REGISTER_SIZE 	(TOF_ADDR + 12)

// Size of the actuator register bank
#define ACTUATOR_REGISTER_SIZE 	(12*3 + 2) // | ID | POS_H | POS _L | .... | TIME_H | TIME_L |



// Function to take a lock on the sensor register bank.
void take_sensor_register_lock();

// Function to release the lock on the sensor register bank.
void release_sensor_register_lock();

// Function to take a lock on the actuator register bank.
void take_actuator_register_lock();

//Function to release the lock on the actuator register bank.
void release_actuator_register_lock();

// Update the sensor register bank with new sensor data.
void 	write_sensor_data(uint8_t addr, uint8_t* buf, uint8_t size);

// Read data from the sensor register bank
uint8_t read_sensor_data(uint8_t addr);

// Update the actuator register bank with new commands.
void 	write_actuator_data(uint8_t addr, uint8_t data);

// Read data from the sensor register bank
bool 	read_actuator_data(uint8_t addr, uint8_t size, uint8_t* buf);

// Specifies if new data is available in the actuator register bank
bool 	is_new_data();


#endif /* INC_REGISTER_BANK_H_ */
