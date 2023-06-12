/*
 * register_bank.c
 *
 *  Created on: May 17, 2021
 *      Author: aditya
 */

#include "main.h"


#include "register_bank.h"


static bool sensor_rb_locked = false;
static bool actuator_rb_locked = false;
static bool new_actuator_data = false;

/* Register banks */
uint8_t sensor_rb[SENSOR_REGISTER_SIZE]={0};
uint8_t actuator_rb[ACTUATOR_REGISTER_SIZE] = {0};


/*
 * @brief Function to take a lock on the sensor register bank.
 *
 * @note The register bank cannot be updated with new
 * data if it is locked
 */
void take_sensor_register_lock()
{
	sensor_rb_locked = true;
}

/*
 * @brief Function to release the lock on the sensor register bank.
 *
 * @note The register bank can be updated with new
 * data if it the lock is released.
 */
void release_sensor_register_lock()
{
	sensor_rb_locked = false;
}


/*
 * @brief Function to take a lock on the actuator register bank.
 *
 * @note The register bank cannot be updated with new
 * data if it is locked
 */
void take_actuator_register_lock()
{
	actuator_rb_locked = true;
}

/*
 * @brief Function to release the lock on the actuator register bank.
 *
 * @note The register bank can be updated with new
 * data if it the lock is released.
 */
void release_actuator_register_lock()
{
	actuator_rb_locked = false;
}



/*
 * @brief Update the sensor register bank with new sensor data.
 *
 * @note The register bank cannot be updated if it is locked.
 *
 * @param addr: Start address for updating the register bank
 * @param buf: Pointer to the data buffer
 * @param size: Size of data to be updated in the register bank
 */

void write_sensor_data(uint8_t addr, uint8_t* buf, uint8_t size)
{
	if(sensor_rb_locked)
		return;

	for(int i=0; i<size; i++)
	{
		sensor_rb[addr + i] = buf[i];
	}
}


/*
 * @brief Read data from the sensor register bank
 *
 * @param addr: Address of the register which is to be read
 *
 * @retval Data at the specified address
 */
uint8_t read_sensor_data(uint8_t addr)
{
	return sensor_rb[addr];
}


/*
 * @brief Update the actuator register bank with new commands.
 *
 * @param addr: Start address for updating the register bank
 * @param data: Data to be written
 */
void write_actuator_data(uint8_t addr, uint8_t data)
{
	actuator_rb[addr] = data;
	new_actuator_data = true;
}


/*
 * @brief Read data from the sensor register bank
 *
 * @note Data cannot be read if the register bank is locked
 *
 * @param addr: Address of the register which is to be read
 * @param size: Size of data to be read
 * @param buf: Buffer to write data to
 *
 * @retval TRUE if data is read, otherwise FALSE
 */
bool read_actuator_data(uint8_t addr, uint8_t size, uint8_t* buf)
{
	if(actuator_rb_locked)
		return false;

	for(int i=0; i<size; i++)
		buf[i] = actuator_rb[addr + i];

	new_actuator_data = false;

	return true;
}


/*
 * @brief Specifies if new data is available in the actuator register bank
 *
 * @retval TRUE if new data is available , FALSE otherwise
 */
bool is_new_data()
{
	return new_actuator_data;
}

