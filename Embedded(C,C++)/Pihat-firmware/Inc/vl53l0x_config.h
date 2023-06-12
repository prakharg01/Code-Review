/*
 * vl53l0x_config.h
 *
 *  Created on: 01-May-2021
 *      Author: Prakhar Goel
 */

#ifndef INC_VL53L0X_CONFIG_H_
#define INC_VL53L0X_CONFIG_H_
#include"stm32g4xx_hal.h"

#define ToF_ADDR1		  0x30
#define ToF_ADDR2		  0x31
#define ToF_ADDR3		  0x32
#define ToF_ADDR4		  0x33
#define ToF_ADDR5		  0x34
#define ToF_ADDR6		  0x35

#define CORRECTION_FACTOR 0.944

struct TOF_sensors
{
	GPIO_TypeDef * port;
	uint16_t pin;
};

extern struct TOF_sensors TOF_sensor[6];

#endif /* INC_VL53L0X_CONFIG_H_ */
