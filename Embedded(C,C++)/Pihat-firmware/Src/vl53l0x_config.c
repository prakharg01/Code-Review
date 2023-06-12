/*
 * vl53l0x_config.c
 *
 *  Created on: 01-May-2021
 *      Author: Prakhar Goel
 */
#include"vl53l0x_config.h"

struct TOF_sensors TOF_sensor[6]=
{
	{.port=GPIOC , .pin=GPIO_PIN_4},//sensor 1 XSHUT
	{.port=GPIOC , .pin=GPIO_PIN_9},//sensor 2 XSHUT
	{.port=GPIOC , .pin=GPIO_PIN_2},//sensor 3 XSHUT
	{.port=GPIOB , .pin=GPIO_PIN_3},//sensor 4 XSHUT
	{.port=GPIOB , .pin=GPIO_PIN_5},//sensor 5 XSHUT
	{.port=GPIOB , .pin=GPIO_PIN_4} //sensor 6 XSHUT
};


