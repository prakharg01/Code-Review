/*
 * VL53L0X_Platform.h
 *
 *  Created on: 20-Apr-2021
 *      Author: Prakhar Goel
 */

#ifndef INC_VL53L0X_PLATFORM_H_
#define INC_VL53L0X_PLATFORM_H_

#include <stdbool.h>
#include"stm32g4xx_hal.h"
#include"stm32g474xx.h"
#include"vl53l0x_config.h"
#include<stdio.h>
#include<stdint.h>
#include<stdlib.h>
#include<stdarg.h>


I2C_HandleTypeDef hi2c1; //I2C handle typedef
UART_HandleTypeDef hlpuart1;//UART handle typedef

GPIO_PinState GPIO_read(int sensor_id);
void GPIO_write(int sensor_id, uint8_t State);
void GPIO_input_state(int sensor_id);
HAL_StatusTypeDef I2C_memory_read(uint16_t DevAddress, uint16_t MemAddress,uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_memory_write(uint16_t DevAddress, uint16_t MemAddress,uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size,uint32_t Timeout);
HAL_StatusTypeDef I2C_receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size,uint32_t Timeout);
HAL_StatusTypeDef UART_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout);
__weak void delay(uint32_t time);
__weak uint32_t get_tick(void);


#endif /* INC_VL53L0X_PLATFORM_H_ */
