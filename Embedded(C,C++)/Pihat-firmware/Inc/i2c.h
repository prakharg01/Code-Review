/*
 * i2c.h
 *
 *  Created on: Jun 11, 2021
 *      Author: Prakhar Goel
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_
#include"stm32g4xx_hal.h"
#include "stm32g4xx_hal_i2c.h"
#include"stm32g474xx.h"
#include "timer.h"
HAL_StatusTypeDef I2C_memory_write_micro(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_memory_read_micro(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef I2C_transmit_micro(I2C_HandleTypeDef *hi2c,uint16_t DevAddress, uint8_t *pData, uint16_t Size,uint32_t Timeout);
HAL_StatusTypeDef I2C_Receive_micro(I2C_HandleTypeDef *hi2c,uint16_t DevAddress, uint8_t *pData, uint16_t Size,uint32_t Timeout);
static HAL_StatusTypeDef WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status,uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static void TransferConfig(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Size, uint32_t Mode,uint32_t Request);
static HAL_StatusTypeDef IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef WaitOnTXISFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef RequestMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                               uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef RequestMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                                uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart);
static void Flush_TXDR(I2C_HandleTypeDef *hi2c);




#endif /* INC_I2C_H_ */
