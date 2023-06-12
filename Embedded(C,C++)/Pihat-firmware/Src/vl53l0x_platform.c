/*
 * VL53L0X_Platform.c
 *
 *  Created on: 20-Apr-2021
 *      Author: Prakhar Goel
 */

#include <vl53l0x_platform.h>
#include "timer.h"
#include "i2c.h"
GPIO_InitTypeDef GPIO_InitStruct = {0}; //GPIO_init_typedef


/**
  * @brief  Read the specified input port pin from the particular sensor
  * @param  sensor_id to read the particular sensor pin in integer -> 1,2,3,4,5,6,etc...
  * @retval The input port pin value.
  */
GPIO_PinState GPIO_read(int sensor_id)
{
	return(HAL_GPIO_ReadPin(TOF_sensor[sensor_id-1].port, TOF_sensor[sensor_id-1].pin));
}



/**
  * @brief  Set or clear the selected data port bit.
  *
  * @note   This function uses GPIOx_BSRR and GPIOx_BRR registers to allow atomic read/modify
  *         accesses. In this way, there is no risk of an IRQ occurring between
  *         the read and the modify access.
  *
  * @param  sensor_id specifies which XSHUT pin to write
  * @param  state specifies the value to be written to the selected bit.
  *         This parameter can be one of the GPIO_PinState enum values:
  *            @arg GPIO_PIN_RESET: to clear the port pin
  *            @arg GPIO_PIN_SET: to set the port pin
  * @retval None
  */

void GPIO_write(int sensor_id, uint8_t State)
{
   HAL_GPIO_WritePin(TOF_sensor[sensor_id-1].port, TOF_sensor[sensor_id-1].pin, State);
}


/**
  * @brief Changes the mode of the XSHUT pin to input mode
  * @param  sensor_id in integer -> 1,2,3,4,5,6,etc...
  * @retval None
  */
void GPIO_input_state(int sensor_id)
{
	 GPIO_InitStruct.Pin = TOF_sensor[sensor_id-1].pin;
     GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
     GPIO_InitStruct.Pull = GPIO_NOPULL;
     HAL_GPIO_Init(TOF_sensor[sensor_id-1].port, &GPIO_InitStruct);

}


/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  DevAddress Target device address
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_memory_read(uint16_t DevAddress, uint16_t MemAddress,uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	//return (HAL_I2C_Mem_Read(&hi2c1, DevAddress<<1|(0x01), MemAddress, MemAddSize, pData, Size, Timeout));
	return (I2C_memory_read_micro(&hi2c1, DevAddress<<1|(0x01), MemAddress, MemAddSize, pData, Size, Timeout));
}

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  DevAddress Target device address
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_memory_write(uint16_t DevAddress, uint16_t MemAddress,uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	//return (HAL_I2C_Mem_Write(&hi2c1, DevAddress<<1 , MemAddress, MemAddSize, pData, Size, Timeout));
	return (I2C_memory_write_micro(&hi2c1, DevAddress<<1 , MemAddress, MemAddSize, pData, Size, Timeout));
}

/**
  * @brief  Transmits in master mode an amount of data in blocking mode.
  * @param  DevAddress Target device address:
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size,uint32_t Timeout)
{
	//return (HAL_I2C_Master_Transmit(&hi2c1, DevAddress<<1, pData, Size, Timeout));
	return (I2C_transmit_micro(&hi2c1, DevAddress<<1, pData, Size, Timeout));
}

/**
  * @brief  Receives in master mode an amount of data in blocking mode.
  * @param  DevAddress Target device address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
HAL_StatusTypeDef I2C_receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size,uint32_t Timeout)
{
	//return (HAL_I2C_Master_Receive(&hi2c1, DevAddress<<1|(0x01), pData, Size, Timeout));
	return (I2C_Receive_micro(&hi2c1, DevAddress<<1|(0x01), pData, Size, Timeout));

}

/**
  * @brief Send an amount of data in blocking mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @note When FIFO mode is enabled, writing a data in the TDR register adds one
  *       data to the TXFIFO. Write operations to the TDR register are performed
  *       when TXFNF flag is set. From hardware perspective, TXFNF flag and
  *       TXE are mapped on the same bit-field.
  * @param pData   Pointer to data buffer (u8 or u16 data elements).
  * @param Size    Amount of data elements (u8 or u16) to be sent.
  * @param Timeout Timeout duration.
  * @retval HAL status
  */
HAL_StatusTypeDef UART_Transmit(uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	return HAL_UART_Transmit(&hlpuart1, pData, Size, Timeout);
}


/**
  * @brief This function provides minimum delay (in milliseconds) based
  *        on variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @param Delay specifies the delay time length, in milliseconds.
  * @retval None
  */

__weak void delay(uint32_t time)
{
	HAL_Delay(time);
}

/**
  * @brief Provides a tick value in millisecond.
  * @note This function is declared as __weak to be overwritten in case of other
  *       implementations in user file.
  * @retval tick value
  */
__weak uint32_t get_tick(void)
{
	return HAL_GetTick();

}

