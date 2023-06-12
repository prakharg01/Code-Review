/*
 * i2c.c
 *
 *  Created on: Jun 11, 2021
 *      Author: Prakhar Goel
 */



#include "i2c.h"
HAL_StatusTypeDef I2C_Receive_micro(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size,
                                         uint32_t Timeout)
{
  uint32_t tickstart;

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = get_tick_micro();

    if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, (25U) , tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_MASTER;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c->XferCount > 255U)
    {
      hi2c->XferSize = 255U;
      TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
    }

    while (hi2c->XferCount > 0U)
    {
      /* Wait until RXNE flag is set */
      if (WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Read data from RXDR */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferSize--;
      hi2c->XferCount--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount > 255U)
        {
          hi2c->XferSize = 255U;
          TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }
    }

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is set */
    if (WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
HAL_StatusTypeDef I2C_transmit_micro(I2C_HandleTypeDef *hi2c,uint16_t DevAddress, uint8_t *pData, uint16_t Size,uint32_t Timeout)

{
	{
	  uint32_t tickstart;

	  if (hi2c->State == 0x20U)
	  {
	    /* Process Locked */
	    __HAL_LOCK(hi2c);

	    /* Init tickstart for timeout management*/
	    tickstart = get_tick_micro();

	    if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, (25U)  , tickstart) != HAL_OK)
	    {
	      return HAL_ERROR;
	    }

	    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
	    hi2c->Mode      = HAL_I2C_MODE_MASTER;
	    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

	    /* Prepare transfer parameters */
	    hi2c->pBuffPtr  = pData;
	    hi2c->XferCount = Size;
	    hi2c->XferISR   = NULL;

	    /* Send Slave Address */
	    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
	    if (hi2c->XferCount > 255U)
	    {
	      hi2c->XferSize = 255U;
	      TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);
	    }
	    else
	    {
	      hi2c->XferSize = hi2c->XferCount;
	      TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_WRITE);
	    }

	    while (hi2c->XferCount > 0U)
	    {
	      /* Wait until TXIS flag is set */
	      if (WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
	      {
	        return HAL_ERROR;
	      }
	      /* Write data to TXDR */
	      hi2c->Instance->TXDR = *hi2c->pBuffPtr;

	      /* Increment Buffer pointer */
	      hi2c->pBuffPtr++;

	      hi2c->XferCount--;
	      hi2c->XferSize--;

	      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
	      {
	        /* Wait until TCR flag is set */
	        if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
	        {
	          return HAL_ERROR;
	        }

	        if (hi2c->XferCount > 255U)
	        {
	          hi2c->XferSize = 255U;
	          TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
	        }
	        else
	        {
	          hi2c->XferSize = hi2c->XferCount;
	          TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
	        }
	      }
	    }

	    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
	    /* Wait until STOPF flag is set */
	    if (WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
	    {
	      return HAL_ERROR;
	    }

	    /* Clear STOP Flag */
	    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

	    /* Clear Configuration Register 2 */
	    I2C_RESET_CR2(hi2c);

	    hi2c->State = HAL_I2C_STATE_READY;
	    hi2c->Mode  = HAL_I2C_MODE_NONE;

	    /* Process Unlocked */
	    __HAL_UNLOCK(hi2c);

	    return HAL_OK;
	  }
	  else
	  {
	    return HAL_BUSY;
	  }
	}

}
HAL_StatusTypeDef I2C_memory_write_micro(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = get_tick_micro();

    if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, (25U), tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_TX;
    hi2c->Mode      = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address and Memory Address */
    if (RequestMemoryWrite(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }

    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE */
    if (hi2c->XferCount > 255U)
    {
      hi2c->XferSize = 255U;
      TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
    }

    do
    {
      /* Wait until TXIS flag is set */
      if (WaitOnTXISFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Write data to TXDR */
      hi2c->Instance->TXDR = *hi2c->pBuffPtr;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferCount--;
      hi2c->XferSize--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount > 255U)
        {
          hi2c->XferSize = 255U;
          TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }

    } while (hi2c->XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    if (WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}
HAL_StatusTypeDef I2C_memory_read_micro(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
  uint32_t tickstart;

  /* Check the parameters */
  assert_param(IS_I2C_MEMADD_SIZE(MemAddSize));

  if (hi2c->State == HAL_I2C_STATE_READY)
  {
    if ((pData == NULL) || (Size == 0U))
    {
      hi2c->ErrorCode = HAL_I2C_ERROR_INVALID_PARAM;
      return  HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(hi2c);

    /* Init tickstart for timeout management*/
    tickstart = get_tick_micro();

    if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, (25U) , tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    hi2c->State     = HAL_I2C_STATE_BUSY_RX;
    hi2c->Mode      = HAL_I2C_MODE_MEM;
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

    /* Prepare transfer parameters */
    hi2c->pBuffPtr  = pData;
    hi2c->XferCount = Size;
    hi2c->XferISR   = NULL;

    /* Send Slave Address and Memory Address */
    if (RequestMemoryRead(hi2c, DevAddress, MemAddress, MemAddSize, Timeout, tickstart) != HAL_OK)
    {
      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);
      return HAL_ERROR;
    }

    /* Send Slave Address */
    /* Set NBYTES to write and reload if hi2c->XferCount > MAX_NBYTE_SIZE and generate RESTART */
    if (hi2c->XferCount > 255U)
    {
      hi2c->XferSize = 255U;
      TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_RELOAD_MODE, I2C_GENERATE_START_READ);
    }
    else
    {
      hi2c->XferSize = hi2c->XferCount;
      TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_GENERATE_START_READ);
    }

    do
    {
      /* Wait until RXNE flag is set */
      if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_RXNE, RESET, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Read data from RXDR */
      *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->RXDR;

      /* Increment Buffer pointer */
      hi2c->pBuffPtr++;

      hi2c->XferSize--;
      hi2c->XferCount--;

      if ((hi2c->XferCount != 0U) && (hi2c->XferSize == 0U))
      {
        /* Wait until TCR flag is set */
        if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, tickstart) != HAL_OK)
        {
          return HAL_ERROR;
        }

        if (hi2c->XferCount >255U)
        {
          hi2c->XferSize = 255U;
          TransferConfig(hi2c, DevAddress, (uint8_t) hi2c->XferSize, I2C_RELOAD_MODE, I2C_NO_STARTSTOP);
        }
        else
        {
          hi2c->XferSize = hi2c->XferCount;
          TransferConfig(hi2c, DevAddress, (uint8_t)hi2c->XferSize, I2C_AUTOEND_MODE, I2C_NO_STARTSTOP);
        }
      }
    } while (hi2c->XferCount > 0U);

    /* No need to Check TC flag, with AUTOEND mode the stop is automatically generated */
    /* Wait until STOPF flag is reset */
    if (WaitOnSTOPFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode  = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}


static HAL_StatusTypeDef WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status,
                                                    uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_I2C_GET_FLAG(hi2c, Flag) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((get_tick_micro() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);
        return HAL_ERROR;
      }
    }
  }
  return HAL_OK;
}
static HAL_StatusTypeDef WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == RESET)
  {
    /* Check if a NACK is detected */
    if (IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Check if a STOPF is detected */
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == SET)
    {
      /* Check if an RXNE is pending */
      /* Store Last receive data if any */
      if ((__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == SET) && (hi2c->XferSize > 0U))
      {
        /* Return HAL_OK */
        /* The Reading of data from RXDR will be done in caller function */
        return HAL_OK;
      }
      else
      {
        /* Clear STOP Flag */
        __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

        /* Clear Configuration Register 2 */
        I2C_RESET_CR2(hi2c);

        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }

    /* Check for the Timeout */
    if (((get_tick_micro() - Tickstart) > Timeout) || (Timeout == 0U))
    {
      hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
      hi2c->State = HAL_I2C_STATE_READY;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }
  }
  return HAL_OK;
}
static void TransferConfig(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t Size, uint32_t Mode,
                               uint32_t Request)
{
  /* Check the parameters */
  assert_param(IS_I2C_ALL_INSTANCE(hi2c->Instance));
  assert_param(IS_TRANSFER_MODE(Mode));
  assert_param(IS_TRANSFER_REQUEST(Request));

  /* update CR2 register */
  MODIFY_REG(hi2c->Instance->CR2,
             ((I2C_CR2_SADD | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_AUTOEND | \
               (I2C_CR2_RD_WRN & (uint32_t)(Request >> (31U - I2C_CR2_RD_WRN_Pos))) | I2C_CR2_START | I2C_CR2_STOP)), \
             (uint32_t)(((uint32_t)DevAddress & I2C_CR2_SADD) |
                        (((uint32_t)Size << I2C_CR2_NBYTES_Pos) & I2C_CR2_NBYTES) | (uint32_t)Mode | (uint32_t)Request));
}
static HAL_StatusTypeDef IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)

{
  if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == SET)
  {
    /* Wait until STOP Flag is reset */
    /* AutoEnd should be initiate after AF */
    while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == RESET)
    {
      /* Check for the Timeout */
      if (Timeout != HAL_MAX_DELAY)
      {
        if (((get_tick_micro() - Tickstart) > Timeout) || (Timeout == 0U))
        {
          hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
          hi2c->State = HAL_I2C_STATE_READY;
          hi2c->Mode = HAL_I2C_MODE_NONE;

          /* Process Unlocked */
          __HAL_UNLOCK(hi2c);

          return HAL_ERROR;
        }
      }
    }

    /* Clear NACKF Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

    /* Clear STOP Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

    /* Flush TX register */
    Flush_TXDR(hi2c);

    /* Clear Configuration Register 2 */
    I2C_RESET_CR2(hi2c);

    hi2c->ErrorCode |= HAL_I2C_ERROR_AF;
    hi2c->State = HAL_I2C_STATE_READY;
    hi2c->Mode = HAL_I2C_MODE_NONE;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_ERROR;
  }
  return HAL_OK;
}
static HAL_StatusTypeDef WaitOnSTOPFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == RESET)
  {
    /* Check if a NACK is detected */
    if (IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Check for the Timeout */
    if (((get_tick_micro() - Tickstart) > Timeout) || (Timeout == 0U))
    {
      hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
      hi2c->State = HAL_I2C_STATE_READY;
      hi2c->Mode = HAL_I2C_MODE_NONE;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }
  }
  return HAL_OK;
}
static HAL_StatusTypeDef WaitOnTXISFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS) == RESET)
  {
    /* Check if a NACK is detected */
    if (IsAcknowledgeFailed(hi2c, Timeout, Tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((get_tick_micro() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->ErrorCode |= HAL_I2C_ERROR_TIMEOUT;
        hi2c->State = HAL_I2C_STATE_READY;
        hi2c->Mode = HAL_I2C_MODE_NONE;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
  }
  return HAL_OK;
}
static HAL_StatusTypeDef RequestMemoryRead(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                               uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
  TransferConfig(hi2c, DevAddress, (uint8_t)MemAddSize, I2C_SOFTEND_MODE, I2C_GENERATE_START_WRITE);

  /* Wait until TXIS flag is set */
  if (WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* If Memory address size is 8Bit */
  if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    /* Send Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }
  /* If Memory address size is 16Bit */
  else
  {
    /* Send MSB of Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_MSB(MemAddress);

    /* Wait until TXIS flag is set */
    if (WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Send LSB of Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }

  /* Wait until TC flag is set */
  if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TC, RESET, Timeout, Tickstart) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}
static HAL_StatusTypeDef RequestMemoryWrite(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress,
                                                uint16_t MemAddSize, uint32_t Timeout, uint32_t Tickstart)
{
  TransferConfig(hi2c, DevAddress, (uint8_t)MemAddSize, I2C_RELOAD_MODE, I2C_GENERATE_START_WRITE);

  /* Wait until TXIS flag is set */
  if (WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* If Memory address size is 8Bit */
  if (MemAddSize == I2C_MEMADD_SIZE_8BIT)
  {
    /* Send Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }
  /* If Memory address size is 16Bit */
  else
  {
    /* Send MSB of Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_MSB(MemAddress);

    /* Wait until TXIS flag is set */
    if (WaitOnTXISFlagUntilTimeout(hi2c, Timeout, Tickstart) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Send LSB of Memory Address */
    hi2c->Instance->TXDR = I2C_MEM_ADD_LSB(MemAddress);
  }

  /* Wait until TCR flag is set */
  if (WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_TCR, RESET, Timeout, Tickstart) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

static void Flush_TXDR(I2C_HandleTypeDef *hi2c)
{
  /* If a pending TXIS flag is set */
  /* Write a dummy data in TXDR to clear it */
  if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXIS) != RESET)
  {
    hi2c->Instance->TXDR = 0x00U;
  }

  /* Flush TX register if not empty */
  if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXE) == RESET)
  {
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_TXE);
  }
}
