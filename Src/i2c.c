#include "i2c.h"
#include "stm32f4xx_hal.h"

I2C_HandleTypeDef I2cHandle;

extern void Error_Handler(void);

void I2C_Config(I2C_HandleTypeDef *I2cHandle)
{
	
  I2cHandle->Instance             = I2Cx;
  
  I2cHandle->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  I2cHandle->Init.ClockSpeed      = 400000;
  I2cHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  I2cHandle->Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  I2cHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  I2cHandle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  I2cHandle->Init.OwnAddress1     = I2C_ADDRESS;
  I2cHandle->Init.OwnAddress2     = 0xFE;
  
  if(HAL_I2C_Init(I2cHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();    
  }
}


/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{  
  GPIO_InitTypeDef  GPIO_InitStruct;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  /* Enable GPIO TX/RX clock */
  I2Cx_SCL_GPIO_CLK_ENABLE();
  I2Cx_SDA_GPIO_CLK_ENABLE();
  /* Enable I2C1 clock */
  I2Cx_CLK_ENABLE(); 
  
  /*##-2- Configure peripheral GPIO ##########################################*/  
  /* I2C TX GPIO pin configuration  */
  GPIO_InitStruct.Pin       = I2Cx_SCL_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
  GPIO_InitStruct.Alternate = I2Cx_SCL_AF;
  
  HAL_GPIO_Init(I2Cx_SCL_GPIO_PORT, &GPIO_InitStruct);
    
  /* I2C RX GPIO pin configuration  */
  GPIO_InitStruct.Pin = I2Cx_SDA_PIN;
  GPIO_InitStruct.Alternate = I2Cx_SDA_AF;
    
  HAL_GPIO_Init(I2Cx_SDA_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = I2Cx_IT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(I2Cx_IT_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = I2Cx_WAKEUP_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  HAL_GPIO_Init(I2Cx_WAKEUP_GPIO_PORT, &GPIO_InitStruct);
}
