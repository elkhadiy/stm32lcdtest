#ifndef _I2C_H_
#define _I2C_H_

#include "stm32f4xx_hal.h"

/* Definition for I2Cx clock resources */
#define I2Cx                             I2C3
#define I2Cx_CLK_ENABLE()                __HAL_RCC_I2C3_CLK_ENABLE()
#define I2Cx_SDA_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOC_CLK_ENABLE()
#define I2Cx_SCL_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE() 

#define I2Cx_FORCE_RESET()               __HAL_RCC_I2C3_FORCE_RESET()
#define I2Cx_RELEASE_RESET()             __HAL_RCC_I2C3_RELEASE_RESET()

/* Definition for I2Cx Pins */
#define I2Cx_SCL_PIN                    GPIO_PIN_8
#define I2Cx_SCL_GPIO_PORT              GPIOA
#define I2Cx_SCL_AF                     GPIO_AF4_I2C3
#define I2Cx_SDA_PIN                    GPIO_PIN_9
#define I2Cx_SDA_GPIO_PORT              GPIOC
#define I2Cx_SDA_AF                     GPIO_AF4_I2C3
#define I2Cx_IT_PIN						GPIO_PIN_8
#define I2Cx_IT_GPIO_PORT				GPIOC
#define I2Cx_WAKEUP_PIN					GPIO_PIN_9
#define I2Cx_WAKEUP_GPIO_PORT			GPIOA

#define I2C_ADDRESS 0x70

#define I2C_TIMEOUT 10000

/**
 * Configure the I2C peripheral
 */
void I2C_Config(I2C_HandleTypeDef *I2cHandle);

/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources : 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);

#endif // _I2C_H__
// EOF
