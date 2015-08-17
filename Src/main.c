/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "sdram.h"

#include "stm32f4xx.h"

#include "lcd.h"

#include <stdlib.h>
#include <string.h>
#include "my_stdio.h"
#include "i2c.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef I2cHandle;
#define RXBUFFERSIZE 31
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
extern void defilement(void);
extern void clear_screen(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
const char *b2b(int x)
{
	static char b[9];
	b[0] = '\0';
	int z;
	for (z=128; z > 0; z>>=1)
	{
		strcat(b, ((x & z) == z) ? "1" : "0");
	}
	return b;
}


void Error_Handler(void)
{
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
  printf("Error Handler Called!\n");
  while(1);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */
  Init_SDRAM_Device();
  HAL_LTDC_MspInit(&LtdcHandle);
  LCD_Config();
  clear_screen();
  HAL_I2C_MspInit(&I2cHandle);
  I2C_Config(&I2cHandle);
  printf("STM32F4 Firmware v1.6.0\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("Testing I2C Tactile\n");

  uint8_t aRxBuffer[RXBUFFERSIZE];
  uint8_t nonITMode[] = {0xA4, 0x01};
  uint8_t normalOP[] = {0x00, 0x00};

/*
  while(HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t*)normalOP, 2, I2C_TIMEOUT)!= HAL_OK)
  {
	//printf("Trying a Master Transmit\n");
    if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
    {
      printf("Time out occured failed to write stuff 1 :o\n");
      Error_Handler();
    }
  }
//*/
/*
  while(HAL_I2C_Master_Transmit(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t*)nonITMode, 2, I2C_TIMEOUT)!= HAL_OK)
  {
	//printf("Trying a Master Transmit\n");
    if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
    {
      printf("Time out occured failed to write stuff 2 :o\n");
      Error_Handler();
    }
  }
//*/
 
  HAL_GPIO_WritePin(I2Cx_WAKEUP_GPIO_PORT, I2Cx_WAKEUP_PIN, GPIO_PIN_RESET);
  HAL_Delay(500); // for screen touch
  HAL_GPIO_WritePin(I2Cx_WAKEUP_GPIO_PORT, I2Cx_WAKEUP_PIN, GPIO_PIN_SET);
  

  while (1)
  {	
  /* USER CODE END WHILE */
  /* USER CODE BEGIN 3 */

  uint8_t didit = 0;
  
  if (!HAL_GPIO_ReadPin(I2Cx_IT_GPIO_PORT, I2Cx_IT_PIN))
  {
//*
	  if (!didit) {
		  while(HAL_I2C_Master_Receive(&I2cHandle, (uint16_t)I2C_ADDRESS, (uint8_t*)aRxBuffer, RXBUFFERSIZE, I2C_TIMEOUT)!= HAL_OK)
		  {
			  //printf("Trying a Master Transmit\n");
			  if (HAL_I2C_GetError(&I2cHandle) != HAL_I2C_ERROR_AF)
			  {
				  printf("Time out occured :o\n");
				  Error_Handler();
			  }
		  }

		  clear_screen();

		  printf("Device Mode  [2..0] : %s\n", b2b((aRxBuffer[0]>>4) & 0b00000111));
		  printf("Gesture ID   [7..0] : %x\n", aRxBuffer[1]);
		  printf("Touch Points [2..0] : %s\n", b2b(aRxBuffer[2] & 0b00000111));

		  int j = 1;
		  for (int i = 3; i < 0x1F; i += 6)
		  {
			  printf("Touch %i EV   [1..0] : %s\n", j, b2b((aRxBuffer[i] >> 6) & 0b00000011));
			  printf("Touch %i ID   [3..0] : %s\n", j, b2b((aRxBuffer[i+2] >> 4) & 0b00001111));
			  uint16_t x = (((uint16_t)(aRxBuffer[i] & 0x0F)) << 8) | ((uint16_t)aRxBuffer[i+1]);
			  uint16_t y = (((uint16_t)(aRxBuffer[i+2] & 0x0F)) << 8) | ((uint16_t)aRxBuffer[i+3]);
			  printf("Touch %i (X,Y)       : (%i, %i)\n", j, x, y);
			  printf("Touch %i (X,Y)       : (%x, %x)\n", j, x, y);
			  j++;
		  }
		didit = 1;
	  }
	  //*/
  }

}
  
  /* USER CODE END 3 */

} 

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  *         The LTDC Clock is configured as follow :
  *            PLLSAIN                        = 192
  *            PLLSAIR                        = 4
  *            PLLSAIDivR                     = 8
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
     clocked below the maximum system frequency, to update the voltage scaling value 
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /*##-1- System Clock Configuration #########################################*/  
  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Activate the Over-Drive mode */
  HAL_PWREx_EnableOverDrive();
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  /*##-2- LTDC Clock Configuration ###########################################*/  
  /* LCD clock configuration */
  /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 MHz */
  /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 MHz */
  /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/4 = 48 MHz */
  /* LTDC clock frequency = PLLLCDCLK / RCC_PLLSAIDIVR_8 = 48/8 = 6 MHz */

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct); 
}


/* USER CODE BEGIN 4 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
	printf("I2C Error callback called!\n");
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
