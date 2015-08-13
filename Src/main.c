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

#include "stm32f429i_discovery.h"
#include "ili9341.h"

#include "st_logo1.h"
#include "st_logo2.h"

#include "stm32f4xx.h"

#include "font8.h"
#include "font16.h"
#include "font24.h"

#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



#define LCD_WIDTH		800
#define LCD_HEIGHT		480
//*
#define HFP 	16
#define HSYNC 	96
#define HBP 	88

#define VFP 	10
#define VSYNC 	3
#define VBP 	33
//*/
/*
#define HFP 	40
#define HSYNC	1
#define HBP 	88

#define VFP 	13
#define VSYNC 	3
#define VBP 	32
//*/
#define ACTIVE_W (HSYNC + LCD_WIDTH + HBP - 1)
#define ACTIVE_H (VSYNC + LCD_HEIGHT + VBP - 1)

#define TOTAL_WIDTH  (HSYNC + HBP + LCD_WIDTH + HFP - 1)
#define TOTAL_HEIGHT (VSYNC + VBP + LCD_HEIGHT + VFP - 1)

LTDC_HandleTypeDef LtdcHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static void LCD_Config(void);
static void Error_Handler();
void HAL_LTDC_MspInit(LTDC_HandleTypeDef *hltdc);

void Put_Pixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code)
{
  /* Write data value to all SDRAM memory */
  *(__IO uint32_t*) (LtdcHandle.LayerCfg[0].FBStartAdress + (2*(Ypos*800 + Xpos))) = RGB_Code;
}

void Render_5x8_Glyph(const uint8_t *glyph, uint16_t x, uint16_t y, uint16_t color)
{
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 5; j++)
		{
			if ( glyph[i] & (0x80 >> j) )
			{
				Put_Pixel(x+j, y+i, color);
			}
		}
	}
}


void Render_11x16_Glyph(const uint8_t *glyph, uint16_t x, uint16_t y, uint16_t color)
{
	for (int i = 0; i < 16; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 8; k++)
			{
				if ( glyph[i*2+j] & (0x80 >> k) )
				{
					Put_Pixel(x+8*j+k, y+i, color);
				}
			}
		}
	}
}

void Render_17x24_Glyph(const uint8_t *glyph, uint16_t x, uint16_t y, uint16_t color)
{
	for (int i = 0; i < 24; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 8; k++)
			{
				if ( glyph[i*3+j] & (0x80 >> k) )
				{
					Put_Pixel(x+8*j+k, y+i, color);
				}
			}
		}
	}
}

uint16_t console_color = 0xFFFF;
uint16_t cursor_pos_x = 0;
uint16_t cursor_pos_y = 0;
uint16_t font_pixel_w = 11;
uint16_t font_pixel_h = 16;
uint16_t font_table_offset = 2;

void Put_Char(char c)
{
  if (c == '\n')
  {
    cursor_pos_x = 0;
	cursor_pos_y += font_pixel_h;
  } else {
    Render_11x16_Glyph(&Font16_Table[(c-' ')*font_pixel_h*font_table_offset], cursor_pos_x, cursor_pos_y, console_color);
  }
  cursor_pos_x += font_pixel_w;
  if (cursor_pos_x >= 800-font_pixel_w)
  {
    cursor_pos_x = 0;
	cursor_pos_y += font_pixel_h;
  }
  if (cursor_pos_y >= 480-font_pixel_h)
	cursor_pos_y = 0;
}

void Put_String(char *str)
{
	while(*str)
	{
		Put_Char(*str);
		str++;
	}
}

void DrawCircle(int x0, int y0, int radius, uint16_t color)
{
  int x = radius;
  int y = 0;
  int decisionOver2 = 1 - x;   // Decision criterion divided by 2 evaluated at x=r, y=0

  while(x >= y)
  {
    Put_Pixel( x + x0,  y + y0, color);
    Put_Pixel( y + x0,  x + y0, color);
    Put_Pixel(-x + x0,  y + y0, color);
    Put_Pixel(-y + x0,  x + y0, color);
    Put_Pixel(-x + x0, -y + y0, color);
    Put_Pixel(-y + x0, -x + y0, color);
    Put_Pixel( x + x0, -y + y0, color);
    Put_Pixel( y + x0, -x + y0, color);
    y++;
    if (decisionOver2<=0)
    {
      decisionOver2 += 2 * y + 1;   // Change in decision criterion for y -> y+1
    }
    else
    {
      x--;
      decisionOver2 += 2 * (y - x) + 1;   // Change for y -> y+1, x -> x-1
    }
  }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void Error_Handler(void)
{
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET);
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


  /*##-1- Configure the ADC peripheral #######################################*/


  for (int x = 0; x < 800; x++)
	for (int y = 0; y < 480; y++)
		Put_Pixel(x, y, 0x0000);

  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //uint32_t v_x = 20, v_y = 10;
  //uint32_t pos_x = 0, pos_y = 0;
  //uint32_t v1_x = 10, v1_y = 20;
  //uint32_t pos1_x = 0, pos1_y = 0;
  
  HAL_LTDC_SetWindowPosition(&LtdcHandle, 0, 0, 0); 
  //DrawCircle(500,200,50,0xFFFF);
/*
  Render_11x16_Glyph(&Font16_Table[('a'-' ')*32], 100+0*11, 100, 0b1111100000000000);
  Render_11x16_Glyph(&Font16_Table[('b'-' ')*32], 100+1*11, 100, 0b0000011111100000);
  Render_11x16_Glyph(&Font16_Table[('c'-' ')*32], 100+2*11, 100, 0b0000000000011111);
  Render_11x16_Glyph(&Font16_Table[('d'-' ')*32], 100+3*11, 100, 0b1111111111111111);
  Render_11x16_Glyph(&Font16_Table[('e'-' ')*32], 100+4*11, 100, 0b1111111111111111);
*/
/*
  Put_Char('a');
  Put_Char('b');
  Put_Char('c');
  Put_Char('d');
  Put_Char('e');
*/
  Put_String("abcd");
  cursor_pos_y += font_pixel_h;
  cursor_pos_x = 0;
  Put_String("Hello World!");

  printf(" it works!");
  while (1)
  {
							
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
/* 
  pos_x += v_x;
  pos_y += v_y;

  if (pos_x >= 800-240 || pos_x <= 0)
  {
	v_x = -v_x;
  }

  if (pos_y >= 480-160 || pos_y <= 0)
  {
	v_y = -v_y;
  }
  
		
  HAL_LTDC_SetWindowPosition(&LtdcHandle, pos_x, pos_y, 0); 

 
  pos1_x += v1_x;
  pos1_y += v1_y;

  if (pos1_x >= 800-240 || pos1_x <= 0)
  {
	v1_x = -v1_x;
  }

  if (pos1_y >= 480-160 || pos1_y <= 0)
  {
	v1_y = -v1_y;
  }

		
  HAL_LTDC_SetWindowPosition(&LtdcHandle, pos1_x, pos1_y, 1); 
*/
  HAL_Delay(50);

}
  
  /* USER CODE END 3 */

}

static void LCD_Config(void)
{  
  LTDC_LayerCfgTypeDef pLayerCfg;
  LTDC_LayerCfgTypeDef pLayerCfg1;

  /* Initialization of ILI9341 component*/
  //ili9341_Init();
  
/* LTDC Initialization -------------------------------------------------------*/
  
  /* Polarity configuration */
  /* Initialize the horizontal synchronization polarity as active low */
  LtdcHandle.Init.HSPolarity = LTDC_HSPOLARITY_AH;
  /* Initialize the vertical synchronization polarity as active low */ 
  LtdcHandle.Init.VSPolarity = LTDC_VSPOLARITY_AH; 
  /* Initialize the data enable polarity as active low */ 
  LtdcHandle.Init.DEPolarity = LTDC_DEPOLARITY_AL; 
  /* Initialize the pixel clock polarity as input pixel clock */  
  LtdcHandle.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  
  /* Timing configuration */
  /* Horizontal synchronization width = Hsync - 1 */  
  LtdcHandle.Init.HorizontalSync = HSYNC - 1;
  /* Vertical synchronization height = Vsync - 1 */
  LtdcHandle.Init.VerticalSync = VSYNC - 1;
  /* Accumulated horizontal back porch = Hsync + HBP - 1 */
  LtdcHandle.Init.AccumulatedHBP = HSYNC + HBP - 1;
  /* Accumulated vertical back porch = Vsync + VBP - 1 */
  LtdcHandle.Init.AccumulatedVBP = VSYNC + VBP - 1; 
  /* Accumulated active width = Hsync + HBP + Active Width - 1 */ 
  LtdcHandle.Init.AccumulatedActiveH = ACTIVE_H;
  /* Accumulated active height = Vsync + VBP + Active Heigh - 1 */
  LtdcHandle.Init.AccumulatedActiveW = ACTIVE_W;
  /* Total height = Vsync + VBP + Active Heigh + VFP - 1 */
  LtdcHandle.Init.TotalHeigh = TOTAL_HEIGHT;
  /* Total width = Hsync + HBP + Active Width + HFP - 1 */
  LtdcHandle.Init.TotalWidth = TOTAL_WIDTH;
  
  /* Configure R,G,B component values for LCD background color */
  LtdcHandle.Init.Backcolor.Blue = 255;
  LtdcHandle.Init.Backcolor.Green = 255;
  LtdcHandle.Init.Backcolor.Red = 255;

  LtdcHandle.Instance = LTDC;
 
/* Layer1 Configuration ------------------------------------------------------*/
  
  /* Windowing configuration */ 
  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 800;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 480;
  
  /* Pixel Format configuration*/ 
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  
  /* Start Address configuration : frame buffer is located at FLASH memory */
  //pLayerCfg.FBStartAdress = (uint32_t)&ST_LOGO_1;
  pLayerCfg.FBStartAdress = (uint32_t)0xD0000000;
  
  /* Alpha constant (255 totally opaque) */
  pLayerCfg.Alpha = 255;
  
  /* Default Color configuration (configure A,R,G,B component values) */
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  
  /* Configure blending factors */
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
  
  /* Configure the number of lines and number of pixels per line */
  pLayerCfg.ImageWidth = 800;
  pLayerCfg.ImageHeight = 480;

/* Layer2 Configuration ------------------------------------------------------*/
  
  /* Windowing configuration */

  pLayerCfg1.WindowX0 = 0;
  pLayerCfg1.WindowX1 = 240;
  pLayerCfg1.WindowY0 = 160;
  pLayerCfg1.WindowY1 = 320;

  /* Pixel Format configuration*/ 
  pLayerCfg1.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  
  /* Start Address configuration : frame buffer is located at FLASH memory */
   pLayerCfg1.FBStartAdress = (uint32_t)&ST_LOGO_2;
  
  /* Alpha constant (255 totally opaque) */
  pLayerCfg1.Alpha = 190;
  
  /* Default Color configuration (configure A,R,G,B component values) */

  pLayerCfg1.Alpha0 = 0;
  pLayerCfg1.Backcolor.Blue = 0;
  pLayerCfg1.Backcolor.Green = 0;
  pLayerCfg1.Backcolor.Red = 0;
 
  /* Configure blending factors */

  pLayerCfg1.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
  pLayerCfg1.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
 
  /* Configure the number of lines and number of pixels per line */

  pLayerCfg1.ImageWidth = 240;
  pLayerCfg1.ImageHeight = 160;  
   
  /* Configure the LTDC */  
  if(HAL_LTDC_Init(&LtdcHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }

  /* Configure the Background Layer*/
  if(HAL_LTDC_ConfigLayer(&LtdcHandle, &pLayerCfg, 0) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler(); 
  }
 
/* 
  if(HAL_LTDC_ConfigLayer(&LtdcHandle, &pLayerCfg1, 1) != HAL_OK)
  {
    Error_Handler(); 
  }
*/ 
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

/**
  * @brief LTDC MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  * @param hltdc: LTDC handle pointer
  * @retval None
  */
void HAL_LTDC_MspInit(LTDC_HandleTypeDef *hltdc)
{  
  GPIO_InitTypeDef GPIO_Init_Structure;
  
  /*##-1- Enable peripherals and GPIO Clocks #################################*/  
  /* Enable the LTDC Clock */
  __HAL_RCC_LTDC_CLK_ENABLE();

  /* Enable GPIOs clock */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*##-2- Configure peripheral GPIO ##########################################*/
  /******************** LTDC Pins configuration *************************/
  /*
   +------------------------+-----------------------+----------------------------+
   +                       LCD pins assignment                                   +
   +------------------------+-----------------------+----------------------------+
   |  LCD_TFT R2 <-> PC.10  |  LCD_TFT G2 <-> PA.06 |  LCD_TFT B2 <-> PD.06      |
   |  LCD_TFT R3 <-> PB.00  |  LCD_TFT G3 <-> PG.10 |  LCD_TFT B3 <-> PG.11      |
   |  LCD_TFT R4 <-> PA.11  |  LCD_TFT G4 <-> PB.10 |  LCD_TFT B4 <-> PG.12      |
   |  LCD_TFT R5 <-> PA.12  |  LCD_TFT G5 <-> PB.11 |  LCD_TFT B5 <-> PA.03      |
   |  LCD_TFT R6 <-> PB.01  |  LCD_TFT G6 <-> PC.07 |  LCD_TFT B6 <-> PB.08      |
   |  LCD_TFT R7 <-> PG.06  |  LCD_TFT G7 <-> PD.03 |  LCD_TFT B7 <-> PB.09      |
   -------------------------------------------------------------------------------
            |  LCD_TFT HSYNC <-> PC.06  | LCDTFT VSYNC <->  PA.04 |
            |  LCD_TFT CLK   <-> PG.07  | LCD_TFT DE   <->  PF.10 |
             -----------------------------------------------------

  */

  /* LTDC pins configuraiton: PA3 -- 12 */  
  GPIO_Init_Structure.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6 |
                                GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_Init_Structure.Mode = GPIO_MODE_AF_PP;
  GPIO_Init_Structure.Pull = GPIO_NOPULL;
  GPIO_Init_Structure.Speed = GPIO_SPEED_FAST;
  GPIO_Init_Structure.Alternate= GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOA, &GPIO_Init_Structure);
  
  /* LTDC pins configuraiton: PB8 -- 11 */  
  GPIO_Init_Structure.Pin = GPIO_PIN_8 | \
                             GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOB, &GPIO_Init_Structure);

  /* LTDC pins configuraiton: PC6 -- 10 */  
  GPIO_Init_Structure.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOC, &GPIO_Init_Structure);

  /* LTDC pins configuraiton: PD3 -- 6 */  
  GPIO_Init_Structure.Pin = GPIO_PIN_3 | GPIO_PIN_6;
  HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);
  
  /* LTDC pins configuraiton: PF10*/  
  GPIO_Init_Structure.Pin = GPIO_PIN_10;
  HAL_GPIO_Init(GPIOF, &GPIO_Init_Structure);     

  /* LTDC pins configuraiton: PG6 -- 7 */  
  GPIO_Init_Structure.Pin = GPIO_PIN_6 | GPIO_PIN_7 | \
                             GPIO_PIN_11;
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);
 
  /* LTDC pins configuraiton: PB1 -- 1 */  
  GPIO_Init_Structure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_Init_Structure.Alternate= GPIO_AF14_LTDC;
  HAL_GPIO_Init(GPIOB, &GPIO_Init_Structure);

  /* LTDC pins configuraiton: PG10 -- 12 */  
  GPIO_Init_Structure.Pin = GPIO_PIN_10 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOG, &GPIO_Init_Structure);
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
