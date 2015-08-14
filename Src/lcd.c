/**
 * @file lcd.c
 * @brief Implementation of the lcd low level functions
 */

#include "stm32f4xx_hal.h"
#include "lcd.h"
#include "stdbool.h"

/**
 * @brief LCD handle
 */
LTDC_HandleTypeDef LtdcHandle;

/*
 * Dumb error handler. TODO make this more streamlined
 */
extern void Error_Handler(void);

/**
 * @brief Configures the LCD GPIOs
 * @param hltdc LCD Handle
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


/**
 * @brief Configures lcd screen
 */
void LCD_Config(void)
{
  LTDC_LayerCfgTypeDef pLayerCfg;
  
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

  /* Place the Background Layer */
  HAL_LTDC_SetWindowPosition(&LtdcHandle, 0, 0, 0); 
}

/**
 * @brief Draws a pixel in the screen framebuffer
 * @param Xpos the pixel's x position
 * @param Ypos the pixel's y position
 * @param RGB_Code the pixel's color
 */
void Put_Pixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code)
{
  *(__IO uint32_t*) (LtdcHandle.LayerCfg[0].FBStartAdress + (2*(Ypos*800 + Xpos))) = RGB_Code;
}

/**
 * @brief Renders a glyph from a given array of uint16_t
 * @param glyph pointer to start of the glyph
 * @param width width of the glyph by uint16_t
 * @param height height of the glyph by pixel
 * @param x x position where to put the glyph
 * @param y y position where to put the glyph
 * @param color the glyph's color
 */
void Render_Glyph(const uint8_t *glyph, uint16_t width, uint16_t height,
					uint16_t x, uint16_t y, uint16_t ct, uint16_t cf)
{
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			for (int k = 0; k < 8; k++)
			{
				Put_Pixel(x+8*j+k, y+i, (glyph[i*width+j] & (0x80 >> k)) ? ct : cf);
			}
		}
	}
}

#define BUFFER  0xD0000000
#define BUFFER1 0xD0000000+800*480*2

static bool cur_is_buffer0 = true;

/**
 * Double buffering
 */
void switch_buffer(void)
{
	LtdcHandle.LayerCfg[0].FBStartAdress = cur_is_buffer0 ? BUFFER1 : BUFFER;
	cur_is_buffer0 = ~cur_is_buffer0;
}

// EOF
