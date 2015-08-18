/**
 * @file lcd.h
 * @brief LCD low level initialisation pixel drawing functions
 */

#ifndef _LCD_H_
#define _LCD_H_

#include "stm32f4xx_hal.h"

/*
 * LCD dimensions and timing constants
 */
#define LCD_WIDTH		800
#define LCD_HEIGHT		480

#define HFP 	16
#define HSYNC 	96
#define HBP 	88

#define VFP 	10
#define VSYNC 	3
#define VBP 	33

#define ACTIVE_W (HSYNC + LCD_WIDTH + HBP - 1)
#define ACTIVE_H (VSYNC + LCD_HEIGHT + VBP - 1)

#define TOTAL_WIDTH  (HSYNC + HBP + LCD_WIDTH + HFP - 1)
#define TOTAL_HEIGHT (VSYNC + VBP + LCD_HEIGHT + VFP - 1)

#define LCD_BUFFER_START 0xD0000000

#define BUFFER         0xD0000000
#define BUFFER1_OFFSET 800*480*2
#define BUFFER1        (BUFFER + BUFFER1_OFFSET)

/**
 * @brief LCD Screen handle
 */
extern LTDC_HandleTypeDef LtdcHandle;

/**
 * @brief DMA2D handle
 */
extern DMA2D_HandleTypeDef Dma2dHandle;

/**
 * @brief Configures the LCD GPIOs
 * @param hltdc LCD Handle
 */
void HAL_LTDC_MspInit(LTDC_HandleTypeDef *hltdc);

/**
 * @brief Configures the DMA2D CLock
 * @param hdma2d DMA2D handle
 */
void HAL_DMA2D_MspInit(DMA2D_HandleTypeDef *hdma2d);

/**
 * @brief Configures lcd screen parameters
 */
void LCD_Config(void);

/**
 * @brief Configures the dma2d parameters
 */
void DMA2D_Config(void);

/**
 * @brief DMA2D error callback
 */
void DMA2D_TransferError(DMA2D_HandleTypeDef* Dma2dHandle);

/**
 * @ brief DMA2D transfer complete callback
 */
void DMA2D_TransferComplete(DMA2D_HandleTypeDef* Dma2dHandle);

/**
 * @brief Draws a pixel in the screen framebuffer
 * @param Xpos the pixel's x position
 * @param Ypos the pixel's y position
 * @param RGB_Code the pixel's color
 */
void Put_Pixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGB_Code);

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
					uint16_t x, uint16_t y, uint16_t ct, uint16_t cf);

/**
 * Double buffering
 */
void switch_buffer(void);

/**
 * prepares a second buffer for double buffered scrolling
 */
void init_second_buffer(void);

/**
 * does what it says
 */
uint32_t get_active_buff_adr(void);

/**
 * same
 */
uint32_t get_secondary_buff_adr(void);

#endif // EOF
