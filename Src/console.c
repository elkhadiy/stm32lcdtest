/**
 * Implementation of the function console_putbytes
 * @author Yassine El Khadiri
 * @date 19/09/2014 14:50
 */

#include "console.h"
#include <inttypes.h>
#include <string.h>
#include "stdio.h"
#include "stm32f4xx_hal.h"

#include "lcd.h"
#include "font16.h"

uint16_t pos_curseur = 0;
char term[COL*LIN] = {' '};

uint16_t cur_pos_x = 0;
uint16_t cur_pos_y = 0;

uint16_t cur_pos_col = 0;
uint16_t cur_pos_lin = 0;

//uint8_t  *font_tb = (uint8_t*)Font16_Table;
uint16_t font_px_w = 11;
uint16_t font_px_h = 16;
uint16_t font_tb_offset = 2;

/// Attributs de texte
uint16_t ct = WHITE;
uint16_t cf = BLACK;

void console_change_color(uint16_t param_ct, uint16_t param_cf)
{
	ct = param_ct;
	cf = param_cf;
}

/**
 * redessine l'écran suivant le contenu du buffer term
 */
void render_screen(void)
{
	for (int i = 0; i < LIN; i++)
	{
		for (int j = 0; j < COL; j++)
		{
			Render_Glyph(&Font16_Table[(term[i*COL+j]-' ')*font_px_h*font_tb_offset],
					font_tb_offset, font_px_h,
					j*font_px_w, i*font_px_h,
					ct, cf);
		}
	}
}

/**
 * Put a character at the given coordinates
 * @param lig the line
 * @param col the column
 * @param c the character to draw
 * @param cf the background color
 * @param ct the text color
 * @param cl the blinking flag
 */
void putc_at(uint32_t lig, uint32_t col, char c, uint16_t ct, uint16_t cf)
{
	term[lig*COL+col] = c;
	Render_Glyph(&Font16_Table[(c-' ')*font_px_h*font_tb_offset],
					font_tb_offset, font_px_h,
					col*font_px_w, lig*font_px_h,
					ct, cf);
}


/**
 * Put cursor at the given position
 * @param lig the line
 * @param col the column
 */
void put_cursor(uint32_t lig, uint32_t col)
{
	cur_pos_x = col*font_px_w;
	cur_pos_y = lig*font_px_h;
	cur_pos_col = col;
	cur_pos_lin = lig;
}

extern DMA2D_HandleTypeDef Dma2dHandle;

/**
 * Actually zeroes the VIDEO RAM.. normally
 */
void clear_screen(void)
{
	
	pos_curseur = 0;
	//memset((void*)0xD0000000, 0, (size_t)800*480*2);
	HAL_DMA2D_DeInit(&Dma2dHandle);

	Dma2dHandle.Init.Mode = DMA2D_R2M;
	Dma2dHandle.Init.ColorMode = DMA2D_RGB565;
	Dma2dHandle.Init.OutputOffset = 0x0;
	Dma2dHandle.LayerCfg[0].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	Dma2dHandle.LayerCfg[0].InputAlpha = 0xFF;
	Dma2dHandle.LayerCfg[0].InputColorMode = CM_RGB565;
	Dma2dHandle.LayerCfg[0].InputOffset = 0x0;
	Dma2dHandle.Instance = DMA2D; 

	HAL_DMA2D_Init(&Dma2dHandle);
	HAL_DMA2D_ConfigLayer(&Dma2dHandle, 0);
	HAL_DMA2D_Start(&Dma2dHandle, BLACK, (uint32_t)LCD_BUFFER_START, LCD_WIDTH, LCD_HEIGHT);
	HAL_DMA2D_PollForTransfer(&Dma2dHandle, 200);
}

/**
 * Scroll screen by 1 line
 */
void defilement(void)
{
	// move the screen up
	memmove((void*)0xD0000000,(const void*)((uint32_t)0xD0000000+(uint32_t)800*16*2),(size_t)((uint32_t)(800*(480-16)*2)));
	// initialise the last line
	for (uint16_t j = 0; j < COL; j++)
		putc_at(LIN-1, j, ' ', ct, cf);
	
/*
	uint32_t secondary_buffer = get_secondary_buff_adr();
	
	HAL_DMA2D_DeInit(&Dma2dHandle);

	Dma2dHandle.Init.Mode = DMA2D_M2M;
	Dma2dHandle.Init.ColorMode = DMA2D_RGB565;
	Dma2dHandle.Init.OutputOffset = 0x0;
	Dma2dHandle.LayerCfg[0].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	Dma2dHandle.LayerCfg[0].InputAlpha = 0xFF;
	Dma2dHandle.LayerCfg[0].InputColorMode = CM_RGB565;
	Dma2dHandle.LayerCfg[0].InputOffset = 0x0;
	Dma2dHandle.Instance = DMA2D; 

	HAL_DMA2D_Init(&Dma2dHandle);
	HAL_DMA2D_ConfigLayer(&Dma2dHandle, 0);
	HAL_DMA2D_Start(&Dma2dHandle, secondary_buffer+LCD_WIDTH*font_px_h*2, secondary_buffer, LCD_WIDTH, LCD_HEIGHT-font_px_h);
	HAL_DMA2D_PollForTransfer(&Dma2dHandle, 200);
	
	HAL_DMA2D_DeInit(&Dma2dHandle);

	Dma2dHandle.Init.Mode = DMA2D_R2M;
	Dma2dHandle.Init.ColorMode = DMA2D_RGB565;
	Dma2dHandle.Init.OutputOffset = 0x0;
	Dma2dHandle.LayerCfg[0].AlphaMode = DMA2D_NO_MODIF_ALPHA;
	Dma2dHandle.LayerCfg[0].InputAlpha = 0xFF;
	Dma2dHandle.LayerCfg[0].InputColorMode = CM_RGB565;
	Dma2dHandle.LayerCfg[0].InputOffset = 0x0;
	Dma2dHandle.Instance = DMA2D; 

	HAL_DMA2D_Init(&Dma2dHandle);
	HAL_DMA2D_ConfigLayer(&Dma2dHandle, 0);
	HAL_DMA2D_Start(&Dma2dHandle, BLACK, secondary_buffer+LCD_WIDTH*(LCD_HEIGHT-font_px_h)*2, LCD_WIDTH, font_px_h);
	HAL_DMA2D_PollForTransfer(&Dma2dHandle, 200);

	switch_buffer();
//*/
}


/**
 * affiche correctement le caractère selon son type
 * @param c le caractère à traiter
 */
void traite_char(char c)
{
	switch(c) {
		case 8: // \b
			if (pos_curseur % COL != 0)
				pos_curseur--;
			put_cursor(pos_curseur/COL,pos_curseur%COL);
			break;
		
		case 9: // \t
			pos_curseur += 8-pos_curseur%8;
			put_cursor(pos_curseur/COL,pos_curseur%COL);
			break;
		case 10: // \n
			if (pos_curseur/COL == LIN-1) {
				defilement();
				pos_curseur -= pos_curseur%COL;
			} else {
				pos_curseur += COL - pos_curseur%COL;
			}
			put_cursor(pos_curseur/COL, pos_curseur%COL);
			break;
		case 12: // \f
			clear_screen();
			pos_curseur = 0;
			put_cursor(pos_curseur/COL, pos_curseur%COL);
			break;
		case 13: // \r
			pos_curseur -= pos_curseur%COL;
			put_cursor(pos_curseur/COL, pos_curseur%COL);
			break;
		default:
			putc_at(pos_curseur/COL, pos_curseur%COL, c, ct, cf);
			pos_curseur++;
			if (pos_curseur >= COL*LIN) {
				defilement();
				pos_curseur = COL*(LIN-1);
			}
			put_cursor(pos_curseur/COL,pos_curseur%COL);
			break;
	}
}


/**
 * Write stuff on the screen
 * @param s pointer to a string
 * @param len length of the string
 */
void console_putbytes(const char *s, int len)
{
	for (uint32_t i = 0; i < len; i++)
		traite_char(s[i]);
}

/**
 * Ecrit en haut à droite de l'écrant
 * @param s: la chaine de caractère à écrire en haut à droite
 * 		DOIT ETRE < 80 caractères
 */
void print_ne(const char *s)
{
	// taille de la chaine à afficher
	size_t len = strlen(s);
	if (len <= COL) { // TODO: quoi faire si la chaine > 80 caractères
	// sauvegarde de la position actuelle du curseur
		uint16_t cur = pos_curseur;
		// placement du curseur pour affichage en haut à droite
		pos_curseur = COL-len;
		console_putbytes(s,len);
		pos_curseur = cur;
		put_cursor(cur/COL,cur%COL);
	}
}
