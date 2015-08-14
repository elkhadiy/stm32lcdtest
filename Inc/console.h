#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#include <inttypes.h>

// CONSOLE COLORS
#define BLACK 0x00
#define BLUE 0b0000000000011111
#define GREEN 0b0000011111100000
#define CYAN 0
#define RED 0b1111100000000000
#define MAGENTA 5
#define BROWN 6
#define GRAY 7
#define DARK_GRAY 8
#define CLEAR_BLUE 9
#define CLEAR_GREEN 10
#define CLEAR_CYAN 11
#define CLEAR_RED 12
#define CLEAR_MAGENTA 13
#define YELLOW 14
#define WHITE 0b1111101111000000

// Console dimentions
#define COL 72
#define LIN 30

/**
 * This is the function called by printf to send its output to the screen.
 * You have to implement it in the kernel and in the user program.
 */
extern void console_putbytes(const char *s, int len);

/**
 * Fonction qui écrit en haut à droite la chaine de caractère
 * @param s: la chaine de caractère à afficher
 */
extern void print_ne(const char *s);

/**
 * Fonction qui change la couleur du texte
 * @param ct: couleur du texte
 * @param cf: couleur de fond
 */
extern void console_change_color(uint16_t ct, uint16_t cf);

/*// DEBUG functions
void putc_at(uint32_t lig, uint32_t col, char c, uint16_t cl,
		uint16_t cf, uint16_t ct);
void put_cursor(uint32_t lig, uint32_t col);
void clear_screen(void);
void traite_char(char c);
void defilement(void);
*/

#endif
