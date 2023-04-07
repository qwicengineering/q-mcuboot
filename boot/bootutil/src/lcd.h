#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#define LCD_COLOR_BLACK     0x0000
#define LCD_COLOR_RED       (0x001F << 10)
#define LCD_COLOR_GREEN     (0x001F << 0)
#define LCD_COLOR_BLUE      (0x001F << 5)
#define LCD_COLOR_YELLOW    (LCD_COLOR_GREEN|LCD_COLOR_BLUE)
#define LCD_COLOR_MAGENTA   (LCD_COLOR_RED|LCD_COLOR_BLUE)
#define LCD_COLOR_CYAN      (LCD_COLOR_GREEN|LCD_COLOR_BLUE)
#define LCD_COLOR_WHITE     0xFFFF

// Initialize the LCD driver (switches LCD on, fades backlight to max)
extern void lcd_init( void );

// Return the LCD's width and height
extern unsigned int lcd_width( void );
extern unsigned int lcd_height( void );

// Flush the contents of the internal LCD representation to the actual hardware
extern void lcd_flush( void );

// Clear the entire LCD and homes the cursor
extern void lcd_clrscr( uint16_t color );

// Plot a single pixel 
extern void lcd_plot( unsigned int x, unsigned int y, uint16_t color );

// Draw a line
extern void lcd_line( unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, uint16_t color );

// Draw a circle
extern void lcd_circle( unsigned int x0, unsigned int y0, unsigned int radius, uint16_t color );

#endif // LCD_H
