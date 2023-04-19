#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#define LCD_COLOR_RED(r)        ((r & 0x1f) << 11)
#define LCD_COLOR_GREEN(g)      ((g & 0x3f) << 5)
#define LCD_COLOR_BLUE(b)       ((b & 0x1f) << 0)
#define LCD_COLOR_RGB(r,g,b)    (LCD_COLOR_RED(r) | LCD_COLOR_GREEN(g) | LCD_COLOR_BLUE(b))

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

#endif // LCD_H
