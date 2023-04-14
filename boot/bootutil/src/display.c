#include <stdbool.h>
#include <string.h>
#include "lcd.h"
// gimp generated c code for the Q image
// To generate a new image in gimp:
//      file->Export As
//      filename=qlogo.c
//    In the "Export image as C-Source" dialog box make sure the following:
//      Prefixed name: qlogo
//      Comment: Created with GIMP
//      Uncheck "Save comment to file"
//      Uncheck "Use GLib types"
//        Check "Use macros instead of struct"
//        Check "Use 1 byte Run-Length-Encoding"
//      Uncheck "Save alpha channel"
//        Check "Save as RGB565 (16-bit)"
//      Opacity=100
//
//      Click Export.
//
#include "qlogo.c"

typedef struct
{
    int pxPercentage;
    bool logoShown;
    uint8_t logoBuf[QLOGO_HEIGHT * QLOGO_WIDTH * QLOGO_BYTES_PER_PIXEL];
} dispData_t;

static dispData_t dispData;

#define DISP_STATUS_BAR_STARTX      48
#define DISP_STATUS_BAR_ENDX        198

#define DISP_STATUS_BAR_STARTY      197
#define DISP_STATUS_BAR_ENDY        210

static void disp_show_logo()
{
    if (dispData.logoShown == false)
    {
        int xoffs = (lcd_width() - QLOGO_WIDTH) / 2;
        int yoffs = (lcd_height() - QLOGO_HEIGHT) / 3;
        dispData.logoShown = true;
        QLOGO_RUN_LENGTH_DECODE(dispData.logoBuf, QLOGO_rle_pixel_data, QLOGO_HEIGHT * QLOGO_WIDTH, QLOGO_BYTES_PER_PIXEL);
        for (int y = 0; y < QLOGO_HEIGHT; y++)
        {
            uint8_t* pRow = dispData.logoBuf + (y * QLOGO_WIDTH * QLOGO_BYTES_PER_PIXEL);
            for (int x = 0; x < QLOGO_WIDTH; x++)
            {
                uint16_t color = pRow[x * QLOGO_BYTES_PER_PIXEL] | (pRow[x * QLOGO_BYTES_PER_PIXEL + 1] << 8);
                lcd_plot(x + xoffs, y + yoffs, color);
            }
        }
    }
}

void swap_progress_update_disp(int percentage)
{
    disp_show_logo();
    int pxPercentage =  (((float)percentage / 100.0f) * (DISP_STATUS_BAR_ENDX - DISP_STATUS_BAR_STARTX)) + DISP_STATUS_BAR_STARTX;
    if (pxPercentage != dispData.pxPercentage)
    {
        dispData.pxPercentage = pxPercentage;
        for (int x = DISP_STATUS_BAR_STARTX; x < DISP_STATUS_BAR_ENDX; x++)
        {
            uint16_t color = LCD_COLOR_RGB(0x03, 0x03, 0x03);
            if (x < dispData.pxPercentage)
            {
                color = LCD_COLOR_RGB(0xff, 0xff, 0xff);
            }
            for (int y = DISP_STATUS_BAR_STARTY; y < DISP_STATUS_BAR_ENDY; y++)
            {
                lcd_plot(x, y, color);
            }
        }
        lcd_flush();
    }
}

void disp_init()
{
    dispData.pxPercentage = -1;
    lcd_init();
}
