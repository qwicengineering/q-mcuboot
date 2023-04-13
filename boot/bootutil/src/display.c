#include "lcd.h"

typedef struct
{
    int pxPercentage;
} dispData_t;

static dispData_t dispData;

#define DISP_STATUS_BAR_STARTX      48
#define DISP_STATUS_BAR_ENDX        198

#define DISP_STATUS_BAR_STARTY      197
#define DISP_STATUS_BAR_ENDY        210

void swap_progress_update_disp(int percentage)
{
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
