#ifndef ST77XX_INTERNAL_H
#define ST77XX_INTERNAL_H

// Register definitions

#define ST_CMD_DELAY 0x80 // special signifier for command lists

#define ST77XX_NOP 0x00
#define ST77XX_SWRESET 0x01
#define ST77XX_RDDID 0x04
#define ST77XX_RDDST 0x09

#define ST77XX_SLPIN 0x10
#define ST77XX_SLPOUT 0x11
#define ST77XX_PTLON 0x12
#define ST77XX_NORON 0x13

#define ST77XX_INVOFF 0x20
#define ST77XX_INVON 0x21
#define ST77XX_DISPOFF 0x28
#define ST77XX_DISPON 0x29
#define ST77XX_CASET 0x2A
#define ST77XX_RASET 0x2B
#define ST77XX_RAMWR 0x2C
#define ST77XX_RAMRD 0x2E

#define ST77XX_PTLAR 0x30
#define ST77XX_TEOFF 0x34
#define ST77XX_TEON 0x35
#define ST77XX_MADCTL 0x36
#define ST77XX_COLMOD 0x3A

#define ST77XX_MADCTL_MY 0x80
#define ST77XX_MADCTL_MX 0x40
#define ST77XX_MADCTL_MV 0x20
#define ST77XX_MADCTL_ML 0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1        0xDA
#define ST77XX_RDID2        0xDB
#define ST77XX_RDID3        0xDC
#define ST77XX_RDID4        0xDD


#define ST77XX_PORCTRK      0xB2
#define ST77XX_GCTRL        0xB7
#define ST77XX_VCOMS        0xBB
#define ST77XX_LCMCTRL      0xC0
#define ST77XX_VDVVRHEN     0xC2
#define ST77XX_VRHS         0xC3
#define ST77XX_VDVS         0xC4
#define ST77XX_FRCTRL2      0xC6
#define ST77XX_PWCTRL1      0xD0
#define ST77XX_PVGAMCTRL    0xE0
#define ST77XX_NVGAMCTRL    0xE1


// Some ready-made 16-bit ('565') color settings:
#define ST77XX_BLACK 0x0000
#define ST77XX_WHITE 0xFFFF
#define ST77XX_RED 0xF800
#define ST77XX_GREEN 0x07E0
#define ST77XX_BLUE 0x001F
#define ST77XX_CYAN 0x07FF
#define ST77XX_MAGENTA 0xF81F
#define ST77XX_YELLOW 0xFFE0
#define ST77XX_ORANGE 0xFC00

#endif // ST77XX_INTERNAL_H
