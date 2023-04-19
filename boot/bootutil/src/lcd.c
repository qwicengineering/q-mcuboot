#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <nrfx_spim.h>
#include <nrfx_glue.h>
#include <zephyr.h>
#include <nrfx.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_dppi.h>
#include <assert.h>
#include "st77xx_internal.h"
#include "lcd.h"


#define SPI_INSTANCE 4

#define LCD_COLS    240
#define LCD_ROWS    320
#define P_LCD_DC        NRF_GPIO_PIN_MAP( 0, 12 )
#define P_LCD_SPI_SS    NRF_GPIO_PIN_MAP( 0, 11 )
#define P_LCD_RST       NRF_GPIO_PIN_MAP( 0, 10 )
#define P_BACKLIGHT     NRF_GPIO_PIN_MAP( 1, 4 )

/* Peripheral handlers */

nrfx_spim_t lcd_spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);
extern void nrfx_isr(const void *irq_handler);
static bool lcd_send( const void * data, size_t len, size_t cmd_len );

static struct k_sem _spiSem;
static struct
{
    uint16_t data[LCD_ROWS * LCD_COLS];
} _screen;

static void lcd_spi_handler( nrfx_spim_evt_t const * evt, void * ctx )
{
    switch( evt->type )
    {
    case NRFX_SPIM_EVENT_DONE :
        k_sem_give(&_spiSem);
        break;
    default :
        printk( "Internal error, SPIM event = 0x%X\r\n", evt->type );
    }
}

// Inititalization code here
static void lcd_backlight( bool enabled )
{
    nrf_gpio_pin_write(P_BACKLIGHT, enabled);
}

static bool lcd_spi_transfer(const void * data, size_t len)
{
    nrfx_err_t err;
    nrfx_spim_xfer_desc_t spi_xfer;
    spi_xfer.p_tx_buffer = data;
    spi_xfer.tx_length = len;
    spi_xfer.p_rx_buffer = NULL;
    spi_xfer.rx_length = 0;
    if (err = nrfx_spim_xfer( &lcd_spi, &spi_xfer, NRFX_SPIM_FLAG_TX_POSTINC ), err != NRFX_SUCCESS )
    {
        printk( "SPI tramsfer failed, code 0x%02X\r\n", err );
    }
    else if (k_sem_take(&_spiSem, K_MSEC(500)) != 0)
    {
        err = NRFX_ERROR_TIMEOUT;
    }
    return err == NRFX_SUCCESS;
}

static bool lcd_send( const void * data, size_t len, size_t cmd_len )
{
    bool ret = false;
    nrf_gpio_pin_clear(P_LCD_SPI_SS);
    if (cmd_len > 0)
    {
        // Don't use the _dcx form of the spi transfer because we can't enable it in the bootloader
        // so manually set the DC pin, and the SS pin
        nrf_gpio_pin_clear(P_LCD_DC);
        ret = lcd_spi_transfer(data, cmd_len);
        nrf_gpio_pin_set(P_LCD_DC);
        data = (uint8_t*)data + cmd_len;
        len -= cmd_len;
    }
    if (len > 0)
    {
        ret = lcd_spi_transfer(data, len);
    }
    nrf_gpio_pin_set(P_LCD_SPI_SS);
    return ret;
}

static void lcd_fail()
{
    for (int i = 0; i < 100; i++)
    {
        nrf_gpio_pin_write(P_BACKLIGHT, i&1);
        k_sleep( K_MSEC( 400 ) );
    }
}

// Note: do not put in flash, these sequences are used by SPI DMA
// Initialize the display (bluntly stolen from AdaFruit: https://github.com/adafruit/Adafruit-ST7735-Library)

static void lcd_data_init( void )
{
    static uint8_t st7789_init_sequence[] =
    {
        // #Bytes, command, [arguments,] delay
        1, ST77XX_SWRESET, 150,                             //  1: Software reset, no args, ~150 ms delay
        1, ST77XX_SLPOUT, 10,                               //  2: Out of sleep mode, no args
        3, ST77XX_RAMCTRL, 0x00, 0xF8, 0,
        2, ST77XX_COLMOD, 0x55, 10,
        1, ST77XX_INVON, 10,
        1, ST77XX_NORON, 10,
        1, ST77XX_DISPON, 100
    } ;

    uint8_t * st7789_cmd = st7789_init_sequence;
    uint8_t * const st7789_sequence_end = st7789_init_sequence + sizeof( st7789_init_sequence );
    while( st7789_cmd < st7789_sequence_end )
    {
        unsigned int bytes_to_send = *st7789_cmd++;
        unsigned int delay_ms;
        if ( !lcd_send( st7789_cmd, bytes_to_send, 1 ) ) lcd_fail( );
        st7789_cmd += bytes_to_send;
        delay_ms = *st7789_cmd++;
        k_sleep( K_MSEC( delay_ms ) );
    }
}

void lcd_square(int x, int y, uint16_t color)
{
    for (int px = x; px < (x + 5); px++)
    {
        for (int py = y; py < (y + 5); py++)
        {
            lcd_plot(px, py, color);
        }
    }
}

// Initialize the LCD
void lcd_init(void)
{
    nrfx_err_t err;

    k_sem_init(&_spiSem, 0, K_SEM_MAX_LIMIT);
    nrfx_spim_config_t const lcd_spi_config = 
    {
        .sck_pin        = NRF_GPIO_PIN_MAP( 0, 8 ),
        .mosi_pin       = NRF_GPIO_PIN_MAP( 0, 9 ),
        .miso_pin       = NRFX_SPIM_PIN_NOT_USED,
        .ss_pin         = NRFX_SPIM_PIN_NOT_USED,
        .frequency      = NRF_SPIM_FREQ_32M,
        .mode           = NRF_SPIM_MODE_0,
        .bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST,
        .miso_pull      = NRF_GPIO_PIN_NOPULL,
        .ss_active_high = false,
        .irq_priority   = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .orc            = 0xFF,
    } ;

    if ( err = nrfx_spim_init( &lcd_spi, &lcd_spi_config, lcd_spi_handler, NULL ), err != NRFX_SUCCESS )
    {
        printk( "Unable to initialize SPI port for LCD\r\n");
        lcd_fail();
    }
    // The following is necessary to fill in the address of our interrupt handler in the vector table. Bit weird...
    IRQ_CONNECT( DT_IRQN(DT_NODELABEL(spi4)), DT_IRQ(DT_NODELABEL(spi4), priority), nrfx_isr, nrfx_spim_4_irq_handler, 0);  

    // Initialize GPIO and reset the display
    nrf_gpio_cfg_output(P_BACKLIGHT);
    nrf_gpio_cfg_output( P_LCD_RST );
    nrf_gpio_pin_clear( P_LCD_RST );
    k_sleep( K_MSEC( 1 ) );
    nrf_gpio_pin_set( P_LCD_RST );
    nrf_gpio_pin_set(P_LCD_SPI_SS);
    nrf_gpio_cfg_output(P_LCD_SPI_SS);
    nrf_gpio_pin_set(P_LCD_DC);
    nrf_gpio_cfg_output(P_LCD_DC);
    lcd_data_init();
    lcd_clrscr(0);
    lcd_flush();
    lcd_backlight(true);
}

unsigned int lcd_width( void )
{
    return LCD_COLS;
}

unsigned int lcd_height( void )
{
    return LCD_ROWS;
}

void lcd_clrscr( uint16_t color )
{
    for ( unsigned int i = 0; i < LCD_COLS * LCD_ROWS; i++ )
    {
        _screen.data[i] = color;
    }
}

void lcd_flush( void )
{
    static uint8_t _caset[] = { ST77XX_CASET, 0 >> 8, 0 & 0xFF, (LCD_COLS-1) >> 8, (LCD_COLS-1) & 0xFF  };
    static uint8_t _raset[] = { ST77XX_RASET, 0 >> 8, 0 & 0xFF, (LCD_ROWS-1) >> 8, (LCD_ROWS-1) & 0xFF  };

    // Set window to max: X = 0..239, Y = 0..319
    if ( !lcd_send( _caset, sizeof( _caset ), 1 ) )
    {
        printk( "CASET command failed\r\n");
    }
    else if ( !lcd_send( _raset, sizeof( _raset ), 1 ) )
    {
        printk( "RASET command failed\r\n" );
    }
    else
    {
        // Setup data transfers
        uint8_t cmd = ST77XX_RAMWR;
        if ( lcd_send( (void *)&cmd, 1, 1 ) )
        {
            for (int i = 0; i < LCD_ROWS; i++)
            {
                if ( !lcd_send( _screen.data + (i * LCD_COLS), (LCD_COLS * 2), 0 ))
                {
                    break;
                }
            }
        }
    }
}

void lcd_plot( unsigned int x, unsigned int y, uint16_t color )
{
    if ( x < LCD_COLS && y < LCD_ROWS )
    {
        _screen.data[x + y * LCD_COLS] = color;
    }
}
