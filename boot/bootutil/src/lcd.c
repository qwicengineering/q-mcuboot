#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <nrfx_spim.h>
#include <nrfx_glue.h>
#include <zephyr.h>
#include <nrfx.h>
#include <nrfx_pwm.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_dppi.h>
#include <assert.h>
#include "st77xx_internal.h"
#include "lcd.h"
#include <logging/log.h>
LOG_MODULE_REGISTER(lcd, LOG_LEVEL_INF);


#define SPI_INSTANCE 4
#define PWM_INSTANCE 0

#define LCD_COLS    240
#define LCD_ROWS    320
#define P_LCD_DC        NRF_GPIO_PIN_MAP( 0, 12 )
#define P_LCD_SPI_SS    NRF_GPIO_PIN_MAP( 0, 11 )
/* Peripheral handlers */

nrfx_spim_t lcd_spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);
nrfx_pwm_t lcd_pwm = NRFX_PWM_INSTANCE(PWM_INSTANCE);

nrf_pwm_values_common_t lcd_softstart_values[1000];     // Initialize before use
nrf_pwm_sequence_t lcd_softstart_seq =
{
    .values.p_common = lcd_softstart_values,
    .length = NRF_PWM_VALUES_LENGTH( lcd_softstart_values ),
    .repeats = 1,
    .end_delay = 0
} ;


extern void nrfx_isr(const void *irq_handler);
static bool lcd_send( void * data, size_t len, size_t cmd_len );

static struct k_sem _spiSem;
static volatile struct
{
    uint8_t reserved;
    uint8_t cmd;
    uint16_t data[LCD_ROWS * LCD_COLS];
    uint16_t *row;
} _screen;

static void lcd_spi_handler( nrfx_spim_evt_t const * evt, void * ctx )
{
    switch( evt->type )
    {
    case NRFX_SPIM_EVENT_DONE :
        k_sem_give(&_spiSem);
        break;
    default :
        LOG_ERR( "Internal error, SPIM event = 0x%X", evt->type );
    }
}

static void lcd_pwm_handler( nrfx_pwm_evt_type_t const evt, void * ctx )
{
    static unsigned int _pwm_event_counter = 0;
    switch( evt )
    {
    case NRFX_PWM_EVT_FINISHED :
        break;
    case NRFX_PWM_EVT_END_SEQ0 :
        break;
    case NRFX_PWM_EVT_END_SEQ1 :
        break;
    case NRFX_PWM_EVT_STOPPED :
        break;
    default :   // Should never happen
        break;
    }
    LOG_INF( "PWM event %u", ++_pwm_event_counter );
}

// Inititalization code here
static void lcd_backlight_init( void )
{
    nrfx_err_t err;
    nrfx_pwm_config_t lcd_pwm_config = 
    {
        .output_pins =
        {
            NRF_GPIO_PIN_MAP( 1, 4 ), // | NRFX_PWM_PIN_INVERTED, 
            NRFX_PWM_PIN_NOT_USED, 
            NRFX_PWM_PIN_NOT_USED, 
            NRFX_PWM_PIN_NOT_USED 
        },
        .irq_priority = NRFX_PWM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .base_clock = NRF_PWM_CLK_500kHz,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value = 1000,
        .load_mode = NRF_PWM_LOAD_COMMON,
        .step_mode = NRF_PWM_STEP_AUTO
    } ;

    // The following is necessary to fill in the address of our interrupt handler in the vector table. Bit weird...
    IRQ_CONNECT( DT_IRQN(DT_NODELABEL(pwm0)), DT_IRQ(DT_NODELABEL(pwm0), priority), nrfx_isr, nrfx_pwm_0_irq_handler, 0);
        
    if ( err = nrfx_pwm_init( &lcd_pwm, &lcd_pwm_config, lcd_pwm_handler, NULL ), err != NRFX_SUCCESS )
    {
        LOG_ERR( "PWM failed to initialize");
        exit( EXIT_FAILURE );
    }

    for ( unsigned long int i = 0; i < NRF_PWM_VALUES_LENGTH( lcd_softstart_values ); i++ )
    {
        lcd_softstart_values[i] = lcd_pwm_config.top_value - (lcd_pwm_config.top_value * i) / NRF_PWM_VALUES_LENGTH( lcd_softstart_values );
    }

    __asm( "nop" );

    nrfx_pwm_simple_playback( &lcd_pwm, &lcd_softstart_seq, 1, NRFX_PWM_FLAG_SIGNAL_END_SEQ0 );
}

static bool lcd_spi_transfer(void * data, size_t len)
{
    nrfx_err_t err;
    nrfx_spim_xfer_desc_t spi_xfer;
    spi_xfer.p_tx_buffer = data;
    spi_xfer.tx_length = len;
    spi_xfer.p_rx_buffer = NULL;
    spi_xfer.rx_length = 0;
    if (err = nrfx_spim_xfer( &lcd_spi, &spi_xfer, NRFX_SPIM_FLAG_TX_POSTINC ), err != NRFX_SUCCESS )
    {
        LOG_ERR( "SPI tramsfer failed, code 0x%02X", err );
    }
    else if (k_sem_take(&_spiSem, K_MSEC(500)) != 0)
    {
        err = NRFX_ERROR_TIMEOUT;
    }
    return err == NRFX_SUCCESS;
}

static bool lcd_send( void * data, size_t len, size_t cmd_len )
{
    bool ret = false;
    nrf_gpio_pin_clear(P_LCD_SPI_SS);
    if (cmd_len > 0)
    {
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

// Note: do not put in flash, these sequences are used by SPI DMA
// Initialize the display (bluntly stolen from AdaFruit: https://github.com/adafruit/Adafruit-ST7735-Library)

void lcd_data_init( void )
{
    nrfx_err_t err;
    static uint8_t st7789_init_sequence[] =
    {
        // #Bytes, command, [arguments,] delay
        1, ST77XX_SWRESET, 150,                             //  1: Software reset, no args, ~150 ms delay
        1, ST77XX_DISPOFF, 10,
        1, ST77XX_SLPOUT, 10,                               //  2: Out of sleep mode, no args, 10 ms delay
//        2, ST77XX_MADCTL , 0x88, 0,                         //  4: Mem access ctrl (directions), 1 arg: Row/col addr, bottom-top refresh (no delay)
        2, ST77XX_MADCTL , 0x08, 0,                         //  4: Mem access ctrl (directions), 1 arg: Row/col addr, bottom-top refresh (no delay)
        2, ST77XX_COLMOD, 0x55, 10,                         //  3: Set color mode, 1 arg (16 bit color), delay 
        6, ST77XX_PORCTRK, 0x0C, 0x0C, 0x00, 0x33, 0x33, 10,
        2, ST77XX_GCTRL, 0x35, 10,
        2, ST77XX_VCOMS, 0x2B, 10,
        2, ST77XX_LCMCTRL, 0x2C, 10,
        3, ST77XX_VDVVRHEN, 0x01, 0x0F, 10,
        2, ST77XX_VRHS, 0x11, 10,
        2, ST77XX_VDVS, 0x20, 10,
        2, ST77XX_FRCTRL2, 0x0F, 10,
        3, ST77XX_PWCTRL1, 0xA4, 0xA1, 10,
        15, ST77XX_PVGAMCTRL, 0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19, 10,
        15, ST77XX_NVGAMCTRL, 0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19, 10,
        5, ST77XX_CASET, 0x00, 0x00, 0x00, 0xEF, 10,
        5, ST77XX_RASET, 0x00, 0x00, 0x01, 0x3F, 10,
        1, ST77XX_INVON, 10,                                //  7: hack
        1, ST77XX_DISPON, 100
    } ;
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
        LOG_ERR( "Unable to initialize SPI port for LCD ");
        exit( EXIT_FAILURE );
    }
// The following is necessary to fill in the address of our interrupt handler in the vector table. Bit weird...
//  IRQ_CONNECT( DT_IRQN(DT_NODELABEL(spi4)), DT_IRQ(DT_NODELABEL(spi4), priority), nrfx_isr, nrfx_spim_4_irq_handler, 0);  

    uint8_t * st7789_cmd = st7789_init_sequence;
    uint8_t * const st7789_sequence_end = st7789_init_sequence + sizeof( st7789_init_sequence );
    while( st7789_cmd < st7789_sequence_end )
    {
        unsigned int bytes_to_send = *st7789_cmd++;
        unsigned int delay_ms;
        LOG_DBG( "Display cmd = 0x%02X, %d bytes", *st7789_cmd, bytes_to_send);
        if ( !lcd_send( st7789_cmd, bytes_to_send, 1 ) ) exit( EXIT_FAILURE );
        st7789_cmd += bytes_to_send;
        delay_ms = *st7789_cmd++;
        k_sleep( K_MSEC( delay_ms ) );
    }
}

// Initialize the LCD
void lcd_init(void)
{
#define P_LCD_RST NRF_GPIO_PIN_MAP( 0, 10 )
    /* Initialize GPIO and reset the display */
    nrf_gpio_cfg_output( P_LCD_RST );
    nrf_gpio_pin_clear( P_LCD_RST );
    NRFX_DELAY_US( 1000 );
    nrf_gpio_pin_set( P_LCD_RST );
    nrf_gpio_pin_set(P_LCD_SPI_SS);
    nrf_gpio_cfg(P_LCD_SPI_SS,
                         NRF_GPIO_PIN_DIR_OUTPUT,
                         NRF_GPIO_PIN_INPUT_DISCONNECT,
                         NRF_GPIO_PIN_NOPULL,
                         NRF_GPIO_PIN_S0S1,
                         NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_pin_set(P_LCD_DC);
    nrf_gpio_cfg(P_LCD_DC,
                    NRF_GPIO_PIN_DIR_OUTPUT,
                    NRF_GPIO_PIN_INPUT_DISCONNECT,
                    NRF_GPIO_PIN_NOPULL,
                    NRF_GPIO_PIN_S0S1,
                    NRF_GPIO_PIN_NOSENSE);


    /* Initilize SPI driver for LCD */
    lcd_data_init();
    lcd_clrscr( 0x0000 );
    lcd_flush();

    /* Initialize PWM for backlight */
    lcd_backlight_init();
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
        for ( unsigned int i = 0; i < LCD_COLS * LCD_ROWS; i++ ) _screen.data[i] = color;
}

static uint8_t _caset[] = { ST77XX_CASET, 0 >> 8, 0 & 0xFF, (LCD_COLS-1) >> 8, (LCD_COLS-1) & 0xFF  };
static uint8_t _raset[] = { ST77XX_RASET, 0 >> 8, 0 & 0xFF, (LCD_ROWS-1) >> 8, (LCD_ROWS-1) & 0xFF  };

void lcd_flush( void )
{
    // Set window to max: X = 0..239, Y = 0..319
    if ( !lcd_send( _caset, sizeof( _caset ), 1 ) ) LOG_ERR( "CASET command failed");
    else if ( !lcd_send( _raset, sizeof( _raset ), 1 ) ) LOG_ERR( "RASET command failed" );
    else
    {
        // Setup data transfers
        _screen.cmd = ST77XX_RAMWR;
        _screen.row = (uint16_t *)_screen.data;
        if ( lcd_send( (void *)&_screen.cmd, (LCD_COLS * 2) + 1, 1 ) )
        {
            for (int i = 0; i < LCD_ROWS; i++)
            {
                _screen.row += LCD_COLS;
                if ( !lcd_send( _screen.row, (LCD_COLS * 2), 0 ))
                {
                    break;
                }
            }
        }
    }
}

void lcd_plot( unsigned int x, unsigned int y, uint16_t color )
{
    if ( x < LCD_COLS && y < LCD_ROWS ) _screen.data[x + y * LCD_COLS] = color; //(color >> 8) | (color << 8);
}

void lcd_line( unsigned int x0, unsigned int y0, unsigned int x1, unsigned int y1, uint16_t color )
{

}

void lcd_circle( unsigned int x0, unsigned int y0, unsigned int radius, uint16_t color )
{

}
