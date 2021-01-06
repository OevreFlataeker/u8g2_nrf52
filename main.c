/*
 * u8g2 Demo Application running on Nordic Semiconductors nRF52 DK with nRF SDK 17.02 and:
 *
 * - TWIM and a Waveshield SH1106 OLED 128x64 display
 * or
 * - SPIM and a Waveshield SSD1327 OLED 128x128 16 grey level display
 * that can be found on Ebay/Amazon for few EUR.
 * 
 * Created 20200102 by @daubsi, based on the great u8g2 library https://github.com/olikraus/u8g2
 * Probably not the cleanest code but it works :-D
 * 
 * TWI: Set up with i2c/TWIM in 400 kHz (100 kHz works equally well) in non-blocking mode
 * SPI: Setup up in SPIM with 4 MHz in non-blocking mode
 *
 * Wiring TWI:
 * Connect VCC and GND of the display with VDD and GND on the nRF52 DK
 * Connect SCL to pin 27 ("P0.27") and SDA to pin 26 ("P0.26") on the nRF52 DK
 * 
 * Wiring SPI:
 * Connect VCC and GND of the diplay with VDD and GND on the NRF52 DK
 * Connect CLK (yellow wire) to pin 27 ("P0.27") and DIN (blue wire) to pin 26 ("P0.26") on the nRF52 DK
 * Connect CS (orange wire) to pin 12 ("P0.12") and DC (green wire) to pin 11 ("P0.11") on the nRF52 DK
 * Connect Reset (white wire) to pin 31 ("P0.31") on the nRF52 DK
 *
 * Settings in sdk_config.h
 *
 * TWI:
 * #define NRFX_TWIM_ENABLED 1
 * #define NRFX_TWIM0_ENABLED 1
 * #define NRFX_TWIM1_ENABLED 0
 * #define NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY 104857600
 * #define NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT 0
 * #define TWI_ENABLED 1
 * #define TWI_DEFAULT_CONFIG_FREQUENCY 104857600
 * #define TWI_DEFAULT_CONFIG_CLR_BUS_INIT 0
 * #define TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT 0
 * #define TWI0_ENABLED 1
 * #define TWI0_USE_EASY_DMA 1
 * #define TWI1_ENABLED 0
 * #define TWI1_USE_EASY_DMA 1
 *
 * SWI:
 * #define NRFX_SPIM_ENABLED 1
 * #define NRFX_SPIM0_ENABLED 0
 * #define NRFX_SPIM1_ENABLED 1
 * #define NRFX_SPIM_EXTENDED_ENABLED 0 // Only available on NRF52840 on SPIM3
 * #define NRFX_SPIM_MISO_PULL_CFG 1
 * #define NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY 6
 * 
 * Probably most of those configs are not needed apart from TWI_ENABLED and TWI0_ENABLED but I kept them there
 *
 * When compiling for SPI (== #define USE_SPI 1) and TWI (== #define USE_TWI 1) at the same time, make sure, that you use different
 * HW resources, e.g. TWI_INSTANCE_ID = 0 and SPI_INSTANCE_ID = 1 (and the corresponding TWI0_ENABLED 1, SPIM1_ENABLED 1 in sdk_config.h
 * Otherwise there vill be linker conflicts when using the same id.
 * https://devzone.nordicsemi.com/f/nordic-q-a/35182/irq-handler-compile-error
 *
 * The u8g2 sources have to be unpacked and the csrc subfolder of that archive should be placed in the main directory of this project in a folder u8g2.
 */

#define USE_TWI 0
#define USE_SPI 1

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#if USE_TWI
#include "nrf_drv_twi.h"
#endif

#if USE_SPI
#include <nrfx_spim.h>
#endif

#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "u8g2.h"
#include "u8x8.h"

#if USE_TWI
#define TWI_ADDRESSES      127

#define OLED_I2C_PIN_SCL 27
#define OLED_I2C_PIN_SDA 26
#define OLED_ADDR        0x3C
#endif

#if USE_SPI

#define SPI_INSTANCE 1

static const nrfx_spim_t m_spi = NRFX_SPIM_INSTANCE(SPI_INSTANCE);

#define NRFX_SPIM_SCK_PIN  27 // yellow wire (CLK)
#define NRFX_SPIM_MOSI_PIN 26 // blue wire (DIN)
#define NRFX_SPIM_CS_PIN 12 // orange wire (CS)
#define NRFX_SPIM_DC_PIN 11 // green wire (DC)
#define NRFX_SPIM_RESET_PIN 31 // white wire (Reset)

void spi_handler(nrfx_spim_evt_t const * p_event, void * p_context);
uint8_t u8x8_HW_com_spi_nrf52832(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
uint8_t u8g2_nrf_gpio_and_delay_spi_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
#endif

/* Indicates if operation has ended. */
static volatile bool m_xfer_done = false;
static uint8_t m_sample;

static u8g2_t u8g2;

static bool readsomething = false;

#if USE_TWI
#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
#endif

#if USE_TWI
/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
        {
            // NRF_LOG_ERROR("Got back NACK");
            m_xfer_done = true;            
            break;
        }
        case NRF_DRV_TWI_EVT_DATA_NACK:
        {
            // NRF_LOG_ERROR("Got back D NACK");
            m_xfer_done = true;            
            break;
        }        
        case NRF_DRV_TWI_EVT_DONE:
        {
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                readsomething=true;
            }
            m_xfer_done = true;
            break;
        }        
        default:
        {
            m_xfer_done = false;
            break;
        }
    }    
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_oled_config = {
       .scl                = OLED_I2C_PIN_SCL,
       .sda                = OLED_I2C_PIN_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false,       
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_oled_config, twi_handler, NULL);    
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

uint8_t u8g2_nrf_gpio_and_delay_twi_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch(msg)
    {   
        case U8X8_MSG_DELAY_MILLI:
            // NRF_LOG_INFO("nrf_delay_ms(%d)", arg_int);
            nrf_delay_ms(arg_int);
            break;

        case U8X8_MSG_DELAY_10MICRO:
            // NRF_LOG_INFO("nrf_delay_us(%d)", 10*arg_int);
            nrf_delay_us(10*arg_int);
            break;
        
        default:
            u8x8_SetGPIOResult(u8x8, 1); // default return value
            break;  
    }
    return 1;
}

uint8_t u8x8_HW_com_twi_nrf52832(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    uint8_t *data;
    bool res = false;
    ret_code_t err_code;    
    static uint8_t buffer[32];
    static uint8_t buf_idx;
    switch(msg)
    {
      case U8X8_MSG_BYTE_SEND:
      {
            data = (uint8_t *)arg_ptr;      
            while( arg_int > 0 )
            {
              buffer[buf_idx++] = *data;
              data++;
              arg_int--;
            }      
            break;  
      }      
      case U8X8_MSG_BYTE_START_TRANSFER:                    
      {
            buf_idx = 0;            
            m_xfer_done = false;
            break;
      }
      case U8X8_MSG_BYTE_END_TRANSFER:
      {            
            uint8_t addr = u8x8_GetI2CAddress(u8x8);
                       
            err_code = nrf_drv_twi_tx(&m_twi, u8x8_GetI2CAddress(u8x8) , buffer, buf_idx, false);
            APP_ERROR_CHECK(err_code);
            while (!m_xfer_done)
            {
                __WFE();
            }
            break;
      }
      default:
            return 0;
    }
    return 1;
}

#endif

#if USE_SPI
uint8_t u8g2_nrf_gpio_and_delay_spi_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch(msg)
    {   
        case U8X8_MSG_GPIO_DC:				
            nrf_gpio_pin_write(NRFX_SPIM_DC_PIN, arg_int);            
            break;

        case U8X8_MSG_GPIO_RESET:            
            nrf_gpio_pin_write(NRFX_SPIM_RESET_PIN, arg_int);
            break;

        case U8X8_MSG_DELAY_MILLI:            
            nrf_delay_ms(arg_int);
            break;

        case U8X8_MSG_DELAY_10MICRO:            
            nrf_delay_us(10*arg_int);
            break;
        
        default:
            u8x8_SetGPIOResult(u8x8, 1); // default return value
            break;  
    }
    return 1;
}

void spi_init (void)
{
    ret_code_t err_code;

    nrfx_spim_config_t spi_oled_config = NRFX_SPIM_DEFAULT_CONFIG;

    spi_oled_config.sck_pin   = NRFX_SPIM_SCK_PIN;
    spi_oled_config.mosi_pin  = NRFX_SPIM_MOSI_PIN;
    spi_oled_config.ss_pin    = NRFX_SPIM_CS_PIN;
    spi_oled_config.frequency = NRF_SPIM_FREQ_4M; // The SSD1326 can go up to 10 MHz clock
    spi_oled_config.mode      = NRF_SPIM_MODE_0;
    spi_oled_config.ss_active_high = false;
       
    err_code = nrfx_spim_init(&m_spi, &spi_oled_config, spi_handler, NULL);    
    APP_ERROR_CHECK(err_code);

    // Enable the out-of-band GPIOs
    nrf_gpio_cfg_output(NRFX_SPIM_DC_PIN);
    nrf_gpio_cfg_output(NRFX_SPIM_RESET_PIN);
    
}

void spi_handler(nrfx_spim_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRFX_SPIM_EVENT_DONE:
                m_xfer_done = true;
                break;
    }
}

uint8_t u8x8_HW_com_spi_nrf52832(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    uint8_t *data;
    bool res = false;
    ret_code_t err_code;    
    static uint8_t buffer[64];
    static uint8_t buf_idx = 1;
    
    switch(msg)
    {
      case U8X8_MSG_BYTE_SEND:
      {
            buf_idx = 0;  
            data = (uint8_t *)arg_ptr;            
            while( arg_int > 0 )
            {
              buffer[buf_idx++] = *data;
              data++;
              arg_int--;
            }

            m_xfer_done = false;
            nrfx_spim_xfer_desc_t spim_xfer_desc = NRFX_SPIM_XFER_TX(&buffer, buf_idx);    
            err_code = nrfx_spim_xfer(&m_spi, &spim_xfer_desc,0);
            APP_ERROR_CHECK(err_code);
            while (!m_xfer_done)
            {
                __WFE();
            }
            break;  
      }      
      case U8X8_MSG_BYTE_SET_DC:
      {            
            u8x8_gpio_SetDC(u8x8, arg_int);
            break;
      }
      case U8X8_MSG_BYTE_START_TRANSFER:                    
      {
            buf_idx = 0;                        
            
            break;
      }
      case U8X8_MSG_BYTE_END_TRANSFER:
      {                   
            break;
      }
      default:
            return 0;
    }
    return 1;
}


#endif

void print_hello()
{
static int x = 30;
static int y= 30;
    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    x = (++x % 20);
    y = (++y % 20);
    u8g2_DrawStr(&u8g2, y, x, "Hello World!");
    u8g2_SendBuffer(&u8g2);
}

void drawLogo(void)
{
    u8g2_SetFontMode(&u8g2, 1);	// Transparent
#ifdef MINI_LOGO

    u8g2.setFontDirection(0);
    u8g2.setFont(u8g2_font_inb16_mf);
    u8g2.drawStr(0, 22, "U");
    
    u8g2.setFontDirection(1);
    u8g2.setFont(u8g2_font_inb19_mn);
    u8g2.drawStr(14,8,"8");
    
    u8g2.setFontDirection(0);
    u8g2.setFont(u8g2_font_inb16_mf);
    u8g2.drawStr(36,22,"g");
    u8g2.drawStr(48,22,"\xb2");
    
    u8g2.drawHLine(2, 25, 34);
    u8g2.drawHLine(3, 26, 34);
    u8g2.drawVLine(32, 22, 12);
    u8g2.drawVLine(33, 23, 12);
#else
    u8g2_SetFontDirection(&u8g2, 0);
    u8g2_SetFont(&u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(&u8g2, 0, 30, "U");
    
    u8g2_SetFontDirection(&u8g2, 1);
    u8g2_SetFont(&u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(&u8g2, 21,8,"8");
        
    u8g2_SetFontDirection(&u8g2, 0);
    u8g2_SetFont(&u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(&u8g2, 51,30,"g");
    u8g2_DrawStr(&u8g2, 67,30,"\xb2");
    
    u8g2_DrawHLine(&u8g2, 2, 35, 47);
    u8g2_DrawHLine(&u8g2, 3, 36, 47);
    u8g2_DrawVLine(&u8g2, 45, 32, 12);
    u8g2_DrawVLine(&u8g2, 46, 33, 12);
    
#endif
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{    
    ret_code_t err_code;
    uint8_t address;
    
    bool detected_device = false;

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

#if USE_TWI    
    twi_init();
    
    readsomething=false;
    for (address = 0x0; address <= TWI_ADDRESSES; address++)
    {
        if (detected_device) break;        
        m_xfer_done = false;        
        err_code = nrf_drv_twi_rx(&m_twi, address, &m_sample, sizeof(m_sample));
        while (!m_xfer_done)
        {
            __WFE();
        }
        if (readsomething)
        {
            detected_device = true;
            NRF_LOG_INFO("TWI device detected at address 0x%x: Read value 0x%02x", address, m_sample);
        }
        NRF_LOG_FLUSH();
    }

    if (!detected_device)
    {
        NRF_LOG_INFO("No device was found.");
        NRF_LOG_FLUSH();
        while (true)
        {
      
        }
    }
           
    
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_HW_com_twi_nrf52832, u8g2_nrf_gpio_and_delay_twi_cb);    
    u8g2_SetI2CAddress(&u8g2, OLED_ADDR);
#elif USE_SPI
    spi_init();        
    u8g2_Setup_ssd1327_ws_128x128_f(&u8g2, U8G2_R0, u8x8_HW_com_spi_nrf52832, u8g2_nrf_gpio_and_delay_spi_cb);

#endif    
    
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2,0);
    
    while (true)
    {
        /*
        u8g2_ClearBuffer(&u8g2);
        drawLogo();
        u8g2_SendBuffer(&u8g2);
        */
        print_hello();
        nrf_delay_ms(10);
    }
}

