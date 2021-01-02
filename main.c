/*
 * u8g2 Demo Application running on Nordic Semiconductors nRF52 DK with nRF SDK 17.02 and TWIM and a SH1106 OLED 128x64 display
 * that can be found on Ebay/Amazon for few EUR.
 * 
 * Created 20200102 by @daubsi, based on the great u8g2 library https://github.com/olikraus/u8g2
 * Probably not the cleanest code but it works :-D
 * 
 * Set up with i2c/TWIM in 400 kHz (100 kHz works equally well) in non-blocking mode
 *
 * Wiring:
 * Connect VCC and GND of the display with VDD and GND on the nRF52 DK
 * Connect SCL to Pin 27 ("P0.27") and SDA to pin 26 ("P0.26") on the nRF52 DK
 * 
 * Settings in sdk_config.h
 *
 * #define NRFX_TWIM_ENABLED 1
 * #define NRFX_TWIM0_ENABLED 0
 * #define NRFX_TWIM1_ENABLED 0
 * #define NRFX_TWIM_DEFAULT_CONFIG_FREQUENCY 104857600
 * #define NRFX_TWIM_DEFAULT_CONFIG_HOLD_BUS_UNINIT 0
 * #define TWI_ENABLED 1
 * #define TWI_DEFAULT_CONFIG_FREQUENCY 104857600
 * #define TWI_DEFAULT_CONFIG_CLR_BUS_INIT 0
 * #define TWI_DEFAULT_CONFIG_HOLD_BUS_UNINIT 0
 * #define TWI0_ENABLED 1
 * #define TWI0_USE_EASY_DMA 1
 * #define TWI1_ENABLED 1
 * #define TWI1_USE_EASY_DMA 1
 *
 * Probably most of those configs are not needed apart from TWI_ENABLED and TWI0_ENABLED but I kept them there
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "u8g2.h"
#include "u8x8.h"

#define TWI_ADDRESSES      127

#define OLED_I2C_PIN_SCL 27
#define OLED_I2C_PIN_SDA 26
#define OLED_ADDR        0x3C


/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;
static uint8_t m_sample;

static u8g2_t u8g2;

static bool readsomething = false;

#if TWI0_ENABLED
#define TWI_INSTANCE_ID     0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID     1
#endif

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
        {
            NRF_LOG_ERROR("Got back NACK");
            m_xfer_done = true;            
            break;
        }
        case NRF_DRV_TWI_EVT_DATA_NACK:
        {
            NRF_LOG_ERROR("Got back D NACK");
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
    //err_code = nrf_drv_twi_init(&m_twi, &twi_oled_config, NULL, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

uint8_t u8g2_nrf_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)

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

uint8_t u8x8_HW_com_nrf52832(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
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
            // NRF_LOG_INFO("Dumping send buffer of len %d ('buf_idx')", buf_idx);
            // NRF_LOG_HEXDUMP_INFO(buffer, buf_idx);
            uint8_t addr = u8x8_GetI2CAddress(u8x8);
            // NRF_LOG_INFO("Now calling 'nrf_drv_twi_tx(&m_twi, 0x%02x, buffer, buf_idx, false)'", addr);
           
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

void print_hello()
{
static int x = 10;
static int y= 0;
    
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);
    x = (++x % 10);
    y = (++y % 10);
    u8g2_DrawStr(&u8g2, y, x, "Hello World!");
    u8g2_SendBuffer(&u8g2);
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
           
    
    u8g2_Setup_sh1106_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_HW_com_nrf52832, u8g2_nrf_gpio_and_delay_cb);    
    u8g2_SetI2CAddress(&u8g2, OLED_ADDR);
    
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2,0);
 
    while (true)
    {        
        nrf_delay_ms(10);
        print_hello();               
    } 
    
}

