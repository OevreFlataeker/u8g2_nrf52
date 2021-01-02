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

Then add the following drivers to the project

$SDKROOT\integration\nrfx\legacy\nrf_drv_twi.c
$SDKROOT\modules\nrfx\drivers\src\nrfx_twi.c
$SDKROOT\modules\nrfx\drivers\src\nrfx_twim.c


