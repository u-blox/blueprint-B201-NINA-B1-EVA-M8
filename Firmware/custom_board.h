#ifndef BOARD_CUSTOM_H
#define BOARD_CUSTOM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_gpio.h"

/**************/
/**** B201 ****/
/**************/
#define LEDS_NUMBER    2

#define LED_1          8	// LED1
#define LED_2          18	// LED2

#define LEDS_ACTIVE_STATE 1

#define LEDS_LIST { LED_1, LED_2 }

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2

#define LEDS_INV_MASK  LEDS_MASK

#define BUTTONS_NUMBER 2

#define BUTTON_1       30	// GPS_EN
#define BUTTON_2       16	// ON/OFF
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1, BUTTON_2 }

#define BSP_BUTTON_0   BUTTON_1
#define BSP_BUTTON_1   BUTTON_2

#define RX_PIN_NUMBER  5
#define TX_PIN_NUMBER  6
#define CTS_PIN_NUMBER 7
#define RTS_PIN_NUMBER 31
#define HWFC           true

#define SPIS_MISO_PIN   12  // SPI MISO signal.
#define SPIS_CSN_PIN    11  // SPI CSN signal.
#define SPIS_MOSI_PIN   13  // SPI MOSI signal.
#define SPIS_SCK_PIN    14  // SPI SCK signal.

#define SPIM0_SCK_PIN   14  // SPI clock GPIO pin number.
#define SPIM0_MOSI_PIN  13  // SPI Master Out Slave In GPIO pin number.
#define SPIM0_MISO_PIN  12  // SPI Master In Slave Out GPIO pin number.
#define SPIM0_SS_PIN    11  // SPI Slave Select GPIO pin number.

#define ARDUINO_SCL_PIN 3  // SCL signal pin
#define ARDUINO_SDA_PIN 2  // SDA signal pin

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,            \
                                 .rc_ctiv       = 0,                                \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM}

#ifdef __cplusplus
}
#endif

#endif // BOARD_CUSTOM_H
