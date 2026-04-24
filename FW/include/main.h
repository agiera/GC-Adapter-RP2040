#include "adapter_includes.h"
#include "joybus.pio.h"
#include "hardware/gpio.h"
#include "pico/stdio.h"

// RGB LED type selector (0 = WS2812, 1 = SK6805-EC15)
#define RGB_LED_TYPE 1

#if RGB_LED_TYPE == 0
#include "ws2812.pio.h"
#define RGB_PIO_PROGRAM ws2812_program
#define RGB_PIO_INIT ws2812_program_init
#else
#include "sk6805_ec15.pio.h"
#define RGB_PIO_PROGRAM sk6805_ec15_program
#define RGB_PIO_INIT sk6805_ec15_program_init
#endif

#define RGB_PIO pio1
#define RGB_SM 0

#define UTIL_RGB_PIN   10
#define UTIL_RGB_COUNT 1
#define UTIL_RGBW_EN 0

#define RED_LED_PIN 8
#define GREEN_LED_PIN 9
#define BLUE_LED_PIN 10

#define JOYBUS_PIO pio0

#define JOYBUS_PORT_1 22
#define JOYBUS_PORT_2 23
#define JOYBUS_PORT_3 24
#define JOYBUS_PORT_4 25

#define DEFAULT_MODE INPUT_MODE_XINPUT
