#include "main.h"

void rgb_itf_update(rgb_s *leds)
{
    for(uint8_t i = 0; i < ADAPTER_RGB_COUNT; i++)
    {
        pio_sm_put_blocking(RGB_PIO, RGB_SM, leds[i].color);
    }
}

void rgb_itf_init()
{
    // set pin 9 to high to give power to led
    gpio_init(GREEN_LED_PIN);
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);
    gpio_put(GREEN_LED_PIN, 1);

    uint offset = pio_add_program(RGB_PIO, &RGB_PIO_PROGRAM);
    RGB_PIO_INIT(RGB_PIO, RGB_SM, offset, UTIL_RGB_PIN, UTIL_RGBW_EN);
}