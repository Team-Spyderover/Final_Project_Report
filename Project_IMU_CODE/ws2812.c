#include <stdio.h>
#include <stdlib.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"
#include "ws2812.pio.h"
#include "D:\pico\pico-sdk\src\boards\include\boards\pico.h"

#define IS_RGBW true
#define NUM_PIXELS 150

#ifdef PICO_DEFAULT_WS2812_PIN
#define WS2812_PIN PICO_DEFAULT_WS2812_PIN
#else
// default to pin 2 if the board doesn't have a default WS2812 pin defined
#define WS2812_PIN 2
#endif
#define PICO_DEFAULT_WS2812_POWER_PIN 11
#define OUTER_FWD_PIN 24
#define OUTER_BCK_PIN 4
#define INNER_FWD_PIN 6
#define INNER_BCK_PIN 25

#define X_AXIS_PIN 29
#define Y_AXIS_PIN 28
#define Z_AXIS_PIN 27

#define CONTROL_SIGNAL 20
#define MOTOR 5

void moveW(){
    
}

int main() {

    const uint OUTER_FWD = OUTER_FWD_PIN;
    const uint OUTER_BCK = OUTER_BCK_PIN;
    const uint INNER_FWD = INNER_FWD_PIN;
    const uint INNER_BCK = INNER_BCK_PIN;

    const uint X_PIN = X_AXIS_PIN;
    const uint Y_PIN = Y_AXIS_PIN;
    const uint Z_PIN = Z_AXIS_PIN;

    const uint incoming_signal = CONTROL_SIGNAL;

    gpio_init(OUTER_FWD);
    gpio_init(OUTER_BCK);
    gpio_init(INNER_FWD);
    gpio_init(INNER_BCK);

    gpio_init(incoming_signal);

    gpio_set_dir(OUTER_FWD, GPIO_OUT);
    gpio_set_dir(OUTER_BCK, GPIO_OUT);
    gpio_set_dir(INNER_FWD, GPIO_OUT);
    gpio_set_dir(INNER_BCK, GPIO_OUT);

    gpio_set_dir(incoming_signal, GPIO_IN);

    stdio_init_all();
    adc_init();

    adc_gpio_init(X_PIN);
    adc_gpio_init(Y_PIN);
    adc_gpio_init(Z_PIN);
    int level = 0;
    float x_pos = 0;
    float y_pos = 0;
    float z_pos = 0;
    float x;
    float y;
    float z;

    while (1) {
        if(gpio_get(incoming_signal)){
            x = x_pos;
            y = y_pos;
            z = z_pos;

            adc_select_input(3);
            uint adc_x_raw = adc_read();
            adc_select_input(2);
            uint adc_y_raw = adc_read();
            adc_select_input(1);
            uint adc_z_raw = adc_read();
            
            const float conversion_factor = (1 << 12) - 1;
            uint16_t result = adc_read();
            x_pos = adc_x_raw / conversion_factor;
            y_pos = adc_y_raw / conversion_factor;
            z_pos = adc_z_raw / conversion_factor;
            printf("X-pos: %f, Y-pos: %f, Z-pos: %f \n", x_pos, y_pos, z_pos);
            printf("Level = %d\n", level);

            if(x - x_pos > 0)
            {
                gpio_put(OUTER_FWD,1);
                gpio_put(INNER_FWD,1);
                gpio_put(OUTER_BCK,0);
                gpio_put(INNER_BCK,0);
            }
            else if(x - x_pos < 0)
            {
                gpio_put(OUTER_FWD,0);
                gpio_put(INNER_FWD,0);
                gpio_put(OUTER_BCK,1);
                gpio_put(INNER_BCK,1);
            }
            
            if(y - y_pos > 0)
            {
                gpio_put(OUTER_FWD,1);
                gpio_put(INNER_FWD,0);
                gpio_put(OUTER_BCK,0);
                gpio_put(INNER_BCK,1);
            }
            else if(y - y_pos < 0 )
            {
                gpio_put(OUTER_FWD,0);
                gpio_put(INNER_FWD,1);
                gpio_put(OUTER_BCK,1);
                gpio_put(INNER_BCK,0);
            }
            sleep_ms(500);
        }
        else{
            gpio_put(OUTER_FWD,0);
            gpio_put(INNER_FWD,0);
            gpio_put(OUTER_BCK,0);
            gpio_put(INNER_BCK,0);
            printf("No target found! Please pan the vision! \n");
            sleep_ms(500);
        }
    }
}