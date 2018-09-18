/**
 ******************************************************************************
 * @file    board_init.c
 * @author  Adam Huang
 * @version V1.0.0
 * @date    26-Nov-2016
 * @brief
 ******************************************************************************
 */
#include "board_init.h"
#include "platform.h"
#include "mico_platform.h"


#define board_log(M, ...) custom_log("Board", M, ##__VA_ARGS__)
#define board_log_trace() custom_log_trace("Board")

extern const platform_gpio_t            platform_gpio_pins[];
extern const platform_adc_t             platform_adc_peripherals[];

void board_gpios_init( void )
{
    platform_pin_config_t pin_config;
    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode = GPIO_MODE_AF_PP;// GPIO_MODE_OUTPUT_PP;//
    pin_config.gpio_pull = GPIO_PULLUP;

    //  Initialise system led
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_SYS_LED, &pin_config );

    EnableSwjAndDisableJtag();

    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
    pin_config.gpio_pull = GPIO_PULLUP;

    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_SYS_LED, &pin_config );

    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_TRIG, &pin_config );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_TRIG );


    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SYS_LED );

    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode = GPIO_MODE_INPUT;
    pin_config.gpio_pull = GPIO_PULLUP;

    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_ULTRA_DATA, &pin_config );
    extern void set_us_io_input_it(void);
    set_us_io_input_it();

    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S0, &pin_config );
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S1, &pin_config );
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S2, &pin_config );
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S3, &pin_config );
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S4, &pin_config );
    //MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S5, &pin_config );

    EnableSwjAndDisableJtag();
}
