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
#include "app_platform.h"
//#include "serial_leds.h"

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
    
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_ULTRA_DATA, &pin_config );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SYS_LED ); 
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_ULTRA_DATA );

    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_24_V_EN, &pin_config );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_24_V_EN ); 
    
    //
    pin_config.gpio_mode = GPIO_MODE_INPUT;

    
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S0, &pin_config );
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S1, &pin_config );
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S2, &pin_config );
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S3, &pin_config );
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S4, &pin_config );
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_KEY_S5, &pin_config );
    
    EnableSwjAndDisableJtag();
}



