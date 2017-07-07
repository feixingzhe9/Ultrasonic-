/**
 ******************************************************************************
 * @file    board_init.h
 * @author  Adam Huang
 * @version V1.0.0
 * @date    26-Nov-2016
 * @brief   
 ******************************************************************************
*/
#pragma once

#include "platform_peripheral.h"
#include "platform.h"
#include "platform_config.h"

typedef struct
{
    const platform_gpio_t* pin_motor;
    const platform_gpio_t* pin_sensor;
    const platform_gpio_t* pin_leds;

} board_module_enable_t;

void board_gpios_init( void );
void board_adc_dma_init( void *buffer, uint32_t length );
void charger_detect_init( void );

void key_switch_init( void );
void key_switch_test( void *arg );
