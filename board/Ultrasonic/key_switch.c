/**
 ******************************************************************************
 * @file    key_switch.c
 * @author  Adam Huang
 * @version V1.0.0
 * @date    28-Nov-2016
 * @brief   
 ******************************************************************************
*/
#include "board_init.h"
#include "app_platform.h"
#include "mico_platform.h"

void key_switch_test( void *arg );
void key_switch_init( void )
{
  platform_pin_config_t pin_config;
  pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
  pin_config.gpio_mode = GPIO_MODE_INPUT;
  pin_config.gpio_pull = GPIO_PULLDOWN;
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_PWRKEY, &pin_config );
  MicoGpioEnableIRQ( MICO_GPIO_PWRKEY, IRQ_TRIGGER_BOTH_EDGES, key_switch_test, NULL );
}

void key_switch_test( void *arg )
{
  static uint8_t presState;
  if ( presState != (uint8_t)MicoGpioInputGet( MICO_GPIO_PWRKEY ) )
  {
    presState = MicoGpioInputGet( MICO_GPIO_PWRKEY );
    if ( presState )
    {
      printf("PWRKEY is High\r\n");
    }
    else
    {
      printf("PWRKEY is Low\r\n");
    }
  }
  
}

