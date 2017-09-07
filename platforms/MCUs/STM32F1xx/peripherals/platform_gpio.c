/**
 ******************************************************************************
 * @file    platform_gpio.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide GPIO driver functions.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include "platform.h"
#include "platform_peripheral.h"
#include "platform_logging.h"
#include "app_platform.h"

/******************************************************
*                    Constants
******************************************************/

/******************************************************
*                   Enumerations
******************************************************/

/******************************************************
*                 Type Definitions
******************************************************/

/******************************************************
*                    Structures
******************************************************/

/* Structure of runtime GPIO IRQ data */
typedef struct
{
    platform_gpio_port_t*        owner_port; // GPIO port owning the IRQ line (line is shared across all GPIO ports)
    platform_gpio_irq_callback_t handler;    // User callback
    void*                        arg;        // User argument to be passed to the callbackA
} platform_gpio_irq_data_t;

/******************************************************
*               Variables Definitions
******************************************************/
/* GPIO peripheral clocks */
const uint32_t gpio_peripheral_clocks[NUMBER_OF_GPIO_PORTS] =
{
    [0] = RCC_APB2ENR_IOPAEN,
    [1] = RCC_APB2ENR_IOPBEN,
    [2] = RCC_APB2ENR_IOPCEN,
    [3] = RCC_APB2ENR_IOPDEN,
#if defined(STM32F103xE)
    [4] = RCC_APB2ENR_IOPEEN,
    [5] = RCC_APB2ENR_IOPFEN,
    [6] = RCC_APB2ENR_IOPGEN,
#endif
};

/* Runtime GPIO IRQ data */
static volatile platform_gpio_irq_data_t gpio_irq_data[NUMBER_OF_GPIO_IRQ_LINES];

/******************************************************
*               Function Declarations
******************************************************/

/******************************************************
*               Function Definitions
******************************************************/

OSStatus platform_gpio_init( const platform_gpio_t* gpio, platform_pin_config_t *config )
{
  GPIO_InitTypeDef  gpio_init_structure;
  uint8_t           port_number;
  OSStatus          err = kNoErr;

  platform_mcu_powersave_disable();
  require_action_quiet( gpio != NULL, exit, err = kParamErr);
  
  port_number = platform_gpio_get_port_number( gpio->port );
  require_action_quiet( port_number != 0xFF, exit, err = kParamErr);

  RCC->APB2ENR |= gpio_peripheral_clocks[port_number];

  gpio_init_structure.Speed = config->gpio_speed;
  gpio_init_structure.Mode  = config->gpio_mode;
  gpio_init_structure.Pull = config->gpio_pull;

  gpio_init_structure.Pin = (uint32_t) ( 1 << gpio->pin_number );

  HAL_GPIO_Init( gpio->port, &gpio_init_structure );
  
exit:
  platform_mcu_powersave_enable();
  return err;
}

OSStatus platform_gpio_deinit( const platform_gpio_t* gpio )
{
  OSStatus          err = kNoErr;

  platform_mcu_powersave_disable();
  require_action_quiet( gpio != NULL, exit, err = kParamErr);

  HAL_GPIO_DeInit( gpio->port, (uint32_t) ( 1 << gpio->pin_number ));

exit:
  platform_mcu_powersave_enable();
  return err;
}

OSStatus platform_gpio_output_high( const platform_gpio_t* gpio )
{
  OSStatus err = kNoErr;

  platform_mcu_powersave_disable();

  require_action_quiet( gpio != NULL, exit, err = kParamErr);

  gpio->port->BSRR = (uint16_t) ( 1 << gpio->pin_number );
  
exit:
  platform_mcu_powersave_enable();
  return err;
}

OSStatus platform_gpio_output_low( const platform_gpio_t* gpio )
{
  OSStatus err = kNoErr;

  require_action_quiet( gpio != NULL, exit, err = kParamErr);

  platform_mcu_powersave_disable();
  
  gpio->port->BSRR = (uint32_t) ( 1 << gpio->pin_number ) << 16;
  
exit:
  platform_mcu_powersave_enable();
  return err;
}

OSStatus platform_gpio_output_trigger( const platform_gpio_t* gpio )
{
  OSStatus err = kNoErr;

  platform_mcu_powersave_disable();

  require_action_quiet( gpio != NULL, exit, err = kParamErr);

  gpio->port->ODR ^= (uint16_t) ( 1 << gpio->pin_number );
  
exit:
  platform_mcu_powersave_enable();
  return err;
}

bool platform_gpio_input_get( const platform_gpio_t* gpio )
{
  bool result = false;

  platform_mcu_powersave_disable();

  require_quiet( gpio != NULL, exit);

  result = ( ( gpio->port->IDR & (uint32_t) ( 1 << gpio->pin_number ) ) != 0 ) ? true : false;
  
exit:
  platform_mcu_powersave_enable();
  return result;
}

OSStatus platform_gpio_irq_enable( const platform_gpio_t* gpio, platform_gpio_irq_trigger_t trigger, platform_gpio_irq_callback_t handler, void* arg )
{
  GPIO_InitTypeDef  gpio_init_structure;
  uint32_t interrupt_line = (uint32_t) ( 1 << gpio->pin_number );
  OSStatus err = kNoErr;
  IRQn_Type        interrupt_vector = (IRQn_Type)0;

  platform_mcu_powersave_disable();
  require_action_quiet( gpio != NULL, exit, err = kParamErr);

  switch ( trigger )
  {
    case IRQ_TRIGGER_RISING_EDGE:
    {
      gpio_init_structure.Mode = GPIO_MODE_IT_RISING;
      break;
    }
    case IRQ_TRIGGER_FALLING_EDGE:
    {
      gpio_init_structure.Mode = GPIO_MODE_IT_FALLING;
      break;
    }
    case IRQ_TRIGGER_BOTH_EDGES:
    {
      gpio_init_structure.Mode = GPIO_MODE_IT_RISING_FALLING;
      break;
    }
    default:
    {
      err =  kParamErr;
      goto exit;
    }
  }
  gpio_init_structure.Speed = GPIO_SPEED_MEDIUM;
  gpio_init_structure.Pull  = GPIO_PULLUP;//GPIO_PULLDOWN;
  
  HAL_GPIO_Init( gpio->port, &gpio_init_structure );
  
  if ( ( interrupt_line & 0x001F ) != 0 )
  {
    /* Line 0 to 4 */
    interrupt_vector = (IRQn_Type) ( EXTI0_IRQn + gpio->pin_number );
  }
  else if ( ( interrupt_line & 0x03E0 ) != 0 )
  {
    /* Line 5 to 9 */
    interrupt_vector = EXTI9_5_IRQn;
  }
  else if ( ( interrupt_line & 0xFC00 ) != 0 )
  {
    /* Line 10 to 15 */
    interrupt_vector = EXTI15_10_IRQn;
  }
  
  /* Clear interrupt flag */
  EXTI->PR = interrupt_line;
  NVIC_ClearPendingIRQ( interrupt_vector ); 

  NVIC_EnableIRQ( interrupt_vector );
  
  gpio_irq_data[gpio->pin_number].owner_port = gpio->port;
  gpio_irq_data[gpio->pin_number].handler    = handler;
  gpio_irq_data[gpio->pin_number].arg        = arg;

exit:
  platform_mcu_powersave_enable();
  return err;
}


OSStatus platform_gpio_irq_disable( const platform_gpio_t* gpio )
{
  uint16_t interrupt_line;
  OSStatus err = kNoErr;

  platform_mcu_powersave_disable();
  require_action_quiet( gpio != NULL, exit, err = kParamErr);

  interrupt_line = (uint16_t) ( 1 << gpio->pin_number );

  if ( ( EXTI->IMR & interrupt_line ) && gpio_irq_data[gpio->pin_number].owner_port == gpio->port )
  {
    bool             interrupt_line_used = 0;
    IRQn_Type        interrupt_vector    = (IRQn_Type)0;

    /* Disable NVIC interrupt */
    if ( ( interrupt_line & 0x001F ) != 0 )
    {
      /* Line 0 to 4 */
      interrupt_vector = (IRQn_Type) ( EXTI0_IRQn + gpio->pin_number );
      interrupt_line_used = false;
    }
    else if ( ( interrupt_line & 0x03E0 ) != 0 )
    {
      /* Line 5 to 9 */
      interrupt_vector = EXTI9_5_IRQn;
      interrupt_line_used = ( ( EXTI->IMR & 0x3e0U ) != 0 ) ? true : false;
    }
    else if ( ( interrupt_line & 0xFC00 ) != 0 )
    {
      /* Line 10 to 15 */
      interrupt_vector = EXTI15_10_IRQn;
      interrupt_line_used = ( ( EXTI->IMR & 0xfc00U ) != 0 ) ? true : false;
    }

    /* Some IRQ lines share a vector. Disable vector only if not used */
    if ( interrupt_line_used == false )
    {
      NVIC_DisableIRQ( interrupt_vector );
    }

    gpio_irq_data[gpio->pin_number].owner_port = 0;
    gpio_irq_data[gpio->pin_number].handler    = 0;
    gpio_irq_data[gpio->pin_number].arg        = 0;
  }

exit:
  platform_mcu_powersave_enable();
  return err;
}


/******************************************************
 *      STM32F1xx Internal Function Definitions
 ******************************************************/
OSStatus platform_gpio_irq_manager_init( void )
{
    memset( (void*)gpio_irq_data, 0, sizeof( gpio_irq_data ) );

    /* Switch on SYSCFG peripheral clock to allow writing into SYSCFG registers */
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    return kNoErr;
}

uint8_t platform_gpio_get_port_number( platform_gpio_port_t* gpio_port )
{
    switch ( (uint32_t) gpio_port )
    {
        case GPIOA_BASE:
            return 0;
        case GPIOB_BASE:
            return 1;
        case GPIOC_BASE:
            return 2;
        case GPIOD_BASE:
            return 3;
#if defined(STM32F103xE)
        case GPIOE_BASE:
            return 4;
        case GPIOF_BASE:
            return 5;
        case GPIOG_BASE:
            return 6;
#endif
        default:
            return 0xFF;
    }
}

OSStatus platform_gpio_enable_clock( const platform_gpio_t* gpio )
{
  uint8_t     port_number;
  OSStatus    err = kNoErr;

  require_action_quiet( gpio != NULL, exit, err = kParamErr);
  
  /* Enable peripheral clock for this port */
  port_number = platform_gpio_get_port_number( gpio->port );
  require_action_quiet( port_number != 0xFF, exit, err = kParamErr);

  RCC->APB2ENR |= gpio_peripheral_clocks[port_number];

exit:
  return err;

}


OSStatus platform_gpio_set_alternate_function( platform_gpio_port_t* gpio_port, uint8_t pin_number, uint32_t pull_type, uint8_t alternation_function )
{
    GPIO_InitTypeDef  gpio_init_structure;
    uint8_t           port_number = platform_gpio_get_port_number( gpio_port );

    platform_mcu_powersave_disable();

    /* Enable peripheral clock for this port */
    RCC->APB2ENR |= gpio_peripheral_clocks[port_number];

    gpio_init_structure.Speed = GPIO_SPEED_HIGH;
    gpio_init_structure.Mode  = GPIO_MODE_AF_PP;
    gpio_init_structure.Pull  = pull_type;
    gpio_init_structure.Pin   = (uint32_t) ( 1 << pin_number );

    HAL_GPIO_Init( gpio_port, &gpio_init_structure );
//    GPIO_PinAFConfig( gpio_port, pin_number, alternation_function );
    (void)alternation_function;
    platform_mcu_powersave_enable();

    return kNoErr;
}

/******************************************************
 *               IRQ Handler Definitions
 ******************************************************/

/* Common IRQ handler for all GPIOs */
MICO_RTOS_DEFINE_ISR( gpio_irq )
{
    uint32_t active_interrupt_vector = (uint32_t) ( ( SCB->ICSR & 0x3fU ) - 16 );
    uint32_t gpio_number;
    uint16_t interrupt_line;

    switch ( active_interrupt_vector )
    {
        case EXTI0_IRQn:
            interrupt_line = 1 << 0;
            gpio_number = 0;
            break;
        case EXTI1_IRQn:
            interrupt_line = 1 << 1;
            gpio_number = 1;
            break;
        case EXTI2_IRQn:
            interrupt_line = 1 << 2;
            gpio_number = 2;
            break;
        case EXTI3_IRQn:
            interrupt_line = 1 << 3;
            gpio_number = 3;
            break;
        case EXTI4_IRQn:
            interrupt_line = 1 << 4;
            gpio_number = 4;
            break;
        case EXTI9_5_IRQn:
            interrupt_line = 1 << 5;
            for ( gpio_number = 5; gpio_number < 10 && ( EXTI->PR & interrupt_line ) == 0; gpio_number++ )
            {
                interrupt_line <<= 1;
            }
            break;
        case EXTI15_10_IRQn:
            interrupt_line = 1 << 10;
            for ( gpio_number = 10; gpio_number < 16 && ( EXTI->PR & interrupt_line ) == 0; gpio_number++ )
            {
                interrupt_line <<= 1;
            }
            break;
        default:
            return;
    }

    /* Clear interrupt flag */
    EXTI->PR = interrupt_line;

    /* Call the respective GPIO interrupt handler/callback */
    if ( gpio_irq_data[gpio_number].handler != NULL )
    {
        void * arg = gpio_irq_data[gpio_number].arg; /* Avoids undefined order of access to volatiles */
        gpio_irq_data[gpio_number].handler( arg );
    }
}

/******************************************************
 *               IRQ Handler Mapping
 ******************************************************/
MICO_RTOS_DEFINE_ISR( EXTI0_IRQHandler )
{
  gpio_irq();
}

MICO_RTOS_DEFINE_ISR( EXTI1_IRQHandler )
{
  gpio_irq();
}

MICO_RTOS_DEFINE_ISR( EXTI2_IRQHandler )
{
  gpio_irq();
}

MICO_RTOS_DEFINE_ISR( EXTI3_IRQHandler )
{
  gpio_irq();
}

MICO_RTOS_DEFINE_ISR( EXTI4_IRQHandler )
{
  gpio_irq();
}


//extern uint32_t time_cnt_test;
//#define UltraSonicLog(format, ...)  custom_log("PowerBoard", format, ##__VA_ARGS__)
//#include "platform_tim.h"
MICO_RTOS_DEFINE_ISR( EXTI9_5_IRQHandler )
{
  gpio_irq();
  //time_cnt_test = GetTimerCount();
  //UltraSonicLog("E:%d",time_cnt_test);
  
  
}

MICO_RTOS_DEFINE_ISR( EXTI15_10_IRQHandler )
{
  gpio_irq();
}

void EnableSwjAndDisableJtag(void)
{
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
}



