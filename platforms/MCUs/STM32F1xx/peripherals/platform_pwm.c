/**
 ******************************************************************************
 * @file    paltform_pwm.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide PWM driver functions.
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

/******************************************************
*               Variables Definitions
******************************************************/

/******************************************************
*               Function Declarations
******************************************************/

/******************************************************
*               Function Definitions
******************************************************/

OSStatus platform_pwm_init( const platform_pwm_t* pwm, uint32_t frequency, float duty_cycle )
{
//  TIM_TimeBaseInitTypeDef tim_time_base_structure;
  TIM_OC_InitTypeDef       tim_oc_init_structure;
  RCC_ClkInitTypeDef       rcc_clock_frequencies = {0};
  uint32_t                latency;  /* Temporary variable to retrieve Flash Latency */
  uint32_t                timer_clock_frequency = 0;             /* Timer clock frequency */
  uint32_t                timer_prescaler = 0;                   /* Time base prescaler to have timebase aligned on minimum frequency possible */
  uint16_t                period              = 0;
  float                   adjusted_duty_cycle = ( ( duty_cycle > 100.0f ) ? 100.0f : duty_cycle );
  OSStatus                err                 = kNoErr;
  
  platform_mcu_powersave_disable();
  
  require_action_quiet( pwm != NULL, exit, err = kParamErr);
    
  HAL_RCC_GetClockConfig( &rcc_clock_frequencies, &latency );
  
  pwm->tim_handle->Instance = pwm->tim;
  if ( pwm->tim_handle->Instance == TIM1 || pwm->tim_handle->Instance == TIM2 )
  {
//    RCC_APB2PeriphClockCmd( pwm->tim_peripheral_clock, ENABLE );
    SET_BIT(RCC->APB2ENR, pwm->tim_peripheral_clock);
    /* If APB2 prescaler is different of 1, timers have a factor x2 on their    */
    /* clock source.                                                            */
    if( rcc_clock_frequencies.APB2CLKDivider == RCC_HCLK_DIV1 )
    {
//      period = (uint16_t)( HAL_RCC_GetPCLK2Freq() / frequency - 1 ); /* Auto-reload value counts from 0; hence the minus 1 */
      timer_clock_frequency = HAL_RCC_GetPCLK2Freq();
    }
    else
    {
//      period = (uint16_t)( HAL_RCC_GetPCLK2Freq()*2 / frequency - 1 ); /* Auto-reload value counts from 0; hence the minus 1 */
      timer_clock_frequency = HAL_RCC_GetPCLK2Freq() *2;
    }
  }
  else
  {
//    RCC_APB1PeriphClockCmd( pwm->tim_peripheral_clock, ENABLE );
    SET_BIT(RCC->APB1ENR, pwm->tim_peripheral_clock);
    /* If APB1 prescaler is different of 1, timers have a factor x2 on their    */
    /* clock source.                                                            */
    if( rcc_clock_frequencies.APB1CLKDivider == RCC_HCLK_DIV1 )
    {
//      period = (uint16_t)( HAL_RCC_GetPCLK1Freq() / frequency - 1 ); /* Auto-reload value counts from 0; hence the minus 1 */
      timer_clock_frequency = HAL_RCC_GetPCLK1Freq();
    }
    else
    {
//      period = (uint16_t)( HAL_RCC_GetPCLK1Freq()*2 / frequency - 1 ); /* Auto-reload value counts from 0; hence the minus 1 */
      timer_clock_frequency = HAL_RCC_GetPCLK1Freq() *2;
    }
  }

  /* Timer prescaler calculation */
  /* (computation for timer 16 bits, additional + 1 to round the prescaler up) */
  timer_prescaler = 1 + 1;//(timer_clock_frequency / ((0xFFFF-1) * 1)) +1;
  period = (timer_clock_frequency / (timer_prescaler * frequency)) - 1;
  /* Set alternate function */
  platform_gpio_set_alternate_function( pwm->pin->port, pwm->pin->pin_number, GPIO_PULLDOWN, pwm->gpio_af );
    
  /* Time base configuration */
  pwm->tim_handle->Init.Period            = (uint32_t) period;
  pwm->tim_handle->Init.Prescaler         = (uint16_t)(timer_prescaler - 1);  /* Divide clock by 1+1 to enable a count of high cycle + low cycle = 1 PWM cycle */
  pwm->tim_handle->Init.ClockDivision     = 0;
  pwm->tim_handle->Init.CounterMode       = TIM_COUNTERMODE_UP;
  pwm->tim_handle->Init.RepetitionCounter = 0;
  require_action_quiet( HAL_TIM_PWM_Init(pwm->tim_handle) == HAL_OK, exit, err = kGeneralErr );
  
  /* PWM1 Mode configuration */
  tim_oc_init_structure.OCMode       = TIM_OCMODE_PWM1;
  tim_oc_init_structure.OCFastMode   = TIM_OCFAST_ENABLE;
  tim_oc_init_structure.Pulse        = (uint16_t) ( adjusted_duty_cycle * (float) period / 100.0f );
  tim_oc_init_structure.OCPolarity   = TIM_OCPOLARITY_HIGH;
  tim_oc_init_structure.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
  tim_oc_init_structure.OCIdleState  = TIM_OCNIDLESTATE_RESET;
  tim_oc_init_structure.OCNIdleState = TIM_OCNIDLESTATE_SET;
  
  
  HAL_TIM_PWM_ConfigChannel( pwm->tim_handle, &tim_oc_init_structure, pwm->channel );
  HAL_TIM_PWM_Start( pwm->tim_handle, pwm->channel);

exit:  
  platform_mcu_powersave_enable();
  return err;
}

OSStatus platform_pwm_start( const platform_pwm_t* pwm )
{
  OSStatus err = kNoErr;
  
  platform_mcu_powersave_disable();

  require_action_quiet( pwm != NULL, exit, err = kParamErr);
//  TIM_Cmd( pwm->tim, ENABLE );
//  TIM_CtrlPWMOutputs( pwm->tim, ENABLE );
  HAL_TIM_PWM_Start( pwm->tim_handle, pwm->channel);
  
exit:  
  platform_mcu_powersave_enable();
  return err;
}

OSStatus platform_set_pwm_duty( const platform_pwm_t* pwm, float duty_cycle )
{
  OSStatus err = kNoErr;
  uint16_t current_period;
  uint16_t set_pulse;
  float    adjusted_duty_cycle = ( ( duty_cycle > 100.0f ) ? 100.0f : duty_cycle );
  
  platform_mcu_powersave_disable();
  require_action_quiet( pwm != NULL, exit, err = kParamErr);
  
  current_period = pwm->tim->ARR;
  set_pulse = (uint16_t) ( adjusted_duty_cycle * (float) current_period / 100.0f );
  switch( pwm->channel )
  {
  case TIM_CHANNEL_1:
    pwm->tim->CCR1 = set_pulse;
    break;
  case TIM_CHANNEL_2:
    pwm->tim->CCR2 = set_pulse;
    break;
  case TIM_CHANNEL_3:
    pwm->tim->CCR3 = set_pulse;
    break;
  case TIM_CHANNEL_4:
    pwm->tim->CCR4 = set_pulse;
    break;
  }
exit:  
  platform_mcu_powersave_enable();
  return err;
}

OSStatus platform_pwm_stop( const platform_pwm_t* pwm )
{
  OSStatus err = kNoErr;
  
  platform_mcu_powersave_disable();

  require_action_quiet( pwm != NULL, exit, err = kParamErr);
  
//  TIM_CtrlPWMOutputs( pwm->tim, DISABLE );
//  TIM_Cmd( pwm->tim, DISABLE );
  HAL_TIM_PWM_Stop( pwm->tim_handle, pwm->channel);
  
exit:  
  platform_mcu_powersave_enable();
  return err;
}




