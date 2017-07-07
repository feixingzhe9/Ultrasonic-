/**
 ******************************************************************************
 * @file    paltform_watchdog.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide WDG driver functions.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include "common.h"
#include "debug.h"

#include "platform.h"
#include "platform_peripheral.h"
#include "stm32f1xx.h"

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
#ifndef MICO_DISABLE_WATCHDOG
//static __IO uint32_t LsiFreq = 0;
#define LsiFreq         40000 // LSI RC frequcey = 40 kHz
//static __IO uint32_t CaptureNumber = 0, PeriodValue = 0;
static IWDG_HandleTypeDef iwdg_handle;
#endif

/******************************************************
 *               Function Definitions
 ******************************************************/

OSStatus platform_watchdog_init( uint32_t timeout_ms )
{
// PLATFORM_TO_DO
#ifndef MICO_DISABLE_WATCHDOG
  OSStatus err = kNoErr;
  uint16_t reloadTick;
  /* Get the LSI frequency:  TIM5 is used to measure the LSI frequency */
  //LsiFreq = GetLSIFrequency();
  __HAL_RCC_LSI_ENABLE();
  /* Set counter reload value to obtain 250ms IWDG TimeOut.
     Counter Reload Value = timeout_ms /IWDG counter clock period
                          = timeout_ms * (LSI/256) / 1000
   */
  reloadTick = LsiFreq*timeout_ms/32000;
  require_action( reloadTick <= 0xFFF, exit, err = kParamErr );

  //IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
  /*IWDG_ENABLE_WRITE_ACCESS( &iwdg_handle );*/

  /* IWDG counter clock: LSI/256 */
  //IWDG_SetPrescaler(IWDG_Prescaler_256);

  //IWDG_SetReload(reloadTick);

  /* Reload IWDG counter */
  //IWDG_ReloadCounter();
  
  iwdg_handle.Instance = IWDG;
  iwdg_handle.Init.Prescaler = IWDG_PRESCALER_32;
  iwdg_handle.Init.Reload = reloadTick;
  require_action( HAL_IWDG_Init( &iwdg_handle ) == HAL_OK, exit, err = kGeneralErr );
  
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  //IWDG_Enable();
  require_action( HAL_IWDG_Start( &iwdg_handle ) == HAL_OK, exit, err = kGeneralErr );

exit:
  return err;
#else
  UNUSED_PARAMETER( timeout_ms );
  return kUnsupportedErr;
#endif
}

OSStatus MicoWdgFinalize( void )
{
    // PLATFORM_TO_DO
    return kNoErr;
}

OSStatus platform_watchdog_kick( void )
{
#ifndef MICO_DISABLE_WATCHDOG
  OSStatus err = kNoErr;
  require_action( HAL_IWDG_Refresh( &iwdg_handle ) == HAL_OK, exit, err = kGeneralErr );
  
exit:
  return err;
#else
  return kUnsupportedErr;
#endif
}
