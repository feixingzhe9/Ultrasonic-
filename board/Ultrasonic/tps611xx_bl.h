/**
 ******************************************************************************
 * @file    tps611xx_bl.h
 * @author  Adam Huang
 * @version V1.0.0
 * @date    29-Nov-2016
 * @brief   
 ******************************************************************************
*/

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif
  
#include "platform.h"
#include "mico_platform.h"
  
#define PWM                             1
#define EASY_SCALE                      2
#define TPS_CONTROL_MODE                PWM //EASY_SCALE //
  
void startTps611xx( void );
void brightness_dimming( uint32_t frequency, uint8_t duty );
void brightness_dimming_by_duty( uint8_t duty );
void stopTps611xx( void );
  
#ifdef __cplusplus
} /*extern "C" */
#endif