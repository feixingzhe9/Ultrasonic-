/**
 ******************************************************************************
 * @file    multi_channel_detection.h
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

#include "common.h"
#include "platform.h"
#include "mico_platform.h"
  
void analog_multiplexer_init( void );
OSStatus select_multi_channel( mico_adc_t adc );
OSStatus multi_channel_adc_dma_init( void );
void read_multi_convert_values( void );
uint16_t  read_channel_values_mV( mico_adc_t adc );

#ifdef __cplusplus
} /*extern "C" */
#endif