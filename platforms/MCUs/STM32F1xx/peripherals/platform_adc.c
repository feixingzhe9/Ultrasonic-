/**
 ******************************************************************************
 * @file    paltform_adc.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide ADC driver functions.
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
#include "stm32f1xx.h"
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
static const uint16_t adc_sampling_cycle[] =
{
    [ADC_SAMPLETIME_1CYCLE_5  ] = 1,
    [ADC_SAMPLETIME_7CYCLES_5 ] = 7,
    [ADC_SAMPLETIME_13CYCLES_5 ] = 13,
    [ADC_SAMPLETIME_28CYCLES_5 ] = 28,
    [ADC_SAMPLETIME_41CYCLES_5 ] = 41,
    [ADC_SAMPLETIME_55CYCLES_5] = 55,
    [ADC_SAMPLETIME_71CYCLES_5] = 71,
    [ADC_SAMPLETIME_239CYCLES_5] = 239
};

/******************************************************
 *               Function Declarations
 ******************************************************/


/******************************************************
 *               Function Definitions
 ******************************************************/


OSStatus platform_adc_init( const platform_adc_t* adc, uint32_t sample_cycle )
{
    ADC_ChannelConfTypeDef      channel_config_structure;  
    RCC_PeriphCLKInitTypeDef    PeriphClkInit;
    uint8_t     a;
    OSStatus    err = kNoErr;

    platform_mcu_powersave_disable();

    require_action_quiet( adc != NULL, exit, err = kParamErr);
    require_action_quiet( adc->port != NULL, exit, err = kParamErr );
    /* Initialize the associated GPIO */
    platform_pin_config_t pin_config;
    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode = GPIO_MODE_INPUT;
    pin_config.gpio_pull = GPIO_PULLUP;
    platform_gpio_init( adc->pin, &pin_config );

    if( adc->port == ADC1 )
    {
      __HAL_RCC_ADC1_CLK_ENABLE();
    }
    else if( adc->port == ADC2 )
    {
      __HAL_RCC_ADC2_CLK_ENABLE();
    }
    else if( adc->port == ADC3 )
    {
      __HAL_RCC_ADC3_CLK_ENABLE();
    }
    /* Configure ADCx clock prescaler */
    /* Caution: On STM32F1, ADC clock frequency max is 14MHz (refer to device   */
    /*          datasheet).                                                     */
    /*          Therefore, ADC clock prescaler must be configured in function   */
    /*          of ADC clock source frequency to remain below this maximum      */
    /*          frequency.                                                      */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
    
    /* Initialize the ADC */
    adc->handle->Instance                       = adc->port;
    adc->handle->Init.DataAlign                 = ADC_DATAALIGN_RIGHT;
    adc->handle->Init.ScanConvMode              = ADC_SCAN_DISABLE;
    adc->handle->Init.ContinuousConvMode        = DISABLE;
    adc->handle->Init.NbrOfConversion           = 1;
    adc->handle->Init.DiscontinuousConvMode     = DISABLE;
    adc->handle->Init.ExternalTrigConv          = ADC_SOFTWARE_START;
    adc->handle->Init.NbrOfDiscConversion       = 1;
    require_action_quiet( HAL_ADC_Init(adc->handle) == HAL_OK, exit, err = kCommandErr );

    /* Find the closest supported sampling time by the MCU */
    for ( a = 0; ( a < sizeof( adc_sampling_cycle ) / sizeof(uint16_t) ) && adc_sampling_cycle[a] < sample_cycle; a++ )
    {
    }

    /* Initialize the ADC channel */
    channel_config_structure.Channel            = adc->channel;
    channel_config_structure.Rank               = adc->rank;
    channel_config_structure.SamplingTime       = a;
    require_action_quiet( \
      HAL_ADC_ConfigChannel( adc->handle, &channel_config_structure) != HAL_OK,\
      exit, err = kCommandErr );
    
    /* Run the ADC calibration */  
    require_action_quiet( HAL_ADCEx_Calibration_Start(adc->handle) != HAL_OK,\
      exit, err = kCommandErr );

exit:
    platform_mcu_powersave_enable();
    return err;
}

OSStatus platform_adc_take_sample( const platform_adc_t* adc, uint16_t* output )
{
    OSStatus    err = kNoErr;

    platform_mcu_powersave_disable();

    require_action_quiet( adc != NULL, exit, err = kParamErr);

    /* Start conversion */
    HAL_ADC_Start( adc->handle );

    /* Wait until end of conversion */
    while ( __HAL_ADC_GET_FLAG( adc->handle, ADC_FLAG_EOC != SET ) )
    {
    }
    /* Read ADC conversion result */
    *output = HAL_ADC_GetValue( adc->handle );

exit:
    platform_mcu_powersave_enable();
    return err;
}

OSStatus platform_adc_stream_init_early( const platform_adc_t* adc, uint32_t channels_num )
{
    OSStatus    err = kNoErr;
    RCC_PeriphCLKInitTypeDef  PeriphClkInit;
    
    platform_mcu_powersave_disable();
    require_action_quiet( adc != NULL, exit, err = kParamErr );
    require_action_quiet( adc->port != NULL, exit, err = kParamErr );
    
    if( adc->port == ADC1 )
    {
      __HAL_RCC_ADC1_CLK_ENABLE();
    }
    else if( adc->port == ADC2 )
    {
      __HAL_RCC_ADC2_CLK_ENABLE();
    }
    else if( adc->port == ADC3 )
    {
      __HAL_RCC_ADC3_CLK_ENABLE();
    }
    /* Configure ADCx clock prescaler */
    /* Caution: On STM32F1, ADC clock frequency max is 14MHz (refer to device   */
    /*          datasheet).                                                     */
    /*          Therefore, ADC clock prescaler must be configured in function   */
    /*          of ADC clock source frequency to remain below this maximum      */
    /*          frequency.                                                      */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit );
  
    adc->handle->Instance                       = adc->port;
    adc->handle->Init.DataAlign                 = ADC_DATAALIGN_RIGHT;
    adc->handle->Init.ScanConvMode              = ADC_SCAN_ENABLE;
    adc->handle->Init.ContinuousConvMode        = ENABLE;
    adc->handle->Init.NbrOfConversion           = channels_num;
    adc->handle->Init.DiscontinuousConvMode     = DISABLE;
    adc->handle->Init.NbrOfDiscConversion       = 1;/* Parameter discarded because sequencer is disabled */
    adc->handle->Init.ExternalTrigConv          = ADC_SOFTWARE_START;
    require_action_quiet( HAL_ADC_Init(adc->handle) == HAL_OK, exit, err = kGeneralErr );

exit:
    platform_mcu_powersave_enable();
    return err;
}

OSStatus platform_add_to_adc_stream( const platform_adc_t* adc, uint32_t sample_cycle )
{
    ADC_ChannelConfTypeDef      channel_config_structure;   
    uint8_t     a;
    OSStatus    err = kNoErr;

    platform_mcu_powersave_disable();

    require_action_quiet( adc != NULL, exit, err = kParamErr );

    /* Initialize the associated GPIO */
    platform_pin_config_t pin_config;
//    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode = GPIO_MODE_ANALOG;
    pin_config.gpio_pull = GPIO_NOPULL;
    platform_gpio_init( adc->pin, &pin_config );
    
    /* Find the closest supported sampling time by the MCU */
    for ( a = 0; ( a < sizeof( adc_sampling_cycle ) / sizeof(uint16_t) ) && adc_sampling_cycle[a] < sample_cycle; a++ )
    {
    }

    /* Initialize the ADC channel */
    channel_config_structure.Channel            = adc->channel;
    channel_config_structure.Rank               = adc->rank;
    channel_config_structure.SamplingTime       = a;
    require_action_quiet( \
      HAL_ADC_ConfigChannel(adc->handle, &channel_config_structure) == HAL_OK,\
      exit, err = kGeneralErr );

exit:
    platform_mcu_powersave_enable();
    return err;  
}

OSStatus platform_adc_stream_init_late( const platform_adc_t* adc, void* buffer, uint16_t buffer_length )
{
    OSStatus    err = kNoErr;   
   
    DMA_HandleTypeDef*  pDmaHandle = NULL;
    platform_mcu_powersave_disable();
    require_action_quiet( adc != NULL, exit, err = kParamErr );
    
    if( adc->port == ADC1 )
    {
      static DMA_HandleTypeDef  DmaHandle1;
      pDmaHandle = &DmaHandle1;
      __HAL_RCC_DMA1_CLK_ENABLE();
      pDmaHandle->Instance = DMA1_Channel1;
    }
    else if( adc->port == ADC3 )
    {
      static DMA_HandleTypeDef  DmaHandle2;
      pDmaHandle = &DmaHandle2;
       __HAL_RCC_DMA2_CLK_ENABLE();
      pDmaHandle->Instance = DMA2_Channel5;
    }
    else
    {
      return kParamErr;
    }

    pDmaHandle->Init.Direction           = DMA_PERIPH_TO_MEMORY;
    pDmaHandle->Init.PeriphInc           = DMA_PINC_DISABLE;
    pDmaHandle->Init.MemInc              = DMA_MINC_ENABLE;
    pDmaHandle->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;   /* Transfer from ADC by half-word to match with ADC configuration: ADC resolution 10 or 12 bits */
    pDmaHandle->Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;   /* Transfer to memory by half-word to match with buffer variable type: half-word */
    pDmaHandle->Init.Mode                = DMA_CIRCULAR;              /* DMA in circular mode to match with ADC configuration: DMA continuous requests */
    pDmaHandle->Init.Priority            = DMA_PRIORITY_HIGH;
    
    /* Deinitialize  & Initialize the DMA for new transfer */
    HAL_DMA_DeInit( pDmaHandle ); 
    HAL_DMA_Init( pDmaHandle );
    
    /* Associate the initialized DMA handle to the ADC handle */
    __HAL_LINKDMA( adc->handle, DMA_Handle, *pDmaHandle );
    require_action_quiet( HAL_ADCEx_Calibration_Start( adc->handle ) == HAL_OK,\
      exit, err = kGeneralErr );
    require_action_quiet( HAL_ADC_Start_DMA(adc->handle,(uint32_t *)buffer, buffer_length) == HAL_OK,\
      exit, err = kGeneralErr );
exit:
    platform_mcu_powersave_enable();
    return err;  
}

OSStatus platform_adc_take_sample_stream( const platform_adc_t* adc, void* buffer, uint16_t buffer_length )
{
    UNUSED_PARAMETER(adc);
    UNUSED_PARAMETER(buffer);
    UNUSED_PARAMETER(buffer_length);
    platform_log("unimplemented");
    return kNotPreparedErr;
}

OSStatus platform_adc_deinit( const platform_adc_t* adc )
{
    UNUSED_PARAMETER(adc);
    platform_log("unimplemented");
    return kNotPreparedErr;
}

void platform_adc_dma_irq( const platform_adc_t* adc )
{
  HAL_DMA_IRQHandler( adc->handle->DMA_Handle );
}
