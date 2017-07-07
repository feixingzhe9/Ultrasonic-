/**
 ******************************************************************************
 * @file    multi_channel_detection.c
 * @author  Adam Huang
 * @version V1.0.0
 * @date    29-Nov-2016
 * @brief   
 ******************************************************************************
*/

#include "multi_channel_detection.h"

#define   ADC1_DMA_CHANNELS_NUM  (16)
#define   FILTER_TIMES          (5)
#define   ADC1_DMA_BURRER_SIZE   (ADC1_DMA_CHANNELS_NUM*FILTER_TIMES)

#define   ADC3_DMA_CHANNELS_NUM  (5)
#define   FILTER_TIMES          (5)
#define   ADC3_DMA_BURRER_SIZE   (ADC3_DMA_CHANNELS_NUM*FILTER_TIMES)

extern const platform_adc_t             platform_adc_peripherals[];
__IO uint16_t adc1_dma_buffer[ADC1_DMA_BURRER_SIZE];
__IO uint16_t adc3_dma_buffer[ADC3_DMA_BURRER_SIZE];

void analog_multiplexer_init( void )
{
  platform_pin_config_t pin_config;
  
  pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
  pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
  pin_config.gpio_pull = GPIO_PULLUP;
  
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_SWITCH_EN, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2, &pin_config );
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3, &pin_config );

  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_EN );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
}


OSStatus select_multi_channel( mico_adc_t adc )
{
  OSStatus err = kNoErr;
  if ( adc >= MICO_ADC_MAX || adc <  MICO_ADC_SWITCH )
    return kUnsupportedErr;
  switch( adc )
  {
  case MICO_ADC_24V_TS:
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_12V_TS:
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_5V_TS:
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_AIR_TS:
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_24V_ALL:
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_12V_ALL:
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_5V_ALL:
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_VDET_24V:
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break; 
  case MICO_ADC_VDET_12V:
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_VDET_5V:
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_VDET_BAT:
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_AIUI:
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  case MICO_ADC_ROUTER_5V:
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
    break;
  default:
    break;
  } 
  return err;
}

#define ADC1_USED_CHANNELELS    (16)
#define ADC3_USED_CHANNELELS    (5)
#define SAMPLE_CYCLE            (235)

OSStatus multi_channel_adc_dma_init( void )
{
  OSStatus err = kNoErr;
  
  err = MicoAdcStreamInitializeEarly( MICO_ADC_5V_RES1, ADC1_USED_CHANNELELS );
  require_noerr_quiet( err, exit );
  
  err = MicoAdcStreamAddChannel( MICO_ADC_5V_RES1, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_12V_RES2, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_BAT_NV, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_12V_NV, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_ROUTER_12V, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_DYP, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_SENSOR, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_DLP, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_MOTOR, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_24V_RES1, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_2_1_PA, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_PAD, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_PRINTER, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_X86, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_IRLED, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_LEDS, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  
  select_multi_channel( MICO_ADC_24V_TS );
  err = MicoAdcStreamInitializeLate( MICO_ADC_LEDS, (void *)adc1_dma_buffer, ADC1_DMA_BURRER_SIZE );
  require_noerr_quiet( err, exit );
/*----------------------------------------------------------------------------*/  
  err = MicoAdcStreamInitializeEarly( MICO_ADC_CHARGE, ADC3_USED_CHANNELELS );
  require_noerr_quiet( err, exit );
  
  err = MicoAdcStreamAddChannel( MICO_ADC_CHARGE, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_BATIN, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_VBUS, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_BAT_MOTOR, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  err = MicoAdcStreamAddChannel( MICO_ADC_SWITCH, SAMPLE_CYCLE );
  require_noerr_quiet( err, exit );
  
  err = MicoAdcStreamInitializeLate( MICO_ADC_SWITCH, (void *)adc3_dma_buffer, ADC3_DMA_BURRER_SIZE );
  require_noerr_quiet( err, exit );
exit:
  return err;
}

uint16_t  read_channel_values_mV( mico_adc_t adc )
{
  uint32_t      temp_total = 0;
  
  if( adc >= MICO_ADC_MAX )
    return 0;
  if ( /*adc >= MICO_ADC_5V_RES1 && */adc <= MICO_ADC_LEDS )
  {
    for( uint8_t i = 0; i < FILTER_TIMES; i++ )
    {
      temp_total += (uint16_t)adc1_dma_buffer[platform_adc_peripherals[adc].rank - 1 + i * ADC1_DMA_CHANNELS_NUM ];
    }
    temp_total /= FILTER_TIMES;
    return (uint16_t)(temp_total/4096.0*3.3*1000);
  }
  else if ( adc > MICO_ADC_SWITCH && adc <=  MICO_ADC_ROUTER_5V )
  {    
    for( uint8_t i = 0; i < FILTER_TIMES; i++ )
    {
      temp_total += (uint16_t)adc3_dma_buffer[ADC3_USED_CHANNELELS - 1 + i * ADC3_DMA_CHANNELS_NUM ];
    }
    temp_total /= FILTER_TIMES;
    return (uint16_t)(temp_total/4096.0*3.3*1000);
  }
  else
  {
    for( uint8_t i = 0; i < FILTER_TIMES; i++ )
    {
      temp_total += (uint16_t)adc3_dma_buffer[platform_adc_peripherals[adc].rank - 1 + i * ADC3_DMA_CHANNELS_NUM ];
    }
    temp_total /= FILTER_TIMES;
    return (uint16_t)(temp_total/4096.0*3.3*1000);
  }
}

#define FIXIT_FACTOR    (5)
#define FIXIT_FACTOR_VOL    (1)
#define FIXIT_FACTOR_CUR     (50)
void read_multi_convert_values( void )
{  
  printf("\r\n");
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_5V_RES1 )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_12V_RES2 )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_BAT_NV )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_12V_NV )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_ROUTER_12V )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_DYP )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_SENSOR )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_DLP )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_MOTOR )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_24V_RES1 )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_2_1_PA )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_PAD )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_PRINTER )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_X86 )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_IRLED )* FIXIT_FACTOR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_LEDS )* FIXIT_FACTOR));
#if 1
  printf("follows are adc3 channels\r\n");
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_CHARGE )* FIXIT_FACTOR_CUR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_BATIN )* FIXIT_FACTOR_CUR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_VBUS )* FIXIT_FACTOR_CUR));
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_BAT_MOTOR )* FIXIT_FACTOR_CUR));
  select_multi_channel( MICO_ADC_24V_TS );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_24V_TS )* FIXIT_FACTOR_VOL));
  select_multi_channel( MICO_ADC_12V_TS );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_12V_TS )* FIXIT_FACTOR_VOL));
  select_multi_channel( MICO_ADC_5V_TS );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_5V_TS )* FIXIT_FACTOR_VOL));
  select_multi_channel( MICO_ADC_AIR_TS );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_AIR_TS )* FIXIT_FACTOR_VOL));
  select_multi_channel( MICO_ADC_24V_ALL );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_24V_ALL )* FIXIT_FACTOR));
  select_multi_channel( MICO_ADC_12V_ALL );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_12V_ALL )* FIXIT_FACTOR));
  select_multi_channel( MICO_ADC_5V_ALL );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_5V_ALL )* FIXIT_FACTOR));
  select_multi_channel( MICO_ADC_VDET_24V );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_VDET_24V )* 11));
  select_multi_channel( MICO_ADC_VDET_12V );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_VDET_12V )* 11));
  select_multi_channel( MICO_ADC_VDET_5V );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_VDET_5V )* 2));
  select_multi_channel( MICO_ADC_VDET_BAT );
  HAL_Delay(1);
  printf("%d\t",(uint16_t)(read_channel_values_mV( MICO_ADC_VDET_BAT )* 11));
#endif
}
