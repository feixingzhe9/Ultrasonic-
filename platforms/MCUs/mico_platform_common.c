/**
 ******************************************************************************
 * @file    mico_platform_common.c
 * @author  William Xu
 * @version V1.0.0
 * @date    20-March-2015
 * @brief   This file provide MICO driver functions.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include <stdint.h>
#include <string.h>
#include "common.h"
#include "platform_peripheral.h"
#include "mico_platform.h"
#include "platform_config.h"
#include "platform_logging.h"

#ifndef BOOTLOADER

#ifdef USE_MiCOKit_EXT
#include "MiCOKit_EXT/micokit_ext.h"   // extension sensor board.
#endif

#ifdef USE_MiCOKit_STMEMS
#include "MiCOKit_STmems/MiCOKit_STmems.h"   //// extension sensor board.
#endif

#endif

/******************************************************
*                      Macros
******************************************************/

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
*               Static Function Declarations
******************************************************/

extern OSStatus mico_platform_init      ( void );

/******************************************************
*               Variable Definitions
******************************************************/

/* Externed from platforms/<Platform>/platform.c */
extern const platform_gpio_t            platform_gpio_pins[];
extern const platform_adc_t             platform_adc_peripherals[];
extern const platform_i2c_t             platform_i2c_peripherals[];
extern platform_i2c_driver_t            platform_i2c_drivers[];
extern const platform_pwm_t             platform_pwm_peripherals[];
extern const platform_spi_t             platform_spi_peripherals[];
extern platform_spi_driver_t            platform_spi_drivers[];
extern const platform_uart_t            platform_uart_peripherals[];
extern platform_uart_driver_t           platform_uart_drivers[];
extern WEAK platform_spi_slave_driver_t platform_spi_slave_drivers[];
extern const platform_flash_t           platform_flash_peripherals[];
extern platform_flash_driver_t          platform_flash_drivers[];
extern const mico_logic_partition_t     mico_partitions[];
extern platform_can_driver_t            platform_can_drivers[];

#ifdef MICO_WIFI_SHARE_SPI_BUS
extern const platform_spi_t wifi_spi;
#endif

/******************************************************
*               Function Definitions
******************************************************/

OSStatus mico_platform_init( void )
{
#if defined(__CC_ARM)
  platform_log("Platform initialised, build by RVMDK");
#elif defined (__IAR_SYSTEMS_ICC__)
  platform_log("Platform initialised, build by IAR");
#endif
  
  if ( true == platform_watchdog_check_last_reset() )
  {
    platform_log( "WARNING: Watchdog reset occured previously. Please see platform_watchdog.c for debugging instructions." );
  }

#ifdef USES_RESOURCE_FILESYSTEM
  platform_filesystem_init();
#endif
 
#ifndef BOOTLOADER
  
#ifdef USE_MiCOKit_EXT
  micokit_ext_init();
#endif
  
#ifdef USE_MiCOKit_STMEMS
  micokit_STmems_init();
#endif
  
#endif
  
  return kNoErr;
}

OSStatus MicoAdcInitialize( mico_adc_t adc, uint32_t sampling_cycle )
{
  if ( adc >= MICO_ADC_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_adc_init( &platform_adc_peripherals[adc], sampling_cycle );
}

OSStatus  MicoAdcFinalize( mico_adc_t adc )
{
  if ( adc >= MICO_ADC_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_adc_deinit( &platform_adc_peripherals[adc] );
}

OSStatus MicoAdcTakeSample( mico_adc_t adc, uint16_t* output )
{
  if ( adc >= MICO_ADC_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_adc_take_sample( &platform_adc_peripherals[adc], output );
}

OSStatus MicoAdcStreamInitializeEarly( mico_adc_t adc, uint8_t channels )
{
  if ( adc >= MICO_ADC_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_adc_stream_init_early( &platform_adc_peripherals[adc], channels );
}

OSStatus MicoAdcStreamAddChannel( mico_adc_t adc, uint32_t sample_cycle )
{
  if ( adc >= MICO_ADC_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_add_to_adc_stream( &platform_adc_peripherals[adc], sample_cycle );
}

OSStatus MicoAdcStreamInitializeLate( mico_adc_t adc, void* buffer, uint16_t buffer_length )
{
  if ( adc >= MICO_ADC_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_adc_stream_init_late( &platform_adc_peripherals[adc], buffer, buffer_length );
}

OSStatus MicoAdcTakeSampleStreram( mico_adc_t adc, void* buffer, uint16_t buffer_length )
{
  if ( adc >= MICO_ADC_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_adc_take_sample_stream( &platform_adc_peripherals[adc], buffer, buffer_length );
}

OSStatus MicoGpioInitialize( mico_gpio_t gpio, mico_gpio_config_t configuration )
{
  if ( gpio >= MICO_GPIO_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_gpio_init( &platform_gpio_pins[gpio], configuration );
}

OSStatus MicoGpioOutputHigh( mico_gpio_t gpio )
{
  if ( gpio >= MICO_GPIO_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_gpio_output_high( &platform_gpio_pins[gpio] );
}

OSStatus MicoGpioOutputLow( mico_gpio_t gpio )
{
  if ( gpio >= MICO_GPIO_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_gpio_output_low( &platform_gpio_pins[gpio] );
}

OSStatus MicoGpioOutputTrigger( mico_gpio_t gpio )
{
  if ( gpio >= MICO_GPIO_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_gpio_output_trigger( &platform_gpio_pins[gpio] );
}

OSStatus MicoGpioFinalize( mico_gpio_t gpio )
{
  if ( gpio >= MICO_GPIO_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_gpio_deinit( &platform_gpio_pins[gpio] );
}

bool MicoGpioInputGet( mico_gpio_t gpio )
{
  if ( gpio >= MICO_GPIO_NONE )
    return kUnsupportedErr;
  return platform_gpio_input_get( &platform_gpio_pins[gpio] );
}

OSStatus MicoGpioEnableIRQ( mico_gpio_t gpio, mico_gpio_irq_trigger_t trigger, mico_gpio_irq_handler_t handler, void* arg )
{
  if ( gpio >= MICO_GPIO_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_gpio_irq_enable( &platform_gpio_pins[gpio], trigger, handler, arg );
}

OSStatus MicoGpioDisableIRQ( mico_gpio_t gpio )
{
  if ( gpio >= MICO_GPIO_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_gpio_irq_disable( &platform_gpio_pins[gpio] );
}

OSStatus MicoI2cInitialize( mico_i2c_device_t* device )
{
  platform_i2c_config_t config;
  OSStatus result;

  if ( device->port >= MICO_I2C_NONE )
    return kUnsupportedErr;
 
  config.address       = device->address;
  config.address_width = device->address_width;
  config.flags         &= ~I2C_DEVICE_USE_DMA ;
  config.speed_mode    = device->speed_mode;

  if( platform_i2c_drivers[device->port].i2c_mutex == NULL)
    mico_rtos_init_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
  
  mico_rtos_lock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
  result = (OSStatus) platform_i2c_init( &platform_i2c_peripherals[device->port], &config );
  mico_rtos_unlock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );

  return result;
}

OSStatus MicoI2cFinalize( mico_i2c_device_t* device )
{
  platform_i2c_config_t config;

  if ( device->port >= MICO_I2C_NONE )
    return kUnsupportedErr;
  
  config.address       = device->address;
  config.address_width = device->address_width;
  config.flags         &= ~I2C_DEVICE_USE_DMA ;
  config.speed_mode    = device->speed_mode;

  if( platform_i2c_drivers[device->port].i2c_mutex != NULL){
    mico_rtos_deinit_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
    platform_i2c_drivers[device->port].i2c_mutex = NULL;
  }
    
  return (OSStatus) platform_i2c_deinit( &platform_i2c_peripherals[device->port], &config );
}

bool MicoI2cProbeDevice( mico_i2c_device_t* device, int retries )
{
  bool ret;
  platform_i2c_config_t config;

  if ( device->port >= MICO_I2C_NONE )
    return kUnsupportedErr;
  
  config.address       = device->address;
  config.address_width = device->address_width;
  config.flags         &= ~I2C_DEVICE_USE_DMA ;
  config.speed_mode    = device->speed_mode;

  mico_rtos_lock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
  ret = platform_i2c_probe_device( &platform_i2c_peripherals[device->port], &config, retries );
  mico_rtos_unlock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );

  return ret;
}

OSStatus MicoI2cBuildTxMessage( mico_i2c_message_t* message, const void* tx_buffer, uint16_t  tx_buffer_length, uint16_t retries )
{
  return (OSStatus) platform_i2c_init_tx_message( message, tx_buffer, tx_buffer_length, retries );
}

OSStatus MicoI2cBuildRxMessage( mico_i2c_message_t* message, void* rx_buffer, uint16_t rx_buffer_length, uint16_t retries )
{
  return (OSStatus) platform_i2c_init_rx_message( message, rx_buffer, rx_buffer_length, retries );
}

OSStatus MicoI2cBuildCombinedMessage( mico_i2c_message_t* message, const void* tx_buffer, void* rx_buffer, uint16_t tx_buffer_length, uint16_t rx_buffer_length, uint16_t retries )
{
  return (OSStatus) platform_i2c_init_combined_message( message, tx_buffer, rx_buffer, tx_buffer_length, rx_buffer_length, retries );
}

OSStatus MicoI2cTransfer( mico_i2c_device_t* device, mico_i2c_message_t* messages, uint16_t number_of_messages )
{
  OSStatus err = kNoErr;
  platform_i2c_config_t config;
  
  if ( device->port >= MICO_I2C_NONE )
    return kUnsupportedErr;

  config.address       = device->address;
  config.address_width = device->address_width;
  config.flags         &= ~I2C_DEVICE_USE_DMA ;
  config.speed_mode    = device->speed_mode;
  
  mico_rtos_lock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );
  err = platform_i2c_transfer( &platform_i2c_peripherals[device->port], &config, messages, number_of_messages );
  mico_rtos_unlock_mutex( &platform_i2c_drivers[device->port].i2c_mutex );

  return err;
}

void MicoMcuPowerSaveConfig( int enable )
{
  if (enable == 1)
    platform_mcu_powersave_enable( );
  else
    platform_mcu_powersave_disable( );
}

void MicoSystemStandBy( uint32_t secondsToWakeup )
{
  platform_mcu_enter_standby( secondsToWakeup );
}


OSStatus MicoPwmInitialize(mico_pwm_t pwm, uint32_t frequency, float duty_cycle)
{
  if ( pwm >= MICO_PWM_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_pwm_init( &platform_pwm_peripherals[pwm], frequency, duty_cycle );
}

OSStatus MicoPwmStart( mico_pwm_t pwm )
{
  if ( pwm >= MICO_PWM_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_pwm_start( &platform_pwm_peripherals[pwm] );
}

OSStatus MicoPwmSetDuty( mico_pwm_t pwm, float duty_cycle )
{
  if ( pwm >= MICO_PWM_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_set_pwm_duty( &platform_pwm_peripherals[pwm], duty_cycle );
}

OSStatus MicoPwmStop( mico_pwm_t pwm )
{
  if ( pwm >= MICO_PWM_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_pwm_stop( &platform_pwm_peripherals[pwm] );
}

OSStatus MicoRtcGetTime(mico_rtc_time_t* time)
{
  return (OSStatus) platform_rtc_get_time( time );
}

OSStatus MicoRtcSetTime(mico_rtc_time_t* time)
{
  return (OSStatus) platform_rtc_set_time( time );
}

OSStatus MicoSpiInitialize( const mico_spi_device_t* spi )
{
  platform_spi_config_t config;
  OSStatus              err = kNoErr;

  if ( spi->port >= MICO_SPI_NONE )
    return kUnsupportedErr;

#ifdef MICO_WIFI_SHARE_SPI_BUS
  if( platform_spi_peripherals[spi->port].port == wifi_spi.port )
  {
    return platform_wlan_spi_init( &platform_gpio_pins[spi->chip_select] );
  }
#endif

  if( platform_spi_drivers[spi->port].spi_mutex == NULL)
    mico_rtos_init_mutex( &platform_spi_drivers[spi->port].spi_mutex );
  
  config.chip_select = &platform_gpio_pins[spi->chip_select];
  config.speed       = spi->speed;
  config.mode        = spi->mode;
  config.bits        = spi->bits;

  mico_rtos_lock_mutex( &platform_spi_drivers[spi->port].spi_mutex );
  err = platform_spi_init( &platform_spi_drivers[spi->port], &platform_spi_peripherals[spi->port], &config );
  mico_rtos_unlock_mutex( &platform_spi_drivers[spi->port].spi_mutex );

  return err;
}

OSStatus MicoSpiFinalize( const mico_spi_device_t* spi )
{
  OSStatus err = kNoErr;

  if ( spi->port >= MICO_SPI_NONE )
    return kUnsupportedErr;

#ifdef MICO_WIFI_SHARE_SPI_BUS
  if( platform_spi_peripherals[spi->port].port == wifi_spi.port )
  {
    return platform_wlan_spi_deinit( &platform_gpio_pins[spi->chip_select] );
  }
#endif

  if( platform_spi_drivers[spi->port].spi_mutex == NULL)
    mico_rtos_init_mutex( &platform_spi_drivers[spi->port].spi_mutex );
  
  mico_rtos_lock_mutex( &platform_spi_drivers[spi->port].spi_mutex );
  err = platform_spi_deinit( &platform_spi_drivers[spi->port] );
  mico_rtos_unlock_mutex( &platform_spi_drivers[spi->port].spi_mutex );

  return err;
}

OSStatus MicoSpiTransfer( const mico_spi_device_t* spi, const mico_spi_message_segment_t* segments, uint16_t number_of_segments )
{
  platform_spi_config_t config;
  OSStatus err = kNoErr;

  if ( spi->port >= MICO_SPI_NONE )
    return kUnsupportedErr;

#ifdef MICO_WIFI_SHARE_SPI_BUS
  if( platform_spi_peripherals[spi->port].port == wifi_spi.port )
  {
    return platform_wlan_spi_transfer( &platform_gpio_pins[spi->chip_select], segments, number_of_segments );
  }
#endif

  if( platform_spi_drivers[spi->port].spi_mutex == NULL)
    mico_rtos_init_mutex( &platform_spi_drivers[spi->port].spi_mutex );

  config.chip_select = &platform_gpio_pins[spi->chip_select];
  config.speed       = spi->speed;
  config.mode        = spi->mode;
  config.bits        = spi->bits;

  mico_rtos_lock_mutex( &platform_spi_drivers[spi->port].spi_mutex );
  err = platform_spi_init( &platform_spi_drivers[spi->port], &platform_spi_peripherals[spi->port], &config );
  err = platform_spi_transfer( &platform_spi_drivers[spi->port], &config, segments, number_of_segments );
  mico_rtos_unlock_mutex( &platform_spi_drivers[spi->port].spi_mutex );

  return err;
}

OSStatus MicoSpiSlaveInitialize( mico_spi_t spi, const mico_spi_slave_config_t* config )
{
  if ( spi >= MICO_SPI_NONE )
    return kUnsupportedErr;

  return (OSStatus) platform_spi_slave_init( &platform_spi_slave_drivers[spi], &platform_spi_peripherals[spi], config );
}

OSStatus MicoSpiSlaveFinalize( mico_spi_t spi )
{
  if ( spi >= MICO_SPI_NONE )
    return kUnsupportedErr;

  return (OSStatus) platform_spi_slave_deinit( &platform_spi_slave_drivers[spi] );
}

OSStatus  MicoSpiSlaveSendErrorStatus( mico_spi_t spi, mico_spi_slave_transfer_status_t error_status )
{
  if ( spi >= MICO_SPI_NONE )
    return kUnsupportedErr;

  return (OSStatus) platform_spi_slave_send_error_status( &platform_spi_slave_drivers[spi], error_status );
}

OSStatus MicoSpiSlaveReceiveCommand( mico_spi_t spi, mico_spi_slave_command_t* command, uint32_t timeout_ms )
{
  if ( spi >= MICO_SPI_NONE )
    return kUnsupportedErr;  

  return (OSStatus) platform_spi_slave_receive_command( &platform_spi_slave_drivers[spi], command, timeout_ms );
}

OSStatus MicoSpiSlaveTransferData( mico_spi_t spi, mico_spi_slave_transfer_direction_t direction, mico_spi_slave_data_buffer_t* buffer, uint32_t timeout_ms )
{
  if ( spi >= MICO_SPI_NONE )
    return kUnsupportedErr;  

  return (OSStatus) platform_spi_slave_transfer_data( &platform_spi_slave_drivers[spi], direction, buffer, timeout_ms );
}

OSStatus MicoSpiSlaveGenerateInterrupt( mico_spi_t spi, uint32_t pulse_duration_ms )
{
  if ( spi >= MICO_SPI_NONE )
    return kUnsupportedErr;

  return (OSStatus) platform_spi_slave_generate_interrupt( &platform_spi_slave_drivers[spi], pulse_duration_ms );
}

OSStatus MicoUartInitialize( mico_uart_t uart, const mico_uart_config_t* config, ring_buffer_t* optional_rx_buffer )
{
  if ( uart >= MICO_UART_NONE )
    return kUnsupportedErr;

#ifndef MICO_DISABLE_STDIO
  /* Interface is used by STDIO. Uncomment MICO_DISABLE_STDIO to overcome this */
  if ( uart == STDIO_UART )
  {
    return kGeneralErr;
  }
#endif
  
  return (OSStatus) platform_uart_init( &platform_uart_drivers[uart], &platform_uart_peripherals[uart], config, optional_rx_buffer );
}

OSStatus MicoUartFinalize( mico_uart_t uart )
{
  if ( uart >= MICO_UART_NONE )
    return kUnsupportedErr;

  return (OSStatus) platform_uart_deinit( &platform_uart_drivers[uart] );
}

OSStatus MicoUartSend( mico_uart_t uart, const void* data, uint32_t size )
{
  if ( uart >= MICO_UART_NONE )
    return kUnsupportedErr;

  return (OSStatus) platform_uart_transmit_bytes( &platform_uart_drivers[uart], (const uint8_t*) data, size );
}

OSStatus MicoUartRecv( mico_uart_t uart, void* data, uint32_t size, uint32_t timeout )
{
  if ( uart >= MICO_UART_NONE )
    return kUnsupportedErr;

  return (OSStatus) platform_uart_receive_bytes( &platform_uart_drivers[uart], (uint8_t*)data, size, timeout );
}

uint32_t MicoUartGetLengthInBuffer( mico_uart_t uart )
{
  if ( uart >= MICO_UART_NONE )
    return 0;
  
  return (OSStatus) platform_uart_get_length_in_buffer( &platform_uart_drivers[uart] );
}

OSStatus MicoRandomNumberRead( void *inBuffer, int inByteCount )
{
  return (OSStatus) platform_random_number_read( inBuffer, inByteCount );
}

void MicoSystemReboot( void )
{
  platform_mcu_reset();
}

OSStatus MicoWdgInitialize( uint32_t timeout )
{
    return (OSStatus) platform_watchdog_init( timeout );
}

void MicoWdgReload( void )
{
    platform_watchdog_kick( );
}

mico_logic_partition_t* MicoFlashGetInfo( mico_partition_t inPartition )
{
  if( (inPartition < 0) || (inPartition >= MICO_PARTITION_MAX))
    return NULL;
  else
    return (mico_logic_partition_t *)&mico_partitions[ inPartition ];
}


OSStatus MicoFlashInitialize( mico_partition_t partition )
{
  OSStatus err = kNoErr;
  
  require_action_quiet( partition > MICO_PARTITION_ERROR, exit, err = kParamErr );
  require_action_quiet( partition < MICO_PARTITION_MAX, exit, err = kParamErr );
  require_action_quiet( mico_partitions[ partition ].partition_owner != MICO_FLASH_NONE, exit, err = kNotFoundErr );
  
  if( platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex == NULL){
    err = mico_rtos_init_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
    require_noerr( err, exit );
  }
  
  mico_rtos_lock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
  
  err = platform_flash_init( &platform_flash_peripherals[ mico_partitions[ partition ].partition_owner ] );
  platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].peripheral = (platform_flash_t *)&platform_flash_peripherals[ mico_partitions[ partition ].partition_owner ];
  platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].initialized = true;
  mico_rtos_unlock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
  
exit: 
  return err;
}

OSStatus MicoFlashErase(mico_partition_t partition, uint32_t off_set, uint32_t size)
{
  OSStatus err = kNoErr;
  uint32_t start_addr = mico_partitions[ partition ].partition_start_addr + off_set;
  uint32_t end_addr = mico_partitions[ partition ].partition_start_addr + off_set + size - 1;

  if (size == 0)
    goto exit;
  require_action_quiet( partition > MICO_PARTITION_ERROR, exit, err = kParamErr );
  require_action_quiet( partition < MICO_PARTITION_MAX, exit, err = kParamErr );  

  require_action_quiet( mico_partitions[ partition ].partition_owner != MICO_FLASH_NONE, exit, err = kNotFoundErr );
#ifndef BOOTLOADER
  require_action_quiet( ( mico_partitions[ partition ].partition_options & PAR_OPT_WRITE_MASK ) == PAR_OPT_WRITE_EN, exit, err = kPermissionErr );
#endif

  require_action_quiet( start_addr >= mico_partitions[ partition ].partition_start_addr, exit, err = kParamErr );
  require_action_quiet( end_addr < mico_partitions[ partition ].partition_start_addr + mico_partitions[ partition ].partition_length, exit, err = kParamErr );

  if( platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].initialized == false )
  {
    err =  MicoFlashInitialize( partition );
    require_noerr_quiet( err, exit );
  }

  mico_rtos_lock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
  err = platform_flash_erase( &platform_flash_peripherals[ mico_partitions[ partition ].partition_owner ], start_addr, end_addr );
  mico_rtos_unlock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );

exit:
  return err;
}

OSStatus MicoFlashWrite( mico_partition_t partition, volatile uint32_t* off_set, uint8_t* inBuffer ,uint32_t inBufferLength)
{
  OSStatus err = kNoErr;

  uint32_t start_addr = mico_partitions[ partition ].partition_start_addr + *off_set;
  uint32_t end_addr = mico_partitions[ partition ].partition_start_addr + *off_set + inBufferLength - 1;

  if (inBufferLength == 0)
    goto exit;
  require_action_quiet( partition > MICO_PARTITION_ERROR, exit, err = kParamErr );
  require_action_quiet( partition < MICO_PARTITION_MAX, exit, err = kParamErr );
  
  require_action_quiet( mico_partitions[ partition ].partition_owner != MICO_FLASH_NONE, exit, err = kNotFoundErr );
#ifndef BOOTLOADER
  require_action_quiet( ( mico_partitions[ partition ].partition_options & PAR_OPT_WRITE_MASK ) == PAR_OPT_WRITE_EN, exit, err = kPermissionErr );
#endif

  require_action_quiet( start_addr >= mico_partitions[ partition ].partition_start_addr, exit, err = kParamErr );
  require_action_quiet( end_addr < mico_partitions[ partition ].partition_start_addr + mico_partitions[ partition ].partition_length , exit, err = kParamErr );
  
  if( platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].initialized == false )
  {
    err =  MicoFlashInitialize( partition );
    require_noerr_quiet( err, exit );
  }

  mico_rtos_lock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
  err = platform_flash_write( &platform_flash_peripherals[ mico_partitions[ partition ].partition_owner ], &start_addr, inBuffer, inBufferLength );
  *off_set = start_addr - mico_partitions[ partition ].partition_start_addr;
  mico_rtos_unlock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
  
exit:
  return err;
}

OSStatus MicoFlashRead( mico_partition_t partition, volatile uint32_t* off_set, uint8_t* outBuffer ,uint32_t inBufferLength)
{
  OSStatus err = kNoErr;
  uint32_t start_addr = mico_partitions[ partition ].partition_start_addr + *off_set;
  uint32_t  end_addr = mico_partitions[ partition ].partition_start_addr + *off_set + inBufferLength - 1;

  if (inBufferLength == 0)
    goto exit;
  require_action_quiet( partition > MICO_PARTITION_ERROR, exit, err = kParamErr );
  require_action_quiet( partition < MICO_PARTITION_MAX, exit, err = kParamErr );
  
  require_action_quiet( mico_partitions[ partition ].partition_owner != MICO_FLASH_NONE, exit, err = kNotFoundErr );
#ifndef BOOTLOADER
  require_action_quiet( ( mico_partitions[ partition ].partition_options & PAR_OPT_READ_MASK ) == PAR_OPT_READ_EN, exit, err = kPermissionErr );
#endif

  require_action_quiet( start_addr >= mico_partitions[ partition ].partition_start_addr, exit, err = kParamErr );
  require_action_quiet( end_addr < mico_partitions[ partition ].partition_start_addr + mico_partitions[ partition ].partition_length , exit, err = kParamErr );

  if( platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].initialized == false )
  {
    err =  MicoFlashInitialize( partition );
    require_noerr_quiet( err, exit );
  }

  mico_rtos_lock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
  err = platform_flash_read( &platform_flash_peripherals[ mico_partitions[ partition ].partition_owner ], &start_addr, outBuffer, inBufferLength );
  *off_set = start_addr - mico_partitions[ partition ].partition_start_addr;
  mico_rtos_unlock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );

exit:
  return err;
}

OSStatus MicoFlashEnableSecurity( mico_partition_t partition, uint32_t off_set, uint32_t size )
{
  OSStatus err = kNoErr;
  uint32_t start_addr = mico_partitions[ partition ].partition_start_addr + off_set;
  uint32_t end_addr = mico_partitions[ partition ].partition_start_addr + off_set + size - 1;

  if (size == 0)
    goto exit;
  require_action_quiet( partition > MICO_PARTITION_ERROR, exit, err = kParamErr );
  require_action_quiet( partition < MICO_PARTITION_MAX, exit, err = kParamErr );

  require_action_quiet( mico_partitions[ partition ].partition_owner != MICO_FLASH_NONE, exit, err = kNotFoundErr );
  require_action_quiet( start_addr >= mico_partitions[ partition ].partition_start_addr, exit, err = kParamErr );
  require_action_quiet( end_addr < mico_partitions[ partition ].partition_start_addr + mico_partitions[ partition ].partition_length, exit, err = kParamErr );

  if( platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].initialized == false )
  {
    err =  MicoFlashInitialize( partition );
    require_noerr_quiet( err, exit );
  }

  mico_rtos_lock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
  err = platform_flash_enable_protect( &platform_flash_peripherals[ mico_partitions[ partition ].partition_owner ], start_addr, end_addr);
  mico_rtos_unlock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
  
exit:
  return err;
}

OSStatus MicoCanInitialize( mico_can_t can )
{
  if ( can >= MICO_CAN_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_can_init( &platform_can_drivers[ can ] );
}

#if 0
OSStatus MicoCanMessageSend( mico_can_t can, const void *msg, uint8_t len )
{
  if ( can >= MICO_CAN_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_can_send_message( &platform_can_drivers[ can ], (uint8_t *)msg, len );
}
#else
OSStatus MicoCanMessageSend( mico_can_t can, const CanTxMsgTypeDef *msg)
{
  if ( can >= MICO_CAN_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_can_send_message( &platform_can_drivers[ can ], msg);
}
#endif
#if 0
OSStatus MicoCanMessageRead( mico_can_t can, const void *msg )
{
  if ( can >= MICO_CAN_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_can_receive_message( &platform_can_drivers[ can ], (uint8_t *)msg );
}
#else
OSStatus MicoCanMessageRead( mico_can_t can, const CanRxMsgTypeDef *msg )
{
  if ( can >= MICO_CAN_NONE )
    return kUnsupportedErr;
  return (OSStatus) platform_can_receive_message( &platform_can_drivers[ can ], (CanRxMsgTypeDef *)msg );
}
#endif
void MicoNanosendDelay( uint64_t delayns )
{
  platform_nanosecond_delay( delayns );
}

char *mico_get_bootloader_ver(void)
{
    static char ver[33];
    const mico_logic_partition_t* bootloader_partition = &mico_partitions[ MICO_PARTITION_BOOTLOADER ];
    uint32_t version_offset = bootloader_partition->partition_length - 0x20;

    memset(ver, 0, sizeof(ver));
    MicoFlashRead( MICO_PARTITION_BOOTLOADER, &version_offset, (uint8_t *)ver , 32);
    return ver;
}

#ifdef BOOTLOADER 
#include "bootloader.h"

void mico_set_bootload_ver(void)
{
   char ver[33];
   mico_logic_partition_t *boot_partition = MicoFlashGetInfo( MICO_PARTITION_BOOTLOADER );
   uint32_t flashaddr =  boot_partition->partition_length - 0x20;
   int i;

   memset(ver, 0, sizeof(ver));
   MicoFlashRead( MICO_PARTITION_BOOTLOADER, &flashaddr, (uint8_t *)ver , 32);
   for(i=0;i<32;i++) {
       if (ver[i] != 0xFF)
           return;
   }
   snprintf(ver, 33, "%s %s %d", MODEL, Bootloader_REVISION , STDIO_UART_BAUDRATE);
   flashaddr =  boot_partition->partition_length - 0x20;
   MicoFlashDisableSecurity( MICO_PARTITION_BOOTLOADER, 0x0, boot_partition->partition_length );
   MicoFlashWrite( MICO_PARTITION_BOOTLOADER, &flashaddr, (uint8_t *)ver , 32);
}

OSStatus MicoFlashDisableSecurity( mico_partition_t partition, uint32_t off_set, uint32_t size )
{
  OSStatus err = kNoErr;
  uint32_t start_addr = mico_partitions[ partition ].partition_start_addr + off_set;
  uint32_t end_addr = mico_partitions[ partition ].partition_start_addr + off_set + size - 1;

  require_action_quiet( partition > MICO_PARTITION_ERROR, exit, err = kParamErr );
  require_action_quiet( partition < MICO_PARTITION_MAX, exit, err = kParamErr );
  require_action_quiet( mico_partitions[ partition ].partition_owner != MICO_FLASH_NONE, exit, err = kNotFoundErr );
  require_action_quiet( start_addr >= mico_partitions[ partition ].partition_start_addr, exit, err = kParamErr );
  require_action_quiet( end_addr < mico_partitions[ partition ].partition_start_addr + mico_partitions[ partition ].partition_length, exit, err = kParamErr );

  if( platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].initialized == false )
  {
    err =  MicoFlashInitialize( partition );
    require_noerr_quiet( err, exit );
  }

  mico_rtos_lock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
  err = platform_flash_disable_protect( &platform_flash_peripherals[ mico_partitions[ partition ].partition_owner ], start_addr, end_addr);
  mico_rtos_unlock_mutex( &platform_flash_drivers[ mico_partitions[ partition ].partition_owner ].flash_mutex );
  
exit:
  return err;
}
#else
OSStatus MicoFlashDisableSecurity( mico_partition_t partition, uint32_t off_set, uint32_t size )
{
  OSStatus err = kNoErr;//kUnsupportedErr;
  return err;
}
#endif

