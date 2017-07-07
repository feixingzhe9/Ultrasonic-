/**
 ******************************************************************************
 * @file    paltform_uart.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   This file provide UART driver functions.
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
#include "debug.h"
#include "mico_rtos.h"
#include "mico_platform.h"
/******************************************************
*                    Constants
******************************************************/
#ifndef BOOTLOADER
#define UART_NOT_USE_DMA
#endif
//#define DMA_INTERRUPT_FLAGS  ( DMA_IT_TC | DMA_IT_TE | DMA_IT_DME | DMA_IT_FE )
#define DMA_INTERRUPT_FLAGS  ( DMA_IT_TC | DMA_IT_TE ) // | DMA_IT_HT

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
/* UART alternate functions */
/* UART peripheral clock functions */
//#define uart_peripheral_clock_functions(uart_ports) __HAL_RCC_USART##uart_ports##_CLK_ENABLE
//static UART_HandleTypeDef ComUartHandle;
static DMA_HandleTypeDef *hdma_tx[3] = NULL;
static DMA_HandleTypeDef *hdma_rx[3] = NULL;
/* UART interrupt vectors */
static const IRQn_Type uart_irq_vectors[NUMBER_OF_UART_PORTS] =
{
    [0] = USART1_IRQn,
    [1] = USART2_IRQn,
    [2] = USART3_IRQn,
};
/******************************************************
*               Function Definitions
******************************************************/
static OSStatus receive_bytes( platform_uart_driver_t* driver, void* data, uint32_t size, uint32_t timeout );
static void clear_dma_interrupts( DMA_TypeDef* controller, uint32_t flags );

OSStatus platform_uart_init( platform_uart_driver_t* driver, const platform_uart_t* peripheral, const platform_uart_config_t* config, ring_buffer_t* optional_ring_buffer )
{  
  uint32_t          uart_number;
  OSStatus          err = kNoErr;

  platform_mcu_powersave_disable();

  require_action_quiet( ( driver != NULL ) && ( peripheral != NULL ) && ( config != NULL ), exit, err = kParamErr);
  require_action_quiet( (optional_ring_buffer == NULL) || ((optional_ring_buffer->buffer != NULL ) && (optional_ring_buffer->size != 0)), exit, err = kParamErr);
  
  uart_number = platform_uart_get_port_number( peripheral->port );
  if( uart_number < 3 )
  {
    require_action_quiet( driver->tx_dma_handle, exit, err = kParamErr );
    hdma_tx[ uart_number ] = driver->tx_dma_handle;
    require_action_quiet( driver->rx_dma_handle, exit, err = kParamErr );
    hdma_rx[ uart_number ] = driver->rx_dma_handle;    
  }
  
  driver->rx_size              = 0;
  driver->tx_size              = 0;
  driver->last_transmit_result = kNoErr;
  driver->last_receive_result  = kNoErr;
  driver->peripheral           = (platform_uart_t*)peripheral;
  require_action_quiet( driver->uart_handle, exit, err = kParamErr );
//  driver->uart_handle          = (UART_HandleTypeDef *)&ComUartHandle;

  mico_rtos_init_semaphore( &driver->tx_complete, 1 );
  mico_rtos_init_semaphore( &driver->rx_complete, 1 );
  mico_rtos_init_semaphore( &driver->sem_wakeup,  1 );
  mico_rtos_init_mutex    ( &driver->tx_mutex );

  /* Configure TX and RX pin_mapping */
  platform_gpio_set_alternate_function( peripheral->pin_tx->port, peripheral->pin_tx->pin_number, GPIO_PULLUP, NULL );
  //platform_gpio_set_alternate_function( peripheral->pin_rx->port, peripheral->pin_rx->pin_number, GPIO_PULLUP, NULL );
  platform_pin_config_t pin_config;
  pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
  pin_config.gpio_mode = GPIO_MODE_INPUT;
  pin_config.gpio_pull = GPIO_PULLUP;
  platform_gpio_init( peripheral->pin_rx, &pin_config );
  
  if ( ( peripheral->pin_cts != NULL ) && ( config->flow_control == FLOW_CONTROL_CTS || config->flow_control == FLOW_CONTROL_CTS_RTS ) )
  {
      platform_gpio_set_alternate_function( peripheral->pin_cts->port, peripheral->pin_cts->pin_number, GPIO_PULLUP, NULL );
  }

  if ( ( peripheral->pin_rts != NULL ) && ( config->flow_control == FLOW_CONTROL_RTS || config->flow_control == FLOW_CONTROL_CTS_RTS ) )
  {
      platform_gpio_set_alternate_function( peripheral->pin_rts->port, peripheral->pin_rts->pin_number, GPIO_PULLUP, NULL );
  }
  
  /* Enable UART peripheral clock */
  switch( uart_number )
  {
  case 0:
    __HAL_RCC_USART1_CLK_ENABLE();
    break;
  case 1:
    __HAL_AFIO_REMAP_USART2_ENABLE();//comment to not remap
    __HAL_RCC_USART2_CLK_ENABLE();
    break;
  case 2:
    __HAL_RCC_USART3_CLK_ENABLE();
    break;
  default:
    break;
  }

  driver->uart_handle->Instance = driver->peripheral->port;
  driver->uart_handle->Init.Mode = UART_MODE_TX_RX;
  driver->uart_handle->Init.BaudRate = config->baud_rate;
  driver->uart_handle->Init.WordLength = ( ( config->data_width == DATA_WIDTH_9BIT ) || ( ( config->data_width == DATA_WIDTH_8BIT ) && ( config->parity != NO_PARITY ) ) ) ? UART_WORDLENGTH_9B : UART_WORDLENGTH_8B;
  driver->uart_handle->Init.StopBits = ( config->stop_bits == STOP_BITS_1 ) ? UART_STOPBITS_1 : UART_STOPBITS_2;

  switch ( config->parity )
  {
    
    case NO_PARITY:
      driver->uart_handle->Init.Parity = UART_PARITY_NONE;
      break;

    case EVEN_PARITY:
      driver->uart_handle->Init.Parity = UART_PARITY_EVEN;
      break;

    case ODD_PARITY:
      driver->uart_handle->Init.Parity = UART_PARITY_ODD;
      break;

    default:
      err = kParamErr;
      goto exit;
  }

  switch ( config->flow_control )
  {
    case FLOW_CONTROL_DISABLED:
      driver->uart_handle->Init.HwFlowCtl = UART_HWCONTROL_NONE;
      break;

    case FLOW_CONTROL_CTS:
      driver->uart_handle->Init.HwFlowCtl = UART_HWCONTROL_CTS;
      break;

    case FLOW_CONTROL_RTS:
      driver->uart_handle->Init.HwFlowCtl = UART_HWCONTROL_RTS;
      break;

    case FLOW_CONTROL_CTS_RTS:
      driver->uart_handle->Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
      break;

    default:
      err = kParamErr;
      goto exit;
  }
  
  /* Initialise USART peripheral */
  if( HAL_UART_Init( driver->uart_handle ) != HAL_OK )
  {
      err = kUnknownErr;
      goto exit;
  }
  
  /**************************************************************************
  * Initialise STM32 DMA registers
  * Note: If DMA is used, USART interrupt isn't enabled.
  **************************************************************************/
  __HAL_UART_DISABLE( driver->uart_handle );
  /* Enable DMA peripheral clock */
  if ( peripheral->tx_dma_config.controller == DMA1 )
  {
      __HAL_RCC_DMA1_CLK_ENABLE();
  }
  
  /* Fill init structure with common DMA settings */
  hdma_rx[ uart_number ]->Instance = peripheral->rx_dma_config.channel;
  hdma_rx[ uart_number ]->Instance->CPAR     = (uint32_t)&peripheral->port->DR;
  hdma_rx[ uart_number ]->Instance->CMAR     = (uint32_t) 0;
  hdma_rx[ uart_number ]->Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_rx[ uart_number ]->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_rx[ uart_number ]->Init.MemInc = DMA_MINC_ENABLE;
  hdma_rx[ uart_number ]->Init.Mode = DMA_NORMAL;
  hdma_rx[ uart_number ]->Init.Priority = DMA_PRIORITY_HIGH;

  if ( config->data_width == DATA_WIDTH_9BIT )
  {
    hdma_rx[ uart_number ]->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_rx[ uart_number ]->Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
  }
  else
  {
    hdma_rx[ uart_number ]->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx[ uart_number ]->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  }

  HAL_DMA_Init( hdma_rx[ uart_number ] );
  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA( driver->uart_handle, hdmarx, *hdma_rx[ uart_number ] );
  
  /* Fill init structure with common DMA settings */
  hdma_tx[ uart_number ]->Instance = peripheral->tx_dma_config.channel;
  hdma_tx[ uart_number ]->Instance->CPAR     = (uint32_t)&peripheral->port->DR;
  hdma_tx[ uart_number ]->Instance->CMAR     = (uint32_t) 0;
  hdma_tx[ uart_number ]->Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_tx[ uart_number ]->Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_tx[ uart_number ]->Init.MemInc = DMA_MINC_ENABLE;
  hdma_tx[ uart_number ]->Init.Mode = DMA_NORMAL;
  hdma_tx[ uart_number ]->Init.Priority = DMA_PRIORITY_HIGH;

  if ( config->data_width == DATA_WIDTH_9BIT )
  {
    hdma_tx[ uart_number ]->Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_tx[ uart_number ]->Init.MemDataAlignment = DMA_PDATAALIGN_HALFWORD;
  }
  else
  {
    hdma_tx[ uart_number ]->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_tx[ uart_number ]->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  }
  HAL_DMA_Init( hdma_tx[ uart_number ] );
  /* Associate the initialized DMA handle to the UART handle */
  __HAL_LINKDMA( driver->uart_handle, hdmatx, *hdma_tx[ uart_number ] );
  /**************************************************************************
  * Initialise STM32 DMA interrupts
  **************************************************************************/
  
  /* Configure TX DMA interrupt on Cortex-M3 */
  NVIC_EnableIRQ( peripheral->tx_dma_config.irq_vector );
  
  /* Enable TC (transfer complete) and TE (transfer error) interrupts on source */
  //clear_dma_interrupts( peripheral->tx_dma_config.stream, peripheral->tx_dma_config.complete_flags | peripheral->tx_dma_config.error_flags );
  clear_dma_interrupts( peripheral->tx_dma_config.controller, peripheral->tx_dma_config.complete_flags | peripheral->tx_dma_config.error_flags );
  //DMA_ITConfig( peripheral->tx_dma_config.stream, DMA_INTERRUPT_FLAGS, ENABLE );
  __HAL_DMA_ENABLE_IT( driver->uart_handle->hdmatx, DMA_INTERRUPT_FLAGS );
  
  /* Enable USART interrupt vector in Cortex-M3 */
  NVIC_EnableIRQ( uart_irq_vectors[uart_number] );
  //USART_DMACmd( driver->peripheral->port, USART_DMAReq_Tx, DISABLE );
  CLEAR_BIT( driver->peripheral->port->CR3, USART_CR3_DMAT );
  
  /* Enable USART */
  //USART_Cmd( peripheral->port, ENABLE );
  __HAL_UART_ENABLE( driver->uart_handle );

  /* Enable both transmit and receive */
  peripheral->port->CR1 |= USART_CR1_TE;
  peripheral->port->CR1 |= USART_CR1_RE;

  /* Setup ring buffer */
  if ( optional_ring_buffer != NULL )
  {
    /* Note that the ring_buffer should've been initialised first */
    driver->rx_buffer = optional_ring_buffer;
    driver->rx_size   = 0;
    receive_bytes( driver, optional_ring_buffer->buffer, optional_ring_buffer->size, 0 );
  }
  else
  {
    /* Not using ring buffer. Configure RX DMA interrupt on Cortex-M3 */
    NVIC_EnableIRQ( peripheral->rx_dma_config.irq_vector );

    /* Enable TC (transfer complete) and TE (transfer error) interrupts on source */    
    //clear_dma_interrupts( peripheral->rx_dma_config.stream, peripheral->rx_dma_config.complete_flags | peripheral->rx_dma_config.error_flags );
    clear_dma_interrupts( peripheral->rx_dma_config.controller, peripheral->rx_dma_config.complete_flags | peripheral->rx_dma_config.error_flags );
    //DMA_ITConfig( peripheral->rx_dma_config.stream, DMA_INTERRUPT_FLAGS, ENABLE );
    __HAL_DMA_ENABLE_IT( driver->uart_handle->hdmarx, DMA_INTERRUPT_FLAGS );
  }
#ifdef UART_NOT_USE_DMA
  peripheral->port->CR1 &= ~USART_CR1_RXNEIE;
  peripheral->port->CR3 &= ~USART_CR3_DMAR;
#endif
exit:
  platform_mcu_powersave_enable( );
  return err;
}

OSStatus platform_uart_deinit( platform_uart_driver_t* driver )
{
  uint8_t          uart_number;
  OSStatus          err = kNoErr;
  
  platform_mcu_powersave_disable();

  require_action_quiet( ( driver != NULL ), exit, err = kParamErr);

  uart_number = platform_uart_get_port_number( driver->peripheral->port );

  /* Disable USART */
  //USART_Cmd( driver->peripheral->port, DISABLE );
  __HAL_UART_DISABLE( driver->uart_handle );
  
  /* Deinitialise USART */
  //USART_DeInit( driver->peripheral->port );

  /**************************************************************************
   * De-initialise STM32 DMA and interrupt
   **************************************************************************/

  /* Deinitialise DMA streams */
  //DMA_DeInit( driver->peripheral->tx_dma_config.stream );
  HAL_DMA_DeInit( driver->uart_handle->hdmatx );
  //DMA_DeInit( driver->peripheral->rx_dma_config.stream );
  HAL_DMA_DeInit( driver->uart_handle->hdmarx );

  /* Disable TC (transfer complete) interrupt at the source */
  //DMA_ITConfig( driver->peripheral->tx_dma_config.stream, DMA_INTERRUPT_FLAGS, DISABLE );
  __HAL_DMA_DISABLE_IT( driver->uart_handle->hdmatx, DMA_INTERRUPT_FLAGS );
  //DMA_ITConfig( driver->peripheral->rx_dma_config.stream, DMA_INTERRUPT_FLAGS, DISABLE );
  __HAL_DMA_DISABLE_IT( driver->uart_handle->hdmarx, DMA_INTERRUPT_FLAGS );

  /* Disable transmit DMA interrupt at Cortex-M3 */
  NVIC_DisableIRQ( driver->peripheral->tx_dma_config.irq_vector );

  /**************************************************************************
   * De-initialise STM32 USART interrupt
   **************************************************************************/

  //USART_ITConfig( driver->peripheral->port, USART_IT_RXNE, DISABLE );
  __HAL_UART_DISABLE_IT( driver->uart_handle, UART_IT_RXNE );

  /* Disable UART interrupt vector on Cortex-M3 */
  NVIC_DisableIRQ( driver->peripheral->rx_dma_config.irq_vector );

  /* Disable registers clocks */
  //uart_peripheral_clock_functions[uart_number]( uart_peripheral_clocks[uart_number], DISABLE );
  switch( uart_number )
  {
  case 0:
    __HAL_RCC_USART1_CLK_DISABLE();
    break;
  case 1:
    __HAL_RCC_USART2_CLK_DISABLE();
    break;
  case 2:
    __HAL_RCC_USART3_CLK_DISABLE();
    break;
  default:
    break;
  }

  mico_rtos_deinit_semaphore( &driver->rx_complete );
  mico_rtos_deinit_semaphore( &driver->tx_complete );
  mico_rtos_deinit_mutex( &driver->tx_mutex );
  driver->rx_size              = 0;
  driver->tx_size              = 0;
  driver->last_transmit_result = kNoErr;
  driver->last_receive_result  = kNoErr;

exit:
  platform_mcu_powersave_enable();
  return err;
}

OSStatus platform_uart_transmit_bytes( platform_uart_driver_t* driver, const uint8_t* data_out, uint32_t size )
{
  OSStatus err = kNoErr;
  
  platform_mcu_powersave_disable();
  
  mico_rtos_lock_mutex( &driver->tx_mutex );

  require_action_quiet( ( driver != NULL ) && ( data_out != NULL ) && ( size != 0 ), exit, err = kParamErr);
#ifdef UART_NOT_USE_DMA
  while (HAL_UART_GetState(driver->uart_handle) != HAL_UART_STATE_READY)
  {
    driver->uart_handle->State = HAL_UART_STATE_READY;
  }
  if (HAL_UART_Transmit( driver->uart_handle, (uint8_t *)data_out, size,1 ) != HAL_OK)
  {
      err = -1;
  }
#else
  __HAL_DMA_DISABLE( driver->uart_handle->hdmatx );
  /* Clear interrupt status before enabling DMA otherwise error occurs immediately */  
  //clear_dma_interrupts( driver->peripheral->tx_dma_config.stream, driver->peripheral->tx_dma_config.complete_flags | driver->peripheral->tx_dma_config.error_flags );
  clear_dma_interrupts( driver->peripheral->tx_dma_config.controller, driver->peripheral->tx_dma_config.complete_flags | driver->peripheral->tx_dma_config.error_flags );
  clear_dma_interrupts( driver->peripheral->tx_dma_config.controller, DMA_ISR_HTIF2 );
  /* Init DMA parameters and variables */
  driver->last_transmit_result                    = kGeneralErr;
  driver->tx_size                                 = size;
    
  //driver->peripheral->tx_dma_config.stream->CR   &= ~(uint32_t) DMA_SxCR_CIRC;
  CLEAR_BIT( driver->peripheral->tx_dma_config.channel->CCR, DMA_CIRCULAR );
  //driver->peripheral->tx_dma_config.stream->NDTR  = size;
  driver->peripheral->tx_dma_config.channel->CNDTR    = size;
  //driver->peripheral->tx_dma_config.stream->M0AR  = (uint32_t)data_out;
  driver->peripheral->tx_dma_config.channel->CMAR     = (uint32_t)data_out;
  
  //USART_DMACmd( driver->peripheral->port, USART_DMAReq_Tx, ENABLE );
  SET_BIT( driver->peripheral->port->CR3, USART_CR3_DMAT );
  //USART_ClearFlag( driver->peripheral->port, USART_FLAG_TC );
  __HAL_UART_CLEAR_FLAG( driver->uart_handle, UART_FLAG_TC );
  //driver->peripheral->tx_dma_config.stream->CR   |= DMA_SxCR_EN;
  __HAL_DMA_ENABLE( driver->uart_handle->hdmatx );
  
/* Wait for transmission complete */
  mico_rtos_get_semaphore( &driver->tx_complete, MICO_NEVER_TIMEOUT );

  //while ( ( driver->peripheral->port->SR & USART_SR_TC ) == 0 )
  while( __HAL_USART_GET_FLAG( driver->uart_handle, USART_FLAG_TC ) == RESET )
  {
  }

  /* Disable DMA and clean up */
  //USART_DMACmd( driver->peripheral->port, USART_DMAReq_Tx, DISABLE );
  CLEAR_BIT( driver->peripheral->port->CR3, USART_CR3_DMAT );
  driver->tx_size = 0;
  err = driver->last_transmit_result;
  
#endif
exit:  
  mico_rtos_unlock_mutex( &driver->tx_mutex );
  platform_mcu_powersave_enable();
  return err;
}

OSStatus platform_uart_receive_bytes( platform_uart_driver_t* driver, uint8_t* data_in, uint32_t expected_data_size, uint32_t timeout_ms )
{
  OSStatus err = kNoErr;

  require_action_quiet( ( driver != NULL ) && ( data_in != NULL ) && ( expected_data_size != 0 ), exit, err = kParamErr);
#ifdef UART_NOT_USE_DMA  
  require_action_quiet(HAL_UART_Receive(driver->uart_handle, (uint8_t *)data_in, expected_data_size, timeout_ms) == HAL_OK, exit, err = kTimeoutErr);
#else
  if ( driver->rx_buffer != NULL)
  {
    while ( expected_data_size != 0 )
    {
      uint32_t transfer_size = MIN( driver->rx_buffer->size / 2, expected_data_size );
      
      /* Check if ring buffer already contains the required amount of data. */
      if ( transfer_size > ring_buffer_used_space( driver->rx_buffer ) )
      {
        /* Set rx_size and wait in rx_complete semaphore until data reaches rx_size or timeout occurs */
        driver->last_receive_result = kNoErr;
        driver->rx_size             = transfer_size;
        
        err = mico_rtos_get_semaphore( &driver->rx_complete, timeout_ms );

        /* Reset rx_size to prevent semaphore being set while nothing waits for the data */
        driver->rx_size = 0;

        if( err != kNoErr )
          goto exit;
      }
      err = driver->last_receive_result;
      expected_data_size -= transfer_size;
      
      // Grab data from the buffer
      do
      {
        uint8_t* available_data;
        uint32_t bytes_available;
        
        ring_buffer_get_data( driver->rx_buffer, &available_data, &bytes_available );
        bytes_available = MIN( bytes_available, transfer_size );
        memcpy( data_in, available_data, bytes_available );
        transfer_size -= bytes_available;
        data_in = ( (uint8_t*) data_in + bytes_available );
        ring_buffer_consume( driver->rx_buffer, bytes_available );
      } while ( transfer_size != 0 );
    }
  }
  else
  {
    err = receive_bytes( driver, data_in, expected_data_size, timeout_ms );
  }
#endif
exit:
  return err;
}

static OSStatus receive_bytes( platform_uart_driver_t* driver, void* data, uint32_t size, uint32_t timeout )
{
  OSStatus err = kNoErr;

  if ( driver->rx_buffer != NULL )
  {
//    driver->peripheral->rx_dma_config.stream->CR |= DMA_SxCR_CIRC;
    //driver->uart_handle->hdmarx->Instance->CCR |= DMA_CIRCULAR;
    SET_BIT( driver->uart_handle->hdmarx->Instance->CCR, DMA_CIRCULAR );
    
    // Enabled individual byte interrupts so progress can be updated
//    USART_ClearITPendingBit( driver->peripheral->port, USART_IT_RXNE );
    __HAL_UART_CLEAR_FLAG( driver->uart_handle, UART_IT_RXNE );
//    USART_ITConfig( driver->peripheral->port, USART_IT_RXNE, ENABLE );
    __HAL_UART_ENABLE_IT( driver->uart_handle, UART_IT_RXNE );
  }
  else
  {
    driver->rx_size = size;
//    driver->peripheral->rx_dma_config.stream->CR &= ~(uint32_t) DMA_SxCR_CIRC;
    driver->uart_handle->hdmarx->Instance->CCR &= ~(uint32_t) DMA_CIRCULAR;
  }

//  clear_dma_interrupts( driver->peripheral->rx_dma_config.stream, driver->peripheral->rx_dma_config.complete_flags | driver->peripheral->rx_dma_config.error_flags );
  clear_dma_interrupts( driver->peripheral->rx_dma_config.controller, driver->peripheral->tx_dma_config.complete_flags | driver->peripheral->rx_dma_config.error_flags );
//  driver->peripheral->rx_dma_config.stream->NDTR  = size;
  driver->uart_handle->hdmarx->Instance->CNDTR = size;
//  driver->peripheral->rx_dma_config.stream->M0AR  = (uint32_t)data;
  driver->uart_handle->hdmarx->Instance->CMAR = (uint32_t)data;
//  driver->peripheral->rx_dma_config.stream->CR   |= DMA_SxCR_EN;
  driver->uart_handle->hdmarx->Instance->CCR |= DMA_CCR_EN;
//  USART_DMACmd( driver->peripheral->port, USART_DMAReq_Rx, ENABLE );
  SET_BIT( driver->uart_handle->Instance->CR3, USART_CR3_DMAR );
  
  if ( timeout > 0 )
  {
    err = mico_rtos_get_semaphore( &driver->rx_complete, timeout );
  }
  return err;
}

uint32_t platform_uart_get_length_in_buffer( platform_uart_driver_t* driver )
{  
  return ring_buffer_used_space( driver->rx_buffer );
}

static void clear_dma_interrupts( DMA_TypeDef* controller, uint32_t flags )
{
    DMA1->IFCR |= flags;
}

static uint32_t get_dma_irq_status( DMA_TypeDef* controller )
{
    return controller->ISR;
}

uint8_t platform_uart_get_port_number( USART_TypeDef* uart )
{
    if ( uart == USART1 )
    {
        return 0;
    }
    else if ( uart == USART2 )
    {
        return 1;
    }
    else if ( uart == USART3 )
    {
        return 2;
    }
    else
    {
        return 0xff;
    }
}
/******************************************************
*            Interrupt Service Routines
******************************************************/


void platform_uart_irq( platform_uart_driver_t* driver )
{
//  platform_uart_port_t* uart = (platform_uart_port_t*) driver->peripheral->port;

  // Clear all interrupts. It's safe to do so because only RXNE interrupt is enabled
  //uart->SR = (uint16_t) ( uart->SR | 0xffff );
  __HAL_UART_CLEAR_FLAG( driver->uart_handle, UART_FLAG_RXNE );

  // Update tail
  driver->rx_buffer->tail = driver->rx_buffer->size - driver->uart_handle->hdmarx->Instance->CNDTR;

  // Notify thread if sufficient data are available
  if ( ( driver->rx_size > 0 ) && ( ring_buffer_used_space( driver->rx_buffer ) >= driver->rx_size ) )
  {
      mico_rtos_set_semaphore( &driver->rx_complete );
      driver->rx_size = 0;
  }
}

void platform_uart_tx_dma_irq( platform_uart_driver_t* driver )
{
#if 1
    if ( ( get_dma_irq_status( driver->peripheral->tx_dma_config.controller ) & driver->peripheral->tx_dma_config.complete_flags ) != 0 )
    {
        clear_dma_interrupts( driver->peripheral->tx_dma_config.controller, driver->peripheral->tx_dma_config.complete_flags );
        driver->last_transmit_result = kNoErr;
    }

    if ( ( get_dma_irq_status( driver->peripheral->tx_dma_config.controller ) & driver->peripheral->tx_dma_config.error_flags ) != 0 )
    {
        clear_dma_interrupts( driver->peripheral->tx_dma_config.controller, driver->peripheral->tx_dma_config.error_flags );
        driver->last_transmit_result = kGeneralErr;
    }

    if ( driver->tx_size > 0 )
    {
        /* Set semaphore regardless of result to prevent waiting thread from locking up */
        mico_rtos_set_semaphore( &driver->tx_complete );
    }
#endif
}

void platform_uart_rx_dma_irq( platform_uart_driver_t* driver )
{
    if ( ( get_dma_irq_status( driver->peripheral->rx_dma_config.controller ) & driver->peripheral->rx_dma_config.complete_flags ) != 0 )
    {
        clear_dma_interrupts( driver->peripheral->rx_dma_config.controller, driver->peripheral->rx_dma_config.complete_flags );
        driver->last_receive_result = kNoErr;
    }

    if ( ( get_dma_irq_status( driver->peripheral->rx_dma_config.controller ) & driver->peripheral->rx_dma_config.error_flags ) != 0 )
    {
        clear_dma_interrupts( driver->peripheral->rx_dma_config.controller, driver->peripheral->rx_dma_config.error_flags );
        driver->last_receive_result = kGeneralErr;
    }

    if ( driver->rx_size > 0 )
    {
        /* Set semaphore regardless of result to prevent waiting thread from locking up */
        mico_rtos_set_semaphore( &driver->rx_complete );
    }
}


