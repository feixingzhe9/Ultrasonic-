/**
 ******************************************************************************
 * @file    paltform_can.c
 * @author  Adam Huang
 * @version V1.0.0
 * @date    11-Jan-2017
 * @brief   This file provide can driver functions.
 ******************************************************************************
 */

#include "platform.h"
#include "platform_peripheral.h"
#include "stm32f1xx.h"
#include "platform_logging.h"

#include "can_protocol.h"
#include "fifo.h"

typedef unsigned int    u32;

#define CAN_FILTER_ID       (0x60)
#define CAN_FILTER_MASK     (0xff<<13)
extern uint8_t GetCanSrcId(void);
uint32_t ultrasonic_src_id;
OSStatus platform_can_init( const platform_can_driver_t* can )
{

    //uint32_t can_mac_id = CAN_SUB_PB_ID;
    ultrasonic_src_id = GetCanSrcId();
    OSStatus    err = kNoErr;
    CAN_FilterConfTypeDef     CAN_FilterInitStructure;
    
    platform_mcu_powersave_disable();
    require_action_quiet( can, exit, err = kParamErr);
    require_action_quiet( can->handle, exit, err = kParamErr);
    require_action_quiet( can->port, exit, err = kParamErr );
    /* Initialize the associated GPIO */
    require_action_quiet( can->pin_tx, exit, err = kParamErr );
    require_action_quiet( can->pin_rx, exit, err = kParamErr );
    
    platform_pin_config_t pin_config;
    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode = GPIO_MODE_AF_PP;
    pin_config.gpio_pull = GPIO_PULLUP;
    platform_gpio_init( can->pin_tx, &pin_config );
    
    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode = GPIO_MODE_INPUT;
    pin_config.gpio_pull = GPIO_PULLUP;
    platform_gpio_init( can->pin_rx, &pin_config );
    
    if( can->port == CAN1 )
    {
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
#if defined(STM32F105xC) || defined(STM32F107xC)
    else if( can->port == CAN2 )
    {
      __HAL_RCC_CAN2_CLK_ENABLE();
    }
#endif
    else
    {
      require_action_quiet( NULL, exit, err = kParamErr );
    }
    
    can->handle->Instance               = can->port;
    can->handle->Init.Prescaler         = 9;
    can->handle->Init.Mode              = CAN_MODE_NORMAL;
    can->handle->Init.SJW               = CAN_SJW_1TQ;
    can->handle->Init.BS1               = CAN_BS1_4TQ;
    can->handle->Init.BS2               = CAN_BS2_3TQ;
    can->handle->Init.TTCM              = DISABLE;
    can->handle->Init.ABOM              = ENABLE;
    can->handle->Init.AWUM              = ENABLE;
    can->handle->Init.NART              = DISABLE;
    can->handle->Init.RFLM              = DISABLE;
    can->handle->Init.TXFP              = DISABLE;
    require_action_quiet( HAL_CAN_Init( can->handle ) == HAL_OK, exit, err = kGeneralErr );
    

    CAN_FilterInitStructure.FilterIdHigh        = ((ultrasonic_src_id<<3)<<13)>>16;//can_mac_id>>16;//((can_mac_id<<3)<<13)>>16;
    CAN_FilterInitStructure.FilterIdLow         = ((ultrasonic_src_id<<3)<<13) & 0xffff| CAN_ID_EXT ;//can_mac_id & 0xffff;//((can_mac_id<<3)<<13) & 0xffff;
    CAN_FilterInitStructure.FilterMaskIdHigh    = (CAN_FILTER_MASK<<3)>>16;
    CAN_FilterInitStructure.FilterMaskIdLow     = (CAN_FILTER_MASK<<3) & 0xffff | CAN_ID_EXT | CAN_RTR_REMOTE;


    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_FilterInitStructure.FilterNumber        = 0;
    CAN_FilterInitStructure.FilterMode          = CAN_FILTERMODE_IDMASK;
    CAN_FilterInitStructure.FilterScale         = CAN_FILTERSCALE_32BIT;
    CAN_FilterInitStructure.FilterActivation    = ENABLE;
    CAN_FilterInitStructure.BankNumber          = 0;
    require_action_quiet( HAL_CAN_ConfigFilter( can->handle, &CAN_FilterInitStructure ) == HAL_OK,\
      exit, err = kGeneralErr );
  
    
    
/////////////////**************************///////////////////////////////   
#if 0
#define CAN_ID_TEST         0x00005678
    CAN_FilterInitStructure.FilterIdHigh        = (CAN_ID_TEST<<3) >> 16;
    CAN_FilterInitStructure.FilterIdLow         = (CAN_ID_TEST<<3) & 0xffff;
    //CAN_FilterInitStructure.FilterMaskIdHigh    = CAN_FILTER_MASK>>16;
    //CAN_FilterInitStructure.FilterMaskIdLow     = CAN_FILTER_MASK & 0xffff;


    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    CAN_FilterInitStructure.FilterNumber        = 2;
    CAN_FilterInitStructure.FilterMode          = CAN_FILTERMODE_IDLIST;
    CAN_FilterInitStructure.FilterScale         = CAN_FILTERSCALE_32BIT;
    CAN_FilterInitStructure.FilterActivation    = ENABLE;
    CAN_FilterInitStructure.BankNumber          = 2;
    require_action_quiet( HAL_CAN_ConfigFilter( can->handle, &CAN_FilterInitStructure ) == HAL_OK,\
      exit, err = kGeneralErr );
#endif
////////////////////*************************//////////////////////////////////////////////////////


    
    __HAL_CAN_ENABLE_IT( can->handle, CAN_IT_FMP0 );
    
    NVIC_ClearPendingIRQ( USB_LP_CAN1_RX0_IRQn ); 
    NVIC_EnableIRQ( USB_LP_CAN1_RX0_IRQn );
  
exit:
    platform_mcu_powersave_enable();
    return err;
}

OSStatus platform_can_loop_message( const platform_can_driver_t* can )
{
  OSStatus    err = kNoErr;
  CanTxMsgTypeDef  *can_tx_msg;
  CanRxMsgTypeDef  *can_rx_msg;
  
  require_action_quiet( can->handle->pTxMsg, exit, err = kParamErr );
  require_action_quiet( can->handle->pRxMsg, exit, err = kParamErr );
  
  can_tx_msg = can->handle->pTxMsg;
  can_rx_msg = can->handle->pRxMsg;
  can_tx_msg->ExtId     = can_rx_msg->ExtId;
  can_tx_msg->IDE       = can_rx_msg->IDE;
  can_tx_msg->RTR       = CAN_RTR_DATA;
  can_tx_msg->DLC       = can_rx_msg->DLC;
  
  for( uint8_t i = 0; i < can_rx_msg->DLC; i++ )
  { 
    can_tx_msg->Data[i] = can_rx_msg->Data[i];
  }
  
  require_action_quiet( HAL_CAN_Transmit_IT( can->handle ) == HAL_OK, exit, err = kGeneralErr );
  
exit:
    return err; 
}

#if 0
OSStatus platform_can_send_message( const platform_can_driver_t* can, uint8_t *msg, uint8_t len )
{
  OSStatus    err = kNoErr;
  CanTxMsgTypeDef  *can_tx_msg;

  require_action_quiet( can->handle->pTxMsg, exit, err = kParamErr );
  can_tx_msg = can->handle->pTxMsg;
  
  //can_tx_msg->IDE       = CAN_ID_STD;
  //can_tx_msg->RTR       = CAN_RTR_DATA;
  can_tx_msg->DLC       = len;
  
  for( uint8_t i = 0; i < len; i++ )
  {
    can_tx_msg->Data[i] = msg[i];
  }
  
  require_action_quiet( HAL_CAN_Transmit( can->handle, 0x10 ) == HAL_OK, exit, err = kGeneralErr );
  
exit:
    return err; 
}

#else

OSStatus platform_can_send_message( const platform_can_driver_t* can, const CanTxMsgTypeDef *msg)
{
  OSStatus    err = kNoErr;

  require_action_quiet( can->handle->pTxMsg, exit, err = kParamErr );
  require_action_quiet( msg->DLC <= 8, exit, err = kParamErr );
  //memcpy(can->handle->pTxMsg->Data, msg->Data, msg->DLC);
  memcpy(can->handle->pTxMsg, msg, sizeof(CanTxMsgTypeDef));
  
  //require_action_quiet( HAL_CAN_Transmit( can->handle, 0x10 ) == HAL_OK, exit, err = kGeneralErr );
  require_action_quiet( HAL_CAN_Transmit( can->handle, 0x10 ) == HAL_OK, exit, err = kGeneralErr );
  
exit:
    return err; 
}
#endif
OSStatus platform_can_receive_message( const platform_can_driver_t* can, uint8_t *msg )
{
  OSStatus    err = kNoErr;
  CanRxMsgTypeDef  *can_rx_msg;

  require_action_quiet( can->handle->pRxMsg, exit, err = kParamErr );
  
  can_rx_msg = can->handle->pRxMsg;
  
  require_action_quiet( HAL_CAN_Receive( can->handle, CAN_FIFO0, 0x0 ) == HAL_OK, exit, err = kGeneralErr );
  memcpy( msg, can_rx_msg->Data, 8 );
  
exit:
    return err; 
}

void platform_can_rx_irq( platform_can_driver_t* can_driver )
{
  if( can_driver->handle )
  {
    HAL_CAN_IRQHandler( can_driver->handle );
  }
  //can_driver->rx_complete = 1;
}

//CanRxMsgTypeDef RxMessage;

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
    can_pkg_t can_pkg_tmp;
    HAL_CAN_Receive_IT( hcan, CAN_FIFO0);
    
#if 1
    ///////// put CAN package into fifo
        can_pkg_tmp.id.CANx_ID = hcan->pRxMsg->ExtId;
        can_pkg_tmp.len = hcan->pRxMsg->DLC;
        memcpy(can_pkg_tmp.data.CanData, hcan->pRxMsg->Data, hcan->pRxMsg->DLC);
        FifoPutCanPkg(can_fifo, can_pkg_tmp);    
#endif
    
#if 0
    memcpy( &RxMessage, hcan->pRxMsg, sizeof(CanRxMsgTypeDef) );
    CanTxMsgTypeDef  *can_tx_msg;
    CanRxMsgTypeDef  *can_rx_msg;
    can_tx_msg = hcan->pTxMsg;
    can_rx_msg = hcan->pRxMsg;

    can_tx_msg->ExtId     = can_rx_msg->ExtId;
    can_tx_msg->IDE       = can_rx_msg->IDE;
    can_tx_msg->RTR       = CAN_RTR_DATA;
    can_tx_msg->DLC       = can_rx_msg->DLC;

    for( uint8_t i = 0; i < can_rx_msg->DLC; i++ )
    {
        can_tx_msg->Data[i] = can_rx_msg->Data[i];
    }

    HAL_CAN_Transmit_IT( hcan );
#endif
}

