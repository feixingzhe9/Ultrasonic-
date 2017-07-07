/**
 ******************************************************************************
 * @file    MicoDriverCan.h
 * @author  Adam Huang
 * @version V1.0.0
 * @date    13-Jan-2017
 * @brief   This file provides all the headers of Can operation functions.
 ******************************************************************************
 **/

#ifndef __MICODRIVERCAN_H__
#define __MICODRIVERCAN_H__

#pragma once

#include "common.h"
#include "platform.h"
#include "platform_peripheral.h"

OSStatus MicoCanInitialize( mico_can_t can );

OSStatus MicoCanMessageSend( mico_can_t can, const CanTxMsgTypeDef *msg);

OSStatus MicoCanMessageRead( mico_can_t can, const void *msg );

#endif

