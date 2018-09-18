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

OSStatus init_can( mico_can_t can );

OSStatus tx_can_pkg( mico_can_t can, const CanTxMsgTypeDef *msg);

OSStatus MicoCanMessageRead( mico_can_t can, const void *msg );

#endif

