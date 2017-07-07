/**
 ******************************************************************************
 * @file    platform_assert.h
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#pragma once

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#ifdef __GNUC__
#define MICO_ASSERTION_FAIL_ACTION() __asm__("bkpt")
#elif defined ( __IAR_SYSTEMS_ICC__ )
#define MICO_ASSERTION_FAIL_ACTION() __asm("bkpt 0")
#elif defined ( __CC_ARM )
#define MICO_ASSERTION_FAIL_ACTION() __asm("bkpt 0")
#endif
 
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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
