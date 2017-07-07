/**
 ******************************************************************************
 * @file    Retarget.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 ******************************************************************************
 *
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 *
 ******************************************************************************
 */

#include <stdlib.h>
#include <yfuns.h>
#include "platform.h"
#include "platform_config.h"
#include "mico_platform.h"


#ifdef BOOTLOADER
int putchar(int ch)
{
  MicoUartSend( STDIO_UART, &ch, 1 );
  return ch;
}
#else
size_t __write( int handle, const unsigned char * buffer, size_t size )
{
  UNUSED_PARAMETER(handle);
  
  if ( buffer == 0 )
  {
    return 0;
  }

#ifndef MICO_DISABLE_STDIO
  MicoUartSend( STDIO_UART, (const char*)buffer, size );
#endif
  
  return size;
}
#endif
