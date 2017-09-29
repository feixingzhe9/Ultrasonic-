/**
 ******************************************************************************
 * @file    BootloaderEntrance.c
 * @author  William Xu
 * @version V2.0.0
 * @date    05-Oct-2014
 * @brief   MICO bootloader main entrance.
 ******************************************************************************
 *
 *  The MIT License
 *  Copyright (c) 2014 MXCHIP Inc.
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is furnished
 *  to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *  all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 *  IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 ******************************************************************************
 */


#include "mico.h"
#include "platform.h"
#include "platform_internal.h"
#include "platform_config.h"
#include "bootloader.h"

#define boot_log(M, ...) custom_log("BOOT", M, ##__VA_ARGS__)
#define boot_log_trace() custom_log_trace("BOOT")

extern void Main_Menu(void);
extern OSStatus update(void);


#ifdef MICO_ENABLE_STDIO_TO_BOOT
extern int stdio_break_in(void);
#endif

static void enable_protection( void )
{
  mico_partition_t i;
  mico_logic_partition_t *partition;

  for( i = MICO_PARTITION_BOOTLOADER; i < MICO_PARTITION_MAX; i++ ){
    partition = MicoFlashGetInfo( i );
    if( PAR_OPT_WRITE_DIS == ( partition->partition_options & PAR_OPT_WRITE_MASK )  )
      MicoFlashEnableSecurity( i, 0x0, MicoFlashGetInfo(i)->partition_length );
  }
}

WEAK bool MicoShouldEnterBootloader( void )
{
  return false;
}

void bootloader_start_app( uint32_t app_addr )
{
  enable_protection( );
  startApplication( app_addr );
}


int main(void)
{
  //init_clocks();

  init_memory();
  init_architecture();
  init_platform_bootloader();

  mico_set_bootload_ver();
  
  update();

  enable_protection();

#ifdef MICO_ENABLE_STDIO_TO_BOOT
  if (stdio_break_in() == 1)
    goto BOOT;
#endif
  
//  if( MicoShouldEnterBootloader() == false )
    bootloader_start_app( (MicoFlashGetInfo(MICO_PARTITION_APPLICATION))->partition_start_addr );

#ifdef MICO_ENABLE_STDIO_TO_BOOT
BOOT:
#endif


  while(1){    
#if 0
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SYS_LED );
    HAL_Delay(1);
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SYS_LED );
    HAL_Delay(1);
#endif
    //Main_Menu ();
  }
}

