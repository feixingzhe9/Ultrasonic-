/* 
*  Author: Adam Huang
*  Date:2016/6/4
*/
#include "app_platform.h"
#include <stdlib.h>
#include "board_init.h"
#include "protocol.h"
#include "upgrade_flash.h"


#define platform_log(M, ...) custom_log("Platform", M, ##__VA_ARGS__)
#define platform_log_trace() custom_log_trace("Platform")

OSStatus Platform_Init( void )
{ 

  OSStatus err = kNoErr;

  if( err != kNoErr )
  {
    platform_log("platform init error: no enough memory!");
  }
  else
  {
    platform_log("platform init success!");
  }
  return err;
}

void bsp_Init( void )
{
  board_gpios_init();
}



/*********************END OF FILE**************/
