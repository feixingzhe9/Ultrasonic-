/**
 ******************************************************************************
 * @file    rtos.c
 * @author  William Xu
 * @version V1.0.0
 * @date    05-May-2014
 * @brief   Definitions of the MiCO RTOS abstraction layer for the special case
 *          of having no RTOS
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

#include "common.h"
#include "platform_peripheral.h"


#define DISABLE_INTERRUPTS() do { __asm("CPSID i"); } while (0)

#define ENABLE_INTERRUPTS() do { __asm("CPSIE i"); } while (0)

typedef volatile struct _noos_semaphore_t
{
  uint8_t count;
} noos_semaphore_t;


OSStatus mico_rtos_init_semaphore( mico_semaphore_t* semaphore, int count )
{
#if 0
    noos_semaphore_t *noos_semaphore;
    UNUSED_PARAMETER( count );
    noos_semaphore = malloc(sizeof(noos_semaphore_t));
    noos_semaphore->count = 0;
    *semaphore = (void *)noos_semaphore;
#endif
    return kNoErr;
}

OSStatus mico_rtos_get_semaphore( mico_semaphore_t* semaphore, uint32_t timeout_ms )
{
#if 0
    noos_semaphore_t *noos_semaphore = (noos_semaphore_t *)*semaphore;
    int delay_start;

    if( noos_semaphore == NULL)
        return kNotInitializedErr;

    delay_start = mico_get_time(); 
    while( noos_semaphore->count == 0){
      if(mico_get_time() >= delay_start + timeout_ms && timeout_ms != MICO_NEVER_TIMEOUT){
        return kTimeoutErr;
      }
    }

    DISABLE_INTERRUPTS();
    noos_semaphore->count--;
    ENABLE_INTERRUPTS();
#endif
    return kNoErr;
}

OSStatus mico_rtos_set_semaphore( mico_semaphore_t* semaphore )
{
#if 0
    noos_semaphore_t *noos_semaphore = (noos_semaphore_t *)*semaphore;

    if( noos_semaphore == NULL)
        return kNotInitializedErr;

    DISABLE_INTERRUPTS();
    noos_semaphore->count++;
    ENABLE_INTERRUPTS();
#endif
    return kNoErr;
}

OSStatus mico_rtos_deinit_semaphore( mico_semaphore_t* semaphore )
{
#if 0
    noos_semaphore_t *noos_semaphore = (noos_semaphore_t *)*semaphore;

    if( noos_semaphore == NULL)
        return kNotInitializedErr;

    free((void *)noos_semaphore);
#endif
    return kNoErr;
}


OSStatus mico_rtos_init_mutex( mico_mutex_t* mutex )
{
    UNUSED_PARAMETER(mutex);
    return kNoErr;
}


OSStatus mico_rtos_lock_mutex( mico_mutex_t* mutex )
{
    UNUSED_PARAMETER(mutex);
    return kNoErr;
}

OSStatus mico_rtos_unlock_mutex( mico_mutex_t* mutex )
{
    UNUSED_PARAMETER(mutex);
    return kNoErr;

}

OSStatus mico_rtos_deinit_mutex( mico_mutex_t* mutex )
{
    UNUSED_PARAMETER(mutex);
    return kNoErr;    
}

extern uint32_t mico_get_time_no_os(void);
uint32_t mico_get_time(void)
{
  return mico_get_time_no_os( );
}

void mico_thread_msleep(uint32_t milliseconds)
{
  int tick_delay_start = mico_get_time();
  while(mico_get_time() < tick_delay_start+milliseconds);
}

