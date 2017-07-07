#pragma once

/******************************************************
*                      Macros
******************************************************/

/******************************************************
*                    Constants
******************************************************/

#define HARDWARE_REVISION   "PM_V1.2"
#define DEFAULT_NAME        "POWERBOARD"
#define MODEL               "POWERBOARD"

/* MICO RTOS tick rate in Hz */
#define NBOS_DEFAULT_TICK_RATE_HZ                   (1000) 

/************************************************************************
 * Uncomment to disable watchdog. For debugging only */
//#define MICO_DISABLE_WATCHDOG

/************************************************************************
 * Uncomment to disable standard IO, i.e. printf(), etc. */
//#define MICO_DISABLE_STDIO

/************************************************************************
 * Uncomment to disable MCU powersave API functions */
//#define MICO_DISABLE_MCU_POWERSAVE

/************************************************************************
 * Uncomment to enable MCU real time clock */
#define NBOS_ENABLE_MCU_RTC

#if 0
#define HSE_SOURCE              RCC_HSE_ON               /* Use external crystal                 */
#define AHB_CLOCK_DIVIDER       RCC_SYSCLK_Div1          /* AHB clock = System clock             */
#define APB1_CLOCK_DIVIDER      RCC_HCLK_Div4            /* APB1 clock = AHB clock / 4           */
#define APB2_CLOCK_DIVIDER      RCC_HCLK_Div2            /* APB2 clock = AHB clock / 2           */
#define PLL_SOURCE              RCC_PLLSource_HSE        /* PLL source = external crystal        */
#define PLL_M_CONSTANT          26                       /* PLLM = 26                            */
#define PLL_N_CONSTANT          240                      /* PLLN = 240                           */
#define PLL_P_CONSTANT          2                        /* PLLP = 2                             */
#define PPL_Q_CONSTANT          5                        /* PLLQ = 5                             */
#define SYSTEM_CLOCK_SOURCE     RCC_SYSCLKSource_PLLCLK  /* System clock source = PLL clock      */
#define SYSTICK_CLOCK_SOURCE    SysTick_CLKSource_HCLK   /* SysTick clock source = AHB clock     */
#define INT_FLASH_WAIT_STATE    FLASH_Latency_3          /* Internal flash wait state = 3 cycles */
#endif
/* The number of UART interfaces this hardware platform has */
#define NUMBER_OF_UART_INTERFACES  2

//#define STDIO_UART       MICO_UART_2

/* Define the address from where user application will be loaded.
Note: the 1st sector 0x08000000-0x08003FFF is reserved for the IAP code */
#define INTERNAL_FLASH_START_ADDRESS   (uint32_t)0x08000000
#define INTERNAL_FLASH_END_ADDRESS     (uint32_t)0x0801FFFF
#define INTERNAL_FLASH_SIZE            (INTERNAL_FLASH_END_ADDRESS - INTERNAL_FLASH_START_ADDRESS + 1)

#define NBOS_FLASH_FOR_BOOT         NBOS_INTERNAL_FLASH
#define BOOT_START_ADDRESS          (uint32_t)0x0800000 
#define BOOT_END_ADDRESS            (uint32_t)0x08004FFF 
#define BOOT_FLASH_SIZE             (BOOT_END_ADDRESS - BOOT_START_ADDRESS + 1)

#define NBOS_FLASH_FOR_PARA         NBOS_INTERNAL_FLASH
#define PARA_START_ADDRESS          (uint32_t)0x08005000 
#define PARA_END_ADDRESS            (uint32_t)0x08005FFF
#define PARA_FLASH_SIZE             (PARA_END_ADDRESS - PARA_START_ADDRESS + 1)  

#define NBOS_FLASH_FOR_APPLICATION  NBOS_INTERNAL_FLASH
#define APPLICATION_START_ADDRESS   (uint32_t)0x08006000
#define APPLICATION_END_ADDRESS     (uint32_t)0x08012FFF
#define APPLICATION_FLASH_SIZE      (APPLICATION_END_ADDRESS - APPLICATION_START_ADDRESS + 1) /* 53.247k bytes */

#define NBOS_FLASH_FOR_UPDATE       NBOS_INTERNAL_FLASH /* Optional */
#define UPDATE_START_ADDRESS        (uint32_t)0x08013000  /* Optional */
#define UPDATE_END_ADDRESS          (uint32_t)0x0801FFFF  /* Optional */
#define UPDATE_FLASH_SIZE           (UPDATE_END_ADDRESS - UPDATE_START_ADDRESS + 1) /* 53.247k bytes, optional*/


