/**
******************************************************************************
* @file    platform.c 
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file provides all MICO Peripherals mapping table and platform
*          specific functions.
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

#include "stdio.h"
#include "string.h"

#include "platform.h"
#include "platform_config.h"
#include "platform_peripheral.h"
#include "platform_internal.h"
#include "platform_logging.h"
#include "mico_platform.h"
//#include "keypad/gpio_button/button.h"
#ifndef BOOTLOADER
#include "app_platform.h"
#endif
/******************************************************
*                      Macros
******************************************************/

/******************************************************
*                    Constants
******************************************************/

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
*               Function Declarations
******************************************************/
extern WEAK void PlatformEasyLinkButtonClickedCallback(void);
extern WEAK void PlatformStandbyButtonClickedCallback(void);
extern WEAK void PlatformEasyLinkButtonLongPressedCallback(void);
extern WEAK void bootloader_start(void);

/******************************************************
*               Variables Definitions
******************************************************/

const platform_gpio_t platform_gpio_pins[] =
{

    [MICO_GPIO_ULTRA_DATA]        = { GPIOB,  13 },
    
    [MICO_GPIO_24_V_EN]           = { GPIOA,  0 },
    //[MICO_GPIO_SYS_LED]           = { GPIOC,  9 },//{ GPIOC,  11 },//{ GPIOB,  3 },
    [MICO_GPIO_SYS_LED]           = { GPIOB,  3 },
    [MICO_GPIO_KEY_S0]            = { GPIOA,  2 },
    [MICO_GPIO_KEY_S1]            = { GPIOA,  3 },
    [MICO_GPIO_KEY_S2]            = { GPIOA,  4 },
    [MICO_GPIO_KEY_S3]            = { GPIOA,  5 },
    [MICO_GPIO_KEY_S4]            = { GPIOA,  6 },
    //[MICO_GPIO_KEY_S5]            = { GPIOA,  7 },
    
    [MICO_GPIO_TRIG]              = { GPIOA,  7 },

    [MICO_GPIO_UART3_TX]          = { GPIOB, 10 },
    [MICO_GPIO_UART3_RX]          = { GPIOB, 11 },

    [MICO_GPIO_LED_PWM]           = { GPIOC, 6  },//{ GPIOB, 12 },
    [MICO_GPIO_ADAPTER_IN]        = { GPIOD, 10 },
    [MICO_GPIO_FAN_CTRL]          = { GPIOD, 11 },

    [MICO_GPIO_RECHARGE_LED]      = { GPIOC,  8 }, 
    [MICO_GPIO_IRLED_PWM]         = { GPIOA,  8 },
    [MICO_GPIO_CAN_RX]            = { GPIOA, 11 },
    [MICO_GPIO_CAN_TX]            = { GPIOA, 12 },
    [MICO_GPIO_CAN_STB]           = { GPIOD,  3 },

    [MICO_GPIO_SPI_NSS]           = { GPIOA, 15 },
    //[MICO_GPIO_SPI_SCK]           = { GPIOB,  3 },
    [MICO_GPIO_SPI_MISO]          = { GPIOB,  4 },
    [MICO_GPIO_SPI_MOSI]          = { GPIOB,  5 },

    [MICO_GPIO_I2C_SCL]           = { GPIOB,  6 },
    [MICO_GPIO_I2C_SDA]           = { GPIOB,  7 },

    [MICO_GPIO_UART2_TX]          = { GPIOD,  5 },
    [MICO_GPIO_UART2_RX]          = { GPIOD,  6 },

    [MICO_GPIO_UART1_TX]          = { GPIOA,  9 },
    [MICO_GPIO_UART1_RX]          = { GPIOA, 10 },
    
    [MICO_GPIO_EMG_STOP]          = { GPIOA, 8  },
    
    
};

/*
* Possible compile time inputs:
* - Set which ADC peripheral to use for each ADC. All on one ADC allows sequential conversion on all inputs. All on separate ADCs allows concurrent conversion.
*/

/* TODO : These need fixing */
ADC_HandleTypeDef adc_handles[2];

const platform_adc_t platform_adc_peripherals[] =
{
  [MICO_ADC_5V_RES1]       = {ADC1, ADC_CHANNEL_0, &adc_handles[0], 1, &platform_gpio_pins[MICO_GPIO_5V_RES1_ADC]},
  [MICO_ADC_12V_RES2]      = {ADC1, ADC_CHANNEL_1, &adc_handles[0], 2, &platform_gpio_pins[MICO_GPIO_12V_RES2_ADC]}
};

TIM_HandleTypeDef tim_handles[1];
/* PWM mappings */
const platform_pwm_t platform_pwm_peripherals[] =
{  
  [MICO_PWM_IRLED] = {TIM1, &tim_handles[0], TIM_CHANNEL_1, RCC_APB2ENR_TIM1EN, NULL, &platform_gpio_pins[MICO_GPIO_IRLED_PWM]},    /* or TIM10/Ch1                       */

  /* TODO: fill in the other options here ... */
};
#if 0
const platform_spi_t platform_spi_peripherals[] =
{
  [MICO_SPI_1]  =
  {
    .port                  = SPI1,
    .gpio_af               = GPIO_AF_SPI1,
    .peripheral_clock_reg  = RCC_APB2Periph_SPI1,
    .peripheral_clock_func = RCC_APB2PeriphClockCmd,
    .pin_mosi              = &platform_gpio_pins[MICO_GPIO_9],
    .pin_miso              = &platform_gpio_pins[MICO_GPIO_8],
    .pin_clock             = &platform_gpio_pins[MICO_GPIO_7],
    .tx_dma =
    {
      .controller          = DMA2,
      .stream              = DMA2_Stream5,
      .channel             = DMA_Channel_3,
      .irq_vector          = DMA2_Stream5_IRQn,
      .complete_flags      = DMA_HISR_TCIF5,
      .error_flags         = ( DMA_HISR_TEIF5 | DMA_HISR_FEIF5 | DMA_HISR_DMEIF5 ),
    },
    .rx_dma =
    {
      .controller          = DMA2,
      .stream              = DMA2_Stream0,
      .channel             = DMA_Channel_3,
      .irq_vector          = DMA2_Stream0_IRQn,
      .complete_flags      = DMA_LISR_TCIF0,
      .error_flags         = ( DMA_LISR_TEIF0 | DMA_LISR_FEIF0 | DMA_LISR_DMEIF0 ),
    },
  }
};

platform_spi_driver_t platform_spi_drivers[MICO_SPI_MAX];
#endif

const platform_uart_t platform_uart_peripherals[] =
{
  [MICO_UART_1] =
  {
    .port                         = USART1,
    .pin_tx                       = &platform_gpio_pins[MICO_GPIO_UART1_TX],
    .pin_rx                       = &platform_gpio_pins[MICO_GPIO_UART1_RX],
    .pin_cts                      = NULL,
    .pin_rts                      = NULL,
    .tx_dma_config =
    {
      .controller                 = DMA1,
      .channel                    = DMA1_Channel4,
      .irq_vector                 = DMA1_Channel4_IRQn,
      .complete_flags             = DMA_ISR_TCIF4,
      .error_flags                = DMA_ISR_TEIF4,
    },
    .rx_dma_config =
    {
      .controller                 = DMA1,
      .channel                    = DMA1_Channel5,
      .irq_vector                 = DMA1_Channel5_IRQn,
      .complete_flags             = DMA_ISR_TCIF5,
      .error_flags                = DMA_ISR_TEIF5,
    },
  },
  [MICO_UART_2] =
  {
    .port                         = USART2,
    .pin_tx                       = &platform_gpio_pins[MICO_GPIO_UART2_TX],
    .pin_rx                       = &platform_gpio_pins[MICO_GPIO_UART2_RX],
    .pin_cts                      = NULL,
    .pin_rts                      = NULL,
    .tx_dma_config =
    {
      .controller                 = DMA1,
      .channel                    = DMA1_Channel7,
      .irq_vector                 = DMA1_Channel7_IRQn,
      .complete_flags             = DMA_ISR_TCIF7,
      .error_flags                = DMA_ISR_TEIF7,
    },
    .rx_dma_config =
    {
      .controller                 = DMA1,
      .channel                    = DMA1_Channel6,
      .irq_vector                 = DMA1_Channel6_IRQn,
      .complete_flags             = DMA_ISR_TCIF6,
      .error_flags                = DMA_ISR_TEIF6,
    },
  },
};
static UART_HandleTypeDef UartHandle[3];
static DMA_HandleTypeDef  uart_tx_dmaHandle[3];
static DMA_HandleTypeDef  uart_rx_dmaHandle[3];
//platform_uart_driver_t platform_uart_drivers[MICO_UART_MAX];
platform_uart_driver_t platform_uart_drivers[] = 
{
  [MICO_UART_1] = 
  {
    .uart_handle = &UartHandle[0],
    .rx_dma_handle = &uart_rx_dmaHandle[0],
    .tx_dma_handle = &uart_tx_dmaHandle[0],
  },
  [MICO_UART_2] = 
  {
    .uart_handle = &UartHandle[1],
    .rx_dma_handle = &uart_rx_dmaHandle[1],
    .tx_dma_handle = &uart_tx_dmaHandle[1],
  }
};
#if 0
const platform_i2c_t platform_i2c_peripherals[] =
{
  [MICO_I2C_1] =
  {
    .port                    = I2C1,
    .pin_scl                 = &platform_gpio_pins[MICO_GPIO_1],
    .pin_sda                 = &platform_gpio_pins[MICO_GPIO_2],
    .peripheral_clock_reg    = RCC_APB1Periph_I2C1,
    .tx_dma                  = DMA1,
    .tx_dma_peripheral_clock = RCC_AHB1Periph_DMA1,
    .tx_dma_stream           = DMA1_Stream7,
    .rx_dma_stream           = DMA1_Stream0,
    .tx_dma_stream_id        = 7,
    .rx_dma_stream_id        = 0,
    .tx_dma_channel          = DMA_Channel_1,
    .rx_dma_channel          = DMA_Channel_1,
    .gpio_af                 = GPIO_AF_I2C1
  },
};

platform_i2c_driver_t platform_i2c_drivers[MICO_I2C_MAX];
#endif
//platform_flash_driver_t platform_flash_drivers[MICO_FLASH_MAX];

/* Flash memory devices */
const platform_flash_t platform_flash_peripherals[] =
{
  [MICO_FLASH_EMBEDDED] =
  {
    .flash_type                   = FLASH_TYPE_EMBEDDED,
    .flash_start_addr             = 0x08000000,
    .flash_length                 = 0x80000,
  },
};

platform_flash_driver_t platform_flash_drivers[MICO_FLASH_MAX];

/* Logic partition on flash devices */
const mico_logic_partition_t mico_partitions[] =
{
  [MICO_PARTITION_BOOTLOADER] =
  {
    .partition_owner           = MICO_FLASH_EMBEDDED,
    .partition_description     = "Bootloader",
    .partition_start_addr      = 0x08000000,
    .partition_length          = 0x4000,    //16k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
  },
  [MICO_PARTITION_APPLICATION] =
  {
    .partition_owner           = MICO_FLASH_EMBEDDED,
    .partition_description     = "Application",
    .partition_start_addr      = 0x08005000, 
    .partition_length          = 0xD000,   //52k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_DIS,
  },
  [MICO_PARTITION_OTA_TEMP] =
  {
    .partition_owner           = MICO_FLASH_EMBEDDED,
    .partition_description     = "OTA Storage",
    .partition_start_addr      = 0x08013000,
    .partition_length          = 0xD000, //52k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MICO_PARTITION_PARAMETER_1] =
  {
    .partition_owner           = MICO_FLASH_EMBEDDED,
    .partition_description     = "PARAMETER1",
    .partition_start_addr      = 0x08004000,
    .partition_length          = 0x1000, // 4k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
  [MICO_PARTITION_PARAMETER_2] =
  {
    .partition_owner           = MICO_FLASH_EMBEDDED,
    .partition_description     = "PARAMETER1",
    .partition_start_addr      = 0x08012000,
    .partition_length          = 0x1000, //4k bytes
    .partition_options         = PAR_OPT_READ_EN | PAR_OPT_WRITE_EN,
  },
};

#if 1//defined (USE_CAN)
CanTxMsgTypeDef         can_tx_msg;
CanRxMsgTypeDef         can_rx_msg;
CAN_HandleTypeDef       can_handle = {
  .pTxMsg      = &can_tx_msg,
  .pRxMsg      = &can_rx_msg,
};

platform_can_driver_t  platform_can_drivers[] = 
{
  [MICO_CAN1] = 
  {
    .port                       = CAN1,
    .handle                     = &can_handle,
    .pin_tx                     = &platform_gpio_pins[MICO_GPIO_CAN_TX],
    .pin_rx                     = &platform_gpio_pins[MICO_GPIO_CAN_RX],
  },
};
#endif
/******************************************************
*           Interrupt Handler Definitions
******************************************************/
MICO_RTOS_DEFINE_ISR( SysTick_Handler )
{
  sysTickHandler();
}

MICO_RTOS_DEFINE_ISR( USART3_IRQHandler )
{
  platform_uart_irq( &platform_uart_drivers[MICO_UART_3] );
}
extern UART_HandleTypeDef huart1;
char uart_data_buf[14];
static uint8_t rcv_data_cnt = 0;
static uint8_t is_frame_start = 0;
static uint8_t is_frame_end = 0;
uint32_t rcv_cnt_test = 0;
uint32_t distance_test = 0;
#include "StringUtils.h"
#include "platform_tim.h"
#include "UltraSonic.h"
const char frame_head[] = "n0.val=";
MICO_RTOS_DEFINE_ISR( USART1_IRQHandler )
{
    uint8_t rcv_data = huart1.Instance->DR;
    rcv_cnt_test++;
    if((ultra_sonic_data->start_flag == 1) && (ultra_sonic_data->end_flag == 0))
    {
        if(rcv_data == 'n')
        {
            is_frame_start = 1;
            is_frame_end = 0;
            rcv_data_cnt = 0;
        }

        if((is_frame_start == 1) && (is_frame_end == 0))
        {
            uart_data_buf[rcv_data_cnt] = rcv_data;
            if(rcv_data_cnt < sizeof(uart_data_buf)/sizeof(uart_data_buf[0]))
            {
                rcv_data_cnt++;
            }
        }
        else
        {
            rcv_data_cnt = 0;
        }
        if(rcv_data == 0xff)
        {
            is_frame_end = 1;
            is_frame_start = 0;
            if(strncmp(&uart_data_buf[0],frame_head, 7) == 0)
            {
                if(rcv_data_cnt == 10)
                {
                    string_to_unsigned( &uart_data_buf[7],2,&distance_test,0);
                }
                else if(rcv_data_cnt == 11)
                {
                    string_to_unsigned( &uart_data_buf[7],3,&distance_test,0);
                }
                else if(rcv_data_cnt == 12)
                {
                    string_to_unsigned( &uart_data_buf[7],4,&distance_test,0);
                }
            }
            rcv_data_cnt = 0;
        } 
    }
    
}

MICO_RTOS_DEFINE_ISR( USART2_IRQHandler )
{
  platform_uart_irq( &platform_uart_drivers[MICO_UART_2] );
}

MICO_RTOS_DEFINE_ISR( DMA1_Channel2_IRQHandler )
{
  platform_uart_tx_dma_irq( &platform_uart_drivers[MICO_UART_3] );
}

MICO_RTOS_DEFINE_ISR( DMA1_Channel4_IRQHandler )
{
  platform_uart_tx_dma_irq( &platform_uart_drivers[MICO_UART_1] );
}

MICO_RTOS_DEFINE_ISR( DMA1_Channel7_IRQHandler )
{
  platform_uart_tx_dma_irq( &platform_uart_drivers[MICO_UART_2] );
}

MICO_RTOS_DEFINE_ISR( DMA1_Channel3_IRQHandler )
{
  platform_uart_rx_dma_irq( &platform_uart_drivers[MICO_UART_3] );
}

MICO_RTOS_DEFINE_ISR( DMA1_Channel5_IRQHandler )
{
  platform_uart_rx_dma_irq( &platform_uart_drivers[MICO_UART_1] );
}

MICO_RTOS_DEFINE_ISR( DMA1_Channel6_IRQHandler )
{
  platform_uart_rx_dma_irq( &platform_uart_drivers[MICO_UART_2] );
}

MICO_RTOS_DEFINE_ISR( DMA1_Channel1_IRQHandler )
{
//  platform_adc_dma_irq( &platform_adc_peripherals[MICO_ADC_5V_RES1] );
  __asm("NOP");
}

MICO_RTOS_DEFINE_ISR( DMA2_Channel4_5_IRQHandler )
{
   __asm("NOP");
}

MICO_RTOS_DEFINE_ISR( USB_LP_CAN1_RX0_IRQHandler )
{
  platform_can_rx_irq( &platform_can_drivers[MICO_CAN1] );
}

/******************************************************
*               Function Definitions
******************************************************/



bool watchdog_check_last_reset( void )
{
  if ( RCC->CSR & RCC_CSR_IWDGRSTF )
  {
    /* Clear the flag and return */
    RCC->CSR |= RCC_CSR_RMVF;
    return true;
  }
  
  return false;
}

void platform_init_peripheral_irq_priorities( void )
{
  /* Interrupt priority setup. Called by MiCO/platform/MCU/STM32F2xx/platform_init.c */
  NVIC_SetPriority( RTC_IRQn         ,  1 ); /* RTC Wake-up event   */
  
  NVIC_SetPriority( USART1_IRQn      ,  6 ); /* MICO_UART_1         */
  NVIC_SetPriority( USART2_IRQn      ,  6 ); /* MICO_UART_2         */
//  NVIC_SetPriority( USART3_IRQn      ,  6 ); /* MICO_UART_3         */
  NVIC_SetPriority( DMA1_Channel4_IRQn,  7 ); /* MICO_UART_1 TX DMA  */
  NVIC_SetPriority( DMA1_Channel5_IRQn,  7 ); /* MICO_UART_1 RX DMA  */
  NVIC_SetPriority( DMA1_Channel2_IRQn,  7 ); /* MICO_UART_2 TX DMA  */
  NVIC_SetPriority( DMA1_Channel3_IRQn,  7 ); /* MICO_UART_2 RX DMA  */
 // NVIC_SetPriority( DMA1_Channel6_IRQn,  7 ); /* MICO_UART_3 TX DMA  */
 // NVIC_SetPriority( DMA1_Channel7_IRQn,  7 ); /* MICO_UART_3 RX DMA  */
  NVIC_SetPriority( DMA1_Channel1_IRQn,  10 ); /* ADC DMA  */
  NVIC_SetPriority( EXTI0_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI1_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI2_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI3_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI4_IRQn       , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI9_5_IRQn     , 14 ); /* GPIO                */
  NVIC_SetPriority( EXTI15_10_IRQn   , 14 ); /* GPIO                */
  
  NVIC_SetPriority( USB_LP_CAN1_RX0_IRQn,  7 ); /* MICO_CAN1         */
}

void init_platform( void )
{ 

  
  
  //AFIO_MAPR_SWJ_CFG_JTAGDISABLE
    
  
#if 0  
  //  Initialise switch
  pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
  pin_config.gpio_mode = GPIO_MODE_IT_RISING_FALLING;
  pin_config.gpio_pull = GPIO_PULLDOWN;
  MicoGpioInitialize( MICO_GPIO_PWRKEY, &pin_config );
  MicoGpioEnableIRQ( MICO_GPIO_PWRKEY , IRQ_TRIGGER_BOTH_EDGES, _switch_irq_handler, NULL);
#endif
}

void init_platform_bootloader( void )
{
  platform_pin_config_t pin_config;
  pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
  pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
  pin_config.gpio_pull = GPIO_PULLUP;
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_SYS_LED, &pin_config );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SYS_LED );
}

#if 0
void MicoSysLed(bool onoff)
{
  if (onoff) {
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SYS_LED );
  } else {
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SYS_LED );
  }
}
#endif

bool MicoShouldEnterBootloader(void)
{
  if( 1 )//MicoGpioInputGet((mico_gpio_t)BOOT_SEL)==false && MicoGpioInputGet((mico_gpio_t)MFG_SEL)==true)
    return true;
  else
    return false;
}

#if 0
void Set_IO_Direction(mico_gpio_t gpio, io_dir_t b)
{
    platform_pin_config_t pin_config;
#if 0
    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    
    if(b == IO_INPUT)
    {
        pin_config.gpio_mode = GPIO_MODE_INPUT;
    } 
    if(b == IO_OUTPUT)
    {
        pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
        pin_config.gpio_pull = GPIO_PULLUP;       
    }
    MicoGpioInitialize( (mico_gpio_t)gpio, &pin_config );
    if(b == IO_OUTPUT)
    {
        MicoGpioOutputHigh( (mico_gpio_t)gpio ); 
    }  
#endif
    if(b == IO_OUTPUT)
    {

        pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
        pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
        pin_config.gpio_pull = GPIO_PULLUP;

        MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_ULTRA_DATA, &pin_config );
        MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_ULTRA_DATA ); 
    }
    if(b == IO_INPUT)
    { 
        pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
        pin_config.gpio_mode = GPIO_MODE_INPUT;
        
        MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_ULTRA_DATA, &pin_config ); 
        MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_ULTRA_DATA ); 
    }
    
    
}
#endif

extern uint32_t time_cnt_test;
#define UltraSonicLog(format, ...)  custom_log("UlSo", format, ##__VA_ARGS__)

static void UltraSonicIRQ_CallBack(void* arg )
{

    
}

void UltraDataIO_Input(void)//interrupt mode 
{
    platform_pin_config_t pin_config;
    
    DISABLE_INTERRUPTS();
    
    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode =GPIO_MODE_INPUT;
    pin_config.gpio_pull = GPIO_PULLUP;
    MicoGpioInitialize( MICO_GPIO_ULTRA_DATA, &pin_config );
    
    ENABLE_INTERRUPTS();
}


void UltraDataIO_InputIT(void)//interrupt mode 
{
    platform_pin_config_t pin_config;
    
    DISABLE_INTERRUPTS();
    
    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode =GPIO_MODE_IT_FALLING |GPIO_MODE_INPUT;
    pin_config.gpio_pull = GPIO_PULLUP;//GPIO_PULLUP;
    //MicoGpioOutputHigh(MICO_GPIO_ULTRA_DATA); 
    MicoGpioInitialize( MICO_GPIO_ULTRA_DATA, &pin_config );
    MicoGpioEnableIRQ( MICO_GPIO_ULTRA_DATA , IRQ_TRIGGER_FALLING_EDGE, UltraSonicIRQ_CallBack, NULL);
    
    
    ENABLE_INTERRUPTS();
}

void UltraDataIO_Output(void)//output high
{
    platform_pin_config_t pin_config;
    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
    pin_config.gpio_pull = GPIO_PULLUP;

    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_ULTRA_DATA, &pin_config );
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_ULTRA_DATA ); 
    
    MicoGpioDisableIRQ(MICO_GPIO_ULTRA_DATA);

}

void V24OutputHigh(void)
{
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_24_V_EN ); 
}


void UltraTrigOutputHigh(void)
{
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_TRIG ); 
}
void UltraTrigOutputLow(void)
{
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_TRIG ); 
}

void V24OutputLow(void)
{
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_24_V_EN ); 
}

void UltraIoOutputLow(void)
{
    MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_ULTRA_DATA ); 
}

void UltraIoOutputHigh(void)
{
    MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_ULTRA_DATA ); 
}

void SysLedTrigger(void)
{
    MicoGpioOutputTrigger(MICO_GPIO_SYS_LED);
}

uint8_t GetKeyValue(mico_gpio_t gpio)
{
    if((gpio >= MICO_GPIO_KEY_S0) && (gpio <= MICO_GPIO_KEY_S5))
    {
        return MicoGpioInputGet(gpio);
    }
    return 0xff;   
}