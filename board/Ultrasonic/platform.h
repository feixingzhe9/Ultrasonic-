/**
******************************************************************************
* @file    platform.h
* @author  William Xu
* @version V1.0.0
* @date    05-May-2014
* @brief   This file provides all MICO Peripherals defined for current platform.
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



#pragma once

#ifdef __cplusplus
extern "C"
{
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

/*
POWERBOARD V1.2 platform pin definitions ...
+-------------------------------------------------------------------------+
| Enum ID       |Pin | STM32| Peripheral  |    Board     |   Peripheral   |
|               | #  | Port | Available   |  Connection  |     Alias      |
|---------------+----+------+-------------+--------------+----------------|
|               | 1  | VCC  |             |              |                |
+---------------+----+--------------------+--------------+----------------+
|               | 2  | C 13 |             |              |                |
+---------------+----+--------------------+--------------+----------------+
|               | 3  | C 14 |             |              |                |
+---------------+----+--------------------+--------------+----------------+
|               | 4  | C 15 |             |              |                |
+---------------+----+--------------------+--------------+----------------+
|               | 5  | D  0 |             |              |                |
+---------------+----+--------------------+--------------+----------------+
|               | 6  | D  1 |             |              |                |
+---------------+----+--------------------+--------------+----------------+
|               | 7  | NRST |             |              |                |
+---------------+----+--------------------+--------------+----------------+
| MICO_UART1_TX | 29 | B 10 | USART3_TX   |              | MICO_UART_3_TX |
|               |    |      | GPIO        |              |                |
|               |    |      | I2C2_SCL    |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MICO_UART1_RX | 30 | B 11 | USART3_RX   |              | MICO_UART_3_RX |
|               |    |      | GPIO        |              |                |
|               |    |      | I2C2_SDA    |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MICO_UART2_TX | 42 | A  9 | USART1_TX   |              | MICO_UART_1_TX |
|               |    |      | GPIO        |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MICO_UART2_RX | 43 | B 10 | USART1_RX   |              | MICO_UART_1_RX |
|               |    |      | GPIO        |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MICO_SYS_LED  | 40 | C  9 | GPIO        |              |                |
+---------------+----+--------------------+--------------+----------------+
| MICO_SWITCH   | 37 | C  6 | GPIO        |              |                |
+---------------+----+--------------------+--------------+----------------+
| MICO_GPIO_1   | 8  | C  0 | ADC12_IN10  |              |  ADC_2.1_PA    |
|               |    |      | GPIO        |              |                |
|---------------+----+------+-------------+--------------+----------------|
| MICO_GPIO_2   | 9  | C  1 | ADC12_IN11  |              |     ADC_PAD    |
|               |    |      | GPIO        |              |                |
|               |    |      |             |              |                |
+---------------+----+------+-------------+--------------+----------------+
*
*/
  
#define MICO_UNUSED 0xFF

typedef enum
{
#if 1
    
    MICO_GPIO_ULTRA_DATA,
    MICO_GPIO_24_V_EN,
    MICO_GPIO_SYS_LED,
    
    MICO_GPIO_KEY_S0,
    MICO_GPIO_KEY_S1,
    MICO_GPIO_KEY_S2,
    MICO_GPIO_KEY_S3,
    MICO_GPIO_KEY_S4,
    MICO_GPIO_KEY_S5,
    
    
    MICO_GPIO_PWRKEY,
    MICO_GPIO_5V_RES1_ADC,
    MICO_GPIO_12V_RES2_ADC,
  
    MICO_GPIO_UART3_TX,
    MICO_GPIO_UART3_RX,
  
    MICO_GPIO_LED_PWM,
    MICO_GPIO_ADAPTER_IN,
    MICO_GPIO_FAN_CTRL,

    MICO_GPIO_RECHARGE_LED,    
    MICO_GPIO_IRLED_PWM,
    MICO_GPIO_CAN_RX,
    MICO_GPIO_CAN_TX,
    MICO_GPIO_CAN_STB,
  
    MICO_GPIO_SPI_NSS,
    MICO_GPIO_SPI_SCK,
    MICO_GPIO_SPI_MISO,
    MICO_GPIO_SPI_MOSI,
  
    MICO_GPIO_I2C_SCL,
    MICO_GPIO_I2C_SDA,
  
    MICO_GPIO_UART2_TX,
    MICO_GPIO_UART2_RX,
  
    MICO_GPIO_UART1_TX,
    MICO_GPIO_UART1_RX,
  
    MICO_GPIO_ID1,
    MICO_GPIO_ID2,
  
#endif
    MICO_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
    MICO_GPIO_NONE,
} mico_gpio_t;

typedef enum
{
    IO_INPUT,
    IO_OUTPUT,
    IO_DIR_MAX,
} io_dir_t;
typedef enum
{
    MICO_SPI_1,
    MICO_SPI_MAX, /* Denotes the total number of SPI port aliases. Not a valid SPI alias */
    MICO_SPI_NONE,
} mico_spi_t;


typedef enum
{
    MICO_I2C_1,
    MICO_I2C_MAX, /* Denotes the total number of I2C port aliases. Not a valid I2C alias */
    MICO_I2C_NONE,
} mico_i2c_t;

typedef enum
{
    MICO_PWM_IRLED,
    MICO_PWM_2,
    MICO_PWM_3,
    MICO_PWM_MAX, /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
    MICO_PWM_NONE,
} mico_pwm_t;

typedef enum
{
    /* following are adc1 channels */
    MICO_ADC_5V_RES1,
    MICO_ADC_12V_RES2,
    MICO_ADC_BAT_NV,

    MICO_ADC_NONE,
} mico_adc_t;

typedef enum
{
    MICO_UART_1,
    MICO_UART_2,
    MICO_UART_3,
    MICO_UART_MAX, /* Denotes the total number of UART port aliases. Not a valid UART alias */
    MICO_UART_NONE,
} mico_uart_t;

typedef enum
{
  MICO_FLASH_EMBEDDED,
  MICO_FLASH_MAX,
  MICO_FLASH_NONE,
} mico_flash_t;

typedef enum
{
  MICO_PARTITION_USER_MAX
} mico_user_partition_t;

typedef enum 
{
    MICO_CAN1,
    MICO_CAN_MAX,
    MICO_CAN_NONE,
} mico_can_t;


#define STDIO_UART          MICO_UART_1
#define STDIO_UART_BAUDRATE (115200) 

#define COMM_UART        MICO_UART_1
#define COMM_UART_BAUDRATE (115200) 
#define UART_FOR_APP     MICO_UART_1

#define USER_TIM_MAX_CNT    65535

#ifdef __cplusplus
} /*extern "C" */
#endif

