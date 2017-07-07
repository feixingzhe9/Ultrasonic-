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
    MICO_GPIO_SYS_LED,
    MICO_GPIO_SWITCH,
    MICO_GPIO_UART1_TX,
    MICO_GPIO_UART1_RX,
    MICO_GPIO_UART2_TX,
    MICO_GPIO_UART2_RX,
    MICO_GPIO_MAX, /* Denotes the total number of GPIO port aliases. Not a valid GPIO alias */
    MICO_GPIO_NONE,
} mico_gpio_t;

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
    MICO_PWM_1,
    MICO_PWM_2,
    MICO_PWM_3,
    MICO_PWM_MAX, /* Denotes the total number of PWM port aliases. Not a valid PWM alias */
    MICO_PWM_NONE,
} mico_pwm_t;

typedef enum
{
    MICO_ADC_1,
    MICO_ADC_2,
    MICO_ADC_3,
    MICO_ADC_MAX, /* Denotes the total number of ADC port aliases. Not a valid ADC alias */
    MICO_ADC_NONE,
} mico_adc_t;

typedef enum
{
    MICO_UART_1,
    MICO_UART_2,
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

#ifdef BOOTLOADER
#define STDIO_UART          MICO_UART_1
#define STDIO_UART_BAUDRATE (115200) 
#else
#define STDIO_UART          MICO_UART_1
#define STDIO_UART_BAUDRATE (115200) 
#endif

#define UART_FOR_APP     MICO_UART_2
#define MFG_TEST         MICO_UART_1
#define CLI_UART         MICO_UART_1

/* Components connected to external I/Os*/
#define Standby_SEL      (MICO_GPIO_29)

/* I/O connection <-> Peripheral Connections */
#define MICO_I2C_CP      (MICO_I2C_1)


#ifdef __cplusplus
} /*extern "C" */
#endif

