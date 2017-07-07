/**
 ******************************************************************************
 * @file    menu.c
 * @author  William Xu
 * @version V2.0.0
 * @date    05-Oct-2014
 * @brief   his file provides the software which contains the main menu routine.
 *          The main menu gives the options of:
 *             - downloading a new binary file,
 *             - uploading internal flash memory,
 *             - executing the binary file already loaded
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

/* Includes ------------------------------------------------------------------*/
#include "mico.h"
#include "ymodem.h"
#include "platform_config.h"
#include "platform_internal.h"
#include "StringUtils.h"
#include "bootloader.h"
#include <ctype.h>                    
#include "tps611xx_bl.h"
#include "voltage_detect.h"
#include "protocol.h"
#include "upgrade_flash.h"
#include "app_platform.h"
   
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CMD_STRING_SIZE       128
   
#define CNTLQ      0x11
#define CNTLS      0x13
#define DEL        0x7F
#define BACKSPACE  0x08
#define CR         0x0D
#define LF         0x0A
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern platform_flash_t platform_flash_peripherals[];

uint8_t tab_1024[1024] =
{
  0
};

char FileName[FILE_NAME_LENGTH];
char ERROR_STR [] = "\n\r*** ERROR: %s\n\r";    /* ERROR message string in code   */

extern char menu[];
#ifdef NO_BLOCK_MENU
extern int getline (char *line, int n);          /* input line               */
#else
extern void getline (char *line, int n);          /* input line               */
#endif
extern void startApplication( uint32_t app_addr );

/* Private function prototypes -----------------------------------------------*/
void SerialDownload(mico_flash_t flash, uint32_t flashdestination, int32_t maxRecvSize);
void SerialUpload(mico_flash_t flash, uint32_t flashdestination, char * fileName, int32_t maxRecvSize);

/* Private functions ---------------------------------------------------------*/
/**
* @brief  Analyse a command parameter
* @param  commandBody: command string address
* @param  para: The para we are looking for
* @param  paraBody: A pointer to the buffer to receive the para body.
* @param  paraBodyLength: The length, in bytes, of the buffer pointed to by the paraBody parameter.
* @retval the actual length of the paraBody received, -1 means failed to find this paras 
*/
#ifdef NO_BLOCK_MENU
static void uart_putchar( int c )
{
  MicoUartSend( STDIO_UART, &c, 1 );
}
#endif
int findCommandPara(char *commandBody, char *para, char *paraBody, int paraBodyLength)
{
  int i = 0;
  int k, j;
  int retval = -1;
  char para_in_ram[100];
  strncpy(para_in_ram, para, 100);
  
  for (i = 0; para_in_ram[i] != 0; i++)  {                /* convert to upper characters */
    para_in_ram[i] = toupper(para_in_ram[i]);
  }
  
  i = 0;
  while(commandBody[i] != 0) {
    if(commandBody[i] == '-' ){
      for(j=i+1, k=0; *(para_in_ram+k)!=0x0; j++, k++ ){
        if(commandBody[j] != *(para_in_ram+k)){
          break;
        } 
      }
      
      if(*(para+k)!=0x0 || (commandBody[j]!=' '&& commandBody[j]!=0x0)){   /* para not found!             */
        i++;
        continue;
      }
      
      retval = 0;
      for (k = j+1; commandBody[k] == ' '; k++);      /* skip blanks                 */
      for(j = 0; commandBody[k] != ' ' && commandBody[k] != 0 && commandBody[k] != '-'; j++, k++){   /* para body found!             */
        if(paraBody) paraBody[j] = commandBody[k];
        retval ++;
        if( retval == paraBodyLength) goto exit;
      }
      goto exit;
    }
    i++;
  }
  
exit:
  if(paraBody) paraBody[retval] = 0x0;
  return retval;
}

/**
* @brief  Download a file via serial port
* @param  None
* @retval None
*/
void SerialDownload(mico_flash_t flash, uint32_t flashdestination, int32_t maxRecvSize)
{
  char Number[10] = "          ";
  int32_t Size = 0;
  
  printf("Waiting for the file to be sent ... (press 'a' to abort)\r\n");
  Size = Ymodem_Receive( &tab_1024[0], flash, flashdestination, maxRecvSize );
  if (Size > 0)
  {
    printf("\n\n\r Successfully!\n\r\r\n Name: %s", FileName);
    
    Int2Str((uint8_t *)Number, Size);
    printf("\n\r Size: %s Bytes\r\n", Number);
  }
  else if (Size == -1)
  {
    printf("\n\n\rImage size is higher than memory!\n\r");
  }
  else if (Size == -2)
  {
    printf("\n\n\rVerification failed!\r\n");
  }
  else if (Size == -3)
  {
    printf("\r\n\nAborted.\r\n");
  }
  else
  {
    printf("\n\rReceive failed!\r\n");
  }
}

/**
* @brief  Upload a file via serial port.
* @param  None
* @retval None
*/
void SerialUpload(mico_flash_t flash, uint32_t flashdestination, char * fileName, int32_t maxRecvSize)
{
  uint8_t status = 0;
  uint8_t key;
  
  printf("Select Receive File\n\r");
  MicoUartRecv( STDIO_UART, &key, 1, MICO_NEVER_TIMEOUT );
  
  if (key == CRC16)
  {
    /* Transmit the flash image through ymodem protocol */
    status = Ymodem_Transmit(flash, flashdestination, (uint8_t *)fileName, maxRecvSize);
    
    if (status != 0)
    {
      printf("\n\rError while Transmitting\n\r");
    }
    else
    {
      printf("\n\rSuccessfully\n\r");
    }
  }
}
#ifdef NO_BLOCK_MENU
static char *pCmdBuf = NULL;
static int cnt = 0;
int getline (char *line, int n)
{
  char c;
  
  pCmdBuf = line + cnt;
  if( cnt >= (CMD_STRING_SIZE-2) )
  {
    goto exit;
  }
  if ( MicoUartRecv( STDIO_UART, &c, 1, 0 ) != kNoErr )
  {
      return -1;
  }
  if (c == CR)      /* read character                 */
  {
    c = LF;
    goto exit;
  }
  if (c == BACKSPACE  ||  c == DEL)  {    /* process backspace              */
    if (cnt != 0)  {
      cnt--;                              /* decrement count                */
      pCmdBuf--;                             /* and line pointer               */
      uart_putchar (BACKSPACE);                /* echo backspace                 */
      uart_putchar (' ');
      uart_putchar (BACKSPACE);
    }
    return -1;
  }
  else if (c != CNTLQ && c != CNTLS)  {   /* ignore Control S/Q             */
    uart_putchar (*pCmdBuf = c);                  /* echo and store character       */
    pCmdBuf++;                               /* increment line pointer         */
    cnt++;                                /* and count                      */
    return -1;
  }
exit:
  *(pCmdBuf) = 0;                          /* mark end of string             */ 
  return 0;
}
#endif
/**
* @brief  Display the Main Menu on HyperTerminal
* @param  None
* @retval None
*/
void Main_Menu(void)
{
  static char cmdbuf [CMD_STRING_SIZE] = {0}; 
  char cmdname[15] = {0};     /* command input buffer        */
  int i, j;                                       /* index for command buffer    */
  char idStr[4], startAddressStr[10], endAddressStr[10], flash_dev_str[4];
  int32_t id, startAddress, endAddress;
  bool inputFlashArea = false;
  mico_logic_partition_t *partition;
  mico_flash_t flash_dev;
  OSStatus err = kNoErr;
  
  while (1)  {                                    /* loop forever                */   
#ifndef NO_BLOCK_MENU
    printf ("\n\rPowerBoard> ");
#endif
#if defined __GNUC__
    fflush(stdout);
#endif
#ifdef NO_BLOCK_MENU
    if( -1 == getline (&cmdbuf[0], sizeof (cmdbuf)) )        /* input command line          */
    {
      return;
    }
#else
    getline (&cmdbuf[0], sizeof (cmdbuf));
#endif
    for (i = 0; cmdbuf[i] == ' '; i++);           /* skip blanks on head         */
    for (; cmdbuf[i] != 0; i++)  {                /* convert to upper characters */
      cmdbuf[i] = toupper(cmdbuf[i]);
    }
    
    for (i = 0; cmdbuf[i] == ' '; i++);        /* skip blanks on head         */
    for(j=0; cmdbuf[i] != ' '&&cmdbuf[i] != 0; i++,j++)  {         /* find command name       */
      cmdname[j] = cmdbuf[i];
    }
    cmdname[j] = '\0';

    
    /***************** Command "0" or "BOOTUPDATE": Update the application  *************************/
    if(strcmp(cmdname, "BOOTUPDATE") == 0 || strcmp(cmdname, "0") == 0) {
      partition = MicoFlashGetInfo( MICO_PARTITION_BOOTLOADER );
      if (findCommandPara(cmdbuf, "r", NULL, 0) != -1){
        printf ("\n\rRead Bootloader...\n\r");
       // SerialUpload( partition->partition_owner, partition->partition_start_addr, "BootLoaderImage.bin", partition->partition_length );
        continue;
      }
      printf ("\n\rUpdating Bootloader...\n\r");
      err = MicoFlashDisableSecurity( MICO_PARTITION_BOOTLOADER, 0x0, partition->partition_length );
      require_noerr( err, exit);

      //SerialDownload( partition->partition_owner, partition->partition_start_addr, partition->partition_length );
    }
#if 0    
    /***************** Command "1" or "FWUPDATE": Update the MICO application  *************************/
    if(strcmp(cmdname, "FWUPDATE") == 0 || strcmp(cmdname, "1") == 0)	{
      partition = MicoFlashGetInfo( MICO_PARTITION_APPLICATION );
      if (findCommandPara(cmdbuf, "r", NULL, 0) != -1){
        printf ("\n\rRead application...\n\r");
        SerialUpload( partition->partition_owner, partition->partition_start_addr, "ApplicationImage.bin", partition->partition_length );
        continue;
      }
      printf ("\n\rUpdating application...\n\r");
      err = MicoFlashDisableSecurity( MICO_PARTITION_APPLICATION, 0x0, partition->partition_length );
      require_noerr( err, exit);
      SerialDownload( partition->partition_owner, partition->partition_start_addr, partition->partition_length ); 							   	
    }
    
    /***************** Command "2" or "PARAUPDATE": Update the application  *************************/
    else if(strcmp(cmdname, "PARUPDATE") == 0 || strcmp(cmdname, "2") == 0)  {
      if (findCommandPara(cmdbuf, "id", idStr, 0) != -1){
        if(Str2Int((uint8_t *)idStr, &id)==0 && id > 0 && id < MICO_PARTITION_MAX ){ //Found Flash start address
          printf ("\n\rIllegal start address.\n\r");
          continue;
        }
        partition = MicoFlashGetInfo( (mico_partition_t)id );
      }else{
        printf ("\n\rMiCO partition not found.\n\r");
        continue;
      }

      if( findCommandPara(cmdbuf, "e", NULL, 0) != -1 ){
        printf( "\n\rErasing %s...\n\r", partition->partition_description );

        err = MicoFlashDisableSecurity( (mico_partition_t)id, 0x0, partition->partition_length );
        require_noerr( err, exit);
        MicoFlashErase( (mico_partition_t)id, 0x0, partition->partition_length );
        continue;
      }
      if (findCommandPara(cmdbuf, "r", NULL, 0) != -1){
        printf ( "\n\rRead %s...\n\r", partition->partition_description );
        SerialUpload( partition->partition_owner, partition->partition_start_addr, "Image.bin", partition->partition_length );
        continue;
      }
      printf ("\n\rUpdating %s...\n\r", partition->partition_description );
      err = MicoFlashDisableSecurity( (mico_partition_t)id, 0x0, partition->partition_length );
      require_noerr( err, exit);
      SerialDownload( partition->partition_owner, partition->partition_start_addr, partition->partition_length );                        
    }
    
    /***************** Command "3" or "FLASHUPDATE": : Update the Flash  *************************/
    else if(strcmp(cmdname, "FLASHUPDATE") == 0 || strcmp(cmdname, "3") == 0) {
      if (findCommandPara(cmdbuf, "dev", flash_dev_str, 1) == -1  ){
        printf ("\n\rUnkown target type! Exiting...\n\r");
        continue;
      }
      
      if(Str2Int((uint8_t *)flash_dev_str, (int32_t *)&flash_dev)==0){ 
        printf ("\n\rDevice Number Err! Exiting...\n\r");
        continue;
      }
      if( flash_dev >= MICO_FLASH_MAX ){
        printf ("\n\rDevice Err! Exiting...\n\r");
        continue;
      }
      
      inputFlashArea = false;
      
      if (findCommandPara(cmdbuf, "start", startAddressStr, 10) != -1){
        if(Str2Int((uint8_t *)startAddressStr, &startAddress)==0){ //Found Flash start address
          printf ("\n\rIllegal start address.\n\r");
          continue;
        }else{
          if (findCommandPara(cmdbuf, "end", endAddressStr, 10) != -1){ //Found Flash end address
            if(Str2Int((uint8_t *)endAddressStr, &endAddress)==0){
              printf ("\n\rIllegal end address.\n\r");
              continue;
            }else{
              inputFlashArea = true;
            }
          }else{
            printf ("\n\rFlash end address not found.\n\r");
            continue;
          }
        }
      }
      
      if(endAddress<startAddress && inputFlashArea == true) {
        printf ("\n\rIllegal address.\n\r");
        continue;
      }
      
      if(inputFlashArea != true){
        startAddress = platform_flash_peripherals[ flash_dev ].flash_start_addr ;
        endAddress = platform_flash_peripherals[ flash_dev ].flash_start_addr 
          + platform_flash_peripherals[ flash_dev ].flash_length - 1;
      }
      
      if (findCommandPara(cmdbuf, "e", NULL, 0) != -1){
        printf ("\n\rErasing dev%d content From 0x%lx to 0x%lx\n\r", flash_dev, startAddress, endAddress);
        platform_flash_init( &platform_flash_peripherals[ flash_dev ] );
        platform_flash_disable_protect( &platform_flash_peripherals[ flash_dev ], startAddress, endAddress );
        platform_flash_erase( &platform_flash_peripherals[ flash_dev ], startAddress, endAddress );
        continue;
      }
      
      if (findCommandPara(cmdbuf, "r", NULL, 0) != -1){
        printf ("\n\rRead dev%d content From 0x%lx to 0x%lx\n\r", flash_dev, startAddress, endAddress);
        SerialUpload(flash_dev, startAddress, "FlashImage.bin", endAddress-startAddress+1);
        continue;
      }
      
      printf ("\n\rUpdating dev%d content From 0x%lx to 0x%lx\n\r", flash_dev, startAddress, endAddress);
      platform_flash_disable_protect( &platform_flash_peripherals[ flash_dev ], startAddress, endAddress );
      SerialDownload(flash_dev, startAddress, endAddress-startAddress+1);                           
    }
    
    
    /***************** Command: MEMORYMAP *************************/
    else if(strcmp(cmdname, "MEMORYMAP") == 0 || strcmp(cmdname, "4") == 0)  {
      printf("\r");
      for( i = 0; i <= MICO_PARTITION_MAX - 1; i++ ){
        partition = MicoFlashGetInfo( (mico_partition_t)i );
        if (partition->partition_owner == MICO_FLASH_NONE)
            continue;
        printf( "|ID:%d| %11s |  Dev:%d  | 0x%08lx | 0x%08lx |\r\n", i, partition->partition_description, partition->partition_owner,
               partition->partition_start_addr, partition->partition_length);
      }
    }
    /***************** Command: Excute the application *************************/
    else if(strcmp(cmdname, "BOOT") == 0 || strcmp(cmdname, "5") == 0)	{
      printf ("\n\rBooting.......\n\r");
      partition = MicoFlashGetInfo( MICO_PARTITION_APPLICATION );
      //bootloader_start_app( partition->partition_start_addr );
    }
    
    /***************** Command: Reboot *************************/
    else if(strcmp(cmdname, "REBOOT") == 0 || strcmp(cmdname, "6") == 0)  {
      printf ("\n\rReBooting.......\n\r");
      MicoSystemReboot();
      break;                              
    }
    else if(strcmp(cmdname, "GPIO") == 0 || strcmp(cmdname, "8") == 0)  {
      static char sel_Str[5];
      if (findCommandPara(cmdbuf, "0", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_SYS_LED );
        printf("\r\n triggered sys led\r\n");
      }
      else if (findCommandPara(cmdbuf, "1", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_5V_EN );
        printf("\r\n triggered 5V \r\n");
      }
      else if (findCommandPara(cmdbuf, "2", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_12V_EN );
        printf("\r\n triggered 12V \r\n");
      }
      else if (findCommandPara(cmdbuf, "3", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_24V_EN );
        printf("\r\n triggered 24V \r\n");
      }
      else if (findCommandPara(cmdbuf, "4", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_MOTOR_EN );
        printf("\r\n triggered 5v motor \r\n");
      }
      else if (findCommandPara(cmdbuf, "5", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_SENSOR_EN );
        printf("\r\n triggered sensor \r\n");
      }
      else if (findCommandPara(cmdbuf, "6", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_LEDS_EN );
        printf("\r\n triggered leds \r\n");
      }
      else if (findCommandPara(cmdbuf, "7", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_5V_RES_EN );
        printf("\r\n triggered 5V reserve \r\n");
      }
      else if (findCommandPara(cmdbuf, "8", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_PAD_EN );
        printf("\r\n triggered pad \r\n");
      }
      else if (findCommandPara(cmdbuf, "9", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_12V_ROUTER_EN );
        printf("\r\n triggered 12V router \r\n");
      }
      else if (findCommandPara(cmdbuf, "A", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_2_1_PA_EN );
        printf("\r\n triggered 2.1 pa \r\n");
      }
      else if (findCommandPara(cmdbuf, "B", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_DYP_EN );
        printf("\r\n triggered dyp \r\n");
      }
      else if (findCommandPara(cmdbuf, "C", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_X86_EN );
        printf("\r\n triggered x86 \r\n");
      }
      else if (findCommandPara(cmdbuf, "D", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_NV_EN );
        printf("\r\n triggered nv \r\n");
      }
      else if (findCommandPara(cmdbuf, "E", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_12V_RES_EN );
        printf("\r\n triggered 12v reserve \r\n");
      }
      else if (findCommandPara(cmdbuf, "F", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_PRINTER_EN );
        printf("\r\n triggered printer \r\n");
      }
      else if (findCommandPara(cmdbuf, "G", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_24V_RES_EN );
        printf("\r\n triggered 24V reserve \r\n");
      }
      else if (findCommandPara(cmdbuf, "H", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_BAT_NV_EN );
        printf("\r\n triggered bat nv \r\n");
      }
      else if (findCommandPara(cmdbuf, "I", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_PWR_NV );
        printf("\r\n triggered PWR NV \r\n");
      }
      else if (findCommandPara(cmdbuf, "J", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_PWR_DLP );
        printf("\r\n triggered PWR DLP \r\n");
      }
      else if (findCommandPara(cmdbuf, "K", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_PWR_PAD );
        printf("\r\n triggered PWR PAD \r\n");
      }
      else if (findCommandPara(cmdbuf, "L", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_PWR_X86 );
        printf("\r\n triggered PWR X86 \r\n");
      }
      else if (findCommandPara(cmdbuf, "M", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_PWR_RES );
        printf("\r\n triggered PWR RES \r\n");
      }
      else if (findCommandPara(cmdbuf, "N", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_DLP_EN );
        printf("\r\n triggered dlp \r\n");
      }
      else if (findCommandPara(cmdbuf, "O", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_AIUI_EN );
        printf("\r\n triggered aiui \r\n");
      }
      else if (findCommandPara(cmdbuf, "P", NULL, 0) != -1){
        MicoGpioOutputTrigger( MICO_GPIO_5V_ROUTER_EN );
        printf("\r\n triggered 5v router \r\n");
      }
      else if ( findCommandPara(cmdbuf, "Q", sel_Str, 0) != -1 )
      {      
        int32_t        sel_num;
        Str2Int((uint8_t *)sel_Str, &sel_num );
        if( sel_num/10000 )
        {
          MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_EN );
          printf("\r\n switch en: 0\r\n");
        }
        else
        {
          MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_EN );
          printf("\r\n switch en: 1\r\n");
        }
        if( sel_num%10000/1000 )
        {
          MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
          printf("\r\n switch sel0: 1\r\n");
        }
        else
        {
          MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL0 );
          printf("\r\n switch sel0: 0\r\n");
        }
        if( sel_num%1000/100 )
        {
          MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
          printf("\r\n switch sel1: 1\r\n");
        }
        else
        {
          MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL1 );
          printf("\r\n switch sel1: 0\r\n");
        }
        if( sel_num%100/10 )
        {
          MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
          printf("\r\n switch sel2: 1\r\n");
        }
        else
        {
          MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL2 );
          printf("\r\n switch sel2: 0\r\n");
        }
        if( sel_num%10 )
        {
          MicoGpioOutputHigh( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
          printf("\r\n switch sel3: 1\r\n");
        }
        else
        {
          MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_SWITCH_SEL3 );
          printf("\r\n switch sel3: 0\r\n");
        }
      }
      else {
        printf(   "\r\n8 -0: sys led\r\n"
                      "8 -1: 5v\r\n"
                      "8 -2: 12v\r\n"
                      "8 -3: 24v\r\n"
                      "8 -4: 5v motor\r\n"
                      "8 -5: sensor\r\n"
                      "8 -6: leds\r\n"
                      "8 -7: 5v reserve\r\n"
                      "8 -8: pad\r\n"
                      "8 -9: 12V_router\r\n"
                      "8 -A: 2.1 pa\r\n"
                      "8 -B: dyp\r\n"
                      "8 -C: x86\r\n"
                      "8 -D: nv\r\n"
                      "8 -E: 12v reserve\r\n"
                      "8 -F: printer\r\n"                      
                      "8 -G: 24v reserve\r\n"
                      "8 -H: bat nv\r\n"
                      "8 -I: PWR NV\r\n"
                      "8 -J: PWR DLP\r\n"
                      "8 -K: PWR PAD\r\n"
                      "8 -L: PWR X86\r\n"
                      "8 -M: PWR RES\r\n"
                      "8 -N: DLP\r\n"
                      "8 -O: AIUI\r\n"
                      "8 -P: 5V_ROUTER\r\n");
      }
      break;
    }
    else if( strcmp(cmdname, "IRLED") == 0 || strcmp(cmdname, "9") == 0 )
    {
      static char dutyStr[2], frequencyStr[6];
      int32_t   duty, freguency;
      if (findCommandPara(cmdbuf, "0", NULL, 0) != -1){
        startTps611xx();
        printf("\r\n start tps611xx work \r\n");
      }
      else if (findCommandPara(cmdbuf, "1", dutyStr, 0) != -1){
        Str2Int((uint8_t *)dutyStr, &duty );
#if (defined TPS_CONTROL_MODE) && (TPS_CONTROL_MODE == EASY_SCALE)
        brightness_dimming_by_duty( (uint8_t)duty );      
        printf("\r\n dimming duty:%d\r\n",duty );
#elif (defined TPS_CONTROL_MODE) && (TPS_CONTROL_MODE == PWM)
        if (findCommandPara(cmdbuf, "f", frequencyStr, 0) != -1)
        {
          Str2Int((uint8_t *)frequencyStr, &freguency );
          brightness_dimming( (uint32_t)freguency, (uint8_t)duty );
          //brightness_dimming_by_duty( (uint8_t)duty );
          printf("\r\n dimming frequency and duty:%d %d\r\n",freguency, duty );
        }
#endif
      }
      else
      {
        stopTps611xx( );
        printf("\r\n stop tps611xx work\r\n" );
      }
      break;
    }
    else if( strcmp(cmdname, "SHOW") == 0 || strcmp(cmdname, "A") == 0 )
    {
      voltageDebug.printType = PRINT_NO;
      printf("\r\nadc sequence num: (mV)\r\n");
      printf("1: _5V_reserve1_currents\r\n");
      printf("2: _12V_reserve2_currents\r\n");
      printf("3: bat_voltage\r\n");
      printf("4: sys_all_currents\r\n");
      printf("5: dh_12V_currents\r\n");
      printf("6: dh_5V_currents\r\n");
      printf("7: sensor_currents\r\n");
      printf("8: dlp_currents\r\n");
      printf("9: motor_5v_currents\r\n");
      printf("10: _24V_reserve1_currents\r\n");
      printf("11: _2_1_pa_currents\r\n");
      printf("12: pad_currents\r\n");
      printf("13: printer_currents\r\n");
      printf("14: x86_currents\r\n");
      printf("15: motor_currents\r\n");
      printf("16: _5V_led_currents\r\n");

      printf("17: bat_nv_currents\r\n");
      printf("18: _12V_nv_currents\r\n");
      printf("19: _12V_router_currents\r\n");
      printf("20: dyp_currents\r\n");
      printf("21: ir_led_currents\r\n");
      printf("22: charger_currents\r\n");
      printf("23: charge_currents\r\n");
      printf("24: _24V_voltage\r\n");
      printf("25: _12V_voltage\r\n");
      printf("26: _5V_voltage\r\n");
      printf("27: _5V_currents\r\n");
      printf("28: _12V_currents\r\n");
      printf("29: _24V_currents\r\n");
      printf("30: ambient_temperature\r\n");
      printf("31: _5V_regulator_temp\r\n");
      printf("32: _12V_regulator_temp\r\n");
      printf("33: _24V_regulator_temp\r\n");
      printf("34: aiui_currents\r\n");
      printf("35: _5V_router_currents\r\n");
      break;
    }
    else if(strcmp(cmdname, "VOL") == 0 || strcmp(cmdname, "B") == 0)	{
      voltageDebug.printType = PRINT_PEROID;
      voltageDebug.peroid = 500;
      break;
    }
    else if(strcmp(cmdname, "COMLOGOFF") == 0 || strcmp(cmdname, "C") == 0)  {
      serial->rx_info->showLogFlag = CLOSE_SHOW_LOG;
      break;
    }
    else if(strcmp(cmdname, "COMLOGON") == 0 || strcmp(cmdname, "D") == 0)  {
      serial->rx_info->showLogFlag = OPEN_SHOW_LOG;
      break;
    }
#ifdef MIKE_TEST
    else if(strcmp(cmdname, "CHARGER") == 0 || strcmp(cmdname, "E") == 0)  {
      printf("\r\n have detected charger plugin: %d times\r\n", boardStatus->charger_times);
      break;
    }     
#endif
    else if(strcmp(cmdname, "HELP") == 0 || strcmp(cmdname, "?") == 0)	{
        printf ( menu, MODEL, SW_VERSION, HARDWARE_REVISION );  /* display command menu        */
      break;
    }
    else if(strcmp(cmdname, "") == 0 )	{                     
      break;
    }
    else{
      printf (ERROR_STR, "UNKNOWN COMMAND");
      break;
    }

#endif   
    

exit:
    continue;
  }

}
