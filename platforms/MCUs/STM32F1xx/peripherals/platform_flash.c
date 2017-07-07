/* 
*  Author: Adam Huang
*  Date:2016/7/4
*/
#include "platform_peripheral.h"

#define WRITE_PROTECTION_DISABLE
//#define WRITE_PROTECTION_ENABLE

/* Private constants --------------------------------------------------------*/
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, page 0 to 3: 4 Kbyte */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08001000) /* Base @ of Sector 1, page 4 to 7: 4 Kbyte */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08002000) /* Base @ of Sector 2, page 8 to 11: 4 Kbyte */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x08003000) /* Base @ of Sector 3, page 12 to 15: 4 Kbyte */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08004000) /* Base @ of Sector 4, page 16 to 19: 4 Kbyte */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08005000) /* Base @ of Sector 5, page 20 to 23: 4 Kbyte */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08006000) /* Base @ of Sector 6, page 24 to 27: 4 Kbyte */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08007000) /* Base @ of Sector 7, page 28 to 31: 4 Kbyte */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08008000) /* Base @ of Sector 8, page 32 to 35: 4 Kbyte */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x08009000) /* Base @ of Sector 9, page 36 to 39: 4 Kbyte */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x0800A000) /* Base @ of Sector 10, page 40 to 43: 4 Kbyte */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x0800B000) /* Base @ of Sector 11, page 44 to 47: 4 Kbyte */
#define ADDR_FLASH_SECTOR_12    ((uint32_t)0x0800C000) /* Base @ of Sector 12, page 48 to 51: 4 Kbyte */
#define ADDR_FLASH_SECTOR_13    ((uint32_t)0x0800D000) /* Base @ of Sector 13, page 52 to 55: 4 Kbyte */
#define ADDR_FLASH_SECTOR_14    ((uint32_t)0x0800E000) /* Base @ of Sector 14, page 56 to 59: 4 Kbyte */
#define ADDR_FLASH_SECTOR_15    ((uint32_t)0x0800F000) /* Base @ of Sector 15, page 60 to 63: 4 Kbyte */
#define ADDR_FLASH_SECTOR_16    ((uint32_t)0x08010000) /* Base @ of Sector 16, page 64 to 67: 4 Kbyte */
#define ADDR_FLASH_SECTOR_17    ((uint32_t)0x08011000) /* Base @ of Sector 17, page 68 to 71: 4 Kbyte */
#define ADDR_FLASH_SECTOR_18    ((uint32_t)0x08012000) /* Base @ of Sector 18, page 72 to 75: 4 Kbyte */
#define ADDR_FLASH_SECTOR_19    ((uint32_t)0x08013000) /* Base @ of Sector 19, page 76 to 79: 4 Kbyte */
#define ADDR_FLASH_SECTOR_20    ((uint32_t)0x08014000) /* Base @ of Sector 20, page 80 to 83: 4 Kbyte */
#define ADDR_FLASH_SECTOR_21    ((uint32_t)0x08015000) /* Base @ of Sector 21, page 84 to 87: 4 Kbyte */
#define ADDR_FLASH_SECTOR_22    ((uint32_t)0x08016000) /* Base @ of Sector 24, page 88 to 91: 4 Kbyte */
#define ADDR_FLASH_SECTOR_23    ((uint32_t)0x08017000) /* Base @ of Sector 23, page 92 to 95: 4 Kbyte */
#define ADDR_FLASH_SECTOR_24    ((uint32_t)0x08018000) /* Base @ of Sector 24, page 96 to 99: 4 Kbyte */
#define ADDR_FLASH_SECTOR_25    ((uint32_t)0x08019000) /* Base @ of Sector 25, page 100 to 103: 4 Kbyte */
#define ADDR_FLASH_SECTOR_26    ((uint32_t)0x0801A000) /* Base @ of Sector 26, page 104 to 107: 4 Kbyte */
#define ADDR_FLASH_SECTOR_27    ((uint32_t)0x0801B000) /* Base @ of Sector 27, page 108 to 111: 4 Kbyte */
#define ADDR_FLASH_SECTOR_28    ((uint32_t)0x0801C000) /* Base @ of Sector 28, page 112 to 115: 4 Kbyte */
#define ADDR_FLASH_SECTOR_29    ((uint32_t)0x0801D000) /* Base @ of Sector 29, page 116 to 119: 4 Kbyte */
#define ADDR_FLASH_SECTOR_30    ((uint32_t)0x0801E000) /* Base @ of Sector 30, page 120 to 123: 4 Kbyte */
#if defined (STM32F103xB)
#define ADDR_FLASH_SECTOR_31    ((uint32_t)0x0801F000) /* Base @ of Sector 31, page 124 to 127: 4 Kbyte */
#elif defined (STM32F103xE)
#define ADDR_FLASH_SECTOR_31    ((uint32_t)0x0801F000) /* Base @ of Sector 32, page 62 to 255: 388 Kbyte */
#endif

/* End of the Flash address */
#if defined (STM32F103xB)
#define FLASH_START_ADDRESS     (uint32_t)0x08000000  
#define FLASH_END_ADDRESS       (uint32_t)0x0801FFFF
#define FLASH_SIZE              (FLASH_END_ADDRESS -  FLASH_START_ADDRESS + 1)
//#define FLASH_PAGE_TO_BE_PROTECTED   OB_WRP_PAGES20TO23
//#define PARA_START_ADDRESS          (uint32_t)0x08005000 
//#define PARA_END_ADDRESS            (uint32_t)0x08005FFF
#elif defined (STM32F103xE)
#define FLASH_START_ADDRESS     (uint32_t)0x08000000  
#define FLASH_END_ADDRESS       (uint32_t)0x0807FFFF
#define FLASH_SIZE              (FLASH_END_ADDRESS -  FLASH_START_ADDRESS + 1)
//#define FLASH_PAGE_TO_BE_PROTECTED   (OB_WRP_PAGES20TO21 | OB_WRP_PAGES22TO23)
//#define PARA_START_ADDRESS          (uint32_t)0x08005000 
//#define PARA_END_ADDRESS            (uint32_t)0x08005FFF
#endif

static OSStatus internalFlashInitialize( void );
static OSStatus internalFlashErase(uint32_t StartAddress, uint32_t EndAddress);
static OSStatus internalFlashWrite(volatile uint32_t* FlashAddress, uint32_t* Data ,uint32_t DataLength);
//static OSStatus internalFlashByteWrite( volatile uint32_t* FlashAddress, uint8_t* Data ,uint32_t DataLength );
static OSStatus internalFlashProtect(uint32_t StartAddress, uint32_t EndAddress, bool enable);
static uint32_t _GetWRPSector(uint32_t Address);
static uint32_t _GetPageAddress(uint32_t Address);
//static OSStatus _GetAddress(uint32_t sector, uint32_t *startAddress, uint32_t *endAddress);

OSStatus platform_flash_init( const platform_flash_t *peripheral )
{
  OSStatus err = kNoErr;

  require_action_quiet( peripheral != NULL, exit, err = kParamErr);

  if( peripheral->flash_type == FLASH_TYPE_EMBEDDED ){
    err = internalFlashInitialize();
    require_noerr(err, exit);
  }
#ifdef USE_MICO_SPI_FLASH
  else if( peripheral->flash_type == FLASH_TYPE_SPI ){
    err = init_sflash( &sflash_handle, 0, SFLASH_WRITE_ALLOWED );
    require_noerr(err, exit);
  }
#endif
  else{
    err = kTypeErr;
    goto exit;
  }
exit:
  return err;
}

OSStatus platform_flash_erase( const platform_flash_t *peripheral, uint32_t start_address, uint32_t end_address )
{
  OSStatus err = kNoErr;

  require_action_quiet( peripheral != NULL, exit, err = kParamErr);
  require_action( start_address >= peripheral->flash_start_addr 
               && end_address   <= peripheral->flash_start_addr + peripheral->flash_length - 1, exit, err = kParamErr);

  if( peripheral->flash_type == FLASH_TYPE_EMBEDDED ){
    err = internalFlashErase( start_address, end_address );    
    require_noerr(err, exit);
  }
#ifdef USE_MICO_SPI_FLASH
  else if( peripheral->flash_type == FLASH_TYPE_SPI ){
    err = spiFlashErase( start_address, end_address );
    require_noerr(err, exit);
  }
#endif
  else{
    err = kTypeErr;
    goto exit;
  }

exit:
  return err;
}

OSStatus platform_flash_write( const platform_flash_t *peripheral, volatile uint32_t* start_address, uint8_t* data ,uint32_t length  )
{
  OSStatus err = kNoErr;

  require_action_quiet( peripheral != NULL, exit, err = kParamErr);
  require_action( *start_address >= peripheral->flash_start_addr 
               && *start_address + length <= peripheral->flash_start_addr + peripheral->flash_length, exit, err = kParamErr);
    
  if( peripheral->flash_type == FLASH_TYPE_EMBEDDED ){
    err = internalFlashWrite( start_address, (uint32_t *)data, length); 
    require_noerr(err, exit);
  }
#ifdef USE_MICO_SPI_FLASH
  else if( peripheral->flash_type == FLASH_TYPE_SPI ){
    err = sflash_write( &sflash_handle, *start_address, data, length );
    require_noerr(err, exit);
    *start_address += length;
  }
#endif
  else{
    err = kTypeErr;
    goto exit;
  }

exit:
  return err;
}

OSStatus platform_flash_read( const platform_flash_t *peripheral, volatile uint32_t* start_address, uint8_t* data ,uint32_t length  )
{
  OSStatus err = kNoErr;

  require_action_quiet( peripheral != NULL, exit, err = kParamErr);
  require_action( (*start_address >= peripheral->flash_start_addr) 
               && (*start_address + length) <= ( peripheral->flash_start_addr + peripheral->flash_length), exit, err = kParamErr);

  if( peripheral->flash_type == FLASH_TYPE_EMBEDDED ){
    memcpy(data, (void *)(*start_address), length);
    *start_address += length;
  }
#ifdef USE_MICO_SPI_FLASH
  else if( peripheral->flash_type == FLASH_TYPE_SPI ){
    err = sflash_read( &sflash_handle, *start_address, data, length );
    require_noerr(err, exit);
    *start_address += length;
  }
#endif
  else{
    err = kTypeErr;
    goto exit;
  }

exit:
  return err;
}

OSStatus platform_flash_enable_protect( const platform_flash_t *peripheral, uint32_t start_address, uint32_t end_address )
{
  OSStatus err = kNoErr;

  require_action_quiet( peripheral != NULL, exit, err = kParamErr);
  require_action( start_address >= peripheral->flash_start_addr 
               && end_address   <= peripheral->flash_start_addr + peripheral->flash_length - 1, exit, err = kParamErr);

  if( peripheral->flash_type == FLASH_TYPE_EMBEDDED ){
#ifdef MCU_EBANLE_FLASH_PROTECT
    err = internalFlashProtect( start_address, end_address, true );  
#endif  
    require_noerr(err, exit);
  }
#ifdef USE_MICO_SPI_FLASH
  else if( peripheral->flash_type == FLASH_TYPE_SPI ){
    err = kNoErr;
    goto exit;
  }
#endif
  else{
    err = kTypeErr;
    goto exit;
  }

exit:
  return err;  
}

OSStatus platform_flash_disable_protect( const platform_flash_t *peripheral, uint32_t start_address, uint32_t end_address )
{
  OSStatus err = kNoErr;

  require_action_quiet( peripheral != NULL, exit, err = kParamErr);
  require_action( start_address >= peripheral->flash_start_addr 
               && end_address   <= peripheral->flash_start_addr + peripheral->flash_length - 1, exit, err = kParamErr);

  if( peripheral->flash_type == FLASH_TYPE_EMBEDDED ){
    err = internalFlashProtect( start_address, end_address, false );    
    require_noerr(err, exit);
  }
#ifdef USE_MICO_SPI_FLASH
  else if( peripheral->flash_type == FLASH_TYPE_SPI ){
    err = kNoErr;
    goto exit;
  }
#endif
  else{
    err = kTypeErr;
    goto exit;
  }

exit:
  return err;  
}

OSStatus internalFlashInitialize( void )
{
  platform_log_trace();
  
  /* Unlock the Flash to enable the flash control register access *************/ 
  HAL_FLASH_Unlock();  

  /* Clear pending flags (if any) */
  __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR );
  return kNoErr;    
}

OSStatus internalFlashErase(uint32_t StartAddress, uint32_t EndAddress)
{
  platform_log_trace();
  OSStatus err = kNoErr;
  uint32_t StartPageAddredss, EndPagesAddress, i = 0, j = 0, totalPages = 0;
  static FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PAGEError = 0;
  
  StartPageAddredss = _GetPageAddress( StartAddress );
  EndPagesAddress = _GetPageAddress( EndAddress );
  totalPages = (EndPagesAddress - StartPageAddredss) / FLASH_PAGE_SIZE + 1;
  
  for( i = 0; i < totalPages; i++ )
  {
    for( j = StartPageAddredss + i*FLASH_PAGE_SIZE; j <= StartPageAddredss + (i+1)*FLASH_PAGE_SIZE - 1; j+=8 )
    {
      if( (*(uint32_t *)(j))!=0xFFFFFFFF )
        break;
    }
    if( j > StartPageAddredss + (i+1)*FLASH_PAGE_SIZE - 1 )
      continue;
    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = StartPageAddredss + i * FLASH_PAGE_SIZE;
    EraseInitStruct.NbPages     = 1;    
    require_action(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) == HAL_OK, exit, err = kWriteErr);
  }  
  
exit:
  return err;
}

OSStatus internalFlashWrite(volatile uint32_t* FlashAddress, uint32_t* Data ,uint32_t DataLength)
{
  platform_log_trace();
  OSStatus err = kNoErr;
  uint32_t i = 0;
  uint32_t dataInRam;
  uint32_t DataLength32 = DataLength;

  if( *FlashAddress%2){
    err = kNoAddressAckErr;
    platform_log("write address @%08x not align",*FlashAddress);
    *FlashAddress += 1;
    goto exit;
  }
  /*First bytes that are not 32bit align*/
  if(*FlashAddress%4){
    require_action(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, *FlashAddress, *(uint16_t *)Data) == HAL_OK, exit, err = kWriteErr);
    require_action(*(uint16_t*)*FlashAddress == *(uint16_t *)Data, exit, err = kChecksumErr);    
    DataLength32 = DataLength - 2;
    *FlashAddress += 2;
    Data = (uint32_t *)((uint32_t)Data + 2);
  }
  
  /*Program flash by words*/
  for (i = 0; (i < DataLength32/4) && (*FlashAddress <= (FLASH_END_ADDRESS-3)); i++)
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
    be done by word */ 
    dataInRam = *Data;
    require_action(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, *FlashAddress, dataInRam) == HAL_OK, exit, err = kWriteErr); 
    require_action(*(uint32_t*)*FlashAddress == dataInRam, exit, err = kChecksumErr); 
    /* Increment FLASH destination address */
    *FlashAddress += 4;
    Data = (uint32_t *)((uint32_t)Data + 4);
  }
  
  /*Last bytes that cannot be write by a 32 bit word*/
  if( DataLength32 % 4 ) {
    if( DataLength32 % 4 == 1 ) {      
      require_action(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, *FlashAddress, ((uint16_t)*Data & 0x00FF) | 0xFF00) == HAL_OK, exit, err = kWriteErr);
      require_action(*(uint16_t*)*FlashAddress == ((uint16_t)*Data & 0x00FF) | 0xFF00, exit, err = kChecksumErr); 
      *FlashAddress += 1;
      Data = (uint32_t *)((uint32_t)Data + 1);
    }
    else if( DataLength32 % 4 == 2 ) {
      require_action(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, *FlashAddress, (uint16_t)*Data) == HAL_OK, exit, err = kWriteErr);
      require_action(*(uint16_t*)*FlashAddress == *(uint16_t *)Data, exit, err = kChecksumErr); 
      *FlashAddress += 2;
      Data = (uint32_t *)((uint32_t)Data + 2);
    }
    else {
      require_action(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, *FlashAddress, (uint16_t)*Data) == HAL_OK, exit, err = kWriteErr);
      require_action(*(uint16_t*)*FlashAddress == *(uint16_t *)Data, exit, err = kChecksumErr); 
      *FlashAddress += 2;
      Data = (uint32_t *)((uint32_t)Data + 2);
      require_action(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, *FlashAddress, (((uint16_t)*Data) & 0x00FF)  | 0xFF00) == HAL_OK, exit, err = kWriteErr);
      require_action(*(uint16_t*)*FlashAddress == ((uint16_t)*Data & 0x00FF) | 0xFF00, exit, err = kChecksumErr); 
      *FlashAddress += 1;
      Data = (uint32_t *)((uint32_t)Data + 1);
    }
  }
  
exit:
  return err;
}

OSStatus internalFlashFinalize( void )
{
  HAL_FLASH_Lock();
  return kNoErr;
}

OSStatus internalFlashProtect(uint32_t StartAddress, uint32_t EndAddress, bool enable)
{
  OSStatus err = kNoErr;
  FLASH_OBProgramInitTypeDef OptionsBytesStruct; 
  static uint32_t StartSector, EndSector, i = 0;
  bool needupdate = false;
  
  StartSector = _GetWRPSector(StartAddress);
  EndSector = _GetWRPSector(EndAddress);

  /* Get pages write protection status ****************************************/
  HAL_FLASHEx_OBGetConfig( &OptionsBytesStruct );  
  
  for( i = StartSector; i <= EndSector && i != 0; i = i<<1 )
  {
    if( ( enable == true && (OptionsBytesStruct.WRPPage & i) == 0x0 ) ||
        ( enable == false && (OptionsBytesStruct.WRPPage & i) ) ) {
      continue;
    }
    if( needupdate == false){
       /* Unlock the Options Bytes *************************************************/
      HAL_FLASH_OB_Unlock();
      needupdate = true;
    }
    if( enable == true )
    {    
      /* Restore write protected pages */
      OptionsBytesStruct.OptionType   = OPTIONBYTE_WRP;
      OptionsBytesStruct.WRPState     = OB_WRPSTATE_ENABLE;
      OptionsBytesStruct.WRPPage      = i;
      require_action( HAL_FLASHEx_OBProgram(&OptionsBytesStruct) == HAL_OK, exit, err = kWriteErr ); 
    }
    else
    {
       /* Restore write protected pages */
      OptionsBytesStruct.OptionType   = OPTIONBYTE_WRP;
      OptionsBytesStruct.WRPState     = OB_WRPSTATE_DISABLE;
      OptionsBytesStruct.WRPPage      = i;
      require_action( HAL_FLASHEx_OBProgram(&OptionsBytesStruct) == HAL_OK, exit, err = kWriteErr ); 
    }
  }
  if( needupdate == true){
    /* Generate System Reset to load the new option byte values ***************/
    HAL_FLASH_OB_Launch();
    /* Lock the Options Bytes *************************************************/
    HAL_FLASH_OB_Lock();
  }

exit:  
  return err;
}
/**
* @brief  Gets the sector of a given address
* @param  Address: Flash address
* @retval The sector of a given address
*/
static uint32_t _GetWRPSector(uint32_t Address)
{
  uint32_t sector = 0;
#if defined(STM32F103xB)  
  sector = ( Address - FLASH_START_ADDRESS ) / (FLASH_PAGE_SIZE * 4);
  return (OB_WRP_PAGES0TO3 << sector);
#elif defined(STM32F103xE)
  if( Address >= ADDR_FLASH_SECTOR_31 )
    return OB_WRP_PAGES62TO255;
  sector = ( Address - FLASH_START_ADDRESS ) / (FLASH_PAGE_SIZE * 2);
  return (OB_WRP_PAGES0TO1 << sector);
#endif
}
/**
* @brief  Gets the page of a given address
* @param  Address: Flash address
* @retval The page of a given address
*/
static uint32_t _GetPageAddress(uint32_t Address)
{
  uint32_t pageAddress = 0;
  uint32_t pageNum = 0;
  
  pageNum = ( Address - FLASH_START_ADDRESS ) / FLASH_PAGE_SIZE;
  pageAddress = pageNum * FLASH_PAGE_SIZE + FLASH_START_ADDRESS;

  return pageAddress;
}
#if 0
/**
* @brief  Gets the address of a given sector
* @param  Sector: The sector of a given address
* @retval Flash address if the sector start
*/
static OSStatus _GetAddress(uint32_t sector, uint32_t *startAddress, uint32_t *endAddress)
{
  OSStatus err = kNoErr; 
#if defined (STM32F103xB)
  uint32_t i, j;

  for( i = OB_WRP_PAGES0TO3, j = 0; i < OB_WRP_PAGES124TO127; i=i<< 1, j++ )
  {
    if( i == sector )
    {
      *startAddress = FLASH_START_ADDRESS + j * FLASH_PAGE_SIZE;
      *endAddress = *startAddress + (j + 4) * FLASH_PAGE_SIZE - 1;
      break;
    }   
  }
  if( j >= 32 )
  {
    err = kNotFoundErr;
  }
#elif defined (STM32F103xE)
  err = kUnsupportedErr;
#endif
  return err;
}
#endif