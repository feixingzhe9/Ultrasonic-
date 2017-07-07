/* 
*  Author: Adam Huang
*  Date:2016/7/4
*/
#include "nbosDriverFlash.h"
#include "platform.h"

#define WRITE_PROTECTION_DISABLE
//#define WRITE_PROTECTION_ENABLE

/* End of the Flash address */
#define FLASH_START_ADDRESS     (uint32_t)0x08000000  
#define FLASH_END_ADDRESS       (uint32_t)0x0801FFFF
#define FLASH_SIZE              (FLASH_END_ADDRESS -  FLASH_START_ADDRESS + 1)

#define FLASH_PAGE_TO_BE_PROTECTED   OB_WRP_PAGES20TO23


OSStatus internalFlashInitialize( void );
OSStatus internalFlashErase(uint32_t StartAddress, uint32_t EndAddress);
OSStatus internalFlashWrite(volatile uint32_t* FlashAddress, uint32_t* Data ,uint32_t DataLength);
OSStatus internalFlashFinalize( void );

OSStatus MicoFlashInitialize( mico_flash_t flash )
{
  internal_log_trace();
  if(flash == MICO_INTERNAL_FLASH){
    return internalFlashInitialize();    
  }
  else
    return kUnsupportedErr;
}

OSStatus MicoFlashErase( mico_flash_t flash, uint32_t StartAddress, uint32_t EndAddress )
{ 
  internal_log_trace();
  if(flash == MICO_INTERNAL_FLASH){
    if(StartAddress<INTERNAL_FLASH_START_ADDRESS || EndAddress > INTERNAL_FLASH_END_ADDRESS)
      return kParamErr;
    return internalFlashErase(StartAddress, EndAddress);    
  }
  else
    return kUnsupportedErr;
}

OSStatus MicoFlashWrite(mico_flash_t flash, volatile uint32_t* FlashAddress, uint8_t* Data ,uint32_t DataLength)
{
  if(flash == MICO_INTERNAL_FLASH){
    if( *FlashAddress<INTERNAL_FLASH_START_ADDRESS || *FlashAddress + DataLength > INTERNAL_FLASH_END_ADDRESS + 1)
      return kParamErr;
    return internalFlashWrite(FlashAddress, (uint32_t *)Data, DataLength);    
  }
  else
    return kUnsupportedErr;
}

OSStatus MicoFlashRead(mico_flash_t flash, volatile uint32_t* FlashAddress, uint8_t* Data ,uint32_t DataLength)
{
  if(flash == MICO_INTERNAL_FLASH){
    if( *FlashAddress<INTERNAL_FLASH_START_ADDRESS || *FlashAddress + DataLength > INTERNAL_FLASH_END_ADDRESS + 1)
      return kParamErr;
    memcpy(Data, (void *)(*FlashAddress), DataLength);
    *FlashAddress += DataLength;
    return kNoErr;
  }
  else
    return kUnsupportedErr;
}

OSStatus MicoFlashFinalize( mico_flash_t flash )
{
  if(flash == MICO_INTERNAL_FLASH){
    return internalFlashFinalize();    
  }
  else
    return kUnsupportedErr;
}

OSStatus internalFlashInitialize( void )
{
  internal_log_trace();
  FLASH_OBProgramInitTypeDef OptionsBytesStruct;
  
  /* Unlock the Flash to enable the flash control register access *************/ 
  HAL_FLASH_Unlock();  
  /* Unlock the Options Bytes *************************************************/
  HAL_FLASH_OB_Unlock();
  /* Get pages write protection status ****************************************/
  HAL_FLASHEx_OBGetConfig(&OptionsBytesStruct);  
#ifdef WRITE_PROTECTION_DISABLE
  /* Check if desired pages are already write protected ***********************/
  if((OptionsBytesStruct.WRPPage & FLASH_PAGE_TO_BE_PROTECTED) != FLASH_PAGE_TO_BE_PROTECTED)
  {
    /* Restore write protected pages */
    OptionsBytesStruct.OptionType   = OPTIONBYTE_WRP;
    OptionsBytesStruct.WRPState     = OB_WRPSTATE_DISABLE;
    OptionsBytesStruct.WRPPage = FLASH_PAGE_TO_BE_PROTECTED;
    if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
    {
      /* Error occurred while options bytes programming. **********************/
    }
    /* Generate System Reset to load the new option byte values ***************/
    HAL_FLASH_OB_Launch();
  }
#elif defined WRITE_PROTECTION_ENABLE
  /* Check if desired pages are not yet write protected ***********************/
  if(((~OptionsBytesStruct.WRPPage) & FLASH_PAGE_TO_BE_PROTECTED )!= FLASH_PAGE_TO_BE_PROTECTED)
  {
    /* Enable the pages write protection **************************************/
    OptionsBytesStruct.OptionType = OPTIONBYTE_WRP;
    OptionsBytesStruct.WRPState   = OB_WRPSTATE_ENABLE;
    OptionsBytesStruct.WRPPage    = FLASH_PAGE_TO_BE_PROTECTED;
    if(HAL_FLASHEx_OBProgram(&OptionsBytesStruct) != HAL_OK)
    {
      /* Error occurred while options bytes programming. **********************/
    }
    /* Generate System Reset to load the new option byte values ***************/
    HAL_FLASH_OB_Launch();
  }
#endif /* WRITE_PROTECTION_DISABLE */
  /* Lock the Options Bytes *************************************************/
  HAL_FLASH_OB_Lock();
  /* Clear pending flags (if any) */
  __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR );
  return kNoErr;    
}

OSStatus internalFlashErase(uint32_t StartAddress, uint32_t EndAddress)
{
  internal_log_trace();
  OSStatus err = kNoErr;
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t PAGEError = 0;
  
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = StartAddress;
  EraseInitStruct.NbPages     = (EndAddress - StartAddress) / FLASH_PAGE_SIZE;
  
  require_action(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) == HAL_OK, exit, err = kWriteErr); 
  
exit:
  return err;
}

OSStatus internalFlashWrite(volatile uint32_t* FlashAddress, uint32_t* Data ,uint32_t DataLength)
{
  internal_log_trace();
  OSStatus err = kNoErr;
  uint32_t i = 0;
  uint32_t dataInRam;
  uint32_t DataLength32 = DataLength;

  if( *FlashAddress%2){
    err = kNoAddressAckErr;
    internal_log("write address @%08x not align",*FlashAddress);
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
      require_action(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, *FlashAddress, ((uint16_t)*Data & 0x00FF)  | 0xFF00) == HAL_OK, exit, err = kWriteErr);
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
