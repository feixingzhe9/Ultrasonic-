/* 
*  Author: Adam Huang
*  Date:2016/8/1
*/

#include "mico.h"
#include "common.h"
#include "upgrade_flash.h"
//#include "nbosDriverFlash.h"
#include "LibMd5.h"
#include "StringUtils.h"

#define upgrade_log(M, ...) custom_log("Upgrade", M, ##__VA_ARGS__)
#define upgrade_log_trace() custom_log_trace("Upgrade")

extern const mico_logic_partition_t     mico_partitions[];
extern const platform_flash_t platform_flash_peripherals[];

//static uint32_t flash_addr;
static uint32_t ota_write_data_offset = 0x0;
static upgrate_t upgrade_content;
upgrate_t *p_upgrade = &upgrade_content;
/*OTA options*/
boot_table_t             bootTable;
save_flash_data_t        flashTable;

OSStatus MICOUpdateConfiguration(boot_table_t *bootTable);

OSStatus upgradePrepareFlash( uint8_t *md5, uint32_t Size )
{
  OSStatus err = kNoErr;
  mico_logic_partition_t *para_partition_info, *ota_partition_info;  
  
  ota_partition_info = MicoFlashGetInfo( MICO_PARTITION_OTA_TEMP );
  require_action( ota_partition_info->partition_owner != MICO_FLASH_NONE, exit, err = kUnsupportedErr );
  require_action( Size <= ota_partition_info->partition_length, exit, err = kGeneralErr );
  
  //flash_addr = ota_partition_info->partition_start_addr;
  ota_write_data_offset = 0x0;
  
  para_partition_info = MicoFlashGetInfo( MICO_PARTITION_PARAMETER_1 );
  require_action( para_partition_info->partition_owner != MICO_FLASH_NONE, exit, err = kUnsupportedErr );

  DISABLE_INTERRUPTS();
  err = MicoFlashErase(MICO_PARTITION_PARAMETER_1, 0x0, para_partition_info->partition_length);
  ENABLE_INTERRUPTS();
  require_noerr(err, exit);
    
  if( p_upgrade )
  {
    memset( p_upgrade, 0x0, sizeof(upgrate_t) );
    memcpy( p_upgrade->md5, md5, 16 );
    p_upgrade->len = Size;
  }
      
  upgrade_log( "start erase OTA flash" );
  DISABLE_INTERRUPTS();
  err = MicoFlashErase( MICO_PARTITION_OTA_TEMP, 0x0, ota_partition_info->partition_length );
  ENABLE_INTERRUPTS();
  require_action( err == kNoErr, exit, upgrade_log("erase error") );
  upgrade_log("erase OK");
  
exit:
  return err;
}

OSStatus upgradeWriteFlashData( uint32_t* Data, uint32_t DataLength )
{
  OSStatus err = kNoErr;
  mico_logic_partition_t *ota_partition_info;  
  
  ota_partition_info = MicoFlashGetInfo( MICO_PARTITION_OTA_TEMP );
  require_action( ota_partition_info->partition_owner != MICO_FLASH_NONE, exit, err = kUnsupportedErr );
  
//  update_data_offset = flash_addr - ota_partition_info->partition_start_addr;
  DISABLE_INTERRUPTS();
  err = MicoFlashWrite( MICO_PARTITION_OTA_TEMP, &ota_write_data_offset, (uint8_t *)Data, DataLength );
  ENABLE_INTERRUPTS();
  require_noerr_action( err, exit, upgrade_log("write error") ); 
//  flash_addr += DataLength;
  upgrade_log( "have write data @%08x %d bytes", \
    ota_partition_info->partition_start_addr + ota_write_data_offset - DataLength, DataLength );
  
exit:
  return err;
}

OSStatus upgradeCheckFlash( void )
{
  OSStatus err = kNoErr;
  MD5_HASH md5_ret;
  Md5Context ctx;
  mico_logic_partition_t *ota_partition_info;
  
  ota_partition_info = MicoFlashGetInfo( MICO_PARTITION_OTA_TEMP );
  require_action( ota_partition_info->partition_owner != MICO_FLASH_NONE, exit, err = kUnsupportedErr );
  Md5Initialise( &ctx );
  DISABLE_INTERRUPTS();
  Md5Update( &ctx, (void *)ota_partition_info->partition_start_addr, ota_write_data_offset );
  ENABLE_INTERRUPTS();
  Md5Finalise( &ctx, &md5_ret );
  
  if( memcmp(md5_ret.bytes, p_upgrade->md5, 16) != 0 )
  {
#if 1
    char *debugString;
    debugString = DataToHexString( p_upgrade->md5, 16 );
    upgrade_log( "correct Md5 is:%s", debugString );
    free( debugString );
    debugString = DataToHexString( md5_ret.bytes, 16 );
    upgrade_log( "Md5:%s,is error", debugString );
    free( debugString );
    
#endif
    err = kGeneralErr;
    goto exit;
  }
  else
  {
    upgrade_log( "Md5:%s,is correct",DataToHexString(md5_ret.bytes,16) );
    memset( &bootTable, 0, sizeof(boot_table_t) );
    bootTable.length = ota_write_data_offset;
    bootTable.start_address = ota_partition_info->partition_start_addr;
    bootTable.type = 'A';
    bootTable.upgrade_type = 'U';
    MICOUpdateConfiguration( &bootTable );
    flashTable.isNeedAutoBoot = 'Y';
    MICOBootConfiguration( &flashTable );
  }
exit:  
  return err;
}

OSStatus MICOUpdateConfiguration( boot_table_t *bootTable )
{
  OSStatus err = kNoErr;
  mico_logic_partition_t *para_partition_info;
  uint32_t update_data_offset = 0x0;
  
  para_partition_info = MicoFlashGetInfo( MICO_PARTITION_PARAMETER_1 );
  require_action( para_partition_info->partition_owner != MICO_FLASH_NONE, exit, err = kUnsupportedErr );
  
  err = MicoFlashErase( MICO_PARTITION_PARAMETER_1, 0x0, para_partition_info->partition_length );
  require_noerr( err, exit );
  err = MicoFlashWrite( MICO_PARTITION_PARAMETER_1, &update_data_offset, (uint8_t *)bootTable, sizeof(boot_table_t) );
  require_noerr( err, exit );
//  err = MicoFlashFinalize(MICO_PARTITION_PARAMETER_1);
//  require_noerr(err, exit);

exit:
  return err;
}

OSStatus MICOBootConfiguration( save_flash_data_t *flashTable )
{
  OSStatus err = kNoErr;
  mico_logic_partition_t *para_partition_info;
  uint32_t update_data_offset = 0x0;
  
  para_partition_info = MicoFlashGetInfo( MICO_PARTITION_PARAMETER_2 );
  require_action( para_partition_info->partition_owner != MICO_FLASH_NONE, exit, err = kUnsupportedErr );
  
  err = MicoFlashErase( MICO_PARTITION_PARAMETER_2, 0x0, para_partition_info->partition_length );
  require_noerr( err, exit );
  err = MicoFlashWrite( MICO_PARTITION_PARAMETER_2, &update_data_offset, (uint8_t *)flashTable, sizeof(save_flash_data_t) );
  require_noerr( err, exit );
//  err = MicoFlashFinalize(MICO_PARTITION_PARAMETER_1);
//  require_noerr(err, exit);

exit:
  return err;
}