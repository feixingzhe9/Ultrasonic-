/* 
*  Author: Adam Huang
*  Date:2016/8/1
*/

#ifndef __UPGRADE_FLASH_H__
#define __UPGRADE_FLASH_H__

typedef struct _upgrade_t {
  uint8_t md5[16];
  uint32_t len;
  uint8_t data[1];
} upgrate_t;

/* Upgrade iamge should save this table to flash */
typedef struct  _boot_table_t {
  uint32_t start_address; // the address of the bin saved on flash.
  uint32_t length; // file real length
  uint8_t version[8];
  uint8_t type; // B:bootloader, P:boot_table, A:application, 
  uint8_t upgrade_type; //u:upgrade, 
  uint8_t reserved[6];
}boot_table_t;

typedef struct {
  uint8_t       isNeedAutoBoot; 
} save_flash_data_t;
extern save_flash_data_t        flashTable;

OSStatus upgradePrepareFlash( uint8_t *md5, uint32_t Size );
OSStatus upgradeWriteFlashData( uint32_t* Data, uint32_t DataLength );
OSStatus upgradeCheckFlash( void );
OSStatus MICOBootConfiguration( save_flash_data_t *flashTable );

#endif //#ifndef __UPGRADE_FLASH_H__

