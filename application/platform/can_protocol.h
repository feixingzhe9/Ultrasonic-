/* 
*  Author: Adam Huang
*  Date:2017/01/23
*/

#ifndef __CAN_PROTOOCOL_H
#define __CAN_PROTOOCOL_H

#include "mico.h"

#define CAN_USED    CAN1




#define DESTID          0x02
//#define CAN_SUB_PB_ID   0x51
#define SRCID           0x06

#define ULTRASONIC_SRC_ID_BASE          0x60

#define CAN_CMD_READ_VERSION            0x01



#define SOURCE_ID_PREPARE_UPDATE        0x10
#define SOURCE_ID_TRANSMIT_UPDATE       0x11
#define SOURCE_ID_CHECK_TRANSMIT        0x12



#define CAN_FIFO_SIZE                   50



//////  function id define  //////
#define CAN_FUN_ID_RESET        0x06
#define CAN_FUN_ID_WRITE        0x01
#define CAN_FUN_ID_READ         0x02
#define CAN_FUN_ID_TRIGGER      0x03


//////  source id define  //////
#define CAN_SOURCE_ID_READ_VERSION      0x01    

#define CAN_SOURCE_ID_READ_MEASURE_DATA     0x80

#define CAN_SOURCE_ID_CAN_TEST              0x03


typedef union
{
	 struct{
		uint32_t start_address; // the address of the bin saved on flash.
		uint32_t length; // file real length
		uint8_t version[8];
		uint8_t type; // B:bootloader, P:boot_table, A:application, 
		uint8_t upgrade_type; //u:upgrade, 
		uint8_t reserved[6];
	}boot_table_t;
 char data[24];
 }BOOTLOADER_UNION;


typedef struct
{
	uint8_t FuncID;
	uint16_t len;
	uint8_t *pdata;
}CAN_TXDATA_STRUCT;




typedef union
{
	struct
	{
		uint32_t SourceID  : 8;
		uint32_t FUNC_ID   : 4;
		uint32_t ACK       : 1;
		uint32_t DestMACID : 8;
		uint32_t SrcMACID  : 8;
		uint32_t res       : 3;
	}CanID_Struct;
	uint32_t  CANx_ID;
}CAN_ID_UNION;

typedef union
{
	struct
	{
        uint8_t SegNum  : 6;
        uint8_t SegPolo : 2;
		uint8_t Data[7];
	}CanData_Struct;
	uint8_t CanData[8];
}CAN_DATA_UNION;

#define CAN_ONE_FRAME_DATA_LENTH    7
#define CAN_SEG_NUM_MAX             64
#define CAN_LONG_FRAME_LENTH_MAX    (CAN_ONE_FRAME_DATA_LENTH*CAN_SEG_NUM_MAX)
typedef struct
{
    uint32_t can_id;
    uint32_t start_time; 
    uint16_t used_len;
    uint8_t rcv_buf[CAN_LONG_FRAME_LENTH_MAX];   
}CAN_RCV_BUFFER_T;

typedef uint8_t (*GetOneFreeBufFn)(void);
typedef uint8_t (*GetTheBufByIdFn)(uint32_t);
typedef void (*FreeBufFn)(uint8_t);

#define CAN_LONG_BUF_NUM    2
typedef struct
{
    CAN_RCV_BUFFER_T can_rcv_buf[CAN_LONG_BUF_NUM];
    GetOneFreeBufFn GetOneFreeBuf; 
    GetTheBufByIdFn GetTheBufById;
    FreeBufFn FreeBuf;
}CAN_LONG_BUF_T;


#if 0
extern uint8_t CanUpdataBuff[64];
extern uint8_t CanRxdataBuff[64];
#endif




void RxMsgHandle(uint32_t ID,uint8_t* pdata);

void CM_CAN_Init(void);
void RxMsgHandle(uint32_t ID,uint8_t* pdata);

void CAN_SetMsg(void);

void CM_CanSetMsg(uint32_t id,uint8_t ide,uint8_t rtr,uint8_t dlc,uint8_t* pdata);
void CM_CAN_Tx( mico_can_t can_type, CAN_ID_UNION id, uint8_t* pdata, uint16_t len );

void UploadAdcData(void);

void can_protocol_period( void );
extern void CanTX(mico_can_t can_type, uint32_t CANx_ID,uint8_t* pdata,uint16_t len);
void CanLongBufInit(void);

/*******************  Bit definition for ExtId bytes  ********************/
#define EXTID_FRAME_TYPE_BITS						((uint32_t)0x10000000)
#define EXTID_SRC_ID_BITS						((uint32_t)0x0FC00000)
#define EXTID_DST_ID_BITS						((uint32_t)0x003F0000)
/* normal */
#define EXTID_PROPERTY_BITS						((uint32_t)0x0000C000)
#define EXTID_FUNC_ID_BITS						((uint32_t)0x00003F80)
#define EXTID_SEGMENT_NUM_D_BITS					((uint32_t)0x00000078)
#define EXTID_ACK_D_BITS						((uint32_t)0x00000004)
#define EXTID_END_FLAG_D_BITS						((uint32_t)0x00000002)
#define EXTID_RESEND_D_BITS						((uint32_t)0x00000001)
/* update */
#define EXTID_ACK_U_BITS						((uint32_t)0x00008000)
#define EXTID_RESEND_U_BITS						((uint32_t)0x00004000)
#define EXTID_SEGMENT_NUM_U_BITS					((uint32_t)0x00003FFF)

#define EXTID_SRC_ID_BITS_NUM						(22U)
#define EXTID_DST_ID_BITS_NUM						(16U)
#define EXTID_PROPERTY_BITS_NUM						(14U)
#define EXTID_FUNC_ID_BITS_NUM						(4U)

#define EXTID_FRAME_TYPE_DATA						(0)
#define EXTID_FRAME_TYPE_UPDATE						(1)
#endif
