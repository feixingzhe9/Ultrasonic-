/*
 *  Author: Adam Huang
 *  Date:2017/01/23
 */

#ifndef __CAN_PROTOOCOL_H
#define __CAN_PROTOOCOL_H

#include "mico.h"

#define CAN_USED    CAN1

#define HW_VERSION                      "12"
#define SW_VERSION                      "US_HC_SR_04_V006"
#define PROTOCOL_VERSION                "20170505R0101"

#define DESTID          0x02
//#define CAN_SUB_PB_ID   0x51
#define SRCID           0x06

#define ULTRASONIC_SRC_ID_BASE          0x60

#define CAN_CMD_READ_VERSION            0x01

#define SOURCE_ID_PREPARE_UPDATE        0x10
#define SOURCE_ID_TRANSMIT_UPDATE       0x11
#define SOURCE_ID_CHECK_TRANSMIT        0x12

#define CAN_FIFO_SIZE                   5

//////  function id define  //////
#define CAN_FUN_ID_RESET        0x06
#define CAN_FUN_ID_WRITE        0x01
#define CAN_FUN_ID_READ         0x02
#define CAN_FUN_ID_TRIGGER      0x03

//////  source id define  //////
#define CAN_SOURCE_ID_READ_VERSION      0x01

#define CAN_SOURCE_ID_READ_MEASURE_DATA      0x80
#define CAN_SOURCE_ID_MEASUREMENT_EN         0x81
#define CAN_SOURCE_ID_GET_VERSION           0x82
#define CAN_SOURCE_ID_SET_GROUP             0x83

#define CAN_SOURCE_ID_CAN_TEST              0x03

typedef struct
{
    uint8_t FuncID;
    uint16_t len;
    uint8_t *pdata;
}can_tx_data_t;

typedef union
{
    struct
    {
        uint32_t source_id  : 8;
        uint32_t func_id   : 4;
        uint32_t ack       : 1;
        uint32_t dest_mac_id : 8;
        uint32_t src_mac_id  : 8;
        uint32_t res       : 3;
    }canx_id_t;
    uint32_t  canx_id;
}can_id_union;

typedef union
{
    struct
    {
        uint8_t seg_num  : 6;
        uint8_t seg_polo : 2;
        uint8_t Data[7];
    }can_data_t;
    uint8_t can_data[8];
}can_data_union;

#define CAN_ONE_FRAME_DATA_LENTH    7
#define CAN_SEG_NUM_MAX             64
#define CAN_LONG_FRAME_LENTH_MAX    (CAN_ONE_FRAME_DATA_LENTH*CAN_SEG_NUM_MAX)
typedef struct
{
    uint32_t can_id;
    uint32_t start_time;
    uint16_t used_len;
    uint8_t rcv_buf[CAN_LONG_FRAME_LENTH_MAX];
}can_rcv_buf_t;

typedef uint8_t (*GetOneFreeBufFn)(void);
typedef uint8_t (*GetTheBufByIdFn)(uint32_t);
typedef void (*FreeBufFn)(uint8_t);

#define CAN_LONG_BUF_NUM    2
typedef struct
{
    can_rcv_buf_t can_rcv_buf[CAN_LONG_BUF_NUM];
    GetOneFreeBufFn get_one_free_buf;
    GetTheBufByIdFn get_the_buf_by_id;
    FreeBufFn free_buf;
}can_long_buf_t;


void can_protocol_period( void );
extern void tx_can_data(mico_can_t can_type, uint32_t canx_id,uint8_t* pdata,uint16_t len);
void can_long_buf_init(void);

#endif
