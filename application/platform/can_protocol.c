/* 
*  Author: Adam Huang
*  Date:2017/01/23
*/

#include "can_protocol.h"
#include "protocol.h"
#include "platform.h"
#include "app_platform.h"
#include "stm32f1xx.h"
#include "stm32f1xx_powerboard.h"
#include "Debug.h"
#include "UltraSonic.h"

#include "fifo.h"

#include "upgrade_flash.h"



#define CanProtocolLog(format, ...)  custom_log("can protocol", format, ##__VA_ARGS__)

__IO uint32_t flag = 0xff;

//CanRxMsgTypeDef RxMessage;
CanTxMsgTypeDef TxMessage;

extern platform_can_driver_t  platform_can_drivers[];
uint8_t CanTxdataBuff[CAN_LONG_FRAME_LENTH_MAX] = {0};



uint8_t swVersion[] = SW_VERSION;
uint8_t hwVersion[] = HW_VERSION;

CAN_TXDATA_STRUCT  CommandProcessing( uint32_t func_ID, uint8_t* dstPtr, uint8_t* pdata, uint32_t len );
//CAN_TXDATA_STRUCT FirmwareUpgrade(uint32_t ID,uint8_t* pdata,uint32_t len);

can_fifo_t can_fifo_ram;
can_fifo_t *can_fifo = &can_fifo_ram;

can_pkg_t can_pkg[CAN_FIFO_SIZE] = {0};

 



#if 1
extern uint8_t GetKeyValue(mico_gpio_t gpio);
//uint32_t my_id;
uint8_t GetCanSrcId(void)
{
  
#define DEBOUNCE_TIME       100/SYSTICK_PERIOD
    uint8_t new_key_value = 0;
    uint8_t old_key_value = 0;
    uint8_t tmp_1 = 0;
    uint8_t tmp_2 = 0;
    uint8_t tmp_3 = 0;
    uint8_t tmp_4 = 0;
    uint8_t tmp_5 = 0;
    uint8_t tmp_6 = 0;
    
    static uint32_t start_time = 0;
    start_time = os_get_time();
    while(os_get_time() - start_time <= DEBOUNCE_TIME)
    {
        old_key_value = new_key_value;
#if 1       
        tmp_1 = GetKeyValue(MICO_GPIO_KEY_S0);
        tmp_2 = GetKeyValue(MICO_GPIO_KEY_S1);
        tmp_3 = GetKeyValue(MICO_GPIO_KEY_S2); 
        tmp_4 = GetKeyValue(MICO_GPIO_KEY_S3);
        //tmp_5 = GetKeyValue(MICO_GPIO_KEY_S4);
        //tmp_6 = GetKeyValue(MICO_GPIO_KEY_S5);
        new_key_value = tmp_1 | (tmp_2 << 1) | (tmp_3 << 2) | (tmp_4 << 3) | (tmp_5 << 4) | (tmp_6 << 5);        
#else
        new_key_value |=  GetKeyValue(MICO_GPIO_KEY_S0);
        new_key_value |=  GetKeyValue(MICO_GPIO_KEY_S1)<<1;
        new_key_value |=  GetKeyValue(MICO_GPIO_KEY_S2)<<2;
        new_key_value |=  GetKeyValue(MICO_GPIO_KEY_S3)<<3;
        new_key_value |=  GetKeyValue(MICO_GPIO_KEY_S4)<<4;
        new_key_value |=  GetKeyValue(MICO_GPIO_KEY_S5)<<5;
#endif   
        if(new_key_value != old_key_value)
        {
            start_time = os_get_time();
        }     
    }

      
    if((new_key_value != 0) && (new_key_value <= 0x0f))
    {
        return new_key_value + ULTRASONIC_SRC_ID_BASE - 1;
    }
    CanProtocolLog("Ultrasonic CAN MAC ID out of range ! ! ! \r\n");
    return 0x60;
        
}
#endif


#define ONLYONCE       0x00
#define BEGIAN         0x01
#define TRANSING       0x02
#define END            0x03


void CanTX(mico_can_t can_type, uint32_t CANx_ID,uint8_t* pdata,uint16_t len)
{
  //return ;
	uint16_t t_len;
	uint16_t roundCount;
	uint8_t modCount;
	CAN_DATA_UNION TxMsg = {0};
	//CanTxMsgTypeDef *TxMessage = platform_can_drivers[can_type].handle->pTxMsg;
    
	t_len = len;
	roundCount = t_len/7;
	modCount = t_len%7;
	
	TxMessage.ExtId = CANx_ID;
	TxMessage.IDE   = CAN_ID_EXT;					 //扩展模式
	TxMessage.RTR   = CAN_RTR_DATA;				 //发送的是数据
	//if(roundCount <= 1)
    if(t_len <= 7)
    {
        TxMsg.CanData_Struct.SegPolo = ONLYONCE;
        TxMessage.DLC = t_len+1;		
        
        
        memcpy(&TxMessage.Data[1],pdata,t_len);
        TxMessage.Data[0] = TxMsg.CanData[0];
        
        if((CAN_USED->TSR&0x1C000000))
        {
            MicoCanMessageSend(MICO_CAN1, &TxMessage);//
        }
        else
        {
            CanProtocolLog("TX busy ! \r\n");
        }
        return ;
    }
    
	{
		int num;
        {
            for(num = 0; num < roundCount; num++)
            {		
        //SET SEGPOLO				
                if( num == 0)
                {
                    TxMsg.CanData_Struct.SegPolo = BEGIAN;
                }
                else
                {
                    TxMsg.CanData_Struct.SegPolo = TRANSING;
                }
                
                if( modCount == 0 && num == roundCount-1)
                {
                    TxMsg.CanData_Struct.SegPolo = END;
                }
                            
                TxMsg.CanData_Struct.SegNum = num;
                memcpy(TxMsg.CanData_Struct.Data, &pdata[num*7], 7);
                memcpy(TxMessage.Data, TxMsg.CanData, 8);
                TxMessage.DLC = 8;
                if((CAN_USED->TSR&0x1C000000))
                {
                    MicoCanMessageSend(MICO_CAN1, &TxMessage);//发送报文	
                }
                else
                {
                    CanProtocolLog("TX busy ! \r\n");
                }
                
                //TRANSMIT LAST MSG
                if( modCount !=0 && num == roundCount-1 )
                {
                    num++;
                    TxMsg.CanData_Struct.SegPolo = END;
                    TxMsg.CanData_Struct.SegNum = num;
                    memcpy(TxMsg.CanData_Struct.Data,&pdata[num*7],modCount);
                    memcpy(TxMessage.Data,TxMsg.CanData,modCount+1);
                    TxMessage.DLC = modCount+1;
                    if((CAN_USED->TSR&0x1C000000))
                    {
                        MicoCanMessageSend(MICO_CAN1, &TxMessage);//
                    }
                    else
                    {
                        CanProtocolLog("TX busy ! \r\n");
                    }
                }
            }
            
        }
        
	}
}


#define CMD_NOT_FOUND   0
extern uint32_t ultrasonic_src_id;
static uint32_t can_test_cnt = 0;

uint16_t CmdProcessing(CAN_ID_UNION *id, const uint8_t *data_in, const uint16_t data_in_len, uint8_t *data_out)
{
    //uint8_t data_out_len;
    id->CanID_Struct.ACK = 1;   
    id->CanID_Struct.DestMACID = id->CanID_Struct.SrcMACID;
    id->CanID_Struct.SrcMACID = ultrasonic_src_id;
    id->CanID_Struct.res = 0;
    //id->CanID_Struct.FUNC_ID = 
    
     
    switch(id->CanID_Struct.FUNC_ID)
    {
        case CAN_FUN_ID_RESET:
            platform_mcu_reset();
            break;
        case CAN_FUN_ID_WRITE:
        case CAN_FUN_ID_READ:
            switch(id->CanID_Struct.SourceID)
            {
                case CAN_SOURCE_ID_READ_VERSION:
                    data_out[0] = data_in[0];
                    if(data_in[0] == 1)//read software version
                    {
                        memcpy(&data_out[1], SW_VERSION, sizeof(SW_VERSION));
                        //return strlen(SW_VERSION) + 1;
                        return sizeof(SW_VERSION) + 1;
                    }
                    else if(data_in[0] == 2)//protocol version
                    {
                        memcpy(&data_out[1], PROTOCOL_VERSION, sizeof(PROTOCOL_VERSION));
                        return sizeof(PROTOCOL_VERSION) +1;
                        
                    }
                    else if(data_in[0] == 3)//hardware version
                    {
                        memcpy(&data_out[1], HW_VERSION, sizeof(HW_VERSION));
                        return sizeof(HW_VERSION) + 1;
                    }
                    return CMD_NOT_FOUND;
                    break;
                case CAN_SOURCE_ID_READ_MEASURE_DATA:
#if 0
                    {
                        uint16_t tmp;
                        tmp =  UltraSonicGetMeasureData();
                        memcpy(&data_out[0], (uint8_t *)&tmp,sizeof(tmp));
                        return  sizeof(tmp);
                    }                   
#else
                    if(ultra_sonic_data->start_flag == 0)
                    {
                        //UltraSonicStart();
                    }
                    
                    return 0;
#endif                   
                    break;
                case CAN_SOURCE_ID_MEASUREMENT_EN:
                    if(data_in_len == 1)
                    {
                        if(data_in[0] == 1)
                        {
                            ultra_sonic_data->i_am_en = true;
                        }
                        else if(data_in[0] == 0)
                        {
                            ultra_sonic_data->i_am_en = false;
                        }
                        data_out[0] = data_in[0];
                        return 1;
                    }
                    
                    break;
                case CAN_SOURCE_ID_GET_VERSION:
                    if(data_in_len == 1)
                    {
                        memcpy(&data_out[1],SW_VERSION,sizeof(SW_VERSION));
                        data_out[0] = sizeof(SW_VERSION);
                        return (data_out[0] + 1);
                    }
                    
                    break;   
                    
                case CAN_SOURCE_ID_SET_GROUP:
                    if(data_in_len == 1)
                    {
                        ultra_sonic_data->group = data_in[0];
                        data_out[0] = ultra_sonic_data->group;
                        return 1;
                    }
                    
                    break;  
                    
                    
                case CAN_SOURCE_ID_CAN_TEST:
                    can_test_cnt++;
                    memcpy(&data_out[0], (uint8_t *)&can_test_cnt, sizeof(can_test_cnt));
                    return sizeof(can_test_cnt);
                default :
                    break;
            }


        default: 
        break;
    }

    return CMD_NOT_FOUND;
}

CAN_LONG_BUF_T can_long_frame_buf_ram;
CAN_LONG_BUF_T *can_long_frame_buf = &can_long_frame_buf_ram;

#define CAN_LONG_BUF_FULL   0xff
static uint8_t GetOneFreeBuf(void)
{
    for(uint8_t i = 0; i < CAN_LONG_BUF_NUM; i++)
    {
        if(can_long_frame_buf->can_rcv_buf[i].used_len == 0)
        {
            return i;
        }
    }
    return CAN_LONG_BUF_FULL;
}
static void FreeBuf(uint8_t index)
{
    can_long_frame_buf->can_rcv_buf[index].can_id = 0;
    can_long_frame_buf->can_rcv_buf[index].used_len = 0;
}
#define CAN_BUF_NO_THIS_ID      0xfe
static uint8_t GetTheBufById(uint32_t id)
{
    for(uint8_t i = 0; i < CAN_LONG_BUF_NUM; i++)
    {
        if(id == can_long_frame_buf->can_rcv_buf[i].can_id)
        {
            return i;
        }
    }
    return CAN_BUF_NO_THIS_ID;
}
void CanLongBufInit(void)
{ 
    can_long_frame_buf->GetOneFreeBuf = GetOneFreeBuf;
    can_long_frame_buf->GetTheBufById = GetTheBufById;
    can_long_frame_buf->FreeBuf = FreeBuf;
    
    //my_id = GetCanMacId();//test 
    
    FifoInit(can_fifo, can_pkg, CAN_FIFO_SIZE);
}





void canAckBack(uint32_t CANx_ID, const uint8_t * const pdata, uint16_t len)
{
  uint16_t t_len;
  CanTxMsgTypeDef TxMessage;
  CAN_ID_UNION id;
  uint8_t src_mac_id_temp;
  CAN_DATA_UNION TxMsg;
  
  id.CANx_ID = CANx_ID;
  id.CanID_Struct.ACK = 1;
  src_mac_id_temp = id.CanID_Struct.DestMACID;
  id.CanID_Struct.DestMACID = id.CanID_Struct.SrcMACID;
  id.CanID_Struct.SrcMACID = src_mac_id_temp;
  
  TxMessage.ExtId = id.CANx_ID;
  TxMessage.IDE   = CAN_ID_EXT;					 //扩展模式
  TxMessage.RTR   = CAN_RTR_DATA;				 //发送的是数据
  
  t_len = len;
  if( t_len <=7 )
  {
      TxMsg.CanData_Struct.SegPolo = ONLYONCE;
      TxMsg.CanData_Struct.SegNum = 0;
      memcpy( TxMsg.CanData_Struct.Data, (const void *)pdata, t_len );
      memcpy( TxMessage.Data, TxMsg.CanData, t_len + 1 );
      
      TxMessage.DLC = t_len + 1;
      if( (CAN_USED->TSR & 0x1C000000) )
      {
          MicoCanMessageSend(MICO_CAN1, &TxMessage );//
      }
  }
}
static OSStatus upgradePrepareProcess(CAN_ID_UNION id, uint8_t *md5, uint8_t *firmware_Size )
{
  OSStatus err = kNoErr;
  uint32_t firmwareSize;
  mico_logic_partition_t *ota_partition_info;
  uint8_t ack;
  
  ota_partition_info = MicoFlashGetInfo( MICO_PARTITION_OTA_TEMP );
  require_action( ota_partition_info->partition_owner != MICO_FLASH_NONE, exit, err = kUnsupportedErr );

  firmwareSize = ReadBig32(firmware_Size);
  if( firmwareSize > ota_partition_info->partition_length )
  {
    ack = 0x01;
    canAckBack(id.CANx_ID, &ack, 1);
    CanProtocolLog( "not enough storage" );
    goto exit;
  }
  CanProtocolLog( "firmwareSize is:%d", firmwareSize );
  if( !upgradePrepareFlash( md5, firmwareSize ) )
  {
    ack = 0x00;
    canAckBack(id.CANx_ID, &ack, 1);
    CanProtocolLog( "mcu prepare ok" );
  }
  else
  {
    ack = 0x02;
    canAckBack(id.CANx_ID, &ack, 1);
    CanProtocolLog( "mcu retry later" );
  }

exit:
  return err;  
}

static OSStatus upgradeFirmwareRecevingProcess( CAN_ID_UNION *id,  uint8_t *rx_data, uint8_t dataLen)
{
  OSStatus err = kNoErr;
  uint32_t packageDataLength = dataLen;
  uint8_t ack;
  if( !upgradeWriteFlashData( (uint32_t *)(rx_data + 2), packageDataLength - 2 ) )
  {
    ack = 0x00;
    canAckBack(id->CANx_ID, rx_data, 2);
  }
  else
  {
    ack = 0x01;
    canAckBack(id->CANx_ID, &ack, 1);
    CanProtocolLog( "mcu write data failed" );
    goto exit;
  }
exit:
  return err;
}

static OSStatus upgradeFinishCheckProcess( CAN_ID_UNION *id )
{
  OSStatus err = kNoErr;
  uint8_t ack;
  
  if( !upgradeCheckFlash() )
  {
    ack = 0x00;
    canAckBack(id->CANx_ID, &ack, 1);
    CanProtocolLog("MD5 success,sent right ack");
    platform_mcu_reset();
  }
  else
  {
    ack = 0x01;
    canAckBack(id->CANx_ID, &ack, 1);
    CanProtocolLog("MD5 err,sent err ack");
  }

  return err;
}







#define CAN_LONG_FRAME_TIME_OUT     5000/SYSTICK_PERIOD

#define CAN_COMM_TIME_OUT           1000
uint32_t can_comm_start_time;
void can_protocol_period( void )
{
  
    if(os_get_time() - can_comm_start_time >= CAN_COMM_TIME_OUT)
    {
        HAL_CAN_DeInit(platform_can_drivers[MICO_CAN1].handle);
        MicoCanInitialize( MICO_CAN1 );
        can_comm_start_time = os_get_time();
    }
    while(IsFifoEmpty(can_fifo) == FALSE)
    {  
        CAN_ID_UNION id;
        can_pkg_t can_pkg_tmp;
        uint16_t tx_len;
        CAN_DATA_UNION rx_buf;
        uint8_t buf_index;
        uint8_t seg_polo;
        uint8_t seg_num;
        uint8_t rx_data_len;
        
        FifoGetCanPkg(can_fifo, &can_pkg_tmp);
        
        memcpy(rx_buf.CanData,  can_pkg_tmp.data.CanData, can_pkg_tmp.len);
        id.CANx_ID = can_pkg_tmp.id.CANx_ID;
        seg_polo = can_pkg_tmp.data.CanData_Struct.SegPolo;
        seg_num = can_pkg_tmp.data.CanData_Struct.SegNum;
        rx_data_len = can_pkg_tmp.len;
        can_comm_start_time = os_get_time();
        
        
        
    
        if( id.CanID_Struct.SourceID == 0x10 )//update_prepare
        {
            static uint8_t md5[16];
            static uint8_t firmwareSize[4];

            //memcpy( &rx_buf, RxMessage.Data, RxMessage.DLC );
            if( rx_buf.CanData_Struct.SegPolo == BEGIAN )
            {
                memcpy(&md5[0], rx_buf.CanData_Struct.Data, 7 );
            }
            if( rx_buf.CanData_Struct.SegPolo == TRANSING )
            {
                memcpy(&md5[7], rx_buf.CanData_Struct.Data, 7 );
            }
            if( rx_buf.CanData_Struct.SegPolo == END )
            {
                memcpy(&md5[14], &rx_buf.CanData_Struct.Data[0], 2 );
                memcpy(&firmwareSize[0], &rx_buf.CanData_Struct.Data[2], 4 );
                upgradePrepareProcess(id, md5, firmwareSize);
            }
            //goto exit;
            continue;
        }
        if( id.CanID_Struct.SourceID == 0x11 )//update_receicing
        {
            upgradeFirmwareRecevingProcess(&id, rx_buf.CanData, rx_data_len);
            //goto exit;
            continue;
        }
        if( id.CanID_Struct.SourceID == 0x12 )//update_finish_check
        {
            upgradeFinishCheckProcess(&id);
            //goto exit;
            continue;
        }


        
        if(seg_polo == ONLYONCE)
        {
            //if( (id.CanID_Struct.SourceID < SOURCE_ID_PREPARE_UPDATE) && (id.CanID_Struct.SourceID > SOURCE_ID_CHECK_TRANSMIT) )
            if(id.CanID_Struct.DestMACID == ultrasonic_src_id)
            {
                tx_len = CmdProcessing(&id, rx_buf.CanData_Struct.Data, rx_data_len - 1, CanTxdataBuff );
                //process the data here//
                
                if(tx_len > 0)
                {
                    CanTX( MICO_CAN1, id.CANx_ID, CanTxdataBuff, tx_len );
                }        
            }
        }
        else //long frame
        {
            for(uint8_t i = 0; i < CAN_LONG_BUF_NUM; i++)
            {
                if(can_long_frame_buf->can_rcv_buf[i].used_len > 0)
                {
                    if(os_get_time() - can_long_frame_buf->can_rcv_buf[i].start_time > CAN_LONG_FRAME_TIME_OUT)
                    {
                        can_long_frame_buf->FreeBuf(i);
                    }
                }     
            }
            
            if(seg_polo == BEGIAN)
            {
                buf_index = can_long_frame_buf->GetTheBufById(id.CANx_ID);
                if(buf_index == CAN_BUF_NO_THIS_ID)
                {
                    buf_index = can_long_frame_buf->GetOneFreeBuf();
                }
                else
                {
                    //
                }
                
                if((buf_index == CAN_LONG_BUF_FULL) || (buf_index >= CAN_LONG_BUF_NUM))
                {
                    CanProtocolLog("LONG FRAME RCV BUF IS FULL! ! ! !\r\n");
                    
                    goto exit;
                }
                memcpy(&can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf[0], rx_buf.CanData_Struct.Data, CAN_ONE_FRAME_DATA_LENTH);
                can_long_frame_buf->can_rcv_buf[buf_index].used_len = CAN_ONE_FRAME_DATA_LENTH;
                can_long_frame_buf->can_rcv_buf[buf_index].can_id = id.CANx_ID;
                can_long_frame_buf->can_rcv_buf[buf_index].start_time = os_get_time();
                CanProtocolLog("begin\r\n");
            }
            else if((seg_polo == TRANSING) || (seg_polo == END))
            {
                buf_index = can_long_frame_buf->GetTheBufById(id.CANx_ID);
                if((buf_index == CAN_BUF_NO_THIS_ID) || (buf_index >= CAN_LONG_BUF_NUM))
                {
                    CanProtocolLog("ERROR ! !\r\n long buff index is %d",buf_index);
                    goto exit;
                }
                can_long_frame_buf->can_rcv_buf[buf_index].start_time = os_get_time();
                if(seg_polo == TRANSING)
                {
                    memcpy(&can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf[seg_num*CAN_ONE_FRAME_DATA_LENTH], rx_buf.CanData_Struct.Data, CAN_ONE_FRAME_DATA_LENTH);
                    can_long_frame_buf->can_rcv_buf[buf_index].used_len += CAN_ONE_FRAME_DATA_LENTH;
                    CanProtocolLog("transing\r\n");
                }
                if(seg_polo == END)
                {
                    memcpy(&can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf[seg_num*CAN_ONE_FRAME_DATA_LENTH], rx_buf.CanData_Struct.Data, rx_data_len - 1);
                    can_long_frame_buf->can_rcv_buf[buf_index].used_len += rx_data_len - 1; 
                    
                    //process the data here//
                    /**********************/
                    //process the data here//
                    
                    CanTX( MICO_CAN1, id.CANx_ID, can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf, can_long_frame_buf->can_rcv_buf[buf_index].used_len);  // test :send the data back;             
                    can_long_frame_buf->FreeBuf(buf_index);
                    CanProtocolLog("end\r\n");
                }       
            }
        }

    }
   
    
exit:    
    return;
}






