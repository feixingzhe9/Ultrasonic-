/* 
*  Author: Adam Huang
*  Date:2017/01/23
*/

#include "can_protocol.h"
#include "protocol.h"
#include "platform.h"
#include "app_platform.h"
#include "stm32f1xx.h"
//#include "serial_leds.h"
#include "stm32f1xx_powerboard.h"
#include "Debug.h"
#include "UltraSonic.h"

#define DESTID          0x02
#define SRCID           0x06
#define CanProtocolLog(format, ...)  custom_log("can protocol", format, ##__VA_ARGS__)

__IO uint32_t flag = 0xff;
//CanTxMsgTypeDef TxMessage;
CanRxMsgTypeDef RxMessage;
CanTxMsgTypeDef TxMessage;

extern platform_can_driver_t  platform_can_drivers[];
uint8_t CanTxdataBuff[CAN_LONG_FRAME_LENTH_MAX] = {0};
#if 0
uint8_t CanUpdataBuff[64];
uint8_t CanRxdataBuff[64];

#endif
uint8_t swVersion[] = SW_VERSION;
uint8_t hwVersion[] = HW_VERSION;

CAN_TXDATA_STRUCT  CommandProcessing( uint32_t func_ID, uint8_t* dstPtr, uint8_t* pdata, uint32_t len );
//CAN_TXDATA_STRUCT FirmwareUpgrade(uint32_t ID,uint8_t* pdata,uint32_t len);

/***************************ID HANDLE BEGIN*****************************************************/
/**
  * @brief  
  * @param   
  * @retval 
	* @RevisionHistory
  */

 
typedef void (*CallBackFunc)(void);

typedef struct 
{
	uint32_t val;
	CallBackFunc callback;	
}CALLBACK_T;

const CALLBACK_T ID_table[]=
{
	{0,NULL},
	{1,NULL},
};

CALLBACK_T FuncId_table[] = {
	{0x00,NULL},
};


extern uint8_t GetKeyValue(mico_gpio_t gpio);
uint32_t my_id;
uint8_t GetCanMacId(void)
{
    uint8_t key_value = 0;
    key_value |=  GetKeyValue(MICO_GPIO_KEY_S0);
    key_value |=  GetKeyValue(MICO_GPIO_KEY_S1)<<1;
    key_value |=  GetKeyValue(MICO_GPIO_KEY_S2)<<2;
    key_value |=  GetKeyValue(MICO_GPIO_KEY_S3)<<3;
    key_value |=  GetKeyValue(MICO_GPIO_KEY_S4)<<4;
    key_value |=  GetKeyValue(MICO_GPIO_KEY_S5)<<5;
    key_value = key_value;
    if((key_value != 0) && (key_value < 0x0f))
    {
        return key_value + 0x60;
    }
    CanProtocolLog("Ultrasonic CAN MAC ID out of range ! ! ! \r\n");
    return 0x60;
        
}

/**
  * @brief  rx msg handle
  * @param   
  * @retval 
	* @RevisionHistory
  */
CALLBACK_T* FuncIdHandle(uint32_t funcid)
{
    int i;
    int func_num = sizeof(FuncId_table)/sizeof(FuncId_table[0]);
    for(i = 0;i <func_num;i++)
    {
      if(FuncId_table[i].val == funcid)
      {
        return (&FuncId_table[i]);
      }
    }
    return NULL;
}

#define ONLYONCE       0x00
#define BEGIAN         0x01
#define TRANSING       0x02
#define END            0x03


void CanTX(mico_can_t can_type, uint32_t CANx_ID,uint8_t* pdata,uint16_t len)
{
	uint16_t t_len;
	uint16_t roundCount;
	uint8_t modCount;
	CAN_DATA_UNION TxMsg;
	//CanTxMsgTypeDef *TxMessage = platform_can_drivers[can_type].handle->pTxMsg;
    
	t_len = len;
	roundCount = t_len/7;
	modCount = t_len%7;
	
	TxMessage.ExtId = CANx_ID;
	TxMessage.IDE   = CAN_ID_EXT;					 //��չģʽ
	TxMessage.RTR   = CAN_RTR_DATA;				 //���͵�������
	
	if( roundCount >= 1)
	{
		int Num;
//		if( modCount == 0)
		{
			for(Num = 0; Num < roundCount; Num++)
			{		
        //SET SEGPOLO				
				if( Num == 0)
				{
					TxMsg.CanData_Struct.SegPolo = BEGIAN;
				}
				else
				{
					TxMsg.CanData_Struct.SegPolo = TRANSING;
				}
				
				if( modCount == 0 && Num == roundCount-1)
				{
					TxMsg.CanData_Struct.SegPolo = END;
				}
							
				TxMsg.CanData_Struct.SegNum = Num;
				memcpy(TxMsg.CanData_Struct.Data,&pdata[Num*7],7);
				memcpy(TxMessage.Data,TxMsg.CanData,8);
				TxMessage.DLC = 8;
				if((CAN_USED->TSR&0x1C000000))
				{
					MicoCanMessageSend(MICO_CAN1, &TxMessage);//���ͱ���	
				}
				
				//TRANSMIT LAST MSG
				if( modCount !=0 && Num == roundCount-1 )
				{
					Num++;
					TxMsg.CanData_Struct.SegPolo = END;
					TxMsg.CanData_Struct.SegNum = Num;
                    memcpy(TxMsg.CanData_Struct.Data,&pdata[Num*7],modCount);
                    memcpy(TxMessage.Data,TxMsg.CanData,modCount+1);
					TxMessage.DLC = modCount+1;
					if((CAN_USED->TSR&0x1C000000))
					{
						MicoCanMessageSend(MICO_CAN1, &TxMessage);//
					}
				}
			}
		}
	}
	else
	{
		TxMsg.CanData_Struct.SegPolo = ONLYONCE;
		TxMessage.DLC = t_len+1;		
		
		memcpy(&TxMessage.Data[1],pdata,t_len);
		if((CAN_USED->TSR&0x1C000000))
		{
			MicoCanMessageSend(MICO_CAN1, &TxMessage);//
		}
	}
}
#if 0
CAN_TXDATA_STRUCT FirmwareUpgrade(uint32_t ID,uint8_t* pdata,uint32_t len)
{
	uint8_t PageNum=0;
	int DataNum=0; 
	uint8_t DeltAdd=0;
	uint8_t MD5Check[16]={0};
	int DataSize=0;
	FLASH_Status FlashEraseState; 
	FLASH_Status FlashWriteState; 
	CAN_TXDATA_STRUCT CanData;
	uint8_t EraseState=0;
	uint8_t WriteState=0;
	uint8_t rx_id;
	uint8_t rx_data[16];
	uint8_t tx_data[32];
	uint8_t rx_len,tx_len;
	uint8_t *ptx_data;
	int i;
	
	Md5Context MD5Ctx;
	MD5_HASH md5_ret;
	
	rx_id = ID;
	rx_len = 0 ;
	memcpy(rx_data,pdata,rx_len);
	//Éý¼¶×¼±¸
	switch(rx_id)
	{
		case 0x00:
				EraseState=0;
				DataNum=0;
				memcpy(MD5Check,&rx_data,16);
				DataSize = (rx_data[3]<<24)+(rx_data[2]<<16)+(rx_data[1]<<8)+rx_data[0];
				//³¬¹ý´óÐ¡
				if(DataSize>128*1024)
				{
					EraseState |= 0x01;
				}
				else
				{
					EraseState &= 0x0E;
					FLASH_Unlock();
					RCC_HSICmd(ENABLE);																									
					FLASH_SetLatency(FLASH_Latency_2);
					FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

					for(PageNum=0;PageNum<0x40;PageNum++)
					{
						IWDG_ReloadCounter();
						
						FlashEraseState=FLASH_ErasePage(UPDATEADDRESS+PageNum*0x800);
						if(FlashEraseState!=4)
						{
							EraseState =0x02;
							break;
						}
						else
						{
							EraseState &=0x0D;
						}
					}				
					RCC_HSICmd(DISABLE);	
				}
				tx_data[0] = ID;
				tx_data[1] = EraseState;	
				CanData.len = 2;
				ptx_data = tx_data;

				break;
	
		//Éý¼¶Êý¾Ý°ü
		case 0x01:
				WriteState=0;
				DeltAdd = 8;	//TBD
				IWDG_ReloadCounter();															
			//½«µ±Ç°½ÓÊÕµÄÊý¾ÝÓëÉÏ´ÎµÄÊý¾Ý½øÐÐ±È½Ï
				if(memcmp(rx_data,RxMessage.Data,8))
				{									
					//°ë×ÖÐ´Èë
					for(i=0;i<DeltAdd;i+=2)
					{
						unsigned short tempData;														
						RCC_HSICmd(ENABLE);
						tempData = (rx_data[i]<<8)+rx_data[i+1];
						FlashWriteState=FLASH_ProgramHalfWord(UPDATEADDRESS+DataNum*2,tempData);																											
						if(FlashWriteState!= 4)
						{
							DataNum-=i/2;
							WriteState = 0x01;
							break;
						}
						else
						{
							DataNum ++;
							WriteState &= 0x0E;
						}
					}
					
					if(DeltAdd%2)
					{
						unsigned short tempData;
						tempData = (0xFF00)+rx_data[i*2+4];
						FlashWriteState=FLASH_ProgramHalfWord(UPDATEADDRESS+DataNum*2,tempData);	
						if(FlashWriteState!= 4)
						{
							DataNum-=i/2;
							WriteState |= 0x02;
							break;
						}
						else
						{
							DataNum ++;
							WriteState &= 0x0D;
						}
						RCC_HSICmd(DISABLE);
					}
				//½«Êý¾Ý±£´æÖÁ±äÁ¿ÓÃÓÚÓëÏÂ´Î½ÓÊÕµÄÊý¾Ý½øÐÐ±È½Ï
					if(WriteState==0)
					{
						memcpy(rx_data,RxMessage.Data,8);		
					}															
				}
				else
				{
					//×Ö·û´®ÖØ¸´
					 WriteState = 0x00;
				}
				tx_data[0] = ID;
				tx_data[1] = WriteState;
				CanData.len = 2;	
				ptx_data = tx_data;				
			break;
		//Éý¼¶Íê³É
		case 0x02:
				UpdateFlag=0;
				DataNum=0;
				WriteState=0;
				Md5Initialise(&MD5Ctx);
				Md5Update( &MD5Ctx, (uint8_t *)UPDATEADDRESS, DataSize);
				Md5Finalise( &MD5Ctx, &md5_ret );
				if(memcmp(md5_ret.bytes, MD5Check, 16)!=0)
				{
						WriteState |= 0x01;																											
				}							
				else
				{
					WriteState &=0x0E;
					bootTable.boot_table_t.length = DataSize;
					bootTable.boot_table_t.start_address = UPDATEADDRESS;
					bootTable.boot_table_t.type = 'A';
					bootTable.boot_table_t.upgrade_type = 'U';
					RCC_HSICmd(ENABLE);
					//²Á³ý²ÎÊýÒ³
					for(PageNum=0;PageNum<0x02;PageNum++)
					{
						IWDG_ReloadCounter();
						FlashWriteState=FLASH_ErasePage(PARAMSADDRESS+PageNum*0x800);
						if(FlashWriteState!= 4)
						{
							DataNum-=i/2;
							WriteState |= 0x02;
							break;
						}
						else
						{
							WriteState &= 0x0D;
						}
					}
					
					if(WriteState==0x00)
					{
						//Ð´²ÎÊýÇø
						for(i=0;i<16;i+=4)
						{
							unsigned int paraData;
							paraData = (bootTable.data[i+3]<<24)+(bootTable.data[i+2]<<16)+(bootTable.data[i+1]<<8)+bootTable.data[i];
							FlashWriteState=FLASH_ProgramWord(PARAMSADDRESS+DataNum*4,paraData);
							if(FlashWriteState!= 4)
							{
								DataNum-=i/2;
								WriteState |= 0x04;
								break;
							}
							else
							{
								DataNum ++;
								WriteState &= 0x0B;
							}
						}
						
						for(i=16;i<24;i+=4)
						{
							unsigned int paraData;
							paraData = (bootTable.data[i+3]<<24)+(bootTable.data[i+2]<<16)+(bootTable.data[i+1]<<8)+bootTable.data[i];
							FlashWriteState=FLASH_ProgramWord(PARAMSADDRESS+DataNum*4,paraData);
							if(FlashWriteState!= 4)
							{
								DataNum-=i/2;
								WriteState |= 0x08;
								break;
							}
							else
							{
								DataNum ++;
								WriteState &= 0x07;
							}
						}																
					}	
					RCC_HSICmd(DISABLE);																					
				}	
				
				tx_data[1] = WriteState;	
				tx_data[0] = 0x02;												
				FLASH_Lock();
				DataNum = 0;													
				CanData.len = 2;
				ptx_data = tx_data;
			break;
				
		default :
			  rx_id = 0xFF;
			  tx_len   = 0x06;
			  ptx_data = tx_data;
			break;
	}
	CanData.len = tx_len;
	CanData.FuncID = rx_id;
	CanData.pdata = ptx_data;
	return CanData;
}
#endif
/*******************************ID HANDLE END*************************************************/
#if 0
CAN_TXDATA_STRUCT  CommandProcessing( uint32_t func_ID, uint8_t* dstPtr, uint8_t* pdata, uint32_t len )
{
    CAN_TXDATA_STRUCT CanData;
    
//    CM_CAN_ID_T TxData;
    uint8_t rx_id;
    uint8_t rx_data[8];
    uint8_t* tx_data;
    uint8_t rx_len,tx_len;
    uint8_t *ptx_data;
    
    rx_id = func_ID;
    rx_len = len ;
    
    tx_data = dstPtr;
    memcpy( rx_data, pdata, rx_len );
    
    switch( rx_id )
    {
        case CAN_CMD_READ_VERSION:
          
          memcpy( tx_data, hwVersion, 2 );
          memcpy( &tx_data[4], swVersion, 11 );
          tx_len = 1;
          ptx_data = tx_data;
          break;
        case CAN_CMD_LEDS_CONTROL:
         // *tx_data++ = serial_leds->modeType;
        //  *tx_data++ = (serial_leds->effectType & 0xff00) >> 8;
         // *tx_data++ = (serial_leds->effectType & 0x00ff);
          tx_len = 3;
          break;
        default :
          break;
    }
  
    CanData.FuncID = rx_id ;
    CanData.len    = tx_len;
    CanData.pdata  = ptx_data;

    return CanData;
}
#endif

//////  function id define  //////
#define CAN_FUN_ID_RESET        0x06
#define CAN_FUN_ID_WRITE        0x01
#define CAN_FUN_ID_READ         0x02
#define CAN_FUN_ID_TRIGGER      0x03


//////  source id define  //////
#define CAN_SOURCE_ID_READ_VERSION  0x01


#define CAN_READ_DATA               0x80
uint16_t CmdProcessing(CAN_ID_UNION *id, uint8_t *data_in, uint16_t data_len, uint8_t *data_out)
{
    id->CanID_Struct.ACK = 1;
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
              //SW_VERSION
              break;
            case CAN_READ_DATA:
                *data_out = UltraSonicGetMeasureData();
                
                return 1;
            default :
              break;
          }
      
          
        default: 
          break;
    }
    
    return 0;
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
}

#define CAN_LONG_FRAME_TIME_OUT     5000/SYSTICK_PERIOD

#define CAN_COMM_TIME_OUT           5000
uint32_t can_comm_start_time;
void can_protocol_period( void )
{
    
    if( platform_can_drivers[MICO_CAN1].rx_complete == 0 )
    {
      return;
    }
    
   
    CAN_ID_UNION id;
    uint16_t tx_len;
    //uint8_t test_data[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
    CAN_DATA_UNION rx_buf;
    uint8_t buf_index;
    uint8_t seg_polo;
    uint8_t seg_num;
    
    memcpy( &RxMessage, platform_can_drivers[MICO_CAN1].handle->pRxMsg, sizeof(CanRxMsgTypeDef) );
    memcpy(&rx_buf, RxMessage.Data, RxMessage.DLC);
    
    seg_polo = rx_buf.CanData_Struct.SegPolo;
    seg_num = rx_buf.CanData_Struct.SegNum;
    
    if(rx_buf.CanData_Struct.SegPolo == ONLYONCE)
    {
        id.CANx_ID = RxMessage.ExtId;
        //if( (id.CanID_Struct.SourceID < SOURCE_ID_PREPARE_UPDATE) && (id.CanID_Struct.SourceID > SOURCE_ID_CHECK_TRANSMIT) )
        {
                //process the data here//
                tx_len = CmdProcessing(&id, rx_buf.CanData_Struct.Data, RxMessage.DLC - 1, CanTxdataBuff);
                //process the data here//
                
                CanTX( MICO_CAN1, id.CANx_ID, CanTxdataBuff, tx_len );
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
            buf_index = can_long_frame_buf->GetTheBufById(RxMessage.ExtId);
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
            can_long_frame_buf->can_rcv_buf[buf_index].can_id = RxMessage.ExtId;
            can_long_frame_buf->can_rcv_buf[buf_index].start_time = os_get_time();
        }
        else if((seg_polo == TRANSING) || (seg_polo == END))
        {
            buf_index = can_long_frame_buf->GetTheBufById(RxMessage.ExtId);
            if((buf_index == CAN_BUF_NO_THIS_ID) || (buf_index >= CAN_LONG_BUF_NUM))
            {
                CanProtocolLog("ERROR ! !\r\n");
                goto exit;
            }
            can_long_frame_buf->can_rcv_buf[buf_index].start_time = os_get_time();
            if(seg_polo == TRANSING)
            {
                memcpy(&can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf[seg_num*CAN_ONE_FRAME_DATA_LENTH], rx_buf.CanData_Struct.Data, CAN_ONE_FRAME_DATA_LENTH);
                can_long_frame_buf->can_rcv_buf[buf_index].used_len += CAN_ONE_FRAME_DATA_LENTH;
            }
            if(seg_polo == END)
            {
                memcpy(&can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf[seg_num*CAN_ONE_FRAME_DATA_LENTH], rx_buf.CanData_Struct.Data, RxMessage.DLC - 1);
                can_long_frame_buf->can_rcv_buf[buf_index].used_len += RxMessage.DLC - 1; 
                
                //process the data here//
                /**********************/
                //process the data here//
                
                CanTX( MICO_CAN1, id.CANx_ID, can_long_frame_buf->can_rcv_buf[buf_index].rcv_buf, can_long_frame_buf->can_rcv_buf[buf_index].used_len);  // test :send the data back;             
                can_long_frame_buf->FreeBuf(buf_index);
            }       
        }
    }
    
    
#if 0
    else if( CanData.FrameType == 1 )
    {
        if( CanData.CanID_U.SegmentNum <= 10 )//update prepare
        {
            if( CanData.CanID_U.ID != 0 )//muti data process
            {
                CANUpDataLen = CanData.CanID_U.SegmentNum*8;
                memcpy( &CanUpdataBuff[CanData.CanID_U.SegmentNum*8], RxMessage.Data, RxMessage.DLC );
            }
            else
            {
                CANUpDataLen = CanData.CanID_U.SegmentNum*8 + RxMessage.DLC;
                memcpy( &CanUpdataBuff[CanData.CanID_U.SegmentNum*8], RxMessage.Data, RxMessage.DLC );				
                //update cmd process
                TxCanData = FirmwareUpgrade( CanData.CanID_U.ID, CanUpdataBuff, CANUpDataLen );
                CM_CAN_Tx( MICO_CAN1, CanData, TxCanData.pdata, TxCanData.len );
                CANUpDataLen = 0;
            }
        }
        else//update data package
        {
            TxCanData = FirmwareUpgrade( CanData.CanID_U.ID, CanUpdataBuff, CANUpDataLen );
            CM_CAN_Tx( MICO_CAN1, CanData, TxCanData.pdata, TxCanData.len );
        }
    }
#endif
exit:    
    platform_can_drivers[MICO_CAN1].rx_complete = 0;
    return;
}






