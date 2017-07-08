#ifndef __FIFO_H
#define __FIFO_H
#include "Common.h"

#include "can_protocol.h"

typedef struct
{
    CAN_ID_UNION    id;
    CAN_DATA_UNION  data;
    uint8_t len;
}can_pkg_t;

typedef struct{
    can_pkg_t   *data;
    uint32_t   size;
    uint32_t   front;
    uint32_t   rear;
}can_fifo_t;

                              
#define TRUE    1
#define FALSE   0

                                  
                              
extern can_fifo_t *can_fifo;


uint8_t FifoInit(can_fifo_t *head, can_pkg_t *buf, uint32_t len);
void FifoRst(can_fifo_t *head);
uint8_t IsFifoEmpty(can_fifo_t *head);
static uint32_t FifoSurplusSize(can_fifo_t *head);
uint32_t FifoValidSize(can_fifo_t *head);
uint8_t FifoPuts(can_fifo_t *head, uint8_t *data, uint32_t len);
uint8_t FifoGets(can_fifo_t *head, uint8_t *data, uint32_t len);
uint8_t FifoPutCanPkg(can_fifo_t *head, const can_pkg_t data);
uint8_t FifoGetCanPkg(can_fifo_t *head, can_pkg_t *data);
							
#endif //queue.h end
/**************************Copyright BestFu 2014-05-14*************************/
