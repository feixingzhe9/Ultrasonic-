
#include "fifo.h"


static uint32_t FifoSurplusSize(can_fifo_t *head); //
static uint8_t IsFifoFull(can_fifo_t *head);         //


uint8_t FifoInit(can_fifo_t *head, can_pkg_t *buf, uint32_t len)
{
    if(head == NULL)
    {
        return FALSE;
    }
    head->data = buf;
    head->size = len;
    head->front = head->rear = 0;

    return TRUE;
}

void FifoRst(can_fifo_t *head)
{
    if(head == NULL)
    {
        return;
    }
    head->front = 0;
	head->rear = 0;
}

uint8_t IsFifoEmpty(can_fifo_t *head)
{    
    return ((head->front == head->rear) ? TRUE : FALSE);
}

static uint8_t IsFifoFull(can_fifo_t *head)
{   
    return ((head->front == ((head->rear + 1) % head->size)) ? TRUE : FALSE);
}


uint32_t FifoValidSize(can_fifo_t *head)
{
	return ((head->rear < head->front)
			? (head->rear + head->size - head->front)
			: (head->rear - head->front));
}



uint8_t FifoPutCanPkg(can_fifo_t *head, const can_pkg_t data)
{
    if(head == NULL)
    {
        return FALSE;
    }
    if(IsFifoFull(head) == TRUE)
    {
        return FALSE;
    }

    memcpy(&head->data[head->rear], &data, sizeof(can_pkg_t));
    head->rear++;
    head->rear = head->rear % head->size;

    return TRUE;   
}


uint8_t FifoGetCanPkg(can_fifo_t *head, can_pkg_t *data)
{
    if(head == NULL)
    {
        return FALSE;
    }
    if(IsFifoEmpty(head) == TRUE)
    {
        return FALSE;
    }
    memcpy(data, &head->data[head->front], sizeof(can_pkg_t));
    head->front++;
    head->front = head->front % head->size;

    return TRUE;   
}

