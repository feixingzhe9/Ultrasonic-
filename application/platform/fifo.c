#include "fifo.h"

static uint8_t is_fifo_full(can_fifo_t *head);         //

uint8_t fifo_init(can_fifo_t *head, can_pkg_t *buf, uint32_t len)
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

void fifo_reset(can_fifo_t *head)
{
    if(head == NULL)
    {
        return;
    }
    head->front = 0;
    head->rear = 0;
}

uint8_t is_fifo_empty(can_fifo_t *head)
{
    return ((head->front == head->rear) ? TRUE : FALSE);
}

static uint8_t is_fifo_full(can_fifo_t *head)
{
    return ((head->front == ((head->rear + 1) % head->size)) ? TRUE : FALSE);
}


uint32_t get_fifo_valid_size(can_fifo_t *head)
{
    return ((head->rear < head->front)
            ? (head->rear + head->size - head->front)
            : (head->rear - head->front));
}

uint8_t put_can_pkg_to_fifo(can_fifo_t *head, const can_pkg_t data)
{
    if(head == NULL)
    {
        return FALSE;
    }
    if(is_fifo_full(head) == TRUE)
    {
        return FALSE;
    }

    memcpy(&head->data[head->rear], &data, sizeof(can_pkg_t));
    head->rear++;
    head->rear = head->rear % head->size;

    return TRUE;
}

uint8_t get_can_pkg_from_fifo(can_fifo_t *head, can_pkg_t *data)
{
    if(head == NULL)
    {
        return FALSE;
    }
    if(is_fifo_empty(head) == TRUE)
    {
        return FALSE;
    }
    memcpy(data, &head->data[head->front], sizeof(can_pkg_t));
    head->front++;
    head->front = head->front % head->size;

    return TRUE;
}
