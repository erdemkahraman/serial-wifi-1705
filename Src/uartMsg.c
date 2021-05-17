/* uart messaging related functions */

#include <string.h>
#include <stdlib.h>
#include "uartMsg.h"


/*   GLOBAL VARIABLES           */
// For debugging purposes, a default uart handle
// will be assigned to uart1
UART_HandleTypeDef *pUartHandle_default = NULL;

 /*  Function prototypes        */
static void ringbuffer_init(struct ringbuffer_s *rb); 
 

/*************    Ringbuffer related functions  ************/ 
int ringbuffer_empty(struct ringbuffer_s *rb)
{
	/* It's empty when the read and write pointers are the same. */
	if (0 == rb->fill) {
		return 1;
	}else {
		return 0;
	}
}


int ringbuffer_full(struct ringbuffer_s *rb)
{
	/* It's full when the write ponter is 1 element before the read pointer*/
	if (rb->size == rb->fill) {
		return 1;
	}else {
		return 0;
	}
}

int ringbuffer_currentSize(struct ringbuffer_s *rb)
{
	return rb->fill;
}



int ringbuffer_read(struct ringbuffer_s *rb, unsigned char* buf, unsigned int len)
{
	
	if (rb->fill >= len) {
		// in one direction, there is enough data for retrieving
		if (rb->write > rb->read) {
			memcpy(buf, rb->read, len);
			rb->read += len;
		}else if (rb->write < rb->read) {
			int len1 = rb->buffer + rb->size - 1 - rb->read + 1;
			if (len1 >= len) {
				memcpy(buf, rb->read, len);
				rb->read += len;
			} else {
				int len2 = len - len1;
				memcpy(buf, rb->read, len1);
				memcpy(buf + len1, rb->buffer, len2);
				rb->read = rb->buffer + len2; // Wrap around
			}
		}
		rb-> fill -= len;
		return len;
	} else	{
		return 0;
	}
}


int ringbuffer_write(struct ringbuffer_s *rb, unsigned char* buf, unsigned int len)
{
	
	if (rb->size - rb->fill < len) {
		return 0;
	}
	else {
		if (rb->write >= rb->read) {
			int len1 = rb->buffer + rb->size - rb->write;
			if (len1 >= len) {
				memcpy(rb->write, buf, len);
				rb->write += len;
			} else {
				int len2 = len - len1;
				memcpy(rb->write, buf, len1);
				memcpy(rb->buffer, buf+len1, len2);
				rb->write = rb->buffer + len2; // Wrap around
			}
		} else {
			memcpy(rb->write, buf, len);
			rb->write += len;
		}
		rb->fill += len;
		return len;
	}
}


static void ringbuffer_init(struct ringbuffer_s *rb)
{
	
	rb->size   = NUMBER_OF_PACKETS * PACKET_SIZE;
	memset(rb->buffer, 0, rb->size);
	rb->fill   = 0;
	rb->read   = rb->buffer;
	rb->write  = rb->buffer;

}



void uartMsg_init(struct msg_mngmnt_s *msg)
{
	msg->rxDataFlag =  BUF_EMPTY;
	msg->txReadyFlag = READY2SEND;
	msg->rxIndex     = 0;
	memset(msg->aRxBuffer_0, 0, PACKET_SIZE);
	memset(msg->aRxBuffer_1, 0, PACKET_SIZE);
	memset(msg->aTxBuffer, 0, PACKET_SIZE);	
	

	ringbuffer_init( &(msg->txRing));
	
}



// These two functions are for simple debugging purposes
void init_dgb_prints(UART_HandleTypeDef *pUart_h)
{
	pUartHandle_default = pUart_h;
}

void print_buf(UART_HandleTypeDef *pUart_h, char *buf)
{
	int len = 0;
	
  len=strlen(buf);
	
	if(pUart_h == NULL){
		HAL_UART_Transmit(pUartHandle_default, (uint8_t *)buf, len, 5000);
	}else{
		HAL_UART_Transmit(pUart_h, (uint8_t *)buf, len, 5000);
	}
	
}
