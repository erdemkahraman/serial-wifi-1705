#ifndef __uartMsg_h__
#define __uartMsg_h__

#include <stm32f1xx_hal.h>
//#include "pack.h"

#define NUMBER_OF_PACKETS       5

#define BUF_EMPTY               0
#define BUF_FULL                1

#define TX_IN_PROGRESS          0
#define READY2SEND              1

#define START_LEN       1
#define ID_LEN          2  /* target ID + source ID */
#define DAC_LEN         2
#define MSG_LEN         1
#define BUF_SIZE        10
#define STOP_LEN        1
#define PACKET_SIZE     (START_LEN + ID_LEN + DAC_LEN + MSG_LEN + BUF_SIZE + STOP_LEN)

// todo: change the buffer to an array of pointers 
//       and avoid the 2 extra memcpy operation
typedef struct ringbuffer_s {
    unsigned char buffer[NUMBER_OF_PACKETS * PACKET_SIZE];
    unsigned int size;
    unsigned int fill;
    unsigned char *read;
    unsigned char *write;
}ringbuffer_t;

typedef struct msg_mngmnt_s{
	
	__IO ITStatus rxDataFlag;
	__IO ITStatus txReadyFlag;
	
	uint8_t aRxBuffer_0[PACKET_SIZE];
	uint8_t aRxBuffer_1[PACKET_SIZE];
	uint8_t rxIndex;
	uint8_t aTxBuffer[PACKET_SIZE];
	
	struct ringbuffer_s txRing;
	
}msg_management_t;



void uartMsg_init(struct msg_mngmnt_s *msg);

int ringbuffer_empty(struct ringbuffer_s *rb);
int ringbuffer_full(struct ringbuffer_s *rb);
int ringbuffer_read(struct ringbuffer_s *rb, unsigned char* buf, unsigned int len);
int ringbuffer_write(struct ringbuffer_s *rb, unsigned char* buf, unsigned int len);
int ringbuffer_currentSize(struct ringbuffer_s *rb);

void init_uart_buffers(void);
void init_dgb_prints(UART_HandleTypeDef *pUart_h);
void print_buf(UART_HandleTypeDef *pUart_h, char *buf);



#endif
