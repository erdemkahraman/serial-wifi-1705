/*
 * uart_mux.h
 *
 *  Created on: 1 Tem 2019
 *      Author: abdullah
 */

#ifndef UART_MUX_H_
#define UART_MUX_H_

#include "stm32f1xx_hal.h"


#define POS_24	0
#define POS_34	1

typedef enum {
  S0  			  	= 0,
  S1				= 1,

  Select__Max_ 	= 2,

}A_Channel_def_t;

typedef struct gpioSettings_s {
    uint16_t      GPIO_Pin;
    GPIO_TypeDef* GPIOx;
} gpioSettings_t;

extern gpioSettings_t gpioSettings[Select__Max_];

extern uint8_t select_gps_buf[Select__Max_][2];
extern uint8_t select_stm_buf[Select__Max_][2];

void select_mux_channel(uint8_t relay,uint8_t position);
void select_gps(void);
void select_stm(void);


#endif /* UART_MUX_H_ */
