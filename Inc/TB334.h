/*
	usage;
	*define CLK,DATA,LE Pins in this file
	*
	*defined parameters,  TB334_SET_05DB  0X01
												TB334_SET_1DB 	0X02
												TB334_SET_2DB   0X04
												TB334_SET_4DB	  0X08
												TB334_SET_8DB 	0X10
												TB334_SET_16DB  0X20
												TB334_SET_31DB  0x3F 
												
	void TB334_WRITE(uint8_t tb334_data);

*/


#ifndef __TB334_H
#define __TB334_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "main.h"
#include "stm32f1xx_hal.h"


#define CLKEN     HAL_GPIO_WritePin(ATT_CLK_GPIO_Port, ATT_CLK_Pin, GPIO_PIN_SET);
#define CLKRST    HAL_GPIO_WritePin(ATT_CLK_GPIO_Port, ATT_CLK_Pin, GPIO_PIN_RESET);
	
#define DATAHIGH  HAL_GPIO_WritePin(ATT_DATA_GPIO_Port,ATT_DATA_Pin,GPIO_PIN_SET);
#define DATALOW   HAL_GPIO_WritePin(ATT_DATA_GPIO_Port,ATT_DATA_Pin,GPIO_PIN_RESET);
	
#define LEEN      HAL_GPIO_WritePin(ATT_LE_GPIO_Port,ATT_LE_Pin,GPIO_PIN_RESET);
#define LERST     HAL_GPIO_WritePin(ATT_LE_GPIO_Port,ATT_LE_Pin,GPIO_PIN_SET);

#define TB334_SET_05DB  0X01
#define TB334_SET_1DB 	0X02
#define TB334_SET_2DB   0X04
#define TB334_SET_4DB	0X08
#define TB334_SET_8DB 	0X10
#define TB334_SET_16DB  0X20
#define TB334_SET_31DB  0x3F


void TB334_WRITE(uint8_t tb334_data);
HAL_StatusTypeDef attenuator_set(uint16_t value);


#ifdef __cplusplus
}
#endif

#endif
