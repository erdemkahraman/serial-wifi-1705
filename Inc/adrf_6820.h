/*
	adrf_6820.h
	usage;
	*remove R9, R10, R24 resistors on evalz board.
	*define CS,SCLK,SDIO PINS and PORTS in this file
	*declare struct ADRFData adrf for storage adrf registers
	*call SDIO_PIN_Setup(void);
	*bitBangWrite(reg,data) and bitBangRead(reg) functions can R/W standalone register
	*for create new setup, define your function and write registers with bitBangWrite(reg,data)
(like void ADRF_Setup_GPS(struct ADRFData *adrf) ), finally read all registers and check the values.

*/

#ifndef __ADRF_6820_H
#define __ADRF_6820_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "stm32f1xx_hal.h"


struct ADRFData{
	uint16_t reg01;
	uint16_t reg02;
	uint16_t reg03;
	uint16_t reg04;
	uint16_t reg10;
	uint16_t reg20;
	uint16_t reg21;
	uint16_t reg22;
	uint16_t reg23;
	uint16_t reg30;
	uint16_t reg31;
	uint16_t reg32;
	uint16_t reg33;
	uint16_t reg34;
	uint16_t reg40;
	uint16_t reg42;
	uint16_t reg43;
	uint16_t reg44;
	uint16_t reg45;
	uint16_t reg46;
	uint16_t reg47;
	uint16_t reg48;
	uint16_t reg49;
	uint16_t reg60;
	uint16_t reg7E;
	uint16_t reg7F;
	
};

#define usDelay for(int j=0;j<150;j++)     /* */
#define usDelay6us for(int j=0;j<150;j++)  /* */
#define usDelay36us for(int j=0;j<600;j++) /* */
#define usDelay2us for(int j=0;j<50;j++)   /* */

#define CSPORT 		GPIOB
#define CSPIN 		GPIO_PIN_12

#define SCKPORT 	GPIOB
#define SCKPIN 		GPIO_PIN_13

#define SDIOPORT 	GPIOB
#define SDIOPIN 	GPIO_PIN_15

#define CSEN		HAL_GPIO_WritePin(CSPORT, CSPIN, GPIO_PIN_RESET)
#define CSCLR		HAL_GPIO_WritePin(CSPORT, CSPIN, GPIO_PIN_SET)

#define SCKSET		HAL_GPIO_WritePin(SCKPORT, SCKPIN, GPIO_PIN_SET)
#define SCKCLR  	HAL_GPIO_WritePin(SCKPORT, SCKPIN, GPIO_PIN_RESET)

#define SDIOSET 	HAL_GPIO_WritePin(SDIOPORT, SDIOPIN, GPIO_PIN_SET)
#define SDIORST 	HAL_GPIO_WritePin(SDIOPORT, SDIOPIN, GPIO_PIN_RESET)

#define SDIOIN 		HAL_GPIO_ReadPin(SDIOPORT, SDIOPIN)


#define tprint(x) HAL_UART_Transmit(&huart2,(uint8_t *)&x,sizeof(x),0x01)

void ADRFInit(struct ADRFData *adrf);


void bitBangWrite(uint8_t reg, uint16_t adrfData);
uint16_t bitBangRead(uint8_t reg);

void SDIO_PIN_Setup(void);

void sdioSetOutput(void);
void sdioSetInput(void);

void ADRF_Setup_GPS(struct ADRFData *adrf);
void ADRF_ReadAll(struct ADRFData *adrf);


#ifdef __cplusplus
}
#endif

#endif
