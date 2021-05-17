/*
 * ADRF6820.c
 *
 *  Created on: 20 Aðu 2019
 *      Author: abdullah
 */


#include "ADRF6820.h"
#include "stm32f1xx_hal.h"

extern SPI_HandleTypeDef hspi2;

void ADRF6820_Reset(){

	uint16_t tx_data[2];

	tx_data[0] = SOFT_RESET_REG;
	tx_data[1] = ADRF6820_soft_reset;


	ADRF6820_Write_Reg((uint8_t*)&tx_data,2);
}


void ADRF6820_Load_activations(){

	uint8_t tx_data[3] = {0};
	uint8_t rxData[3] = {0};

	tx_data[0] = Enables_REG;
	tx_data[2] = ((Enables.LO_DRV1X_EN << 7) |
				 (Enables.VCO_MUX_EN << 6) |
				 (Enables.REF_BUF_EN << 5) |
				 (Enables.VCO_EN 	<< 4) |
				 (Enables.DIV_EN 	<< 3) |
				 (Enables.CP_EN 	<< 2) |
				 (Enables.VCO_LDO_EN << 1)) | 0x01;

	tx_data[1] = ((Enables.DMOD_EN 	<< 2) |
				 (Enables.QUAD_DIV_EN << 1) |
				 (Enables.LO_DRV2X_EN << 0)) | 0xF8;



	ADRF6820_Write_Reg((uint8_t*)&tx_data,3);

}


static void ADRF6820_Write_Reg(uint8_t *value, uint8_t len){

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, (uint8_t*)value, len, 1000);						// Send data byte
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}



