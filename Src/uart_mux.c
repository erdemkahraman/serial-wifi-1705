/*
 * uart_mux.c
 *
 *  Created on: 1 Tem 2019
 *      Author: abdullah
 */

#include "uart_mux.h"


uint8_t select_gps_buf[Select__Max_][2] = {

    {S0, GPIO_PIN_RESET},
    {S1, GPIO_PIN_RESET},

};

uint8_t select_stm_buf[Select__Max_][2] = {

    {S0, GPIO_PIN_RESET},
    {S1, GPIO_PIN_SET},

};

struct gpioSettings_s gpioSettings[Select__Max_] = {

	{mux_select0_pin,		mux_select0_Port},
	{mux_select1_pin,		mux_select1_Port},

};

void select_mux_channel(uint8_t relay,uint8_t position){

  HAL_GPIO_WritePin(gpioSettings[relay].GPIOx,
      gpioSettings[relay].GPIO_Pin,
      position);

}

void select_gps(void){

//	select_mux_channel(select_gps_buf[0][0], select_gps_buf[0][1]);
//	select_mux_channel(select_gps_buf[1][0], select_gps_buf[1][1]);
	HAL_GPIO_WritePin(mux_select0_Port, mux_select0_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mux_select1_Port, mux_select1_pin, GPIO_PIN_RESET);

}

void select_stm(void){

//	select_mux_channel(select_stm_buf[0][0], select_stm_buf[0][1]);
//	select_mux_channel(select_stm_buf[1][0], select_stm_buf[1][1]);
	HAL_GPIO_WritePin(mux_select0_Port, mux_select0_pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(mux_select1_Port, mux_select1_pin, GPIO_PIN_SET);


}

void enable_mux(void){

	HAL_GPIO_WritePin(mux_ENABLE_Port, mux_ENABLE_Pin, GPIO_PIN_RESET);

}

void disable_mux(void){

	HAL_GPIO_WritePin(mux_ENABLE_Port, mux_ENABLE_Pin, GPIO_PIN_SET);

}
