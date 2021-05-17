/*
 * adrf_6720.h
 *
 *  Created on: 13 Oca 2020
 *      Author: Cezeri
 */

#ifndef ADRF_6720_H_
#define ADRF_6720_H_

#include "adrf_6820.h"

void ADRF6720_Setup_GPS(void);

#define CSPORT_6720 	GPIOB
#define CSPIN_6720 		GPIO_PIN_11

#define CSEN_6720		HAL_GPIO_WritePin(CSPORT_6720, CSPIN_6720, GPIO_PIN_RESET)
#define CSCLR_6720		HAL_GPIO_WritePin(CSPORT_6720, CSPIN_6720, GPIO_PIN_SET)

#endif /* ADRF_6720_H_ */
