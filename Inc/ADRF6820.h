/*
 * ADRF6820.h
 *
 *  Created on: 20 Aðu 2019
 *      Author: abdullah
 */

#ifndef ADRF6820_H_
#define ADRF6820_H_

#include <stdint.h>

/*Register map*/
#define SOFT_RESET_REG        	0x00;
#define Enables_REG          	0x01;
#define INT_DIV_REG         	0x02;
#define FRAC_DIV_REG         	0x03;
#define MOD_DIV_REG         	0x04;
#define PWRDWN_MASK_REG        	0x10;
#define CP_CTL_REG         		0x20;
#define PFD_CTL_REG         	0x21;
#define VCO_CTL_REG         	0x22;
#define DGA_CTL_REG         	0x23;
#define BALUN_CTL_REG         	0x30;
#define MIXER_CTL_REG         	0x31;
#define MOD_CTL0_REG         	0x32;
#define MOD_CTL1_REG         	0x33;
#define MOD_CTL2_REG         	0x34;
#define PFD_CTL2_REG         	0x40;
#define DITH_CTL1_REG         	0x42;
#define DITH_CTL2_REG        	0x43;
#define DIV_SM_CTL_REG        	0x44;
#define VCO_CTL2_REG         	0x45;
#define VCO_RB_REG         		0x46;
#define VCO_CTL3_REG         	0x49;


#define ADRF6820_soft_reset     0x01;

struct Enables_s{

	uint8_t DMOD_EN;
	uint8_t QUAD_DIV_EN;
	uint8_t LO_DRV2X_EN;
	uint8_t LO_DRV1X_EN;
	uint8_t VCO_MUX_EN;
	uint8_t REF_BUF_EN;
	uint8_t VCO_EN;
	uint8_t DIV_EN;
	uint8_t CP_EN;
	uint8_t VCO_LDO_EN;

};

struct Enables_s Enables;

struct INT_DIV_s{
   uint8_t  DIV_MODE;
   uint16_t INT_DIV;

};

typedef enum {
	Fractional,
	Integer,
}Divide_mode;

struct MOD_DIV_s{
   uint16_t MOD_DIV;	//reset value 0x600

};

struct FRAC_DIV_s{
   uint16_t FRAC_DIV;	//reset value 0x128
};

struct PWRDWN_MASK_s{

	uint8_t DMOD_MASK;
	uint8_t QUAD_DIV_MASK;
	uint8_t LO_DRV2X_MASK;
	uint8_t LO_DRV1X_MASK;
	uint8_t VCO_MUX_MASK;
	uint8_t REF_BUF_MASK;
	uint8_t VCO_MASK;
	uint8_t DIV_MASK;
	uint8_t CP_MASK;
	uint8_t VCO_LDO_MASK;
};

struct CP_CTL_s{

	uint8_t CPSEL;		//Charge pump reference current select ( 1 or 0)
	uint8_t CSCALE;		//Charge pump coarse scale current (0x01:250 uA  <->  0x0F:1000uA)
	uint8_t BLEED;		//Charge pump bleed ( 0: 0 uA <-> 0x3F: 484.375 uA )
};

typedef enum {
	Internal,
	External,
}Charge_pump_ref_current_select;

struct PFD_CTL_s{

	uint8_t REF_MUX_SEL;
	uint8_t PFD_POLARITY;
	uint8_t REFSEL;
};

typedef enum {
	LOCK_DET,
	VPTAT,
	REFCLK,
	REFCLK_2,	// REFCLK/2
	REFCLK__2,	// REFCLK*2
	REFCLK_8,	// REFCLK/8
	REFCLK_4,	// REFCLK/4
	SCAN,
}Reference_mux_select;

typedef enum {
	Positive,
	Negative,
}Set_PFD_polarity;

typedef enum {
	__2,
	__1,
	Divide_by_2,
	Divide_by_4,
	Divide_by_8,
}Set_REF_input_ratio;

struct VCO_CTL_s{

	uint8_t LO_DRV_LVL;
	uint8_t DRVDIV2_EN;		// 0:disable, 1:enable
	uint8_t DIV8_EN;		// 0:disable, 1:enable
	uint8_t DIV4_EN;		// 0:disable, 1:enable
	uint8_t VCO_SEL;
};

struct DGA_CTL{

	uint8_t RFSW_MUX;
	uint8_t RFSW_SEL;
	uint8_t RFDSA_SEL;
};






static void ADRF6820_Write_Reg(uint8_t *value, uint8_t len);

void ADRF6820_Reset();
void ADRF6820_Load_activations();

/*End*/
#endif /* ADRF6820_H_ */
