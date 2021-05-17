#include "adrf_6820.h"
#include "adrf_6720.h"
#include "main.h"
#include "uart_protocol.h"

extern UartReqPackConf_t        		uartReqPackConf;
extern UartResPackConf_t        		uartResPackConf;

void ADRFInit(struct ADRFData *adrf)
{

	/* DEFAULT SET-UP */
	HAL_Delay(1000);
	bitBangRead(0x01);
	bitBangWrite(0x01,0x0000);
	HAL_Delay(1000);
	bitBangWrite(0x01,0xFE7F);
	bitBangRead(0x02);
	bitBangRead(0x03);
	bitBangRead(0x04);
	bitBangRead(0x20);
	bitBangWrite(0x20, 0x0026);
	bitBangWrite(0x20, 0x0026);
	bitBangRead(0x01);
	bitBangRead(0x21);
	bitBangWrite(0x21,0x0003);
	bitBangWrite(0x22,0x0002);
	bitBangWrite(0x22,0x0002);
	bitBangWrite(0x22,0x0002);
	bitBangWrite(0x22,0x0002);
	bitBangWrite(0x02,0x0034);
	bitBangWrite(0x03,0x0080);
	bitBangWrite(0x04,0x0600);
	bitBangWrite(0x02,0x0034);
	bitBangWrite(0x03,0x0080);
	bitBangWrite(0x04,0x0600);
	bitBangWrite(0x21,0x0003);
	bitBangWrite(0x21,0x0003);
	bitBangRead(0x22);
	bitBangWrite(0x22,0x0002);
	bitBangWrite(0x22,0x0002);
	bitBangWrite(0x22,0x0002);
	bitBangRead(0x23);
	bitBangWrite(0x23,0x0000);
	bitBangWrite(0x23,0x0000);
	bitBangWrite(0x23,0x0000);
	bitBangWrite(0x23,0x0000);
	bitBangRead(0x30);
	bitBangWrite(0x30,0x0000);
	bitBangWrite(0x30,0x0000);
	bitBangRead(0x31);
	bitBangWrite(0x31,0x0001);
	bitBangWrite(0x31,0x0101);
	bitBangWrite(0x31,0x1101);
	bitBangRead(0x32);
	bitBangWrite(0x32,0x0500);
	bitBangWrite(0x32,0x0500);
	bitBangWrite(0x32,0x0500);
	bitBangWrite(0x32,0x0900);
	bitBangRead(0x33);
	bitBangWrite(0x33,0x0000);
	bitBangWrite(0x33,0x0000);
	bitBangRead(0x34);
	bitBangWrite(0x34,0x0300);
	bitBangWrite(0x34,0x0B00);
	bitBangRead(0x40);
	bitBangWrite(0x40,0x0000);
	bitBangWrite(0x40,0x0010);
	bitBangWrite(0x40,0x0010);
	bitBangRead(0x22);
	bitBangWrite(0x42,0x0000);
	bitBangWrite(0x43,0x0000);
	bitBangWrite(0x42,0x0006);
	bitBangWrite(0x43,0x0000);
	bitBangWrite(0x42,0x000E);
	bitBangWrite(0x43,0x0000);
	bitBangWrite(0x44,0x0002);
	bitBangRead(0x43);
	bitBangRead(0x44);
	bitBangWrite(0x44,0x0002);
	bitBangRead(0x45);
	bitBangWrite(0x45,0x0000);
	bitBangWrite(0x45,0x0000);
	bitBangWrite(0x45,0x0000);
	bitBangRead(0x46);
	bitBangRead(0x47);
	bitBangRead(0x48);
	bitBangRead(0x49);
	bitBangRead(0x60);
	bitBangRead(0x7E);
	bitBangRead(0x7F);
	
	HAL_Delay(50);
	
	/* CHECK REGISTER VALUES. */
	ADRF_ReadAll(adrf);
	HAL_Delay(250);
}
void ADRF_ReadAll(struct ADRFData *adrf)
{
	adrf->reg01=bitBangRead(0x01);
	adrf->reg02=bitBangRead(0x02);
	adrf->reg03=bitBangRead(0x03);
	adrf->reg04=bitBangRead(0x04);
	adrf->reg20=bitBangRead(0x20);
	adrf->reg01=bitBangRead(0x01);
	adrf->reg21=bitBangRead(0x21);
	adrf->reg22=bitBangRead(0x22);
	adrf->reg23=bitBangRead(0x23);
	adrf->reg30=bitBangRead(0x30);
	adrf->reg31=bitBangRead(0x31);
	adrf->reg32=bitBangRead(0x32);
	adrf->reg33=bitBangRead(0x33);
	adrf->reg34=bitBangRead(0x34);
	adrf->reg40=bitBangRead(0x40);
	adrf->reg42=bitBangRead(0x42);
	adrf->reg43=bitBangRead(0x43);
	adrf->reg44=bitBangRead(0x44);
	adrf->reg45=bitBangRead(0x45);
	adrf->reg46=bitBangRead(0x46);
	adrf->reg47=bitBangRead(0x47);
	adrf->reg48=bitBangRead(0x48);
	adrf->reg49=bitBangRead(0x49);
	adrf->reg60=bitBangRead(0x60);
	adrf->reg7E=bitBangRead(0x7E);
	adrf->reg7F=bitBangRead(0x7F);
	adrf->reg01=bitBangRead(0x01);
	HAL_Delay(6);
}
void ADRF_Setup_GPS(struct ADRFData *adrf)
{
	bitBangWrite(0x01,0x0000);  /*Disable everything for safety*/
	
	bitBangWrite(0x32,0x0903); 	/*1: ILO = 3 */ 
	bitBangWrite(0x33,0x1700);  /*2: DCOFFI = 23 */ 
	bitBangWrite(0x32,0x09f3);	/*3: QLO =15 */
	bitBangWrite(0x33,0x1702);	/*4: DCOFFQ=2 */
	bitBangWrite(0x34,0x0A00);	/*5: BWSEL=2*/
	bitBangWrite(0x34,0x0E00);	/*6: BB_BIAS=15*/
	bitBangWrite(0x30,0x0002);	/*7: BAL_CIN=1*/
	bitBangWrite(0x30,0x0022);	/*8: BAL_COUT=1*/
	bitBangWrite(0x31,0x1501);	/*9: MIX_BIAS=5*/
	bitBangWrite(0x31,0x1521);	/*10: MIX_RDAC=9*/
	bitBangWrite(0x31,0x1525);	/*11: MIX_CDAC=5*/
	/* 12 PLL REF IN (MHZ) = 38.5 */
	bitBangWrite(0x22,0x0002);
	bitBangWrite(0x02,0x0068);
	bitBangWrite(0x03,0x0080);
	bitBangWrite(0x04,0x0300);
	/* 13 PLL REF DIVIDER = x1 */
	bitBangWrite(0x21,0x0001);
	bitBangWrite(0x02,0x0034);
	bitBangWrite(0x03,0x0080);
	bitBangWrite(0x04,0x0600);
		/* clk edge */
	bitBangWrite(0x40,0x0030);
	/* charge pump */
	bitBangWrite(0x20,0x0c26);
	/* */
	bitBangWrite(0x43,0x0001);
	bitBangWrite(0x40,0x0000);
	/* 0X10 */
	//bitBangWrite(0x10,0xFE7F);
	/* 14 POLARITY = NEG */
	bitBangWrite(0x21,0x0009);
	/* 15 STEP SIZE = 10 */
	bitBangWrite(0x22,0x0002);
	bitBangWrite(0x02,0x0034);
	bitBangWrite(0x03,0x0140);
	bitBangWrite(0x04,0x0F00);
	/* 16 LO FREQ (MHZ) = 2018.65 */
	bitBangWrite(0x22,0x0002);
	bitBangWrite(0x02,0x0034);
	bitBangWrite(0x03,0x048a);
	bitBangWrite(0x04,0x0F00);
	
	/*17. Lock pll ref divider */
	bitBangWrite(0x21,0x0009);
	bitBangWrite(0x22,0x0001);
	bitBangWrite(0x02,0x0034);
	bitBangWrite(0x03,0x048a);
	bitBangWrite(0x04,0x0F00);
	/* 18. pll sets */
	bitBangWrite(0x21,0x000a);
	bitBangWrite(0x22,0x0001);
	bitBangWrite(0x02,0x0069);
	bitBangWrite(0x03,0x0109);
	bitBangWrite(0x04,0x0780);
	/* 19. kapa aç */
	bitBangWrite(0x21,0x0009);
	bitBangWrite(0x22,0x0001);
	bitBangWrite(0x02,0x0034);
	bitBangWrite(0x03,0x889);
	bitBangWrite(0x04,0x0F00);
	bitBangWrite(0x40,0x0030);
	
//	bitBangWrite(0x10,0xFE7F);

	bitBangWrite(0x44,0X0002);
	bitBangWrite(0x46,0x0010);
	
	bitBangWrite(0x21,0x0009);
	bitBangWrite(0x22,0x0002);
	
	bitBangWrite(0x01,0xFE7F);
	
	//LO freq: 2018,65
//	bitBangWrite(0x02,0x0034);
//	bitBangWrite(0x03,0x889);
//	bitBangWrite(0x04,0x0F00);
	//LO freq: 2008,42
	bitBangWrite(0x02,0x0034);
	bitBangWrite(0x03,0x048a);
	bitBangWrite(0x04,0x0F00);
	
	bitBangWrite(0x21,0x0009);
		
	//bitBangWrite(0x01,0xFE7F); /* Enables after setup */
	
	ADRF_ReadAll(adrf);
}
void sdioSetOutput()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Pin = SDIOPIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SDIOPORT, &GPIO_InitStruct);
}
void SDIO_PIN_Setup()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_GPIO_WritePin(CSPORT,CSPIN,GPIO_PIN_SET);
	HAL_GPIO_WritePin(SCKPORT,SCKPIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SDIOPORT,SDIOPIN,GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = CSPIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CSPORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SCKPIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SCKPORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = SDIOPIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SDIOPORT, &GPIO_InitStruct);

	HAL_GPIO_WritePin(SCKPORT,SCKPIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SDIOPORT,SDIOPIN,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CSPORT,CSPIN,GPIO_PIN_SET);

	CSCLR;
	usDelay;
	SDIORST;
	SCKCLR;
}

void sdioSetInput()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	GPIO_InitStruct.Pin = SDIOPIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDIOPORT, &GPIO_InitStruct);
}
void bitBangWrite(uint8_t reg, uint16_t adrfData)
{
	/* bit shift for read operation last bit must be 0*/
	reg = reg << 1;
	HAL_Delay(6);
	if(uartReqPackConf.ADRF_id == 1){
		CSEN;
	}else if(uartReqPackConf.ADRF_id == 2){
		CSEN_6720;
	}
	usDelay;
	for(int i=0; i<8; i++)
	{
		if((reg&0x80)==0x80) /* 0b10000000 */
		{
			
			/* _|-|_|-|__|-|_|-|__|-|_|-|__|-|_|-|__|-|_|-|__|-|_|-|__|-|_|-|_ */
			usDelay2us;
			SDIOSET;
			usDelay;
			usDelay;
			SCKSET;
			usDelay;
			SCKCLR;
			SDIORST;
			usDelay;

			reg = reg << 1;
		}
		else if((reg&0x80)==0x00)
		{
			usDelay2us;
			SDIORST;
			usDelay;
			usDelay;
			SCKSET;
			usDelay;
			SCKCLR;
			SDIORST;
			usDelay;
			
			reg = reg << 1;
		}
		
			
	}
	usDelay36us;
	for(int i=0; i<16; i++)
	{
		if((adrfData&0x8000)==0x8000) /* 0b10000000 */
		{
			usDelay2us;
			SDIOSET;
			usDelay;
			usDelay;
			SCKSET;
			usDelay;
			SCKCLR;
			SDIORST;
			usDelay;

			adrfData = adrfData << 1;
		}
		else if((adrfData&0x8000)==0x0000)
		{
			usDelay2us;
			SDIORST;
			usDelay;
			usDelay;
			SCKSET;
			usDelay;
			SCKCLR;
			SDIORST;
			usDelay;
			
			adrfData = adrfData << 1;
		}
		usDelay36us;
	}
	usDelay;
	if(uartReqPackConf.ADRF_id == 1){
		CSCLR;
	}else if(uartReqPackConf.ADRF_id == 2){
		CSCLR_6720;
	}	usDelay;
}
uint16_t bitBangRead(uint8_t reg)
{
	/* bit shift for read operation and last bit must be 1 */
	reg = reg << 1;
	reg = reg + 1;
	/* 6ms delay between frames */
	HAL_Delay(6);
	
	/***** send header before read operation *****/
	uint8_t header[3] = {0x0A,0x00,0x00};
	HAL_Delay(1);
	CSEN;
	for(int t=0;t<3;t++)
	{
		for(int i=0; i<8;i++)
		{
			if((header[t]&0x80)==0x80)
			{
				usDelay2us;
				SDIOSET;
				usDelay;
				usDelay;
				SCKSET;
				usDelay;
				SCKCLR;
				SDIORST;
				usDelay;
				
				header[t] = header[t] << 1;
			}
			else if((header[t]&0x80)==0x00)
			{
				usDelay2us;
				SDIORST;
				usDelay;
				usDelay;
				SCKSET;
				usDelay;
				SCKCLR;
				SDIORST;
				usDelay;
				
				header[t] = header[t] << 1;
			}
			
		}
		usDelay36us;
	}
	/* cs clear after header and delay (~45us) */
	CSCLR;
	
	HAL_Delay(1);
	
	CSEN;
	usDelay;
	uint16_t RxData = 0;
	for(int i=0; i<8; i++)
	{
		if((reg&0x80)==0x80) /* 0b10000000 */
		{
			/* TO-DO: sdio ile sck arasina usDelay eklenebilir */
			/* önce data gider sonra sck, önce sck resetlenir sonra data */
			usDelay2us;
			SDIOSET;
			usDelay;
			usDelay;
			SCKSET;
			usDelay;
			SCKCLR;
			SDIORST;
			usDelay;

			reg = reg << 1;
		}
		else if((reg&0x80)==0x00)
		{
			usDelay2us;
			SDIORST;
			usDelay;
			usDelay;
			SCKSET;
			usDelay;
			SCKCLR;
			SDIORST;
			usDelay;
			reg = reg << 1;
		}		
	}
	usDelay36us;
	sdioSetInput();
	
	usDelay;
	for(int i=0; i<16; i++)
	{
		//TESTH;
		SCKSET;
		//usDelay;
		if(SDIOIN == GPIO_PIN_SET) //GPIO_PIN_SET
		{
			RxData = RxData + 0x01;
			if(i<15)
				RxData = RxData << 1;
		}
		else
		{
			if(i<15)
				RxData = RxData << 1;
		}
		usDelay6us;
		//TESTL;
		SCKCLR;
		if(i==7)
		{
			usDelay36us;
			usDelay;
		}
		else
		{
			usDelay;
			usDelay;
			usDelay;
		}
	}
	
	usDelay36us;
	SCKCLR;
	usDelay;
	CSCLR;
	usDelay;
	sdioSetOutput();
	return RxData;
	
}
