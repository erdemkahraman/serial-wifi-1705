#include "TB334.h"

#define usDelayM 			HAL_Delay(1);


void TB334_WRITE(uint8_t tb334_data)
{
	usDelayM;
	usDelayM;
	usDelayM;
	LEEN;
	for(int i=0; i<6; i++)
	{
		if((tb334_data & 0x20) == 0x20)
		{

			DATAHIGH;
			usDelayM;
			CLKEN;
			usDelayM;
			CLKRST;
			usDelayM;
			DATALOW;
		}
		else
		{
			
			usDelayM;
			CLKEN;
			usDelayM;
			CLKRST;
			usDelayM;
			
		}
		tb334_data = tb334_data << 1;
	}
	usDelayM;
	LERST;
	usDelayM;
	LEEN;

}

HAL_StatusTypeDef attenuator_set(uint16_t value)
{


	int db_c = value / 10;
	int db_c05 = value % 10;

	if(db_c05 != 0){
		db_c = (db_c << 1) + 1;
	}else{
		db_c = (db_c << 1);

	}

	if(db_c <= 0x3F){
		TB334_WRITE((db_c) & 0x3F);
		return HAL_OK;

	}else{
		return HAL_ERROR;

	}
}




