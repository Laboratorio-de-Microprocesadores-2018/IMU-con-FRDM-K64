
#include "FXOS8700CQDriver.h"
#include "FXOS8700CQRegisterAddressMap.h"
#include "GPIO.h"
#include <stdlib.h>
#include "I2C_Library.h"


#define DEF_SLAVE_ADDR 0x1D

static sData accData;
static sData magData;


static int8_t dataBuff[256];
static I2C_CONTROL_T i2cConfig={0,0,0,0,0,0,0,dataBuff,0,0,NULL};
static bool hayData;



static void readData(void);
static void sDataReady(void);



bool LlegoAlgo()
{
	return hayData;
}

bool FXInit()
{

	I2C_init();
	I2C_SetDefaultConfig(&i2cConfig, DEF_SLAVE_ADDR,readData);

	i2cConfig.address_reg=WHO_AM_I;
	i2cConfig.dataSize=1;
	I2C_Blocking_ReadData(&i2cConfig);

	i2cConfig.address_reg=CTRL_REG1;
	i2cConfig.dataSize=1;
	i2cConfig.data[0]=0x00;
	I2C_Blocking_WriteData(&i2cConfig);

	i2cConfig.address_reg=XYZ_DATA_CFG;
	i2cConfig.dataSize=1;
	i2cConfig.data[0]=0b00000001;
	I2C_Blocking_WriteData(&i2cConfig);






	i2cConfig.address_reg=CTRL_REG4;
	i2cConfig.dataSize=1;
	i2cConfig.data[0]=0b00000001;
	I2C_Blocking_WriteData(&i2cConfig);


	i2cConfig.address_reg=CTRL_REG5;
	i2cConfig.dataSize=1;
	i2cConfig.data[0]=0x00;
	I2C_Blocking_WriteData(&i2cConfig);




	i2cConfig.address_reg=M_CTRL_REG1;
	i2cConfig.dataSize=1;
	i2cConfig.data[0]=0b00011111;
	I2C_Blocking_WriteData(&i2cConfig);

	i2cConfig.address_reg=M_CTRL_REG2;
	i2cConfig.dataSize=1;
	i2cConfig.data[0]=0b0010000;
	I2C_Blocking_WriteData(&i2cConfig);

	pinMode(PORTNUM2PIN(PC,13),INPUT);
	pinConfigureIRQ(PORTNUM2PIN(PC,13),IRQC_INTERRUPT_FALLING, sDataReady);


	//por ahora lo dejo al final cosa de que el driver hace enable en init

	i2cConfig.address_reg=CTRL_REG1;
	i2cConfig.dataSize=1;
	i2cConfig.data[0]=0b00001101;
	I2C_Blocking_WriteData(&i2cConfig);



	return 0;//VER DESPUES SI SE PONE ALGO

}


bool GetData(sData *acc, sData *mag)
{
	*acc=accData;
	*mag=magData;
	hayData=0;
	return 0;//ver si se usa el bool para manejo de errores

}





static void readData(void)//Read data de I2C
{
	switch(i2cConfig.address_reg)
	{
		case WHO_AM_I:
			//assert(i2cConfig.data[0] != DEF_SLAVE_ADDR)
			break;
		case OUT_X_MSB:
			if(i2cConfig.dataSize==12)
			{
				accData.x=i2cConfig.data[0] |  i2cConfig.data[1];
				accData.y=i2cConfig.data[2] |  i2cConfig.data[3];
				accData.z=i2cConfig.data[4] |  i2cConfig.data[5];
				magData.x=i2cConfig.data[6] |  i2cConfig.data[7];
				magData.y=i2cConfig.data[8] |  i2cConfig.data[9];
				magData.z=i2cConfig.data[10] | i2cConfig.data[11];
				hayData=true;
			}
			break;


	}
}


static void sDataReady(void)
{
	I2C_SetDefaultConfig(&i2cConfig, DEF_SLAVE_ADDR,readData);/*REVISAR*/
	i2cConfig.address_r=OUT_X_MSB;
	i2cConfig.dataSize=12;
	I2C_ReadData(&i2cConfig);
}


