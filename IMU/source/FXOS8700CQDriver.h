#ifndef FXOS8700CQDriver_H_
#define FXOS8700CQDriver_H_

#include<stdint.h>
#include<stdbool.h>


typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;

}sData;


typedef struct {// To Complete
	uint8_t scale;
	uint8_t ODR;




}fxConfig;

void GetDefaultConfig(fxConfig *config);
bool FXInit(void);
void FXEnable(void);
void FxDisable(void);
bool FXConfigure();//WARNING: while configuring the sensor is disabled and then enabled (only if it was when calling)
bool GetData(sData *acc, sData *mag);




#endif /* FXOS8700CQDriver_H_*/
