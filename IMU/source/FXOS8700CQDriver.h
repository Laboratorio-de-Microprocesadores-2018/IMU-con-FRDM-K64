/////////////////////////////////////////////////////////////////////////////////
//                         														//
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/*The code is based on the FXOS8700CQ driver example of its datasheet */

/**
 * @file     FXOS8700CQDriver.h
 * @brief    FXOS8700CQ driver to control its accelerometer and/or magnetometer
 * via I2C for FRDM-K64
 * @author   Diego Juarez
 */
#ifndef FXOS8700CQDriver_H_
#define FXOS8700CQDriver_H_


/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include<stdint.h>
#include<stdbool.h>



/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

/**/
typedef struct {
	float x;
	float y;
	float z;

}sData;


/*FX_modes:There are three posible modes:
 * HYBRID: Both de Accelerometer and the Magnetometer are activated (ODR is reduced by a factor of 2)
 * ACC_ONLY:Only Accelerometer(normal ODR)
 * MAG_ONLY:Only Magnetometer(normal ODR)
 * */
typedef enum{FX_ACC_ONLY=0,FX_MAG_ONLY,FX_HYBRID}FX_modes;

/*FX_ODRs:There are seven posible output data rates
 * reminder: in hybrid mode the ODR is reduced by a factor of 2 )
 * FX_ODR_800=800Hz; FX_ODR_400=400Hz;FX_ODR_200=200Hz;
 * FX_ODR_50=50Hz;FX_ODR_12=12.5Hz;FX_ODR_6=6.25Hz;
 * FX_ODR_1=1.5625Hz;
 * */
typedef enum {FX_ODR_800=0,FX_ODR_400,FX_ODR_200,FX_ODR_50,FX_ODR_12,FX_ODR_6,FX_ODR_1}FX_ODRs;


/*FX_accScales: there are 3 accelerometer scales with their respective sensitivities:
 *  SCALE2=+-2g range with +-0.244mg/LSB
 *  SCALE4=+-4g range with +-0.488mg/LSB
 *  SCALE8=+-8g range with +-0.0.976mg/LSB
 *	SCALE2 and SCALE4 use FXOS8700CQ's low noise mode and SCALE8 doesn't(it's incompatible) in this driver.
 *  */
typedef enum{FX_SCALE2=0,FX_SCALE4,FX_SCALE8}FX_accScales;

typedef struct {
	FX_modes mode;
	FX_accScales scale;
	FX_ODRs ODR;
	//bool lowNoise;
}FX_config;

/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////


FX_config FX_GetDefaultConfig(void);
bool FX_Init(FX_config conf);
void FX_Enable(void);//Not developed
void Fx_Disable(void);//Not developed
//bool FX_Configure();//Not developed (WARNING: while configuring the sensor should be disabled)
FX_modes FX_GetMode(void);
FX_accScales FX_GetScale(void);

bool FX_GetData(sData *acc, sData *mag);

bool FX_newData();




#endif /* FXOS8700CQDriver_H_*/
