/////////////////////////////////////////////////////////////////////////////////
//                     XXXXXXXX_GENERIC_XXXXXXXXX                              //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file App.c
 * @brief
 */

/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "FXOS8700CQDriver.h"
#include "CANCommunications.h"
#include "DesktopCommunications.h"
#include "SysTick.h"
#include "math.h"

/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////
#define MEASURE_PERIOD_MS 100 // Time in milliseconds between two measurements
#define TIMEOUT_MS 2000		// Maximum time in milliseconds between to measurements
#define THRESHOLD 5 		// Threshold in degrees for measurements

/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

typedef struct{
	int roll;
	int pitch;
	int yaw;
}Orientation;


/////////////////////////////////////////////////////////////////////////////////
//                         Global variables definition                         //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////

static sData accelerometer,magnetometer;
static uint32_t lastMeasureTime,lastRollTime,lastPitchTime;
static Orientation lastPos;
static Measurement m;

/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////

static Orientation computePosition(sData accelerometer,sData magnetometer);


/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////

void App_Init (void)
{
	// Init accelerometer
	FX_config accelConfig = FX_GetDefaultConfig();
	FX_Init(accelConfig);

	// Init communications with other boards throug CAN bus
	otherBoardCommunicationsInit();

	// Init communication with desktop PC through UART
	desktopCommunicationsInit();
}


void App_Run (void)
{
	uint64_t now = millis();

	if((now-lastMeasureTime)>MEASURE_PERIOD_MS)
	{
		if(FX_GetData(&accelerometer,&magnetometer)==true)
		{
			Orientation currPos = computePosition(accelerometer,magnetometer);

			if(fabs(currPos.roll-lastPos.roll)>THRESHOLD || (now-lastRollTime) > TIMEOUT_MS)
			{
				m.boardID = MY_BOARD_ID;
				m.angleID = 'R';
				m.angleVal = currPos.roll;
				sendMeasurement2OtherBoards(m); // Si habilito self-receive en CAN se manda solo a la compu mas abajo
				lastRollTime = now;
			}

			if(fabs(currPos.pitch-lastPos.pitch)>THRESHOLD || (now-lastPitchTime) > TIMEOUT_MS)
			{
				m.boardID = MY_BOARD_ID;
				m.angleID = 'P';
				m.angleVal = currPos.roll;
				sendMeasurement2OtherBoards(m);
				lastPitchTime = now;
			}
			lastMeasureTime = now;
		}
	}

	if(receiveOtherBoardsMeasurement(&m) == true);
		sendMeasurement2Desktop(m.boardID,m.angleID,m.angleVal);


}

Orientation computePosition(sData a,sData magnetometer)
{
	Orientation o;
	int norm = sqrt(a.x*a.x+a.y*a.y+a.z*a.z);
	a.x/=norm;
	a.y/=norm;
	a.z/=norm;
	o.pitch=atan(a.x/sqrt(a.y*a.y+a.z*a.z));
	o.roll=atan2(a.y,sqrt(a.x*a.x+a.z*a.z));
	o.yaw=0;
	return o;
}
