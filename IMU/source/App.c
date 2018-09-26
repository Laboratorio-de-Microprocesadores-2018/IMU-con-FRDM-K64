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
#include "CANCommunications.h"
#include "SysTick.h"
#include "math.h"

/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////
#define MEASURE_PERIOD_MS 100 // Time in milliseconds between two position measurements
#define TIMEOUT_MS 2000
#define THRESHOLD 5

/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

typedef struct{
	int roll;
	int pitch;
	int yaw;
}AngularPosition;

typedef struct{
	struct Vector{
		uint32_t x,y,z;
	}accelerometer,magnetometer;
}RawData;


/////////////////////////////////////////////////////////////////////////////////
//                         Global variables definition                         //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////

static uint32_t lastMeasureTime,lastRollTime,lastPitchTime;
static AngularPosition lastPos;
static Measurement m;

/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////

static AngularPosition computePosition(RawData d);


/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////

void App_Init (void)
{
	otherBoardCommunicationsInit();
	//desktopCommunicationsInit();
	//AccelerometerInit();

}
static int i=0;

void App_Run (void)
{

	uint64_t now = millis();

	if((now-lastMeasureTime)>MEASURE_PERIOD_MS)
	{
		RawData d ;//= accelometerGetRawData();
		AngularPosition currPos = computePosition(d);
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

	if(receiveOtherBoardsMeasurement(&m) == true)
		//sendMeasurement2Desktop(m.boardID,m.angleID,m.angleVal);


	if(receiveOtherBoardsMeasurement(&m) == true)
	{
		if(m.boardID == MY_BOARD_ID)
			i+=1;
		elseart
			i+=2;
	}
}

AngularPosition computePosition(RawData d)
{

}
