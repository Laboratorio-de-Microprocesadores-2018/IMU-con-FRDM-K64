/////////////////////////////////////////////////////////////////////////////////
//                     XXXXXXXX_GENERIC_XXXXXXXXX                              //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file App.c
 * @brief Generic project.
 */

/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "CAN.h"
/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////

#define TX_MB_INDEX 1
#define RX_MB_INDEX 2

#define MIN_ID 0x100
#define MAX_ID 0x10F
/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                         Global variables definition                         //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                   Local variable definitions ('static')                     //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                   Local function prototypes ('static')                      //
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////

/** Función que se llama 1 vez, al comienzo del programa */
void App_Init (void)
{
	/* Init CAN module. */
	CAN_Config config;
	CAN_GetDefaultConfig(&config);
	CAN_Init(CAN0,&config,50000000);

	/* Enable CAN module. */
	CAN_Enable(CAN0);



}

/** Función que se llama constantemente en un ciclo infinito */
void App_Run (void)
{
	CAN_DataFrame frame;
	CAN_ConfigureRxMB(CAN0,RX_MB_INDEX,0x108);
	while(1)
	{
		while(CAN_GetMbStatusFlag(CAN0, RX_MB_INDEX)==false);

		CAN_ReadRxMB(CAN0,RX_MB_INDEX,&frame);
		CAN_ClearMbStatusFlag(CAN0,RX_MB_INDEX);
	}




	/* Prepares the transmit frame for sending. */
	frame.ID     = 0x101;
	frame.length = 8;
	frame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) |
	                    CAN_WORD0_DATA_BYTE_1(0x22) |
	                    CAN_WORD0_DATA_BYTE_2(0x33) |
	                    CAN_WORD0_DATA_BYTE_3(0x44);
	frame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) |
	                    CAN_WORD1_DATA_BYTE_5(0x66) |
	                    CAN_WORD1_DATA_BYTE_6(0x77) |
	                    CAN_WORD1_DATA_BYTE_7(0x88);

	/* Writes a transmit message buffer to send a CAN Message. */
	CAN_WriteTxMB(CAN0, TX_MB_INDEX, &frame);

	/* Waits until the transmit message buffer is empty. */
	while (CAN_GetMbStatusFlag(CAN0, TX_MB_INDEX)==false);

	/* Cleans the transmit message buffer empty status. */
	CAN_ClearMbStatusFlag(CAN0,  TX_MB_INDEX);

}
