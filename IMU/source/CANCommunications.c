#include "CANCommunications.h"
#include "CircularBuffer.h"
#include "CAN.h"
//#include "SysTick.h"
#include "stdlib.h"
#include "stdio.h"
#include "Assert.h"

NEW_CIRCULAR_BUFFER(RxBuf,20,sizeof(CAN_DataFrame)); ///SI SE USA LA FIFO DE CAN NO HARIA FALTA UNA FIFO POR SOFT??

#define TX_MB_INDEX 0
#define RX_MB_INDEX 1



static void receiveCallback(CAN_DataFrame frame,CAN_Status status, void* buffer);
/*
void autoSend()
{
	CAN_DataFrame frame;

	// Prepares the transmit frame for sending.
	frame.ID     = MY_BOARD_ID;
	frame.length = snprintf((char*)frame.data, sizeof(frame.data), "c%d", +184);

	// Writes a transmit message buffer to send a CAN Message.
	CAN_WriteTxMB(CAN0, 0, &frame);
}
*/
void otherBoardCommunicationsInit()
{
	CAN_Config config;
	CAN_GetDefaultConfig(&config);
	config.enableLoopBack = false;
	config.maxMbNum = 2;
	config.enableSelfReception = false;
	config.enableLoopBack = false;

	if(CAN_Init(CAN0,&config,50000000)!=CAN_SUCCESS)
		while(1);

	CAN_ConfigureRxMB(CAN0,RX_MB_INDEX,BASE_ID);
	CAN_SetRxIndividualMask(CAN0,RX_MB_INDEX,MASK_ID);

	CAN_ConfigureRxMB(CAN0,RX_MB_INDEX+1,BASE_ID);
	CAN_SetRxIndividualMask(CAN0,RX_MB_INDEX+1,MASK_ID);

	CAN_ConfigureRxMB(CAN0,RX_MB_INDEX+2,BASE_ID);
	CAN_SetRxIndividualMask(CAN0,RX_MB_INDEX+2,MASK_ID);

	CAN_EnableMbInterrupts(CAN0, RX_MB_INDEX, &receiveCallback);
	CAN_EnableMbInterrupts(CAN0, RX_MB_INDEX+1, &receiveCallback);

	CAN_EnableMbInterrupts(CAN0, RX_MB_INDEX+2, &receiveCallback);

	//sysTickInit();
	//sysTickAddCallback(&autoSend,3);
}

void sendMeasurement2OtherBoards(Measurement m)
{
	CAN_DataFrame frame;

	/* Prepares the transmit frame for sending. */
	frame.ID     = MY_BOARD_ID;
	frame.length = snprintf((char*)frame.data, sizeof(frame.data), "%c%d", m.angleID,m.angleVal);

	/* Writes a transmit message buffer to send a CAN Message. */
	CAN_Status s = CAN_WriteTxMB(CAN0, TX_MB_INDEX, &frame);

	ASSERT(s==CAN_SUCCESS);

}


bool receiveOtherBoardsMeasurement(Measurement * m)
{
	if(isEmpty(&RxBuf))
		return false;
	else
	{
		CAN_DataFrame frame;
		pop(&RxBuf,&frame);
		if(frame.length > 0)
		{
			m->boardID = frame.ID;
			m->angleID = frame.dataByte0;
			frame.data[frame.length]='\0';
			m->angleVal = atoi((char*)frame.data+1);
			return true;
		}
		else
			return false;
	}
}

void receiveCallback(CAN_DataFrame frame,CAN_Status status, void* buffer)
{
	push(&RxBuf,&frame);

}
