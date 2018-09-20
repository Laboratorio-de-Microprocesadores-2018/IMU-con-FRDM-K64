#include "CANCommunications.h"
#include "CircularBuffer.h"
#include "CAN.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

NEW_CIRCULAR_BUFFER(RxBuf,10,sizeof(CAN_DataFrame)); ///SI SE USA LA FIFO DE CAN NO HARIA FALTA UNA FIFO POR SOFT??

#define TX_MB_INDEX 0
#define RX_MB_INDEX 1

static void receiveCallback(CAN_DataFrame frame,CAN_Status status, void* buffer);

void initCANCommunications()
{
	CAN_Config config;
	CAN_GetDefaultConfig(&config);
	config.enableRxMBIndividulMask = false;
	config.RxMBGlobalMask = MASK_ID; /// Ignore the last nibble of the 11 bit ID
	config.maxMbNum = 2;

	CAN_Init(CAN0,&config,50000000);
	CAN_ConfigureRxMB(CAN0,RX_MB_INDEX,BASE_ID);
	CAN_EnableMbInterrupts(CAN0, RX_MB_INDEX, &receiveCallback);
}

void sendMeasurement(Measurement m)
{
	CAN_DataFrame frame;

	/* Prepares the transmit frame for sending. */
	frame.ID     = BOARD_ID;
	frame.length = snprintf((char*)frame.data, sizeof(frame.data), "%d", m.angleVal);

	/* Writes a transmit message buffer to send a CAN Message. */
	CAN_WriteTxMB(CAN0, TX_MB_INDEX, &frame);

}


bool readMeasurement(Measurement * m)
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
	push(buffer,&frame);
}
