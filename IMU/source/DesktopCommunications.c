#include "DesktopCommunications.h"
#include "MeasurementProtocol.h"
#include "UART.h"

void desktopCommunicationsInit()
{
	UARTInit();
	UARTSetBaudRate (UART0, 9600);
}

void sendMeasurement2Desktop(uint8_t boardID, uint8_t angleID, int16_t angleVal)
{

	switch(angleID)
	{
	case 'C':
		angleID=PITCH;
		break;
	case 'R':
		angleID=ROLL;
		break;
	case 'Y':
		angleID=YAW;
		break;

	}
	uint8_t packet[PACKET_SIZE] = { PACK_BYTE0(boardID,angleID,angleVal),
									PACK_BYTE1(boardID,angleID,angleVal),
									PACKET_TERMINATOR};

	UARTSendData(packet, PACKET_SIZE);
}
