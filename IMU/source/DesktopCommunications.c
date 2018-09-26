#include "DesktopCommunications.h"
#include "MeasurementProtocol.h"
#include "UART.h"

void desktopCommunicationsInit()
{
	UARTInit();
	UARTSetBaudRate (UART0, 9600);
}

void sendMeasurement2Desktop(uint16_t boardID, uint8_t angleID, int16_t angleVal)
{
	uint8_t packet[PACKET_SIZE] = { PACK_BYTE0(boardID,angleID,angleVal),
									PACK_BYTE1(boardID,angleID,angleVal),
									PACK_BYTE2(boardID,angleID,angleVal)};

	UARTSendData(packet, PACKET_SIZE);
}
