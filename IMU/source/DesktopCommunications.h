/////////////////////////////////////////////////////////////////////////////////
//                    	   TP2 -  Comunicaciones Serie                         //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////


/**
 *	@file     DesktopCommunication.h
 *	@brief    Implement angle measurement sending through UART.

 */

#ifndef __DESKTOPCOMMUNICATIONS_H_
#define __DESKTOPCOMMUNICATIONSH_

#include "stdint.h"

void desktopCommunicationsInit();

void sendMeasurement2Desktop(uint8_t boardID, uint8_t angleID, int16_t angleVal);

#endif
