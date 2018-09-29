/////////////////////////////////////////////////////////////////////////////////
//                    	   TP2 -  Comunicaciones Serie                         //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/*
 * @file MeasurementProtocol.h
 * @author Tobías Lifschitz
 * @date 21 sep. 2018
 *
 */
#ifndef MEASUREMENTPROTOCOL_H_
#define MEASUREMENTPROTOCOL_H_

#include "stdint.h"
/**
 * The number of bits used in UART communication can be optimized, packing
 * the information "back to back" instead of simply sending the variables,
 * or sending ASCII values.
 *
 * The number of bits needed are:
 *
 * 4 bits boardID (up to 16 IDs)
 * 2 bits angleId (roll, pitch, yaw)
 * 9 bits angleVal (signed range -256 to 255)
 *
 * This gives the following packet formed by three bytes:
 *
 *       b1 = AAAABBCC           b1 = CCCCCCCX            b2 = TERMINATOR
 *
 * Where A:boardID, B=angleID, C:angleVal and X:don't care
 *
 * In order to pack the variables in this way, the following macros are defined
 * and a test code is given below.
 *
 * TEST CODE:
 *
 *	uint8_t boardID = 16;
 *	uint8_t angleID = 2;
 *	int16_t angleVal = -180; // Also try with a negative angleVal!
 *
 *	uint8_t b0 = PACK_BYTE0(boardID, angleID, angleVal);
 *	uint8_t b1 = PACK_BYTE1(boardID, angleID, angleVal);
 *
 *
 *	uint8_t boardID_received = UNPACK_BOARD_ID(b0, b1);
 *	uint8_t angleID_received = UNPACK_ANGLE_ID(b0, b1);
 *	int16_t angleVal_received = UNPACK_ANGLE_VAL(b0, b1);
 *
 */

#define PACKET_SIZE (3)

#define PACKET_TERMINATOR ('\n')

#define BOARD_ID_B0_MASK (0b00001111)
#define BOARD_ID_B0_SHIFT (0x4)

#define ANGLE_ID_B0_MASK (0b00000011)
#define ANGLE_ID_B0_SHIFT  (0x2)

#define ANGLE_VAL_B0_MASK (0b0000000110000000)
#define ANGLE_VAL_B0_SHIFT (0x7)

#define ANGLE_VAL_B1_MASK (0b0000000001111111)
#define ANGLE_VAL_B1_SHIFT (0x1)

#define PACK_BYTE0(boardID,angleID,angleVal) (uint8_t)(((boardID&BOARD_ID_B0_MASK)<<BOARD_ID_B0_SHIFT) | ((angleID&ANGLE_ID_B0_MASK)<< ANGLE_ID_B0_SHIFT) | ((angleVal&ANGLE_VAL_B0_MASK)>>ANGLE_VAL_B0_SHIFT))
#define PACK_BYTE1(boardID,angleID,angleVal) (uint8_t)((angleVal&ANGLE_VAL_B1_MASK)<<ANGLE_VAL_B1_SHIFT)

#define UNPACK_BOARD_ID(b0,b1) (uint8_t)((b0>>BOARD_ID_B0_SHIFT)&BOARD_ID_B0_MASK)
#define UNPACK_ANGLE_ID(b0,b1) (uint8_t)((b0>>ANGLE_ID_B0_SHIFT)&ANGLE_ID_B0_MASK)
#define UNPACK_ANGLE_VAL(b0,b1) (int16_t)( (((int16_t)((int8_t)(b0<<6)))<<1) | ((((int16_t)b1)>>ANGLE_VAL_B1_SHIFT)&ANGLE_VAL_B1_MASK))
// The last macro first shifts 6, then casts to int16_t, and then shifts 1.
// This in order to extend the sign of the number!!!

typedef struct
{
	uint16_t boardID;
	uint8_t angleID;
	int16_t angleVal;
}Measurement;

typedef enum { ROLL, PITCH, YAW }Angle;

#endif /* MEASUREMENTPROTOCOL_H_ */
