/////////////////////////////////////////////////////////////////////////////////
//                       Intertial Motion Unit (IMU)                           //
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
 * the information "back to back" in stead of simply sending the varaibles.
 * The number of bits needed are:
 *
 * 11 bits boardID + 2 bits angleId + 10 bits angleVal (signed range +-360)
 *
 * This gives the following packet formed by three bytes:
 *
 *       b1=AAAAAAAA           b1=AAABBCCC           b2=CCCCCCCX <-- este bit podria ser siempre un 0 para detectar errores??
 *
 * Where A:boardID, B=angleID, C:angleVal and X:don't care
 *
 * In order to pack the variables in this way, the following macros are defined
 * and a test code is given below.
 *
 * TEST CODE:
 *
 * uint16_t boardID = 500;
 * uint8_t angleID = 2;
 * int16_t angleVal = 345; // Also try with a negative angleVal!
 *
 * uint8_t b0 = PACK_BYTE0(boardID,angleID,angleVal);
 * uint8_t b1 = PACK_BYTE1(boardID,angleID,angleVal);
 * uint8_t b2 = PACK_BYTE2(boardID,angleID,angleVal);
 *
 * uint16_t boardID_received = UNPACK_BOARD_ID(b0,b1,b2);
 * uint8_t angleID_received = UNPACK_ANGLE_ID(b0,b1,b2);
 * int16_t angleVal_received = UNPACK_ANGLE_VAL(b0,b1,b2);
 *
 */

#define PACKET_SIZE (3)

#define BOARD_ID_B0_MASK (0b0000011111111000)
#define BOARD_ID_B0_SHIFT (0x3)

#define BOARD_ID_B1_MASK (0b0000000000000111)
#define BOARD_ID_B1_SHIFT (0x5)

#define ANGLE_ID_B1_MASK (0b0000000000000011)
#define ANGLE_ID_B1_SHIFT  (0x3)

#define ANGLE_VAL_B1_MASK (0b0000001110000000)
#define ANGLE_VAL_B1_SHIFT (0x7)

#define ANGLE_VAL_B2_MASK (0b0000000001111111)
#define ANGLE_VAL_B2_SHIFT (0x1)

#define PACK_BYTE0(boardID,angleID,angleVal) (uint8_t)((boardID&BOARD_ID_B0_MASK)>>BOARD_ID_B0_SHIFT)
#define PACK_BYTE1(boardID,angleID,angleVal) (uint8_t)(((boardID&BOARD_ID_B1_MASK)<<BOARD_ID_B1_SHIFT) | ((angleID&ANGLE_ID_B1_MASK)<<ANGLE_ID_B1_SHIFT) | ((angleVal&ANGLE_VAL_B1_MASK)>>ANGLE_VAL_B1_SHIFT))
#define PACK_BYTE2(boardID,angleID,angleVal) (uint8_t)((angleVal&ANGLE_VAL_B2_MASK)<<ANGLE_VAL_B2_SHIFT)

#define UNPACK_BOARD_ID(b0,b1,b2) (uint16_t)(((((uint16_t)b0)<<BOARD_ID_B0_SHIFT)&BOARD_ID_B0_MASK) | ((((uint16_t)b1)>>BOARD_ID_B1_SHIFT)&BOARD_ID_B1_MASK))
#define UNPACK_ANGLE_ID(b0,b1,b2) (uint8_t)((b1>>ANGLE_ID_B1_SHIFT)&ANGLE_ID_B1_MASK)
#define UNPACK_ANGLE_VAL(b0,b1,b2) (int16_t)( (((int16_t)((int8_t)(b1<<5)))<<2) | ((((int16_t)b2)>>ANGLE_VAL_B2_SHIFT)&ANGLE_VAL_B2_MASK))
// The last macro first shifts 5, then casts to int16_t, and then shifts2.
// This in order to extend the sign of the number!!!


#endif /* MEASUREMENTPROTOCOL_H_ */
