/////////////////////////////////////////////////////////////////////////////////
//                       Intertial Motion Unit (IMU)                           //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/*
 * @file CANCommunications.h
 * @author Tobías Lifschitz
 * @date 20 sep. 2018
 *
 */

#ifndef CANCOMMUNICATIONS_H_
#define CANCOMMUNIACTIONS_H_


#define BASE_ID 0b00100000000
#define MASK_ID 0b11111110000
#define MY_BOARD_ID 0x109

#include "stdbool.h"
#include "stdint.h"

typedef struct
{
	uint32_t boardID;
	char angleID; 		///	ID representing roll (rolido 'R') o pitch (cabeceo 'C')
	int angleVal;   /// V
}Measurement;

/**
 * @brief Configures CAN module
 */
void initCANCommunications();

/**
 *  @brief Sends a measurement to other boards connected to CAN bus.
 *  @param m Structure containing measurement information.
 */
void sendMeasurement(Measurement m);


/**
 *  @brief Read a measurement of other boards connected to CAN bus.
 *  @param m Pointer to a Measurement structure to store data.
 *  @return True if a new measurement was read, false if not.
 */
bool readMeasurement(Measurement * m);

#endif
