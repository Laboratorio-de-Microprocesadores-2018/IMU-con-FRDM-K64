/////////////////////////////////////////////////////////////////////////////////
//                       Intertial Motion Unit (IMU)                           //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/*
 * @file CAN.h
 * @author Tobías Lifschitz
 * @date 12 sep. 2018
 *
 */

#ifndef CAN_H_
#define CAN_H_

/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include "stdbool.h"
#include "stdint.h"
#include "MK64F12.h"

/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

typedef enum {CAN_SUCCESS,CAN_FAILED,CAN_ERROR,CAN_NON_STD_BAUD,CAN_RX_BUSY,CAN_RX_OVERFLOW}CAN_Status;

typedef enum {CAN_OSC_CLOCK,CAN_PERI_CLOCK}CAN_ClockSource;

typedef enum {CAN_SINGLE_SAMPLE,CAN_TRIPLE_SAMPLE}CAN_SamplingMode;

typedef enum {
  CAN_BUS_OFF_INTERRUPT = CAN_CTRL1_BOFFMSK_MASK,
  CAN_EROR_INTERRUPT = CAN_CTRL1_ERRMSK_MASK,
  CAN_RX_WARNING_INTERRUPT = CAN_CTRL1_RWRNMSK_MASK,
  CAN_TX_WARNING_INTERRUPT = CAN_CTRL1_TWRNMSK_MASK,
  CAN_WAKE_UP_INTERRUPT = CAN_MCR_WAKMSK_MASK
} CAN__Interrupt;



typedef struct{
	uint32_t baudRate;		 // CAN baud rate in bps.
	CAN_ClockSource clkSrc; // Clock source for CAN Protocol Engine.
	CAN_SamplingMode sampling;
	uint8_t maxMbNum;		 // The maximum number of Message Buffers used by user.
	bool enableLoopBack; 	 // Enable or Disable Loop Back Self Test Mode.
	bool enableSelfReception;
	bool enableRxMBIndividulMask; // Enable individual masks for MB (if not use global masks, see MCR[IMRQ]).
	uint32_t RxMBGlobalMask; // Global mask used during matching process if individual MB masking is disabled.
}CAN_Config;

typedef struct{

}CAN_FIFOConfig;


typedef struct{
	uint32_t ID;
	uint32_t length;
	union{
		struct{
			uint32_t dataWord0;
			uint32_t dataWord1;
		};
		struct{
			uint8_t dataByte0;
			uint8_t dataByte1;
			uint8_t dataByte2;
			uint8_t dataByte3;
			uint8_t dataByte4;
			uint8_t dataByte5;
			uint8_t dataByte6;
			uint8_t dataByte7;
		};
		uint8_t data[8];
	};
}CAN_DataFrame;


typedef void (*CAN_MB_Callback)(CAN_DataFrame frame,CAN_Status status, void * data);

/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////

/*
 * @brief Returns the default configuration for CAN module.
 * @param config Pointer to struct to store default configuration.
 */
void CAN_GetDefaultConfig(CAN_Config * config);

/*
 * @brief Initialize CAN module
 * @param config Pointer to a struct containing module configuration.
 * @param clockHz Protocol Engine clock source frequency in Hz.
 */
CAN_Status CAN_Init(CAN_Type * base, CAN_Config * config, uint32_t sourceClockHz );

/*
 * @brief Disable CAN module clock and reset all registers.
 * @param base CAN peripheral base address.
 */
void CAN_Deinit(CAN_Type * base);

/**
 * @brief Enable the CAN module.
 * @param base CAN peripheral base address.
 */
void CAN_Enable(CAN_Type * base);

/**
 * @brief Disable the CAN module.
 * @param base CAN peripheral base address..
 */
void CAN_Disable(CAN_Type * base);


/**
 * @brief Configure a message buffer for receiving.
 * @param base CAN peripheral base address
 * @param index Number of message buffer.
 * @param config Pointer to struct containing MB Rx configuration
 */
void  CAN_ConfigureRxMB(CAN_Type * base,uint8_t index,uint32_t ID);


/**
 * @breif Enable interrupt of a message buffer and attach a callback.
 * @param base CAN peripheral base address
 * @param index Number of message buffer.
 * @param config Pointer to struct containing MB Rx configuration
 */
void CAN_EnableMbInterrupts	(CAN_Type * base, uint8_t index, CAN_MB_Callback callback);

/**
 *
 */
CAN_Status CAN_ConfigureRxFifo(CAN_Type * base, CAN_FIFOConfig * config);

/**
 * @brief Sets the global mask for CAN message buffers during matching process.
 *
 * Note that the configuration is only effective when the Rx individual mask
 * is disabled when calling CAN_Init().
 *
 */
void CAN_SetRxMbGlobalMask	(CAN_Type *	base, uint32_t 	mask);
void CAN_SetRxIndividualMask (CAN_Type *	base, uint8_t index, uint32_t 	mask);

/**
 * @brief Poll the flag status of a message buffer.
 * @param base CAN peripheral base address
 * @param index Number of message buffer.
 * @return True if a message was sent/received, false if not.
 */
bool CAN_GetMbStatusFlag(CAN_Type * base,uint8_t index);

/**
 * @brief Clear the interrupt flag of the indicated message buffer
 * @param base CAN peripheral base address
 * @param index Number of message buffer.
 */
void CAN_ClearMbStatusFlag(CAN_Type * base,uint8_t index);


/*
 * @brief Read a frame from a message buffer.
 * @param base CAN peripheral base address
 * @param index Number of message buffer to read.
 * @param frame Pointer to frame to store received data.
 * @return
 */
CAN_Status CAN_ReadRxMB(CAN_Type * base,uint8_t index, CAN_DataFrame * frame);

/*
 * @brief Write a frame to a message buffer to be sent.
 * @param base CAN peripheral base address
 * @param index Number of message buffer to write.
 * @param frame Pointer to frame to be sent.
 * @return
 */
CAN_Status CAN_WriteTxMB(CAN_Type * base,uint8_t index, CAN_DataFrame * frame);

#endif /* CAN_H_ */
