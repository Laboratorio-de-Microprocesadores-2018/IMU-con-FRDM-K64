/*
 * UART.c
 *
 *  Created on: 14 sep. 2018
 *      Author: sebas
 */

#include <stdint.h>
#include "hardware.h"
#include "UART.h"


#define UART_HAL_DEFAULT_BAUDRATE 9600

#define UART0_TX_PIN 	17   //PTB17
#define UART0_RX_PIN 	16   //PTB16

typedef enum
{
	PORT_mAnalog,
	PORT_mGPIO,
	PORT_mAlt2,
	PORT_mAlt3,
	PORT_mAlt4,
	PORT_mAlt5,
	PORT_mAlt6,
	PORT_mAlt7,

} PORTMux_t;

typedef enum
{
	PORT_eDisabled				= 0x00,
	PORT_eDMARising				= 0x01,
	PORT_eDMAFalling			= 0x02,
	PORT_eDMAEither				= 0x03,
	PORT_eInterruptDisasserted	= 0x08,
	PORT_eInterruptRising		= 0x09,
	PORT_eInterruptFalling		= 0x0A,
	PORT_eInterruptEither		= 0x0B,
	PORT_eInterruptAsserted		= 0x0C,
} PORTEvent_t;



void UART_Init (void)
{

// Note: 5.6 Clock Gating page 192
// Any bus access to a peripheral that has its clock disabled generates an error termination.
	    SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	    SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
/*		SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
		SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
		SIM->SCGC4 |= SIM_SCGC4_UART3_MASK;
		SIM->SCGC1 |= SIM_SCGC1_UART4_MASK;
		SIM->SCGC1 |= SIM_SCGC1_UART5_MASK;
*/

	    NVIC_EnableIRQ(UART0_RX_TX_IRQn);
/*		NVIC_EnableIRQ(UART1_RX_TX_IRQn);
		NVIC_EnableIRQ(UART2_RX_TX_IRQn);
		NVIC_EnableIRQ(UART3_RX_TX_IRQn);
		NVIC_EnableIRQ(UART4_RX_TX_IRQn);
		NVIC_EnableIRQ(UART5_RX_TX_IRQn);
*/

	    //UART0 Set UART Speed

		UART_SetBaudRate(UART0, UART_HAL_DEFAULT_BAUDRATE);

		//Configure UART0 TX and RX PINS

		PORTB->PCR[UART0_TX_PIN] = 0x0; //Clear all bits
		PORTB->PCR[UART0_TX_PIN] |= PORT_PCR_MUX(PORT_mAlt3); 	 //Set MUX to UART0
		PORTB->PCR[UART0_TX_PIN] |= PORT_PCR_IRQC(PORT_eDisabled); //Disable interrupts
//----------------------------------------------------------------------------------
		PORTB->PCR[UART0_RX_PIN] = 0x0; //Clear all bits
		PORTB->PCR[UART0_RX_PIN] |= PORT_PCR_MUX(PORT_mAlt3); 	 //Set MUX to UART0
		PORTB->PCR[UART0_RX_PIN] |= PORT_PCR_IRQC(PORT_eDisabled); //Disable interrupts



	//Enable UART0 Xmiter and Rcvr

	UART0->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;





}


void UART_SetBaudRate (UART_Type *uart, uint32_t baudrate)
{
	uint16_t sbr, brfa;
	uint32_t clock;

	clock = ((uart == UART0) || (uart == UART1))?(__CORE_CLOCK__):(__CORE_CLOCK__ >> 1);

	baudrate = ((baudrate == 0)?(UART_HAL_DEFAULT_BAUDRATE):
			((baudrate > 0x1FFF)?(UART_HAL_DEFAULT_BAUDRATE):(baudrate)));

	sbr = clock / (baudrate << 4);               // sbr = clock/(Baudrate x 16)
	brfa = (clock << 1) / baudrate - (sbr << 5); // brfa = 2*Clock/baudrate - 32*sbr

	uart->BDH = UART_BDH_SBR(sbr >> 8);
	uart->BDL = UART_BDL_SBR(sbr);
	uart->C4 = (uart->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}



void UART_Send_Data(unsigned char tx_data)
{
	while(((UART0->S1)& UART_S1_TDRE_MASK) ==0);

				UART0->D = tx_data;
}

unsigned char UART_Recieve_Data(void)
{
	while(((UART0->S1)& UART_S1_RDRF_MASK) ==0);

			return(UART0->D);
}


