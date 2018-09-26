/*
 * UART.c
 *
 *  Created on: 14 sep. 2018
 *      Author: sebas
 */

#include <stdint.h>
#include "hardware.h"
#include "UART.h"
//#include "SysTick.h"
#include "CircularBuffer.h"


#define UART_HAL_DEFAULT_BAUDRATE	9600

#define UART_CALL_FREQUENCY			100

#define BUFFER_SIZE					100

#define UART0_TX_PIN 	17   //PTB17
#define UART0_RX_PIN 	16   //PTB16

#define UART0FIFOEXP	(((UART0->PFIFO) & UART_PFIFO_TXFIFOSIZE_MASK) >> (UART_PFIFO_TXFIFOSIZE_SHIFT))
#define UART0FIFOSIZE	(1 << (UART0FIFOEXP + 1))

#define TIESTAT(x)		((((uint8_t)(((uint8_t)(x)) & UART_C2_TIE_MASK)) == UART_C2_TIE_MASK)? true: false)
#define TDRESTAT(x)		((((uint8_t)(((uint8_t)(x)) & UART_S1_TDRE_MASK)) == UART_S1_TDRE_MASK)? true: false)
// #define TDRESTAT(x)		(((uint8_t)(((uint8_t)(x)) & UART_S1_TDRE_MASK)) >> UART_S1_TDRE_SHIFT)

#define RIESTAT(x)		((((uint8_t)(((uint8_t)(x)) & UART_C2_RIE_MASK)) == UART_C2_RIE_MASK)? true: false)
#define RDRFSTAT(x)		((((uint8_t)(((uint8_t)(x)) & UART_S1_RDRF_MASK)) == UART_S1_RDRF_MASK)? true: false)
// #define RDRFSTAT(x)		(((uint8_t)(((uint8_t)(x)) & UART_S1_RDRF_MASK)) >> UART_S1_RDRF_SHIFT)

#define TXOFStat(x)		(((uint8_t)(((uint8_t)(x)) & UART_SFIFO_TXOF_MASK)) >> UART_SFIFO_TXOF_SHIFT)
#define RXOFStat(x)		(((uint8_t)(((uint8_t)(x)) & UART_SFIFO_RXOF_MASK)) >> UART_SFIFO_RXOF_SHIFT)

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

typedef enum{TXOF_ERR = 0, BUFFFULL_ERR, BUFFEMPTY_ERR, UART0IRQ_ERR, NO_ERR};

static uint8_t err;

static uint8_t transferWord;

static bool errFlag;

NEW_CIRCULAR_BUFFER(transmitBuffer,BUFFER_SIZE,sizeof(transferWord));
NEW_CIRCULAR_BUFFER(recieveBuffer,BUFFER_SIZE,sizeof(transferWord));

//static void UARTPisr(void);
static void transmitData(void);
static void recieveData(void);
static void loadBuffer2Register(void);
static void loadRegister2Buffer(void);


void UARTInit (void)
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

	UARTSetBaudRate(UART0, UART_HAL_DEFAULT_BAUDRATE);

	//Configure UART0 TX and RX PINS

	PORTB->PCR[UART0_TX_PIN] = 0x0; //Clear all bits
	PORTB->PCR[UART0_TX_PIN] |= PORT_PCR_MUX(PORT_mAlt3); 	 //Set MUX to UART0
	PORTB->PCR[UART0_TX_PIN] |= PORT_PCR_IRQC(PORT_eDisabled); //Disable interrupts
//----------------------------------------------------------------------------------
	PORTB->PCR[UART0_RX_PIN] = 0x0; //Clear all bits
	PORTB->PCR[UART0_RX_PIN] |= PORT_PCR_MUX(PORT_mAlt3); 	 //Set MUX to UART0
	PORTB->PCR[UART0_RX_PIN] |= PORT_PCR_IRQC(PORT_eDisabled); //Disable interrupts



	UART0->PFIFO = UART_PFIFO_RXFE_MASK | UART_PFIFO_TXFE_MASK;

	//Enable UART0 Xmiter and Rcvr
	UART0->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK | UART_C2_RIE_MASK;

	err = NO_ERR;

/*	The callback of the display is added to the sysTick callback list*/
// SysTickAddCallback(&UARTPisr,(float)(1/UART_CALL_FREQUENCY));

}

void UARTSetBaudRate (UART_Type *uart, uint32_t baudrate)
{
	uint16_t sbr, brfa;
	uint32_t clock;

	clock = ((uart == UART0) || (uart == UART1)) ?(__CORE_CLOCK__):(__CORE_CLOCK__ >> 1);

	baudrate = ((baudrate == 0)?(UART_HAL_DEFAULT_BAUDRATE):
			((baudrate > 0x1FFF)?(UART_HAL_DEFAULT_BAUDRATE):(baudrate)));

	sbr = clock / (baudrate << 4);               // sbr = clock/(Baudrate x 16)
	brfa = (clock << 1) / baudrate - (sbr << 5); // brfa = 2*Clock/baudrate - 32*sbr

	uart->BDH = UART_BDH_SBR(sbr >> 8);
	uart->BDL = UART_BDL_SBR(sbr);
	uart->C4 = (uart->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);
}



uint8_t UARTSendData( uint8_t * tx_data, uint8_t len)
{
	if( len > 0)
	{
		for(int i = 0; (i < len) && (err == NO_ERR); i++)
		{
			if(!push(&transmitBuffer, &tx_data[i]))
			{
				err = BUFFFULL_ERR;
				return false;
			}
		}
		UART0->C2 |= UART_C2_TIE_MASK;
	}

	return true;
}

uint8_t UARTRecieveData( uint8_t * rx_data, uint8_t len)
{
	uint8_t lenRet = 0 ;
	while( (lenRet < len) && pop(&recieveBuffer, &rx_data[lenRet]))
		lenRet ++;
	return lenRet;
}


__ISR__ UART0_RX_TX_IRQHandler(void)
{
	uint8_t debugSRegister, debugCRegister;
	debugSRegister = UART0->S1;
	debugCRegister = UART0->C2;

	if(TDRESTAT(UART0->S1) && TIESTAT(UART0->C2))
		transmitData();

	else if(RDRFSTAT(UART0->S1) && RIESTAT(UART0->C2))
		recieveData();

	else{}
		//err = UART0IRQ_ERR;

}

void transmitData(void)
{
	uint8_t debugRegister;
	debugRegister = UART0->S1;

	int tBuffCount = numel(&transmitBuffer);
	int FIFOLeft = UART0FIFOSIZE - (UART0->TCFIFO);
	if( tBuffCount < FIFOLeft )
		UART0->C2 &= (~UART_C2_TIE_MASK);

	if(isEmpty(&transmitBuffer) == false)
	{
		for(int i = 0; (i < (tBuffCount - 1)) && (i < (FIFOLeft-1)) && (err == NO_ERR); i++)
		{
			loadBuffer2Register();
		}
		if(err == NO_ERR)
		{
			bool a = TDRESTAT(UART0->S1);
			bool b = !TXOFStat(UART0->SFIFO);
			//a = ( TDRESTAT(UART0->S1) || !TXOFStat(UART0->SFIFO) );
			if( a || b)
			{
				loadBuffer2Register();
			}else
			{
				err = TXOF_ERR;
				return;
			}
		}

	}

}
void recieveData(void)
{
	if(isFull(&recieveBuffer) == false)
	{
		int FIFOCount = UART0->RCFIFO;
		for(int i = 0; (i < (FIFOCount - 1)) && (err == NO_ERR); i++)
		{
			loadRegister2Buffer();
		}
		if(err == NO_ERR)
		{
			if((RDRFSTAT(UART0->S1)) && (err == NO_ERR))
			{
				loadRegister2Buffer();
			}else
			{
				errFlag = true;
				return;
			}
		}
	}else
	{
		err = BUFFFULL_ERR;
		return;
	}
}

void loadBuffer2Register(void)
{
	if(pop(&transmitBuffer, &transferWord))
		UART0->D = transferWord;
	else
	{
		err = BUFFEMPTY_ERR;
		return;
	}
}

void loadRegister2Buffer(void)
{
	transferWord = UART0->D;
	if(!push(&recieveBuffer, &transferWord))
	{
		err = BUFFFULL_ERR;
		return;
	}
}
