/////////////////////////////////////////////////////////////////////////////////
//                        Intertial Motion Unit (IMU)						   //
//																			   //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/*
 * @file CAN.c
 * @author Tobías Lifschitz
 * @date 12 sep. 2018
 *
 */

#include "CAN.h"
#include "MK64F12.h"
#include "Board.h"
#include "stdlib.h"

#define PIN_CAN0_TX PORTNUM2PIN(PB,18)
#define PIN_CAN0_RX PORTNUM2PIN(PB,19)
#define PIN_CAN0_STB PORTNUM2PIN(PC,1)

// Uncomment this define to raise a pin when in a CAN function
//#define MEASURE_CAN

#ifdef MEASURE_CAN
	#define MEASURE_CAN_PORT PORTC
	#define MEASURE_CAN_GPIO GPIOC
	#define MEASURE_CAN_PIN	8
#endif

typedef enum{RX_INACTIVE = 0b0000,
			 RX_EMPTY    = 0b0100,
		     RX_FULL     = 0b0010,
		     RX_OVERRUN  = 0b0011,
		     RX_RANSWER  = 0b1010,
			 RX_BUSY     = 0b0001}RxMBCode;

typedef enum{TX_INACTIVE = 0b1000,
			 TX_ABORT    = 0b1001,
			 TX_DATA     = 0b1100, // RTR = 0
			 TX_REMOTE   = 0b1100, // RTR = 1
			 TX_TANSWER  = 0b1110}TxMBCode;

typedef struct{
	uint32_t baudRate;
	//uint8_t prescaler;
	//uint8_t numberOfQuantas;
	//uint8_t seg1;
	//uint8_t seg2;
	//uint8_t samplePoint;
	uint8_t PRESDIV;
	uint8_t PROPSEG;
	uint8_t PSEG1;
	uint8_t PSEG2;
}TimingParameters;






static void CAN_Freeze(CAN_Type * base, bool freeze);

// Static array to store messaage buffer callbacks
static CAN_MB_Callback callbacks[CAN_CS_COUNT];
static void * callbacksData[CAN_CS_COUNT];

void CAN_GetDefaultConfig(CAN_Config * config)
{
	 config->clkSrc = CAN_OSC_CLOCK;
	 config->sampling = CAN_SINGLE_SAMPLE;
	 config->baudRate = 125000U;
	 config->maxMbNum = 16;
	 config->enableLoopBack = false;
	 config->enableSelfReception = false;
	 config->enableRxMBIndividulMask = true;
}



CAN_Status CAN_Init(CAN_Type * base, CAN_Config * config, uint32_t sourceClockHz )
{
#ifdef MEASURE_CAN
	MEASURE_CAN_PORT->PCR[MEASURE_CAN_PIN] = PORT_PCR_MUX(1);
	MEASURE_CAN_GPIO->PDDR |= (1<<MEASURE_CAN_PIN);
	MEASURE_CAN_GPIO->PDOR &= ~(1<<MEASURE_CAN_PIN);
#endif

	/// Enable clock for CAN module
	SIM->SCGC6 |= SIM_SCGC6_FLEXCAN0_MASK;

	/// Enable module and force a soft reset
	CAN_Enable(base);

	/// Soft reset
	base->MCR |= CAN_MCR_SOFTRST(1);
	while((base->MCR&CAN_MCR_SOFTRST_MASK)>> CAN_MCR_SOFTRST_SHIFT);

	/// Configure the TX and RC pins (B18 B19)
	PORT_Type * ports[] = PORT_BASE_PTRS;
	uint32_t PCR = PORT_PCR_MUX(2) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; // REVISAR COMO CONFIGURAR BIEN EL PCR
	ports[PIN_CAN0_TX/32]->PCR[PIN_CAN0_TX%32] = PCR;
	ports[PIN_CAN0_RX/32]->PCR[PIN_CAN0_RX%32] = PCR;


	/// Disable CAN module in order to modify registers
	CAN_Disable(base);

	/// Select clock source for CAN by setting/clearing CTRL1[CLK_SRC] bit.
	if(config->clkSrc==CAN_OSC_CLOCK)
		base->CTRL1 = CAN_CTRL1_CLKSRC(0);
	else if(config->clkSrc==CAN_PERI_CLOCK)
		base->CTRL1 = CAN_CTRL1_CLKSRC(1);

	/// Enable CAN module.
	CAN_Enable(base);

	/// Module will enter freeze mode automatically.
	/// Wait until module goes into freeze mode (MCR[FRZ_ACK] = 1).
	while((base->MCR & CAN_MCR_FRZACK_MASK)!=CAN_MCR_FRZACK_MASK);

	/// Timing parameters (source: http://www.bittiming.can-wiki.info/#Freescale)
	TimingParameters table[] =
	//BitRate | PRESDIV | PROPSEG | PSEG1 | PSEG2 | Prescaler | # quantas | Seg 1 | Seg 2 | Sample Point % |
	{
	{1000000  ,   0x01  ,   0x07  ,  0x07 ,  0x07},	/* 2, 25, 16, 8, 68,*/
	{500000   ,   0x04  ,   0x07  ,  0x07 ,  0x02},	/* 5, 20, 16, 3, 85,*/
	{250000   ,   0x09  ,   0x07  ,  0x07 ,  0x02},	/*10, 20, 16, 3, 85,*/
	{125000   ,   0x13  ,   0x07  ,  0x07 ,  0x02},	/*20, 20, 16, 3, 85,*/
	{100000   ,   0x18  ,   0x07  ,  0x07 ,  0x02},	/*25, 20, 16, 3, 85,*/
	{83333    ,	  0x1d  ,   0x07  ,  0x07 ,  0x02},	/*30, 20, 16, 3, 85,*/
	{50000    ,	  0x31  ,   0x07  ,  0x07 ,  0x02},	/*50, 20, 16, 3, 85,*/
	{20000    ,	  0x7c  ,   0x07  ,  0x07 ,  0x02},	/*125, 20, 16, 3, 85,*/
	{10000    ,	  0xf9  ,   0x07  ,  0x07 ,  0x02},	/*250, 20, 16, 3, 85,*/
	};
	uint8_t nBaudSettings = sizeof(table)/sizeof(TimingParameters);

	int i;
	for(i=0; i<nBaudSettings; i++)
	{
		if(table[i].baudRate == config->baudRate)
		{
			base->CTRL1 |= CAN_CTRL1_PRESDIV(table[i].PRESDIV);
			base->CTRL1 |= CAN_CTRL1_PROPSEG(table[i].PROPSEG);
			base->CTRL1 |= CAN_CTRL1_PSEG1(table[i].PSEG1);
			base->CTRL1 |= CAN_CTRL1_PSEG2(table[i].PSEG2);
			break;
		}
	}

	/// Given baud rate has no pre-calculated settings.. sorry :)
	if(i==nBaudSettings)
		return CAN_NON_STD_BAUD;

	if(config->enableSelfReception == false)
		base->MCR |= CAN_MCR_SRXDIS_MASK;

	if(config->enableLoopBack == true)
	{
		base->CTRL1 |= CAN_CTRL1_LPB_MASK;
		base->MCR &= ~CAN_MCR_SRXDIS_MASK;
	}

	if(config->sampling == CAN_TRIPLE_SAMPLE)
		base->CTRL1 |= CAN_CTRL1_SMP_MASK;

	if(config->enableRxMBIndividulMask == true)
		base->MCR |= CAN_MCR_IRMQ_MASK;
	else
		base->RXMGMASK = config->RxMBGlobalMask;

	// Reset all message buffers
	for(int i=0; i<CAN_CS_COUNT; i++)
	{
		base->MB[i].CS = ( base->MB[i].CS &= ~CAN_CS_CODE_MASK ) | CAN_CS_CODE(RX_INACTIVE);
		base->MB[i].ID = CAN_ID_STD(0);
		base->RXIMR[i] = 0xFFFFFFFF;
	}

	// Exit Freeze mode
	CAN_Freeze(base,false);

	return CAN_SUCCESS;
}



void CAN_Deinit(CAN_Type * base)
{
	/// Soft reset
	base->MCR |= CAN_MCR_SOFTRST(1);
	while((base->MCR&CAN_MCR_SOFTRST_MASK)>> CAN_MCR_SOFTRST_SHIFT);

	/// Disable module
	CAN_Disable(base);

	/// And disable clock module from SIM
	SIM->SCGC6 &= ~SIM_SCGC6_FLEXCAN0_MASK;
}



void CAN_Enable(CAN_Type * base)
{
	base->MCR &= ~CAN_MCR_MDIS_MASK;
	// Wait until CAN module is out of low power mode (MCR[LPM_ACK] = 0).
	while(base->MCR & CAN_MCR_LPMACK_MASK);
}



void CAN_Disable(CAN_Type * base)
{
	base->MCR |= CAN_MCR_MDIS_MASK;
	// Wait until module is in low power mode (MCR[LPM_ACK] = 1).
	while((base->MCR & CAN_MCR_LPMACK_MASK)!=CAN_MCR_LPMACK_MASK);
}




void  CAN_ConfigureRxMB(CAN_Type * base, uint8_t index, uint32_t ID)
{
	/// If the Mailbox is active (either Tx or Rx) inactivate the Mailbox.
	base->MB[index].CS = ( base->MB[index].CS &= ~CAN_CS_CODE_MASK ) | CAN_CS_CODE(RX_INACTIVE);

	/// Write the ID word.
	base->MB[index].ID = CAN_ID_STD(ID);

	/// Write the EMPTY code to the CODE field of the Control and Status word to activate the Mailbox.
	base->MB[index].CS = CAN_CS_CODE(RX_EMPTY) | CAN_CS_IDE(0);
}


void CAN_EnableMbInterrupts	(CAN_Type * base, uint8_t index, CAN_MB_Callback callback)
{
	NVIC_EnableIRQ(CAN0_ORed_Message_buffer_IRQn);
	base->IMASK1 |= (1UL<<index);
	callbacks[index] = callback;
}

void CAN_DisableMbInterrupts	(CAN_Type * base, uint8_t index)
{
	base->IMASK1 &= ~(1UL<<index);
	callbacks[index] = NULL;
}



void CAN_SetRxMbGlobalMask	(CAN_Type *	base, uint32_t 	mask)
{
	CAN_Freeze(base,true);
	base->RXMGMASK = mask;
	CAN_Freeze(base,false);
}
void CAN_SetRxIndividualMask (CAN_Type *	base, uint8_t index, uint32_t 	mask)
{
	CAN_Freeze(base,true);
	base->RXIMR[index] = mask;
	CAN_Freeze(base,false);
}

bool CAN_GetMbStatusFlag(CAN_Type * base,uint8_t index)
{
	return (((base->IFLAG1>>index)&1UL)==1UL);
}

void CAN_ClearMbStatusFlag(CAN_Type * base,uint8_t index)
{
	base->IFLAG1 |= (1<<index); // W1C
}

CAN_Status CAN_ReadRxMB(CAN_Type * base,uint8_t index, CAN_DataFrame * frame)
{
#ifdef MEASURE_CAN
	BITBAND_REG(MEASURE_CAN_GPIO->PDOR, MEASURE_CAN_PIN) = 1;
#endif

	/// Check if the BUSY bit is deasserted.
	if(base->MB[index].CS>>CAN_CS_CODE_SHIFT & 1UL)
		return CAN_RX_BUSY;

	uint32_t code = (base->MB[index].CS & CAN_CS_CODE_MASK)>>CAN_CS_CODE_SHIFT;
	CAN_Status retVal;
	switch(code)
	{
	case RX_EMPTY:
		retVal = CAN_FAILED;
		break;

	case RX_FULL:
		/// Read the contents of the Mailbox.
		frame->ID = (base->MB[index].ID & CAN_ID_STD_MASK)>> CAN_ID_STD_SHIFT;
		frame->length = (base->MB[index].CS & CAN_CS_DLC_MASK) >> CAN_CS_DLC_SHIFT;

		uint32_t W0 = base->MB[index].WORD0;
		frame->dataWord0 =  ((W0 & CAN_WORD0_DATA_BYTE_0_MASK)>>24)|
							((W0 & CAN_WORD0_DATA_BYTE_1_MASK)>>8)|
							((W0 & CAN_WORD0_DATA_BYTE_2_MASK)<<8)|
							((W0 & CAN_WORD0_DATA_BYTE_3_MASK)<<24);

		uint32_t W1 = base->MB[index].WORD1;
		frame->dataWord1 =  ((W1 & CAN_WORD1_DATA_BYTE_4_MASK)>>24)|
							((W1 & CAN_WORD1_DATA_BYTE_5_MASK)>>8)|
							((W1 & CAN_WORD1_DATA_BYTE_6_MASK)<<8)|
							((W1 & CAN_WORD1_DATA_BYTE_7_MASK)<<24);
		/// Acknowledge the proper flag at IFLAG registers.
		base->IFLAG1 |= (1<<index); // W1C

		/// Read the Free Running Timer to unlock Mailbox as soon as possible and make it available for reception.
		base->TIMER;

		retVal = CAN_SUCCESS;
		break;

	case RX_OVERRUN:
		retVal =  CAN_RX_OVERFLOW;
		break;
	}

#ifdef MEASURE_CAN
	BITBAND_REG(MEASURE_CAN_GPIO->PDOR, MEASURE_CAN_PIN) = 0;
#endif
	return retVal;
}



CAN_Status  CAN_WriteTxMB(CAN_Type * base,uint8_t index, CAN_DataFrame * frame)
{
#ifdef MEASURE_CAN
	BITBAND_REG(MEASURE_CAN_GPIO->PDOR, MEASURE_CAN_PIN) = 1;
#endif
	CAN_Status retVal;
	if(index < CAN_CS_COUNT) // CHEQUEAR QUE NO PERTENEZCA A LA FIFO SI ESTA ACTIVA
	{
		/// Check whether the respective interrupt bit is set and clear it.
		if(base->IFLAG1& (1<<index))
			base->IFLAG1 |= (1<<index); // W1C

		/// Write INACTIVE code to the CODE field (a pending frame may be transmitted without notification).
		base->MB[index].CS = CAN_CS_CODE(TX_INACTIVE);

		/// Write the ID word.
		base->MB[index].ID = CAN_ID_STD(frame->ID);

		/// Write the data bytes in order using macros.
		base->MB[index].WORD0 = CAN_WORD0_DATA_BYTE_0(frame->dataWord0) |
		                    	CAN_WORD0_DATA_BYTE_1(frame->dataWord0) |
		                    	CAN_WORD0_DATA_BYTE_2(frame->dataWord0) |
		                    	CAN_WORD0_DATA_BYTE_3(frame->dataWord0);
		base->MB[index].WORD1 = CAN_WORD1_DATA_BYTE_4(frame->dataWord1) |
		                        CAN_WORD1_DATA_BYTE_5(frame->dataWord1) |
		                    	CAN_WORD1_DATA_BYTE_6(frame->dataWord1) |
		                    	CAN_WORD1_DATA_BYTE_7(frame->dataWord1);

		/// Write the DLC and CODE fields of the Control and Status word to activate the MB.
		base->MB[index].CS = CAN_CS_CODE(TX_DATA) | CAN_CS_DLC(frame->length) | CAN_CS_SRR(1) | CAN_CS_IDE(0);


		retVal = CAN_SUCCESS;
	}
	else
		retVal = CAN_ERROR;

#ifdef MEASURE_CAN
	BITBAND_REG(MEASURE_CAN_GPIO->PDOR, MEASURE_CAN_PIN) = 0;
#endif
	return retVal;
}



// CUANDO INICIALICE LA FIFO CONFIGURAR EL IDAM


/*
/// Enable interrupts for FlexCAN module (Tengo que habilitar todos aca???)
NVIC_EnableIRQ(CAN_Rx_Warning_IRQSn);
NVIC_EnableIRQ(CAN_Tx_Warning_IRQSn);
NVIC_EnableIRQ(CAN_Wake_Up_IRQSn);
NVIC_EnableIRQ(CAN_Error_IRQSn);
NVIC_EnableIRQ(CAN_Bus_Off_IRQSn);
*/


void CAN0_ORed_Message_buffer_IRQHandler(void)
{
#ifdef MEASURE_CAN
	BITBAND_REG(MEASURE_CAN_GPIO->PDOR, MEASURE_CAN_PIN) = 1;
#endif
	/// If Rx FIFO is enabled
	if(CAN0->MCR & CAN_MCR_RFEN_MASK)
	{

	}
	else
	{
		CAN_DataFrame frame;
		for(int i=0; i<CAN_CS_COUNT; i++)
		{
			if( CAN0->IFLAG1 & (1<<i) )
			{
				if(((CAN0->MB[i].CS&CAN_CS_CODE_MASK)>>CAN_CS_CODE_SHIFT)==RX_FULL)
				{
					CAN_Status s = CAN_ReadRxMB(CAN0,i,&frame);
					callbacks[i](frame,s,callbacksData[i]);
				}
			}
		}
	}
#ifdef MEASURE_CAN
	BITBAND_REG(MEASURE_CAN_GPIO->PDOR, MEASURE_CAN_PIN) = 0;
#endif
}

/*
The interrupt sources for Bus Off, Error, Wake Up, Tx Warning, and Rx Warning
generate interrupts like the MB interrupt sources, and they can be read from ESR1 and
ESR2. The Bus Off, Error, Tx Warning, and Rx Warning interrupt mask bits are located
in CTRL1; the Wake-Up interrupt mask bit is located in MCR.


void CAN0_Bus_Off_IRQHandler(void)
{

}
void CAN0_Error_IRQHandler(void)
{

}
void CAN0_Tx_Warning_IRQHandler(void)
{

}
void CAN0_Rx_Warning_IRQHandler(void)
{

}
void CAN0_Wake_Up_IRQHandler(void)
{

}
*/








































static void CAN_Freeze(CAN_Type * base, bool freeze)
{
	if(freeze)
	{
		base->MCR |= CAN_MCR_HALT_MASK;
		// Wait until module goes into freeze mode (MCR[FRZ_ACK] = 1).
		while((base->MCR & CAN_MCR_FRZACK_MASK)!=CAN_MCR_FRZACK_MASK);
	}
	else
	{
		base->MCR &= ~CAN_MCR_HALT_MASK;
		// Wait until module goes out freeze mode (MCR[FRZ_ACK] = 0).
		while((base->MCR & CAN_MCR_FRZACK_MASK)==CAN_MCR_FRZACK_MASK);
	}
}
