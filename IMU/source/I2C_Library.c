#include "MK64F12.h"
#include "I2C_Library.h"
#include <stdbool.h>

#define I2C_CHECK_BUSY 			((I2C0->S & I2C_S_BUSY_MASK) != 0)		// 0 IF BUS IS IDLE
#define I2C_CHECK_RX_ACK 		((I2C0->S & I2C_S_RXAK_MASK) == 0)		// 0 IF RECEIVED ACK
#define I2C_DATA 				(I2C0->D)
#define I2C_SET_TX_MODE			(I2C0->C1 |= 1<<I2C_C1_TX_SHIFT)
#define I2C_SET_RX_MODE 		(I2C0->C1 &= 0<<I2C_C1_TX_SHIFT)
#define I2C_NEXT_SEND_NACK		(I2C0->C1 |= 1<<I2C_C1_TXAK_SHIFT)
#define I2C_NEXT_SEND_ACK		(I2C0->C1 &= 0<<I2C_C1_TXAK_SHIFT)
#define I2C_SEND_START 			(I2C0->C1 |= 1<<I2C_C1_MST_SHIFT)
#define I2C_SEND_STOP			(I2C0->C1 &= 0<<I2C_C1_MST_SHIFT)
#define I2C_SEND_RSTART			(I2C0->C1 |= 1<<I2C_C1_RSTA_SHIFT)
#define I2C_CLEAR_IICIF			(I2C0->S |= 1<<I2C_S_IICIF_SHIFT)
#define I2C_GET_IICIF			((I2C0->S & I2C_S_IICIF_MASK) != 0)

#define I2C_TIMEOUT_COUNT 100000				// Number of "while" cycles to wait if trying to send data and the bus is busy
// I2C0_SCL_PIN	PE24
// I2C0_SDA_PIN	PE25

/**
 * @brief Used for initiation of the transmission
 */
static I2C_FAULT_T I2C_Write();

/**
 * @brief Used for initiation of the transmission
 */
static I2C_FAULT_T I2C_Read();

// This function executes the blocking I2C communication.
// Receives 1 if read_blocking or 0 if write_blocking
I2C_FAULT_T I2C_Block_RnW(I2C_CONTROL_T * i2cInput, bool readNwrite);

static I2C_CONTROL_T * i2cControl;

void I2C_init()
{
	// Clock gating to the I2C0 module
	SIM->SCGC4 |= SIM_SCGC4_I2C0(1);
	// Clock frequency divider
	uint8_t multiplier = 1; // multiplier = x2
	uint8_t SCL_div = 0x2B;
	I2C0->F = ((multiplier << 6) | SCL_div);

	// Enable NVIC interrupts
	__NVIC_EnableIRQ(I2C0_IRQn);
	// Write I2C0 Control Register 1
	I2C0->C1 = 0x00;
	I2C0->C1 |= (	I2C_C1_IICEN_MASK |		// enable I2C
						I2C_C1_IICIE_MASK );		// enable I2C interrupts

 	I2C_NEXT_SEND_ACK;		// Hace falta?
 	PORTE->PCR[24] |= (1<<PORT_PCR_ODE_SHIFT);
 	PORTE->PCR[25] |= (1<<PORT_PCR_ODE_SHIFT);
}

void I2C_SetDefaultConfig(I2C_CONTROL_T * i2cInput, uint8_t address, void (* userCallback) () )
{
	i2cInput->address_w = (address<<1) | 1;
	i2cInput->address_r = (address<<1) & 0;
	i2cInput->state = I2C_STATE_NONE;
	i2cInput->flag = I2C_FLAG_NONE;
	i2cInput->mode = I2C_MODE_READ;
	i2cInput->fault = I2C_NO_FAULT;
	i2cInput->dataIndex = 0;
	i2cInput->callback = userCallback;
}

static I2C_FAULT_T I2C_Write()
{
	I2C_FAULT_T retVal = I2C_NO_FAULT;
	//Check if the bus is not busy
	if(I2C_CHECK_BUSY)
		retVal = I2C_BUS_BUSY;
	else
	{
		i2cControl->mode = I2C_MODE_WRITE;	// Set write mode in control structure
		i2cControl->dataIndex = 0;
		i2cControl->flag = I2C_FLAG_TRANSMISSION_PROGRESS;	// Set transmission in progress flag
		i2cControl->state = I2C_STATE_WRITE_DATA;	// Set the next state
		i2cControl->fault = I2C_NO_FAULT;
		I2C_SET_TX_MODE;		// Set peripheral mode to master
		I2C_SEND_START;		// Write start signal on the data bus. From now on assume im the master.
		I2C_DATA = i2cControl->address_w;		// Send the desired address to the bus + write bit
	}
	return retVal;
}

I2C_FAULT_T I2C_WriteData(I2C_CONTROL_T * i2cInput)
{
	i2cControl = i2cInput;

	I2C_FAULT_T retVal = I2C_NO_FAULT;
	unsigned busTimeoutCount = I2C_TIMEOUT_COUNT;
	while( (I2C_Write() == I2C_BUS_BUSY) && busTimeoutCount > 0)	{ busTimeoutCount--; }
	if(busTimeoutCount == 0)
		retVal = I2C_BUS_FAULT;

	return retVal;
}

I2C_FAULT_T I2C_Blocking_WriteData(I2C_CONTROL_T * i2cInput)
{
	I2C_FAULT_T retVal = I2C_Block_RnW(i2cInput, 0);
	return retVal;
}

static I2C_FAULT_T I2C_Read()
{
	I2C_FAULT_T retVal = I2C_NO_FAULT;
	//Check if the bus is not busy
	if(I2C_CHECK_BUSY)
		retVal = I2C_BUS_BUSY;
	else
	{
		i2cControl->mode = I2C_MODE_READ;	// Set read mode in control structure
		i2cControl->dataIndex = 0;
		i2cControl->flag = I2C_FLAG_TRANSMISSION_PROGRESS;	// Set transmission in progress flag
		i2cControl->state = I2C_STATE_WRITE_REG_ADDRESS;	// Set the next state
		i2cControl->fault = I2C_NO_FAULT;
		I2C_SET_TX_MODE;		// Set peripheral mode to master
		I2C_SEND_START;		// Write start signal on the data bus
		I2C_DATA = i2cControl->address_w;		// Send the desired address to the bus + write bit
	}
	return retVal;
}

I2C_FAULT_T I2C_ReadData(I2C_CONTROL_T * i2cInput)
{
	i2cControl = i2cInput;

	I2C_FAULT_T retVal = I2C_NO_FAULT;
	unsigned busTimeoutCount = I2C_TIMEOUT_COUNT;
	while( (I2C_Read() == I2C_BUS_BUSY) && busTimeoutCount > 0)	{ busTimeoutCount--; }
	if(busTimeoutCount == 0)
		retVal = I2C_BUS_FAULT;

	return retVal;
}

I2C_FAULT_T I2C_Blocking_ReadData(I2C_CONTROL_T * i2cInput)
{
	I2C_FAULT_T retVal = I2C_Block_RnW(i2cInput, 1);
	return retVal;
}

I2C_FAULT_T I2C_Block_RnW(I2C_CONTROL_T * i2cInput, bool readNwrite)
{
	i2cControl = i2cInput;

	I2C_FAULT_T retVal = I2C_NO_FAULT;
	unsigned busTimeoutCount = I2C_TIMEOUT_COUNT;

	if(readNwrite == 0)
		while( (I2C_Write() == I2C_BUS_BUSY) && busTimeoutCount > 0)	{ busTimeoutCount--; }
	else
		while( (I2C_Read() == I2C_BUS_BUSY) && busTimeoutCount > 0)	{ busTimeoutCount--; }

	if(busTimeoutCount == 0)
		retVal = I2C_BUS_FAULT;
	else
	{
		while(i2cControl->flag == I2C_FLAG_TRANSMISSION_PROGRESS && i2cControl->fault == I2C_NO_FAULT)
		{
			while(I2C0->S & I2C_S_IICIF_MASK == 0);
			I2C_isrCallback();
		}
		while(I2C_CHECK_BUSY);		// wait until the bus is free again before exit
		retVal = i2cControl->fault;
	}

	return retVal;
}

void I2C_isrCallback()
{
	I2C_CLEAR_IICIF;
	switch(i2cControl->mode)
	{
		case I2C_MODE_WRITE:
		{
			if(i2cControl->dataIndex == i2cControl->dataSize)	// if this is ack after last byte transmitted
			{
				I2C_SEND_STOP;
				i2cControl->state = I2C_STATE_NONE;
				i2cControl->flag = I2C_FLAG_NONE;
				i2cControl->callback();
			}
			else
			{
				if(I2C_CHECK_RX_ACK)	// Received an ACK
				{
					I2C_DATA = i2cControl->data[i2cControl->dataIndex++];	// write next byte
				}
				else	// Slave did not respond (NACK)
				{
					I2C_SEND_STOP;
					i2cControl->state = I2C_STATE_NONE;
					i2cControl->flag = I2C_FLAG_NONE;
					i2cControl->fault = I2C_SLAVE_NACK;
					i2cControl->callback();
				}
			}
			break;
		}
		case I2C_MODE_READ:
		{
			switch(i2cControl->state)
			{
				case I2C_STATE_WRITE_REG_ADDRESS:
				{
					// Write register address and switch to receive mode
					I2C_DATA = i2cControl->address_reg;
					i2cControl->state = I2C_STATE_RSTART;
					break;
				}
				case I2C_STATE_RSTART:
				{
					I2C_SEND_RSTART;
					I2C_DATA = i2cControl->address_r;
					i2cControl->state = I2C_STATE_READ_DUMMY;
					break;
				}
				case I2C_STATE_READ_DUMMY:
				{
					// read dummy data and prepare for incoming data
					I2C_SET_RX_MODE;
					I2C_NEXT_SEND_ACK;
					uint8_t dummy = I2C_DATA;
					i2cControl->state = I2C_STATE_READ_DATA;
					break;
				}
				case I2C_STATE_READ_DATA:
				{
					if(i2cControl->dataIndex == i2cControl->dataSize-1)	// if its last byte to be read
					{
						I2C_SEND_STOP;
						i2cControl->state = I2C_STATE_NONE;
						i2cControl->flag = I2C_FLAG_NONE;
						i2cControl->callback();
					}
					else if(i2cControl->dataIndex == i2cControl->dataSize-2)	// if its second to last
						I2C_NEXT_SEND_NACK;												// then on next byte send NACK

					i2cControl->data[i2cControl->dataIndex++] = I2C_DATA;

					break;
				}
			}
			break;
		}
	}
}


void I2C0_IRQHandler(void)
{
	// Must clear NVIC flag??
	I2C_isrCallback();
}
