/**
 * @file I2C.h
 * @date 14 Sep 2018
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 * @see http://www.stack.nl/~dimitri/doxygen/docblocks.html
 * @see http://www.stack.nl/~dimitri/doxygen/commands.html
 */


typedef enum
{
	I2C_STATE_NONE = 0,
	I2C_STATE_WRITE_DATA,
	I2C_STATE_WRITE_REG_ADDRESS,
	I2C_STATE_RSTART,
	I2C_STATE_READ_DUMMY,
	I2C_STATE_READ_DATA
} I2C_STATE_T;

typedef enum
{
	I2C_MODE_READ = 0,
	I2C_MODE_WRITE
} I2C_MODE_T;

typedef enum
{
	I2C_FLAG_NONE = 0,
	I2C_FLAG_TRANSMISSION_PROGRESS
} I2C_FLAG_T;

typedef enum
{
	I2C_NO_FAULT = 0,
	I2C_BUS_BUSY,
	I2C_TIMEOUT,
	I2C_BUS_FAULT,
	I2C_SLAVE_NACK	// the slave did NACK befor transmission was completed
} I2C_FAULT_T;

typedef struct
{
	I2C_STATE_T state;
	I2C_FLAG_T 	flag;
	I2C_MODE_T 	mode;
	I2C_FAULT_T fault;
	uint8_t		address_w;
	uint8_t		address_r;
	uint8_t 	address_reg;
	uint8_t *	data;
	uint8_t		dataSize;
	uint8_t		dataIndex;
	void (* callback)(void);
}I2C_CONTROL_T;

void I2C_init();
void I2C_SetDefaultConfig(I2C_CONTROL_T * i2cInput, uint8_t address, void (* userCallback) () );
I2C_FAULT_T I2C_WriteData(I2C_CONTROL_T * i2cInput);
I2C_FAULT_T I2C_ReadData(I2C_CONTROL_T * i2cInput);
I2C_FAULT_T I2C_Blocking_WriteData(I2C_CONTROL_T * i2cInput);
I2C_FAULT_T I2C_Blocking_ReadData(I2C_CONTROL_T * i2cInput);
void I2C_isrCallback();

