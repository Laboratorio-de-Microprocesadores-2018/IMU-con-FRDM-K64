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
	I2C_STATE_WRITE_DEV_ADDRESS_W,
	I2C_STATE_WRITE_DEV_ADDRESS_R,
	I2C_STATE_WRITE_REG_ADDRESS,
	I2C_STATE_READ_DUMMY_DATA,
	I2C_STATE_READ_DATA,
	I2C_STATE_NACK
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
	I2C_PERMANENT_BUS_FAULT
} I2C_FAULT_t;
