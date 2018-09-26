/**
 * @file I2C.h
 * @date 14 Sep 2018
 * @brief .
 *
 */

// Uncomment this define to raise a pin when in a I2C function
//#define MEASURE_I2C

#ifdef MEASURE_I2C
	#define MEASURE_I2C_PORT PORTC
	#define MEASURE_I2C_GPIO GPIOC
	#define MEASURE_I2C_PIN	0
#endif



typedef enum { I2C_FREQ_13K, I2C_FREQ_39K, I2C_FREQ_48K8, I2C_FREQ_97K6 , I2C_FREQ_195K3 , 
				I2C_FREQ_312K5 , I2C_FREQ_625K , I2C_FREQ_892K , I2C_FREQ_2M5 } I2C_FREQUENCY_T;

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
	I2C_BUS_FAULT,		// Permanent bus fault, to be checked at start of transmission
	I2C_SLAVE_NACK		// Received NACK from slave before transmission was completed, to be checked in callback
} I2C_FAULT_T;

typedef struct
{
	I2C_STATE_T state;
	I2C_FLAG_T 	flag;
	I2C_MODE_T 	mode;
	I2C_FAULT_T fault;
	I2C_FREQUENCY_T freq;
	uint8_t		address_w;
	uint8_t		address_r;
	uint8_t 	address_reg;
	uint8_t *	data;
	uint8_t		dataSize;
	uint8_t		dataIndex;
	void (* callback)(void);
}I2C_CONTROL_T;

void I2C_init(I2C_CONTROL_T * i2cInput);
void I2C_SetDefaultConfig(I2C_CONTROL_T * i2cInput, uint8_t address, I2C_FREQUENCY_T frequency, void (* userCallback) () );
I2C_FAULT_T I2C_WriteData(I2C_CONTROL_T * i2cInput);
I2C_FAULT_T I2C_ReadData(I2C_CONTROL_T * i2cInput);
I2C_FAULT_T I2C_Blocking_WriteData(I2C_CONTROL_T * i2cInput);
I2C_FAULT_T I2C_Blocking_ReadData(I2C_CONTROL_T * i2cInput);
void I2C_isrCallback();

