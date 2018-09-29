/////////////////////////////////////////////////////////////////////////////////
//                    	   TP2 -  Comunicaciones Serie                         //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file I2C.h
 * @author Marcos Brito
 * @date 14 Sep 2018
 *
 */

// Uncomment this define to raise a pin when in a I2C function
#define MEASURE_I2C

#ifdef MEASURE_I2C
	#define MEASURE_I2C_PORT PORTC
	#define MEASURE_I2C_GPIO GPIOC
	#define MEASURE_I2C_PIN	0
#endif


/**
 * @brief Enum with the frequencies available for the I2C master clock.
 */
typedef enum { I2C_FREQ_13K, I2C_FREQ_39K, I2C_FREQ_48K8, I2C_FREQ_97K6 , I2C_FREQ_195K3 , 
				I2C_FREQ_312K5 , I2C_FREQ_625K , I2C_FREQ_892K , I2C_FREQ_2M5 } I2C_FREQUENCY_T;

typedef enum {
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

typedef enum {
	I2C_NO_FAULT = 0,
	I2C_BUS_BUSY,		// I2C bus is busy. To be checked when a write or read sequence is called.
	I2C_SLAVE_ABSENT,	// Slave did not respond ACK after address was sent. To be checked during I2C funtion callback.
	I2C_TRANSMISSION_INCOMPLETE	// Slave responded NACK unexpectedly. To be checked during I2C function callback.
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

/**
 * @brief Initialize I2C driver with parameters set in i2cInput structure
 * @param i2cInput Structure of type I2C_CONTROL_T with the parameters to set the driver functionality
 * */
void I2C_init(I2C_CONTROL_T * i2cInput);

/**
 * @brief Configures the I2C_CONTROL_T structure with the data given
 * @param i2cInput Structure of type I2C_CONTROL_T to set the driver functionality parameters
 * @param address Address of the I2C slave
 * @param frequency Sets the frequency of the I2C communication. Determines the frequency of the master clock.
 * @param userCallback Callback function to be called when error has occured during transmission or if transmission is completed.
 * */
void I2C_SetDefaultConfig(I2C_CONTROL_T * i2cInput, uint8_t address, I2C_FREQUENCY_T frequency, void (* userCallback) () );

/**
 * @brief Function to be called to start a non-blocking write sequence. 
 * @param i2cInput Structure must have the address_reg, data size and data pointer set before beggining transmission.
 * @return Returns I2C_BUS_BUSY if the bus is busy. I2C_NO_FAULT if transmission began correctly.
 * */
I2C_FAULT_T I2C_WriteData(I2C_CONTROL_T * i2cInput);

/**
 * @brief Function to be called to start a non-blocking read sequence. 
 * @param i2cInput Structure must have the address_reg, data size and data pointer set before beggining transmission.
 * @return Returns I2C_BUS_BUSY if the bus is busy. I2C_NO_FAULT if transmission began correctly.
 * */
I2C_FAULT_T I2C_ReadData(I2C_CONTROL_T * i2cInput);

/**
 * @brief Function to be called to start a blocking write sequence. Funtion will exit only when transmission is finished or if bus busy timeout. 
 * @param i2cInput Structure must have the address_reg, data size and data pointer set before beggining transmission.
 * @return Returns I2C_BUS_BUSY if the bus is busy. I2C_NO_FAULT if transmission began correctly.
 * */
I2C_FAULT_T I2C_Blocking_WriteData(I2C_CONTROL_T * i2cInput);

/**
 * @brief Function to be called to start a blocking read sequence. Funtion will exit only when transmission is finished or if bus busy timeout. 
 * @param i2cInput Structure must have the address_reg, data size and data pointer set before beggining transmission.
 * @return Returns I2C_BUS_BUSY if the bus is busy. I2C_NO_FAULT if transmission began correctly.
 * */
I2C_FAULT_T I2C_Blocking_ReadData(I2C_CONTROL_T * i2cInput);

void I2C_isrCallback();

