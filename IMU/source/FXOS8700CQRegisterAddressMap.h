#ifndef FXOS8700CQRegisterAddressMap_H_
#define FXOS8700CQRegisterAddressMap_H_


/*---------------------------Register Address Map from FXOS8700CQ datasheeet(Table 16)---------------------------*/
/*		Name			Address	  	Access	Def.Val.					Comment									 */

#define STATUS 		        0x00   // R 	0x00 	Real-time data-ready status or FIFO status (DR_STATUS or F_STATUS)
#define OUT_X_MSB 			0x01   // R 	Data 	[7:0] are 8 MSBs of 14-bit sample. Root pointer to XYZ FIFO data.
#define OUT_X_LSB 			0x02   // R 	Data 	[7:2] are 6 LSBs of 14-bit real-time sample
#define OUT_Y_MSB 			0x03   // R 	Data 	[7:0] are 8 MSBs of 14-bit real-time sample
#define OUT_Y_LSB 			0x04   // R 	Data 	[7:2] are 6 LSBs of 14-bit real-time sample
#define OUT_Z_MSB 			0x05   // R 	Data 	[7:0] are 8 MSBs of 14-bit real-time sample
#define OUT_Z_LSB 			0x06   // R 	Data 	[7:2] are 6 LSBs of 14-bit real-time sample
					/*0x07 to 0x08 Reserved DO NOT MODIFY*/
#define F_SETUP 			0x09   // R/W 	0x00 	FIFO setup
#define TRIG_CFG 			0x0A   // R/W 	0x00 	FIFO event trigger configuration register
#define SYSMOD 			    0x0B   // R 	0x00 	Current system mode
#define INT_SOURCE 		    0x0C   // R 	0x00 	Interrupt status
#define WHO_AM_I 			0x0D   // R 	0xC7 	Device ID
#define XYZ_DATA_CFG 		0x0E   // R/W 	0x00 	Acceleration dynamic range and filter enable settings
#define HP_FILTER_CUTOFF 	0x0F   // R/W 	0x00 	Pulse detection high-pass and low-pass filter enable bits. High-pass filter cutoff frequency selection
#define PL_STATUS           0x10   // R 	0x00 	Landscape/portrait orientation status
#define PL_CFG           0x11   // R/W 	0x80 	Landscape/portrait configuration
#define PL_COUNT 			0x12   // R/W 	0x00 	Landscape/portrait debounce counter
#define PL_BF_ZCOMP 		0x13   // R/W 	0x84 	Back/front trip angle threshold
#define PL_THS_REG 		    0x14   // R/W 	0x44 	Portrait to landscape trip threshold angle and hysteresis settings
#define A_FFMT_CFG 		    0x15   // R/W 	0x00 	Freefall/motion function configuration
#define A_FFMT_SRC 		    0x16   // R 	0x00 	Freefall/motion event source register
#define A_FFMT_THS 		    0x17   // R/W 	0x00 	Freefall/motion threshold register
#define A_FFMT_COUNT 		0x18   // R/W 	0x00 	Freefall/motion debounce counter
					/*0x19 to 0x1C Reserved DO NOT MODIFY*/
#define TRANSIENT_CFG 		0x1D   // R/W 	0x00 	FIFO setup
#define TRANSIENT_SRC 		0x1E   // R 	0x00 	Transient event status register
#define TRANSIENT_THS 		0x1F   // R/W 	0x00 	Transient event threshold
#define TRANSIENT_COUNT 	0x20   // R/W 	0x00 	Transient debounce counter
#define PULSE_CFG 			0x21   // R/W 	0x00 	Pulse function configuration
#define PULSE_SRC 			0x22   // R 	0x00 	Pulse function source register
#define PULSE_THSX 		    0x23   // R/W 	0x00 	X-axis pulse threshold
#define PULSE_THSY 		    0x24   // R/W 	0x00 	Y-axis pulse threshold
#define PULSE_THSZ 		    0x25   // R/W 	0x00 	Z-axis pulse threshold
#define PULSE_TMLT 		    0x26   // R/W 	0x00 	Time limit for pulse detection
#define PULSE_LTCY 		    0x27   // R/W 	0x00 	Latency time for second pulse detection
#define PULSE_WIND 		    0x28   // R/W 	0x00 	Window time for second pulse detection
#define ASLP_COUNT 		    0x29   // R/W 	0x00 	In activity counter setting for auto-sleep
#define CTRL_REG1 			0x2A   // R/W 	0x00 	System ODR, accelerometer OSR, operating mode
#define CTRL_REG2 			0x2B   // R/W 	0x00 	Self-test, reset, accelerometer OSR and sleep mode settings
#define CTRL_REG3 			0x2C   // R/W 	0x00 	Sleep mode interrupt wake enable, interrupt polarity, push-pull/open-drain configuration
#define CTRL_REG4 			0x2D   // R/W 	0x00 	Interrupt enable register
#define CTRL_REG5 			0x2E   // R/W 	0x00 	Interrupt pin (INT1/INT2) map
#define OFF_X 				0x2F   // R/W 	0x00 	X-axis accelerometer offset adjust
#define OFF_Y 				0x30   // R/W 	0x00 	Y-axis accelerometer offset adjust
#define OFF_Z 				0x31   // R/W 	0x00 	Z-axis accelerometer offset adjust
#define M_DR_STATUS 		0x32   // R 	0x00 	Magnetic data ready
#define M_OUT_X_MSB 		0x33   // R 	Data 	MSB of 16-bit magnetic data for X-axis
#define M_OUT_X_LSB 		0x34   // R 	Data 	LSB of 16-bit magnetic data for X-axis
#define M_OUT_Y_MSB 		0x35   // R 	Data 	MSB of 16-bit magnetic data for Y-axis
#define M_OUT_Y_LSB 		0x36   // R 	Data 	LSB of 16-bit magnetic data for Y-axis
#define M_OUT_Z_MSB 		0x37   // R 	Data 	MSB of 16-bit magnetic data for Z-axis
#define M_OUT_Z_LSB 		0x38   // R 	Data 	LSB of 16-bit magnetic data for Z-axis
#define CMP_X_MSB 			0x39   // R 	Data 	Bits [13:8] of integrated X-axis accerleration data
#define CMP_X_LSB 			0x3A   // R 	Data 	Bits [7:0] of integrated X-axis accerleration data
#define CMP_Y_MSB 			0x3B   // R 	Data 	Bits [13:8] of integrated Y-axis accerleration data
#define CMP_Y_LSB 			0x3C   // R 	Data 	Bits [7:0] of integrated Y-axis accerleration data
#define CMP_Z_MSB 			0x3D   // R 	Data 	Bits [13:8] of integrated Z-axis accerleration data
#define CMP_Z_LSB 			0x3E   // R 	Data 	Bits [7:0] of integrated Z-axis accerleration data
#define M_OFF_X_MSB 		0x3F   // R/W 	0x00 	MSB of magnetometer of X-axis offset
#define M_OFF_X_LSB 		0x40   // R/W 	0x00 	LSB of magnetometer of X-axis offset
#define M_OFF_Y_MSB 		0x41   // R/W 	0x00 	MSB of magnetometer of Y-axis offset
#define M_OFF_Y_LSB 		0x42   // R/W 	0x00 	LSB of magnetometer of Y-axis offset
#define M_OFF_Z_MSB 		0x43   // R/W 	0x00 	MSB of magnetometer of Z-axis offset
#define M_OFF_Z_LSB 		0x44   // R/W 	0x00 	LSB of magnetometer of Z-axis offset
#define MAX_X_MSB 			0x45   // R 	Data 	Magnetometer X-axis maximum value MSB
#define MAX_X_LSB 			0x46   // R 	Data 	Magnetometer X-axis maximum value LSB
#define MAX_Y_MSB 			0x47   // R 	Data 	Magnetometer Y-axis maximum value MSB
#define MAX_Y_LSB 			0x48   // R 	Data 	Magnetometer Y-axis maximum value LSB
#define MAX_Z_MSB 			0x49   // R 	Data 	Magnetometer Z-axis maximum value MSB
#define MAX_Z_LSB 			0x4A   // R 	Data 	Magnetometer Z-axis maximum value LSB
#define MIN_X_MSB 			0x4B   // R 	Data 	Magnetometer X-axis minimum value MSB
#define MIN_X_LSB 			0x4C   // R 	Data 	Magnetometer X-axis minimum value LSB
#define MIN_Y_MSB 			0x4D   // R 	Data 	Magnetometer Y-axis minimum value MSB
#define MIN_Y_LSB 			0x4E   // R 	Data 	Magnetometer Y-axis minimum value LSB
#define MIN_Z_MSB 			0x4F   // R 	Data 	Magnetometer Z-axis minimum value MSB
#define MIN_Z_LSB 			0x50   // R 	Data 	Magnetometer Z-axis minimum value LSB
#define TEMP 				0x51   // R 	Data 	Device temperature, valid range of –128 to 127 °C when M_CTRL1[m_hms] > 0b00
#define M_THS_CFG 			0x52   // R/W	0x00 	Magnetic threshold detection function configuration
#define M_THS_SRC 			0x53   // R 	Data	Magnetic threshold event source register
#define M_THS_X_MSB 		0x54   // R/W	0x00 	X-axis magnetic threshold MSB
#define M_THS_X_LSB 		0x55   // R/W	0x00 	X-axis magnetic threshold LSB
#define M_THS_Y_MSB 		0x56   // R/W	0x00 	Y-axis magnetic threshold MSB
#define M_THS_Y_LSB 		0x57   // R/W	0x00 	Y-axis magnetic threshold LSB
#define M_THS_Z_MSB 		0x58   // R/W	0x00 	Z-axis magnetic threshold MSB
#define M_THS_Z_LSB 		0x59   // R/W	0x00 	Z-axis magnetic threshold LSB
#define M_THS_COUNT 		0x5A   // R/W	0x00 	Magnetic threshold debounce counter
#define M_CTRL_REG1 		0x5B   // R/W	0x00 	Control for magnetic sensor functions
#define M_CTRL_REG2 		0x5C   // R/W	0x00 	Control for magnetic sensor functions
#define M_CTRL_REG3 		0x5D   // R/W	0x00 	Control for magnetic sensor functions
#define M_INT_SRC 			0x5E   // R 	0x00 	Magnetic interrupt source
#define A_VECM_CFG 		    0x5F   // R/W 	0x00 	Acceleration vector-magnitude configuration register
#define A_VECM_THS_MSB 	    0x60   // R/W 	0x00 	Acceleration vector-magnitude threshold MSB
#define A_VECM_THS_LSB 	    0x61   // R/W 	0x00 	Acceleration vector-magnitude threshold LSB
#define A_VECM_CNT 		    0x62   // R/W 	0x00 	Acceleration vector-magnitude debounce count
#define A_VECM_INITX_MSB 	0x63   // R/W 	0x00 	Acceleration vector-magnitude X-axis reference value MSB
#define A_VECM_INITX_LSB 	0x64   // R/W 	0x00 	Acceleration vector-magnitude X-axis reference value LSB
#define A_VECM_INITY_MSB 	0x65   // R/W 	0x00 	Acceleration vector-magnitude Y-axis reference value MSB
#define A_VECM_INITY_LSB 	0x66   // R/W 	0x00 	Acceleration vector-magnitude Y-axis reference value LSB
#define A_VECM_INITZ_MSB 	0x67   // R/W 	0x00 	Acceleration vector-magnitude Z-axis reference value MSB
#define A_VECM_INITZ_LSB 	0x68   // R/W 	0x00 	Acceleration vector-magnitude Z-axis reference value LSB
#define M_VECM_CFG 		    0x69   // R/W 	0x00 	Magnetic vector-magnitude configuration register
#define M_VECM_THS_MSB 	    0x6A   // R/W 	0x00 	Magnetic vector-magnitude threshold MSB
#define M_VECM_THS_LSB 	    0x6B   // R/W 	0x00 	Magnetic vector-magnitude threshold LSB
#define M_VECM_CNT 		    0x6C   // R/W 	0x00 	Magnetic vector-magnitude debounce count
#define M_VECM_INITX_MSB 	0x6D   // R/W 	0x00 	Magnetic vector-magnitude reference value X-axis MSB
#define M_VECM_INITX_LSB 	0x6E   // R/W 	0x00 	Magnetic vector-magnitude reference value X-axis LSB
#define M_VECM_INITY_MSB 	0x6F   // R/W 	0x00 	Magnetic vector-magnitude reference value Y-axis MSB
#define M_VECM_INITY_LSB 	0x70   // R/W 	0x00 	Magnetic vector-magnitude reference value Y-axis LSB
#define M_VECM_INITZ_MSB 	0x71   // R/W 	0x00 	Magnetic vector-magnitude reference value Z-axis MSB
#define M_VECM_INITZ_LSB 	0x72   // R/W 	0x00 	Magnetic vector-magnitude reference value Z-axis LSB
#define A_FFMT_THS_X_MSB 	0x73   // R/W 	0x00 	X-axis FMT threshold MSB
#define A_FFMT_THS_X_LSB 	0x74   // R/W 	0x00 	X-axis FFMT threshold LSB
#define A_FFMT_THS_Y_MSB 	0x75   // R/W 	0x00 	Y-axis FFMT threshold MSB
#define A_FFMT_THS_Y_LSB 	0x76   // R/W 	0x00 	Y-axis FFMT threshold LSB
#define A_FFMT_THS_Z_MSB 	0x77   // R/W 	0x00 	Z-axis FFMT threshold MSB
#define A_FFMT_THS_Z_LSB 	0x78   // R/W 	0x00 	Z-axis FFMT threshold LSB
				/*0x79 Reserved DO NOT MODIFY*/




#endif /* FXOS8700CQRegisterAddressMap_H_*/
