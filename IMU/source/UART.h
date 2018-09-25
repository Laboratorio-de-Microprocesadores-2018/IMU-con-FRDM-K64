

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "MK64F12.h"

void UARTInit (void);
void UARTSetBaudRate (UART_Type *uart, uint32_t baudrate);

uint8_t UARTSendData( uint8_t * tx_data, uint8_t len);
uint8_t UARTRecieveData( uint8_t * rx_data, uint8_t len);


#endif /* UART_H_ */
