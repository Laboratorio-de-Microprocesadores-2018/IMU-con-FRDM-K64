

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "MK64F12.h"

void UARTInit (void);
void UARTSetBaudRate (UART_Type *uart, uint32_t baudrate);

void UARTSendData(unsigned char txdata);
unsigned char UARTRecieveData(void);


#endif /* UART_H_ */
