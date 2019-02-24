#ifndef UART_H
#define UART_H

#include <avr/io.h>
#include <stdint.h>

// ********************************************************************************
// Function Prototypes
// ********************************************************************************
void UART_init(void);
void UART_putChar(uint16_t c);
uint8_t UART_getChar(void);
void UART_putString(uint16_t* buf);
uint8_t UART_getString(uint16_t* buf);

#endif