/*
 * uart.h
 *
 * Created: 2/26/2021 1:20:34 PM
 *  Author: jonah
 */ 
#ifndef UART_H
#define UART_H

void UART_init(int prescale);

void UART_send( unsigned char data);

void UART_putstring(char* StringPtr);

#endif