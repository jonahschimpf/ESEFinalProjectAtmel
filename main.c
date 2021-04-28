/*
 * SerialCommunication.c
 *
 * Created: 4/25/2021 9:41:00 PM
 * Author : jonah
 */ 

#define F_CPU 16000000UL
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#define BUF_SIZE 400
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <string.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>

#include "uart/uart.h"
#include "SWSerial/SWseriale.h"
//Function definitions
void getString(void);

//Global variables
char espString[BUF_SIZE];
int validString = 0;
void getString(void) {
	
	int i = 0;
	int startReading = 0;
	//espString[0] = 0; //initialize ESPstring back to an empty
	while (SWseriale_available()){ // Checks if any character has been received
		validString = 1;
		uint8_t temp = SWseriale_read(); // Reads one character from SWseriale received data buffer
		
		char buf[2];
		sprintf(buf, "%c\n", temp);
		UART_putstring(buf);
		if (i < BUF_SIZE) {
			espString[i] = (char) temp;
		} else {
			break;
		}
		i++;
		UART_putstring(espString);
		UART_putstring("\n");
	}
	espString[i] = '\0';
//	UART_putstring(espString);
}


int main(void) {
	sei();
	SWseriale_begin(); // Initialize INT1, Timer2, Pin 3 (Input, Pull-up) and Pin 4 (Output)
	
	//SWseriale_write("HI",2);
	UART_init(BAUD_PRESCALER);

	
	while (1) {
		getString();
		if (validString) {
			UART_putstring("ESPstring is: \n");
			UART_putstring(espString);
			UART_putstring("\n");
			validString = 0;
		}
		_delay_ms(10); // Wait 10 ms, optional
	}
}
