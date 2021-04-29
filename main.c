/*
 * FinalProjectTest.c
 *
 * Created: 16/04/21 12:14:42 AM
 * Author : rcmar & jonah
 */ 

#define F_CPU 16000000UL
#define PERIOD 100
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)
#define TEST 0

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#include "uart/uart.h"
#include "SWSerial/SWseriale.h"
//Global variables
//Jonah's
#define BUF_SIZE 200
char espString[BUF_SIZE];
int validString = 0;
unsigned int timer0_overflows = 0;
int secondPassed = 0;
//Rafa's
char String[200];
int print = 0;


#define ARRAY_SIZE 100
#define BUFFER_SIZE 500

#define DISPLAY_START 30
#define DISPLAY_END 60

short int buffer[BUFFER_SIZE];
int buffer_pointer;

short int display[ARRAY_SIZE];
int ticks_per_rotation = 15000;
int ticks_per_part = 300;

void resetDisplay(void) {
	int i = 0;
	for (i = 0; i < ARRAY_SIZE; i++){
		display[i] = 0x0000;
	}
}

//testing functions

void setJonah(void) {
	resetDisplay();
		//jonah
		
		display[67] = 0x0011;
		display[68] = 0x0011;
		display[69] = 0x001F;
		display[70] = 0x0001;
		display[71] = 0x0001;
		display[74] = 0x001F;
		display[75] = 0x0011;
		display[76] = 0x0011;
		display[77] = 0x0011;
		display[78] = 0x001F;
		display[81] = 0x001F;
		display[82] = 0x0003;
		display[83] = 0x000E;
		display[84] = 0x0018;
		display[85] = 0x001F;
		display[88] = 0x001F;
		display[89] = 0x0005;
		display[90] = 0x0005;
		display[91] = 0x0005;
		display[92] = 0x001F;
		display[95] = 0x001F;
		display[96] = 0x0004;
		display[97] = 0x0004;
		display[98] = 0x0004;
		display[99] = 0x001F;
}
void setMaya(void) {
	resetDisplay();
	
	//maya
	display[67] = 0x001F;
	display[68] = 0x0001;
	display[69] = 0x0006;
	display[70] = 0x0001;
	display[71] = 0x001F;
	display[74] = 0x001F;
	display[75] = 0x0005;
	display[76] = 0x0005;
	display[77] = 0x0005;
	display[78] = 0x001F;
	display[81] = 0x0007;
	display[82] = 0x0004;
	display[83] = 0x001C;
	display[84] = 0x0004;
	display[85] = 0x0007;
	display[88] = 0x001F;
	display[89] = 0x0005;
	display[90] = 0x0005;
	display[91] = 0x0005;
	display[92] = 0x001F;
}

void getString(void) {
	if (!SWseriale_available()) {
		return;
	}
	int j = 0;
	while (SWseriale_available()){ // Checks if any character has been received
		validString = 1;
		uint8_t temp = SWseriale_read(); // Reads one character from SWseriale received data buffer
		if (TEST) {
			char buf[2];
			sprintf(buf, "%c\n", temp);
			UART_putstring(buf);
		}
		if (j < BUF_SIZE) {
			espString[j] = (char) temp;
			} else {
			break;
		}
		j++;
	//	UART_putstring(espString);
	//UART_putstring("continue");
	}
	espString[j] = '\0';
//	UART_putstring(espString);
}




void Initialize()
{
	cli();
	
	int i;
	
	ticks_per_part = ticks_per_rotation/ARRAY_SIZE;
	
		
	//configure outputs
	DDRD = 0x00F7;
	DDRB |= (1<<DDB3);
	
	//configure input
	DDRB &= ~(1<<DDB0);
		
	//set clock prescaling to 1/64
	TCCR1B |= (1<<CS10);
	TCCR1B |= (1<<CS11);
	TCCR1B &= ~(1<<CS12);
	
	//set timer to normal mode
	TCCR1A &= ~(1<<WGM10);
	TCCR1A &= ~(1<<WGM11);
	TCCR1B &= ~(1<<WGM12);
	TCCR1B &= ~(1<<WGM13);
	
	//start looking for a falling edge
	TCCR1B &= ~(1<<ICES1);
	
	//turn on noise canceling
	TCCR1B |= (1<<ICNC1);
	
	//Clear interrupt flag
	TIFR1 |= (1<<ICF1);
	
	//enable interrupts
	TIMSK1 |= (1<<ICIE1);
	

	//for (i = 180; i < 360; i++){
		//display[i] = 0xFFFF;
	//}
	
		for (i = 0; i < ARRAY_SIZE; i++){
			display[i] = 0x0000;
		}
		for (i = 0; i < BUFFER_SIZE; i++){
			buffer[i] = 0x0000;
		}
		//for (i = 180; i < 360; i++){
		//display[i] = 0xFFFF;
		//}
		
		buffer[9] = 0x0011;
		buffer[10] = 0x0011;
		buffer[11] = 0x001F;
		buffer[12] = 0x0001;
		buffer[13] = 0x0001;
		buffer[16] = 0x001F;
		buffer[17] = 0x0011;
		buffer[18] = 0x0011;
		buffer[19] = 0x0011;
		buffer[20] = 0x001F;
		buffer[23] = 0x001F;
		buffer[24] = 0x0003;
		buffer[25] = 0x000E;
		buffer[26] = 0x0018;
		buffer[27] = 0x001F;
		buffer[30] = 0x001F;
		buffer[31] = 0x0005;
		buffer[32] = 0x0005;
		buffer[33] = 0x0005;
		buffer[34] = 0x001F;
		buffer[37] = 0x001F;
		buffer[38] = 0x0004;
		buffer[39] = 0x0004;
		buffer[40] = 0x0004;
		buffer[41] = 0x001F;
		buffer[48] = 0x001F;
		buffer[49] = 0x0005;
		buffer[50] = 0x0005;
		buffer[51] = 0x0005;
		buffer[52] = 0x001F;
		buffer[55] = 0x001F;
		buffer[56] = 0x0003;
		buffer[57] = 0x000E;
		buffer[58] = 0x0018;
		buffer[59] = 0x001F;
		buffer[62] = 0x001F;
		buffer[63] = 0x0011;
		buffer[64] = 0x0011;
		buffer[65] = 0x0011;
		buffer[66] = 0x000E;
		buffer[73] = 0x001F;
		buffer[74] = 0x000D;
		buffer[75] = 0x000D;
		buffer[76] = 0x0015;
		buffer[77] = 0x0017;
		buffer[80] = 0x001F;
		buffer[81] = 0x0005;
		buffer[82] = 0x0005;
		buffer[83] = 0x0005;
		buffer[84] = 0x001F;
		buffer[87] = 0x001F;
		buffer[88] = 0x0005;
		buffer[89] = 0x0005;
		buffer[90] = 0x0001;
		buffer[93] = 0x001F;
		buffer[94] = 0x0005;
		buffer[95] = 0x0005;
		buffer[96] = 0x0005;
		buffer[97] = 0x001F;
		buffer[104] = 0x0A00;
	
	//jonah
	/*
	display[67] = 0x0011;
	display[68] = 0x0011;
	display[69] = 0x001F;
	display[70] = 0x0001;
	display[71] = 0x0001;
	display[74] = 0x001F;
	display[75] = 0x0011;
	display[76] = 0x0011;
	display[77] = 0x0011;
	display[78] = 0x001F;
	display[81] = 0x001F;
	display[82] = 0x0003;
	display[83] = 0x000E;
	display[84] = 0x0018;
	display[85] = 0x001F;
	display[88] = 0x001F;
	display[89] = 0x0005;
	display[90] = 0x0005;
	display[91] = 0x0005;
	display[92] = 0x001F;
	display[95] = 0x001F;
	display[96] = 0x0004;
	display[97] = 0x0004;
	display[98] = 0x0004;
	display[99] = 0x001F;
	//*/
	
	
	//camila
	/*
	display[67] = 0x001F;
	display[68] = 0x0011;
	display[69] = 0x0011;
	display[70] = 0x0011;
	display[71] = 0x0011;
	display[74] = 0x001F;
	display[75] = 0x0005;
	display[76] = 0x0005;
	display[77] = 0x0005;
	display[78] = 0x001F;
	display[81] = 0x001F;
	display[82] = 0x0001;
	display[83] = 0x0006;
	display[84] = 0x0001;
	display[85] = 0x001F;
	display[88] = 0x0011;
	display[89] = 0x0011;
	display[90] = 0x001F;
	display[91] = 0x0011;
	display[92] = 0x0011;
	display[95] = 0x001F;
	display[96] = 0x0010;
	display[97] = 0x0010;
	display[98] = 0x0010;
	display[99] = 0x0010;
	display[2] = 0x001F;
	display[3] = 0x0005;
	display[4] = 0x0005;
	display[5] = 0x0005;
	display[6] = 0x001F;
	//*/
	
	
	//martina
	/*
	display[67] = 0x001F;
	display[68] = 0x0001;
	display[69] = 0x0006;
	display[70] = 0x0001;
	display[71] = 0x001F;
	display[74] = 0x001F;
	display[75] = 0x0005;
	display[76] = 0x0005;
	display[77] = 0x0005;
	display[78] = 0x001F;
	display[81] = 0x001F;
	display[82] = 0x000D;
	display[83] = 0x000D;
	display[84] = 0x0015;
	display[85] = 0x0017;
	display[88] = 0x0001;
	display[89] = 0x0001;
	display[90] = 0x001F;
	display[91] = 0x0001;
	display[92] = 0x0001;
	display[95] = 0x0011;
	display[96] = 0x0011;
	display[97] = 0x001F;
	display[98] = 0x0011;
	display[99] = 0x0011;
	display[2] = 0x001F;
	display[3] = 0x0003;
	display[4] = 0x000E;
	display[5] = 0x0018;
	display[6] = 0x001F;
	display[9] = 0x001F;
	display[10] = 0x0005;
	display[11] = 0x0005;
	display[12] = 0x0005;
	display[13] = 0x001F;
	
	//*/

	//mari
	/*
	display[67] = 0x001F;
	display[68] = 0x0001;
	display[69] = 0x0006;
	display[70] = 0x0001;
	display[71] = 0x001F;
	display[74] = 0x001F;
	display[75] = 0x0005;
	display[76] = 0x0005;
	display[77] = 0x0005;
	display[78] = 0x001F;
	display[81] = 0x001F;
	display[82] = 0x000D;
	display[83] = 0x000D;
	display[84] = 0x0015;
	display[85] = 0x0017;
	display[88] = 0x0011;
	display[89] = 0x0011;
	display[90] = 0x001F;
	display[91] = 0x0011;
	display[92] = 0x0011;
	
	//*/

	//maya
	/*
	display[67] = 0x001F;
	display[68] = 0x0001;
	display[69] = 0x0006;
	display[70] = 0x0001;
	display[71] = 0x001F;
	display[74] = 0x001F;
	display[75] = 0x0005;
	display[76] = 0x0005;
	display[77] = 0x0005;
	display[78] = 0x001F;
	display[81] = 0x0007;
	display[82] = 0x0004;
	display[83] = 0x001C;
	display[84] = 0x0004;
	display[85] = 0x0007;
	display[88] = 0x001F;
	display[89] = 0x0005;
	display[90] = 0x0005;
	display[91] = 0x0005;
	display[92] = 0x001F;
	
	//*/
	
	//clara
	/*
	display[64] = 0x001F;
	display[65] = 0x0011;
	display[66] = 0x0011;
	display[67] = 0x0011;
	display[68] = 0x0011;
	display[71] = 0x001F;
	display[72] = 0x0010;
	display[73] = 0x0010;
	display[74] = 0x0010;
	display[75] = 0x0010;
	display[78] = 0x001F;
	display[79] = 0x0005;
	display[80] = 0x0005;
	display[81] = 0x0005;
	display[82] = 0x001F;
	display[85] = 0x001F;
	display[86] = 0x000D;
	display[87] = 0x000D;
	display[88] = 0x0015;
	display[89] = 0x0017;
	display[92] = 0x001F;
	display[93] = 0x0005;
	display[94] = 0x0005;
	display[95] = 0x0005;
	display[96] = 0x001F;
	//*/
	//Normal mode
	TCCR0A &= ~(1 << WGM00);
	TCCR0A &= ~(1 << WGM01);
	TCCR0B &= ~(1 << WGM02);
	
	//1024 Prescale T0
	TCCR0B |= (1 << CS02);
	TCCR0B &= ~(1 << CS01);
	TCCR0B |= (1 << CS00);
	
	//Enable interrupt
	TIMSK0 |= (1 << TOIE0);
	sei();
}

void moveText()
{
	int i;
	
	for (i = DISPLAY_START; i < DISPLAY_END; i++){
		display[i] = display[i+1];
	}
	
	buffer_pointer++;
	
	if((buffer[buffer_pointer] == 0x0A00) | (buffer_pointer == BUFFER_SIZE) ){
		buffer_pointer = 0;
	}
	
	display[DISPLAY_END - 1] = buffer[buffer_pointer];
	
}

void initBuffer()
{
	int i;
	int j = 0;
	
	for (i = DISPLAY_START; i < DISPLAY_END; i++){
		
		if(buffer[j] == 0x0A00){
			j = 0;
		}
		
		display[i] = buffer[j];
		
		j++;
	}
	
	buffer_pointer = j - 1;
}

ISR(TIMER0_OVF_vect) {
	timer0_overflows++;
	if (timer0_overflows == 7) { //one second has passed
		secondPassed = 1;
	}
}

int main(void)
{
	
	Initialize();
	SWseriale_begin();
//	UART_init(BAUD_PRESCALER);
	sei();
    /* Replace with your application code */
	DDRB |= (1 << DDB5);
	
	initBuffer();
	
    while (1) 
    {
	//	_delay_ms(500); SOLVES IT
	if (secondPassed) {
		getString();	
		moveText();
		secondPassed = 0;
		timer0_overflows = 0;
	}
	//	UART_putstring(espString);
		int place = ((TCNT1/ticks_per_part) % ARRAY_SIZE);
		
		PORTD = (display[place] & ~(1<<3)) | (PORTD & ((1<<3)|(1<<7)));
		PORTB = (PORTB & ~(1<<3)) | (display[place] & (1<<3));
	//	UART_putstring(espString);
	//	_delay_ms(1000);
		//UART_putstring("\n");
		
//		int x = strcmp(espString, "Hello");
		char buf[10];
	//	sprintf(buf, "%d\n", x);
		//UART_putstring(buf);
	//	x = strlen(espString);
		//sprintf(buf, "%d\n", x);
	//	UART_putstring(x);
		if (strcmp(espString, "Hello") == 0) {
		//	UART_putstring("hi\n");
			//setMaya();
			PORTB|=(1<<PORTB5);
		} else {
		//	UART_putstring("bye\n");
			//setJonah();
			PORTB &= ~(1<<PORTB5);
		} 
    }
}

ISR(TIMER1_CAPT_vect)
{
	ticks_per_rotation = ICR1;
	TCNT1 = 0;
	ticks_per_part = ticks_per_rotation/ARRAY_SIZE;

}
