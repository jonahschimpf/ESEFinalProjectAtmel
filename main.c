/*
 * FinalProjectTest.c
 *
 * Created: 16/04/21 12:14:42 AM
 * Author : rcmar
 */ 

#define F_CPU 16000000UL
#define PERIOD 100
#define ARRAY_SIZE 100
#define BAUD_RATE 9600
#define BAUD_PRESCALER (((F_CPU / (BAUD_RATE * 16UL))) - 1)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include "uart.h"

char String[200];
int print = 0;

short int display[ARRAY_SIZE];
int ticks_per_rotation = 15000;
int ticks_per_part = 300;

void Initialize()
{
	cli();
	
	int i;
	
	ticks_per_part = ticks_per_rotation/ARRAY_SIZE;
	
	//set up UART
	//UART_init(BAUD_PRESCALER);
	
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
	
	for (i = 0; i < ARRAY_SIZE; i++){
		display[i] = 0x0000; 
	}
	//for (i = 180; i < 360; i++){
		//display[i] = 0xFFFF;
	//}
	
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
	
	
	sei();
}

int main(void)
{
	
	Initialize();
	
	
    /* Replace with your application code */
    while (1) 
    {
		
		//PORTD = display[((TCNT1/ticks_per_part) % ARRAY_SIZE)];
		int place = ((TCNT1/ticks_per_part) % ARRAY_SIZE);
			
		PORTD = (display[place] & ~(1<<3)) | (PORTD & (1<<3));
		PORTB = (PORTB & ~(1<<3)) | (display[place] & (1<<3));
		
    }
}

//Hall Sensor Interrupt 

ISR(TIMER1_CAPT_vect)
{
	ticks_per_rotation = ICR1;
	TCNT1 = 0;
	ticks_per_part = ticks_per_rotation/ARRAY_SIZE;
	
} 

