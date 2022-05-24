/*
 * milliseconds.c
 *
 * Created: 05 May 09:36:33
 *  Author: Robert
 */ 

#include "milliseconds.h"

//global variable
volatile uint32_t milliseconds = 0;

//set up timer one to generate an output compare interrupt on channel A every 1 millisecond
void milliseconds_init(void)
{
	TCCR5A = 0; //no pin outputs required
	TCCR5B = (1<<WGM52); //CTC mode
	TCNT5 = 0; //reset timer
	OCR5A = 15999; //generate an interrupt once per millisecond
	TIMSK5 |= (1<<OCIE5A); //Output Compare Interrupt Enable 1 A
	TCCR5B |= (1<<CS50); //set prescaler to 1, starting timer
	sei(); // enable global interrupts
}

ISR(TIMER5_COMPA_vect){	//interrupt flag in ICF1 will be automatically cleared	milliseconds++; //increment number of milliseconds since boot}