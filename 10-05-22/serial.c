//Example ATmega2560 Project
//File: serial.h
//Author: Robert Howie
//Created: 2 March 2015
//V1.0 Basic serial setup for printing to the serial terminal

//Derived from: http://www.github.com/abcminiuser/avr-tutorials/blob/master/USART/Output/USART.pdf?raw=true by Dean Camera
//See http://www.fourwalledcubicle.com/AVRArticles.php for more

#include "serial.h"

void serial0_init(void)
{
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //turn on the transmission and reception circuitry
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01); //use 8- bit character sizes
} //end serial2_init

void serial1_init(void)
{
	UCSR1B = (1<<RXEN1)|(1<<TXEN1); //turn on the transmission and reception circuitry
	UCSR1C = (1<<UCSZ10)|(1<<UCSZ11); //use 8- bit character sizes
} //end serial2_init

void serial2_init(void)
{
	UCSR2B = (1<<RXEN2)|(1<<TXEN2); //turn on the transmission and reception circuitry
	UCSR2C = (1<<UCSZ20)|(1<<UCSZ21); //use 8- bit character sizes
} //end serial2_init
{
	UCSR3B = (1<<RXEN3)|(1<<TXEN3); //turn on the transmission and reception circuitry
	UCSR3C = (1<<UCSZ30)|(1<<UCSZ31); //use 8- bit character sizes
} //end serial3_init