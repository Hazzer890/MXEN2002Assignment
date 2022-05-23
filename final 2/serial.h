/*
 * Serial.h
 *
 * Created: 2/03/2015 5:41:43 PM
 *  Author: Robert
 */ 


#ifndef SERIAL_H_
#define SERIAL_H_

//include libraries
# include <avr/io.h>

//macros
# define BAUD_PRESCALE ((((F_CPU/16)+(USART_BAUDRATE/2))/(USART_BAUDRATE))-1)
#endif /* SERIAL_H_ */