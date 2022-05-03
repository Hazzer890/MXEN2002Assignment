//Example ATmega2560 Project
//File: ATmega2560Project.h
//Author: Robert Howie

#ifndef ROBOT_H_ //double inclusion guard
#define ROBOT_H_

//include standard libraries
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>

//include header files
#include "../serial.h" //minimal serial lib
#include "../adc.h" //minimal adc lib
#include "../milliseconds.h" //milliseconds timekeeping lib

//constants
//constants
#define BIG_NUMBER 120000
#define SERIAL_PRINT_MESSAGE "Hello student!\n"
#define BUILD_DATE __TIME__ " " __DATE__"\n"

#endif /* ATMEGA2560_H_ */