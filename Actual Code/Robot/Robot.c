//include this .c file's header file
#include "Robot.h"

#define MAX(x, y) (((x) > (y)) ? (x) : (y))  //Max and Min Macros to help with functions later
#define MIN(x, y) (((x) < (y)) ? (x) : (y))  //Uses ternary operators to select either the larger or smaller value


volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0, dataByte5=0;		// data bytes received
uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0, sendDataByte5=0;		// data bytes sent
uint32_t current_ms=0, last_send_ms=0; //timer variables
volatile bool new_message_received_flag=false, auton=false;

int servoComp1 = 1000; //Initial duty cycle
int servoComp2 = 1000;
static char serial_string[200] = {0};

int main(void)
{
	//Variable Declaration
	int xInput, yInput, durationSet;
	
	//initialization code
	adc_init();
	serial0_init();
	serial2_init();
	
	DDRL |= (1<<PL7)|(1<<PL6)|(1<<PL5)|(1<<PL4);
	DDRB |= (1<<PB5)|(1<<PB6)|(1<<PB1); //set pins 11 and 12 as output
	DDRE |= (1<<PE3)|(1<<PE4);
	
	TCCR1A = 0; TCCR1B = 0; //Clear PWM registers
	TCCR1A |= (1 << WGM11); // set waveform gen mode to -> Phase correct, ICRn [1 0 1 0] - note registers
	TCCR1B |= (1 << WGM13);
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);  // Clear on up-count, set on down-count
	TCCR1B |= (1 << CS11);   //  Clock select bit -> 8 prescaler
	TCCR3A = 0; TCCR3B = 0; //Clear PWM registers
	TCCR3A |= (1 << WGM31); // set waveform gen mode to -> Phase correct, ICRn [1 0 1 0] - note registers
	TCCR3B |= (1 << WGM33);
	TCCR3A |= (1 << COM3A1) | (1 << COM3B1);  // Clear on up-count, set on down-count
	TCCR3B |= (1 << CS31);   //  Clock select bit -> 8 prescaler
	ICR1 = 10000; //Top value, Use TOP = Fm/2*Fb*PRE
	ICR3 = 10000;
	OCR1A = 0; // Initial comp values, Duty=COMP/TOP
	OCR1B = 0;
	OCR3A = 0;
	OCR3B = 0;
	
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	
	milliseconds_init();
	sei();
	
	PORTL |= (1<<PL7);
	PORTL &= ~(1<<PL6);
	PORTL |= (1<<PL4);
	PORTL &= ~(1<<PL5);
	
	_delay_ms(100);
	while(1)//main loop
	{
		current_ms = milliseconds;
		//Send Data
		if(current_ms-last_send_ms >= 100) {//sending rate controlled here one message every 100ms (10Hz)
			sendData();
		}
		
		if(adc_read(15) < 170){
			PORTB |= (1<<PB1);
		}
		
		if (dataByte5 == 1) {
			auton = !auton;
			_delay_ms(200);
		}
		
		servoComp1 = 1500-(dataByte2*4);
		OCR3A = servoComp1;

		
		if(auton) {
			
			if (adc_read(0) >= 150) { //detected a wall
				yInput = 0;
				xInput = 0;
				if (adc_read(1) > adc_read(2)) {
					xInput = 300;
				} else {
					xInput = -300;
				}
				durationSet = 300; //300ms delay
			} else { //drive forwards code
				yInput = 300;
				xInput = 0;
				durationSet = 0;
				
				//correct if close to wall
				if (adc_read(1) >= 300) {
					xInput = 150;
				} else if (adc_read(2) >= 300) {
					xInput = -150;
				}
			} /*
			int v1 = adc_read(1) < 150 ? -400 : 0;
			int v2 = adc_read(2) < 150 ? 400 : 0;
			xInput = v1 + v2;
			yInput = (450 - adc_read(0))*0.5; */

		} else {
			if(new_message_received_flag){
			xInput = (dataByte3-126)*3;
			yInput = -(dataByte4-126)*3;
			durationSet = 0;
			new_message_received_flag = false;
			}
		}
		driveFunction(xInput, yInput, durationSet);
		
		sprintf(serial_string, "%4d, %4d, %4d || \n", adc_read(0), adc_read(1), adc_read(2));
		serial0_print_string(serial_string);
		
	}
	return(1);
}

//Function to Map values from a set of two intervals, to another set - Math from arduino reference
float mapF(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Generic function for driving motors, inputs are ["joystick x", "joystick y"]
void driveFunction(int xValue, int yValue, int dValue){
	float leftValue, rightValue;
	int leftMotor, rightMotor;
	
	leftValue = yValue + xValue;
	rightValue = yValue - xValue;
	
	leftMotor = mapF(leftValue, -512, 512, -9999, 9999);
	rightMotor = mapF(rightValue, -512, 512, -9999, 9999);
	
	
	if(rightMotor > 100){
		PORTL |= (1<<PL7);
		PORTL &= ~(1<<PL6);
	} else if(rightMotor < -100){
		PORTL |= (1<<PL6);
		PORTL &= ~(1<<PL7);
	} else { rightMotor = 0; }
		
	if(leftMotor > 100){
		PORTL |= (1<<PL4);
		PORTL &= ~(1<<PL5);
	} else if(leftMotor < -100){
		PORTL |= (1<<PL5);
		PORTL &= ~(1<<PL4);
	} else { leftMotor = 0; }
		
	OCR1A = abs(rightMotor);
	OCR1B = abs(leftMotor);
	if(dValue == 300){
		_delay_ms(300);
	}
}

//Sending data over serial
void sendData(){
	if(adc_read(0)/4 > 253) {sendDataByte1 = 253;} else {sendDataByte1 = adc_read(0)/4;} // update bytes to new values if they are not greater than 253
	if(adc_read(1)/4 > 253) {sendDataByte2 = 253;} else {sendDataByte2 = adc_read(1)/4;}
	if(adc_read(2)/4 > 253) {sendDataByte3 = 253;} else {sendDataByte3 = adc_read(2)/4;}
	if(adc_read(15)/4 > 253) {sendDataByte4 = 253;} else {sendDataByte4 = adc_read(15)/4;}
	
	last_send_ms = current_ms;
	serial2_write_byte(0xFF); 		//send start byte = 255
	serial2_write_byte(sendDataByte1);
	serial2_write_byte(sendDataByte2);
	serial2_write_byte(sendDataByte3);
	serial2_write_byte(sendDataByte4);
	serial2_write_byte(sendDataByte5);
	serial2_write_byte(0xFE); 		//send stop byte = 254
}

//Receiving data from serial
ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0, recvByte5=0;		// data bytes received
	static uint8_t serial_fsm_state=0;									// used in the serial receive ISR
	uint8_t	serial_byte_in = UDR2; //move serial byte into variable
	
	switch(serial_fsm_state) //switch by the current state
	{
		case 0:
		//do nothing, if check after switch case will find start byte and set serial_fsm_state to 1
		break;
		case 1: //waiting for first parameter
		recvByte1 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 2: //waiting for second parameter
		recvByte2 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 3: //waiting for third parameter
		recvByte3 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 4: //waiting for forth parameter
		recvByte4 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 5:
		recvByte5 = serial_byte_in;
		serial_fsm_state++;
		break;
		case 6: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// now that the stop byte has been received, set a flag so that the
			// main loop can execute the results of the message
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			dataByte3 = recvByte3;
			dataByte4 = recvByte4;
			dataByte5 = recvByte5;
			
			new_message_received_flag=true;
		}
		// if the stop byte is not received, there is an error, so no commands are implemented
		serial_fsm_state = 0; //do nothing next time except check for start byte (below)
		break;
	}
	if(serial_byte_in == 0xFF) //if start byte is received, we go back to expecting the first data byte
	{
		serial_fsm_state=1;
	}
}