//include this .c file's header file
#include "Robot.h"


volatile uint8_t dataByte1=0, dataByte2=0, dataByte3=0, dataByte4=0;		// data bytes received
volatile bool new_message_received_flag=false, auton=true;

int main(void)
{
	//Variable Declaration
	int servoComp1 = 1000; //Initial duty cycle
	int servoComp2 = 1000;
	
	int rightMotorInt = 0; //Servo input integers
	int leftMotorInt = 0;
	
	//initialization code
	adc_init();
	serial2_init();
	
	DDRL |= (1<<PL7)|(1<<PL6)|(1<<PL5)|(1<<PL4);
	DDRB |= (1<<PB5)|(1<<PB6); //set pins 11 and 12 as output
	
	TCCR1A = 0; TCCR1B = 0; //Clear PWM registers
	TCCR1A |= (1 << WGM11); // set waveform gen mode to -> Phase correct, ICRn [1 0 1 0] - note registers
	TCCR1B |= (1 << WGM13);
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1);  // Clear on up-count, set on down-count
	TCCR1B |= (1 << CS11);   //  Clock select bit -> 8 prescaler
	ICR1 = 10000; //Top value, Use TOP = Fm/2*Fb*PRE
	OCR1A = 0; // Initial comp values, Duty=COMP/TOP
	OCR1B = 0;
	
	uint8_t sendDataByte1=0, sendDataByte2=0, sendDataByte3=0, sendDataByte4=0;		// data bytes sent
	uint32_t current_ms=0, last_send_ms=0; //timer variables
	UCSR2B |= (1 << RXCIE2); // Enable the USART Receive Complete interrupt (USART_RXC)
	milliseconds_init();
	sei();
	
	PORTL |= (1<<PL7);
	PORTL &= ~(1<<PL6);
	PORTL |= (1<<PL4);
	PORTL &= ~(1<<PL5);
	
	while(1)//main loop
	{
		//Battery Monitor
		if (adc_read(10) < 700){
			PORTL |= (1<<PL0);
		} else {
			PORTL &= ~(1<<PL0);
		}
		
		
		current_ms = milliseconds;
	//sending section
		if(current_ms-last_send_ms >= 20) //sending rate controlled here one message every 100ms (10Hz)
		{
			if(adc_read(0)/4 > 253) {sendDataByte1 = 253;} else {sendDataByte1 = adc_read(0)/4;} // update bytes to new values if they are not greater than 253 
			if(adc_read(1)/4 > 253) {sendDataByte2 = 253;} else {sendDataByte2 = adc_read(1)/4;}
			if(adc_read(14)/4 > 253) {sendDataByte3 = 253;} else {sendDataByte3 = adc_read(14)/4;}
			if(adc_read(15)/4 > 253) {sendDataByte4 = 253;} else {sendDataByte4 = adc_read(15)/4;}
			
			
			last_send_ms = current_ms;
			serial2_write_byte(0xFF); 		//send start byte = 255
			serial2_write_byte(sendDataByte1); 	
			serial2_write_byte(sendDataByte2); 
			serial2_write_byte(sendDataByte3); 	
			serial2_write_byte(sendDataByte4); 	
			serial2_write_byte(0xFE); 		//send stop byte = 254
		}
		
		if (auton){
			servoComp1 = adc_read(0) > 450 ? adc_read(2)*10 : 10000 - (adc_read(2)*8 - adc_read(1)*4);
			servoComp2 = adc_read(0) > 450 ? adc_read(1)*10 : 10000 - (adc_read(1)*8 - adc_read(2)*4);
			
			
		} else {
			if(new_message_received_flag)
			{
				rightMotorInt = dataByte1-128;
				leftMotorInt = dataByte4-128;

				new_message_received_flag=false;	// set the flag back to false
			}
		
		
			if(rightMotorInt > 0){
				PORTL |= (1<<PL6);
				PORTL &= ~(1<<PL7);
				servoComp1 = rightMotorInt*80;
			} else if(rightMotorInt < 0){
				PORTL |= (1<<PL7);
				PORTL &= ~(1<<PL6);
				servoComp1 = -rightMotorInt*80;
			} else { servoComp1 = 0; }
			

			if(leftMotorInt > 0){
				PORTL |= (1<<PL4);
				PORTL &= ~(1<<PL5);
				servoComp2 = leftMotorInt*80;
			} else if(leftMotorInt < 0){
				PORTL |= (1<<PL5);
				PORTL &= ~(1<<PL4);
				servoComp2 = -leftMotorInt*80;
			} else { servoComp2 = 0; }
		
		
			if(servoComp1 > 10000){
				servoComp1 = 10000;
			}
			if(servoComp2 > 10000){
				servoComp2 = 10000;
			}
		}
		
		OCR1A = servoComp1; // Duty=COMP/TOP
		OCR1B = servoComp2;
	}
	return(1);
}//end main

ISR(USART2_RX_vect)  // ISR executed whenever a new byte is available in the serial buffer
{
	static uint8_t recvByte1=0, recvByte2=0, recvByte3=0, recvByte4=0;		// data bytes received
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
		case 5: //waiting for stop byte
		if(serial_byte_in == 0xFE) //stop byte
		{
			// update data bytes to the recived info
			dataByte1 = recvByte1;
			dataByte2 = recvByte2;
			dataByte3 = recvByte3;
			dataByte4 = recvByte4;
			
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

int clamp(int d, int min, int max) {
	const int t = d < min ? min : d;
	return t > max ? max : t;
}