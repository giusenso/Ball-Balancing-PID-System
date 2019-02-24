#include <util/delay.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "../utils.h"

#define BAUD_RATE 9600
#define MYUBRR (F_CPU/16/BAUD_RATE-1)

#define   PERIOD          1/F_CPU
#define   DEAD_BAND       3   *0.000001
#define   HALF_DEGREE     3.5 *0.000001
#define		MIN_ANGLE		    1100*0.000001
#define		MAX_ANGLE       1800*0.000001
#define		CENTER_ANGLE	  ((MAX_ANGLE-MIN_ANGLE)/2)+MIN_ANGLE

#define   MIN_STEP        HALF_DEGREE/PERIOD
#define   MIN_VALUE       MIN_ANGLE/PERIOD
#define   MAX_VALUE       MAX_ANGLE/PERIOD

void UART_init(void){
  // Set baud rate
  UBRR0H = (uint8_t)(MYUBRR>>8);
  UBRR0L = (uint8_t)MYUBRR;

  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); /* 8-bit data */ 
  UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);   /* Enable RX and TX */  

}

void UART_putChar(uint8_t c){
  // wait for transmission completed, looping on status bit
  while ( !(UCSR0A & (1<<UDRE0)) );

  // Start transmission
  UDR0 = c;
}

uint8_t UART_getChar(void){
  // Wait for incoming data, looping on status bit
  while ( !(UCSR0A & (1<<RXC0)) );
  
  // Return the data
  return UDR0;
}

//----------------------------------------------------------
// reads a string until the first newline or 0
// returns the size read
uint8_t UART_getString(uint8_t* buf){
  uint8_t* b0=buf; //beginning of buffer
  while(1){
    uint8_t c=UART_getChar();
    *buf=c;
    ++buf;
    // reading a 0 terminates the string
    if (c==0)
      return buf-b0;
    // reading a \n  or a \r return results
    // in forcedly terminating the string
    if(c=='\n'||c=='\r'){
      *buf=0;
      ++buf;
      return buf-b0;
    }
  }
}

void UART_putString(uint8_t* buf){
  while(*buf){
    UART_putChar(*buf);
    ++buf;
  }
}
//---------------------------------------------------------

//_uint8_t* ===> SERVOCONFIG_T*
inline void decodeConfig(uint8_t* buf, ServoConfig_t* config){
	config->servoX = (buf[0]<<8)|(buf[1]);
	config->servoY = (buf[2]<<8)|(buf[3]);
}

void PWMSetup(void){
	DDRB |= 0xFF; //PWM Pins as Out

	//Configure TIMER1
	TCCR1A|=(1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);	//NON Inverted PWM
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS10);	//FAST PWM with NO PRESCALING

  ICR1 = 48047;	//fPWM=333Hz (Period = 3ms Standard).
}

//******************************************************************************
int main(void){
  
  UART_init();
  UART_putString((uint8_t*)"write something, i'll repeat it\n");
  PWMSetup();
  
  uint8_t buf[4] = {0,0,0,0};
  ServoConfig_t* config = (ServoConfig_t*)malloc(sizeof(ServoConfig_t));
  config->servoX = 20000;
  config->servoY = 30000;
  while(1) {

    UART_getString(buf); _delay_ms(10);

    //decodeConfig(buf, config);
    UART_putString((uint8_t*)"received: ");
  
    OCR1A = config->servoX;
    UART_putString((uint8_t*)"A\n");
		_delay_ms(2000);

    OCR1A = 30000;
    UART_putString((uint8_t*)"C\n");
		_delay_ms(2000);

    UART_putString(buf);
    UART_putString((uint8_t*)"\n");
  }

  free(config);
}
