#include <util/delay.h>
#include <avr/io.h>
#include <stdint.h>
#include "../utils.h"
#include "../serial_port/serial_port.h" /*servo specs here*/

#define TCCRA_MASK	(1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);	//NON Inverted PWM
#define	TCCRB_MASK	(1<<WGM13)|(1<<WGM12)|(1<<CS10);	//FAST PWM with NO

//---------------------------------------------------------
void UART_init(void){
  // Set baud rate
  UBRR0H = (uint8_t)(MYUBRR>>8);
  UBRR0L = (uint8_t)MYUBRR;

  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00); /* 8-bit data */
  UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);   /* Enable RX and TX */

}
//---------------------------------------------------------
void UART_putChar(uint8_t c){
  // wait for transmission completed, looping on status bit
  while ( !(UCSR0A & (1<<UDRE0)) );

  // Start transmission
  UDR0 = c;
}
//---------------------------------------------------------
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
//---------------------------------------------------------
void UART_putString(uint8_t* buf){
  while(*buf){
    UART_putChar(*buf);
    ++buf;
  }
}

//---------------------------------------------------------
inline uint16_t decodeX(uint8_t* buf){
	return( (buf[1]<<8) | buf[0] );
}

inline uint16_t decodeY(uint8_t* buf){
	return( (buf[4]<<8) | buf[3] );
}
//---------------------------------------------------------
void PWM_init(void){

  //Data direction register
	DDRB |= 0xFF;    //digital pin 11
  DDRE |= 0xFF;    //digital pin 5

	//Configure TIMER1
	TCCR1A = TCCRA_MASK;
	TCCR1B = TCCRB_MASK;

  //Configure TIMER3
  TCCR3A = TCCRA_MASK;
  TCCR3B = TCCRB_MASK;

  //Top timers value
  ICR1 = 48047;
  ICR3 = 48047;

}
//---------------------------------------------------------

//*** M A I N ******************************************************************
int main(void){

  UART_init();
  UART_putString((uint8_t*)"Ready to control servos.\n");
  PWM_init();

  uint8_t buf[6] = {0,0,0,0,0,0};

  uint16_t x_pulse = HALF_ANGLE;
  uint16_t y_pulse = HALF_ANGLE;

  while(1) {//_______________

    UART_getString(buf);
    OCR1A = decodeX(buf);
    OCR3A = decodeY(buf);
  }
}
