#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdint.h>
#include "../utils.h"
#include "../serial/serial.h" /*servo specs here*/

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

uint8_t UART_getManyString(uint8_t* buf, uint8_t N){
	uint8_t i = 0;	
	uint8_t bytes_read = 0;
	while(i<N){
		bytes_read += UART_getString(buf);
		i++;
	}
	return bytes_read;
}
//---------------------------------------------------------
void UART_putString(uint8_t* buf){
  while(*buf){
    UART_putChar(*buf);
    ++buf;
  }
}

//---------------------------------------------------------
/*bool handShake(){
    uint8_t buf[5];
    //echo the buffer for 5 times
    for (int i=0 ; i<5 ; i++){
        uint8_t buf[5];
        UART_getString(buf);
        UART_putString(buf);
    }

    UART_getString(buf);
    if(buf == "done"){
        UART_putString((uint8_t*)"handshake done.\n");
        return 1;
    }
    else return 0;
}*/
//---------------------------------------------------------
inline uint16_t decodeX(uint8_t* buf){
	return( (buf[1]<<8) | buf[0] );
}

inline uint16_t decodeY(uint8_t* buf){
	return( (buf[3]<<8) | buf[2] );
}
/*
void motionTesting(){
  uint16_t pulses[6] = { 22500, 2350, 24500, 25500, 24500, 2350 };
  uint8_t i=0, n=50;
  for (uint8_t j=0; j<n ; i++){
    OCR3A = pulses[i];
    OCR4A = pulses[i];
    if( i==5 ) i=0;
    else i++;
    _delay_ms(100);
  }
}
*/


//---------------------------------------------------------
void PWM_init(void){
  /*	ServoX = Timer 3 = digital pin 5 = DDRE
	    ServoY = Timer 4 = digital pin 6 = DDRH
  */
  //Data direction register
  DDRE |= 0xFF;    //digital pin 5
  DDRH |= 0xFF;    //digital pin 6

  //Configure TIMER3
  TCCR3A = TCCRA_MASK;
  TCCR3B = TCCRB_MASK;

  //Configure TIMER4
  TCCR4A = TCCRA_MASK;
  TCCR4B = TCCRB_MASK;

  //Top timers value
  ICR3 = 48047;
  ICR4 = 48047;

}
//---------------------------------------------------------

//*** M A I N ********************************************/
	
uint8_t buf[5];
volatile uint8_t running, msg_rcv;
	  	
int main(void){
	cli();

	//set flags
	running = msg_rcv = 0;
	_delay_ms(100);
	buf[0]=buf[1]=buf[2]=buf[3]=buf[4] = 0x5D;

  const uint8_t mask = (1<<7);
  DDRB |= mask;
	PORTB = mask;
	
	UART_init();
	UART_getManyString(buf, 5);
	_delay_ms(100);
	
	PWM_init();
  OCR3A = 24000;
  OCR4A = 24000;
	
	PORTB = 0;
	sei();
	running = 1;

	while(running){ ////////////////////////////////// 
		if(msg_rcv){
			PORTB = mask;
			_delay_ms(10);
			PORTB = 0;
			msg_rcv = 0;
		}
		continue;
	} ////////////////////////////////////////////////
}

ISR(USART0_RX_vect){
	UART_getString(buf);
  OCR3A = decodeX(buf);
  OCR4A = decodeY(buf);
	msg_rcv = 1;
}

/********************************************
*	@Author: .                        		*
*											*
*********************************************/

