#include <util/delay.h>
#include <avr/io.h>
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
int main(void){

  UART_init();
  //if(!handShake()){ exit() }; //to be tested

  PWM_init();
  UART_putString((uint8_t*)"Ready to control servos.\n");

  uint8_t buf[5] = {0,0,0,0,0}; //[ x , x , y , y , '\n' ]
  
  OCR3A = (uint16_t)X_HALF_ANGLE;
  OCR4A = (uint16_t)Y_HALF_ANGLE;

  UART_getString(buf);
  UART_getString(buf);

  while(1) {
    UART_getString(buf);
    OCR3A = decodeX(buf);
    OCR4A = decodeY(buf);
  }
}











/********************************************
*	@Author: .                        		*
*											*
*********************************************/
