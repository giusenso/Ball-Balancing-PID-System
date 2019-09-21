/**
 * @file avr.c
 * @author Giuseppe Sensolini [https://github.com/JiuSenso/Ball-Balancing-PID-System.git]
 * @brief 
 *      contains all the AVR work needed for this project.
 *      provide this feature:
 *      - read and write (UART)
 *      - packet decode
 *      - Pulse width modulation
 *      - ISR (Interrupt Service Routine)
 *      - led blink on receipt
 * 
 * @compiler: avr-libc
 * 
 * @date 2019-01-22
 * 
 */

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdint.h>
#include "pid.h"
#include "../utils.h"
#include "../serial/serial.h" /*servo specs here*/

#define MYUBRR F_CPU/16/9600-1

#define TCCRA_MASK	(1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);	//NON Inverted PWM
#define	TCCRB_MASK	(1<<WGM13)|(1<<WGM12)|(1<<CS10);	  //FAST PWM with NO

//---------------------------------------------------------
void UART_init(void){
  // Set baud rate
  UBRR0H = (uint8_t)((MYUBRR)>>8);
  UBRR0L = (uint8_t)(MYUBRR);

  UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);               /* 8-bit data */
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

void setupTimer(void){
  TCNT5 = 0x00;
	TCCR5A = 0x00;
	TCCR5B = (1 << WGM52) | (1 << CS50) | (1 << CS52); // set the prescaler to 1024
	OCR5A = (F_CPU/1024)*4-1; // count up to 4 per seconds (62499)
}

uint16_t getDeltaT(void){
  uint16_t dt = TCNT5 / 15624 * 1000;
  TCNT5 = 0x00;
  return dt;
}

//---------------------------------------------------------
/**
 * @brief correct saturated values
 * 
 * @param value 
 * @param T_MIN 
 * @param T_MAX 
 * @return short 
 */
short saturationFilter(short value , short T_MIN, short T_MAX){
  if (value <= T_MIN) return T_MIN;
  if (value >= T_MAX) return T_MAX;
  else return value;
}
/**
 * @brief compute PI-D control action.
 * 
 * @param pid 
 * @param b_axis 
 */
uint16_t PIDCompute(PID_t* pid, BallAxis_t b_axis) {

  //set old error and output
  pid->error[1] = pid->error[0];
  pid->output[1] = pid->output[0]; 

  //compute new error
  if (!pid->inverted_mode) pid->error[0] = pid->setpoint - b_axis.p[0];
  else pid->error[0] = b_axis.p[0] - pid->setpoint;

  //Integral: update and filter
  pid->integral += pid->error[0] * (pid->dt/1000);
  pid->integral = saturationFilter(pid->integral, -150, +150); 

  //Derivative: Update and filter
  b_axis._dp = saturationFilter(b_axis._dp, -150, +150);
  if(!pid->inverted_mode) b_axis._dp = -b_axis._dp;
    
  //output
  pid->output[0] =
    X_HALF_ANGLE +
    pid->Kp * pid->error[0] +
    pid->Ki * pid->integral +
    pid->Kd * (b_axis._dp/pid->dt);
    
  //output filter   
  pid->output[0] = saturationFilter(pid->output[0], pid->output[1]-1500, pid->output[1]+1500);
  pid->output[0] = saturationFilter(pid->output[0], pid->min, pid->max);
  return pid->output[0];
}



//========================================================/
//::::: M A I N ::::::::::::::::::::::::::::::::::::::::::/
//========================================================/
	
uint8_t buf[5];
volatile uint8_t running, msg_rcv;
Ball_t ball;
PID_t XPID, YPID;
	  	
int main(void){
	
  cli();  //interrupts disabled
  UART_init();

	//set flags
	running = msg_rcv = 0;
	buf[0]=buf[1]=buf[2]=buf[3]=buf[4] = 0x5D;

  //led on
  const uint8_t mask = (1<<7);
  DDRB |= mask;
	PORTB = mask;

	UART_getManyString(buf, 3); //burn some packets 
	_delay_ms(100);

  //initialize pid structures
  XPID.Kp         =   15;
  XPID.Ki         =   10;
  XPID.Kd         =   1.5;
  XPID.setpoint   =   SETPOINT_X;
  XPID.min        =   X_MIN_ANGLE;
  XPID.max        =   X_MAX_ANGLE;
  XPID.inverted_mode = true;

  YPID.Kp         =   15;
  YPID.Ki         =   10;
  YPID.Kd         =   1.5;
  YPID.setpoint   =   SETPOINT_Y;
  YPID.min        =   Y_MIN_ANGLE;
  YPID.max        =   Y_MAX_ANGLE;
  YPID.inverted_mode = false;

  //pid params set up
  //UART_getString(buf);
  //XPID.Kp = ((buf[1]<<8) | buf[0]) + 0.01*((buf[3]<<8) | buf[2]);
  

  //initialize ball structure
  ball.detected = false;
  ball.xb.p[0] = 0;
  ball.yb.p[0] = 0;
  ball.xb._dp = 0;
  ball.yb._dp = 0;

	PWM_init();
  OCR3A = 24000;  // X servo
  OCR4A = 24000;  // Y servo
	
	PORTB = 0;
	sei();    //interrupts enabled
	running = 1;

  /////////////////////////////////////
	while(running){  

		if(msg_rcv){
      PIDCompute(&XPID, ball.xb);
      PIDCompute(&YPID, ball.yb);
      OCR3A = XPID.output[0];
      OCR4A = YPID.output[0];

			PORTB = mask; //led blink
			_delay_ms(10);
			PORTB = 0;
			msg_rcv = 0;
		}
    
		continue;
	} ///////////////////////////////////

}

//========================================================/
/*
 * INTERRUPT SERVICE ROUTINE
 */
ISR(USART0_RX_vect){

  XPID.dt = YPID.dt = getDeltaT();
	UART_getString(buf);

  ball.xb.p[2] = ball.xb.p[1];
  ball.xb.p[1] = ball.xb.p[0]; 
  ball.xb.p[0] = (buf[1]<<8) | buf[0]; //decode X
  ball.xb._dp = 0.5 * (ball.xb.p[0] - ball.xb.p[2]);

  ball.yb.p[2] = ball.yb.p[1];
  ball.yb.p[1] = ball.yb.p[0];
  ball.yb.p[0] = (buf[3]<<8) | buf[2]; //decode Y
  ball.yb._dp = 0.5 * (ball.yb.p[0] - ball.yb.p[2]);

	msg_rcv = 1;
}



