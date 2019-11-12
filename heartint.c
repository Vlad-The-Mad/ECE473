//heartint.c
//setup TCNT1 in pwm mode, TCNT3 in normal mode 
//set OC1A (PB5) as pwm output 
//pwm frequency:  (16,000,000)/(1 * (61440 + 1)) = 260hz
//
//Timer TCNT3 is set to interrupt the processor at a rate of 30 times a second.
//When the interrupt occurs, the ISR for TCNTR3 changes the duty cycle of timer 
//TCNT1 to affect the brightness of the LED connected to pin PORTB bit 5.
//
//to download: 
//wget http://www.ece.orst.edu/~traylor/ece473/inclass_exercises/timers_and_counters/heartint_skeleton.c
//to print: a2ps -P <printer> -1 --font-size=9 heartint_skeleton.c

#include <avr/io.h>
#include <avr/interrupt.h>

#define TRUE  1
#define FALSE 0

uint16_t brightness[10] = {} ;

ISR() {
//if (alarm_ringing == 1) 
PORTC ^= 0xfe;
//else PORTC = 0;

}

int main() {

  DDRC = 0b00000001; //set port C bit one to output

//setup timer counter 1 as the pwm source

  TCCR1A |= ((1 << COM1A1) | (1 << COM1A0) | (1 << WGM11) );//fast pwm, normal port operation, clear@bottom, 
                                        //(inverting mode) ICR1 holds TOP, load 2x frequency

  TCCR1B |= ((1 << WGM12) | (1 << WGM13) | (1 << CS10)); //use ICR1 as source for TOP, use clk/1

  TCCR1C  = 0;//no forced compare 

  ICR1    = 33333;//top is at 33,333, giving frequency 

  
//setup timer counter 3 as the interrupt source, 30 interrupts/sec
// (16,000,000)/(8 * 2^16) = 30 cycles/sec
/*
  TCCR3A =                              //normal mode

  TCCR3B =                              //use clk/8  (15hz)  

  TCCR3C =                              //no forced compare 

  ETIMSK =                              //enable timer 3 interrupt on TOV
*/
  sei();                                //set GIE to enable interrupts
  while(1) { } //do forever
} 
