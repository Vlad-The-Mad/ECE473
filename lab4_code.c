//lab3_code.c
//Vladimir Vesely
//10.29.19


// lab2_skel.c 
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include "hd44780.h"

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
// Port mapping:
// Port A:  bit0 segment A
//          bit1 segment B
//          bit2 segment C
//          bit3 segment D
//          bit4 segment E
//          bit5 segment F
//          bit6 segment G
//          bit7 decimal point

volatile uint8_t dec_to_7seg[12];
volatile uint8_t past_astate[2];
volatile uint8_t alarm_state[2];
volatile uint8_t acount[2];
volatile uint16_t Sme_alarm;
volatile uint8_t add_offset;
volatile uint8_t add_mode;
volatile uint16_t timer_ticks;
volatile uint16_t time_array_secminhr[3];
volatile uint16_t alarm_time[2];
volatile uint16_t snooze_end[3];
volatile uint16_t ALM[3]; //make ALM 3 to match the prototype for the min_hr function
volatile uint8_t alarm_ringing; //tells whether alarm is going off at any point
volatile uint8_t snoozed;
uint16_t adc_result;     //holds adc result
uint8_t brightness;


//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button array.  It shifts ones in to a register for each
//button till a button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//******************************************************************************

uint8_t chk_buttons() {
        uint8_t check = 0;
        uint8_t inc = 0;
        static uint16_t  state[5] = {0};  //holds present state of each pushbutton

	for (inc=0; inc < 20; inc++) {
	for (check=0; check < 5; check++) { //shifts each array left by its button state
		state[check] = (state[check] << 1) | (! bit_is_clear(PINA, check)) | 0xE000; 
		if (state[check] == 0xF000) //return a value when one of the pins has been down for 12 cycles
				return (1 << check); //returns the pin number that was pulled down, or 0
	}
	/*for (check=0; check < 5; check++) { //shifts each array left by its button state
		state[check-1] = (state[check-1] << 1) | (! bit_is_clear(PINA, check)) | 0xE000; 
		if (state[check-1] == 0xF000) //return a value when one of the pins has been down for 12 cycles
				return check; //returns the pin number that was pulled down, or 0
	}*/
	}
	return 0;
}
//******************************************************************************

//***********************************************************************************
//                                   min_hr
//takes an array of 16-bit binary values and places the appropriate equivalent hr:min
//decimal time segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//***********************************************************************************
void min_hr(volatile uint16_t time_array[3]) {
	uint16_t temp_hr = time_array[2];
	uint16_t temp_min = time_array[1];
	uint8_t number_items = 0;
	//determine how many digits there are 
	//break up decimal sum into 4 digit-segments
	while (temp_hr >= 10) {
		temp_hr -= 10;
		//increment the number of items
		number_items++;
	} 
		//set the upper hours digit
		segment_data[4] = dec_to_7seg[number_items];
		//set the upper hours digit
		segment_data[3] = dec_to_7seg[temp_hr];
	//0 out digit if it's leading
	//if (sum < 1000)
	//	segment_data[4] = dec_to_7seg[11];

	number_items = 0;
	while (temp_min >= 10) {
		temp_min -= 10;
		//increment the number of items
		number_items++;
	} 
		//set the upper hours digit
		segment_data[1] = dec_to_7seg[number_items];
		//set the upper hours digit
		segment_data[0] = dec_to_7seg[temp_min];
	//blank out leading zero digits 
	//now move data to right place for misplaced colon position
}//segment_sum

//***********************************************************************                                                                             
//                           spi_init                                                                                                              
//Initalizes the SPI port on the mega128. Does not do any further   
//external device specific initalizations.  Sets up SPI to be:                        
//master mode, clock=clk/2, cycle half phase, low polarity, MSB first, interrupts enabled
//***********************************************************************                                                                             
void spi_init(void) {
	//set portb bit 2 as output to drive the output to allow it to drive the clock
	DDRB |= ((1 << PB7) | (1 << PB2) | (1 << PB1) | (1 << PB0));//output mode for SS, MOSI, SCL
	DDRD |= (1 << PD2); //make PORTD3 a ground for the encoders
	SPCR = 0b01010000; //master mode, clk low on idle, leading edge sample, 1/2 speed clk
	SPSR = 0b00000001; //choose double speed operation
}

//***********************************************************************                                                                             
//                           init_tcnt0                                                                                                               
//Initalize tcnt0 counter for counting up
//***********************************************************************                                                                             
void init_tcnt0(){                                                                                                                                    
  ASSR  |=  (1<<AS0);                //run off external 32khz osc (TOSC)                                                                              
  //enable interrupts for normal mode, overflow
  //timer counter 0 setup, running off i/o clock
  TIMSK |= (1<<OCIE0);             //enable interrupts
  TCCR0 |= ((1<<CS00));//normal mode, no prescale
  //with only one bit, no prescale
  OCR0 = 1; //32,768 / 2^8 
}                                                                                                                                                     

//***********************************************************************                                                                             
//                           init_tcnt1
//***********************************************************************                                                                             
void init_tcnt1(){                                                                                                                                    
//  DDRB   |= (1<<PB7) | (1<<PB5);   //set port B bit five and seven as outputs
//setup TCNT1                                                                                                                                         
  TCCR1A = 0x00;                 //outputs disabled
  TCCR1B = ((1<<WGM12) | (1<<CS11));   //use OCR1A as source for TOP, use clk/8                                                                       
  TCCR1C = 0x00;                          //no forced compare                                                                                         
  OCR1A = 4000; //top value
}                                                                                                                                                     

//***********************************************************************                                                                             
//                           init_tcnt2
//Initalize the timer 2 to fast pwm mode in order to change the brightness
//of the 7 segment display
//***********************************************************************                                                                             
void init_tcnt2(){                                                                                                                                    
  ASSR  |=  (1<<AS0);                //run off external 32khz osc (TOSC)
  //enable interrupts for normal mode, overflow
  //timer counter 0 setup, running off i/o clock
  //TIMSK |= (1<<OCIE2);             //enable interrupts
  TCCR2 |= ((1<<COM21) | (1<<CS20)  | (1<<WGM20)| (1<<WGM21));// fast pwm mode
  //with only one bit, no prescale, so should be giving interrupts every 31us????
  OCR2 = 100; // start with high brightness
}                                                                                                                                                     

//***********************************************************************                                                                             
//                           adc_init
//Initalize the ADC to get values from the CDS cell
//***********************************************************************                                                                             
void adc_init(){                                                                                                                     
	//Initalize ADC and its ports                                                                                                           
	DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input                                                                       
	PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off
	ADMUX = 0b01000111;//single-ended, input PORTF bit 7, right adjusted, 10 bits
	ADCSRA = 0b10000111;//ADC enabled, don't start yet, single shot mode
	//take these out when running actual code
	DDRC  |= (1 << PC4);                                                                                                                     
	PORTC |= (1 << PC4); 
}                                                                                                                                                     
//***********************************************************************                                                                             
//                           init_general
//Initalizes coded values for display on seven seg, alarm values to ensure good
//starting logic in state machine, global interrupt
//***********************************************************************                                                                             
void init_general() {
	
	DDRD |= (1 << PD4); //make PORTD3 a ground for the encoders
	DDRC |= ((1 << PC3) | (1 << PC2)); //enable output for alarm
	PORTD &= ~(1 << PD4);
	//NOTE: PB3 has some sort of error.  when I set it to act as
	// an output, it prevents me from using PB2.  
	//set port bits 4-7 B as outputs for LED decoder
	DDRB |= ((1 << PB2) | (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));
	PORTB &= ~(1<<7);    // set PB7 low so it acts like gnd
	PORTC &= ~(1<<PC3);    // set PRC1 high so it supplies voltage to the amplifier
	PORTC &= ~(1<<PC2);    // set PRC1 high so it supplies voltage to the amplifier

	//set pins E 6 and 7 to output
	//crontrols the register load and shift out commands	
	DDRE |= ((1 << PE6) | (1 << PE7));
                    	 //PGFEDCBA
        dec_to_7seg[0] = 0b11000000; //0  
        dec_to_7seg[1] = 0b11111001; //1                                                                                                              
        dec_to_7seg[2] = 0b10100100; //2                                                                                                              
        dec_to_7seg[3] = 0b10110000; //3                                                                                                              
        dec_to_7seg[4] = 0b10011001; //4                                                                                                              
        dec_to_7seg[5] = 0b10010010; //5                                                                                                              
        dec_to_7seg[6] = 0b10000010; //6                                                                                                              
        dec_to_7seg[7] = 0b11111000; //7                                                                                                              
        dec_to_7seg[8] = 0b10000000; //8                                                                                                              
        dec_to_7seg[9] = 0b10010000; //9                                                                                                              
        dec_to_7seg[10] = 0b11111100; //colon                                                                                                         
        dec_to_7seg[11] = 0b11111111; //no digit                                                                                                      

	alarm_state[0] = 3;    //set initial alarm states to make sure we don't begin with wack values
	acount[0] = 0;// no acSon    
	alarm_state[1] = 3;    
	acount[1] = 0;// no acSon    
	add_offset = 0;
	add_mode= 0;
	Sme_alarm = 0;
	//initalizes the counter that leads up to seconds
	timer_ticks = 0;
	time_array_secminhr[2] = (__TIME__[0]-48)*10 + (__TIME__[1]-48);
	time_array_secminhr[1] = (__TIME__[3]-48)*10 + (__TIME__[4]-48);
	time_array_secminhr[0] = (__TIME__[6]-48)*10 + (__TIME__[7]-48);
	ALM[2] = (__TIME__[0]-48)*10 + (__TIME__[1]-48);
	ALM[1] = (__TIME__[3]-48)*10 + (__TIME__[4]-48)+1;
	//ALM[2] = 8; //alarm hr for compare
	//ALM[1] = 0; //alarm minutes for compare
	ALM[0] = 0; //alarm enabled bit.  initially off
	
}

//***********************************************************************                                                                             
//                           alarm_routine
// Changes the state of the alarm.  State one prevents the alarm from triiggering
// State starts the alarm ringing
// State 3 checks to see if it should ring, based on the snooze setting
//***********************************************************************                                                                             
void alarm_routine(uint8_t option) {
	switch(option) {
	//turn off
		case 0: alarm_ringing = 0; 
			TIMSK &= ~(1<<OCIE1A); //turns off interrupt, 
			PORTC &= 0b11110111;//ensures the output to speaker is off
			break;
	//turn on
		case 1: alarm_ringing = 1; 
			ALM[0] = 1; 
			clear_display();
			string2lcd(" Ringing"); 
			PORTC |= (1 << PC3);//ensures the output to speaker is on
			TIMSK |= (1<<OCIE1A); //enable interrupt
			break;
	//check against snooze
		case 2: 		if(snooze_end[0] == time_array_secminhr[0] 
							&& snooze_end[1] == time_array_secminhr[1] 
							&& snooze_end[2] == time_array_secminhr[2]) {
						alarm_routine(1);
						snoozed = 0;
					}
			break;
	}
}

void increment_timer() {
	timer_ticks++;
	if (timer_ticks % 11 == 0) {
		//check the adc, then use the value at the CDC cell to change the duty cycle of
		//the 7 seg display
		ADCSRA |= (1<<ADSC);//poke ADSC and start conversion                                                  
			 while(bit_is_clear(ADCSRA,ADIF));//spin while interrupt flag not set
			 ADCSRA |= (1<<ADIF);//its done, clear flag by writing a one
			 adc_result = ADC;                      //read the ADC output as 16 bits
			 brightness = (adc_result >> 1);
			 OCR2 = brightness;
	}
	switch(timer_ticks){
		//turn off the colin
		case 70: segment_data[2] = dec_to_7seg[11];
			 break;
			 //perform the time check function lol
			 //also turn on the colin
		case 128: time_array_secminhr[0]++; 
			  segment_data[2] = dec_to_7seg[10];
			  timer_ticks = 0;
			  if (time_array_secminhr[0] == 60) { //at the end of a min, increment min
			  //this check is once a second (would be 1/min for non-debugging) to see if alarm should start,
			  //not whether it should continue after a snooze command
				  time_array_secminhr[1]++;
				  time_array_secminhr[0] = 0; //reset seconds to 0
			  if (ALM[0] == 1 && ALM[1] == time_array_secminhr[1] && ALM[2] == time_array_secminhr[2] && !snoozed) {
				  alarm_routine(1);
			  }
				  //only check to see if alarm should trigger on the minute transition
				  if (time_array_secminhr[1] == 60) {//at the end of a hr, increment min
					  time_array_secminhr[2]++;
					  time_array_secminhr[1] = 0; //reset min to 0
					  if (time_array_secminhr[2] == 25) {//at the end of a day, reset hr to 0
						  time_array_secminhr[2] = 0; 
					  }
				  }
			  }
			  break;
		default: break;
	}
}

void encoder_adjust(volatile uint16_t a_to_set[3]) {
	//send dummy value to SPDR to allow it to recieve values 
	SPDR = 0xff;
	while (bit_is_clear(SPSR, SPIF));
	uint8_t spi_input = SPDR;
	//Filter out voltage values for each encoder
	uint8_t encoder[2];
	encoder[1] = 0b00000011 & spi_input;
	encoder[0] = 0b00001100 & spi_input;
	if (encoder[0] != 0)
		encoder[0] = encoder[0] >> 2;
	//check for state transisions indicating rotation for each encoder
	int i;
	for (i = 0; i < 2; i++) {
		//switch statement originally from slides on debouncing encoder inputs
		alarm_state[i] = encoder[i];
		switch(encoder[i]){ 
			//3 2 0 1 - > 3 CW
			//3 1 0 2 - > 3 CCW
			//left encoder is responsible for HR, right encoder for minutes
			case 0:    
				if(past_astate[i] == 2){acount[i]++;} // CW    
				if(past_astate[i] == 1){--acount[i];} // CCW    
				if(encoder[i] == 1){alarm_state[i] = 1;}    
				if(encoder[i] == 2){alarm_state[i] = 2;}  break;
				past_astate[i] = encoder[i];
				break; 
			case 1:
				if(past_astate[i] == 0){acount[i]++;} // CW    
				if(past_astate[i] == 3){--acount[i];} // CCW    
				if(encoder[i] == 3){alarm_state[i] = 3;}    
				if(encoder[i] == 0){alarm_state[i] = 0;} break;
			case 2:
				if(past_astate[i] == 3){acount[i]++;} // CW    
				if(past_astate[i] == 0){--acount[i];} // CCW    
				past_astate[i] = alarm_state[i];    
				if(encoder[i] == 0){alarm_state[i] = 0;}    
				if(encoder[i] == 3){alarm_state[i] = 3;} break; 
			case 3:
				if(past_astate[i] == 1){acount[i]++;}    
				if(past_astate[i] == 2){--acount[i];}    
				past_astate[i] = alarm_state[i];    
				if((acount[i] >= 1) && (acount[i] <100)){ 
					a_to_set[i+1]++; // CW rotation increases
				} 
				if((acount[i] <= 0xFF) && (acount[i] > 0x90)){ 
					--a_to_set[i+1]; // CCS rotation decreases
				}    
				acount[i] = 0;     
				if(encoder[i] == 2){alarm_state[i] = 2;}// CW    
				if(encoder[i] == 1){alarm_state[i] = 1;}// CCW    
				if(encoder[i] == 0){alarm_state[i] = 0;} break; 
			default:    
				alarm_state[i] = 3;    
				acount[i] = 0;// no acSon
				past_astate[i] = encoder[i];    
		}// switch  
		past_astate[i] = alarm_state[i];
		if(a_to_set[1] == 60) { //Positive minute roll over -> min = 0, hr++
			a_to_set[1] = 0;
			a_to_set[2]++;
		}
		if(a_to_set[1] >= 60) { //Negative minute roll over -> min = 59, hr--
			a_to_set[1] = 59;
			--a_to_set[2];
		}
		if(a_to_set[2] == 25) //Positive hr roll over -> hr = 0
			a_to_set[2] = 0;
		if(a_to_set[2] >= 25) //Negative hr roll over -> hr = 24
			a_to_set[2] = 24;
	}


}

void time_set() {
	encoder_adjust(time_array_secminhr);
	min_hr(time_array_secminhr);
}

void alarm_set() {
	encoder_adjust(ALM);
	min_hr(ALM);
}


//***********************************************************************                                                                             
//                     snooze_set
//sets a time 10 seconds from when its
//***********************************************************************                                                                             
void snooze_set() {
	snooze_end[0] = time_array_secminhr[0]+10;
	snooze_end[1] = time_array_secminhr[1];
	snooze_end[2] = time_array_secminhr[2];
	if (snooze_end[0] >= 60) { //takes care of edge cases where snooze time would be impossible (near the minute)
		snooze_end[0] -= 60;
		snooze_end[1] += 1;
		if (snooze_end[1] >= 60) {
			snooze_end[1] -= 60;
			snooze_end[2] += 1;
			if (snooze_end[2] == 25) {
				snooze_end[2] = 0;
			}
		}
	}
	clear_display();
	string2lcd(" Snoozed");
	alarm_routine(0);
	snoozed = 1;
}
//***********************************************************************                                                                             
//                     ISR for timer counter one
//Flips output bit, sending waveform to the amplifier circuit.
//***********************************************************************                                                                             
ISR(TIMER1_COMPA_vect) {
	PORTC ^= 0b00001000;
}
//***********************************************************************                                                                             
//                     ISR for timer counter zero                                                                                                     
//***********************************************************************                                                                             
ISR(TIMER0_COMP_vect){
	increment_timer();
	//check portA for button presses
	DDRA = 0x00;                                                                                                                          
	PORTA = 0xff;                                                                                                                         
	//enable tristate buffer for pushbutton switches                                                                                      
	PORTB |= ((1 << PB4) | (1 << PB5) | (1 << PB6));                                                                                      
	PORTB &= ~(1 << PB0);
	//now check each button and increment the count as needed                                                                             
	//XOR so that previously enabled states will be disabled by another button press
	add_mode ^= chk_buttons();
	add_mode &= 0b00001111; // remove extra 1's
	//disable tristate buffer for pushbutton switches                                                                                     
	PORTB &= ~(1 << PB4);                                                                                                                 
	//send the armed message on the transition between it being not-armed and ALM[0] being set to 1
	//these should only happen when actually disabled, not just snoozed
	if (ALM[0] == 0 && (add_mode & 0x01) && !snoozed) 
		string2lcd(" ARMED");
	//disabled armed message on the transition between it being armed and ALM[0] being set to 0
	else if (ALM[0] == 1 && !(add_mode & 0x01) && !snoozed)
		clear_display();
	ALM[0] = (add_mode & 0x01); 
	if (alarm_ringing == 1 && ALM[0] == 0 && !snoozed) //end alarm tone if alarm disable is pressed
		alarm_routine(0);
	//add the call to LCD screen with "ALARMED"	
	switch(add_mode) {
		case 4: 
			//alarm setting mode: enabled/disabled by pressing button 2
			alarm_set(); break;
		case 3: 
			//if the alarm is going off, allow the user to snooze for 10 min
			if (alarm_ringing == 1)
				snooze_set(); 
			add_mode &= 0b11111101; break;
		case 8: 
			//time setting mode: moved/set by pressing button 1
			time_set(); break;
		default: 
			//min_hr mode (modified to be min/sec for testing): default, sends hr, min to display
			min_hr(time_array_secminhr); break;
	}
	alarm_routine(2); //check to see if snooze button was pressed
	//resets the snooze button.  need a soluation so that a snooze 
	//will not be submitted while the clock or alarm time is being set
	//add_mode &= ~4; 
	//alarm_snooze(); 

	//send indiciactor light patterns to the bar graph via SPI
	//SPDR = ALM[0];
	SPDR = add_mode & 0xfe;
	// Wait for reception complete 
	while (bit_is_clear(SPSR, SPIF)){}
	PORTD |= (1 << PD2);
	PORTD &= ~(1 << PD2);
	PORTE &= ~(1 << PE7);  //turn off inhibitor
	//PORTE |= (1 << PE7);  //turn off the clock inhibitor
	PORTE &= ~(1 << PE6);  //enable the encoder load output command
	PORTE |= (1 << PE6);  //enable the encoder load output command
	
}

int main() {
	//set up interrupt vector
	init_general(); //initalize all other widely used items
	init_tcnt0();   //initalize timer counter zero (checking inputs)
	init_tcnt1();   //initalize timer counter one (alarm tone generation)
	init_tcnt2();   //initalize timer counter two (PWM for LED brightness)
	spi_init();   //initalize spi 
	lcd_init();
	adc_init();
	sei();          //enable global interrupts
        uint8_t digit;                                                                                                                                
        uint8_t dec_set[5];                                                                                                                           
        dec_set[0] = 0b00000000; //sets the first bjt to low / saturation                                                                             
        dec_set[1] = 0b00010000; //sets the second bjt to low / saturation                                                                            
        dec_set[2] = 0b00100000; //sets the third bjt to low / saturation                                                                             
        dec_set[3] = 0b00110000; //sets the fourth bjt to low / saturation                                                                            
	dec_set[4] = 0b01000000; //sets the fifth bjt to low / saturation
	while(1){                                                                                                                                     
                //make PORTA an input port with pullups                                                                                               
                //do this initalization only just before using the ports!!!!                                                                          
                //bind a counter (0-4) to keep track of digit to display                                                                             
                digit = 0;                                                                                                                            
                //make PORTA an output                                                                                                                
                DDRA = 0xff;                                                                                                                          
                //send 7 segment code to LED segments                                                                                                 
                //send PORTB the digit to display                                                                                                     
                //iterate, turning on each digit / led array
                for (digit; digit < 5; digit++) {                                                                                                     
                        //PORTB &= 0x0f;                                                                         
                        PORTB &= 0b10001111;                                                                         
                        PORTB |= dec_set[digit];                                                                         
			if (ALM[0] == 1 && alarm_ringing == 0) //turning on the periods is the 7-seg "armed" indication
				PORTA = segment_data[digit] & 0b01111111;
			else 
				PORTA = segment_data[digit];
			_delay_us(200);                                                                                                                 
		}
        }//while                                                                                                                                      
        return 0;        
}//main
