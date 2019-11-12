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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

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
volatile uint8_t inc_value;
volatile uint8_t add_mode;


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
        static uint16_t  state[2] = {0};  //holds present state of each pushbutton

	for (inc=0; inc < 20; inc++) {
	for (check=0; check < 3; check++) { //shifts each array left by its button state
		state[check-1] = (state[check-1] << 1) | (! bit_is_clear(PINA, check)) | 0xE000; 
		if (state[check-1] == 0xF000) //return a value when one of the pins has been down for 12 cycles
				return check; //returns the pin number that was pulled down, or 0
	}
	}
	return 0;
}
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
	uint16_t temp = sum;
	uint8_t number_items = 0;
	//determine how many digits there are 
	//break up decimal sum into 4 digit-segments
	while (temp >= 1000) {
		temp -= 1000;
		//increment the number of items
		number_items++;
		//set the 1000's output
		segment_data[4] = dec_to_7seg[1];
	} 
	//0 out digit if it's leading
	if (sum < 1000)
		segment_data[4] = dec_to_7seg[11];

	number_items = 0;
	segment_data[2] = dec_to_7seg[11];
	while (temp >= 100) {
		temp -= 100;
		//increment the number of items
		number_items++;
	} 
	//set the 100's
	segment_data[3] = dec_to_7seg[number_items];
	//0 out digit if it's leading
	if (sum < 100)
		segment_data[3] = dec_to_7seg[11];

	number_items = 0;
	while (temp >= 10) {
		temp -= 10;
		//increment the number of items
		number_items++;
	} 
	//set the 10's output
	segment_data[1] = dec_to_7seg[number_items];
	//0 out digit if it's leading
	if (sum < 10)
		segment_data[1] = dec_to_7seg[11];
	//set the 1's output
	segment_data[0] = dec_to_7seg[temp];
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
//Initalize tcnt0 counter for interrupt to trigger every 0.25 ms
//***********************************************************************                                                                             
 
void init_tcnt0(){                                                                                                                                    
  ASSR  |=  (1<<AS0);                //run off external 32khz osc (TOSC)                                                                              
  //enable interrupts for normal mode, overflow
  //timer counter 0 setup, running off i/o clock
  TIMSK |= (1<<OCIE0);             //enable interrupts
  TCCR0 |= ((1<<CS00));//| (1<<CS01));  //normal mode, prescale by 8
  OCR0 = 1; //32khz / 8 = 0.25 ms period
}                                                                                                                                                     

//***********************************************************************                                                                             
//                           init_general
//Initalizes coded values for display on seven seg, alarm values to ensure good
//starting logic in state machine, global interrupt
//***********************************************************************                                                                             
void init_general() {
	
	DDRD |= (1 << PD4); //make PORTD3 a ground for the encoders
	PORTD &= ~(1 << PD4);
	//NOTE: PB3 has some sort of error.  when I set it to act as
	// an output, it prevents me from using PB2.  
	//set port bits 4-7 B as outputs for LED decoder
	DDRB |= ((1 << PB2) | (1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));
	PORTB &= ~(1<<7);    // set PB7 low so it acts like gnd

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
	inc_value = 0;
	add_mode= 0;
	Sme_alarm = 0;
}

//***********************************************************************                                                                             
//                     ISR for timer counter zero                                                                                                     
//***********************************************************************                                                                             
ISR(TIMER0_COMP_vect){
	//store digit display before sending it
	uint8_t temp = PORTB;
	uint8_t temp_digits = PORTA;
	//check portA for button presses
	DDRA = 0x00;                                                                                                                          
	PORTA = 0xff;                                                                                                                         
	//enable tristate buffer for pushbutton switches                                                                                      
	PORTB |= ((1 << PB4) | (1 << PB5) | (1 << PB6));                                                                                      
	//now check each button and increment the count as needed                                                                             
	//If porta pin 1 is pressed, flip the increment 2 flag
	//If porta pin 2 is pressed, flip the increment 4 flag
	//XOR so that previously enabled states will be disabled by another button press
	add_mode ^= chk_buttons();
	add_mode &= 0b00000011; // remove extra 1's
	PORTA = temp_digits;                                                                                                                         
	//disable tristate buffer for pushbutton switches                                                                                     
	PORTB &= ~(1 << PB4);                                                                                                                 
	//send indiciactor light patterns to the bar graph via SPI
	//(inc_value will be added to the current total for each encoder tick)
	switch(add_mode) {
		case 0: SPDR = 0x00; inc_value = 1; break;
		case 1:	SPDR = 0x01; inc_value = 2; break;
		case 2:	SPDR = 0x02; inc_value = 4; break;
		case 3:	SPDR = 0x03; inc_value = 0; break;
	}
	// Wait for reception complete 
	while (bit_is_clear(SPSR, SPIF)){}
	PORTD |= (1 << PD2);
	PORTD &= ~(1 << PD2);
	PORTE &= ~(1 << PE7);  //turn off inhibitor
	//PORTE |= (1 << PE7);  //turn off the clock inhibitor
	PORTE &= ~(1 << PE6);  //enable the encoder load output command
	PORTE |= (1 << PE6);  //enable the encoder load output command
	//send dummy value to SPDR to allow it to recieve values 
	SPDR = 0xff;
	while (bit_is_clear(SPSR, SPIF));
	uint8_t spi_input = SPDR;
	//send saved pattern back to the LED display
	PORTB = temp;
	//Filter out voltage values for each encoder
	uint8_t encoder[2];
	encoder[0] = 0b00000011 & spi_input;
	encoder[1] = 0b00001100 & spi_input;
	if (encoder[1] != 0)
		encoder[1] = encoder[1] >> 2;
	//check for state transisions indicating rotation for each encoder
	int i;
	for (i = 0; i < 2; i++) {
		//switch statement originally from slides on debouncing encoder inputs
		alarm_state[i] = encoder[i];
		switch(encoder[i]){ 
			//3 2 0 1 - > 3 CW
			//3 1 0 2 - > 3 CCW
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
					Sme_alarm += inc_value;
					//bound the count to 0 - 1023;
					if(Sme_alarm > 1023){Sme_alarm = 1;}
				} 
				if((acount[i] <= 0xFF) && (acount[i] > 0x90)){ 
					Sme_alarm -= inc_value; 
					//bound the count to 0 - 1023
					if(Sme_alarm > 1023){Sme_alarm = 1023;}
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
	}
	//sets the 7seg display output based on sum of the count so far
	//break up the disp_value to 4, BCD digits in the array: call (segsum)
	segsum(Sme_alarm);
}

int main() {
	uint8_t display_count = 1;
	uint8_t i;
	//set up interrupt vector
	init_tcnt0();   //initalize timer counter zero
	spi_init();   //initalize spi 
	init_general(); //initalize all other widely used items
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
                        PORTB &= 0x0f;                                                                         
                        PORTB |= dec_set[digit];                                                                         
			PORTA = segment_data[digit];
                        _delay_us(200);                                                                                                                 
		}
        }//while                                                                                                                                      
        return 0;        
}//main
