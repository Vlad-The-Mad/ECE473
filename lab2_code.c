//lab2_code.c
//Vladimir Vesely
//10.15.19


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
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
// Port mapping:
// Port A:  bit0 brown  segment A
//          bit1 red    segment B
//          bit2 orange segment C
//          bit3 yellow segment D
//          bit4 green  segment E
//          bit5 blue   segment F
//          bit6 purple segment G
//          bit7 grey   decimal point
//               black  Vdd
//               white  Vss
uint8_t dec_to_7seg[12];


//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button array.  It shifts ones in to a register for each
//button till a button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
//uint8_t chk_buttons(uint8_t button) {
uint8_t chk_buttons() {
	uint8_t check = 0;
	static uint16_t  state[8] = {0};  //holds present state of each pushbutton
	for (check; check < 8; check++) { //shifts each array left by its button state
		state[check] = (state[check] << 1) | (! bit_is_clear(PINA, check)) | 0xE000; 
		if (state[check] == 0xF000) //return a value when one of the pins has been down for 12 cycles
			if (check == 0)
				return 8; //0 is false, so when the first button is pushed, return 8
			else 
				return check; //returns the pin number that was pulled down
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
//***********************************************************************************
//git init to make a git repository

//***********************************************************************************
uint8_t main()
{
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
	uint16_t count = 0;
	uint8_t button_register = 0;
	uint8_t digit;
	uint8_t dec_set[5];
	dec_set[0] = 0b00000000; //sets the first bjt to low / saturation
	dec_set[1] = 0b00010000; //sets the second bjt to low / saturation
	dec_set[2] = 0b00100000; //sets the third bjt to low / saturation
	dec_set[3] = 0b00110000; //sets the fourth bjt to low / saturation
	dec_set[4] = 0b01000000; //sets the fifth bjt to low / saturation
	//set port bits 4-7 B as outputs
	DDRB |= ((1 << PB4) | (1 << PB5) | (1 << PB6) | (1 << PB7));
	while(1){
		//insert loop delay for debounce
		//make PORTA an input port with pullups 
		//do this initalization only just before using the ports!!!!
		DDRA = 0x00;
		PORTA = 0xff;
		//enable tristate buffer for pushbutton switches
		PORTB |= ((1 << PB4) | (1 << PB5) | (1 << PB6));
		//now check each button and increment the count as needed
		uint8_t move;
		move = chk_buttons();
		if (move) {
			if (move == 8)
				count++;
			else
				count += (1 << move);
		}
		//disable tristate buffer for pushbutton switches
		PORTB &= ~(1 << PB4);
		PORTA = 0x00;
		//bound the count to 0 - 1023
		if (count > 1023)
			count = 1;
		//break up the disp_value to 4, BCD digits in the array: call (segsum)
		segsum(count);
		//bound a counter (0-4) to keep track of digit to display 
		digit = 0;
		//make PORTA an output
		DDRA = 0xff;
		//send 7 segment code to LED segments
		//send PORTB the digit to display
		//iterate, turning on each digit / led array 
		for (digit; digit < 5; digit++) {
			PORTB = dec_set[digit]; 
			PORTA = segment_data[digit];
			_delay_ms(1);
		}
		//update digit to display
	}//while
	return 0;
}//main
