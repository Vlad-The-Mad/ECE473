// lab1_code.c 
// R. Traylor
// 7.21.08

//This program increments a binary display of the number of button pushes on switch 
//S0 on the mega128 board.

#include <avr/io.h>
#include <util/delay.h>

//*******************************************************************************
//                            debounce_switch                                  
// Adapted from Ganssel's "Guide to Debouncing"            
// Checks the state of pushbutton S0 It shifts in ones till the button is pushed. 
// Function returns a 1 only once per debounced button push so a debounce and toggle 
// function can be implemented at the same time.  Expects active low pushbutton on 
// Port D bit zero.  Debounce time is determined by external loop delay times 12. 
//*******************************************************************************
int8_t debounce_switch() {
  static uint16_t state = 0; //holds present state
  state = (state << 1) | (! bit_is_clear(PIND, 0)) | 0xE000;
  if (state == 0xF000) return 1;
  return 0;
}
//*******************************************************************************
// 				count 
// Takes the number of button presses and decrements by 10, counting how many
// times this occurs before the value is less than 10.  This value is shifted 
// 4 bits left, then combined with the remainer.  Returns the 
//  
//*******************************************************************************

int8_t count (uint8_t total) {
	uint8_t output_val = 0; //reset all the registers
	uint8_t lower_val = 0;
	uint8_t upper_val = 0;
	while(total >= 0x0A) { //subtract from the total until there are no more 10's
		upper_val++; //add to 10's count
		total = total - 10;
	}
	lower_val = total; //once the total is below 10, total is now the lower 8 bits
	upper_val = (upper_val << 4); //shifting the number of 10's left makes the upper level of 
	output_val = upper_val | lower_val; // ORing the high and low values together combines them.
	return output_val;
}

//*******************************************************************************
// Check switch S0.  When found low for 12 passes of "debounce_switch(), increment
// the total.  Calculate the display to show Binary Coded Decimal.  When the total
// hits 100, it is reset to 0 to imitate wrapping around.
//*******************************************************************************
int main()
{
DDRB = 0xFF;  //set port B to all outputs
uint8_t total = 0;
while(1){     //do forever
 if(debounce_switch()) {
	 total++; //
	 if (total == 100) //reset the count when it hits 100
	 total = 0; 
	 PORTB = count(total);
 }  //if switch true for 12 passes, increment port B
  _delay_ms(2);                    //keep in loop to debounce 24ms
  } //while 
} //main

