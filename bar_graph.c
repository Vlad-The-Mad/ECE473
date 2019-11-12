// bar_graph.c 
// R. Traylor
// 10.12.16

// Every half second, a new led is lit on the bargraph display via SPI
// !!Disconnect existing PORTA and PORTB connections!!
//
// Expected Connections:
// Bargraph board            mega128
// --------------       --------------------     
//     reglck           PORTD bit 0                        
//     srclk            PORTB bit 1 (sclk)
//     sdin             PORTB bit 2 (mosi)
//     oe_n             ground (ground (gnd on any port)
//     gnd              ground (gnd on any port)
//     vdd              vcc    (vcc on any port)
//     sd_out           no connect

#include <avr/io.h>
#include <util/delay.h>
//***********************************************************************
//                            spi_init                               
//**********************************************************************
void spi_init(void){

  DDRB   = 0b10000111; //output mode for SS, MOSI, SCLK, PB7
  PORTB &= ~(1<<7);    // set PB7 low so it acts like gnd
  DDRD   = (1<<2);     //output mode for PD2, the regclk

  SPCR   = 0b01010000; //master mode, clk low on idle, leading edge sample

  SPSR   = 0b00000001; //choose double speed operation
 }//spi_init
//**********************************************************************
//                                main                                 
//**********************************************************************
int main(){     

uint8_t display_count = 1; //holds count for display 

uint8_t i; //dummy counter

spi_init();  //initalize SPI port
while(1){                             //main while loop

    SPDR = display_count; //send display_count to the display 
    while (bit_is_clear(SPSR, SPIF)){} //spin till SPI data has been sent
    PORTD |= (1<<PD2);  //send rising edge to regclk on HC595
    PORTD &= ~(1<<PD2);  //send falling edge to regclk on HC595;
    for(i=0; i<=4; i++){_delay_ms(100);}         //0.5 sec delay

    SPDR = 0x0f; //send display_count to the display 
    while (bit_is_clear(SPSR, SPIF)){} //spin till SPI data has been sent
    PORTD |= (1<<PD2);  //send rising edge to regclk on HC595
    PORTD &= ~(1<<PD2);  //send falling edge to regclk on HC595;
    display_count = display_count << 1;//shift display_count for next time 

    if(display_count == 0x00) display_count = 1;//put indicator back to 1st positon

    for(i=0; i<=4; i++){_delay_ms(100);}         //0.5 sec delay
  } //while(1)
} //main
