/**

 * Original https://www.mikrocontroller.net/topic/185804
 * Editiert von Christoph Nils Schäfer <mail@christoph-nils.de>
 * DMX Switch zur Signalisierung der DMX Verfügbarkeit
 * und ein per DMX schaltbarer Kanal
 * Startadresse per Taster und DMX einstellbar

 * DMX Receiver 
 * 2*8bit linear pwm, 2*16bit exponential pwm
 *
 * @author Andi Dittrich <andi.dittrich@a3non.org>
 * @website http://www.a3non.org
 * @modified 31.07.2010
 * @license CC BY-NC
 * @license http://creativecommons.org/licenses/by-nc/3.0/de/
 * @target AtTiny2313
 * @cpu 16MHz
*/
#define F_CPU 16000000UL

#include <avr/interrupt.h>
#include <stdio.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "mydefs.h"

// number of receiving channels
#define DMX_CHANNELS 1

//PD0	RX
//PD6	Taster Set-CH-n
//PB0	Relais DMX-OK
//PB1	Relais DMX-CH-n

#define	RX_PORT			SBIT( PORTD, 0 )
#define	RX_DDR			SBIT( DDRD,  0 )
#define	Taster_PORT		SBIT( PORTD, 6 )
#define	Taster_PIN		SBIT( PIND,  6 )
#define	Taster_DDR		SBIT( DDRD,  6 )
#define	Relais_0_OUT	SBIT( PORTB, 0 )
#define	Relais_0_DDR	SBIT( DDRB,  0 )
#define	Relais_1_OUT	SBIT( PORTB, 1 )
#define	Relais_1_DDR	SBIT( DDRB,  1 )

// DMX address
volatile uint16_t DMXAddress = 1;

// dmx storage array
volatile uint8_t dmx_data[DMX_CHANNELS];

// dmx states
enum {IDLE = 0, BREAK, READ, SETADDRESS};

// actual dmx state
volatile uint8_t DMXState = IDLE;

// valid sequence
volatile uint8_t VALID_DATA = 0;

// channel input read count
volatile uint16_t DMXFrameCount = 0;


// MAIN
int main(){
	// disable ISR
	cli();
	
	RX_DDR = 0;
	RX_PORT = 0;
	Taster_DDR = 0;
	Taster_PORT = 1;
	Relais_0_OUT = 0;
	Relais_0_DDR = 1;
	Relais_1_OUT = 0;
	Relais_1_DDR = 1;

	// 250k baud - 16MHz - U2X=0
	UBRRL = 3;
	UBRRH = 0;
	
	
	// UART CONTROL
	// 7:0 - RXC - USART Receive Complete
	// 6:0 - TXC - USART Transmit Complete
	// 5:0 - UDRE - USART Data Register Empty
	// 4:0 - FE - Frame Error
	// 3:0 - DOR - Data OverRun
	// 2:0 - UPE - USART Parity Error
	// 1:0 - U2X - Double the USART Transmission Speed
	// 0:0 - MPCM - Multi-processor Communication Mode
	UCSRA = 0b00000000;
	
	// 7:0 - reserved
	// 6:0 - UMSEL - async operation
	// 5:0 - UPM1 - no paritiy
	// 4:0 - UPM0 - no paritiy
	// 3:1 - USBS - 2 stop bits
	// 2:1 - UCSZ1 - 8bit mode
	// 1:1 - UCSZ0 - 8bit mode
	// 0:0 - UCPOL - clock polarity, rising edge
	UCSRC = 0b00001110;

	// UART CONTROL
	// 7:1 - RXCIE - RX Complete Interrupt Enable
	// 6:0 - TXCIE - TX Complete Interrupt Disable
	// 5:0 - UDRIE - USART Data Register Empty Interrupt Disable
	// 4:1 - RXEN - Receiver Enable
	// 3:0 - TXEN - Transmitter Disable
	// 2:0 - UCSZ2 - 8bit mode
	// 1:0 - RXB8 - Receive Data Bit 8
	// 0:0 - TXB8 - Transmit Data Bit 8
	UCSRB = 0b10010000;

	// WATCHDOG - 4s
	// 7:0 - Watchdog Interrupt Flag
	// 6:1 - Watchdog Interrupt Enable
	// 5:1 - Watchdog Timer Prescaler3
	// 4:1 - Watchdog Change Enable
	// 3:0 - Watchdog System Reset Disable
	// 2:0 - Watchdog Timer Prescaler2
	// 1:0 - Watchdog Timer Prescaler1
	// 0:0 - Watchdog Timer Prescaler0
	WDTCR = 0b01110000;	

	// reset watchdog
	MCUSR &= ~(1<<WDRF);
			
	// initialize DMX State
	DMXState = IDLE;

	// initialize data array
	for (uint8_t i=0;i<DMX_CHANNELS;i++){
		dmx_data[i]= 0;
	}

	//Load DMX Address
	DMXAddress = eeprom_read_word(0);

	// enable dmx rx
	sei();

	// main loop
	while (true){
		// assign channels
		if (VALID_DATA){
			// on valid frame, assign values to ouputs
			
			if(dmx_data[0]>=128)
				Relais_1_OUT = 1;
			else
				Relais_1_OUT = 0;
			Relais_0_OUT = 1; //DMX OK
		}else{
			// deactivate all outputs on dmx-line error
			
			Relais_1_OUT = 0;	
			Relais_0_OUT = 0; //DMX not OK
		}

	}

	return 1;
};


/**
 *	ISR SECTION
 */
// DMX RECEIVE
ISR (USART_RX_vect){
	// store ucsra register for frame error flag
	uint8_t statusreg = UCSRA;

	// get data
	uint8_t DMXDataByte = UDR;

	// check for break (error flag is set)
	if (bit_is_set(statusreg, FE)){
		// reset frame counter
		DMXFrameCount = 0;

		// set break status
		DMXState = BREAK;
	
		// abort on frame error
		return;
	}

	// state machine	
	switch (DMXState){
		case BREAK:
			// check for valid startbyte
			DMXState = (DMXDataByte == 0) ? READ : IDLE;
			if(DMXState==READ&&Taster_PIN==0) DMXState = SETADDRESS;
			break;

		case IDLE:
			// do nothing -> wait for break
			break;

		case READ:
			// increment frame count
			DMXFrameCount++;				
			
			// wait for dmx address
			if (DMXFrameCount>=DMXAddress){
				// store value in array
				dmx_data[DMXFrameCount-DMXAddress] = DMXDataByte;
			}
				
			// wait for end of reading channels
			if (DMXFrameCount >= DMXAddress+DMX_CHANNELS-1){
				// activate PWM/Ouput updates
				VALID_DATA = 1;
					
				// set status idle => no action
				DMXState = IDLE;
					
				// reset watchdog
				wdt_reset();
			}			

			break;
		case SETADDRESS:
			// increment frame count
			DMXFrameCount++;
			if(DMXDataByte>=128)
			{
				//Save DMX Address
				eeprom_write_word(0, DMXFrameCount);
				DMXAddress = DMXFrameCount;
				// reset watchdog
				wdt_reset();
				Relais_1_OUT = 0;
				_delay_ms(200);
				Relais_1_OUT = 1;
				_delay_ms(200);
				Relais_1_OUT = 0;
				_delay_ms(200);
				Relais_1_OUT = 1;
				_delay_ms(200);
				Relais_1_OUT = 0;
			}

			if(DMXFrameCount >= 512)
			// set status idle => no action
			DMXState = IDLE;

			// reset watchdog
			wdt_reset();
			break;
	}
}

// Watchdog ISR
ISR(WDT_OVERFLOW_vect){
	// deactivate outputs
	VALID_DATA = 0;
}


