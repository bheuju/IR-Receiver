/*
	Title:
		IR Remote Control
		
	Description:
		Make use of existing TV/DVD remotes which utilize NEC Protocol to control other equipments
	
	Author:
		Bishal Heuju
	
	Hardware:
		uC: Atmega8
		Xtal: 12 MHz
		Fuse Bits: HFUSE=0xC9	LFUSE=0xEF
		Sensor: TSOP 1738, Connected to INT0 pin PD2 (pin 4 of uC)
	
	Useful Tools:
		IR_protocol_analyzer_v1.1	http://www.ostan.cz/IR_protocol_analyzer/
		Nero WaveEditor
	
	Other Features:
		Can show decoded NEC Protocol command in binary o/p
	
	Notes:
		* LED at pin B0 glows on power on and blinks when a valid NEC code is received
		* Load pins are D5, D6, D7
		* Power button 
		
	Operation Guidelines:
		If it is first run or power button (command - 0x2) is pressed within 1 sec of booting, then the keymapper is run indicated by blinker at pin B0
		In keymapper:
			Corresponding led and relay is activated to indicate that it is the one being mapped.
			When a button is pressed, it is written to the EEPROM and also assigned to the keys[].
			Successful mapping for a key is indicated by clearing of the blinker for about half a sec.
			If it is again blinking, it is asking for another key; enter till the blinker stops and glows continuously 
			Continuous glow of blinker indicates, the receiver is ready to receive commands
		If not first run; the keys are loaded from EEPROM to keys[]; indicated by the continuous glow of the blinker.
		Universal turn off (power button (0x2)) has been added to trun off all at once.
 */ 

/* TODO:
	D - Add EEPROM
	Add UART
	Simplify the defines for ticks (use variables as 9ms pre, 9ms post or similar)
	Select better o/p ports/pins
	Add description of algorithm used to decode NEC protocol
	Add option to display decoded data or not (use flag); may be making use of keys to choose which one to display, or add LCD feature
	Clean up the code, remove unnecessary variables, functions and comments
	Create an external documentation
	Add sleep mode to save power
	Currently MAX_KEYS set the no. of keymapping replace keymapper of power button with open/close;
		- open/close starts keymapper and again pressing open/close ends keymapper - can set any number of keys
	In keymapper, instead of blinking pin B0, use blinking of individual corresponding indicator to indicate asking ok key
*/

#define F_CPU	12000000

#define SETBIT(x, y) (x |= _BV(y))
#define CLEARBIT(x, y) (x &= ~(_BV(y)))

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>

#define TIMER_COMP_VAL	120		//For F_CPU = 12 MHz,	10us for 1 TICK

#define FALLING	0
#define RISING	1

#define MAX_KEYS	5		//defines how many keys to use

#define RECEIVE_START_BIT_HIGH	0
#define RECEIVE_START_BIT_LOW	1
#define RECEIVE_DATA_BIT		2
#define RECEIVE_STOP_BIT		3

#define TOL			0.1		//tolerance for timings; (not used currently)

//for 9 ms
#define TICKS11ms	1100
#define TICKS8ms	800
//for 4.5 ms
#define TICKS6ms	600
#define TICKS3ms	300
//for 0.562 ms
#define TICKS0o2ms	20
#define TICKS0o8ms	80
//for 1.687 ms
#define TICKS1o2ms	120
#define TICKS2o3ms	230
//for 2.25 ms (repeat pulse)
#define TICKS2ms	200
#define TICKS2o5ms	250

//volatile keyword is required so that the variable is modified by ISR
volatile unsigned int time;			//main timer,	updated by ISR
volatile unsigned int dataReady = 0;	//flag variable to set when data is completely received and ready

volatile unsigned int tDiff;		//holds time interval of each pulse; high or low
volatile uint8_t rxBuffer;		//stores the received data bits; address and command; later copied to respective variables

volatile unsigned int edge;			//edge of interrupt; Rising=1 Falling=0
volatile unsigned int state;

volatile unsigned int timeRise;		//time when rise started
volatile unsigned int timeFall;		//time when fall started

volatile unsigned int TIMEOUT;
volatile unsigned int PREPULSE;

//these are 8 bit variables
volatile uint8_t address = 0xFF, notaddress = 0xFF;	//variable to store received address
volatile uint8_t command = 0xFF, notcommand = 0xFF;	//variable to store received command

volatile unsigned int bitNo;

uint8_t keys[3];	//stores the key codes obtained from the IR remote; 5 keys mapped in total
volatile int t = 0;

/* Set INT0 pin on falling or rising edge */
/* Arguments: RISING=1 and FALLING=0 */
void setINT0Edge(unsigned int e)
{
	if (e == FALLING)
	{
		MCUCR |= (1<<ISC01);	//INT on falling edge
		MCUCR &= (1<<ISC00);	//ICS01 = 1, ICS00 = 0
		
		edge = FALLING;
	}
	else if (e == RISING)
	{
		MCUCR |= (1<<ISC01);	//INT on rising edge
		MCUCR |= (1<<ISC00);	//ICS01 = 1, ICS00 = 0
		
		edge = RISING;
	}
}
void setTimeout(unsigned int s)
{
	switch (s)
	{
		case RECEIVE_START_BIT_HIGH:
		{
			TIMEOUT = TICKS11ms;
			PREPULSE = TICKS8ms;
			break;
		}
		case RECEIVE_START_BIT_LOW:
		{
			TIMEOUT = TICKS6ms;
			PREPULSE = TICKS3ms;
			break;
		}
		case RECEIVE_DATA_BIT:
		{
			TIMEOUT = TICKS2o3ms;
			PREPULSE = TICKS0o2ms;
			break;
		}
	}
}

//for checking the address and command variables
//display 8 bit o/p at port B and C combined in the format,
//MSB: C5 C4 C3 C2 C1 C0 B2 B1 : LSB
//argument "code" is 8-bit
void displayCode(uint8_t code)
{
	//clear the respective port pins for new "code"
	PORTB &= ~(0x06);
	PORTC &= ~(0x3F);
	
	
	//shifting to arrange the format
	PORTB |= (code<<1) & (0x06);	//copy bit-1,0 of "code" => bit-2,1 of PORTB
	PORTC |= (code>>2) & (0x3F);	//copy bit-7,2 of "code" => bit-5,0 of PORTC
}

void initInterrupt()
{
	GICR |= (1<<INT0);		//Enable INT0 interrupt pin
	
	MCUCR |= (1<<ISC01);	//INT on falling edge
	MCUCR &= (1<<ISC00);	//ICS01 = 1, ICS00 = 0
}
void initTimer1()
{
	//Using CTC mode for hardware clear timer on compare
	TCCR1B |= (1<<CS10);	//Prescaler: 1;	CS12=0, CS11=0, CS10=1
	TCCR1B |= (1<<WGM12);	//WGM: CTC - TOP=OCR1A;	WGM13=0, WGM12=1, WGM11=0, WGM10=0
	
	TIMSK |= (1<<OCIE1A);	//Enable o/p compare interrupt
							//calls ISR when OCF1A flag is set
							//OCF1A flag is set when TCNT1 value matches TOP (OCR1A) value; 120 in our case
	
	OCR1A = TIMER_COMP_VAL;	//Set compare value; 120
}

void initRemote()
{	
	edge = FALLING;		//Need to capture falling edge
	
	state = RECEIVE_START_BIT_HIGH;
}
void resetRemote()
{
	//TODO: initRemote and resetRemote are same; need to use only one
	
	state = RECEIVE_START_BIT_HIGH;
	
	setINT0Edge(FALLING);	//Because start bit starts with falling edge (start 9ms burst); this is inverted to normal convention due to o/p of tsop is inverted
}

void keyMapper()
{
	uint8_t i;
	_delay_ms(500);	//wait for 1 sec to receive power button command; if the user choose to reset the keymappings
		
	//if no key in eeprom set; then load keymapper
	//memory 0 of EEPROM indicates if EEPROM contains data or not
	//0xFF - empty; 0x00 - data present
	if ((eeprom_read_byte(0) == 0xFF) || (command == 0x02))	//if first run or power button(0x2) pressed
	{
		for (i = 0; i < MAX_KEYS; i++)	//MAX_KEYS determine the max number of keys that can be mapped
		{
			command = 0xFF;		//reset command
			
			PORTB |= (1<<(i+1));	//turn on to indicate which device key is being mapped
			
			while(1)
			{
				PORTB ^= (1<<PINB0);	//this is for blinker to indicate keymapper has been activated
				_delay_ms(50);			//
				
				if (command != 0xFF)	//if a valid command is obtained from the remote
				{
					keys[i] = command;
					//write to eeprom
					eeprom_update_byte(i+1, command);	//not using 0; address 0 is uded to store 0xFF or 0x00 to indicate anything present in the memory or not
					break;
				}
			}
			
			//to indicate mapping successful and move to next one
			CLEARBIT(PORTB, PINB0);
			_delay_ms(500);
			//SETBIT(PORTB, PINB0);
			
			PORTB &= ~(1<<(i+1));	//turn off after key has been mapped for the device
		}
		eeprom_update_byte(0, 0x00);	//0x00 at memory 0 of EEPROM indicates that the data has been written; needed for checking during next bootup
	}
	else	//load keys[] from eeprom
	{
		for (i = 0; i < MAX_KEYS; i++)
		{
			keys[i] = eeprom_read_byte(i+1);	//read from eeprom
		}
	}
	command = 0xFF;		//reset command; before exiting keymapper
	SETBIT(PORTB, PINB0);
}

int main(void)
{
	DDRB |= 0x3F;		//B0-B4 set as o/p
	PORTB &= ~(0x3F);	//Reset o/p pins B0-B4
	
	DDRC |= 0x3F;		//C0-C5 as o/p
	PORTC &= ~(0x3F);
	DDRD |= 0xE1;		//D5-D7 and D0 as o/p
	PORTD &= ~(0xE1);
		
	sei();		//Set global interrupt flag,	similar to SREG |= (1<<7)
	
	initInterrupt();
	initTimer1();
	
	initRemote();
	
	//call keyMapper to check if keys need to assigned; toggled if first run or power button (0x2) is pressed
	keyMapper();
	uint8_t i;
	
	while(1)
	{		
		//displayCode(command);
		if (dataReady)
		{
			for (i = 0; i < MAX_KEYS; i++)
			{
				if (command == keys[i])
				{
					PORTB ^= (1<<(i+1));	//(i+5) because D5-D7 are the loads
				}
			}
			
			if (command == 0x2)		//universal turn off with power button
			{
				for (i = 0; i < MAX_KEYS; i++)
				{
					PORTB &= ~(1<<(i+1));	//direct writing to PORTB can also be done such as; PORTB &= ~(0x3E);
				}
			}
			
			dataReady = 0;
		}//end check_dataready
	}
}

ISR (INT0_vect)
{
	//CLEARBIT(PORTD, PIND5);
	
	GICR &= ~(1<<INT0);		//Disable INT0 interrupt
	
	tDiff = time;	//calculate how much time has been passed since last interrupt
	
	TCNT1 = 0;		//reset TCNT1 to measure next edge (interrupt) of input
	time = 0;		//reset the time variable to 0
	
	/* State machine starts here */
	/*
	 Start here
	 
	 
	*/
	switch (state)
	{
		case RECEIVE_START_BIT_HIGH:
		{
			if (edge == RISING)
			{
				if ((tDiff > TICKS8ms) && (tDiff < TICKS11ms))
				{
					//9ms pulse is verified
					//now move on to verifying 4.5ms pulse
					state = RECEIVE_START_BIT_LOW;
					//setTimeout(RECEIVE_START_BIT_LOW);
					setINT0Edge(FALLING);
				}
				else
				{
					//if 9ms pulse is not obtained start over
					resetRemote();
					
				}
			}
			else if (edge == FALLING)
			{
				setINT0Edge(RISING);	//to capture rising edge of 9ms burst, i.e. end of 9ms burst
			}
			break;
		}//end start_bit_high
		
		case RECEIVE_START_BIT_LOW:
		{
			if ((tDiff > TICKS3ms) && (tDiff < TICKS6ms))
			{
				//4.5ms pulse is verified
				//now move on to verifying data pulse
				state = RECEIVE_DATA_BIT;
				//setTimeout(RECEIVE_DATA_BIT);
				setINT0Edge(RISING);
				
				//reset variables to store address and command
				rxBuffer = 0;
				bitNo = 0;
			}
			else if ((tDiff > TICKS2ms) && (tDiff < TICKS2o5ms))
			{
				//TODO: for repeat pulse obtained
				
				//PORTB |= (1<<PINB2);
			}
			else
			{
				//if 4.5ms pulse is not obtained
				//i.e Start pulse is wrong; start over
				resetRemote();
			}
			break;
		}//end start_bit_low
		
		case RECEIVE_DATA_BIT:
		{
			//PORTD |= (1<<PIND5);
			if (edge == RISING)
			{
				if ((tDiff > TICKS0o2ms) && (tDiff < TICKS0o8ms))
				{
					//0.5ms pulse verified
					//correct beginning of bit is verified
					//now move on to verifying gap to determine if it is 0 or 1
					setINT0Edge(FALLING);
				}
				else
				{
					//beginning of data bit is incorrect; start over
					resetRemote();
				}
			}
			else if (edge == FALLING)
			{
				//PORTD |= (1<<PIND5);
				//detects 0 or 1
				rxBuffer = (rxBuffer >> 1);		//Right shifting buffer; stors received data bits
				if ((tDiff > TICKS0o2ms) && (tDiff < TICKS0o8ms))
				{
					//we got a 0 here
					//0.562ms pulse verified
					
					rxBuffer &= ~(0x80);		//make MSB 0; this does not become MSB of overall data, it is later shifted towards LSB by right shifting
					
					//SETBIT(PORTD, PIND6);
					
					setINT0Edge(RISING);
				}
				else if ((tDiff > TICKS1o2ms) && (tDiff < TICKS2o3ms))
				{
					//we got a 1 here
					//1.687ms pulse verified
					
					rxBuffer |= (0x80);		//make MSB 1;
					//PORTB |= (1<<PINB1);
					setINT0Edge(RISING);
				}
				else
				{
					//if some pulse is neither 0 nor 1 then corrupted data received
					//so reset; start over
					resetRemote();
				}
				
				if (bitNo == 7)				//address byte completed
				{
					address = rxBuffer;		//copy address	
					//displayCode(address);
				}
				else if (bitNo == 15)		//not address byte completed
				{
					notaddress = rxBuffer;	//copy !address	
					//displayCode(notaddress);
				}
				else if (bitNo == 23)		//command byte completed
				{
					command = rxBuffer;	//copy command
					//displayCode(command);
				}
				else if (bitNo == 31)		//not command byte completed
				{
					notcommand = rxBuffer;	//copy !command
					//displayCode(notcommand);
				}
				
				bitNo++;
					
				if (bitNo == 32)
				{
					//all address and command bits are received successfully
					//need to verify stop bit
					//move on to verifying stop bit
					state = RECEIVE_STOP_BIT;
				}
			}
			break;	
		}//end data_bit
		
		case RECEIVE_STOP_BIT:
		{
			//PORTB |= (1<<PINB1);
			if (edge == RISING)
			{
				if ((tDiff > TICKS0o2ms) && (tDiff < TICKS0o8ms))
				{
					//0.562ms pulse verified
					//stop bit high pulse verified
					//PORTB |= (1<<PINB1);
					
					/* Check for errors in received data bits */
					//SETBIT(PORTD, PIND5);
					if (!(address & notaddress) && !(command & notcommand))
					{
						//
						dataReady = 1;
					}					
					else
					{
						dataReady = 0;
					}
					
					//All data received, so reset; start over
					resetRemote();
					//SETBIT(PORTD, PIND5);
				}
			}
			break;
		}//end stop_bit
	}
	
	GICR |= (1<<INT0);		//Re-Enable INT0 interrupt
}

ISR (TIMER1_COMPA_vect)
{
	//This is called every 10us
	//i.e. time increments every 10us; 1 time = 10 us
	time++;
}