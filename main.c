/*

	Check fuses:

	>avrdude -c usbtiny -p ATMega168 -P usb -U lfuse:r:-:h

	Factory default for ATMega168:

		lfuse = 0x62
		hfuse = 0xdf
		efuse = 0x01

	Wrote new lfuse to use the 14.745600 MHz crystal:
		>avrdude -c usbtiny -p ATMEga168 -P usb -U lfuse:w:0xFF:m
		lfuse - 0xFF

*/

#include <stdio.h>
#include <stdlib.h>				// atoi()
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#define LEDPIN		PB2			// LED indicator

#define TACHA		PB0			// Tach signal from MotorA
#define TACHB		PB1			// Tach signal from MotorB

#define DIRA		PC0			// MotorA direction signal
#define DIRB		PC1			// MotorB direction signal
#define ENABLEPINA	PC2			// Enable motion on A when HIGH
#define ENABLEPINB	PC3			// Enable motion on B when HIGH

#define HOMEA		PD2			// Home position sensor
#define HOMEB		PD3			// Home position sensor
#define MOTORBPWM	PD5			// MotorB PWM signal
#define MOTORAPWM	PD6			// MotorA PWM signal

#define MOTORA		1
#define MOTORB		2
#define FORWARD		1
#define REVERSE		-1
#define UP			1
#define DOWN		0
#define MINSPEED	52			// Nidec 22H 20% duty cycle min is 52 (but 25 works)
#define MAXSPEED	254

#define TRUE		1
#define FALSE		0

#define speedA(X)	(OCR0A = X)			// PWM on MotorA (PD6)
#define speedB(X)	(OCR0B = X)			// PWM on MotorB (PD5)
#define enable(X)	(PORTC |= _BV(X))	// "X" is ENABLEPINA or ENABLEPINB
#define disable(X)	(PORTC &= ~_BV(X))	// "X" is ENABLEPINA or ENABLEPINB
#define forward(X)	(PORTC |= _BV(X))	// "X" is DIRA or DIRB
#define reverse(X)	(PORTC &= ~_BV(X))	// "X" is DIRA or DIRB
#define AisMoving	(PINC & _BV(ENABLEPINA))
#define BisMoving	(PINC & _BV(ENABLEPINB))

#define BAUDRATE 19200
#define MYUBRR ((F_CPU / 16 / BAUDRATE) -1)

#define COMMANDSENT (UCSR0A & _BV(RXC0))	// Is there something in the RX buffer?
#define TXREADY (UCSR0A & _BV(UDRE0))		// Is the transmit buffer empty?


// Function Prototypes
uint16_t getNum(void);
void init_ATMega168(void);
void moveA(uint8_t, int16_t);
//void moveRel(uint8_t, uint8_t, uint8_t, int16_t);
void prtCounter(void);
void rampA(uint8_t, uint8_t, uint8_t);
void rampB(uint8_t, uint8_t, uint8_t);
void rampDownA(void);
void rampDownB(void);
void rampUpA(uint8_t, uint8_t);
void rampUpDown(uint8_t, uint8_t, uint8_t, uint8_t);
void sendPrompt(void);
void sendByte(uint8_t);
void sendCRLF(void);
void sendString(char *);

// Global variables
volatile int16_t counterA, counterB;	// Local counters
volatile uint16_t oldTimeA, oldTimeB;	// For computing motor RPM
volatile uint16_t newTimeA, newTimeB;	// For computing motor RPM
int16_t targetA, targetB;				// Local target value
int8_t incrementA, incrementB;			// Either +1 or -1 (Forward or reverse)

int main(void)
{

	uint8_t i, cmd, newSpeed, motorChoice;
	int16_t presentCount, amount;
	char strBuf[20];

	init_ATMega168();
	sei();

	for (;;) {
		if (AisMoving) {
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{ presentCount = counterA; }
			if (abs(presentCount - targetA) <= 3) {
				rampDownA();
			}			
			
		}
		if (COMMANDSENT) {

			cmd = UDR0;
			sendByte(cmd);		// echo the command

			switch(cmd) {

				case ('\r'):
					break;

				case ('+'):
					i = OCR0A;
					if (i < MINSPEED) {
						i = MINSPEED;
					}
					if (i == 255) {
						newSpeed = 255;
					} else {
						newSpeed = i + 1;;
					}
					speedA(newSpeed);
					i = OCR0B;
					if (i == 255) {
						newSpeed = 255;
					} else {
						newSpeed = i+1;
					}
					speedB(newSpeed);
					sendCRLF();
					sendString("faster");
					break;

				case ('-'):
					i = OCR0A;
					newSpeed = i-1;
					if (newSpeed < MINSPEED) {
						newSpeed = MINSPEED;
					}
					OCR0A = newSpeed;
					i = OCR0B;
					newSpeed = i-1;
					if (newSpeed < MINSPEED) {
						newSpeed = MINSPEED;
					}
					OCR0B = newSpeed;
					sendCRLF();
					sendString("slower");
					break;

				case ('A'):					// Stop all motion
					speedA(MINSPEED);
					speedB(MINSPEED);
					disable(ENABLEPINA);
					disable(ENABLEPINB);
					break;

				case ('f'):					// forward (clockwise)
					forward(DIRA);
					incrementA = 1;
					sendCRLF();
					sendString("forward=CW");
					break;

				case ('g'):
					targetA = 32767;
					enable(ENABLEPINA);
					sendCRLF();
					sendString("go");
					break;

				case ('m'):
					sendString("ove motor ");
					while (!COMMANDSENT) {
						asm("nop");
					}
					motorChoice = UDR0;
					sendByte(motorChoice);
					sendString(" ");
					if (motorChoice == 'a') {
						motorChoice = MOTORA;
					} else if (motorChoice == 'b') {
						motorChoice = MOTORB;
					}
					amount = getNum();
					moveA(FORWARD, amount);
					break;

				case ('M'):
//					moveRel(MOTORA, FORWARD, MINSPEED, 4096);
					break;

				case ('p'):					// Print motion status
					prtCounter();
					break;

				case ('r'):					// reverse (counter-clockwise)
					reverse(DIRA);
					incrementA = -1;
					sendCRLF();
					sendString("reverse=CCW");
					break;

				case ('R'):
					wdt_enable(WDTO_60MS);
					while (1) {};
					break;

				case ('s'):
					disable(ENABLEPINA);
					sendCRLF();
					sendString("stop");
					break;

				case ('t'):
					sendCRLF();
					sendString(itoa(getNum(), strBuf, 10));
					break;

				default:
					sendString("?");
					sendPrompt();
					break;

			}

			sendPrompt();
		}

	}

}


uint16_t getNum()
{

	char strBuf[20];
	uint8_t i = 0;

	for (;;) {
		while (!COMMANDSENT) {
			asm("nop");
		}
		strBuf[i] = UDR0;
		sendByte(strBuf[i]);
		if (strBuf[i] == '\r') {
			strBuf[i] = 0x00;
			sendCRLF();
			break;
		}

		i++;

	}

	return(atoi(strBuf));

}


void moveA(uint8_t direction, int16_t amount)
{

	uint8_t speed;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{ counterA = 0; }
	targetA = amount;

	if (((amount/16) + MINSPEED) > 255) {
		speed = 254;
	} else {
		speed = MINSPEED + (uint8_t)(amount/16);
		if (speed >= 255) {
			speed = 254;
		}
	}

	rampUpA(direction, speed);
	
}

void prtCounter()
{

	char strBuf[20];
	uint16_t count, rpm, newTime, oldTime;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		count = counterA;
		newTime = newTimeA;
		oldTime = oldTimeA;
	}
	rpm = 0;
	sendCRLF();
	sendString("counterA: ");
	sendString(itoa(count, strBuf, 10));
	sendString(" targetA: ");
	sendString(itoa(targetA, strBuf, 10));
	sendString(" speedA: ");
	sendString(itoa(OCR0A, strBuf, 10));
	if (newTime > oldTime) {
		rpm = 9600 / (newTime - oldTime);	// rps
		rpm *= 60;
	}
	sendString(" ");
	sendString(itoa(rpm, strBuf, 10));
	sendString(" RPM");

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		count = counterB;
		newTime = newTimeB;
		oldTime = oldTimeB;
	}
	rpm = 0;
	sendCRLF();
	sendString("counterB: ");
	sendString(itoa(count, strBuf, 10));
	sendString(" targetB: ");
	sendString(itoa(targetB, strBuf, 10));
	sendString(" speedB: ");
	sendString(itoa(OCR0B, strBuf, 10));
	sendString("  ");
	if (newTime > oldTime) {
		rpm = 9600 / (newTime - oldTime);
		rpm *= 60;
	}
	sendString(" ");
	sendString(itoa(rpm, strBuf, 10));
	sendString(" RPM");

}

void rampDownA()
{

	uint8_t i, currentSpeed;

	currentSpeed = OCF0A;

	for (i = currentSpeed; i >= MINSPEED; i--) {
		speedA(i);
		_delay_ms(1);
	}
	disable(ENABLEPINA);
}

void rampDownB()
{

	uint8_t i, currentSpeed;

	currentSpeed = OCF0B;

	for (i = currentSpeed; i >= MINSPEED; i--) {
		speedB(i);
		_delay_ms(1);
	}
	disable(ENABLEPINB);
}

void rampUpA(uint8_t direction, uint8_t speed)
{

	uint8_t i, currentSpeed;

	currentSpeed = OCF0A;

	if (currentSpeed < MINSPEED) {
		currentSpeed = MINSPEED;
	}

	if (direction == FORWARD) {
		incrementA = 1;
		forward(DIRA);
	} else {
		incrementA = -1;
		reverse(DIRA);
	}

	enable(ENABLEPINA);
	for (i = MINSPEED; i <= speed; i++) {
		speedA(i);
		_delay_ms(5);
	}

}

void rampUpB(uint8_t direction, uint8_t speed)
{

	uint8_t i, currentSpeed;

	currentSpeed = OCF0B;

	if (currentSpeed < MINSPEED) {
		currentSpeed = MINSPEED;
	}

	if (direction == FORWARD) {
		incrementB = 1;
		forward(DIRA);
	} else {
		incrementB = -1;
		reverse(DIRA);
	}

	enable(ENABLEPINB);
	for (i = MINSPEED; i <= speed; i++) {
		speedB(i);
		_delay_ms(5);
	}

}

void sendByte(uint8_t x)
{

	while (!TXREADY) {
		asm("nop");
	}
	UDR0 = x;

}

void sendCRLF(void)
{

	sendByte('\r');
	sendByte('\n');

}

void sendPrompt(void)
{

	sendCRLF();
	sendByte('>');

}

void sendString(char str[])
{

	uint8_t i = 0;

	while (str[i]) {
		sendByte(str[i++]);
	}

}

/*
===========================================================================

	UBRR0L and UBRR0H USART Baud Rate Registers. 12 bits, computed from:
	
	UBRR0 = (F_CPU / (16 * Baudrate)) - 1 where "Baudrate" is the common
	understanding.

===========================================================================

	UCSR0B Control STatus Register B
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| RXCIE	| TXCIE	| UDRIE	|  RXEN	|  TXEN	| UCSZ2	|  RXBB	|  TXBB	|
	Set	|	0	|	0	|	0	|	1	|	1	|	0	|	0	|	0	|
	---------------------------------------------------------------------

	RXCIE is RX Complete Interrupt Enable (we won't use this)
	TXCIE is the TX Complete Interrupt ENable
	UDRIE is USART Data Register Empty Interrupt Enable
	RXEN is Receiver Enable
	TXEN is Transmitter Enable
	UCSZ2, combined with UCSZ[1:0] in UCSRC sets the number of data bits
	RXB8 Receive Data Bit 8 (ninth bit of data, if present)
	TXB8 Transmit bit 8 (ninth bit of data)

	UCSRB = 0b00011000

===========================================================================

	UCSR0C Control STatus Register C
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item|UMSEL01|UMSEL00| UPM01	| UPM00	| USBS0	| UCSZ01| UCSZ00| UCPOL0|
	Set	|	0	|	0	|	0	|	0	|	0	|	1	|	1	|	0	|
	---------------------------------------------------------------------

	UMSEL0[1:0] is the USART mode select. We want 0b00, Asynchronous UART
	UPM0[1:0] is Parity mode. 0b00 is disabled, 0b01 reserved, 0b10 Enabled,
	even parity, 0b11 Enabled, odd parity.
	USBS0 is Stop Bit Select (how many stop bits). 0->1-bit, 1->2-bits.
	UCSZ0[1:0] is the rest of the character size description. An 8-bit
	situation says both of these bits should be high.
	UCPOL0 is clock polarity. Changes fall/rising edges.

	UCSR0C = 0b00000110

===========================================================================

	TCCR0A (Timer/Counter Control Register A for Timer0) settings
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| COM0A1| COM0A0| COM0B1| COM0B0|	-	|	-	| WGM01	| WGM00	|
	Set	|	1	|	0	|	1	|	0	|	0	|	0	|	0	|	1	|
	---------------------------------------------------------------------

	For phase-correct PWM with TOP=OCR0A, WGM0[2:0] should be 0b001 (WGM0[2]
	is in TCCR0B). In phase correct PWM mode the timer counts up, then down.

	We're using both compare registers here so COM0B[1:0] is 0b10 and
	COM0A[1:0] is 0b10, which sets OC0A on the way up and clears it on the
	way down ("Clear OC0A on Compare Match").

	TCCR0A = 0b10100001

===========================================================================

	TCCR0B (Timer/Counter Control Register B for Timer0) settings
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| FOC0A	| FOC0B	|	-	|	-	| WGM02	|  CS02	|  CS01	|  CS00	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|
	---------------------------------------------------------------------

	CS0[2:0] are the clock select bits:

		0b000	No clock source (timer stopped)
		0b001	clk/1 (no prescaling)
		0b010	clk/8
		0b011	clk/64
		0b100	clk/256
		0b101	clk/1024
		0b110	External clock on T0 pin, falling edge
		0b111	External clock on T0 pin, rising edge

	The PWM frequency is clk/(N*510) where N is the prescale factor. If
	our nominal CPU clock frequency is 14.745600 MHz, the frequency and
	period for the prescaled timer are are:

		======================================
		CS0[2:0]	Frequency		Period
		--------------------------------------
		0b001		28.913 kHz		 34.6 us
		0b010		 3.614 kHz		276.7 us
		0b011	   452.765 Hz		  2.214 ms
		0b100	   112.941 Hz		  8.854 ms
		0b101	    28.235 Hz		 35.417 ms
		======================================

	The Faulhaber 2622 B-SC motor takes a PWM frequency input in the range
	of 500 Hz to 18 kHz. Let's choose 28.913 kHz (half for phase correct).
	The motor stops turning at duty cycle < 2.0% (5/255) and starts turning
	at duty cycle > 3.0% (8/255).

	For phase-correct PWM, WGM0[2:0] (Wavform Generation Mode) should be
	0b001 if we're using OC0A (setting WGM0[2] to "1" means that OC0A becomes
	TOP and OC0B is the compared register). WGM0[1:0] are in TCCR0A.

	The Force Output Compare (FOC0A and FOC0B bits) must be set to zero (0)
	when using the timer in PWM mode.

	TCCR0B = 0b00000001

===========================================================================

	MCUSR MCU Status Register (what caused the MCU Reset?)
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item|   -	|   -	|	-	|	-	| WDRF	|  BORF	| EXTRF	|  PORF	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	0	|
	---------------------------------------------------------------------

	WDRF: Watchdog Reset Flag is set if a WD Reset occurs. Reset by power
	on or writing zero to the flag.
	
	BORF: Brown-out Reset flag is set if a brown-out Reset happens.
	
	EXTRF: External Reset Flag
	
	PORF: Power-on Reset Flag

	NB: THe WDRF *MUST* be reset or the system will immediately reset and
	loop forever.

===========================================================================

	WDTCSR Watchdog Timer Control Register (128kHz oscillator)
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| WDIF	| WDIE	| WDP3	| WDCE	|  WDE	| WDP2	| WDP1	|  WDP0	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	0	|
	---------------------------------------------------------------------

	WDIF: Watchdog Timeout Interrupt Flag is set when a timeout occurs
	and the WDT is configured for interrupt.
	WDIE: Watchdog Timeout Interrupt Enable.
	WDP3: One of three prescaler bits (see below)
	WDCE: Watchdog Change Enable (must be set to change WDE)
	WDE: Watchdog enable
	WDP[3:0]: Timer prescaler bits

		=========================================
		WDP3	WDP2	WDP1	WDP0	Appx time
		-----------------------------------------
		 0		 0		 0		 0		16 ms
		 0		 0		 0		 1		32 ms
		 0		 0		 1		 0		64 ms
		 0		 0		 1		 1		0.125 s
		 0		 1		 0		 0		0.25 s
		 0		 1		 0		 1		0.5 s
		 0		 1		 1		 0		1.0 s
		 0		 1		 1		 1		2.0 s
		 1		 0		 0		 0		4.0 s
		 1		 0		 0		 1		8.0 s
		=========================================

	Recovering from a WDT reset is a little tricky. The steps:
		1. Clear the MCUSR watchdog reset flag (WDRF).
		2. In one operation, write logic one to both WDCE and WDE.
		3. In the very next line, write a logic 0 to WDE.

===========================================================================

	PCICR Pin Change Interrupt Control Register
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item|   -	|   -	|   -	|   -	|   -	| PCIE2	| PCIE1	| PCIE0	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	1	|
	---------------------------------------------------------------------

	PCIE[2..0] are Pin Change Interrupt Enable bits

	We're using PCINT0 and PCINT1 on pins PB0 and PB1 respectively for
	the tachometer. MotorA is hooked up to PB0, MotorB on PB1.
	
	PCICR = 0b00000011

===========================================================================

	PCMSK0 Pin Change Mask Register 0
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item|PCINT7	|PCINT6	|PCINT5	|PCINT4	|PCINT3	|PCINT2	|PCINT1	|PCINT0	|
	Set	|	0	|	0	|	0	|	0	|	0	|	0	|	1	|	1	|
	---------------------------------------------------------------------

	PCINT[7..0] enable pin change interrupts on the corresponding pin

	We're using PCINT0 and PCINT1 on pins PB0 and PB1 respectively for
	the tachometer. MotorA is hooked up to PB0, MotorB on PB1.
	
	PCMSK0 = 0b00000011

===========================================================================

	TCCR1B Timer/Counter1 control register B (for clock select)
	---------------------------------------------------------------------
	Bit	|	7	|	6	|	5	|	4	|	3	|	2	|	1	|	0	|
	Item| ICNC1	| ICES1	|   -	| WGM13	| WGM12	|  CS12	|  CS11	|  CS10	|
	Set	|	0	|	0	|	0	|	0	|	0	|	1	|	0	|	0	|
	---------------------------------------------------------------------

	ICNC1 - Input capture noise canceler
	ICES1 - Input capture edge select
	WGM13:2 Waveform generation mode
	CS12:0 Clock select

		-----------------------------------------------------------------
		|	CS12  |	CS11  |	CS10  |	Note								|			|
		|----------------------------------------------------------------
		|	  0	  |	  0	  |	  0	  |	No clock source (timer stopped)		|
		|	  0	  |	  0	  |	  1	  |	clk/1 (no prescaling)				|
		|	  0	  |	  1	  |	  0	  |	clk/8								|
		|	  0	  |	  1	  |	  1	  |	clk/64								|
		|	  1	  |	  0	  |	  0	  |	clk/256								|
		|	  1	  |	  0	  |	  1	  |	clk/1024							|
		|	  1	  |	  1	  |	  0	  |	External clock on T1, falling edge	|
		|	  1	  |	  1	  |	  1	  |	External clock on T1, rising edge	|
		-----------------------------------------------------------------

	For the Niden 22H motor with 5300 RPM no-load max speed, the fastest
	pulse rate from the tachometer-1 wire is 6x5300=31,800 per minute or
	530 Hz, or a period of 1.887 ms.  A /256 prescaler gives a 57.6 kHz
	signal or a 17.361 us period. About 1% accuracy. This overflows in
	1.138 seconds.

	TCCR1B = 0b00000100
	TCCR1A and TCCR1C can both be set to 0x00 for normal operation.

===========================================================================

*/


void init_ATMega168()
{

	uint8_t junk, reset = 0;

	// RECOVER FROM A WATCHDOG RESET
	if (MCUSR & _BV(WDRF)) {	// If there was a watchdog reset...
		MCUSR = 0x00;			// Clear the WDRF flag (and everything else)
		WDTCSR = 0x18;			// Both WDCE and WDE
		WDTCSR = 0x00;			// Now clear WDE
		reset = 1;
	}

	// SETUP THE USART
	UBRR0H = (uint8_t) (MYUBRR >> 8);		// Set baud rate
	UBRR0L = (uint8_t) MYUBRR;				// Set baud rate
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);		// Enable transmit & receive
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);	// 8 data bits (UCSZ0[1:0]), no parity, one stop bit

	TCCR0A = 0b10100001;		// Phase-correct PWM on OC0A and OC0B (see above)
	TCCR0B = 0b00000001;		// Select no clock prescaler (see above)
	TCCR1B = 0b00000100;		// Select /256 prescaler for Timer1 (16-bit)

	DDRB |= _BV(LEDPIN);		// Setup the indicator LED
	PORTB |= _BV(LEDPIN);		// Flash the indicator LED
	_delay_ms(500);
	PORTB &= ~_BV(LEDPIN);		// LED off

	DDRB &= ~_BV(TACHA);		// TACHA pin is input
	DDRB &= ~_BV(TACHB);		// TACHB pin is input
	PORTB |= _BV(TACHA);		// Pullup TACHA pin
	PORTB |= _BV(TACHB);		// Pullup TACHB pin
	// TACH PIN INTERRUPTS - we're only using one pin on each of PB0 and PB1
	PCICR = 0b00000011;			// Enable interrupts on PCINT0 and PCINT1
	PCMSK0 = 0b00000011;		// Enable interrupts on PB0 and PB1 (TACHA and TACHB)

	DDRC |= _BV(DIRA);			// Direction bit for motorA is output
	DDRC |= _BV(DIRB);			// Direction bit for motorA is output
	PORTC |= _BV(DIRA);			// Set the direction bit for motorA
	PORTC |= _BV(DIRB);			// Set the direction bit for motorB

	DDRC |= _BV(ENABLEPINA);	// Enable pin for motorA is output
	DDRC |= _BV(ENABLEPINB);	// Enable pin for motorB is output
	PORTC &= ~_BV(ENABLEPINA);	// Set enable pin for motorA off
	PORTC &= ~_BV(ENABLEPINB);	// Set enable pin for motorB off

	// need to enable INT0 and INT1 when we get the sensors
	DDRD &= ~_BV(HOMEA);		// HOMEA pin is input
	DDRD &= ~_BV(HOMEB);		// HOMEB pin is input

	DDRD |= _BV(MOTORAPWM);		// MOTORA PWM pin is output
	DDRD |= _BV(MOTORBPWM);		// MOTORB PWM pin is output
	speedA(MINSPEED);			// Start PWM for motorA
	speedB(MINSPEED);			// Start PWM for motorB

	while (COMMANDSENT) {		// Clear the serial port
		junk = UDR0;
	}

	counterA = 0;
	counterB = 0;
	newTimeA = 0;
	newTimeB = 0;
	oldTimeA = 0;
	oldTimeB = 0;
	forward(DIRA);
	forward(DIRB);
	incrementA = 1;
	incrementB = 1;

	sendCRLF();

	if (reset) {
		sendString("Reset");
	} else {
		sendString("PwrUP");
	}

	sendPrompt();

}

ISR(PCINT0_vect)
{

	if (PINB & _BV(PINB0)) {	// Rising edge (LOW to HIGH) pin change
		counterA += incrementA;
		oldTimeA = newTimeA;
		newTimeA = TCNT1;
		if ((counterA % 24) == 0) {
			PORTB ^= _BV(LEDPIN);
		}
	}

}
