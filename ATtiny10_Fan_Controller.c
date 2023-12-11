/*
 ATtiny10_Fan_Controller.ino - Fan controller based on Dallas Semiconductors DS18B20 digital temperature sensor and ATtiny10 microcontroller

 MIT License

 Copyright (c) 2022 Dmitry Muravyev (youtube.com/@DmitryMuravyev)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constants and Types
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

const uint16_t PWM_top = 400;					// PWM Top for 25 KHz frequency: 10 MHz / 25 KHz = 400
const int16_t f_min_rps_diff = 25;				// 25 pulses = 12.5 RPS = 750 RPM - the minimum difference between 100% and 0% PWM Duty to detect if PWM and Hall sensor are available
const int16_t f_min_rps = 4;					// 4 pulses = 2 RPS = 120 RPM - the minimum RPM to check if Fan runs
const int8_t f_max_restarts = 10;				// The maximum number of attempts to start the Fan, after which we switch to power management mode (On-Off)
const int16_t f_acc_dec_time = 4000;			// 4s for Fan acceleration/deceleration (this can be increased for larger Fans)
const int16_t f_measurement_time = 1000;		// 1s for RPS measurement

// We get from the DS18B20 sensor the temperature measured in 1/16 of degree (Celsius). For greater accuracy and less code, we will use integer values.
const int16_t t_lower_threshold = 400;			// 25C (25 x 16 = 400) - below this temperature the Fan is Off or runs at minimum speed (0%)
const int16_t t_upper_threshold = 640;			// 40C (40 x 16 = 640) - above this temperature the Fan is On or runs at maximum speed (100%)
const int16_t t_no_pwm_trigger = 480;			// 30C (30 x 16 = 480) - switch-on threshold, used in the power management mode (On-Off)
const int16_t t_max_common_divisor = 80;		// Maximum common divisor for PWM_top and (t_upper_threshold - t_lower_threshold) = 240. For our case it is 80.

const uint8_t one_wire_pin = 1;					// DS18B20 sensor data pin 
const uint8_t skip_ROM_command = 0xCC;			// Skip DS18B20 SN command
const uint8_t convert_command = 0x44;			// Start temperature conversion command
const uint8_t read_scratchpad_command = 0xBE;	// Read Scratchpad command


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

static uint8_t fanControlMode = 0;				// Control mode (PWM / On-Off) + counter of Fan start attempts. The default mode is power management (On-Off).

#define ResetResult(x)		(x)[2]				// Use the 3rd byte to temporarily store the results of resetting OneWire 
#define ConfigReg(x)		(x)[4]				// DS18B20 configuration register

static union {
	uint8_t dataBytes[9];						// Buffer for DS18B20 sensor data
	int16_t temperature;						// Temperature bytes
	volatile int16_t sensorPulses;				// Hall sensor pulses counter
};


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt handlers
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// INT0 HW interrupt handler
/*
ISR(INT0_vect) {
	sensorPulses++;
}
*/

// The compiler generates a several unnecessary push/pop, so let's write our own handler
ISR(INT0_vect, ISR_NAKED) {
	__asm__ __volatile__ (
		"push	r20						\n"
		"in		r20, __SREG__			\n"
		"push	r20						\n"
		"push	r21						\n"

		"lds	r20, %[counter]			\n"
		"lds	r21, %[counter]+1		\n"
		"subi	r20,0xFF				\n"
		"sbci	r21,0xFF				\n"
		"sts	%[counter], r20			\n"
		"sts	%[counter]+1, r21		\n"

		"pop	r21						\n"
		"pop	r20						\n"
		"out	__SREG__,r20			\n"
		"pop	r20						\n"
		"reti							\n"
		: 
		: [counter] "m" (sensorPulses)
		: //"r20", "r21"				// We will take care of the registers ourselves
	);
}

// We can also use PCINT0 interrupt (Pin Change), but it triggers on any change, and the INT0 can be configured to trigger on the rising edge of the pulse
/*
ISR(PCINT0_vect) {
	sensorPulses++;
}
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Initialization
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

void setup() {

// Set Clock
	OSCCAL = 214;						// 0-255 Oscillator Frequency calibration value: 0 for ~3055KHz, 255 for ~11.9MHz, 214 for ~10MHz, default ~7789KHz
	CCP = 0xD8;							// Enable CCP
	//CLKMSR = 0;						// 0 for Internal 8MHz Oscillator - default value
	CLKPSR = 0;							// Clock prescaler = 1

// Set PWM output
	TCCR0A = _BV(COM0A1) | _BV(WGM01); 					// Channel_A Fast PWM (mode 14) non-inverting (clear OC0A on match, set at bottom)
	TCCR0B = _BV(WGM03) | _BV(WGM02) | _BV(CS00);		// Fast PWM (mode 14), PWM prescaler = 1
	ICR0 = PWM_top;						// Top value for 25 KHz
	
// Configure pins
	DDRB = 0b1011;						// Configure PB0, PB1, PB3 as OUTPUTs (*** don't forget to set RSTDISBL fuse ***), PB2 as INPUT
	PUEB = 0b0000; 						// No pull-up
	PORTB = 0b1000;						// Turn on DC/DC converter, other pins to 0

// Configure interrupts
	EICRA = (1 << ISC01) | (1 << ISC00);				// The rising edge of INT0 generates an interrupt request
	EIMSK = 1 << EIMSK;									// Enable hardware INT0 interrupts

// We can also use PCINT0 interrupt (Pin Change), but it triggers on any change, and the INT0 can be configured to trigger on the rising edge of the pulse
	//PCMSK = 0b0100;					// Set PCINT pins
	//PCICR = 1<<PCIE0; 				// Enable Pin Change interrupts

// Detect if PWM and Hall sensor are available
	OCR0A = PWM_top; 					// PWM 100%
	delayMillis(f_acc_dec_time);		// Fan acceleration
	measureRPS();
	int16_t highRPS = sensorPulses;
	OCR0A = 0;							// PWM 0%
	delayMillis(f_acc_dec_time);		// 4s for deceleration (this can be increased for larger Fans)
	measureRPS();

	if ((highRPS - sensorPulses) > f_min_rps_diff) {
		fanControlMode = 1;				// PWM control mode
	}

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Delay functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Delay for microseconds
// Since the only timer is occupied by PWM - we cannot use it directly, so we will use workarounds.
// This one is based on processor cycles. It's more accurate and takes less memory.
void delayMicros(uint16_t microseconds) {

// We don't need a case of one microsecond in this project
/*
	if (microseconds == 1) {
		__asm__ __volatile__ (
			"dec	%A[counter]			\n"				// 1 cycle ('dec' added to avoid compiler optimization and for future needs)
			"nop						\n"				// 1 cycle
			"nop						\n"				// 1 cycle
			"nop						\n"				// 1 cycle
			:
			// Output operands
			:
			// Input operands
			[counter] "r" (microseconds)
			:
			// Clobbers
		);
		return;
	}
*/

// And a case of two microseconds
/*
	if (microseconds == 2) {
		microseconds--;					// Correction for function initialization time
	}
*/

// Small cycle for less than 10 microseconds
	if (microseconds < 10) {
		__asm__ __volatile__ (
			"1: nop						\n"				// 1 cycle
			"nop						\n"				// 1 cycle
			"nop						\n"				// 1 cycle
			"nop						\n"				// 1 cycle
			"dec	%A[counter]			\n"				// 1 cycle
			"brne	1b					\n"				// 2 cycles
			:
			// Output operands
			:
			// Input operands
			[counter] "r" (microseconds)
			:
			// Clobbers
		);
		return;
	}

	microseconds -= 3;					// Correction for function initialization time (you can play with this parameter to get the best results over the entire range)

// Large cycle for large delays
	__asm__ __volatile__ (
		"1: nop							\n"				// 1 cycle
		"nop							\n"				// 1 cycle
		"nop							\n"				// 1 cycle
		"nop							\n"				// 1 cycle
		"nop							\n"				// 1 cycle
		"nop							\n"				// 1 cycle
		"subi	%A[counter], 1			\n"				// 1 cycle
		"sbci	%B[counter], 0			\n"				// 1 cycle
		"brcc	1b						\n"				// 2 cycles

		//
		// We have 10 CPU cycles in total, so DELAY = 10MHz / (microseconds * 10 cycles)
		//
		
		:
		// Output operands
		:
		// Input operands
		[counter] "r" (microseconds)
		:
		// Clobbers
	);
}


// This one is based on processor cycles + Timer0. It's less accurate and takes more memory, but I'll leave it here as another example of delay implementation.
/*
void delayMicros(uint16_t microseconds) {

// Small cycle for less than 10 microseconds
	if (microseconds < 10) {
		__asm__ __volatile__ (
			"1: nop						\n"				// 1 cycle
			"nop						\n"				// 1 cycle
			"nop						\n"				// 1 cycle
			"nop						\n"				// 1 cycle
			"dec	%A[counter]			\n"				// 1 cycle
			"brne	1b					\n"				// 2 cycles
			:
			// Output operands
			:
			// Input operands
			[counter] "r" (microseconds)
			:
			// Clobbers
		);
		return;
	}

// Large cycle for large delays
	int16_t lastTimerValue = TCNT0;
	uint16_t microX10 = microseconds * 10 - 35;			// The number of CPU cycles minus correction for function initialization time
	
	while (microX10 > 0) {
		int16_t currentTimerValue = TCNT0;
		int16_t diff = currentTimerValue - lastTimerValue;
		if (diff < 0) diff += PWM_top;					// Detect if TCNT0 passed the PWM_top since the last iteration 
		if (diff > microX10) {
			break;
		} else {
			microX10 -= diff;
		}
		lastTimerValue = currentTimerValue;
	}

}
*/


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Delay for milliseconds
void delayMillis(uint16_t milliseconds) {
	while (--milliseconds) delayMicros(1000);
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// OneWire functions, based on this project: http://www.technoblogy.com/show?2G8A (Licensed under a Creative Commons Attribution 4.0 International license: http://creativecommons.org/licenses/by/4.0/ )
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Configure OneWire pin as OUTPUT (low level was pre-configured in setup())
inline void pinLow() {
	DDRB = DDRB | (1 << one_wire_pin);
}

// Configure OneWire pin as INPUT
inline void pinRelease() {
	DDRB = DDRB & ~(1 << one_wire_pin);
}

// Read OneWire pin
inline uint8_t pinRead() {
	return ((PINB >> one_wire_pin) & 1);
}

// Transmit inverted 1-0 sequence
void pinLowRelease(uint16_t low, uint16_t high) {
	pinLow();
	delayMicros(low);
	pinRelease();
	delayMicros(high);
}

// Initialize OneWire
void oneWireReset() {
	pinLowRelease(480, 70);
	ResetResult(dataBytes) = pinRead();
	delayMicros(410);
}

// Write byte
void oneWireWrite(uint8_t data) {
	uint16_t low, high;
	for (uint8_t i = 0; i < 8; i++) {
		if ((data & 1) == 1) {
			low = 10;
			high = 55;
		} else {
			low = 65;
			high = 5;
		}
		pinLowRelease(low, high);
		data = data >> 1;
	}
}

// Read byte
uint8_t oneWireRead() {
	uint8_t data = 0;
	for (uint8_t i = 0; i < 8; i++) {
		pinLowRelease(3, 10);
		data = data | (pinRead() << i);
		delayMicros(53);
	}
	return data;
}

// Read bytes into the buffer
void oneWireReadBytes(uint8_t bytes) {
	for (uint8_t i = 0; i < bytes; i++) {
		dataBytes[i] = oneWireRead();
	}
}

// Calculate CRC over the buffer - 0x00 is correct
uint8_t oneWireCRC(uint8_t bytes) {
	uint8_t crc = 0;

	for (uint8_t j = 0; j < bytes; j++) {
		crc = crc ^ dataBytes[j];
		for (uint8_t i = 0; i < 8; i++) crc = (crc >> 1) ^ ((crc & 1) ? 0x8c : 0);
	}

// An alternative variant from the OneWire library ( http://www.pjrc.com/teensy/td_libs_OneWire.html ). 
/*
	uint8_t addr = 0;
	while (bytes--) {
		uint8_t inbyte = dataBytes[addr++];
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
*/

	return crc;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Other functions
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Switch DC/DC On and set PWM
inline void fanOn(uint16_t duty) {
	OCR0A = duty;
	PORTB = PORTB | 0b1000;
}

// Switch DC/DC Off and set PWM to 0%
inline void fanOff() {
	OCR0A = 0;
	PORTB = PORTB & 0b0111;
}

// Measure Fan RPS
void measureRPS () {
	sensorPulses = 0;
	sei();								// Enable interrupts SREG |= (1<<SREG_I)
	delayMillis(f_measurement_time);	// RPS measurement
	cli();								// Disable interrupts
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//

void loop() {

	oneWireReset();										// Reset OneWire

	if (ResetResult(dataBytes) == 0) {
		oneWireWrite(skip_ROM_command);					// We have only one DS18B20 in this project, so skip ROM (SN)
		oneWireWrite(convert_command);					// Start temperature conversion
	}

	//delayMillis(750);									// We can just wait, but it is better to check if the Fan is spinning during the conversion
	measureRPS();										// Measure RPS while we're waiting for the conversion to complete (we will use DS18B20 configured for 12 bit accuracy, it takes 750ms)

// Check RPS only in PWM mode and running Fan
	if ((fanControlMode > 0) && (PORTB & 0b1000)) {
		if (sensorPulses < f_min_rps) {
			fanOn(PWM_top);								// Try to start Fan if it's not spinning
			delayMillis(f_measurement_time);
			if (++fanControlMode > f_max_restarts) fanControlMode = 0;		// Switch to the power management mode (On-Off)
		} else {
			fanControlMode = 1;
		}
	}

// Read temperature
	if (ResetResult(dataBytes) == 0) {
		oneWireReset();
		oneWireWrite(skip_ROM_command);
		oneWireWrite(read_scratchpad_command);			// Read Scratchpad
		oneWireReadBytes(9);
	}

// Check data presence and CRC
	if ((ConfigReg(dataBytes) == 0) || (oneWireCRC(9) != 0)) {				// Check for low level on data pin: lower 5 bits of DS18B20 configuration register (dataBytes[4]) should be set to 1
																			// ...also the 5th register should be set to 0xFF, but my exemplar has the value 0xA5, so we will use the config reg.
		fanOn(PWM_top);					// If the sensor unavailable - set Fan to maximum RPM
		return;
	}

// Check if we can switch off the Fan
	if (temperature < t_lower_threshold) {
		fanOff();						// Too cold
		return;
	} 

// Depending on the control mode (PWM or power On-Off)
	if (fanControlMode > 0) {

		if (temperature >= t_upper_threshold) {
			fanOn(PWM_top);				// Too hot
		} else {

		// Calculate and set the PWM Duty cycle relative to the current temperature.
		// To avoid exceeding of 16 bits we will use minimum coefficients.
		// To get them, let's use the largest common divisor.
		// At compile time, the compiler will replace these expressions with constants:
		//
		//    PWM_top / t_max_common_divisor   =   400 / 80 = 5
		//
		//    (t_upper_threshold - t_lower_threshold) / t_max_common_divisor   =   (640 - 400) / 80 = 3
		//
		// So the final expression will look like this:
		//
		//    (temperature - t_lower_threshold) * 5 / 3
		//

			fanOn((temperature - t_lower_threshold) * (PWM_top / t_max_common_divisor) / ((t_upper_threshold - t_lower_threshold) / t_max_common_divisor));

		}

	} else {

		if (temperature >= t_no_pwm_trigger) {			// Switch-on threshold exceeded 
			fanOn(PWM_top);
		}

	}

}

// END-OF-FILE
