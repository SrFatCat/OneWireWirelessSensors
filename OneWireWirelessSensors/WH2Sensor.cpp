/*
Name:		WH2Sensor.cpp
Created:	01.11.2017
Author:	Luc Small modifyed Alexey Bogdan aka Sr.FatCat

Based on BetterWH2 https://github.com/lucsmall/BetterWH2
An improved Arduino sketch for decoding packets from the WH2 outdoor temperature and humidity
sensor from Fine Offset Electronics.
Created Luc Small on 19 July 2013. Released into the public domain.
This code contains a CRC - 8 function adapted
from the Arduino OneWire library : http://www.pjrc.com/teensy/td_libs_OneWire.html Thanks go to the authors of that project.

*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "WH2Sensor.h"

//WH2 Timer Handler
ISR(TIMER1_COMPA_vect)
{
	static byte sampling_state = 0;
	static byte count;
	static boolean was_low = false;

	switch (sampling_state) {
	case 0: // waiting
		WH2TimerDecoder::resetPacketState(); // packet_state = 0;
		if (RF_HI) {
			if (was_low) {
				count = 0;
				sampling_state = 1;
				was_low = false;
			}
		}
		else {
			was_low = true;
		}
		break;
	case 1: // acquiring first pulse
		count++;
		// end of first pulse
		if (RF_LOW) {
			if (IS_HI_PULSE(count)) {
				WH2TimerDecoder::setFlags(GOT_PULSE | LOGIC_HI); // wh2_flags = GOT_PULSE | LOGIC_HI;
				sampling_state = 2;
				count = 0;
			}
			else if (IS_LOW_PULSE(count)) {
				WH2TimerDecoder::setFlags(GOT_PULSE);// wh2_flags = GOT_PULSE; // logic low
				sampling_state = 2;
				count = 0;
			}
			else {
				sampling_state = 0;
			}
		}
		break;
	case 2: // observe 1ms of idle time
		count++;
		if (RF_HI) {
			if (IDLE_HAS_TIMED_OUT(count)) {
				sampling_state = 0;
			}
			else if (IDLE_PERIOD_DONE(count)) {
				sampling_state = 1;
				count = 0;
			}
		}
		break;
	}

	if (WH2TimerDecoder::isTimeout()) {
		WH2TimerDecoder::incTimeout();
		if (WH2TimerDecoder::checkTimeout()) {
			WH2TimerDecoder::resetPacketState();
			WH2TimerDecoder::resetTimeout();
		}
	}
}


volatile byte WH2TimerDecoder::flags = 0;
volatile byte WH2TimerDecoder::packet_state = 0;
volatile int WH2TimerDecoder::timeout = 0;

bool WH2TimerDecoder::getSensorData() {
	bool retrn = false;
	if (flags) {
		if (accept()) {
			// calculate the CRC
			calculate_crc();
			if (valid()) {
				retrn = true;
			}

		}
		flags = 0x00;
	}
	return retrn;
}

// processes new pulse
boolean  WH2TimerDecoder::accept()
{
	static byte packet_no, bit_no, history;

	// reset if in initial wh2_packet_state
	if (packet_state == 0) {
		// should history be 0, does it matter?
		history = 0xFF;
		packet_state = 1;
		// enable wh2_timeout
		timeout = 1;
	} // fall thru to wh2_packet_state one

	  // acquire preamble
	if (packet_state == 1) {
		// shift history right and store new value
		history <<= 1;
		// store a 1 if required (right shift along will store a 0)
		if (flags & LOGIC_HI) {
			history |= 0x01;
		}
		// check if we have a valid start of frame
		// xxxxx110
		if ((history & B00000111) == B00000110) {
			// need to clear packet, and counters
			packet_no = 0;
			// start at 1 becuase only need to acquire 7 bits for first packet byte.
			bit_no = 1;
			packet[0] = packet[1] = packet[2] = packet[3] = packet[4] = 0;
			// we've acquired the preamble
			packet_state = 2;
		}
		return false;
	}
	// acquire packet
	if (packet_state == 2) {

		packet[packet_no] <<= 1;
		if (flags & LOGIC_HI) {
			packet[packet_no] |= 0x01;
		}

		bit_no++;
		if (bit_no > 7) {
			bit_no = 0;
			packet_no++;
		}

		if (packet_no > 4) {
			// start the sampling process from scratch
			packet_state = 0;
			// clear wh2_timeout
			timeout = 0;
			return true;
		}
	}
	return false;
}

void  WH2TimerDecoder::calculate_crc()
{
	//wh2_calculated_crc = crc8(wh2_packet, 4);
	uint8_t len = 4;
	uint8_t crc = 0;
	uint8_t* addr = packet;

	// Indicated changes are from reference CRC-8 function in OneWire library
	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x80; // changed from & 0x01
			crc <<= 1; // changed from right shift
			if (mix) crc ^= 0x31;// changed from 0x8C;
			inbyte <<= 1; // changed from right shift
		}
	}
	calculated_crc = crc;

}

bool  WH2TimerDecoder::valid()
{
	bool retn = (calculated_crc == packet[4]);
	return retn && temperature() <600 && temperature() > -600 && humidity() >= 0 && humidity() <= 100;
}

int  WH2TimerDecoder::sensor_id()
{
	return (packet[0] << 4) + (packet[1] >> 4);
}

byte  WH2TimerDecoder::humidity()
{
	return packet[3];
}

int  WH2TimerDecoder::temperature()
{
	int temperature;
	temperature = ((packet[1] & B00000111) << 8) + packet[2];
	// make negative
	if (packet[1] & B00001000) {
		temperature = -temperature;
	}
	return temperature;
}

void WH2TimerDecoder::init() {
	TCCR1A = 0x00;
	TCCR1B = 0x09;
	TCCR1C = 0x00;
	OCR1A = COUNTER_RATE;
	startTimerHandler();
	sei();
}
