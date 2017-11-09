#include "WH2Sensor.h"
#include "RCSwitchOregon.h"
#include "OneWireSlave.h"

#define PIN_ONE_WIRE 3
OneWireSlave ds(PIN_ONE_WIRE);
unsigned char rom[8] = { 0x01, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x00, 0xFF }; 

#define SENSORS_NUM 2
struct SSensorData {
	char buf_data[8] = {0,0,0,0,0,0,0,0};
	bool received = false;
	unsigned long prev_t = 0;
} sensor_data[SENSORS_NUM];

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

	if (WH2TimerDecoder::isTimeout() /*wh2_timeout > 0*/) {
		WH2TimerDecoder::incTimeout(); //wh2_timeout++;
		if (WH2TimerDecoder::checkTimeout() /*HAS_TIMED_OUT(wh2_timeout)*/) {
			WH2TimerDecoder::resetPacketState(); // wh2_packet_state = 0;
			WH2TimerDecoder::resetTimeout(); // wh2_timeout = 0;
		}
	}
}

WH2TimerDecoder wh2;

OregonDecoderV3 orscV3; // Oregon Scientific V3 sensor

RCSwitch rcs; //RCswitch sensor

unsigned int timings[RC_MAX_PULSE_BUFFER];

void oneWireHandler() {
	uint8_t cmd;

	ds.waitForRequest(false);
	for (;;) {
		cmd = ds.recv();
		if (cmd == 0xA1) {
			ds.sendData(/*test_data */sensor_data[0].buf_data, 8);
			sensor_data[0].received = false;
		}
		else if (cmd == 0xA2) {
			ds.sendData(/*test_data*/sensor_data[1].buf_data, 8);
			sensor_data[1].received = false;
		}
		if (s1 && s2) {
			wh2.startTimerHandler();
			break;
		}
	}
	Serial.print("^");
}

void fillSensorData(uint8_t idx) {
	int d = wh2.sensor_id();
	sensor_data[idx].received = true;
	sensor_data[idx].buf_data[0] = highByte(d);
	sensor_data[idx].buf_data[1] = lowByte(d);
	sensor_data[idx].prev_t = millis() - sensor_data[idx].prev_t;
	sensor_data[idx].buf_data[2] = highByte(int(sensor_data[idx].prev_t / 1000));
	sensor_data[idx].buf_data[3] = lowByte(int(sensor_data[idx].prev_t / 1000));
	d = wh2.temperature();
	sensor_data[idx].buf_data[4] = highByte(d);
	sensor_data[idx].buf_data[5] = lowByte(d);
	sensor_data[idx].buf_data[6] = wh2.humidity();
	sensor_data[idx].buf_data[7] = OneWireSlave::crc8(sensor_data[0].buf_data, 7);
	for (int i = 0; i < 8; i++) {
		Serial.print((int)(sensor_data[0].buf_data[i]));
		Serial.print(".");
	}
	Serial.println();
}

void setup() {
	Serial.begin(9600);
	Serial.println("\n[ookDecoder]");

	pinMode(2, INPUT);
	rcs.enableReceive(0);

	wh2.init();

	ds.setRom(rom); 

	sensor_data[0].buf_data[0] = sensor_data[0].buf_data[1] = sensor_data[1].buf_data[0] = sensor_data[1].buf_data[1] = 0;
}
// id id t t tm tm vl

void loop() {
	if (wh2.getSensorData()) {
		wh2.stopTimerHandler();
		bool isReceivedComplete = false;
		if (sensor_data[0].buf_data[0] << 8 | sensor_data[0].buf_data[1] == wh2.sensor_id()) {
			if (sensor_data[0].received) isReceivedComplete = true;
			fillSensorData(0);
		}
		else
			if (sensor_data[1].buf_data[0] << 8 | sensor_data[1].buf_data[1] == wh2.sensor_id()) {
				if (sensor_data[1].received) isReceivedComplete = true;
				fillSensorData(1);
			}
			else {
				if (!sensor_data[0].received) fillSensorData(0); else fillSensorData(1);
			}


		int id = wh2.sensor_id();
		
		s1 = s2 = false;
		oneWireHandler();
	}
}

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
	return (calculated_crc == packet[4]);
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
