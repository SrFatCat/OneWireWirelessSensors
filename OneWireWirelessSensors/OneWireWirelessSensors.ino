#include "WH2Sensor.h"
#include "RCSwitchOregon.h"
#include "OneWireSlave.h"

#define PIN_ONE_WIRE 3
OneWireSlave ds(PIN_ONE_WIRE);
unsigned char rom[8] = { 0x01, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x00, 0xFF }; 
			
struct SSensorData {
	uint8_t buf_data[8];
	bool received = false;
	unsigned long prev_t = 0;
} sensor_data[2];

char test_data[8] = { 0,55,0,100,1,255,30 };

bool mustOneWire = false;

//WH2 Timer Handler
ISR(TIMER1_COMPA_vect)
{
	static byte sampling_state = 0;
	static byte count;
	static boolean was_low = false;

	switch (sampling_state) {
	case 0: // waiting
		wh2_packet_state = 0;
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
				wh2_flags = GOT_PULSE | LOGIC_HI;
				sampling_state = 2;
				count = 0;
			}
			else if (IS_LOW_PULSE(count)) {
				wh2_flags = GOT_PULSE; // logic low
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

	if (wh2_timeout > 0) {
		wh2_timeout++;
		if (HAS_TIMED_OUT(wh2_timeout)) {
			wh2_packet_state = 0;
			wh2_timeout = 0;
		}
	}
}
WH2TimerDecoder wh2;

OregonDecoderV3 orscV3; // Oregon Scientific V3 sensor

RCSwitch rcs; //RCswitch sensor

unsigned int timings[RC_MAX_PULSE_BUFFER];

static unsigned long tttt = 0;
bool s1 = false, s2 = false;

void oneWireHandler() {
	uint8_t cmd;

	ds.waitForRequest(false);
	for (;;) {
		cmd = ds.recv();
		if (cmd == 0xA1) {
			ds.sendData(test_data /*sensor_data[0].buf_data*/, 8);
			s1 = true;
		}
		else if (cmd == 0xA2) {
			ds.sendData(test_data/*sensor_data[1].buf_data*/, 8);
			s2 = true;
		}
		if (s1 && s2) {
			//mustOneWire = false;
			wh2.startTimerHandler();
			break;
		}
	}

	//if (millis() - tttt > 2000) mustOneWire = false;
	//if (ds.waitReset(0) && ds.presence(25)) {
	//	bool isBreak = false;
	//	char addr[8];
	//	for (;;) {
	//		switch (ds.recv()) {
	//		case 0xF0: // SEARCH ROM
	//			ds.search();
	//			Serial.println("SEARCH ROM");
	//			isBreak = true;
	//			break;
	//		case 0x33: // READ ROM
	//			ds.sendData((char *)rom, 8);
	//			Serial.println("READ ROM");
	//			isBreak = true;
	//			break;
	//		case 0x55: // MATCH ROM
	//			ds.recvData(addr, 8);
	//			cmd = ds.recv();
	//			if (s1 = (cmd == 0xA1)) ds.sendData(test_data /*sensor_data[0].buf_data*/, 8);
	//			else if (s2 = (cmd == 0xA2)) ds.sendData(test_data/*sensor_data[1].buf_data*/, 8);
	//			if (s1 && s2) {
	//				mustOneWire = false;
	//				wh2.startTimerHandler();
	//			}
	//			//if (errno != ONEWIRE_NO_ERROR)
	//			//	return FALSE;
	//			//ds.recvData(buf, sizeof(buf));
	//			//for (int i = 0; i < 8; i++) {
	//			//	Serial.print(buf[i]); Serial.print(" ");
	//			//}
	//			//for (int i = 0; i < 5; i++) {
	//			//	ds.send(buf1[i]);
	//			//}
	//			//	if (rom[i] != addr[i])
	//			//		return FALSE;
	//			//return TRUE;
	//			//Serial.println("MATCH ROM");
	//			isBreak = true;
	//			break;
	//		case 0xCC: // SKIP ROM
	//				   //ds.recvData(buf, sizeof(buf)); //��������� ������
	//				   //Serial.print("I: ");
	//				   //for (int i = sizeof(buf) - 1; i >= 0; i--) {
	//				   //	//Serial.print((int)(buf[i]), HEX);
	//				   //	//Serial.print(" ");
	//				   //	ds.send(buf1[i]); //�������� ��������
	//				   //}
	//				   //Serial.println("SKIP ROM");
	//			isBreak = true;
	//			break;
	//		default: // Unknow command
	//				 //if (errno == ONEWIRE_NO_ERROR)
	//				 //	break; // skip if no error
	//				 //else
	//				 //	return FALSE;
	//			break;
	//		}
	//		if (isBreak) break;
	//	}
}

void setup() {
	Serial.begin(9600);
	Serial.println("\n[ookDecoder]");

	pinMode(2, INPUT);
	rcs.enableReceive(0);

	wh2.init();
	test_data[7] = OneWireSlave::crc8((char *)test_data, 7);
	ds.setRom(rom); 
}
// id id t t tm tm vl

void loop() {

	if (mustOneWire) {
		oneWireHandler();
	}
	else {
		if (processTimerWH2Handler()) {
			//int d = wh2.sensor_id();
			//sensor_data[0].received = true;
			//sensor_data[0].buf_data[0] = highByte(d);
			//sensor_data[0].buf_data[1] = lowByte(d);
			//sensor_data[0].prev_t = millis() - sensor_data[0].prev_t;
			//sensor_data[0].buf_data[2] = highByte(int(sensor_data[0].prev_t/1000));
			//sensor_data[0].buf_data[3] = lowByte(int(sensor_data[0].prev_t / 1000));
			//d = wh2.temperature();
			//sensor_data[0].buf_data[4] = highByte(d);
			//sensor_data[0].buf_data[5] = lowByte(d);
			//sensor_data[0].buf_data[6] = wh2.humidity();
			//sensor_data[0].buf_data[7] = OneWireSlave::crc8(sensor_data[0].buf_data, 7);
			mustOneWire = true;
			tttt = millis();
			s1 = s2 = false;
			wh2.stopTimerHandler();
		}
	}
}

bool processTimerWH2Handler() {
	static uint16_t hits = 0;
	static unsigned long old = 0, packet_count = 0, bad_count = 0, average_interval;
	unsigned long spacing, now;
	bool ret = false;

	if (wh2_flags) {
		if (wh2.accept()) {
			// calculate the CRC
			wh2.calculate_crc();

			now = millis();
			spacing = now - old;
			old = now;
			packet_count++;
			average_interval = now / packet_count;
			if (!wh2.valid()) {
				bad_count++;
			}
			else {
				//Serial.print(packet_count, DEC);
				//Serial.print(" | ");
				//Serial.print(bad_count, DEC);
				//Serial.print(" | ");
				//Serial.print(spacing, DEC);
				//Serial.print(" | ");
				//Serial.print(average_interval, DEC);
				//Serial.print(" | ");

		/*		if (true || wh2_packet[0] == 73) {
					Serial.print("{" + String(++hits) + " " + String(millis() / 1000l) + "} ");

					for (i = 0; i < 5; i++) {
						Serial.print(wh2_packet[i], BIN);
						Serial.print(" ");
					}
					Serial.print(" / ");
					for (i = 0; i < 5; i++) {
						Serial.print(wh2_packet[i], DEC);
						Serial.print(" ");
					}
					Serial.println();
				}*/
				//for (i = 0; i<5; i++) {
				//	Serial.print("0x");
				//	Serial.print(wh2_packet[i], HEX);
				//	Serial.print("/");
				//	Serial.print(wh2_packet[i], DEC);
				//	Serial.print(" ");
				//}
				Serial.print("| Sensor ID: 0x");
				Serial.print(wh2.sensor_id(), HEX);
				Serial.print(" | ");
				Serial.print(wh2.humidity(), DEC);
				Serial.print("% | ");
				Serial.println(wh2.temperature(), DEC);
				//Serial.print(" | ");
				//Serial.println((wh2_valid() ? "OK" : "BAD"));
				ret = true;
			}
		}
		wh2_flags = 0x00;
	}
	return ret;
}

// processes new pulse
boolean  WH2TimerDecoder::accept()
{
	static byte packet_no, bit_no, history;

	// reset if in initial wh2_packet_state
	if (wh2_packet_state == 0) {
		// should history be 0, does it matter?
		history = 0xFF;
		wh2_packet_state = 1;
		// enable wh2_timeout
		wh2_timeout = 1;
	} // fall thru to wh2_packet_state one

	  // acquire preamble
	if (wh2_packet_state == 1) {
		// shift history right and store new value
		history <<= 1;
		// store a 1 if required (right shift along will store a 0)
		if (wh2_flags & LOGIC_HI) {
			history |= 0x01;
		}
		// check if we have a valid start of frame
		// xxxxx110
		if ((history & B00000111) == B00000110) {
			// need to clear packet, and counters
			packet_no = 0;
			// start at 1 becuase only need to acquire 7 bits for first packet byte.
			bit_no = 1;
			wh2_packet[0] = wh2_packet[1] = wh2_packet[2] = wh2_packet[3] = wh2_packet[4] = 0;
			// we've acquired the preamble
			wh2_packet_state = 2;
		}
		return false;
	}
	// acquire packet
	if (wh2_packet_state == 2) {

		wh2_packet[packet_no] <<= 1;
		if (wh2_flags & LOGIC_HI) {
			wh2_packet[packet_no] |= 0x01;
		}

		bit_no++;
		if (bit_no > 7) {
			bit_no = 0;
			packet_no++;
		}

		if (packet_no > 4) {
			// start the sampling process from scratch
			wh2_packet_state = 0;
			// clear wh2_timeout
			wh2_timeout = 0;
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
	uint8_t* addr = wh2_packet;

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
	wh2_calculated_crc = crc;

}

bool  WH2TimerDecoder::valid()
{
	return (wh2_calculated_crc == wh2_packet[4]);
}

int  WH2TimerDecoder::sensor_id()
{
	return (wh2_packet[0] << 4) + (wh2_packet[1] >> 4);
}

byte  WH2TimerDecoder::humidity()
{
	return wh2_packet[3];
}

int  WH2TimerDecoder::temperature()
{
	int temperature;
	temperature = ((wh2_packet[1] & B00000111) << 8) + wh2_packet[2];
	// make negative
	if (wh2_packet[1] & B00001000) {
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
