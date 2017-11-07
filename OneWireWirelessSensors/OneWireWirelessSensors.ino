// RF Receiver

#include "RCSwitchOregon.h"


// Read data from 433MHz receiver on digital pin 2
#define RF_IN 2
// For better efficiency, the port is read directly
// the following two lines should be changed appropriately
// if the line above is changed.
#define RF_IN_RAW PIND2
#define RF_IN_PIN PIND

#define COUNTER_RATE 3200-1 // 16,000,000Hz / 3200 = 5000 interrupts per second, ie. 200us between interrupts
// 1 is indicated by 500uS pulse
// wh2_accept from 2 = 400us to 3 = 600us
#define IS_HI_PULSE(interval)   (interval >= 2 && interval <= 3)
// 0 is indicated by ~1500us pulse
// wh2_accept from 7 = 1400us to 8 = 1600us
#define IS_LOW_PULSE(interval)  (interval >= 7 && interval <= 8)
// worst case packet length
// 6 bytes x 8 bits x (1.5 + 1) = 120ms; 120ms = 200us x 600
#define HAS_TIMED_OUT(interval) (interval > 600)
// we expect 1ms of idle time between pulses
// so if our pulse hasn't arrived by 1.2ms, reset the wh2_packet_state machine
// 6 x 200us = 1.2ms
#define IDLE_HAS_TIMED_OUT(interval) (interval > 6)
// our expected pulse should arrive after 1ms
// we'll wh2_accept it if it arrives after
// 4 x 200us = 800us
#define IDLE_PERIOD_DONE(interval) (interval >= 4)
// Shorthand for tests
//#define RF_HI (digitalRead(RF_IN) == HIGH)
//#define RF_LOW (digitalRead(RF_IN) == LOW)
#define RF_HI (bit_is_set(RF_IN_PIN, RF_IN_RAW))
#define RF_LOW (bit_is_clear(RF_IN_PIN, RF_IN_RAW))

#include "OneWireSlave.h"
#define PIN_ONE_WIRE 3
OneWireSlave ds(PIN_ONE_WIRE);
unsigned char rom[8] = { 0x01, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x00, 0xFF }; //����� ����������, 8 ���� �� ���������
																		   //��������� ���� (CRC8 �����) ������� ������ �� �����������, ����� setRom() ������������� ���������� ���.
char buf_data2[8] = { 0,'B','C','4','5', 0, 0, 0 };
char buf_data1[8] = { 0,'B','C','4','5', 0, 0, 0 };



// wh2_flags 
#define GOT_PULSE 0x01
#define LOGIC_HI  0x02
volatile byte wh2_flags = 0;
volatile byte wh2_packet_state = 0;
volatile int wh2_timeout = 0;
byte wh2_packet[5];
byte wh2_calculated_crc;

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

// 433 MHz decoders
class OregonDecoderV2 : public DecodeOOK
{
public:
	OregonDecoderV2() {}

	// add one bit to the packet data buffer
	virtual void gotBit(char value)
	{
		if (!(total_bits & 0x01))
		{
			data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
		}

		total_bits++;
		pos = total_bits >> 4;

		if (pos >= sizeof data)
		{
			resetDecoder();
			return;
		}
		state = OK;
	}

	virtual char decode(word width)
	{
		if (100 <= width && width < 1200)
		{
			byte w = width >= 700;

			switch (state)
			{
			case UNKNOWN:

				if (w != 0)
				{
					// Long pulse
					++flip;
				}
				else if (w == 0 && 5 <= flip)
				{
					// Short pulse, start bit
					flip = 0;
					state = T0;
				}
				else
				{
					// Reset decoder
					return -1;
				}
				break;
			case OK:

				if (w == 0)
				{
					// Short pulse
					state = T0;
				}
				else
				{
					// Long pulse
					manchester(1);
				}
				break;
			case T0:
				if (w == 0)
				{
					// Second short pulse
					manchester(0);
				}
				else
				{
					// Reset decoder
					return -1;
				}
				break;
			}
		}
		else if (width >= 1500 && pos >= 8)
		{
			return 1;
		}
		else
		{
			return -1;
		}
		return 0;
	}
};

OregonDecoderV2 orscV2;
OregonDecoderV3 orscV3;
WH2Decoder wh2;

RCSwitch rcs;

unsigned int timings[RC_MAX_PULSE_BUFFER];

float temperature(const byte* data)
{
	int sign = (data[6] & 0x8) ? -1 : 1;
	float temp = ((data[5] & 0xF0) >> 4) * 10 + (data[5] & 0xF) + (float)(((data[4] & 0xF0) >> 4) / 10.0);
	return sign * temp;
}

byte humidity(const byte* data)
{
	return (data[7] & 0xF) * 10 + ((data[6] & 0xF0) >> 4);
}

float humi_ext(const byte* data)
{
	return  humidity(data) + ((data[7] & 0xF0) >> 4) / 10.0;
}

byte battery(const byte* data)
{
	return (data[4] & 0xF);
}

byte serial(const byte* data)
{
	return (data[3]);
}

byte channel(const byte* data)
{
	byte channel;
	switch (data[2] >> 4)
	{
	case 0x1:
		channel = 1;
		break;
	case 0x2:
		channel = 2;
		break;
	case 0x3:
		channel = 3;
		break;
	case 0x4:
		if ((data[2] & 0xF) == 3) {
			channel = 4;
		}
		else {
			channel = 3;
		}
		break;
	case 0x5:
		channel = 5;
		break;
	case 0x6:
		channel = 6;
		break;
	case 0x7:
		channel = 7;
		break;
	}
	return channel;
}

int Sum(byte count, const byte* data)
{
	int s = 0;

	for (byte i = 0; i<count; i++)
	{
		s += (data[i] & 0xF0) >> 4;
		s += (data[i] & 0xF);
	}

	if (int(count) != count)
		s += (data[count] & 0xF0) >> 4;

	return s;
}


//void reportSerial(const char* s, class DecodeOOK& decoder) {
//	byte pos;
//	const byte* data = decoder.getData(pos);
//
//	String SerialPrint;
//
//	//Serial.print(s);
//	SerialPrint = String(s) + ' ';
//	//Serial.print(' ');
//	for (byte i = 0; i < pos; ++i) {
//		//Serial.print(data[i] >> 4, HEX);
//		//Serial.print(data[i] & 0x0F, HEX);
//		SerialPrint += String(data[i] >> 4, HEX); 
//		SerialPrint += String(data[i] & 0x0F, HEX);
//	}
//
//	if (pos>8) {
//		if (data[8] == (Sum(8, data) - 0xa) & 0xFF) {
//			Serial.print(" ");
//			unsigned long m = millis();
//			if ((m - upd[channel(data)]) > 50000) {
//				Serial.print("!!!!! ");
//			}
//			Serial.print((m - upd[channel(data)]) / 1000., 3);
//			upd[channel(data)] = m;
//			Serial.print(SerialPrint);
//			Serial.print(" " + String(channel(data)) + " " + String(serial(data), HEX));
//			Serial.print(" " + String(temperature(data), 1) + "*C " + String(humidity(data)) + "%");
//			Serial.println(" " + String(battery(data)));
//		}
//		else {
//			//Serial.print(" Checksum error");
//		}
//	}
//	else if (pos>6) {
//		int sum = ((Sum(6, data) + (data[6] & 0xF) - 0xa) & 0xff);
//		if ((sum & 0xF) == (data[6] >> 4) && (sum >> 4) == (data[7] & 0xF)) {
//			Serial.print(SerialPrint);
//			Serial.print(" " + String(channel(data)) + " " + String(serial(data), HEX));
//			Serial.print(" " + String(temperature(data), 1));
//			Serial.println(" " + String(battery(data)));
//		}
//		else {
//			//Serial.print(" Checksum error");
//		}
//	}
////	Serial.println();
//
//	decoder.resetDecoder();
//}

void oneWireHandler() {
	uint8_t cmd;
	if (ds.waitReset(0) && ds.presence(25)) {
		bool isBreak = false;
		char addr[8];
		for (;;) {
			switch (ds.recv()) {
			case 0xF0: // SEARCH ROM
				ds.search();
				Serial.println("SEARCH ROM");
				isBreak = true;
				break;
			case 0x33: // READ ROM
				ds.sendData((char *)rom, 8);
				Serial.println("READ ROM");
				isBreak = true;
				break;
			case 0x55: // MATCH ROM
				ds.recvData(addr, 8);
				cmd = ds.recv();
				if (cmd == 0xA1) ds.sendData(buf_data1, 8);
				else if (cmd == 0xA2) ds.sendData(buf_data2, 8);
				//if (errno != ONEWIRE_NO_ERROR)
				//	return FALSE;
				//ds.recvData(buf, sizeof(buf));
				//for (int i = 0; i < 8; i++) {
				//	Serial.print(buf[i]); Serial.print(" ");
				//}
				//for (int i = 0; i < 5; i++) {
				//	ds.send(buf1[i]);
				//}
				//	if (rom[i] != addr[i])
				//		return FALSE;
				//return TRUE;
				//Serial.println("MATCH ROM");
				isBreak = true;
				break;
			case 0xCC: // SKIP ROM
					   //ds.recvData(buf, sizeof(buf)); //��������� ������
					   //Serial.print("I: ");
					   //for (int i = sizeof(buf) - 1; i >= 0; i--) {
					   //	//Serial.print((int)(buf[i]), HEX);
					   //	//Serial.print(" ");
					   //	ds.send(buf1[i]); //�������� ��������
					   //}
					   //Serial.println("SKIP ROM");
				isBreak = true;
				break;
			default: // Unknow command
					 //if (errno == ONEWIRE_NO_ERROR)
					 //	break; // skip if no error
					 //else
					 //	return FALSE;
				break;
			}
			if (isBreak) break;
		}

	}
}

void setup() {
	Serial.begin(115200);
	Serial.println("\n[ookDecoder]");

	//uint16_t i = (0x1A << 8) | 0xFD;
	//
	//Serial.println(i==0x1AFD);
	pinMode(2, INPUT);
	rcs.enableReceive(0);

	TCCR1A = 0x00;
	TCCR1B = 0x09;
	TCCR1C = 0x00;
	OCR1A = COUNTER_RATE;
	TIMSK1 = 0x02;

	sei();
	ds.setRom(rom); //��� ������������ CRC8 ����� ������ � ������������� ���. setRomnc() - �� ������������ CRC8, �������� ������ ������ ����������.
	buf_data1[7] = OneWireSlave::crc8(buf_data1, 7);
	buf_data2[7] = OneWireSlave::crc8(buf_data2, 7);
	//pinMode(PORT, INPUT);  // use the AIO pin

	//attachInterrupt(digitalPinToInterrupt(PORT), ext_int_1, CHANGE);

}

void loop() {
	static uint16_t hits = 0;
	static unsigned long prev_t = 0;
	const unsigned long t = millis();
	static uint8_t oregonPulseNum = 0;

	//cli();
	//word p = pulse;
	//pulse = 0;
	//sei();

	//if (t % 10000 == 0) Serial.print(".");

	processTimerWH2Handler();

	oneWireHandler();

	//if (t - prev_t > 30000) {
	//	prev_t = t;
	//	Serial.println("p = " + String(rcs.getPulseNum()));
	//}
	if (oregonPulseNum == 0) {
		oregonPulseNum = rcs.getTimings(timings);
		//if (oregonPulseNum > 5) Serial.println("Overtime detected (" + String(oregonPulseNum) + ")\n");
	}
	else {
		static uint8_t i = 0;
		if (wh2.nextPulse(timings[i++])) {
			if (true || wh2.getData()[0] == 73) {
				Serial.print("[" + String(++hits) + " " + String(t / 1000l) + "] ");
				for (int j = 0; j < 5; j++) {
					Serial.print(wh2.getData()[j], BIN);
					Serial.print(" ");
				}
				Serial.print("/ ");
				for (int j = 0; j < 5; j++) {
					Serial.print(wh2.getData()[j], DEC);
					Serial.print(" ");
				}
				Serial.println("\n");
			}
			wh2.resetDecoder();
		}

		//if (orscV3.nextPulse(timings[i++])) {
		//	orscV3.okCRC();
		//}
		if (i == oregonPulseNum) {
			i = 0;
			oregonPulseNum = 0;
		}
	}

	//	if (orscV3.isComplete()) {
	//		if (orscV3.getChannel() == 1 && orscV3.getSerial() == 0xEC && abs(orscV3.getTemperature()) < 100 && orscV3.getHumidity() < 100) {
	//			float tm = ((float)((t - prev_t))) / 60000.;
	//			prev_t = t;
	//			Serial.print("\n[" + String(tm, 2) + "min] ");
	//		}
	//		Serial.print(" " + String(orscV3.getSensorType(), HEX) + " " + String(orscV3.getChannel()) + " " + String(orscV3.getSerial(), HEX));
	//		Serial.print(" " + String(orscV3.getTemperature(), 1) + "*C " + String(orscV3.getHumidity(), 1) + "%");
	//		Serial.println(" " + String(orscV3.getBattery()));
	//		orscV3.resetDecoder();
	//	}
	//
	//	if (rcs.available()) {
	//		unsigned long receivedCode = rcs.getReceivedValue();
	//		if (receivedCode == 0) {
	//			// ���������:�� ������ ������ ������
	//			Serial.println("Incorrect data received");
	//		}
	//		else {
	//			//Serial.println(receivedCode);
	//#define keyTemper 11000
	//			if (keyTemper <= receivedCode && keyTemper + 999 > receivedCode) {
	//				float temperOut = ((float)receivedCode - (float)keyTemper - 500.) / 10.;
	//				Serial.print("[");
	//				Serial.print(millis());
	//				Serial.print("] t=");
	//				Serial.print(temperOut);
	//				Serial.println("*C");
	//			}
	//		}
	//		rcs.resetAvailable(); // ����� ������.
	//	} // end available
	//
}

void processTimerWH2Handler() {
	static uint16_t hits = 0;
	static unsigned long old = 0, packet_count = 0, bad_count = 0, average_interval;
	unsigned long spacing, now;
	byte i;

	if (wh2_flags) {
		if (wh2_accept()) {
			// calculate the CRC
			wh2_calculate_crc();

			now = millis();
			spacing = now - old;
			old = now;
			packet_count++;
			average_interval = now / packet_count;
			if (!wh2_valid()) {
				bad_count++;
			}

			//Serial.print(packet_count, DEC);
			//Serial.print(" | ");
			//Serial.print(bad_count, DEC);
			//Serial.print(" | ");
			//Serial.print(spacing, DEC);
			//Serial.print(" | ");
			//Serial.print(average_interval, DEC);
			//Serial.print(" | ");

			if (true || wh2_packet[0] == 73) {
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
			}
			//for (i = 0; i<5; i++) {
			//	Serial.print("0x");
			//	Serial.print(wh2_packet[i], HEX);
			//	Serial.print("/");
			//	Serial.print(wh2_packet[i], DEC);
			//	Serial.print(" ");
			//}
			//Serial.print("| Sensor ID: 0x");
			//Serial.print(wh2_sensor_id(), HEX);
			//Serial.print(" | ");
			//Serial.print(wh2_humidity(), DEC);
			//Serial.print("% | ");
			//Serial.print(wh2_temperature(), DEC);
			//Serial.print(" | ");
			//Serial.println((wh2_valid() ? "OK" : "BAD"));
		}
		wh2_flags = 0x00;
	}
}

// processes new pulse
boolean wh2_accept()
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

void wh2_calculate_crc()
{
	wh2_calculated_crc = crc8(wh2_packet, 4);
}

bool wh2_valid()
{
	return (wh2_calculated_crc == wh2_packet[4]);
}

int wh2_sensor_id()
{
	return (wh2_packet[0] << 4) + (wh2_packet[1] >> 4);
}

byte wh2_humidity()
{
	return wh2_packet[3];
}

/* Temperature in deci-degrees. e.g. 251 = 25.1 */
int wh2_temperature()
{
	int temperature;
	temperature = ((wh2_packet[1] & B00000111) << 8) + wh2_packet[2];
	// make negative
	if (wh2_packet[1] & B00001000) {
		temperature = -temperature;
	}
	return temperature;
}

uint8_t crc8(uint8_t *addr, uint8_t len)
{
	uint8_t crc = 0;

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
	return crc;
}