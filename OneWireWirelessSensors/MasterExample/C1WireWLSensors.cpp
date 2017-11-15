// 
// 
// 

#include "MyArduino.h"
#include "C1WireWLSensors.h"

///const OneWire& C1WireWLSensors::ds = OneWire(PIN_ONE_WIRE);

void C1WireWLSensors::init()
{
	if (!ds->search((uint8_t*)addr)) {
		ds->reset_search();
		return;
	}
#ifdef DEBUG
	Serial.print("ROM =");
	for (int i = 0; i < 8; i++) {
		Serial.write(' ');
		Serial.print(addr[i], HEX);
	}
	Serial.println();
#endif // DEBUG
	if (OneWire::crc8(addr, 7) != addr[7]) {
		DEBUG_PRINT("CRC is not valid!\n");
		return;
	}
	else isFinded = true;
}

bool C1WireWLSensors::receive(CWLSensorData* sensorsData)
{
	bool retn = false;
	DEF_TMENEGMENT;

	IF_TMENEGMENT(5000) {
		if (!isFinded) init();
		if (isFinded) {
			const uint32_t t = millis();
			bool isBreak = true;
			while (1) {
				for (int idx = 0; idx < SENSORS_NUM; idx++) {
					ds->reset();
					ds->select(addr);
					if (!data[idx].received) {
						ds->write(0xA1 + idx);
						ds->read_bytes(data[idx].data, 8);
						if (OneWire::crc8(data[idx].data, 7) == data[idx].data[7]) {
							data[idx].received = true;
							DEBUG_PRINT("Data %i ok\n", idx);
						}
						else isBreak = false;
					}
#ifdef ESP8266
					yield();
#endif // ESP8266
				}
				if (isBreak) break;
				if (millis() - t > ONEWIRE_READ_TIMEOUT) { DEBUG_PRINT("."); break; }
			}

			for (int idx = 0; idx < SENSORS_NUM; idx++) {
				if (!data[idx].received) continue;
				sensorsData[idx].received = data[idx].received;
				if (data[idx].received) sensorsData[idx].prevSecs = (t / 1000L);
				sensorsData[idx].id = (data[idx].data[0] << 8 | data[idx].data[1]);
				sensorsData[idx].temperature = (float)(data[idx].data[4] << 8 | data[idx].data[5]) / 10.;
				sensorsData[idx].humidity = (float)(data[idx].data[6]);
#ifdef DEBUG
				for (int i = 0; i < 8; i++) {
					Serial.printf("%i ", data[idx].data[i]);
				}
				int d = data[idx].data[0] << 8 | data[idx].data[1];
				Serial.print("\n0x");
				Serial.print(d, HEX);
				Serial.print(" time = ");
				d = data[idx].data[2] << 8 | data[idx].data[3];
				Serial.print(d);
				Serial.print("s. temper = ");
				d = data[idx].data[4] << 8 | data[idx].data[5];
				Serial.print((float)d / 10.);
				Serial.print("*C humid = ");
				Serial.print((int)data[idx].data[6]);
				Serial.println("%");
#endif
				data[idx].received = false;
				retn = true;
			}
		}
		PASS_TMENEGMENT;
	}
	return retn;
}


//C1WireWLSensors C1WireWLSensors;

