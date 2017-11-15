/*
 Name:		OneWireMaster.ino
 Created:	04.11.2017 14:01:27
 Author:	Alex
*/
// the setup function runs once when you press reset or power the board

//#define ONEWIRE_READ_TIMEOUT 200
//#define SENSORS_NUM 2
#define DEBUG
#include "C1WireWLSensors.h"
#include "MyArduino.h"

//struct SSensorData {
//	byte data[8];
//	bool received = false;
//} data[SENSORS_NUM];

CWLSensorData sensorData[SENSORS_NUM];
C1WireWLSensors sensors;

void setup() {
#ifdef DEBUG
	Serial.begin(9600);
	Serial.println("\nStart 1-Wire master...");
#endif // DEBUG
	sensors.init();
}


void loop() {

	if (sensors.receive(sensorData)) {
		for (int idx = 0; idx < SENSORS_NUM; idx++) {
			if (!sensorData[idx].received) continue; else sensorData[idx].received = false;
			DEBUG_PRINT("%i at %isec. received [%i] %i.%02i*C %i%%\n", idx, 
				sensorData[idx].prevSecs, sensorData[idx].id,
				int(sensorData[idx].temperature), FRACT(sensorData[idx].temperature),
				int(sensorData[idx].humidity));
		}
		//sensorsData[idx].received = data[idx].received;
		//if (data[idx].received) sensorsData[idx].prevSecs = (t / 1000L);
		//sensorsData[idx].id = (data[idx].data[0] << 8 | data[idx].data[1]);
		//sensorsData[idx].temperature = (float)(data[idx].data[4] << 8 | data[idx].data[5]) / 10.;
		//sensorsData[idx].humidity = (float)(data[idx].data[6]);
	}

	//DEF_TMENEGMENT;
//
	//IF_TMENEGMENT(5000){
	//	const uint32_t t = millis();
	//	bool isBreak = true;
	//	while (1) {
	//		for (int idx = 0; idx < SENSORS_NUM; idx++) {
	//			ds.reset();
	//			ds.select(addr);
	//			if (!data[idx].received) {
	//				ds.write(0xA1 + idx);
	//				ds.read_bytes(data[idx].data, 8);
	//				if (OneWire::crc8(data[idx].data, 7) == data[idx].data[7]) {
	//					data[idx].received = true;
	//					Serial.println("Data " + String(idx) + " ok");
	//				}
	//				else isBreak = false;
	//			}
	//			yield();
	//		}
	//		if (isBreak) break;
	//		if (millis() - t > ONEWIRE_READ_TIMEOUT) { Serial.print("."); break; }
	//	}

	//	for (int idx = 0; idx < SENSORS_NUM; idx++) {
	//		if (!data[idx].received) continue;
	//		for (int i = 0; i < 8; i++) {
	//			Serial.printf("%i ", data[idx].data[i]);
	//		}
	//		int d = data[idx].data[0] << 8 | data[idx].data[1];
	//		Serial.print("\n0x");
	//		Serial.print(d, HEX);
	//		Serial.print(" time = ");
	//		d = data[idx].data[2] << 8 | data[idx].data[3];
	//		Serial.print(d);
	//		Serial.print("s. temper = ");
	//		d = data[idx].data[4] << 8 | data[idx].data[5];
	//		Serial.print((float)d / 10.);
	//		Serial.print("*C humid = ");
	//		Serial.print((int)data[idx].data[6]);
	//		Serial.println("%");
	//		data[idx].received = false;
	//	}
	//	PASS_TMENEGMENT;
	//}
}
