// C1WireWLSensors.h

#ifndef _C1WIREWLSENSORS_h
#define _C1WIREWLSENSORS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#include <OneWire.h>

#ifndef PIN_ONE_WIRE
#define PIN_ONE_WIRE D1
#endif

#ifndef ONEWIRE_READ_TIMEOUT
#define ONEWIRE_READ_TIMEOUT 200
#endif

#ifndef SENSORS_NUM
#define SENSORS_NUM 2
#endif

class CWLSensorData {
public:
	bool received;
	uint16_t id = 0;
	uint32_t prevSecs = 0;
	uint8_t channel = 0;
	float temperature = 0;
	float humidity = 0;
	uint8_t battery;
};

class C1WireWLSensors{
	uint8_t addr[8];
	OneWire* ds;
	struct SSensorData {
		byte data[8];
		bool received = false;
	} data[SENSORS_NUM];
	bool isFinded = false;
	CWLSensorData* outputData = nullptr;
 protected:
 public:
	C1WireWLSensors() { ds = new OneWire(PIN_ONE_WIRE);  }
	void init();
	bool receive(CWLSensorData* );
	//CWLSensorData* getData() { if (!outputData) for (int idx = 0; idx < SENSORS_NUM; idx++) outputData[idx].received = false; return outputData; }
};

//extern C1WireWLSensors C1WireWLSensors;

#endif

