﻿/*
Name:		OneWireWirelessSensors.ino
Created:	01.11.2017
Author:	Alexey Bogdan aka Sr.FatCat
ToDo:
	- сделать пример со всеми типами датчиков
	- установка командами 1-Wire буфера датчиков
	- установка командами 1-Wire типа датчиков
	- привести все классы датчиков в к одному виду 
	и сделать от одного предка

*/

//#define DEBUG

#define DIRECT_MODE_INPUT(base, mask)  ((*(base+1)) &= ~(mask))
#define DIRECT_READ(base, mask)        (((*(base)) & (mask)) ? 1 : 0)

#include "MyArduino.h"
#include "WH2Sensor.h"
#include "RCSwitchOregon.h"
#include "OneWireSlave.h"
#include "EEPROM.h"

#define PIN_ONE_WIRE 3
OneWireSlave ds(PIN_ONE_WIRE);
unsigned char rom[8] = { 0x01, 0xAD, 0xDA, 0xCE, 0x0F, 0x00, 0x00, 0xFF }; 

#define SENSORS_NUM 2
struct SSensorData {
	char buf_data[8] = {0,0,0,0,0,0,0,0};
	bool received = false;
	unsigned long prev_t = 0;
	int id=0;
} sensor_data[SENSORS_NUM];

struct SEEPROMData {
	uint8_t sensorsNum;
	uint8_t rom[8];
	SSensorData sensor_data[SENSORS_NUM];
};

bool isEEPROMSaved = false;

WH2TimerDecoder wh2; //WH2 TESA TRANSMITTER FOR WS1150, FOSHK WH1150, WH1170 

OregonDecoderV3 orscV3; // Oregon Scientific V3 sensor

//RCSwitch rcs; //RCswitch sensor

// unsigned int timings[RC_MAX_PULSE_BUFFER]; //orsc timings buffer
void resetData() {
	for (uint8_t idx = 0; idx < SENSORS_NUM; idx++) {
		for (int i=0;  i<8; i++) sensor_data[idx].buf_data[i] = 0;
		sensor_data[idx].received = false;
		sensor_data[idx].id = 0;
	}
	isEEPROMSaved = false;
}

void oneWireHandler() {
	uint8_t cmd;
	int p = 0;
	ds.waitForRequest(false);
	while (1) {
		cmd = ds.recv();
		if (cmd >= 0xA0 && cmd <= 0xAF) {
			uint8_t idx = cmd - 0xA1;
			if (idx < SENSORS_NUM) {
				if (sensor_data[idx].received) {
					ds.sendData(sensor_data[idx].buf_data, 8);
					sensor_data[idx].received = false;
				}
				p++;
			}
			if (p == SENSORS_NUM) break;
		}
		else if (cmd == 0xCC) {
			resetData();
			char buf[3] = "Ok";
			ds.sendData(buf, 3);
			DEBUG_PRINT("Reset ok\n");
			break;
		}
	}
	DEBUG_PRINT("^");
	wh2.startTimerHandler();
}

void fillSensorData(uint8_t idx) {
	uint16_t d = wh2.sensor_id();
	sensor_data[idx].received = true;
	sensor_data[idx].id = d;
	sensor_data[idx].buf_data[0] = highByte(d);
	sensor_data[idx].buf_data[1] = lowByte(d);
	d = (uint16_t)(millis() / 1000L);
	sensor_data[idx].prev_t = d;
	sensor_data[idx].buf_data[2] = highByte(d);
	sensor_data[idx].buf_data[3] = lowByte(d);
	d = wh2.temperature();
	sensor_data[idx].buf_data[4] = highByte(d);
	sensor_data[idx].buf_data[5] = lowByte(d);
	sensor_data[idx].buf_data[6] = wh2.humidity();
	sensor_data[idx].buf_data[7] = OneWireSlave::crc8(sensor_data[idx].buf_data, 7);
}

void saveEEPROM() {
	SEEPROMData tmp;
	tmp.sensorsNum = SENSORS_NUM;
	memcpy(tmp.rom, rom, sizeof(rom));
	for (uint8_t idx = 0; idx < SENSORS_NUM; idx++) {
		memcpy( &(tmp.sensor_data[idx]), &(sensor_data[idx]), sizeof(SSensorData));
	}
	EEPROM.put(0, tmp);
	isEEPROMSaved = true;
}

void setup() {
#ifdef DEBUG
	Serial.begin(115200);
	Serial.println("\n1-Wire WL-Sensors...");
#endif
	pinMode(2, INPUT);
	pinMode(PIN_ONE_WIRE, INPUT);
	//rcs.enableReceive(0);
	SEEPROMData tmp;
	EEPROM.get(0, tmp);
	if (tmp.sensorsNum == SENSORS_NUM) {
#ifdef DEBUG
		Serial.println("Read from EEPROM");
#endif
		memcpy(rom, tmp.rom, sizeof(rom));
		for (uint8_t idx = 0; idx < SENSORS_NUM; idx++) {
			memcpy(&(sensor_data[idx]), &(tmp.sensor_data[idx]), sizeof(SSensorData));
		}
	}

	wh2.init();

	ds.setRom(rom); 
}

void loop() {
	static uint8_t mask = digitalPinToBitMask(PIN_ONE_WIRE);
	volatile static uint8_t *reg asm("r30") = portInputRegister(digitalPinToPort(PIN_ONE_WIRE));

	static uint32_t time_low_signal = millis();
	static bool p = DIRECT_READ(reg, mask);

	uint32_t t = millis();

	if ( DIRECT_READ(reg, mask) ) {
		if (time_low_signal > 0 && t - time_low_signal > 990) {
			DEBUG_PRINT("v");
			wh2.stopTimerHandler();
			oneWireHandler();
			cli();
			DIRECT_MODE_INPUT(reg, mask);
			sei();
			wh2.startTimerHandler();
		}
		time_low_signal = 0;
	}
	else if (time_low_signal == 0) time_low_signal = t;

	if (wh2.getSensorData()) {
		wh2.stopTimerHandler();
		bool isReceivedDuplet = false;
		bool isIDFinded = false;
		int id = wh2.sensor_id(); 
		for (uint8_t idx = 0; idx < SENSORS_NUM; idx++) {
			if (sensor_data[idx].id == id) {
				if (sensor_data[idx].received && (uint16_t)(millis() / 1000L) - sensor_data[idx].prev_t > 5) {
					isReceivedDuplet = true;
				}
				fillSensorData(idx);
				isIDFinded = true;
				break;
			}
		}
		if (!isIDFinded) {
			for (uint8_t idx = 0; idx < SENSORS_NUM; idx++) {
				if (sensor_data[idx].id == 0) {
					fillSensorData(idx);
					isIDFinded = true;
					break;
				}
			}
		}
#ifdef DEBUG
		if (!isIDFinded) {
			Serial.println("err");
		}
		for (uint8_t idx = 0; idx < SENSORS_NUM; idx++) {
			Serial.print("[" + String(idx) + "] R=(" + sensor_data[idx].received + ") 0x");
			Serial.print(sensor_data[idx].buf_data[0] << 8 | sensor_data[idx].buf_data[1], HEX);
			Serial.print(": ");
			for (int i = 0; i < 8; i++) {
				Serial.print((int)(sensor_data[idx].buf_data[i]));
				Serial.print(".");
			}
			Serial.println();
		}
#endif
		if (!isReceivedDuplet) {
			for (uint8_t idx = 0; idx < SENSORS_NUM; idx++)
				if (!sensor_data[idx].received) {
					wh2.startTimerHandler();
					return;
				}
			if (!isEEPROMSaved) saveEEPROM();
		}
		oneWireHandler();
		cli();
		DIRECT_MODE_INPUT(reg, mask);
		sei();
		time_low_signal = 0;
	}

	// Позже
	//if (rcs.available()) {
	//	unsigned long receivedCode = rcs.getReceivedValue();
	//	if (receivedCode == 0) {
	//		// обработка:не верный формат данных
	//		DEBUG_PRINT("Incorrect data received");
	//	}
	//	else {
	//		if (KEY_TEMPER <= receivedCode && KEY_TEMPER + 999 > receivedCode) {
	//			pump.setTemperature((float)(receivedCode - KEY_TEMPER - 500) / 10);
	//		}
	//	}

}
