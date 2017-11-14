/*
Name:		RCSwitchOregon.h
Created:	01.11.2017
Author: (c) 2011 Suat ?zg?r modifyed Alexey Bogdan aka Sr.FatCat
	 Oregon V2 decoder added - Dominique Pierre
	 Oregon V3 decoder revisited - Dominique Pierre
	 New code to decode OOK signals from weather sensors, etc.
	 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
	 $Id: ookDecoder.pde 5331 2010-04-17 10:45:17Z jcw $
ToDo:


RCSwitch - Arduino libary for remote control outlet switches
Copyright (c) 2011 Suat ?zg?r.  All right reserved.
Contributors:
- Andre Koehler / info(at)tomate-online(dot)de
- Gordeev Andrey Vladimirovich / gordeev(at)openpyro(dot)com
- Skineffect / http://forum.ardumote.com/viewtopic.php?f=2&t=46
- Dominik Fischer / dom_fischer(at)web(dot)de
- Frank Oltmanns / <first name>.<last name>(at)gmail(dot)com
- Max Horn / max(at)quendi(dot)de
- Robert ter Vehn / <first name>.<last name>(at)gmail(dot)com

Project home: https://github.com/sui77/rc-switch/
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.
You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef RCSwitchOregon_h
#define RCSwitchOregon_h

#if defined(ARDUINO) && ARDUINO >= 100
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#include <stdint.h>

// Number of maximum High/Low changes per packet.
// We can handle up to (unsigned long) => 32 bit * 2 H/L changes per bit + 2 for sync
#define RCSWITCH_MAX_CHANGES 67
#define RC_MAX_PULSE_BUFFER 250

class DecodeOOK {
protected:
	byte total_bits, bits, flip, state, pos, data[12];

	virtual char decode(word width) = 0;

public:

	enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

	DecodeOOK() { resetDecoder(); }

	bool nextPulse(word width);

	bool isDone() const { return state == DONE; }

	const byte* getData(byte& count) const { count = pos; return data; }

	void resetDecoder() { total_bits = bits = pos = flip = 0; state = UNKNOWN; }

	// add one bit to the packet data buffer

	virtual void gotBit(char value);

	// store a bit using Manchester encoding
	void manchester(char value);

	// move bits to the front so that all the bits are aligned to the end
	void alignTail(byte max = 0);

	void reverseBits();

	void reverseNibbles();

	void done() { while (bits) gotBit(0); /* padding*/ state = DONE; }
};

class WH2Decoder : public DecodeOOK {
public:
	WH2Decoder() {}
	virtual void gotBit(char value);
	virtual char decode(word width);
	byte* getData() { return data; }
};

class OregonDecoder : public DecodeOOK {
	bool complete = false;
	byte SumCRC();
public:
	OregonDecoder() {}
	virtual void gotBit(char value) = 0;
	virtual char decode(word width) = 0;
	float getTemperature();
	float getHumidity();
	uint16_t getSensorType() { return (data[0] << 8) | data[1]; }
	byte getBattery() { return (data[4] & 0xF); }
	byte getChannel();
	byte getSerial() { return (data[3]); }
	bool okCRC();
	bool isComplete() { return complete; }
	void resetDecoder() { DecodeOOK::resetDecoder(); complete = false; }

};

class OregonDecoderV3 : public OregonDecoder {
public:
	OregonDecoderV3() {}

	// add one bit to the packet data buffer
	virtual void gotBit(char value);

	virtual char decode(word width);
};

class RCSwitch {

  public:
	RCSwitch();

    void switchOn(int nGroupNumber, int nSwitchNumber);
    void switchOff(int nGroupNumber, int nSwitchNumber);
    void switchOn(const char* sGroup, int nSwitchNumber);
    void switchOff(const char* sGroup, int nSwitchNumber);
    void switchOn(char sFamily, int nGroup, int nDevice);
    void switchOff(char sFamily, int nGroup, int nDevice);
    void switchOn(const char* sGroup, const char* sDevice);
    void switchOff(const char* sGroup, const char* sDevice);
    void switchOn(char sGroup, int nDevice);
    void switchOff(char sGroup, int nDevice);

    void sendTriState(const char* sCodeWord);
    void send(unsigned long code, unsigned int length);
    void send(const char* sCodeWord);
    
    void enableReceive(int interrupt);
    void enableReceive();
    void disableReceive();
    bool available();
    void resetAvailable();

    unsigned long getReceivedValue();
    unsigned int getReceivedBitlength();
    unsigned int getReceivedDelay();
    unsigned int getReceivedProtocol();
    unsigned int* getReceivedRawdata();
  
    void enableTransmit(int nTransmitterPin);
    void disableTransmit();
    void setPulseLength(int nPulseLength);
    void setRepeatTransmit(int nRepeatTransmit);
    void setReceiveTolerance(int nPercent);

    struct HighLow {
        uint8_t high;
        uint8_t low;
    };

    struct Protocol {
        int pulseLength;
        HighLow syncFactor;
        HighLow zero;
        HighLow one;
        /** @brief if true inverts the high and low logic levels in the HighLow structs */
        bool invertedSignal;
    };

    void setProtocol(Protocol protocol);
    void setProtocol(int nProtocol);
    void setProtocol(int nProtocol, int nPulseLength);
	
	volatile static unsigned long pulse_num;
	static unsigned long getPulseNum() { unsigned long p = pulse_num; pulse_num = 0;  return p; }

	static uint8_t getTimings(unsigned int* outputTimings) {
		cli();
		uint8_t ret = 0;
		if (oregonTimingsCount > 0) {
			memcpy(outputTimings, oregonTimings, sizeof(unsigned int) * oregonTimingsCount);
			ret = oregonTimingsCount;
			oregonTimingsCount = 0;
		}
		sei();
		return ret;
	}

  private:
    char* getCodeWordA(const char* sGroup, const char* sDevice, bool bStatus);
    char* getCodeWordB(int nGroupNumber, int nSwitchNumber, bool bStatus);
    char* getCodeWordC(char sFamily, int nGroup, int nDevice, bool bStatus);
    char* getCodeWordD(char group, int nDevice, bool bStatus);
    void transmit(HighLow pulses);

    static void handleInterrupt();
    static bool receiveProtocol(const int p, unsigned int changeCount);
    int nReceiverInterrupt;
    int nTransmitterPin;
    int nRepeatTransmit;
    
    Protocol protocol;

    static int nReceiveTolerance;
    static unsigned long nReceivedValue;
    static unsigned int nReceivedBitlength;
    static unsigned int nReceivedDelay;
    static unsigned int nReceivedProtocol;
    const static unsigned int nSeparationLimit;
    /* 
     * timings[0] contains sync timing, followed by a number of bits
     */
    static unsigned int timings[RCSWITCH_MAX_CHANGES];

	static unsigned int oregonTimings[RC_MAX_PULSE_BUFFER];
	static uint8_t oregonTimingsCount;
    
};

#endif
