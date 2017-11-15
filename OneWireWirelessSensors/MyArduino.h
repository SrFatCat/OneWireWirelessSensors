// MyArduino.h

#ifndef _MYARDUINO_h
#define _MYARDUINO_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif
#define DEF_TMENEGMENT	const unsigned long t = millis(); \
						static unsigned long prev_t = 0;
#define IF_TMENEGMENT(x) if (t - prev_t > (x))
#define PASS_TMENEGMENT prev_t = t;

#define FRACT100(X) (int(round((X) * 100.))%100)
#define FRACT10(X) (int(round((X) * 10.))%10)
#define FRACT1000(X) (int(round((X) * 1000.))%1000)
#define FRACT(X) FRACT100(X)

template <typename T> inline Print & operator << (Print &s, T n) { s.print(n); return s; }

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

#if defined (BLYNK_PRINT) || defined (DEBUG)

#ifdef ESP8266
#define DEBUG_PRINT(...) Serial.printf( __VA_ARGS__ )
#else
#define DEBUG_PRINT(...) Serial.print( __VA_ARGS__ )
#endif // ESP8266

#else

#define DEBUG_PRINT(...)

#endif // BLYNK_PRINT // debug

#endif

