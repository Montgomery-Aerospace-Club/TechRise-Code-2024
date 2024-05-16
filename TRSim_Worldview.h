// Written by Mark DeLoura and Arnie Martin for Future Engineers
// Last Updated: 2/12/2023

#ifndef _TRSIM_WORLDVIEW_H
#define _TRSIM_WORLDVIEW_H

#include <Arduino.h>

namespace TRSim_Worldview {

// Internal constants
#define _BUFFER_SIZE 1024
#define _BUFFER_SIZE_MASK 0x3ff
#define _RECEIVER_BUFFER_SIZE 512
#define _PACKET_SIZE 38
#define _DEFAULT_BAUD_RATE 9600
#define _DEFAULT_TIMEOUT 0.1

#define _SYNCBYTE1 0x71
#define _SYNCBYTE2CMDA 0x0F
#define _SYNCBYTE2CMDB 0xA7
#define _SYNCBYTE2CMDC 0xF0
#define _SYNCBYTE2DATA 0xFF

#define _MODE_FINDING_SYNCBYTE1 1
#define _MODE_FINDING_SYNCBYTE2 2
#define _MODE_FINDING_PAYLOAD   3

// pin mappings
//   Note: this pin already defined in header files
// #define PIN_NEOPIXEL     40
#define PIN_PBF 2
#define PIN_GO  3

// Set up events
const unsigned int STATUS_NUM_EVENTS = 5;
const unsigned int STATUS_UNKNOWN = 0;
const unsigned int STATUS_INITIALIZING = 1;
const unsigned int STATUS_LAUNCHING = 2;
const unsigned int STATUS_FLOATING = 3;
const unsigned int STATUS_TERMINATING = 4;

// Baud rate for USB serial connection
#define SERIAL_BAUD 9600


// Simulator class
//
class Simulator {
	bool _new_data;
	unsigned int _stream_timeout;

	unsigned char _buffer[_BUFFER_SIZE];
	unsigned char _currPacketBuffer[_BUFFER_SIZE];
	unsigned char _returnBuffer[_PACKET_SIZE];

	bool _isCommandPacket;
	unsigned int _flightStatus;

	unsigned int _mode;
	unsigned int _saveStart;
	unsigned int _saveEnd;
	unsigned int _packetStart;
	unsigned int _packetEnd;
	unsigned int _packetBytesFound;

	int _pbfPin;
	int _goPin;
	int _pbfState;

	void _setFlightStatus(unsigned char eventbyte);

public:
	Simulator();
	~Simulator();
	void init(int pbfPin=PIN_PBF, int goPin=PIN_GO);
	void update();
	bool isNewData();
	bool isStreaming();
	int getPBF();
	int getGo();
	void setGo(int val);

	unsigned char* getData();
	unsigned int getTime();
	unsigned int getStatus();
	float getLatitude();
	float getLongitude();
	float getAltitude();
	float getSpeed();
	float getHeading();
	float getVelocityDown();
	float getPressure();
	float getTemperature();
	void serialPrintCurrentPacket();
	void serialPrintHexString(unsigned char* buff);
};

} // namespace

#endif

