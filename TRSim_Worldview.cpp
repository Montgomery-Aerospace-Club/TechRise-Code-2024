#include "TRSim_Worldview.h"

// Written by Mark DeLoura and Arnie Martin for Future Engineers
// Last Updated: 3/3/2023

namespace TRSim_Worldview {

/** @file TRSim_Worldview.cpp
 * @brief
 * An Arduino library to read World View flight telemetry data from the Future Engineers TechRise flight simulator.
 * Data is read over UART (RX pin on most CircuitPython boards).
 * Defines a Simulator class with properties to determine when a new packet is available.
 * Helper property functions are included to convert from telemetry format to usable values.
 * @details
@verbatim embed:rst
World View data format:
    * Transmitted via serial connection: 9600 baud, 8N1
    * 38-byte data packet, little endian
    * 1Hz telemetry packets

Data fields:
    Header (2):
        | uint8[2]  *Sync Word*, 0x71 [0x0F, 0xA7, 0xF0, 0xFF], 0xFF = data packet, others are events
    Payload (32):
        | float32   *Latitude*, in degrees
        | float32   *Longitude*, in degrees
        | float32   *Altitude*, in meters
        | float32   *Speed*, in meters/second
        | float32   *Heading*, in degrees
        | float32   *VelocityD*, in meters/second
        | float32   *Pressure*, in pascals
        | float32   *Temperature*, in celsius
    Timestamp (2):
        | uint16    *Timestamp*, elapsed time, [12..15] hours, [6..11] minutes, [0..5] seconds
    CRC (2):
        | uint16    *CRC*, checksum
@endverbatim
*/

/** 
 * @class Simulator TRSim_Worldview.h
 * @brief Conducts World View flight telemetry processing
 * @details
 * - Simulator::update(): Processes incoming flight telemetry data
 * - Simulator::isNewData(): True if new data has been received
 * - Simulator::isStreaming(): True if data is streaming
 * - Simulator::getPBF(): Value of the Pull-Before-Flight Header, True = removed
 * - Simulator::getGo(): Value of the GO LED, True = lit
 * - Simulator::setGo(val): Sets GO LED, HIGH = lit
 * - Simulator::getData(): Value of the new full data packet as a bytearray
 *
 * Use getData() to acquire all telemetry fields of the current packet. 
 * To acquire individual data fields from the current packet, use the helper functions detailed below.
 */
Simulator::Simulator() :
	_new_data(false), _stream_timeout(0), _mode(_MODE_FINDING_SYNCBYTE1),
	_saveStart(0), _saveEnd(0), _packetStart(0), _packetEnd(0),
	_packetBytesFound(0), _isCommandPacket(false), _flightStatus(STATUS_UNKNOWN) {
}

Simulator::~Simulator() {

}

/** 
 * @brief Initializes the simulator
 * @param pbfPin A board pin number, connected to the PBF pin on PIB J5
 * @param goPin A board pin number, connected to the GO pin on PIB J5
 */
void Simulator::init(int pbfPin, int goPin) {
	// set the onboard UART pin
	Serial1.begin(_DEFAULT_BAUD_RATE);

	// Store incoming pin parameters
	_pbfPin = pbfPin;
	_goPin = goPin;

	// Setup the _pbfPin and _goPin
	pinMode(_pbfPin, INPUT);
	if (_pbfPin != PIN_PBF) {
		Serial.print("Pull Before Flight header active on ");
		Serial.println(_pbfPin);
	}
	pinMode(_goPin, OUTPUT);
	if (_goPin != PIN_GO) {
		Serial.print("Go LED active on ");
		Serial.println(_goPin);
	}
	// New behavior as of 3/3/2023 - GO LED is always lit
	setGo(HIGH);

	// Print out the starting state of the PBF header
	_pbfState = getPBF();
	if (_pbfState == HIGH) {
		Serial.println("Pull-before flight header removed");
	} else {
		Serial.println("Pull-before flight header inserted");
	}
	Serial.println();

}

/**
 * @brief Internal helper function for processing event command packets
 * @param eventbyte Command packet function byte
 */
void Simulator::_setFlightStatus(unsigned char eventbyte) {
	if (eventbyte == _SYNCBYTE2CMDA) {
		_flightStatus = STATUS_LAUNCHING;
	} else if (eventbyte == _SYNCBYTE2CMDB) {
		_flightStatus= STATUS_FLOATING;
	} else if (eventbyte == _SYNCBYTE2CMDC) {
		_flightStatus = STATUS_TERMINATING;
	}
}

/**
 * @brief Processes incoming flight telemetry data
 */
void Simulator::update() {
	// Check for removal of PBF header
	//   True = removed, so new value is True, stored state is False
	if ((getPBF() == HIGH) && (_pbfState == LOW)) {
		// flash the GO led a few times
		// go_pin will be set according to the state of the pbf_pin when TRsim.update() is called
		Serial.println("Pull Before Flight header has been removed");
		for (int i=0; i<6; i++) {
			setGo((getGo()==HIGH)?LOW:HIGH);
			delay(500);
		}
	}
	_pbfState = getPBF();

	// New behavior as of 3/3/2023 - GO LED is always lit
/*
	// Light Go LED if the PBF header is removed
	if (_pbfState == LOW) {
		// it is not, so turn on the go led
		setGo(LOW);
	} else {
		// it must be so turn off the go led
		setGo(HIGH);
	}
*/

	int bytesWaiting = Serial1.available();
	if (bytesWaiting > 0) {
		int firstPackets = 0;
		bool isWrappedLoad = false;
		bool isWrappedPacket = false;

		// Copy waiting bytes to ring buffer, from saveStart to saveEnd
		//   Wrap ring buffer if load goes past end of buffer
		_saveEnd = _saveStart + bytesWaiting;
		isWrappedLoad = (_saveEnd >= _BUFFER_SIZE);
		if (isWrappedLoad == true) {
			firstPackets = _BUFFER_SIZE - _saveStart;
			Serial1.readBytes(_buffer+_saveStart, firstPackets);
			Serial1.readBytes(_buffer, bytesWaiting - firstPackets);
		} else {
			Serial1.readBytes(_buffer+_saveStart, bytesWaiting);
		}

		// Process new bytes one by one
		//   Advance through modes based on data:
		//     _MODE_FINDING_SYNCBYTE1 - looking for 0x71 packet start
		//     _MODE_FINDING_SYNCBYTE2 - looking for 0xFF, second data packet byte
		//     _MODE_FINDING_PAYLOAD   - looking for rest of 38 byte packet
		//   When full packet found, copy to currMV. Deal with wrap if necessary.
		unsigned int wrappedIndex;
		for (int index=_saveStart; index<_saveEnd; index++) {
			wrappedIndex = index & _BUFFER_SIZE_MASK;

			if (_mode == _MODE_FINDING_SYNCBYTE1) {
				if (_buffer[wrappedIndex] == _SYNCBYTE1) {
					_mode = _MODE_FINDING_SYNCBYTE2;
					_packetStart = wrappedIndex;
				}
			} else if (_mode == _MODE_FINDING_SYNCBYTE2) {
				if (_buffer[wrappedIndex] == _SYNCBYTE2DATA) {
					_mode = _MODE_FINDING_PAYLOAD;
					_isCommandPacket = false;
					_packetBytesFound = 2;
				} else if ((_buffer[wrappedIndex] == _SYNCBYTE2CMDA) ||
					(_buffer[wrappedIndex] == _SYNCBYTE2CMDB) ||
					(_buffer[wrappedIndex] == _SYNCBYTE2CMDC)) {
					_mode = _MODE_FINDING_PAYLOAD;
					_isCommandPacket = true;
					_packetBytesFound = 2;
				} else { // if _SYNCBYTE2 is not immediately after _SYNCBYTE1, start over
					_mode = _MODE_FINDING_SYNCBYTE1;
				}
			} else if (_mode == _MODE_FINDING_PAYLOAD) {
				_packetBytesFound = _packetBytesFound + 1;
				if (_packetBytesFound == _PACKET_SIZE) {     // Full packet found
					_packetEnd = wrappedIndex;
					isWrappedPacket = ((_packetEnd+1) < _PACKET_SIZE);
					if (isWrappedPacket == true) {
						firstPackets = _BUFFER_SIZE - _packetStart;
						memcpy(_currPacketBuffer, _buffer+_packetStart, firstPackets);
						memcpy(_currPacketBuffer+firstPackets, _buffer, _packetEnd+1);
					} else {
						memcpy(_currPacketBuffer, _buffer+_packetStart, _PACKET_SIZE);
					}

					// We have a full packet
					//   If it's a command packet, ignore payload and set event
					//   If it's a data packet, set _new_data
					//     If time == 1, set flight status to INITIALIZING
					if (_isCommandPacket == true) {
						_setFlightStatus(_currPacketBuffer[1]);
					} else {
						_new_data = true;
						if (getTime() == 1) {
							_flightStatus = STATUS_INITIALIZING;
						}
					}

					_stream_timeout = millis() + 1500;
					_mode = _MODE_FINDING_SYNCBYTE1;
				}
			} else {
				//raise Exception('UART data state machine error')
				exit(-1);
			}
		}
		_saveStart = _saveEnd & _BUFFER_SIZE_MASK;
	}
}

/**
 * @brief Returns True if new data has been received
 */
bool Simulator::isNewData() {
	if (_new_data == true) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief Returns True if data is streaming, and recieve timeout has not been exceeded
 */
bool Simulator::isStreaming() {
	if (millis() <= _stream_timeout) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief Returns value of the Pull-Before-Flight Header, HIGH if Header removed
 */
int Simulator::getPBF() {
	if (_pbfPin != -1) {
		return digitalRead(_pbfPin);
	} else {
		return -1;
	}
}

/**
 * @brief Returns value of the GO LED, HIGH if LED is lit
 */
int Simulator::getGo() {
	if (_goPin != -1) {
		return digitalRead(_goPin);
	} else {
		return -1;
	}
}

/**
 * @brief Sets GO LED
 * @param val Value for GO LED (HIGH or LOW)
 */
void Simulator::setGo(int val) {
	if (_goPin != -1) {
		digitalWrite(_goPin, val);
	}
}

/**
 * @brief Returns the new full data packet as an array of bytes
 */
unsigned char* Simulator::getData() {
	if (_new_data == true) {
		_new_data = false;
		memcpy(_returnBuffer, _currPacketBuffer, _PACKET_SIZE);
		return _returnBuffer;
	} else {
		return (unsigned char*)0; // Error
	}
}

/**
 * @brief Returns flight status, as a constant unsigned int
 */
unsigned int Simulator::getStatus() {
	if (isStreaming() == true) {
		return _flightStatus;
	} else {
		return 0; // Error
	}
}

/**
 * @brief Returns timestamp, in seconds
 */
unsigned int Simulator::getTime() {
	if (isStreaming() == true) {
		unsigned int intval = ((_currPacketBuffer[35]<<8)| \
								(_currPacketBuffer[34]));
		unsigned int retval = ((intval >> 12) & 0xf) * 3600 + \
					((intval >> 6) & 0x3f) * 60 + \
					(intval & 0x3f);
		return retval;
	} else {
		return 0; // Error
	}
}

/**
 * @brief Returns latitude, in degrees
 */
float Simulator::getLatitude() {
	if (isStreaming() == true) {
		float retval;
		unsigned int* retvalptr = (unsigned int*) &retval;
		*retvalptr = (_currPacketBuffer[5]<<24) | 
					(_currPacketBuffer[4]<<16) | 
					(_currPacketBuffer[3]<<8) | 
					(_currPacketBuffer[2]);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns longitude, in degrees
 */
float Simulator::getLongitude() {
	if (isStreaming() == true) {
		float retval;
		unsigned int* retvalptr = (unsigned int*) &retval;
		*retvalptr = (_currPacketBuffer[9]<<24) | 
					(_currPacketBuffer[8]<<16) | 
					(_currPacketBuffer[7]<<8) | 
					(_currPacketBuffer[6]);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns altitude, in meters
 */
float Simulator::getAltitude() {
	if (isStreaming() == true) {
		float retval;
		unsigned int* retvalptr = (unsigned int*) &retval;
		*retvalptr = (_currPacketBuffer[13]<<24) | 
					(_currPacketBuffer[12]<<16) | 
					(_currPacketBuffer[11]<<8) | 
					(_currPacketBuffer[10]);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns speed, signed, in meters/second
 */
float Simulator::getSpeed() { 
	if (isStreaming() == true) {
		float retval;
		unsigned int* retvalptr = (unsigned int*) &retval;
		*retvalptr = (_currPacketBuffer[17]<<24) | 
					(_currPacketBuffer[16]<<16) | 
					(_currPacketBuffer[15]<<8) | 
					(_currPacketBuffer[14]);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns heading, signed, in degrees
 */
float Simulator::getHeading() { 
	if (isStreaming() == true) {
		float retval;
		unsigned int* retvalptr = (unsigned int*) &retval;
		*retvalptr = (_currPacketBuffer[21]<<24) | 
					(_currPacketBuffer[20]<<16) | 
					(_currPacketBuffer[19]<<8) | 
					(_currPacketBuffer[18]);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns velocity down, signed, in meters/second
 */
float Simulator::getVelocityDown() { 
	if (isStreaming() == true) {
		float retval;
		unsigned int* retvalptr = (unsigned int*) &retval;
		*retvalptr = (_currPacketBuffer[25]<<24) | 
					(_currPacketBuffer[24]<<16) | 
					(_currPacketBuffer[23]<<8) | 
					(_currPacketBuffer[22]);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns pressure, in pascals
 */
float Simulator::getPressure() { 
	if (isStreaming() == true) {
		float retval;
		unsigned int* retvalptr = (unsigned int*) &retval;
		*retvalptr = (_currPacketBuffer[29]<<24) | 
					(_currPacketBuffer[28]<<16) | 
					(_currPacketBuffer[27]<<8) | 
					(_currPacketBuffer[26]);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Returns temperature, in celsius
 */
float Simulator::getTemperature() { 
	if (isStreaming() == true) {
		float retval;
		unsigned int* retvalptr = (unsigned int*) &retval;
		*retvalptr = (_currPacketBuffer[33]<<24) | 
					(_currPacketBuffer[32]<<16) | 
					(_currPacketBuffer[31]<<8) | 
					(_currPacketBuffer[30]);
		return retval;
	} else {
		return 0.0f; // Error
	}
}

/**
 * @brief Prints current data packet to USB serial connection 
 */
void Simulator::serialPrintCurrentPacket() {
	char outputString[255];

	serialPrintHexString(_currPacketBuffer);

	Serial.print("  time ");
	Serial.println(getTime());
	Serial.print("  latitude ");
	Serial.println(getLatitude());
	Serial.print("  longitude ");
	Serial.println(getLongitude());
	Serial.print("  altitude ");
	Serial.println(getAltitude());
	Serial.print("  speed ");
	Serial.println(getSpeed());
	Serial.print("  heading ");
	Serial.println(getHeading());
	Serial.print("  velocity down ");
	Serial.println(getVelocityDown());
	Serial.print("  pressure ");
	Serial.println(getPressure());
	Serial.print("  temperature ");
	Serial.println(getTemperature());
}

/**
 * @brief Prints current data packet in hex to USB serial connection 
 */
void Simulator::serialPrintHexString(unsigned char* buff) {
	char outputString[255];
	int cursor = 0;

	for (int i=0; i<_PACKET_SIZE; i++) {
		sprintf(&(outputString[cursor]), "%02x", buff[i]);
		cursor += 2;
	}
	outputString[cursor] = 0;

	Serial.println(outputString);  
}


} // namespace
