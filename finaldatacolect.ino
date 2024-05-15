#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <TRSim_Worldview.h>


#include "DFRobot_MultiGasSensor.h"
#include <Arduino.h> // imports lib
#include <SensirionI2CScd4x.h> // imports lib
#include <Wire.h> // imports lib
#include "DFRobot_OzoneSensor.h" // imports lib
#include <SD.h> // imports lib
#include <SPI.h> // imports lib
#include "Adafruit_LTR390.h"



//--------------------------------------------------
// Set up Neopixel hardware constants and object for the M4's on-board Neopixel
const unsigned int NEOPIXEL_COUNT = 1;
const unsigned int NEOPIXEL_BRIGHTNESS = 0.2;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, PIN_NEOPIXEL);

// Set up some basic color constants for use later
const unsigned int COLOR_RED = 0xff0000;
const unsigned int COLOR_GREEN = 0x00ff00;
const unsigned int COLOR_BLUE = 0x0000ff;
const unsigned int COLOR_YELLOW = 0xffff00;
const unsigned int COLOR_MAGENTA = 0xff00ff;
const unsigned int COLOR_CYAN = 0x00ffff;
const unsigned int COLOR_BLACK = 0x000000;
const unsigned int COLOR_GRAY = 0x7f7f7f;
const unsigned int COLOR_WHITE = 0xffffff;

// Set up flight status event Neopixel colors index
unsigned int statusColors[TRSim_Worldview::STATUS_NUM_EVENTS];

// Set up simulator data library
TRSim_Worldview::Simulator TRsim;

// Initialize flight status
int currStatus = TRSim_Worldview::STATUS_UNKNOWN;
int prevStatus = currStatus;

// Variable for tracking number of full telemetry packets received
int numPackets = 0;

// setup()
//   Initialization functions
//
//--------------------------------------------------


#define COLLECT_NUMBER  10      // collect number, the collection range is 1-100
#define Ozone_IICAddress OZONE_ADDRESS_3 // tells the arduino what i2c address it has so that arudino can focus on multiple sensors using i2c bus

Adafruit_LTR390 ltr = Adafruit_LTR390();
SensirionI2CScd4x scd4x; // creates a sensor object named scd4x
DFRobot_OzoneSensor Ozone; // creates a sensor object named Ozone

int chipSelect = 4; // telling arduino what CS pin we have connected to

File SensorData; // creates a file named SensorData

//Turn on by default, using I2C communication at the time, switch to serial port communication after turning off
#define I2C_COMMUNICATION

#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x74
  DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);
#else
/* ---------------------------------------------------------------------------------------------------------------------
 *    board   |             MCU                | Leonardo/Mega2560/M0 |    UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *     VCC    |            3.3V/5V             |        VCC           |    VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *     GND    |              GND               |        GND           |    GND    |   GND   |  GND  |     X      |  gnd  |
 *     RX     |              TX                |     Serial1 TX1      |     5     |   5/D6  |  D2   |     X      |  tx1  |
 *     TX     |              RX                |     Serial1 RX1      |     4     |   4/D7  |  D3   |     X      |  rx1  |
 * ----------------------------------------------------------------------------------------------------------------------*/
/* Baud rate cannot be changed  */
  #if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
    SoftwareSerial mySerial(4, 5);
    DFRobot_GAS_SoftWareUart gas(&mySerial ,9600);
  #elif defined(ESP32)
    DFRobot_GAS_HardWareUart gas(&Serial1 ,9600 ,/*rx*/D2 ,/*tx*/D3);
  #else
    DFRobot_GAS_HardWareUart gas(&Serial1 ,9600);
  #endif
#endif





void printUint16Hex(uint16_t value) {
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}



void setup() {

    Serial.begin(9600);
    if (!Serial) {
        delay(100);
    }
    Serial.println("Check 1");

    //--------------------------------------------------
    //ltr.begin();
    delay(1000);
    Serial.println("Check 2");

    while (!ltr.begin()) {
      Serial.println("Couldn't find LTR sensor!");
      }

  Serial.println("Found LTR sensor!");

  ltr.setMode(LTR390_MODE_ALS);
  if (ltr.getMode() == LTR390_MODE_ALS) {
    Serial.println("In ALS mode");
  } else {
    Serial.println("In UVS mode");
  }

  ltr.setGain(LTR390_GAIN_1);
  Serial.println("Gain : ");
  switch (ltr.getGain()) {
    case LTR390_GAIN_1: Serial.println(1); break;
    case LTR390_GAIN_3: Serial.println(3); break;
    case LTR390_GAIN_6: Serial.println(6); break;
    case LTR390_GAIN_9: Serial.println(9); break;
    case LTR390_GAIN_18: Serial.println(18); break;
  }

  ltr.setResolution(LTR390_RESOLUTION_16BIT);
  Serial.println("Resolution : ");
  switch (ltr.getResolution()) {
    case LTR390_RESOLUTION_13BIT: Serial.println(13); break;
    case LTR390_RESOLUTION_16BIT: Serial.println(16); break;
    case LTR390_RESOLUTION_17BIT: Serial.println(17); break;
    case LTR390_RESOLUTION_18BIT: Serial.println(18); break;
    case LTR390_RESOLUTION_19BIT: Serial.println(19); break;
    case LTR390_RESOLUTION_20BIT: Serial.println(20); break;
  }

  ltr.setThresholds(100, 1000);
  ltr.configInterrupt(true, LTR390_MODE_UVS);


  //--------------------------------------------------

    pinMode(10, OUTPUT); // reserve in 10 as output , dont use it for otherthing


    SD.begin(chipSelect); // intialize sd card with CS connected to pin 4

    Wire.begin();

    uint16_t error;
    char errorMessage[256];

    scd4x.begin(Wire);

    // stop potentially previously started measurement
    error = scd4x.stopPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    uint16_t serial0;
    uint16_t serial1;
    uint16_t serial2;
    error = scd4x.getSerialNumber(serial0, serial1, serial2);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        printSerialNumber(serial0, serial1, serial2);
    }

    // Start Measurement
    error = scd4x.startPeriodicMeasurement();
    if (error) {
        Serial.print("Error trying to execute startPeriodicMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    Serial.println("Waiting for first measurement... (5 sec)");

    while(!Ozone.begin(Ozone_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
  
  Serial.println("I2c connect success !");

  
    Ozone.setModes(MEASURE_MODE_PASSIVE);

// --------------------------------- NO2 Setup

  while(!gas.begin())
    {
      Serial.println("NO Deivces !");
      delay(1000);
    }
    Serial.println("The device is connected successfully!");

   //Mode of obtaining data: the main controller needs to request the sensor for data
   gas.changeAcquireMode(gas.PASSIVITY);
   delay(1000);

    Serial.println("Check 2");

   /**
    *Turn on temperature compensation: gas.ON : turn on
     *             gas.OFF：turn off
    */
   gas.setTempCompensation(gas.ON);

   Serial.println("Check 3");

   //--------------------------------------------------------

   Serial.println("Running World View Launch demo");

	// Set up simulator data library
	//   By default, PBF pin = 2, GO pin = 3. If your wiring is different, see
	//   documentation for how to change pins using this function call.
	TRsim.init();

	// Set up status colors
	statusColors[TRSim_Worldview::STATUS_UNKNOWN] = COLOR_GRAY;
	statusColors[TRSim_Worldview::STATUS_INITIALIZING] = COLOR_YELLOW;
	statusColors[TRSim_Worldview::STATUS_LAUNCHING] = COLOR_GREEN;
	statusColors[TRSim_Worldview::STATUS_FLOATING] = COLOR_CYAN;
	statusColors[TRSim_Worldview::STATUS_TERMINATING] = COLOR_BLUE;

	// Display flight status (unknown) on Neopixel
	pixels.begin();
	pixels.setBrightness(255 * NEOPIXEL_BRIGHTNESS);
	pixels.fill(statusColors[currStatus], 0, NEOPIXEL_COUNT);
	pixels.show();


    }

void loop() {
    uint16_t error;
    char errorMessage[256];
    delay(100);


    SensorData = SD.open("cos.txt", FILE_WRITE) ;  //open file cos.txt as a file to write it 
    
    
    if (SensorData) {

    
    // Read Measurement
    uint16_t co2 = 0;
    float temperature = 0.0f;
    float humidity = 0.0f;
    bool isDataReady = false;
    error = scd4x.getDataReadyFlag(isDataReady);
    if (error) {
        Serial.print("Error trying to execute getDataReadyFlag(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
        return;
    }
    if (!isDataReady) {
        return;
    }
    error = scd4x.readMeasurement(co2, temperature, humidity);
    if (error) {
        Serial.print("Error trying to execute readMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else if (co2 == 0) {
        Serial.println("Invalid sample detected, skipping.");
    } else {
        Serial.print("Co2:");
        Serial.println(co2);
 
        Serial.print("Temperature:");
        Serial.println(temperature);
        
        Serial.print("Humidity:");
        Serial.println(humidity);


        SensorData.println("co2: ");
        SensorData.print(co2); // prints co2 data to file SensorData
        SensorData.print(","); // prints ,  to file SensorData for comma dilemination 
        SensorData.println("Temperature: ");
        SensorData.println(temperature); // prints temperature data to file SensorData
        SensorData.print(","); // prints ,  to file SensorData for comma dilemination 
        SensorData.println("Humidity: ");
        SensorData.println(humidity); // prints humidity data to file SensorData
        SensorData.print(","); // prints ,  to file SensorData for comma dilemination 
        
  
       
        
    }
    
  int16_t ozoneConcentration = Ozone.readOzoneData(COLLECT_NUMBER);
  Serial.print("Ozone concentration is ");
  Serial.print(ozoneConcentration);
  Serial.println(" PPB.");

  
  SensorData.print(ozoneConcentration); // prints ozone data to filke
  SensorData.print(","); // prints ,  to file SensorData for comma dilemination 
  
  


  if (ltr.newDataAvailable()) {
      ltr.setMode(LTR390_MODE_ALS);
      delay(50);
      Serial.print("ALS data: ");
      SensorData.print("ALS data: ");
      Serial.println(ltr.readALS()); 
      SensorData.println(ltr.readALS());
      delay(50);
      ltr.setMode(LTR390_MODE_UVS);
      delay(50);
      Serial.print("UV data: "); 
      SensorData.print("UV data: ");
      Serial.println(ltr.readUVS());
      SensorData.println(ltr.readUVS());
      delay(50);

      
  }
// -----------------------------------------------
  Serial.println("Ambient ");
  SensorData.println("Ambient: ");
  Serial.print(gas.queryGasType());
  SensorData.print(gas.queryGasType());
  Serial.println(" concentration is: ");
  SensorData.println("Concentration is:");
  Serial.print(gas.readGasConcentrationPPM());
  SensorData.print(gas.readGasConcentrationPPM());
  Serial.println(" %vol");
  SensorData.println(" %vol");
  SensorData.println("");


  Serial.println();
  

// -----------------------------------------------

  float seconds = (millis() / 1000);
  Serial.println(seconds);
  Serial.println("");
  
  SensorData.println("Seconds: ");
  SensorData.print(seconds);
  

  SensorData.close(); // close file
    }

    // Pointer for keeping track of incoming data
	unsigned char *data;

	// Call Simulator update() function to catch serial input and check PBF.
	//   This must be called at the top of the loop.
	TRsim.update();

	// Check if the PBF header is closed (False). If it is, light Neopixel red.
	//   If it is open (True), we are free to do in-flight work!
	if (TRsim.getPBF() == LOW) {
		// Light the neopixel red to highlight that the PBF is inserted
		pixels.fill(COLOR_RED, 0, NEOPIXEL_COUNT);
		pixels.show();
	} else {
		// PBF header is open, we are flying and can do some work!

		// TRsim.isStreaming() will be True while valid data is incoming
		// If data is paused for more than 1.5 seconds it will become False
		if (TRsim.isStreaming() == true) {
			// TRsim.isNewData() will be True after a new packet arrives
			// When TRsim.getData() is called TRsim.isNewData() will become False.
			if (TRsim.isNewData() == true) {
				// Got a new telemetry packet, let's count it!
				numPackets += 1;

				// Grab new data - NOTE this sets isNewData() to False
				data = TRsim.getData();

				// You can add code here that needs to execute each time new
				//   telemetry data is received.

				// Get current flight status and check to see if it has changed
				currStatus = TRsim.getStatus();
				// See if the status has changed by comparing with previous value
				if (currStatus != prevStatus) {
					// If it has changed, save the current status to prevStatus
					prevStatus = currStatus;
					// Since the event changed, print something to indicate change.
					// You can initiate activity for your payload in this
					//   section, since it will only execute on status change.
					//   However, when running the simulator, please note that
					//   this section may execute again if you pause and unpause
					//   the simulator.
					if (currStatus == TRSim_Worldview::STATUS_INITIALIZING) {
						Serial.println("We are initializing");
					} else if (currStatus == TRSim_Worldview::STATUS_LAUNCHING) {
						Serial.println("We are launching");
					} else if (currStatus == TRSim_Worldview::STATUS_FLOATING) {
						Serial.println("We are floating");
					} else if (currStatus == TRSim_Worldview::STATUS_TERMINATING) {
						Serial.println("We are terminating");
					}
					// Indicate the new event with a color from the status list
					pixels.fill(statusColors[currStatus], 0, NEOPIXEL_COUNT);
					pixels.show();
				}

				// Print every 100th packet to verify data
				if ((numPackets % 100) == 1) {
					TRsim.serialPrintCurrentPacket();
				}
			}
		} else { // not streaming
			// Data stream has stopped for 1.5s, fill pixels with unknown (idle) color
			pixels.fill(statusColors[TRSim_Worldview::STATUS_UNKNOWN], 0, NEOPIXEL_COUNT);
			pixels.show();

			// Reset the previous status to unknown (idle)
			prevStatus = TRSim_Worldview::STATUS_UNKNOWN;
		}
	}

	delay(10);



  



} 
