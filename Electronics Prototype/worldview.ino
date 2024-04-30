// World View Launch demo
//   This simple demo reads data coming from the Future Engineers TechRise web-based
//     simulator.
//   It is designed to run on a Metro M4 Express and uses the on-board Neopixel.
//   The demo runs a simple update loop, and does three actions:
//     1 - Keeps track of the number of telemetry packets received*
//     2 - Monitors flight status and prints a message each time it changes
//     3 - Prints out every 100th telemetry packet
//         * Note: Data is sent by the World View flight computer at 1Hz,
//         which is to say: one packet of data per second.
//    Important note: If the web simulator is run faster than 10x speed, data will
//      not be sent to the microcontroller, as this would flood the USB connection.
//
// Written by Mark DeLoura and Arnie Martin for Future Engineers
// Last Updated: 12/12/2022

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <TRSim_Worldview.h>

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

unsigned long lastBlinkTime = 0; 
int ledState = LOW;

// setup()
//   Initialization functions
//
void setup() {
	// Set up Serial
	//   Serial = USB
	//   TRSim = Serial1 = UART
	Serial.begin(SERIAL_BAUD);
	delay(2000); // Improves chances of seeing first printed messages over USB serial

	Serial.println("Running World View Launch demo");

	// Set up simulator data library
	//   By default, PBF pin = 2, GO pin = 3. If your wiring is different, see
	//   documentation for how to change pins using this function call.
	TRsim.init();

  //LED
   pinMode(PIN_GO, OUTPUT);

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

// loop()
//   Do forever, start the main loop
//
void loop() {
	// Pointer for keeping track of incoming data
	unsigned char *data;

	// Call Simulator update() function to catch serial input and check PBF.
	//   This must be called at the top of the loop.
	TRsim.update();

  //LED
  unsigned long currentMillis = millis();
  bool pbfInserted = TRsim.getPBF() == LOW;
  bool dataStreaming = TRsim.isStreaming();

      // Determine the LED behavior based on the PBF header and streaming status
    if (pbfInserted && !dataStreaming) {
        // Blink with interval of 5 seconds
        if (currentMillis - lastBlinkTime >= 2500) {
            lastBlinkTime = currentMillis;
            ledState = !ledState;
            digitalWrite(PIN_GO, ledState);
        }
    } else if (!pbfInserted && !dataStreaming) {
        // Blink every second (1 second ON, 1 second OFF)
        if (currentMillis - lastBlinkTime >= 1000) {
            lastBlinkTime = currentMillis;
            ledState = !ledState;
            digitalWrite(PIN_GO, ledState);
        }
    } else if (pbfInserted && dataStreaming) {
        // Blink with interval of 3 seconds
        if (currentMillis - lastBlinkTime >= 1500) {
            lastBlinkTime = currentMillis;
            ledState = !ledState;
            digitalWrite(PIN_GO, ledState);
        }
    } else if (!pbfInserted && dataStreaming) {
        // Constantly ON
        digitalWrite(PIN_GO, HIGH);
    }

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
