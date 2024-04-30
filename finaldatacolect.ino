
#include <Arduino.h> // imports lib
#include <SensirionI2CScd4x.h> // imports lib
#include <Wire.h> // imports lib
#include "DFRobot_OzoneSensor.h" // imports lib
#include <SD.h> // imports lib
#include <SPI.h> // imports lib

#define COLLECT_NUMBER  10      // collect number, the collection range is 1-100
#define Ozone_IICAddress OZONE_ADDRESS_3 // tells the arduino what i2c address it has so that arudino can focus on multiple sensors using i2c bus

SensirionI2CScd4x scd4x; // creates a sensor object named scd4x
DFRobot_OzoneSensor Ozone; // creates a sensor object named Ozone

int chipSelect = 4; // telling arduino what CS pin we have connected to

File SensorData; // creates a file named SensorData





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
    while (!Serial) {
        delay(100);
    }

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
        Serial.println("Co2:");
        Serial.print(co2);
        Serial.print("\t");
        Serial.println("Temperature:");
        Serial.print(temperature);
        Serial.print("\t");
        Serial.println("Humidity:");
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
  delay(5000);
  
  SensorData.print(ozoneConcentration); // prints ozone data to filke
  SensorData.print(","); // prints ,  to file SensorData for comma dilemination 
  
  SensorData.close(); // close file
    }
} 
