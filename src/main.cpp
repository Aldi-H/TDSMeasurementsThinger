//! Library Initialization Goes Here!
#include <Arduino.h>

//* TDS Library
#include "GravityTDS.h"

//* Temperature Library
#include <OneWire.h>
#include <DallasTemperature.h>

//* RTC Library
#include <SPI.h>
#include "RTClib.h"

//! Variable and PIN Initialization  Goes Here!
//* TDS PIN
#define TDS_PIN 34

//! Variable Instance Goes Here!
//* TDS Library Insatance
GravityTDS gravityTds;

//* Temperature Library Instance
OneWire oneWire(33);
DallasTemperature DS18B20_Sensor(&oneWire);

//* Global Variable TDS and Temperature
//* Initial Value for TDS and Temperature
float tdsValue = 0;
float temperatureValue = 0;

//! Function Declaration Goes Here!
//* FUnction declaration for readTDS
void readTDS();

//! Main Program Goes Here!
void setup()
{
  Serial.begin(115200);

  //* Gravity TDS Setup
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();
}

void loop()
{
  readTDS();
}
//! MAIN PROGRAM END HERE!

//! Function Definition Goes Here!
//* readTDS() function definition
void readTDS()
{
  DS18B20_Sensor.requestTemperatures();
  temperatureValue = DS18B20_Sensor.getTempCByIndex(0);
  gravityTds.setTemperature(temperatureValue);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();

  Serial.printf("Temperature: %.2fC ", temperatureValue);
  Serial.printf("| TDS Value: %.2fppm \n", tdsValue);
}