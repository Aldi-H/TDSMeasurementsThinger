//! Library Initialization Goes Here!
#include <Arduino.h>

//* Thinger Library
#include <ThingerESP32.h>
#include "arduino_secrets.h"

//* TDS Library
#include "GravityTDS.h"

//* Temperature Library
#include <OneWire.h>
#include <DallasTemperature.h>

//* RTC Library
#include <SPI.h>
#include "RTClib.h"

//* LCD I2C Library
#include <LiquidCrystal_I2C.h>

//* EEPROM Library
#include <EEPROM.h>

//! Variable and PIN Initialization  Goes Here!
//* TDS PIN
#define TDS_PIN 34

//* Thinger Initialization
ThingerESP32 thing(USERNAME, DEVICE_ID, DEVICE_CREDENTIAL);

//* SSR PIN
#define SSR_PIN1 26
#define SSR_PIN2 14

//* FLow Meter Pin and Interrupt
#define sensorInterrupt 0
#define FLOW_PIN1 35
#define FLOW_PIN2 32

//* EEPROM Variable
#define EEPROM_SIZE 16

//* Median Variable
#define SCOUNT 30

//* WiFi Initialization
// const char *ssid = "Kuro";
// const char *password = "kuro_1905";
// const char *ssid = "CMCC-Xsaben";
// const char *password = "12345678";
const char *ssid = "Huawei-23GH";
const char *password = "7tffh1r3";

//! Variable Instance Goes Here!
//* TimePoint and paralel time
long timePointLED = 1000; // Blink LED every 1s
unsigned long prevTimeLED = millis();

//* LED BUILTIN State
int LED_Builtin_State = LOW;

//* TDS Library Insatance
GravityTDS gravityTds;

//* Temperature Library Instance
OneWire oneWire(33);
DallasTemperature DS18B20_Sensor(&oneWire);

//* RTC Instance
RTC_DS1307 rtc_DS1307;

//* LCD I2C Instance
LiquidCrystal_I2C lcd_I2C(0x27, 20, 4);

//* Global Variable TDS and Temperature
//* Initial Value for TDS and Temperature
float tdsValue = 0;
float temperatureValue = 0;

//* Thinger Instance
bool isSendToEndpoint = false;

//* Used to make sure the flowmeter not opened when device turned on
int globalCounter = 0;

//* Flow Meter Variables
//* Flow Meter Counter
volatile int flowMeterCount1;
volatile int flowMeterCount2;
//* Flow Meter Rate
float calibrationFactor = 55;
float flowRate1 = 0.0;
float flowRate2 = 0.0;
unsigned int flowMilliLitres1 = 0;
unsigned long totalMilliLitres1 = 0;
unsigned int flowMilliLitres2 = 0;
unsigned long totalMilliLitres2 = 0;
unsigned long flowMeterOldTime = 0;

//* LCD Variable
unsigned long backlightOnTime = 0;
const unsigned long backlightOnDuration = 3000;

//* Median Instance
float MedianBuffer[SCOUNT];
int MedianBufferIndex = 0;
int copyIndex = 0;
float TDSMedian = 0;

//! Function Declaration Goes Here!
//* Function declaration for getMedianNum
float getMedianNum(float bArray[], int iFilterLen);

//* Function declaration for readTDS_Median
void readTDS_Median();

//* Function declaration for readTDS
void readTDS();

//* Function declaration for openSelenoidValve
void openSelenoidValve(int flowRate);

void setCalibrationFactor(int flowRate);

//* Function declaration for IRAM_ATTR IRAMFlow
void IRAM_ATTR IRAMFlow1();
void IRAM_ATTR IRAMFlow2();

//! Main Program Goes Here!
void setup()
{

  //* EEPROM Begin
  EEPROM.begin(EEPROM_SIZE);

  Serial.begin(115200);

  thing.add_wifi(ssid, password);

  //
  lcd_I2C.init();
  lcd_I2C.backlight();

  lcd_I2C.clear();

  //* Check RTC
  if (!rtc_DS1307.begin())
  {
    Serial.println("Couldn't find RTC");
    lcd_I2C.clear();
    lcd_I2C.setCursor(0, 1);
    lcd_I2C.print("Couldn't find RTC");
    while (1)
      ;
  }

  //* SSR Setup
  pinMode(SSR_PIN1, OUTPUT);
  digitalWrite(SSR_PIN1, HIGH);
  pinMode(SSR_PIN2, OUTPUT);
  digitalWrite(SSR_PIN2, HIGH);

  //* Flow Meter Setup
  pinMode(FLOW_PIN1, INPUT);
  pinMode(FLOW_PIN2, INPUT);
  attachInterrupt(FLOW_PIN1, IRAMFlow1, FALLING);
  attachInterrupt(FLOW_PIN2, IRAMFlow2, FALLING);

  thing["data"] >> [](pson &out)
  {
    float tdsVal;
    float tempVal;

    out["ppm"] = TDSMedian;
    out["temperature"] = temperatureValue;
    out["source"] = "auto";
    out["deviceId"] = "yuanth";
  };

  thing["valve"] << [](pson &in)
  {
    int flow = in;
    if (globalCounter != 0)
    {
      Serial.println(flow);
      openSelenoidValve(flow);
    }
  };
  globalCounter++;

  //* Gravity TDS Setup
  gravityTds.setPin(TDS_PIN);
  gravityTds.setAref(3.3);
  gravityTds.setAdcRange(4096);
  gravityTds.begin();
}

void loop()
{
  thing.handle();

  unsigned long millisCurrentTime = millis();

  // Blink Internal LED every 1s
  if (millisCurrentTime - prevTimeLED > timePointLED)
  {
    LED_Builtin_State = !LED_Builtin_State;
    digitalWrite(BUILTIN_LED, LED_Builtin_State);

    prevTimeLED = millisCurrentTime;
  }

  DateTime rtcCurrentTime = rtc_DS1307.now();
  readTDS();

  MedianBuffer[MedianBufferIndex] = tdsValue; //  store into the buffer
  MedianBufferIndex++;
  if (MedianBufferIndex >= SCOUNT)
  {
    TDSMedian = getMedianNum(MedianBuffer, SCOUNT);
    Serial.printf("Temperature: %.2fC ", temperatureValue);
    Serial.printf("| TDS Median Value: %.2fppm \n", TDSMedian);
    MedianBufferIndex = 0;
  }

  // You can remove isSendToEndpoint condition if not double sent like MQTT
  if (rtcCurrentTime.second() == 0 && !isSendToEndpoint)
  {
    if (rtcCurrentTime.minute() % 1 == 0)
    {
      thing.call_endpoint("endpoint_speedtest", thing["data"]);
      Serial.println("Send to thinger");

      lcd_I2C.noBacklight();
      lcd_I2C.clear();
      lcd_I2C.setCursor(0, 0);
      lcd_I2C.print("Send to Thinger");
      lcd_I2C.clear();

      lcd_I2C.setCursor(0, 1);
      lcd_I2C.print("Temperature: ");
      lcd_I2C.setCursor(12, 1);
      lcd_I2C.print(temperatureValue);

      lcd_I2C.setCursor(0, 2);
      lcd_I2C.print("TDS Value: ");
      lcd_I2C.setCursor(10, 2);
      lcd_I2C.print(TDSMedian);

      isSendToEndpoint = true;
    }
  }
  else if (rtcCurrentTime.second() != 0)
  {
    isSendToEndpoint = false;
  }
}
//! MAIN PROGRAM END HERE!

//! Function Definition Goes Here!
//* getMedianNum function definition
float getMedianNum(float bArray[], int iFilterLen)
{
  int bTab[iFilterLen];

  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;

  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
  {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else
  {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

//* readTDS_Median function definition
void readTDS_Median()
{
  DS18B20_Sensor.requestTemperatures();
  temperatureValue = DS18B20_Sensor.getTempCByIndex(0);
  gravityTds.setTemperature(temperatureValue);
  gravityTds.update();
  tdsValue = gravityTds.getTdsValue();
  // median
  MedianBuffer[MedianBufferIndex] = tdsValue; //  store into the buffer

  MedianBufferIndex++;
  if (MedianBufferIndex >= SCOUNT)
  {
    TDSMedian = getMedianNum(MedianBuffer, SCOUNT);
    Serial.printf("Temperature: %.2fC ", temperatureValue);
    Serial.printf("| TDS Value: %.2fppm \n", TDSMedian);
    MedianBufferIndex = 0;
  }
}

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

  lcd_I2C.setCursor(0, 3);
  lcd_I2C.print("                    ");
  lcd_I2C.setCursor(10, 3);
  lcd_I2C.print(tdsValue);
}

void IRAM_ATTR IRAMFlow1()
{
  flowMeterCount1++;
}

void IRAM_ATTR IRAMFlow2()
{
  flowMeterCount2++;
}

void setCalibrationFactor(int flowRate)
{
  if (flowRate == 20)
  {
    calibrationFactor = 50;
  }
  else if (flowRate = 50)
  {
    calibrationFactor = 55;
  }
  else
  {
    calibrationFactor = 60;
  }
}

//* openSelenoidValve function definition
void openSelenoidValve(int flowRate)
{
  float calFactor;

  if (flowRate == 20)
  {
    calFactor = 50;
  }
  else if (flowRate == 50)
  {
    calFactor = 55;
  }
  else
  {
    calFactor = 60;
  }

  Serial.println(calFactor);

  digitalWrite(SSR_PIN1, LOW);
  digitalWrite(SSR_PIN2, LOW);

  //* uncomment this later
  attachInterrupt(FLOW_PIN1, IRAMFlow1, FALLING);
  attachInterrupt(FLOW_PIN2, IRAMFlow2, FALLING);

  while (totalMilliLitres2 <= flowRate && totalMilliLitres1 <= flowRate)
  {
    // only process counters once per second
    if ((millis() - flowMeterOldTime) > 1000)
    {
      // Disable the interrupt while calculating flow rate and sending the value to the host
      detachInterrupt(FLOW_PIN1);
      detachInterrupt(FLOW_PIN2);

      // //* Check valve one by one
      if (totalMilliLitres1 >= flowRate)
      {
        digitalWrite(SSR_PIN1, HIGH);
        detachInterrupt(FLOW_PIN1);
      }

      if (totalMilliLitres2 >= flowRate)
      {
        digitalWrite(SSR_PIN2, HIGH);
        detachInterrupt(FLOW_PIN2);
      }

      /*
       * Because this loop may not complete in exactly 1 second intervals
       * we calculate the number of milliseconds that have passed since the last execution
       * and use that to scale the output.
       * We also apply the calibrationFactor to scale the output based on the number of pulses per second per units of measure
       * (litres/minute in this case) coming from the sensor.
       */
      flowRate1 = ((1000.0 / (millis() - flowMeterOldTime)) * flowMeterCount1) / calFactor;
      flowRate2 = ((1000.0 / (millis() - flowMeterOldTime)) * flowMeterCount2) / calFactor;

      /*
       * Note the time this processing pass was executed.
       * Note that because we've disabled interrupts the millis() function won't actually be incrementing right at this point,
       * but it will still return the value it was set to just before interrupts went away.
       */
      flowMeterOldTime = millis();

      /*
       * Divide the flow rate in litres/minute by 60 to determine how many litres have  passed through the sensor in this 1 second interval,
       * then multiply by 1000 to convert to millilitres.
       */
      flowMilliLitres1 = (flowRate1 / 60) * 1000;
      flowMilliLitres2 = (flowRate2 / 60) * 1000;

      //* Add the millilitres passed in this second to the cumulative total
      totalMilliLitres1 += flowMilliLitres1;
      totalMilliLitres2 += flowMilliLitres2;

      Serial.print("Flow rate1: ");
      Serial.print(flowMilliLitres1, DEC); // Print the integer part of the variable
      Serial.print("mL/Second");
      Serial.print("\t");

      // Print the cumulative total of litres flowed since starting
      Serial.print("Output Liquid Quantity1: ");
      Serial.print(totalMilliLitres1, DEC);
      Serial.println("mL");
      Serial.print("\t");

      lcd_I2C.clear();
      lcd_I2C.setCursor(0, 1);
      lcd_I2C.print("Total ml A: ");
      lcd_I2C.setCursor(12, 1);
      lcd_I2C.print(totalMilliLitres1, DEC);

      // Print the flow rate for this second in litres / minute
      Serial.print("Flow rate2: ");
      Serial.print(flowMilliLitres2, DEC); // Print the integer part of the variable
      Serial.print("mL/Second");
      Serial.print("\t");

      // Print the cumulative total of litres flowed since starting
      Serial.print("Output Liquid Quantity2: ");
      Serial.print(totalMilliLitres2, DEC);
      Serial.println("mL");
      Serial.print("\t");

      lcd_I2C.setCursor(0, 2);
      lcd_I2C.print("Total ml B: ");
      lcd_I2C.setCursor(12, 2);
      lcd_I2C.print(totalMilliLitres2, DEC);

      // Reset the pulse counter so we can start incrementing again
      flowMeterCount1 = 0;
      flowMeterCount2 = 0;

      // Enable the interrupt again now that we've finished sending output
      attachInterrupt(FLOW_PIN1, IRAMFlow1, FALLING);
      attachInterrupt(FLOW_PIN2, IRAMFlow2, FALLING);
    }
  }

  digitalWrite(SSR_PIN1, HIGH);
  detachInterrupt(FLOW_PIN1);

  digitalWrite(SSR_PIN2, HIGH);
  detachInterrupt(FLOW_PIN2);

  totalMilliLitres1 = 0;
  totalMilliLitres2 = 0;

  lcd_I2C.clear();

  globalCounter++;
}