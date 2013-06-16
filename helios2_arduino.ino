
// Helios 2 Arduino Sketch
// Operation Helios (operationhelios.com)

#include <SoftwareSerial.h>
#include <SD.h>
#include "Time.h"
#include "TinyGPS.h"

// PINS
const unsigned int STATUS = A0;  // status LED: off when all good, flashing when error
const unsigned int SD_SELECT = 8;
const unsigned int GPS_RX = 3;
const unsigned int GPS_TX = A5;  // unused
const unsigned int DTMF_SIGNAL = 9;
const unsigned int DTMF0 = 2;  // LSB
const unsigned int DTMF1 = 4;
const unsigned int DTMF2 = 5;
const unsigned int DTMF3 = 7;  // MSB
const unsigned int CUT_BALLOON = 6;
const unsigned int CUT_PARACHUTE = 10;
const unsigned int BATTERY_VOLTAGE = A1;
const unsigned int TEMP_IN = A2;  // temperature inside the capsule
const unsigned int TEMP_OUT = A3;  // temperature outside

// CONSTANTS
const unsigned int SERIAL_BAUD = 57600;
const unsigned int GPS_BAUD = 4800;
const unsigned int CUT_BALLOON_TIME = 5000;  // measured in milliseconds
const unsigned int CUT_PARACHUTE_TIME = 5000;
const unsigned int ERROR_FLASH_DELAY = 100;

const byte balloonPattern[] = {1,2,3};  // please don't repeat digits one after the other!
const byte parachutePattern[] = {7,8,9};  // don't repeat digits
const unsigned int balloonPatternLength = sizeof(balloonPattern);
const unsigned int parachutePatternLength = sizeof(parachutePattern);

// GLOBAL VARIABLES
unsigned int balloonPatternPosition = 0;
unsigned int parachutePatternPosition = 0;

// FUNCTION PROTOTYPES
void setPinModes();
void error();  // flash status LED for error
bool validDate();  // gps has valid date data
void gpsGetData();
void setTime();

File file;
TinyGPS gps;
SoftwareSerial gpsSerial = SoftwareSerial(GPS_RX, GPS_TX);

void setup()
{
  // init serial for debugging
  Serial.begin(SERIAL_BAUD);
  
  setPinModes();
  
  // light status LED
  digitalWrite(STATUS, HIGH);
  
  // init GPS serial
  gpsSerial.begin(GPS_BAUD);
  
  // init SD card
  if (!SD.begin(SD_SELECT))
    error();
  
  // wait for date/time data to calibrate clock
  while (!validDate())
    gpsGetData();
  
  // set the date/time
  setTime();
  
  // setup well well, turn light off
  digitalWrite(STATUS, LOW);
}

void loop()
{
  // print time for debugging
  time_t time = now();
  String timestamp = String(day(time)) + String("/") + String(month(time)) + String("/") + String(year(time));
  timestamp += String(" ") + String(hour(time)) + String(":") + String(minute(time)) + String(":") + String(second(time));
  timestamp += " " + String(time);
  Serial.println(timestamp);
  delay(1000);
}

void setPinModes()
{
  pinMode(STATUS, OUTPUT);
  pinMode(DTMF_SIGNAL, INPUT);
  pinMode(DTMF0, INPUT);
  pinMode(DTMF1, INPUT);
  pinMode(DTMF2, INPUT);
  pinMode(DTMF3, INPUT);
  pinMode(CUT_BALLOON, OUTPUT);
  pinMode(CUT_PARACHUTE, OUTPUT);
  pinMode(BATTERY_VOLTAGE, INPUT);
  pinMode(TEMP_IN, INPUT);
  pinMode(TEMP_OUT, INPUT);
}

// flash LED continuously to show error
void error()
{
  while (true)
  {
    digitalWrite(STATUS, LOW);
    delay(ERROR_FLASH_DELAY);
    digitalWrite(STATUS, HIGH);
    delay(ERROR_FLASH_DELAY);
  }
}

// check if GPS has received valid date data
bool validDate()
{
  unsigned long date;
  unsigned long time;
  unsigned long fixAge;
  gps.get_datetime(&date, &time, &fixAge);
  return (date != 0);
}

// read GPS data and feed it to TinyGPS
void gpsGetData()
{
  while (gpsSerial.available())
  {
    int data = gpsSerial.read();
    gps.encode(data);
  }
}

// set the time from GPS data
void setTime()
{
  int years;
  byte months;
  byte days;
  byte hours;
  byte minutes;
  byte seconds;
  byte hundredths;
  unsigned long fixAge;
  gps.crack_datetime(&years, &months, &days, &hours, &minutes, &seconds, &hundredths, &fixAge);
  setTime(hours, minutes, seconds, days, months, years);
}

