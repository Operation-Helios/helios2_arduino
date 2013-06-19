
// Helios 2 Arduino Sketch
// Operation Helios (operationhelios.com)

#include <SoftwareSerial.h>
#include <SD.h>
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
char filename[] = "00000000.000";
const byte filenameLength = 13;
unsigned int balloonPatternPosition = 0;
unsigned int parachutePatternPosition = 0;

// FUNCTION PROTOTYPES
void setPinModes();
void error();  // flash status LED for error
void gpsGetData(void (*callback)(bool*), bool* returnValue);
void checkFix(bool* returnValue);  // tells if the GPS has a fix
void calcFilename();

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
  
  // wait for good GPS data
  bool gotFix = false;
  while (!gotFix)
    gpsGetData(&checkFix, &gotFix);
  
  // store the date to use as the filename of the log
  calcFilename();
  
  // setup well well, turn light off
  digitalWrite(STATUS, LOW);
}

void loop() {}

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

// read GPS data and feed it to TinyGPS and call a callback if a valid sentence is received
void gpsGetData(void (*callback)(bool*), bool* returnValue)
{
  while (gpsSerial.available())
  {
    int data = gpsSerial.read();
    if (gps.encode(data))
      callback(returnValue);
  }
}

// detect if the GPS has a fix
void checkFix(bool* returnValue)
{
  unsigned long date, time, fixAge;
  gps.get_datetime(&date, &time, &fixAge);
  *returnValue = (fixAge != 0);
}

// store the date as the filename of the log file
void calcFilename()
{
  // get the date values
  int year;
  byte month, day, hour, minute, second, hundredth;
  unsigned long fixAge;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredth, &fixAge);
  
  // convert them into Strings
  String dayString, monthString, yearString;
  dayString = String(day);
  if (dayString.length() < 2)  // length is always at least 1
    dayString = String("0") + dayString;
  monthString = String(month);
  if (monthString.length() < 2)
    monthString = String("0") + monthString;
  yearString = String(year);  // already four digits
  
  String baseName = yearString + monthString + dayString;  // ISO 8601
  baseName += String(".");
  
  // find the next available name
  // sketch will not work if it has been used more than 999 times in the same day
  unsigned int counter = 0;
  while (true)
  {
    // convert counter value into a three digit string
    String counterString = String(counter);
    switch (counterString.length())
    {
      case 0:
        counterString = "000";
        break;
      case 1:
        counterString = "00" + counterString;
        break;
      case 2:
        counterString = "0" + counterString;
        break;
      default:
        break;
    }
    
    // test if such a file exists
    String tempFilename = baseName + counterString;
    tempFilename.toCharArray(filename, filenameLength);
    if (SD.exists(filename))
      break;
    else
      counter++;
  }
}

