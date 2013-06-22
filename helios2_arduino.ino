
// Helios 2 Arduino Sketch
// Operation Helios (operationhelios.com)

// LOG FILE FORMAT -
// date,time,latitude,longitude,altitude,speed,course,satellites,fixage,
// (cont.) characters,sentences,failedchecksums,batteryvoltage,tempin,tempout,
// date is in ddmmyy
// time is in hhmmsscc
// altitude is in metres
// speed is in 100ths of a knot
// course is in 100ths of a degree
// fixage is in milliseconds
// batteryvoltage, tempin and tempout are raw values from analogRead()

#include <SoftwareSerial.h>
#include <SD.h>
#include "TinyGPS.h"

// PINS
const unsigned int STATUS = A0;  // status LED: off when all good, flashing when error
const unsigned int SD_SELECT = 8;
const unsigned int GPS_RX = 3;
const unsigned int GPS_TX = 10;  // unused
const unsigned int DTMF_SIGNAL = 9;  // pin is high when receiving DTMF signal
const unsigned int DTMF0 = 2;  // LSB
const unsigned int DTMF1 = 4;
const unsigned int DTMF2 = 5;
const unsigned int DTMF3 = 7;  // MSB
const unsigned int CUT_BALLOON = 6;
const unsigned int CUT_PARACHUTE = A5;
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
unsigned long balloonTimeout = 0;  // stop cutting balloon when this time has been reached
unsigned long parachuteTimeout = 0;

// FUNCTION PROTOTYPES
void setPinModes();
void error();  // flash status LED for error
void gpsGetData(void (*callback)(bool*), bool* returnValue);
void checkFix(bool* returnValue);  // tells if the GPS has a fix
void logData(bool* returnValue);
void calcFilename();
bool gotSignal();  // DTMF tone coming in?
byte getPinValue(unsigned int pin);  // digitalRead() a pin, returning 1 or 0
byte readDTMF();  // return the value of the received tone
void matchTone(
  byte toneValue,
  const byte* pattern,
  unsigned int* patternPosition,
  unsigned int patternLength,
  int CUT_PIN,
  unsigned long* timeout,
  unsigned int CUT_TIME,
  const char* cutString);  // match tone with pattern, cutting if appropriate
void checkTimeout(
  unsigned long time,
  unsigned long* timeout,
  int CUT_PIN,
  const char* cutString);  // stop cutdown if timeout reached

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
  
  // setup went well, turn light off
  digitalWrite(STATUS, LOW);
}

void loop()
{
  // get data from the GPS, logging data when new GPS data available
  bool returnValue;  // unused
  gpsGetData(&logData, &returnValue);
  
  // check if a DTMF tone is being received, then process it
  if (gotSignal())
  {
    byte toneValue = readDTMF();
    
    matchTone(
      toneValue,
      balloonPattern,
      &balloonPatternPosition,
      balloonPatternLength,
      CUT_BALLOON,
      &balloonTimeout,
      CUT_BALLOON_TIME,
      "BALLOON");  // check balloon cutdown
    
    matchTone(
      toneValue,
      parachutePattern,
      &parachutePatternPosition,
      parachutePatternLength,
      CUT_PARACHUTE,
      &parachuteTimeout,
      CUT_PARACHUTE_TIME,
      "PARACHUTE");  // check parachute cutdown
  }
  
  // check if it is time to stop cutting either the balloon or the parachute
  unsigned long time = millis();
  checkTimeout(time, &balloonTimeout, CUT_BALLOON, "BALLOON");
  checkTimeout(time, &parachuteTimeout, CUT_PARACHUTE, "PARACHUTE");
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
  
  digitalWrite(CUT_BALLOON, LOW);
  digitalWrite(CUT_PARACHUTE, LOW);
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
  *returnValue = (fixAge != TinyGPS::GPS_INVALID_AGE);
}

// log all available data
void logData(bool* returnValue)
{
  // acquire GPS data
  unsigned long date, time, speed, course, chars, fixAge;
  long latitude, longitude;
  float altitude;
  unsigned short satellites, sentences, failed;
  
  gps.get_datetime(&date, &time, &fixAge);
  gps.get_position(&latitude, &longitude, &fixAge);
  gps.stats(&chars, &sentences, &failed);
  speed = gps.speed();
  course = gps.course();
  altitude = gps.f_altitude();
  satellites = gps.satellites();
  
  // acquire sensor data
  int batteryVoltage = analogRead(BATTERY_VOLTAGE);
  int tempIn = analogRead(TEMP_IN);
  int tempOut = analogRead(TEMP_OUT);
  
  // open log file
  File file = SD.open(filename, FILE_WRITE);
  
  // write data to the log file
  file.print(date); file.print(",");
  file.print(time); file.print(",");
  file.print(latitude); file.print(",");
  file.print(longitude); file.print(",");
  file.print(altitude); file.print(",");
  file.print(speed); file.print(",");
  file.print(course); file.print(",");
  file.print(satellites); file.print(",");
  file.print(fixAge); file.print(",");
  file.print(chars); file.print(",");
  file.print(sentences); file.print(",");
  file.print(failed); file.print(",");
  file.print(batteryVoltage); file.print(",");
  file.print(tempIn); file.print(",");
  file.print(tempOut); file.print(",");
  file.println("");
  
  // close log file
  file.close();
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
  // sketch will just use YYYYMMDD.999 if it has been used more than 999 times
  for (unsigned int counter = 0; counter <= 999; counter++)
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
    if (!SD.exists(filename))
      break;
  }
}

bool gotSignal()
{
  return (digitalRead(DTMF_SIGNAL) == HIGH);
}

byte getPinValue(unsigned int pin)
{
  return ((digitalRead(pin) == HIGH)? 1 : 0);
}

byte readDTMF()
{
  byte pin0 = getPinValue(DTMF0) * B0001;
  byte pin1 = getPinValue(DTMF1) * B0010;
  byte pin2 = getPinValue(DTMF2) * B0100;
  byte pin3 = getPinValue(DTMF3) * B1000;
  return pin0 + pin1 + pin2 + pin3;
}

// match a DTMF tone to the given pattern and cut a line if appropriate
void matchTone(
  byte toneValue,
  const byte* pattern,
  unsigned int* patternPosition,
  unsigned int patternLength,
  int CUT_PIN,
  unsigned long* timeout,
  unsigned int CUT_TIME,
  const char* cutString)
{
  // make sure the current tone isn't the same as the last
  byte lastTone = (*patternPosition > 0)? pattern[(*patternPosition)-1] : toneValue+1;
  if (toneValue == lastTone);
  else if (toneValue == pattern[*patternPosition])  // try to match the received tone
  {
    (*patternPosition)++;
    if (*patternPosition == patternLength)  // if pattern totally matched
    {
      digitalWrite(CUT_PIN, HIGH);  // cut the thing
      *patternPosition = 0;  // reset pattern matching
      *timeout = millis() + CUT_TIME;  // set timeout
      
      // log this event to the SD card
      File file = SD.open(filename, FILE_WRITE);
      file.print(cutString); file.print(".S"); file.println(",");
      file.close();
    }
  }
  else
    *patternPosition = 0;  // reset pattern matching if matching failed
}

// stop cutdown if timeout has been reached
void checkTimeout(
  unsigned long time,
  unsigned long* timeout,
  int CUT_PIN,
  const char* cutString)
{
  if (*timeout != 0 && time >= *timeout)
  {
    digitalWrite(CUT_PIN, LOW);  // stop cutdown
    *timeout = 0;  // reset timeout
    
    // log this event to the SD card
    File file = SD.open(filename, FILE_WRITE);
    file.print(cutString); file.print(".E"); file.println(",");
    file.close();
  }
}

