#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include "Adafruit_seesaw.h"
#include <TimeLib.h>

#include <SoftwareSerial.h>

#include <ArduinoJson.h>

#include "LPD8806.h"
#include "SPI.h" // Comment out this line if using Trinket or Gemma
#ifdef __AVR_ATtiny85__
 #include <avr/power.h>
#endif


/*
STEMMA soil sensor in dry air and water:
dry: 320
wet: 540

These dont seem to align with whats seen in the tank, but the potting mix was the same
*/

// constants
const char separator = ':';
const int settingsMemAddress = 0;
const int drySoilCap = 353;
const int wetSoilCap = 540;

// pin defs (more constants)
//const int actionPin = 13;
const int fanControlPin = 3;
const int hcRxD = 7;
const int hcTxD = 6;
const int pumpControlPin = 9;

// settings struct - changing order will change byte offsets of variables, so be careful when modifying
struct Settings {
  int temperature;
  int soilMoisture;
  int soilAcidity;
  int humidity;
  uint32_t color;

  // these relate to daily lighting needs
  float hoursDailyLight;

  /*
  we run days on a 49-day cycle per the arduino `millis` function which rolls over after ~50 days,
  so we chop off the extra 17.04 hours to be just a tad more accurate
  */
  unsigned int lastDay;
  float hoursLit;
  unsigned long lastMeasurementTime;
};

// globals
Settings settings;
int numSerialBytes;
float currSoilTemp;
uint16_t currCapacative;
int currSoilMoistPercent;
float currTemp;
float currHumidity;

unsigned long startLightTime;

unsigned long lastTempCheckTime = 0;

uint32_t start;
uint32_t stop;
Adafruit_SHT31 sht30 = Adafruit_SHT31(); // temp/humidity sensor, I2C

Adafruit_seesaw soilSensor; // soil sensor

SoftwareSerial HC05(hcRxD,hcTxD); // RX, TX

// Number of RGB LEDs in strand, 2 pins for output; can be any valid output pins:
int nLEDs = 14;
int dataPin  = 4;
int clockPin = 5;

// First parameter is the number of LEDs in the strand.  The LED strips
// are 32 LEDs per meter but you can extend or cut the strip.  Next two
// parameters are SPI data and clock pins:
LPD8806 lightStrip = LPD8806(nLEDs, dataPin, clockPin);

uint32_t testColor = lightStrip.Color(127, 0, 127); // Violet

/*
arrays to store timeline data. 
Each array currently stores data sampled in 5 min increments
*/
int minsPerSample = 5;
int samplesPerHour = 60 / minsPerSample; 
int numHours = 1;
const int numIndices = samplesPerHour * numHours;

int currDataIndex = -1;
int indicesWritten = 0;
bool bufferLapped = false;
float *temperatureData = (float*) malloc(sizeof(float)*numIndices);
float *humidityData = (float*) malloc(sizeof(float)*numIndices);
int *soilMoistureData = (int*) malloc(sizeof(int)*numIndices);

/*

*/

void setup() {
  // put your setup code here, to run once:

  // DEBUG MODE
  Serial.begin(9600);
  Wire.begin();

  // read saved settings from EEPROM
  EEPROM.get(0, settings);

  // fan starts OFF
  setFan(0);

  // pump starts OFF
  setPump(false);

  // init soil sensor
  if (!soilSensor.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while(1) delay(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(soilSensor.getVersion(), HEX);
  }

  // set SHT-31 address
  if (!sht30.begin(0x44)) {
    Serial.println("Could not find SHT-30 sensor");
    while (1) delay(1);
  } else {
    Serial.print("SHT-30 started!");
  }

  Wire.setClock(100000);
  uint16_t stat = sht30.readStatus();
  Serial.print(stat, HEX);
  Serial.println();

  Serial.print("Heater Enabled State: ");
  if (sht30.isHeaterEnabled())
    Serial.println("ENABLED");
  else
    Serial.println("DISABLED");


  // should check for issues w/ settings from EEPROM and set to defaults as necessary
  /*
  settings = {
    72,
    0,
    0
  };
  */

  // init to 0
  numSerialBytes = 0;

  // LED action/indicator pins
  //pinMode(actionPin, OUTPUT);

  // software serial pin modes, start serial comm
  pinMode(hcRxD, INPUT);
  pinMode(hcTxD, OUTPUT);
  HC05.begin(9600);


  // LPD8806 LED strip inits
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
  #endif

  // Start up the LED strip
  lightStrip.begin();

  // Update the strip, to start they are all 'off'
  lightStrip.show();

}

void loop() {
  // put your main code here, to run repeatedly:

  // check for BT message in serial buffer
  //numSerialBytes = Serial.available();
  numSerialBytes = Serial.available();
  if (numSerialBytes > 0 || Serial.available() > 0) {
    Serial.println("Recieved data");
    readCommand(false);
  }

  int numSoftBytes = HC05.available();
  if (numSoftBytes > 0) {
    Serial.println("Recieved software data");
    readCommand(true);
  }

  // update day
  int day = getDay();
  if (day != settings.lastDay) {
    // anything we need to do/reset when we hit a new day
    settings.hoursLit = 0;
    startLightTime = 0;
    settings.lastDay = day;
  }

  // check if we should take and record environment measurements
  bool shouldRecord = shouldRecordMeasurement();

  if (shouldRecord) {
    Serial.println("Taking and recording measurements");
    readEnvironmentData();
    saveEnvironmentData();
    printCollectedData();
  }

  // check if we should be lighting
  bool shouldLight = shouldLightEnclosure();
  //Serial.print("Should Light: "); Serial.println(shouldLight);
  if (shouldLight) {
      // if we should be lighting and are not, turn color on and set relevant vars
    if (lightStrip.getPixelColor(0) == 0) {    
      startLightTime = getTime();
      colorWipe(settings.color);
    }

    // if we should be lighting and are, increment the time we have lit
    if (lightStrip.getPixelColor(0) != 0) {
      settings.hoursLit = settings.hoursLit + ((getTime() - startLightTime) / 3600000);
    }
  }

  // if we should not be lighting and are, turn lights off
  else if (!shouldLight && lightStrip.getPixelColor(0) != 0) {
    colorOff();
  }

  // check temp every minute and see if we should be heating or cooling
  //Serial.print(getTime()); Serial.print(" - "); Serial.println(lastTempCheckTime);
  if ((getTime() - lastTempCheckTime) > 60000 || lastTempCheckTime == 0) {
    readTempSensor();
    float currTempFahr = (currTemp * 1.8) + 32;
    //Serial.print("Temp, curr:setting"); Serial.print(currTempFahr); Serial.print(":"); Serial.println(settings.temperature);
    if (currTempFahr < settings.temperature-2) {
      //heat by blowing hot air
      
      //setFan(100)
    }

    else if (currTempFahr > settings.temperature+2) {
      //cool by circulating air
      setFan(100);
    }

    else if (currTempFahr > settings.temperature-2 && currTempFahr < settings.temperature+2) {
      setFan(0);
    }

    lastTempCheckTime = getTime();
  }


  //readTempSensor();
  //printTempAndHumid();

  // TESTING SOIL SENSOR
  //printSoilDetails();
  //delay(1000);

  // print settings
  //printSettings();
  //delay(10000);
  
  // fan usage example
  /*
  setFan(100);
  delay(5000);
  setFan(50);
  delay(5000);
  digitalWrite(actionPin, LOW);
  setFan(0);
  delay(10000);
  */

}

int getDay() {
  unsigned long remainderTime = 61367295; // time over 49 days for 32-but unsigned long
  unsigned long singleDay = 86400000;
  
  unsigned long currTime = millis();


  // if sub 17.04 hours, we know the day is 0. same goes for < a single day's time, which just saves us some math
  if (currTime < singleDay) {
    return 0;
  }

  currTime = currTime - remainderTime;
  unsigned int currDay = currTime / singleDay;

  Serial.print("Got day: "); Serial.println(currDay);

  return currDay;
}

// an extra layer of indirection so we can make changes to later
unsigned long getTime() {
  return millis();
}

// time calculations for whether we should take a measurement right now
bool shouldRecordMeasurement() {
  unsigned long currTime = getTime();

  // if rollover occurred
  if (currTime < settings.lastMeasurementTime) {
    return true;
  }

  unsigned long timeSinceLastMeasurement = currTime - settings.lastMeasurementTime;
  if (timeSinceLastMeasurement > (minsPerSample) * 60000) {
    return true;
  } else {
    return false;
  }
}

bool shouldLightEnclosure() {
  //Serial.print(settings.hoursLit); Serial.print(" < "); Serial.println(settings.hoursDailyLight);
  if (settings.hoursLit < settings.hoursDailyLight) {
    return true;
  }

  else {
    return false;
  }
}

void readEnvironmentData() {
  readTempSensor();
  readSoilSensor();
}

void saveEnvironmentData() {
  currDataIndex = currDataIndex + 1;
  if (currDataIndex > numIndices-1) {
    currDataIndex = 0;
  }

  temperatureData[currDataIndex] = currTemp;
  humidityData[currDataIndex] = currHumidity;
  soilMoistureData[currDataIndex] = currSoilMoistPercent;

  if (indicesWritten < numIndices) {
    indicesWritten = indicesWritten + 1;
  } else {
    bufferLapped = true;
  }

  settings.lastMeasurementTime = getTime();
}

void sendCollectedData(){
  // migrate from static to dynamic JSON docs
  const size_t CAPACITY = JSON_ARRAY_SIZE(30);
  StaticJsonDocument<CAPACITY> doc;
  JsonArray array = doc.to<JsonArray>();

  array.add("Temperature");
  if (bufferLapped) {
    for (int i=currDataIndex+1; i<numIndices; i++) {
      array.add(temperatureData[i]);
    }
  }
  for (int i=0; i<=currDataIndex; i++) {
    //Serial.print("adding "); Serial.println(temperatureData[i]);
    array.add(temperatureData[i]);
  }

  array.add("Humidity");
  if (bufferLapped) {
    for (int i=currDataIndex+1; i<numIndices; i++) {
      array.add(humidityData[i]);
    }
  }
  for (int i=0; i<=currDataIndex; i++) {
    array.add(humidityData[i]);
  }

  array.add("SoilMoisture");
  if (bufferLapped) {
    for (int i=currDataIndex+1; i<numIndices; i++) {
      array.add(soilMoistureData[i]);
    }
  }
  for (int i=0; i<=currDataIndex; i++) {
    array.add(soilMoistureData[i]);
  }

  serializeJson(doc, Serial);
  serializeJson(doc, HC05);

}


void printCollectedData(){
  Serial.print("Temperature: ");

  if (bufferLapped) {
    for (int i=currDataIndex+1; i<numIndices; i++) {
      Serial.print(temperatureData[i]);
      Serial.print(" ");
    }
  }
  for (int i=0; i<=currDataIndex; i++) {
    Serial.print(temperatureData[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Humidity: ");
  if (bufferLapped) {
    for (int i=currDataIndex+1; i<numIndices; i++) {
      Serial.print(humidityData[i]);
      Serial.print(" ");
    }
  }
  for (int i=0; i<=currDataIndex; i++) {
    Serial.print(humidityData[i]);
    Serial.print(" ");
  }
  Serial.println();

  Serial.print("Soil Moisutre: ");
  if (bufferLapped) {
    for (int i=currDataIndex+1; i<numIndices; i++) {
      Serial.print(soilMoistureData[i]);
      Serial.print(" ");
    }
  }
  for (int i=0; i<=currDataIndex; i++) {
    Serial.print(soilMoistureData[i]);
    Serial.print(" ");
  }
  Serial.println();
}

void readTempSensor() {
  sht30.readBoth(&currTemp, &currHumidity);
}

void readSoilSensor() {
  currSoilTemp = soilSensor.getTemp();
  currCapacative = soilSensor.touchRead(0);
  currSoilMoistPercent = (int) map(currCapacative, drySoilCap, wetSoilCap, 0, 100);
}

void printTempAndHumid() {
  Serial.print("Temp: "); Serial.print(currTemp);
  Serial.print("\tHumidity: "); Serial.println(currHumidity);
}

void printSoilDetails() {
  Serial.print("Soil Temperature: "); Serial.print(currSoilTemp); Serial.println("*C");
  Serial.print("Capacitive: "); Serial.println(currCapacative);
  Serial.print("Percent: "); Serial.println(currSoilMoistPercent);
}

void printSettings() {
  Serial.println("Settings----------");
  Serial.print("Temperature: ");
  Serial.println(settings.temperature);
  Serial.print("Soil Moisture: ");
  Serial.println(settings.soilMoisture);
  Serial.print("Soil Acidity: ");
  Serial.println(settings.soilAcidity);
  Serial.print("Humidity: ");
  Serial.println(settings.humidity);
}

void readCommand(bool software) {
  String command;
  String arg;
  if (!software) {
    // read command and arg from serial
    command = Serial.readStringUntil(separator);
    arg = Serial.readStringUntil(separator);
  } else {
    // read command and arg from software serial
    command = HC05.readStringUntil(separator);
    arg = HC05.readStringUntil(separator);
  }

    //String command = HC05.readStringUntil(separator);
    //String arg = HC05.readStringUntil(separator);
    command.trim();
    arg.trim();

    Serial.println("Command: " + command + " Arg: " + arg);

    if (command.equals("SetTemp")) {
      settings.temperature = arg.toInt();
      lastTempCheckTime = 0;
      //Serial.println(settings.temperature);
    }

    else if (command.equals("SetSoilMoist")) {
      settings.soilMoisture = arg.toInt();
      //Serial.println(settings.temperature);
    }

    else if (command.equals("SetSoilAcid")) {
      settings.soilAcidity = arg.toInt();
      //Serial.println(settings.temperature);
    }

    else if (command.equals("SetHoursLight")) {
      settings.hoursDailyLight = arg.toFloat();
    }

    else if (command.equals("SetFan")) {
      setFan(arg.toInt());
    }

    else if (command.equals("SetColorOn")) {
      colorWipe(settings.color);
    }

    else if (command.equals("SetColorOff")) {
      colorOff();
    }

    else if (command.equals("PrintColors")) {
      printPixelColors();
    }

    else if (command.equals("SetPumpOn")) {
      setPump(true);
    }

    else if (command.equals("SetPumpOff")) {
      setPump(false);
    }

    // currently uses 7-bit RGB values
    else if (command.equals("SetColor")) {
      setColor(arg);
    }

    else if (command.equals("GetSnapshot")) {
      // this command should only come over the software serial comms
      StaticJsonDocument<200> doc;
      doc["temperature"] = currTemp;
      doc["humidity"] = currHumidity;
      doc["soilMoisture"] = currSoilMoistPercent;
      serializeJson(doc, Serial);
      serializeJson(doc, HC05);
      //HC05.println();
    }

    else if (command.equals("GetCollectedData")) {
      Serial.println("Sending collected data...");
      sendCollectedData();
    }

    // save new settings to EEPROM
    EEPROM.put(0, settings);


}

void setColor(String arg) {
  // get RGB components from arg variable
  int start = 0;
  int end = arg.indexOf(",", start+1);
  //Serial.print(start); Serial.print(" "); Serial.println(end);
  //Serial.println(arg.substring(start, end));
  
  byte r = (byte) arg.substring(start, end).toInt();

  start = end + 1;
  end = arg.indexOf(",", start+1);
  //Serial.print(start); Serial.print(" "); Serial.println(end);
  //Serial.println(arg.substring(start, end));
  byte g = (byte) arg.substring(start, end).toInt();

  start = end + 1;
  //Serial.print(start); Serial.print(" "); Serial.println(end);
  //Serial.println(arg.substring(start));
  byte b = (byte) arg.substring(start).toInt();

  settings.color = lightStrip.Color(r, g, b);

  // set new color on if color is currently on
  if (lightStrip.getPixelColor(0) != 0) {
    colorWipe(settings.color);
  }
  // sanity check
  /*
  Serial.print("Color Comps: ");
  Serial.print(r); Serial.print(" ");
  Serial.print(g); Serial.print(" ");
  Serial.print(b); Serial.println(" ");
  */
}

void setPump(bool state) {
  if (state == false) {
    Serial.println("Pu,p to OFF");
    digitalWrite(pumpControlPin, LOW);
  } else {
    Serial.println("Pump to ON");
    digitalWrite(pumpControlPin, HIGH);
  }
} 

// Uses PWM to set fan to given % of full speed (e.g. from 0-100, though only 0,40-100 is supported to lengthen fan motor life)
void setFan(int percentSpeed) {
  if (percentSpeed > 100) {
    percentSpeed = 100;
    //Serial.println("Over 100. Step down to 100.");
  } else if (percentSpeed != 0 && percentSpeed < 40) {
    percentSpeed = 40;
   // Serial.println("Under 40. Step up to 40.");
  }
  percentSpeed = (int) map(percentSpeed, 0, 100, 0, 255);
  analogWrite(fanControlPin, percentSpeed);
  //Serial.print("Set fan to ");
  //Serial.println(percentSpeed);
}

// Fill the dots progressively along the strip.
void colorWipe(uint32_t c) {
  Serial.println("Setting color on");
  Serial.println(lightStrip.numPixels());
  int i;

  for (i=0; i < lightStrip.numPixels(); i++) {
      lightStrip.setPixelColor(i, c);
  }
  lightStrip.show();
}

void colorOff() {
  Serial.println("Setting Color Off");
  // Start by turning all pixels off:
  for(int i=0; i<lightStrip.numPixels(); i++) lightStrip.setPixelColor(i, 0);
  lightStrip.show();
}

void printPixelColors() {
  for(int i=0; i<lightStrip.numPixels(); i++) Serial.println(lightStrip.getPixelColor(i));
}
