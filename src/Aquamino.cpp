/*
    Aquamino - A simple fresh water tank controller
    Copyright (C) 2016  Eric HENRY

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Project can be found on Github : https://github.com/e-henry/aquamino/

 */

//Include Arduino when not using Arduino IDE
#include <Arduino.h>


// include the library code:
// For the DS18B20 :
#include <OneWire.h>
#include <DallasTemperature.h>
// For the Rtc1307 on arduino SDA and SCL pins :
#include <Wire.h> // must be included here so that Arduino library object file references work
#include <RtcDS1307.h>
#include <LiquidCrystal_I2C.h>

RtcDS1307<TwoWire> Rtc(Wire);

// RtcDS1307 CONNECTIONS:
// DS1307 SDA --> SDA
// DS1307 SCL --> SCL
// DS1307 VCC --> 5v
// DS1307 GND --> GND


// The DS18B20 are on pin 8
#define WATER_TEMP_PIN 8 //Digital 8
#define WARMER 9 //Digital 9
#define LIGHT 10 //Digital 10
#define FAN 11 //Digital 11
#define HOUR_ON 12
#define HOUR_OFF 22

#define TEMPERATURE 25.00
const float MIN = TEMPERATURE - 0.2;
const float MAX = TEMPERATURE;
const float OVERHEAT_TEMP = TEMPERATURE - 2.00;

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(WATER_TEMP_PIN);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
// arrays to hold device addresses
DeviceAddress waterThermometer = { 0x28, 0x5C, 0x1B, 0x28, 0x00, 0x00, 0x80, 0xA1 };
DeviceAddress airThermometer = { 0x28, 0x32, 0x23, 0x28, 0x00, 0x00, 0x80, 0xCB };
// If you are using serial interface LCD
// initialize the LCD library with the SerialLCD lib
//SerialLCD lcd(11,12);//assign soft serial pins Tx Rx (or sda scl)

// set the LCD address to 0x30 for a 16 chars and 2 line display
// (Most I2c lcd use 0x27, find what works for yours)
LiquidCrystal_I2C lcd(0x3f,16,2);

// If you are using a parallel interface LCD
// initialize the LiquidCrystal library with the numbers of the interface pins
//LiquidCrystal lcd(rs, enable, d4, d5, d6, d7)
//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

float fWaterTemp = 0;
float fAirTemp = 0;
boolean bWarmerOn = false;
boolean bLightOn = false;
boolean bFanOn = false;
boolean bNeedPrintScreen = false;
boolean bOverheat = false;
int ledPin =  13;

// Global Variables
byte minute, hour, dayOfWeek, year;
byte prevHour, prevMinute;
//byte test;
byte zero=0x00;
const char  *Day[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
const char  *Mon[] = {"","Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt) {
    char datestring[20];

    snprintf_P(datestring,
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}

void light(int iState){
  digitalWrite(LIGHT, iState);
}

void warmer(int iState){
  digitalWrite(ledPin, iState);
  digitalWrite(WARMER, !iState);//Fixme : Need to know why this relay is reversed
}

void fan(int iState){
  digitalWrite(FAN, !iState);//Fixme : Need to know why this relay is reversed
}

float getTemperature(DeviceAddress deviceAddress){
  float fT = 99;
  //Get the temperature from the first sensor found
  #if DEBUG
  Serial.print("Temperature: ");
  #endif
  fT = sensors.getTempC(deviceAddress);
  #if DEBUG
  Serial.println(fT);
  #endif
  return fT;

}

void manageTemperature() {
  // Send the command to get temperature
  sensors.requestTemperatures();

  fAirTemp = getTemperature(airThermometer);
  fWaterTemp = getTemperature(waterThermometer);

  if (fWaterTemp == -127.00 || fWaterTemp == 85.00 || fWaterTemp < 1.00 || year == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.print(fWaterTemp);
    warmer(LOW);
    return;
  }
  if(fAirTemp > OVERHEAT_TEMP && !bOverheat){
    bOverheat = true;
    //stop the warmer immediately
    warmer(LOW);
    bWarmerOn = false;
    bNeedPrintScreen = true;
  }
  if (fAirTemp < (OVERHEAT_TEMP - 0.5)) {
    bOverheat = false;
    bNeedPrintScreen = true;
  }
  if((fWaterTemp < MIN && !bOverheat)){
    warmer(HIGH);
    bWarmerOn = true;
    bNeedPrintScreen = true;
  }
  if((fWaterTemp > MAX)){
    warmer(LOW);
    bWarmerOn = false;
    bNeedPrintScreen = true;
  }
  if((fWaterTemp > MAX && bOverheat && !bFanOn)){
    fan(HIGH);
    bFanOn = true;
    bNeedPrintScreen = true;
  }
  if((fWaterTemp < MIN && bFanOn)){
    fan(LOW);
    bFanOn = false;
    bNeedPrintScreen = true;
  }

}

/*
*****************
*A:20.25! W:25.50
* 18:02 SAT  OFFD
*****************
*/
void printScreen(){
  char floatBuffer[10];

  if(hour != prevHour){
    prevHour=hour;
    lcd.begin();
    if(bLightOn)
      lcd.backlight();
    else
      lcd.noBacklight();
  }
  //Temperature
  lcd.setCursor(0, 0);
  lcd.print("A:");
  dtostrf(fAirTemp, 2, 1, floatBuffer);
  //lcd.print((const char *) floatBuffer, 4);
  lcd.print(fAirTemp);
  if (bFanOn)
    lcd.print( "* " );
  else if (bOverheat)
    lcd.print( "! " );
  else
    lcd.print( "  " );
  lcd.print("W:");
  dtostrf(fWaterTemp, 2, 1, floatBuffer);
  //lcd.print((const char *) floatBuffer, 4);
  lcd.print(fWaterTemp);
  lcd.print( "  " );


  lcd.setCursor(0, 1);
  lcd.print(" ");
  if (hour < 10)
    lcd.print("0");
  lcd.print((unsigned long)hour, DEC);
  lcd.print(":");
  if (minute < 10)
    lcd.print("0");
  lcd.print((unsigned long)minute, DEC);
  /*lcd.print(":");
  if (second < 10)
    lcd.print("0");
  lcd.print((unsigned long)second, DEC);*/
  lcd.print(" ");
  lcd.print(Day[dayOfWeek]);
  lcd.print("  ");
  //Warmer state
  if(bWarmerOn)
    lcd.print(" ON");
  else
    lcd.print("OFF");
  #if DEBUG
  lcd.print("D");
  #endif

}

void setup() {
  Serial.begin(57600);

  Serial.print("compiled: ");
  Serial.print(__DATE__);
  Serial.println(__TIME__);


  //--------PIN SETUP-------------
  pinMode(WARMER, OUTPUT);
  digitalWrite(WARMER, LOW);
  pinMode(LIGHT, OUTPUT);
  digitalWrite(LIGHT, LOW);
  pinMode(FAN, OUTPUT);
  digitalWrite(FAN, LOW);

  //--------RTC SETUP ------------
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);
  Serial.println();

  // Start up the Dallas library
  #if DEBUG
  Serial.println("Init temperature");
  #endif
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" temperature devices.");
  // report parasite power requirements
  Serial.print("Parasite power is: ");
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
//Uncomment to know the adress of your sensors
/*
  if (!sensors.getAddress(waterThermometer, 0)) Serial.println("Unable to find address for Device 0");
  if (!sensors.getAddress(airThermometer, 1)) Serial.println("Unable to find address for Device 1");

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(waterThermometer);
  Serial.println();

  Serial.print("Device 1 Address: ");
  printAddress(airThermometer);
  Serial.println();
*/

  sensors.setResolution(11);// 20.12°C

  #if DEBUG
  Serial.println("Init LCD");
  #endif
  prevHour = hour;
  prevMinute = minute;
  lcd.begin();
  lcd.backlight();
  lcd.noCursor();
  lcd.setCursor(0, 0);
  lcd.print("    Aquamino    ");
  lcd.setCursor(0, 1);
  #if DEBUG
  lcd.print(__DATE__);
  #else
  lcd.print("     v");
  lcd.print(VERSION);
  #endif

  if (!Rtc.IsDateTimeValid())
  {
      // Common Cuases:
      //    1) first time you ran and the device wasn't running yet
      //    2) the battery on the device is low or even missing
      Serial.println("RTC lost confidence in the DateTime!");

      // following line sets the RTC to the date & time this sketch was compiled
      // it will also reset the valid flag internally unless the Rtc device is
      // having an issue

      Rtc.SetDateTime(compiled);
  }

  if (!Rtc.GetIsRunning())
  {
      Serial.println("RTC was not actively running, starting now");
      Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled)
  {
      Serial.println("RTC is older than compile time!  (Updating DateTime)");
      Rtc.SetDateTime(compiled);
  }
  else if (now > compiled)
  {
      Serial.println("RTC is newer than compile time. (this is expected)");
  }
  else if (now == compiled)
  {
      Serial.println("RTC is the same as compile time! (not expected but all is fine)");
  }

  // never assume the Rtc was last configured by you, so
  // just clear them to your needed state
  Rtc.SetSquareWavePin(DS1307SquareWaveOut_Low);

  delay(4000);
  fan(LOW);
  light(LOW);
  warmer(LOW);

  lcd.clear();
  #if DEBUG
  Serial.println("End setup ");
  #endif
}


void loop() {
  #if DEBUG
  Serial.println("Begin loop ");
  #endif
  delay(1000);
  if (!Rtc.IsDateTimeValid()) {
        // Common Causes:
        //    1) the battery on the device is low or even missing and the power line was disconnected
        Serial.println("RTC lost confidence in the DateTime!");
  }

  RtcDateTime now = Rtc.GetDateTime();
  hour = now.Hour();
  minute = now.Minute();
  dayOfWeek = now.DayOfWeek();
  year = now.Year();

  /**********Light management*********/
  if (hour >= HOUR_ON && hour < HOUR_OFF && !bLightOn) {
    lcd.backlight();
    bLightOn=true;
    light(HIGH);
    bNeedPrintScreen = true;
  }

  if (((hour < HOUR_ON || hour >= HOUR_OFF) && bLightOn)  || year == 0) {
    lcd.noBacklight();
    bLightOn=false;
    light(LOW);
    bNeedPrintScreen = true;
  }

  /******Temperature management******/
  manageTemperature();

  if (minute != prevMinute || bNeedPrintScreen) {
    printScreen();
    prevMinute = minute;
    bNeedPrintScreen = false;
  }
}
