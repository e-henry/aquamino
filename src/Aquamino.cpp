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


// The LM35 temperature sensor data pin is connected
//to the analog 0 on the Arduino
#define AIR_TEMP_PIN 0 //Analog 0
// The DS18B20 is on 8
#define WATER_TEMP_PIN 8 //Digital 8
#define MIN 24.50
#define MAX 25.00
#define WARMER 9 //Digital 9
#define LIGHT 10 //Digital 10
#define HOUR_ON 12
#define HOUR_OFF 22

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(WATER_TEMP_PIN);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

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
boolean bNeedPrintScreen = false;
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


void setup() {
  Serial.begin(57600);

  Serial.print("compiled: ");
  Serial.print(__DATE__);
  Serial.println(__TIME__);

  //--------RTC SETUP ------------
  Rtc.Begin();

  pinMode(WARMER, OUTPUT);
  digitalWrite(WARMER, LOW);
  pinMode(LIGHT, OUTPUT);
  digitalWrite(LIGHT, LOW);

  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);
  Serial.println();

  // Start up the Dallas library
  #if DEBUG
  Serial.println("Init temperature");
  #endif
  sensors.begin();
  sensors.setResolution(11);// 20.125°C

  #if DEBUG
  Serial.println("Init LCD");
  #endif
  prevHour = hour;
  prevMinute = minute;
  lcd.begin();
  lcd.backlight();
  lcd.noCursor();
  lcd.setCursor(0, 0);
  lcd.print("    Aquamino    ");//25/04/2016
  lcd.setCursor(0, 1);
  lcd.print("     v");
  lcd.print(VERSION);
  #if DEBUG
  lcd.print(" DEBUG");
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

  lcd.clear();
  #if DEBUG
  Serial.println("End setup ");
  #endif
}

void light(int iState){
  digitalWrite(LIGHT, iState);
}

void warmer(int iState){
  digitalWrite(ledPin, iState);
  digitalWrite(WARMER, iState);
}

float getAirTemp(){
  float fT;
  analogRead(AIR_TEMP_PIN);
  delay(10);
  fT = analogRead(AIR_TEMP_PIN);        //read the value from the sensor
  fT = (5.0 * fT * 100.0)/1024.0;  //convert the analog data to temperature
  return fT;
  //return (float)(25+(second%10));
}

float getWaterTemp(){
  float fT = 99;
  // Send the command to get temperatures
  sensors.requestTemperatures();
  //Get the temperature from the first sensor found
  #if DEBUG
  Serial.print("Temperature for 1st DS18B20 probe is: ");
  #endif
  fT = sensors.getTempCByIndex(0);
  #if DEBUG
  Serial.println(fT);
  #endif
  return fT;

}

void manageTemperature() {
  fAirTemp = getAirTemp();
  fWaterTemp = getWaterTemp();

  if (fWaterTemp == -127.00 || fWaterTemp == 85.00 || fWaterTemp < 1.00 || year == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.print(fWaterTemp);
    warmer(LOW);
    return;
  }
  if((fWaterTemp < MIN)){
    warmer(HIGH);
    bWarmerOn = true;
    bNeedPrintScreen = true;
  }
  if((fWaterTemp > MAX)){
    warmer(LOW);
    bWarmerOn = false;
    bNeedPrintScreen = true;
  }
}

/*
*****************
* A:20.3  W:25.2
* 18:02 SAT  OFF
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
  lcd.print(" A:");
  dtostrf(fAirTemp, 2, 1, floatBuffer);
  //lcd.print((const char *) floatBuffer, 4);
  lcd.print(fAirTemp);
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
