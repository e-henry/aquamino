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

// Set to 1 to have debug info on serial interface
#define DEBUG 0
//Include Arduino when not using Arduino IDE
#include <Arduino.h>


// include the library code:
#include <SerialLCD.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "Wire.h"

#define DS1307_I2C_ADDRESS 0x68  // This is the I2C address
#if defined(ARDUINO) && ARDUINO >= 100   // Arduino v1.0 and newer
  #define I2C_WRITE Wire.write
  #define I2C_READ Wire.read
#else                                   // Arduino Prior to v1.0
  #define I2C_WRITE Wire.send
  #define I2C_READ Wire.receive
#endif



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
SerialLCD lcd(11,12);//assign soft serial pins Tx Rx


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
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
byte prevHour, prevMinute;
//byte test;
byte zero=0x00;
const char  *Day[] = {"","Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
const char  *Mon[] = {"","Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};

// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}

// Gets the date and time from the ds1307 and prints result
void getTime()
{
  // Reset the register pointer
  Wire.beginTransmission(DS1307_I2C_ADDRESS);
  I2C_WRITE(zero);
  Wire.endTransmission();

  Wire.requestFrom(DS1307_I2C_ADDRESS, 7);

  // A few of these need masks because certain bits are control bits
  second     = bcdToDec(I2C_READ() & 0x7f);
  minute     = bcdToDec(I2C_READ());
  hour       = bcdToDec(I2C_READ() & 0x3f);  // Need to change this if 12 hour am/pm
  dayOfWeek  = bcdToDec(I2C_READ());
  dayOfMonth = bcdToDec(I2C_READ());
  month      = bcdToDec(I2C_READ());
  year       = bcdToDec(I2C_READ());

#if DEBUG
  if (hour < 10)
    Serial.print("0");
  Serial.print(hour, DEC);
  Serial.print(":");
  if (minute < 10)
    Serial.print("0");
  Serial.print(minute, DEC);
  Serial.print(":");
  if (second < 10)
    Serial.print("0");
  Serial.print(second, DEC);
  Serial.print("  ");
  Serial.print(Day[dayOfWeek]);
  Serial.print(", ");
  Serial.print(dayOfMonth, DEC);
  Serial.print(" ");
  Serial.print(Mon[month]);
  Serial.print(" 20");
  if (year < 10)
    Serial.print("0");
  Serial.println(year, DEC);
#endif

}

void setup() {
  Wire.begin();//For the RTC
  Serial.begin(57600);


  pinMode(WARMER, OUTPUT);
  digitalWrite(WARMER, LOW);
  pinMode(LIGHT, OUTPUT);
  digitalWrite(LIGHT, LOW);

  // Start up the Dallas library
  sensors.begin();
  sensors.setResolution(11);// 20.125°C

  prevHour = hour;
  prevMinute = minute;
  lcd.begin();
  lcd.backlight();
  lcd.noCursor();
  lcd.setCursor(0, 0);
  lcd.print("    Aquamino    ");//14/04/2016
  lcd.setCursor(0, 1);
  lcd.print("     v0.2.3");
  #if DEBUG
  lcd.print(" DEBUG");
  #endif
  delay(4000);

  lcd.clear();
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
* A:20.3 W:25.2
* 18:02 SAT OFF
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
  lcd.write((const char *) floatBuffer, 4);
  lcd.print( "  " );
  lcd.print("W:");
  dtostrf(fWaterTemp, 2, 1, floatBuffer);
  lcd.write((const char *) floatBuffer, 4);
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
  lcd.print(" D");
  #endif

}


void loop() {
  delay(1000);
  getTime();

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
