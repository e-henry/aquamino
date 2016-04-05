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


// include the library code:
#include <SerialLCD.h>
#include <SoftwareSerial.h> //this is a must
#include <OneWire.h>
#include <DallasTemperature.h>

#include "Wire.h"
//Pour savoir la ram utilisée
//#include <MemoryFree.h>

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
#define MIN 24.00
#define MAX 25.00
#define WARMER 9 //Digital 9
#define LIGHT 10 //Digital 10
#define HOUR_ON 12
#define HOUR_OFF 23

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(WATER_TEMP_PIN);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// initialize the LCD library
SerialLCD lcd(11,12);//assign soft serial pins Tx Rx


// initialize the library with the numbers of the interface pins
//LiquidCrystal lcd(rs, enable, d4, d5, d6, d7)
//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

float fWaterTemp=0;
float fAirTemp=0;
boolean bWarmerOn=false;
boolean bLightOn=false;
int ledPin =  13;

// Global Variables
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
byte prechour;
//byte test;
byte zero=0x00;
const char  *Day[] = {"","Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
const char  *Mon[] = {"","Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
 
// Convert normal decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val/10*16) + (val%10) );
}
 
// Convert binary coded decimal to normal decimal numbers
byte bcdToDec(byte val)
{
  return ( (val/16*10) + (val%16) );
}
 
// 1) Sets the date and time on the ds1307
// 2) Starts the clock
// 3) Sets hour mode to 24 hour clock
// Assumes you're passing in valid numbers, Probably need to put in checks for valid numbers.
 
void setTime()                
{
 
   second = (byte) ((Serial.read() - 48) * 10 + (Serial.read() - 48)); // Use of (byte) type casting and ascii math to achieve result.  
   minute = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   hour  = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   dayOfWeek = (byte) (Serial.read() - 48);
   dayOfMonth = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   month = (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   year= (byte) ((Serial.read() - 48) *10 +  (Serial.read() - 48));
   Wire.beginTransmission(DS1307_I2C_ADDRESS);
   I2C_WRITE(zero);
   I2C_WRITE(decToBcd(second) & 0x7f);    // 0 to bit 7 starts the clock
   I2C_WRITE(decToBcd(minute));
   I2C_WRITE(decToBcd(hour));      // If you want 12 hour am/pm you need to set
                                   // bit 6 (also need to change readDateDs1307)
   I2C_WRITE(decToBcd(dayOfWeek));
   I2C_WRITE(decToBcd(dayOfMonth));
   I2C_WRITE(decToBcd(month));
   I2C_WRITE(decToBcd(year));
   Wire.endTransmission();
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
 
  /*if (hour < 10)
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
  */
 
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
  
  prechour=hour;
  lcd.begin();
  lcd.backlight();
  lcd.noCursor();
  lcd.setCursor(0, 0);
  lcd.print("    Aquamino    ");//05/04/2016
  lcd.setCursor(0, 1);
  lcd.print("     v0.2.0 ");
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
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  //Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  //Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  //Serial.print("Temperature for the device 1 (index 0) is: ");
  fT = sensors.getTempCByIndex(0);
  //Serial.println(fT);  
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
  if((fWaterTemp < MIN) && !bWarmerOn){
    warmer(HIGH);
    bWarmerOn = true;
  }
  if((fWaterTemp > MAX) && bWarmerOn){
    warmer(LOW);
    bWarmerOn = false;
  }
}
void loop() {
  delay(1000);//TODO :2s
  getTime();
  
  /**********Light management*********/
  if (hour >= HOUR_ON && hour < HOUR_OFF && !bLightOn) {
    lcd.backlight();
    bLightOn=true;
    light(HIGH);
  }

  if (((hour < HOUR_ON || hour >= HOUR_OFF) && bLightOn)  || year == 0) {
    lcd.noBacklight();
    bLightOn=false;
    light(LOW);
  }
  
  /******Temperature management******/
  manageTemperature();
  affiche();
}



/*
*****************
*29.25°c OFF
*30000m   300kWh
*****************
*/
void affiche(){
  int iConso=0;
  int i=0;
  
  if(hour != prechour){
    prechour=hour;
    lcd.begin();
    if(bLightOn)
      lcd.backlight();
    else
      lcd.noBacklight();
  }
  //Temperature
  lcd.setCursor(0, 0);
  lcd.print("A:");
  lcd.print(fAirTemp, DEC);
  lcd.print( " " );
  lcd.print("W:");
  lcd.print(fWaterTemp, DEC);
  lcd.print( " " );

  //Warmer state
  lcd.setCursor(10, 0);
  if(bWarmerOn)
    lcd.print("ON ");
  else
    lcd.print("OFF");
    
  lcd.setCursor(0, 1);
  //if(second<30){//Display Time
    if (hour < 10)
      lcd.print("0");
    lcd.print((unsigned long)hour, DEC);
    lcd.print(":");
    if (minute < 10)
      lcd.print("0");
    lcd.print((unsigned long)minute, DEC);
    lcd.print(":");
    if (second < 10)
      lcd.print("0");
    lcd.print((unsigned long)second, DEC);
    lcd.print("  ");
    lcd.print(Day[dayOfWeek]);
    lcd.print("   ");
  //}else{
  //Time On
    /*if(bWarmerOn){
      //i=iTimeOn+((millis()/1000)-iStart)/60;
      //lcd.print((unsigned long)i,DEC);
    }
    else
      lcd.print((unsigned long)iTimeOn,DEC);
    lcd.print("m      ");
    lcd.setCursor(10, 1);
    if(bWarmerOn){
      iConso=(i)*40.0/60.0;//40w
    }
    else
      iConso=(iTimeOn)*40.0/60.0;//40w
    lcd.print((unsigned long)iConso,DEC);
    lcd.print("Wh");
    */
  //}
  
}

