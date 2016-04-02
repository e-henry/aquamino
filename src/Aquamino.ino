



/*
 

*/


/*
  
 
 
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



// The temperature sensor data pin is connected 
//to the analog 0 on the Arduino
#define AIR_TEMP_PIN 0 //Analog 0
#define WATER_TEMP_PIN 8 //Digital 8
#define MIN 24.00
#define MAX 25.00
#define WARMER 9 //Digital 9
#define LIGHT 10 //Digital 10
#define HOUR_ON 8
#define HOUR_OFF 20

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(WATER_TEMP_PIN);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// initialize the LCD library
SerialLCD lcd(11,12);//this is a must, assign soft serial pins Tx Rx


// initialize the library with the numbers of the interface pins
//LiquidCrystal lcd(rs, enable, d4, d5, d6, d7)
//LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

float fWaterTemp=0;
float fAirTemp=0;
boolean bOn=false;
boolean bLightOn=false;
int ledPin =  13;

// Global Variables
byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;
byte prechour;
//byte test;
byte zero=0x00;
char  *Day[] = {"","Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
char  *Mon[] = {"","Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
 
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
  lcd.print("    Aquamino    ");//xx/04/2016
  lcd.setCursor(0, 1);
  lcd.print("     v0.1.0 ");
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
    warmer(LOW);
    light(LOW);
  }
    
  
  
  /******Temperature management******/
  fAirTemp = getAirTemp();
  fWaterTemp = getWaterTemp();

  if (fWaterTemp == -127.00 || fWaterTemp == 85.00 || fWaterTemp < 1.00 || year == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.print(fWaterTemp);
    warmer(LOW);
    return;
  }
  if(bLightOn){
    if((fWaterTemp<MIN) && (!bOn)){
      warmer(HIGH);
      //iStart=millis()/1000;
      bOn=true;
      //delay(1000);TODO remettre
    }
    if((fWaterTemp>MAX) && (bOn)){
      warmer(LOW);
      bOn=false;
      //iStop=millis()/1000;
      //iTimeOn+=(iStop-iStart)/60;
      affiche();
      delay(60000);//TODO : do something better
    }
  }
  ////Serial.print("Temp:");
  //Serial.print(fWaterTemp);
  //Serial.print(", Warmer: ");
  /*if(bOn)
    Serial.println("ON");
  else
    Serial.println("OFF");
  */
  
  affiche();
  //delay(1000);
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
  if(bOn)
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
    /*if(bOn){
      //i=iTimeOn+((millis()/1000)-iStart)/60;
      //lcd.print((unsigned long)i,DEC);
    }
    else
      lcd.print((unsigned long)iTimeOn,DEC);
    lcd.print("m      ");
    lcd.setCursor(10, 1);
    if(bOn){
      iConso=(i)*40.0/60.0;//40w
    }
    else
      iConso=(iTimeOn)*40.0/60.0;//40w
    lcd.print((unsigned long)iConso,DEC);
    lcd.print("Wh");
    */
  //}
  
}

