/*
KarvajFészekHő (Bird Nest Temp)
This sketch is for temperature reading and storing in SD card with the resolution of every minute.
wiring:
˚Arduino pro mini board
˚DS18B20 Temp sensor to pin 4
˚DS3232 RTC trough I2C
˚SD card module trough the miso/mosi/sck/ss(10) pins
˚Battery: 6 NiMH cells (AA size) in series
˚7 IN5408 rectifier diode:
one is series with the arduino board, so the Arduino will shut down when the 6 NiMH cells Output voltage is dropped to 6V, this will protect the cells from over discharging.
the remaining 6 diode is for measuring the remaining Voltage in the battery (they are connected to the  battery's + in series, so we get a voltage, what is decreased by 6 Volts, it will be under 5V (unless it culd be kill the board) and we can measure it with the A0 pin.  
˚LED to pin 9, this will be the display for problems, battery state 
˚ tacticle button to pin 8, when you push it, the arduino measure the battery state then blink it out on the led, so you can see if the board is freezed or the battery is runnning down.

This Sketch is published in the public domain

 Made by: Klucsik Krisztián Pál, 2015_02_03
University Of Pannon, Veszprém Hungary
*/
#include <SPI.h>
#include <SD.h>

#include <OneWire.h>
#define tempnumber 1 //number of ds18b20 chips 
OneWire  ds(4);
byte order[tempnumber];

File myFile;

#define stocksize (5)
float tempstock[stocksize];
int minstock[stocksize];
int d = 0;



#include <Wire.h>
#include <DS3232RTC.h>
#include <Time.h>

#define led 9
#define Ain A0  //the battery Voltage is decreased with a series of diodes by 6 Volts, then the remaining Voltage we measuring and "using", the source is 6 NiMH cells is series, be sure you dont apply to the measuring pin anything above 5 Volts, it will burn your board!
#define button 8

void setup() {
  pinMode(led, OUTPUT);
   pinMode(button, INPUT_PULLUP); 
  delay(4000);
  Serial.begin(9600);
  if (!SD.begin(10)) {
    while (1) {
            digitalWrite(led, HIGH);
      Serial.println("SD error");
      delay(1000);
      if(SD.begin(10)) break;      
    }
  }
  
    char filename[] = "KFH01_00.CSV"; //KarvajFészekHő ( bird nest temperature)
    for (uint8_t i = 0; i < 100; i++) {  //this filenameing section is from the adafruit data logging sketch
      filename[6] = i / 10 + '0';
      filename[7] = i % 10 + '0';
      if (! SD.exists(filename)) {        
        myFile = SD.open(filename, FILE_WRITE);
        Serial.println("SDopened:");
        Serial.println(filename);
        break;
      }
    

    myFile.print(F("Year;Month;Day;Hour;Minute;Temp(C): ")); //header
    ds_setup(tempnumber);
    myFile.println();
    digitalWrite(led, HIGH);
     delay(1000);   
    digitalWrite(led, LOW);
  }
}


void loop() {
  delay(500);  
if(digitalRead(button)==LOW){  //battery charge reading trough Volts, displaying it by led blinks
        int reading= analogRead(Ain); //each blink represent 0,4 V above 6V in the battery
        for(int i=0; i<reading/100; i++){
        digitalWrite(led, HIGH);
         delay(500);       
        digitalWrite(led, LOW);
            delay(500);    
        }
        }
    setSyncProvider(RTC.get);
  if (timeStatus() != timeSet); {  
        Serial.println(F("RTC err"));  // the function to get the time from the RTC
digitalWrite(led,HIGH);
 delay(1000);       
        
}
  while (d < stocksize) {   // read to the memoryuntil the given amount of data is aquaried, this is for saving the SD writes, becouse its consumes a lot of power (according to Adafruit.com) 
    if ( second() < 5) {   //reading every minute
      digitalWrite(13, HIGH);
      delay(5000);
      minstock[d] = minute();
Serial.println(d);
//this section is from the oneWire DS18b200 lib

      byte data[12];
      byte addr[8];

      byte present = 0;
      byte type_s;


      if ( !ds.search(addr)) {
        ds.reset_search();
        delay(250);
      }


      if (OneWire::crc8(addr, 7) != addr[7]) {
        myFile.println("CRC is not valid!");
      //  return;
      }


      // the first ROM byte indicates which chip
      switch (addr[0]) {
        case 0x10:
          type_s = 1;
          break;
        case 0x28:
          type_s = 0;
          break;
        case 0x22:
          type_s = 0;
          break;
        default:
          return;
      }

      ds.reset();
      ds.select(addr);
      ds.write(0x44, 1);        // start conversion, with parasite power on at the end

      delay(1000);     // maybe 750ms is enough, maybe not
      // we might do a ds.depower() here, but the reset will take care of it.

      present = ds.reset();
      ds.select(addr);
      ds.write(0xBE);         // Read Scratchpad

      for (int i = 0; i < 9; i++) {           // we need 9 bytes
        data[i] = ds.read();
      }
      // Convert the data to actual temperature
      // because the result is a 16 bit signed integer, it should
      // be stored to an "int16_t" type, which is always 16 bits
      // even when compiled on a 32 bit processor.
      int16_t raw = (data[1] << 8) | data[0];
      if (type_s) {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) {
          // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      } else {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
      }

      tempstock[d] = (float)raw / 16.0;

d++;
digitalWrite(13, LOW);
    }
  }
  
  for (int i = 0; i < stocksize; i++) {
    myFile.print(year());
    myFile.print(";");
    myFile.print(month());
    myFile.print(";");
    myFile.print(day());
    myFile.print(";");
    myFile.print(hour());
    myFile.print(";");
    myFile.print(minstock[i]);
    myFile.print(";");
    myFile.print(tempstock[i]);
    myFile.print(";");
    myFile.println();
  }
  delay(1000);
  digitalWrite(9, HIGH);
  myFile.flush();
  delay(100);
  digitalWrite(9, LOW);
  Serial.println("it is done");
  d = 0;

}



boolean ds_setup (int tempnum) {
  //DS setup function, this will make the header, we inditify each DS chip with their second addres byte.
  byte addr[8];

  for (int i = 0 ; i < tempnum + 1; i++) {
    if (ds.search(addr)) {

      if (OneWire::crc8(addr, 7) != addr[7]) {
        myFile.println("CRC is not valid!");
        return false;
      }
      else {
        order[i] = addr[1];
      }
    }
  }

  for (int i = 0; i < tempnum; i++) {
    myFile.print(order[i], HEX);
  }
}
