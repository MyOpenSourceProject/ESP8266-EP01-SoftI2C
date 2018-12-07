/************************************************************************
*   ESP8266 Software I2C LCD RTC EEPROM with NTP RTC Time updating
*   File:   main.c
*   Author:  Jithin Krishnan.K
*       Rev. 0.0.1 : 12/11/2018 :  08:26 PM
* 
* This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
* Email: jithinkrishnan.k@gmail.com
*   
************************************************************************/

#include <Wire.h>
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
#include <Time.h>
#include <TimeLib.h>

#include "LiquidCrystal_I2C.h"
#include "DS3231.h"
#include "Eeprom24C32_64.h"


#define SOFT_SDA  0			
#define SOFT_SCL  2
#define EEPROM_ADDRESS 0x57
#define LCD_ADDRESS 0x27

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

Eeprom24C32_64 eeprom(EEPROM_ADDRESS);
DS3231 clk;
LiquidCrystal_I2C lcd(LCD_ADDRESS,20 ,4);
RTCDateTime dt, ist_dt;

char ssid[] = "***********"; // SSID
char pass[] = "***********"; // Wifi Password
unsigned int localPort = 2390;

IPAddress timeServerIP; 
const char* ntpServerName = "time.nist.gov";
const int NTP_PACKET_SIZE = 48;
byte packetBuffer[ NTP_PACKET_SIZE];
//unsigned long ntp_hr, ntp_min, ntp_sec;
char do_once = 0;
uint8_t deg_sym[8]  = {0x6,0x9,0x9,0x6,0x0,0,0,0};

WiFiUDP udp;

void setup()
{
  //Serial.begin(115200);
  clk.begin(SOFT_SDA, SOFT_SCL); // In ESP8266-01, SDA=0, SCL=2   
  lcd.begin(SOFT_SDA, SOFT_SCL);
  eeprom.begin(SOFT_SDA, SOFT_SCL);                  
  lcd.backlight();
  lcd.createChar(0, deg_sym);
  lcd.setCursor(0, 0);
  lcd.print("Wifi Connecting");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  lcd.setCursor(0, 0);
  lcd.print("WiFi Connected!");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  long rssi = WiFi.RSSI();
  lcd.setCursor(13, 1);
  lcd.print(rssi);
  udp.begin(localPort);
  //eeprom.writeByte(0, 0xAA);
  //byte data = eeprom.readByte(0);
}

void loop()
{

  if (!do_once) {
    WiFi.hostByName(ntpServerName, timeServerIP);
    sendNTPpacket(timeServerIP);
    delay(1000);

    int cb = udp.parsePacket();
  
    if (!cb) {
        //lcd.print("00:00:00");
    } else {
    udp.read(packetBuffer, NTP_PACKET_SIZE);
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    const unsigned long seventyYears = 2208988800UL;
    unsigned long epoch = secsSince1900 - seventyYears;
    epoch += 19800UL; // GMT+5:30
        
    ist_dt.hour   = hour(epoch);
    ist_dt.minute = minute(epoch);
    ist_dt.second = second(epoch);
    ist_dt.year   = year(epoch);
    ist_dt.month  = month(epoch);
    ist_dt.day    = day(epoch);
    
    dt = clk.getDateTime(); 
    
    if((ist_dt.year != dt.year || ist_dt.month != dt.month || ist_dt.day != dt.day ||
        ist_dt.hour != dt.hour) || (ist_dt.minute != dt.minute) || (ist_dt.second != dt.second))  {
        clk.setDateTime(ist_dt.year, ist_dt.month, ist_dt.day, ist_dt.hour, ist_dt.minute, ist_dt.second);
    }
   }
   do_once = -1;
  }
  
  dt = clk.getDateTime();
  clk.forceConversion();
  
  lcd.setCursor(0, 2);
  if (dt.day < 10)
     lcd.print('0');
  lcd.print(dt.day);    lcd.print("-");
  if (dt.month < 10)
     lcd.print('0');
  lcd.print(dt.month);  lcd.print("-");
  lcd.print(dt.year);   lcd.print(" ");
  if (dt.hour < 10)
     lcd.print('0');
  lcd.print(dt.hour);   lcd.print(":");
  if (dt.minute < 10)
     lcd.print('0');
  lcd.print(dt.minute); lcd.print(":");
  if (dt.second < 10)
     lcd.print('0');
  lcd.print(dt.second); lcd.print("");
  lcd.setCursor(0, 3);
  lcd.print(clk.readTemperature());
  lcd.printByte(0);
  lcd.print("C");
  delay(500);
}


unsigned long sendNTPpacket(IPAddress& address) {
   memset(packetBuffer, 0, NTP_PACKET_SIZE);
   packetBuffer[0] = 0b11100011;   // LI, Version, Mode
   packetBuffer[1] = 0;     // Stratum, or type of clock
   packetBuffer[2] = 6;     // Polling Interval
   packetBuffer[3] = 0xEC;  // Peer Clock Precision
   packetBuffer[12]  = 49;
   packetBuffer[13]  = 0x4E;
   packetBuffer[14]  = 49;
   packetBuffer[15]  = 52;
   udp.beginPacket(address, 123); //NTP requests are to port 123
   udp.write(packetBuffer, NTP_PACKET_SIZE);
   udp.endPacket();
}


