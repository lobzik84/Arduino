#include "U8glib.h"
#include <idDHTLib.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Time.h>
#include <DS1302.h>

//EEPROM data map
#define EEPROM_MAC_START_ADDR 0x00   //6 bytes
#define EEPROM_IP_START_ADDR 0x06    //4 bytes
#define EEPROM_MASK_START_ADDR 0x0a  //4 bytes
#define EEPROM_GW_START_ADDR 0x0e    //4 bytes
#define EEPROM_CONFIG_BYTE_ADDR 0x12 //1 byte
#define EEPROM_CONTROL_VALUE_ADDR 0xEE

//DS1302 RAM data map
#define RAM_CONTROL_VALUE_ADDR 0x00  //1 byte

// control values to check memory is ok
#define RAM_CONTROL_VALUE 0xEB
#define EEPROM_CONTROL_VALUE 0xEB

#define RTC_CE_PIN 5
#define RTC_IO_PIN 6
#define RTC_SCLK_PIN 7
#define PWR_BTN_PIN 24
#define DOWN_BTN_PIN 26
#define UP_BTN_PIN 28
#define MODE_BTN_PIN 30

#define LCD_RS_PIN 32
#define LCD_RW_PIN 34
#define LCD_E_PIN 36

#define TRIAC_A_PIN 15
#define TRIAC_B_PIN 16
#define TRIAC_C_PIN 17
/*
  Board	          int.0	  int.1	  int.2	  int.3	  int.4	  int.5
 Uno, Ethernet	  2	  3
 Mega2560	  2	  3	  21	  20	  19	  18
 Leonardo	  3	  2	  0	  1
 Due	          (any pin, more info http://arduino.cc/en/Reference/AttachInterrupt)
 */

#define ZCD_A_PIN 21
#define ZCD_A_INT 2 
#define ZCD_B_PIN 20
#define ZCD_B_INT 3
#define ZCD_C_PIN 19 
#define ZCD_C_INT 4

#define DHT22_PIN 18
#define DHT22_INT 5

#define BACKLIGHT_PIN 44

#define POWER_ON_BRIGHTNESS 255
#define STANDBY_BRIGHTNESS 20
#define UPDATE_PERIOD 4000
#define HYSTEREZIS 1
#define MAX_TEMP_SETTING 113
#define MIN_TEMP_SETTING 1

#define SERVER_PORT 23 //telnet server port
#define TEXT_BUFFER_SIZE 39 //length of longest command string plus two spaces for CR + LF

#define TIMER_THRESHOLD 200
#define TRIAC_PULSE_WIDTH_TIMER_PRELOAD 65000

U8GLIB_ST7920_128X64_1X u8g(LCD_E_PIN, LCD_RW_PIN, LCD_RS_PIN);

byte MAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte IP[4] = {192, 168, 0, 10};
byte MASK[4] = {255, 255, 255, 0};
byte GW[4] = {192, 168, 0, 1};
boolean DHCP = true;

//system global devices
DS1302 rtc(RTC_CE_PIN, RTC_IO_PIN, RTC_SCLK_PIN);
EthernetServer server(SERVER_PORT); // Telnet listens on port 23
EthernetClient client = 0; // Client needs to have global scope so it can be called

//buffers
String textBuffer = "";  
byte parsedBytesBuffer[12];

float temperature = 0.0;
float humidity = 0.0;
float tempSetting = 112.0;
boolean heater = false;
boolean powerOn = false;
byte btnState = 0;
long btnChanged = 0;
long updated = 0;

volatile unsigned int phaseTime = 0;

void dhtLib_wrapper(); // must be declared before the lib initialization

idDHTLib DHTLib(DHT22_PIN, DHT22_INT, dhtLib_wrapper);
String error = "";

void setup()
{
  pinMode(TRIAC_A_PIN, OUTPUT);
  digitalWrite(TRIAC_A_PIN, LOW);
  pinMode(TRIAC_B_PIN, OUTPUT);
  digitalWrite(TRIAC_B_PIN, LOW);
  pinMode(TRIAC_C_PIN, OUTPUT);
  digitalWrite(TRIAC_C_PIN, LOW);

  digitalWrite(ZCD_A_PIN, HIGH);
  digitalWrite(ZCD_B_PIN, HIGH);
  digitalWrite(ZCD_C_PIN, HIGH);
  
  pinMode(PWR_BTN_PIN, INPUT_PULLUP);
  pinMode(UP_BTN_PIN, INPUT_PULLUP);
  pinMode(DOWN_BTN_PIN, INPUT_PULLUP);
  
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 1;            
  TCCR1B |= (1 << CS11);    // 8 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 0;
  OCR3B = 0;
  OCR3C = 0;
  
  TCCR3B |= (0 << WGM12);
  TCCR3B |= (1 << CS11);    // 8 prescaler 
  TIMSK3 |= (1 << OCIE3A);   
  TIMSK3 |= (1 << OCIE3B);   
  TIMSK3 |= (1 << OCIE3C);   

  TCCR4A = 0;
  TCCR4B = 0;
  TCNT4 = 0;
  OCR4A = 0;
  OCR4B = 0;
  OCR4C = 0;
  
  TCCR4B |= (0 << WGM12);
  TCCR4B |= (1 << CS11);    // 8 prescaler 
  TIMSK4 |= (1 << OCIE4A);   
  TIMSK4 |= (1 << OCIE4B);   
  TIMSK4 |= (1 << OCIE4C);   

  interrupts(); 

  attachInterrupt(ZCD_A_INT, zcA, CHANGE);
  attachInterrupt(ZCD_B_INT, zcB, CHANGE);
  attachInterrupt(ZCD_C_INT, zcC, CHANGE);

  Serial.begin(9600);
  if (checkEEPROM())
    readNetworkSettings();
  initNetwork();
  checkRTC();
  if (checkEEPROM())
    readNetworkSettings();
  setSyncProvider(getRTCtime);   // the function to get the time from the RTC
  if(timeStatus()!= timeSet) Serial.println ("Unable to sync with the RTC");
  delay(100);
  
  byte phaseCheck = 0;
  if (OCR3A == 0 && OCR4A == 0)
    error = "no phase A sync";
  else if (OCR3B == 0 && OCR4B == 0)
    error = "no phase B sync";
  else if (OCR3C == 0 && OCR4C == 0)
    error = "no phase C sync";
  else
  {
    phaseCheck = 1;
    unsigned int timediffB = OCR3B - OCR3A;
    unsigned int timediffC = OCR3C - OCR3A;
    if (timediffB > timediffC)
    {
      error = "phase swap";
      phaseCheck = 2;
    }
  }
  Serial.println(error);
}

void dhtLib_wrapper() 
{
  DHTLib.dht22Callback(); 
}

void zcA()
{
  if (digitalRead(ZCD_A_PIN))
    OCR3A = TCNT3 + phaseTime;
  else
    OCR4A = TCNT4 + phaseTime;
}

void zcB()
{
  if (digitalRead(ZCD_B_PIN))
    OCR3B = TCNT3 + phaseTime;
  else
    OCR4B = TCNT4 + phaseTime;
}

void zcC()
{
  if (digitalRead(ZCD_C_PIN))
    OCR3C = TCNT3 + phaseTime;
  else
    OCR4C = TCNT4 + phaseTime;
}

ISR(TIMER1_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  digitalWrite(TRIAC_A_PIN, false);
  digitalWrite(TRIAC_B_PIN, false);
  digitalWrite(TRIAC_C_PIN, false);
  TCNT1 = 1;
}

ISR(TIMER3_COMPA_vect)
{
  if (powerOn)
  {
    digitalWrite(TRIAC_A_PIN, true);
    //digitalWrite(TRIAC_C_PIN, true);
    TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
  }
}

ISR(TIMER3_COMPB_vect)
{
  if (powerOn) 
  {
    digitalWrite(TRIAC_B_PIN, true);
    //digitalWrite(TRIAC_A_PIN, true);
    TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
  }
}

ISR(TIMER3_COMPC_vect)
{
  if (powerOn) 
  {
    digitalWrite(TRIAC_C_PIN, true);
    //digitalWrite(TRIAC_B_PIN, true);
    TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
  }
}

ISR(TIMER4_COMPA_vect)
{
  if (powerOn)
  {
    digitalWrite(TRIAC_A_PIN, true);
    //digitalWrite(TRIAC_C_PIN, true);
    TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
  }
}

ISR(TIMER4_COMPB_vect)
{
  if (powerOn) 
  {
    digitalWrite(TRIAC_B_PIN, true);
    //digitalWrite(TRIAC_A_PIN, true);
    TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
  }
}

ISR(TIMER4_COMPC_vect)
{
  if (powerOn) 
  {
    digitalWrite(TRIAC_C_PIN, true);
    //digitalWrite(TRIAC_B_PIN, true);
    TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
  }
}


void loop()
{
  handleButtons();
  phaseTime = tempSetting * 170;
  if (server.available()) client = server.available();
  
  if (client.connected() && client.available()) readCommand(&client);
  if (Serial.available()) readCommand(&Serial);
 
  if (millis() - updated > UPDATE_PERIOD)
  {
    updated = millis();
    DHTLib.acquire();
    long acquireStart = millis();
    while (DHTLib.acquiring() && millis() - acquireStart < 1000)
      ;
    int result = DHTLib.getStatus();
    if  (millis() - acquireStart >= 1000) result = IDDHTLIB_ERROR_TIMEOUT;
    
    if (result == IDDHTLIB_OK)
    {
      humidity = DHTLib.getHumidity(); 
      temperature = DHTLib.getCelsius();
      error = "";
      Serial.println("got ok");
    }
    else
    {
      switch (result)
      {
      case IDDHTLIB_ERROR_CHECKSUM: 
        error = "Checksum error"; 
        break;
      case IDDHTLIB_ERROR_TIMEOUT: 
        error = "Time out error"; 
        break;
      case IDDHTLIB_ERROR_ACQUIRING: 
        error = "Acquiring"; 
        break;
      case IDDHTLIB_ERROR_DELTA: 
        error = "Delta time to small"; 
        break;
      case IDDHTLIB_ERROR_NOTSTARTED: 
        error = "Not started"; 
        break;
      default: 
        error = "Unknown error"; 
        break;
      }
      Serial.print("DHT22 error: "); Serial.println(error);
    }
    //responseln(error);
    //handleButtons();
    //handleHeater();
    draw();
    //handleButtons();
    /*
    volatile unsigned int timediffB = crBR - crAR;
    volatile unsigned int timediffC = crCR - crAR;
    char bbuf[11];
        
    dtostrf(timediffB, 5, 0, bbuf);
    Serial.print(bbuf);
    Serial.print(" and timeC ");
    dtostrf(timediffC, 5, 0, bbuf);
    Serial.print(bbuf);
    if (timediffB > timediffC ) Serial.print(" SWAP!");
    Serial.println();*/
  }
}


void handleHeater()
{
  if (!powerOn)
   {
     heater = false;
     return;
   }
  if (heater && temperature >= tempSetting + HYSTEREZIS)
  {
    heater = false;
    responseln("DEBUG: heater off");
  }
  if (!heater && temperature <= tempSetting - HYSTEREZIS)
  {
    heater = true;
    responseln("DEBUG: heater on");
  }
}

void handleButtons()
{
  byte state = 0;
  if (!digitalRead(PWR_BTN_PIN))
    bitSet(state, 0);
  if (!digitalRead(UP_BTN_PIN))
    bitSet(state, 1);
  if (!digitalRead(DOWN_BTN_PIN))
    bitSet(state, 2);
  if (state != btnState)
  {
    btnState = state;
    btnChanged = millis();
  }
  if (btnState != 0 && millis() - btnChanged > 50)
  {
    if (bitRead(btnState, 0) && millis() - btnChanged > 1000)
    {
      if (powerOn)
        powerOn = false;
      else
        powerOn = true;
      updated = 0;
    }
    if (bitRead(btnState, 1))
    {
      tempSetting++;
      if (tempSetting >= MAX_TEMP_SETTING)
        tempSetting = MAX_TEMP_SETTING;
    }
    if (bitRead(btnState, 2))
    {
      tempSetting--; 
      if (tempSetting <= MIN_TEMP_SETTING)
        tempSetting = MIN_TEMP_SETTING;
    }
    draw();
    delay(300);
  }
}

void draw() 
{
  char charbuf[25];
  char buf[25];
  u8g.firstPage();  
  do 
  {
    if (powerOn)
    {
      analogWrite(BACKLIGHT_PIN, POWER_ON_BRIGHTNESS);  
      if (error == "")
      {
        dtostrf(temperature, 2, 1, charbuf); //measured temperature
        sprintf(buf, "%s\xB0\x43", charbuf);
        u8g.setFont(u8g_font_fub25);
        u8g.drawStr( temperature<100?13:0, 26, buf);
      }
      else
      {
        sprintf(buf, "ERROR!", charbuf);
        u8g.setFont(u8g_font_8x13);
        u8g.drawStr( 25, 10, buf);
        
        error.toCharArray(charbuf, 25);
        sprintf(buf, "%s", charbuf);
        u8g.setFont(u8g_font_8x13);
        u8g.drawStr( 5, 25, buf);
      }
      
      dtostrf(tempSetting, 2, 1, charbuf); //temperature preset
      sprintf(buf, "%s\xB0\x43", charbuf);
      u8g.setFont(u8g_font_8x13);
      u8g.drawStr( tempSetting<100?78:70, 39, buf);
      
      dtostrf(humidity, 3, 0, charbuf); //humidity
      sprintf(buf, "%s%%", charbuf); 
      u8g.setFont(u8g_font_8x13);
      u8g.drawStr( 5, 39, buf);
      
      sprintf(buf, "%d:%02d", hour(), minute()); //time
      u8g.setFont(u8g_font_fub20);
      u8g.drawStr( 25, 64, buf); 
     
      if (heater)
      {
        sprintf(buf, "\x2F");// heater symbol
        u8g.setFont(u8g_font_10x20_67_75);
        u8g.drawStr( 117, 64, buf);
      }
    }
    else
    {
      analogWrite(BACKLIGHT_PIN, STANDBY_BRIGHTNESS);
      if (error == "")
      {
        sprintf(buf, "%d:%02d", hour(), minute()); //time
        u8g.setFont(u8g_font_fub25);
        u8g.drawStr( 19, 42, buf);
        char bbuf[11];
        sprintf(bbuf, "%d.%02d.%d", year(), month(), day()); //date
        u8g.setFont(u8g_font_8x13);
        u8g.drawStr( 24, 63, bbuf);
      }
      else
      {
        sprintf(buf, "ERROR!", charbuf);
        u8g.setFont(u8g_font_8x13);
        u8g.drawStr( 25, 10, buf);
        
        error.toCharArray(charbuf, 25);
        sprintf(buf, "%s", charbuf);
        u8g.setFont(u8g_font_8x13);
        u8g.drawStr( 5, 25, buf);
      }
    }
   } 
   while( u8g.nextPage() );
}

void initNetwork()
{
  if (DHCP) //if DHCP is enabled
  {
     // start the Ethernet connection using just MAC, with DHCP:
    if (Ethernet.begin(MAC) == 0)  
      Serial.println("Failed to configure Ethernet using DHCP");
    else
    {
      // print your local IP address:
      Serial.print("My IP address: ");
      for (byte thisByte = 0; thisByte < 4; thisByte++) 
      {
        // print the value of each byte of the IP address:
        Serial.print(Ethernet.localIP()[thisByte], DEC);
        Serial.print("."); 
      }
      Serial.println(); 
    }
  }
  else //if DHCP disabled
  {
    //configuring static IP and start the Ethernet connection
    Ethernet.begin(MAC, IP, GW, MASK);     
  }
  server.begin();
}

boolean checkRTC()
{
  byte controlByte = rtc.peek(RAM_CONTROL_VALUE_ADDR);
  if (controlByte != RAM_CONTROL_VALUE)
  {
    responseln ("ERROR: RTC Control byte check failed! New or unitialized RTC detected. Initializing... Please set Time!");
    rtc.halt(false);
    rtc.writeProtect(false);
    for (int i = 0; i < 31; i++) rtc.poke(i, 0x00);
    rtc.poke(RAM_CONTROL_VALUE_ADDR, RAM_CONTROL_VALUE);
    return false;
  }
  else return true;
}

boolean checkEEPROM()
{
  byte controlByte = EEPROM.read(EEPROM_CONTROL_VALUE_ADDR);
  if (controlByte != EEPROM_CONTROL_VALUE)
  {
    Serial.println("ERROR: EEPROM Control byte check failed! New EEPROM detected. Default values loaded... Check your settings!");
    writeNetworkSettings();
    EEPROM.write(EEPROM_CONTROL_VALUE_ADDR, EEPROM_CONTROL_VALUE);
    return false;
  }
  else return true;
}

void readCommand(Stream *str)
{
  char c;
  while (str->available())
  {
    c = str->read();
    if (c > 31 && c < 126) //filtering bullshit out
      textBuffer += String(c);
    if(c == 0x0d) 
    {
     if (textBuffer.length() > 0) 
     {
       printEcho();
       parseReceivedText();
     }
      textBuffer="";
      str->flush(); //flushing everything else
    }
    if (textBuffer.length() > TEXT_BUFFER_SIZE)
    {
      printErrorMessage();
      textBuffer="";
      str->flush(); //flushing everything else
      break;
    }
  }
}

void parseReceivedText()
{
  textBuffer.trim();
  textBuffer.toLowerCase();
  if (textBuffer.substring(0,2) == "ps")
  {//print state
      char bbuf[30];
      sprintf(bbuf, "date: %d.%02d.%d %d:%02d:%02d", year(), month(), day(), hour(), minute(), second()); //time
      responseln(bbuf);
      response("temperature setting: ");
      dtostrf(tempSetting, 2, 1, bbuf);
      responseln(bbuf);
      response("measured temperature: ");
      dtostrf(temperature, 2, 1, bbuf);
      responseln(bbuf);
      response("measured humidity: ");
      dtostrf(humidity, 2, 1, bbuf);
      responseln(bbuf);
      response("power: ");
      responseln(powerOn?"ON":"OFF");
      response("heater: ");
      responseln(heater?"ON":"OFF");
      printOK();
  }
  else if (textBuffer.substring(0,4) == "sts=")
  {
    textBuffer = textBuffer.substring(4);
    int value = parseDigits();
    if (value >= MIN_TEMP_SETTING && value <= MAX_TEMP_SETTING)
    {
      tempSetting = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,4) == "pwr0")
  {
    powerOn = false;
    printOK();
  }
  else if (textBuffer.substring(0,4) == "pwr1")
  {
    powerOn = true;
    printOK();
  }
  else if (textBuffer.substring(0,3) == "snp")
  {//set network parameters
    textBuffer = textBuffer.substring(3);
    doNetworkSettings();
    writeNetworkSettings();
  }
  else if (textBuffer.substring(0,3) == "pns")
  {//print 
    printNetworkSettings();
  }
  else if (textBuffer.substring(0,2) == "st")
  {
    textBuffer = textBuffer.substring(2);
    doTimeSettings();
  }
  else if (textBuffer.substring(0,2) == "cl")
  {
    closeConnection();
  }
  else if (textBuffer.substring(0,1) == "?")
  {
    printHelpMessage();
  }
  else printErrorMessage();
 }

void printNetworkSettings()
{
  response("MAC: ");
  for (int i = 0; i < 6; i++) response(String(MAC[i], HEX) + ":");
  responseln("");
  response("DHCP: ");
  if (DHCP)
    responseln("ON");
  else
    responseln("OFF");
  response("IP: ");
  for (int i = 0; i < 4; i++) response(String(IP[i], DEC) + ".");
  responseln("");
  response("MASK: ");
  for (int i = 0; i < 4; i++) response(String(MASK[i], DEC) + ".");
  responseln("");
  response("GW: ");
  for (int i = 0; i < 4; i++) response(String(GW[i], DEC) + ".");
  responseln("");
  printOK();
}

void doTimeSettings()
{
  int value = 0;
  tmElements_t setTm;
  breakTime(now(), setTm);
  if (textBuffer.substring(0,5) == "year=")
  {
    textBuffer = textBuffer.substring(5);
    value = parseDigits();
    if (value >= 2000 && value <= 2099)
    {
      setTm.Year = value - 1970;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,6) == "month=")
  {
    textBuffer = textBuffer.substring(6);
    value = parseDigits();
    if (value >= 1 && value <= 12)
    {
      setTm.Month = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,4) == "day=")
  {
    textBuffer = textBuffer.substring(4);
    value = parseDigits();
    if (value >= 1 && value <= 31)
    {
      setTm.Day = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "hour=")
  {
    textBuffer = textBuffer.substring(5);
    value = parseDigits();
    if (value >= 0 && value <= 23)
    {
      setTm.Hour = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,4) == "min=")
  {
    textBuffer = textBuffer.substring(4);
    value = parseDigits();
    if (value >= 0 && value <= 59)
    {
      setTm.Minute = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,4) == "sec=")
  {
    textBuffer = textBuffer.substring(4);
    value = parseDigits();
    if (value >= 0 && value <= 59)
    {
      setTm.Second = value;
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,9) == "unixsecs=")
  {
    textBuffer = textBuffer.substring(9);
    long unixsecs = parseLongDigits();
    if (unixsecs > 0)
    {
      breakTime(unixsecs, setTm);
      printOK();
    }
    else
     printErrorMessage();
  }
  else
     printErrorMessage();
  rtc.setDOW(setTm.Wday);       
  rtc.setTime(setTm.Hour, setTm.Minute, setTm.Second);
  rtc.setDate(setTm.Day, setTm.Month, tmYearToCalendar(setTm.Year)); //adjust time in RTC first 

  setTime(getRTCtime()); //then update local 
}

void doNetworkSettings()
{
  if (textBuffer.substring(0,4) == "mac=")
  {
    //set mac
    textBuffer = textBuffer.substring(4);
    if (parseBytes(6))
    {
      for(int i = 0; i < 6; i++) MAC[i] = parsedBytesBuffer[i];
      printOK();
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "dhcp=")
  {
    //set dhcp
    textBuffer = textBuffer.substring(5);
    if (textBuffer == "on")
    {
      DHCP = true;
      printOK();
    }
    else if (textBuffer == "off")
    {
      DHCP = false;
      printOK();
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,3) == "ip=")
  {
    //set ip
    textBuffer = textBuffer.substring(3);
    if (parseDecAddr())
    {
      for(int i = 0; i < 4; i++) IP[i] = parsedBytesBuffer[i];
      printOK();
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "mask=")
  {
    //set mask
    textBuffer = textBuffer.substring(5);
    if (parseDecAddr())
    {
      for(int i = 0; i < 4; i++) MASK[i] = parsedBytesBuffer[i];
      printOK();
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,3) == "gw=")
  {
    //set gateway
    textBuffer = textBuffer.substring(3);
    if (parseDecAddr())
    {
      for(int i = 0; i < 4; i++) GW[i] = parsedBytesBuffer[i];
      printOK();
    }
    else
      printErrorMessage();
  }
  
  else
      printErrorMessage();

}

boolean parseBytes(int numBytes)
{
  String strByte;
  int bytes = 0;
  while (textBuffer.indexOf(":") > 0 && bytes < numBytes)
  {
    int pos = textBuffer.indexOf(":");
    strByte = textBuffer.substring(0,pos);
    textBuffer = textBuffer.substring(pos+1);
    int oneByte = parseByte(strByte);
    if (oneByte < 0) return false;
    parsedBytesBuffer[bytes] = oneByte;
    bytes++;
  }
  if (bytes != numBytes || textBuffer.indexOf(":") > 0) return false;
  return true;
}

boolean parseDecAddr()
{
  String strByte;
  int bytes = 0;
  for (int i = 0; i < 4; i++)
  {
    int oneByte = parseDigits();
    if (oneByte < 0 || oneByte > 255) return false;
    parsedBytesBuffer[i] = oneByte;
    textBuffer = textBuffer.substring(1);
  }
  return true;
}


int parseDigit(char c)
{
  int digit = -1;
  digit = (int) c - 0x30; // subtracting 0x30 from ASCII code gives value
  if(digit < 0 || digit > 9) digit = -1;
  return digit;
}

long parseLongDigits() //parses buffer from begin, shrinks it till first non-digit, returns number;
{
  long num = -1;
  int digit = 0;
  for (int i=0; i < textBuffer.length(); i++)
  {
    digit = parseDigit(textBuffer.charAt(i));
    if (digit < 0) 
    {
      textBuffer = textBuffer.substring(i);
      break;
    }
    if (num == -1) num = 0;
    num = num*10 + digit;
  }
  return num;
}

int parseDigits() //parses buffer from begin, shrinks it till first non-digit, returns number;
{
  int num = -1;
  int digit = 0;
  for (int i=0; i < textBuffer.length(); i++)
  {
    digit = parseDigit(textBuffer.charAt(i));
    if (digit < 0) 
    {
      textBuffer = textBuffer.substring(i);
      break;
    }
    if (num == -1) num = 0;
    num = num*10 + digit;
  }
  return num;
}

int parseByte(String byteStr)
{
  int oneByte = 0;
  if (byteStr.length() == 1)
  {
    oneByte = (int)byteStr.charAt(0) - 0x30;
    if (oneByte < 0)
      return -1;
    if (oneByte > 9)
    {
      oneByte = (int)byteStr.charAt(0) - 0x57;
      if (oneByte < 0x0A || oneByte > 0x0F) return -1;
    }
    return oneByte;
  }
  else if (byteStr.length() == 2)
  {
    oneByte = (int)byteStr.charAt(0) - 0x30;
    if (oneByte < 0)
      return -1;
    if (oneByte > 9)
    {
      oneByte = (int)byteStr.charAt(0) - 0x57;
      if (oneByte < 0x0A || oneByte > 0x0F) return -1;
    }
    int secondByte = (int)byteStr.charAt(1) - 0x30;
    if (secondByte < 0)
      return -1;
    if (secondByte > 9)
    {
      secondByte = (int)byteStr.charAt(1) - 0x57;
      if (secondByte < 0x0A || secondByte > 0x0F) return -1;
    }
    return (oneByte*16 + secondByte);
  }
  return -1;
}

void printEcho()
{
  responseln(textBuffer); //printing command back to all clients
}

void printOK()
{
  responseln(" OK");
}

void printErrorMessage()
{
  responseln("ERROR: unrecognized command");
}

void closeConnection()
{
  responseln("\nBye.\n");
  client.stop();
}


void readNetworkSettings()
{
  for (int i = 0; i < 6; i++) MAC[i] = EEPROM.read(EEPROM_MAC_START_ADDR + i); //reading 6 bytes of MAC from EEPROM
  for (int i = 0; i < 4; i++) IP[i] = EEPROM.read(EEPROM_IP_START_ADDR + i);
  for (int i = 0; i < 4; i++) MASK[i] = EEPROM.read(EEPROM_MASK_START_ADDR + i);
  for (int i = 0; i < 4; i++) GW[i] = EEPROM.read(EEPROM_GW_START_ADDR + i);  
  DHCP = bitRead(EEPROM.read(EEPROM_CONFIG_BYTE_ADDR), 0);
}


void writeNetworkSettings()
{
  for (int i = 0; i < 6; i++) EEPROM.write(EEPROM_MAC_START_ADDR + i, MAC[i]); //reading 6 bytes of MAC from EEPROM
  for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_IP_START_ADDR + i, IP[i]);
  for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_MASK_START_ADDR + i, MASK[i]);
  for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_GW_START_ADDR + i, GW[i]); 
  byte romByte = EEPROM.read(EEPROM_CONFIG_BYTE_ADDR);
  bitWrite(romByte, 0, DHCP);
  EEPROM.write(EEPROM_CONFIG_BYTE_ADDR, romByte);

}

unsigned long getRTCtime()
{
  Time t = rtc.getTime();
  
  tmElements_t rtcTm;
  rtcTm.Year = t.year -1970;
  rtcTm.Month = t.mon;
  rtcTm.Day = t.date;
  rtcTm.Hour = t.hour;
  rtcTm.Minute = t.min;
  rtcTm.Second = t.sec;
  
  unsigned long tmil = makeTime(rtcTm);
 
  return tmil;
}

void responseln(String text)
{
  server.println(text);
  Serial.println(text);
}

void response(String text)
{
  server.print(text);
  Serial.print(text);
}

void printHelpMessage()
{
  responseln("Supported commands:");
  responseln("");
  responseln(" sr                        - start recharge");
  responseln("");
  responseln(" sp                        - set configuration parameters. ");
  responseln("                             Device reboot needed for settings to take effect.");
  responseln(" spMAC=0A:0B:0B:0D:0E:0F   - set 6 bytes of MAC address");
  responseln(" spIP=192.168.0.3");
  responseln(" spMASK=255.255.255.0");
  responseln(" spGW=192.168.0.1          - set ip, subnet mask, default gateway");
  responseln(" spDHCP=ON                 - set DHCP ON. If DHCP is ON, IP ");
  responseln("                             obtained from DHCP server automatically on");
  responseln("                             power-on, and IP, GW, MASK settings are ignored.");
  responseln("");
  responseln(" pc                        - print configuration parameters");
  responseln(" cl                        - close connection");
  responseln(" ?                         - print this help message");
  responseln("");
  responseln("  Commands are not case sensitive.");
  responseln("");

}
