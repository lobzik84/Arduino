#include "U8glib.h"
#include <idDHTLib.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Time.h>
#include <DS1302.h>
#include <PID_v1.h>

//EEPROM data map
#define EEPROM_MAC_START_ADDR 0x00   //6 bytes
#define EEPROM_IP_START_ADDR 0x06    //4 bytes
#define EEPROM_MASK_START_ADDR 0x0a  //4 bytes
#define EEPROM_GW_START_ADDR 0x0e    //4 bytes
#define EEPROM_CONFIG_BYTE_ADDR 0x12 //1 byte
#define EEPROM_MODE_ADDR 0x20
#define EEPROM_KI_ADDR 0x21
#define EEPROM_KP_ADDR 0x22
#define EEPROM_KD_ADDR 0x23

#define EEPROM_CONTROL_VALUE_ADDR 0xEE

//DS1302 RAM data map
#define RAM_CONTROL_VALUE_ADDR 0x00  //1 byte
#define RAM_TEMP_SETTING_START_ADDR 0x01  //4 bytes

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
 Mega2560	  2	  3	  21	  20	  19	  18
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
#define UPDATE_PERIOD 2000
#define HYSTEREZIS 1
#define MAX_TEMP_SETTING 113
#define MIN_TEMP_SETTING 1

#define SERVER_PORT 23 //telnet server port
#define TEXT_BUFFER_SIZE 39 //length of longest command string plus two spaces for CR + LF

#define TRIAC_PULSE_WIDTH_TIMER_PRELOAD 65200
#define TRIAC_PULSE_RELAY_TIMER_OCR 2000
#define MINIMUM_PHASE_PERIOD 16000

#define MODE_RELAY 0
#define MODE_PIM_MAN 1
#define MODE_PIM_PID 2

#define AVERAGES_COUNT 20

U8GLIB_ST7920_128X64_1X u8g(LCD_E_PIN, LCD_RW_PIN, LCD_RS_PIN);

void dhtLib_wrapper(); // must be declared before the lib initialization
idDHTLib DHTLib(DHT22_PIN, DHT22_INT, dhtLib_wrapper);

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

double temperature = 0.0;
double humidity = 0.0;
double tempSetting = 35.0;
byte manPimSetting = 1;
boolean heater = false;
boolean powerOn = false;
byte btnState = 0;
long btnChanged = 0;
long updated = 0;
String error = "";

byte mode = MODE_PIM_PID;

volatile unsigned int phaseTime = 0;
volatile unsigned int periodA = 0;
unsigned int phasePowerMap[256] = {19400, 17769, 17339, 17035, 16791, 16584, 16402, 16238, 16088, 15950, 15821, 15700, 15585, 15476, 15372, 15272, 15176, 15084, 14995, 14909, 14825, 14744, 14665, 14588, 14513, 14440, 14368, 14298, 14229, 14162, 14096, 14031, 13967, 13904, 13843, 13782, 13722, 13663, 13605, 13547, 13491, 13435, 13380, 13325, 13271, 13218, 13165, 13113, 13061, 13010, 12960, 12910, 12860, 12811, 12762, 12713, 12665, 12618, 12570, 12524, 12477, 12431, 12385, 12339, 12294, 12249, 12204, 12160, 12116, 12072, 12028, 11985, 11941, 11898, 11856, 11813, 11771, 11729, 11687, 11645, 11603, 11562, 11521, 11479, 11439, 11398, 11357, 11317, 11276, 11236, 11196, 11156, 11116, 11076, 11037, 10997, 10958, 10919, 10879, 10840, 10801, 10762, 10724, 10685, 10646, 10608, 10569, 10530, 10492, 10454, 10415, 10377, 10339, 10301, 10263, 10224, 10186, 10148, 10110, 10072, 10035, 9997, 9959, 9921, 9883, 9845, 9807, 9769, 9732, 9694, 9656, 9618, 9580, 9542, 9504, 9466, 9429, 9391, 9353, 9315, 9277, 9238, 9200, 9162, 9124, 9086, 9047, 9009, 8971, 8932, 8893, 8855, 8816, 8777, 8739, 8700, 8661, 8622, 8582, 8543, 8504, 8464, 8425, 8385, 8345, 8305, 8265, 8225, 8184, 8144, 8103, 8062, 8022, 7980, 7939, 7898, 7856, 7814, 7772, 7730, 7688, 7645, 7603, 7560, 7516, 7473, 7429, 7385, 7341, 7297, 7252, 7207, 7162, 7116, 7070, 7024, 6977, 6931, 6883, 6836, 6788, 6739, 6690, 6641, 6591, 6541, 6491, 6440, 6388, 6336, 6283, 6230, 6176, 6121, 6066, 6010, 5954, 5896, 5838, 5779, 5719, 5658, 5597, 5534, 5470, 5405, 5339, 5272, 5203, 5133, 5061, 4988, 4913, 4836, 4757, 4676, 4592, 4506, 4417, 4325, 4229, 4129, 4025, 3916, 3801, 3680, 3551, 3413, 3263, 3099, 2917, 2710, 2466, 2162, 1732, 100};

byte Kp = 150;
byte Ki = 3;
byte Kd = 50;
double PIDOutput;
double PIDInput;
double consKp= (double)Kp / 10; 
double consKi= (double)Ki / 100; 
double consKd= (double)Kd / 10;
  
PID pid(&PIDInput, &PIDOutput, &tempSetting, consKp, consKi, consKd, DIRECT);

boolean errors[100];
byte errN = 0;

double measures[AVERAGES_COUNT];
byte measureN = 0;
boolean debugTemp = false;

union mytype
{
  double d;
  byte b[4];
} tmp; //for long to byte[4] conversion and back


void zcA()
{
  if (mode != MODE_RELAY && TCNT3 + phaseTime - OCR3A > MINIMUM_PHASE_PERIOD) 
  {  
    periodA = TCNT3 + phaseTime - OCR3A;
    OCR3A = TCNT3 + phaseTime;
  }
}

void zcB()
{
  if (mode != MODE_RELAY  && TCNT3 + phaseTime - OCR3B > MINIMUM_PHASE_PERIOD) OCR3B = TCNT3 + phaseTime;
}

void zcC()
{
  if (mode != MODE_RELAY  && TCNT3 + phaseTime - OCR3C > MINIMUM_PHASE_PERIOD) OCR3C = TCNT3 + phaseTime;
}

ISR(TIMER3_COMPA_vect)
{
  if (heater) 
  {
    digitalWrite(TRIAC_A_PIN, true);
    TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
  }
  if (mode == MODE_RELAY) OCR3A = TCNT3 + TRIAC_PULSE_RELAY_TIMER_OCR;
}

ISR(TIMER3_COMPB_vect)
{
  if (heater) 
  {
    digitalWrite(TRIAC_B_PIN, true);
    TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
  }
  if (mode == MODE_RELAY) OCR3B = TCNT3 + TRIAC_PULSE_RELAY_TIMER_OCR;
}

ISR(TIMER3_COMPC_vect)
{
  if (heater) 
  {
    digitalWrite(TRIAC_C_PIN, true);
    TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
  }
  if (mode == MODE_RELAY) OCR3C = TCNT3 + TRIAC_PULSE_RELAY_TIMER_OCR;
}

ISR(TIMER1_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  digitalWrite(TRIAC_A_PIN, false);
  digitalWrite(TRIAC_B_PIN, false);
  digitalWrite(TRIAC_C_PIN, false);
  TCNT1 = 1;
}

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

  interrupts(); 

  attachInterrupt(ZCD_A_INT, zcA, CHANGE);
  attachInterrupt(ZCD_B_INT, zcB, CHANGE);
  attachInterrupt(ZCD_C_INT, zcC, CHANGE);

  Serial.begin(9600);
  if (checkEEPROM())
  {
    readNetworkSettings();
    readRegulatorSettings();
  }
  initNetwork();
  if (checkRTC())
    readRTC();
  setSyncProvider(getRTCtime);   // the function to get the time from the RTC
  if(timeStatus()!= timeSet) Serial.println ("Unable to sync with the RTC");

  delay(100);
  
  byte phaseCheck = 0;
  if (OCR3A == 0)
    error = "no phase A sync";
  else if (OCR3B == 0)
    error = "no phase B sync";
  else if (OCR3C == 0)
    error = "no phase C sync";
  else
  {
    phaseCheck = 1;
/*    unsigned int timediffB = OCR3B - OCR3A;
    unsigned int timediffC = OCR3C - OCR3A;
    if (timediffB > timediffC)
    {
      error = "phase swap";
      phaseCheck = 2;
    }*/
  }
  Serial.println(error);
  setPIDParams();
  pid.SetSampleTime(UPDATE_PERIOD);
  pid.SetMode(AUTOMATIC);
  
  for (byte i = 0; i<100; i++)
    errors[i] = 0;
}

void loop()
{
  handleButtons();
  if (server.available()) client = server.available();
  
  if (client.connected() && client.available()) readCommand(&client);
  if (Serial.available()) readCommand(&Serial);
  if (mode == MODE_PIM_MAN) handleHeaterPimMan();
  
  if (millis() - updated > UPDATE_PERIOD)
  {
    updated = millis();
    errN++;
    if (errN >= 100) errN = 0;
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
      errors[errN] = false;
      measureN++;
      if (measureN >= AVERAGES_COUNT) measureN = 0;
      measures[measureN] = temperature;
      //Serial.println("got ok");
    }
    else
    {
      errors[errN] = true;
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
    draw();
    /*
    volatile unsigned int timediffB = crBR - crAR;
    volatile unsigned int timediffC = crCR - crAR;*/
    /*char bbuf[11];
        
    dtostrf(periodA, 5, 0, bbuf);
    Serial.print(bbuf);
//    Serial.print(" and timeC ");
//    dtostrf(timediffC, 5, 0, bbuf);
//    Serial.print(bbuf);
//    if (timediffB > timediffC ) Serial.print(" SWAP!");
    Serial.println();
    */
      switch (mode)
    {
      case MODE_RELAY:
        handleHeaterRelay();
      break;
      case MODE_PIM_PID:
        handleHeaterPimPid();
      break;
    }
    writeRTC();
  }
}

void dhtLib_wrapper() 
{
  DHTLib.dht22Callback(); 
}

void handleHeaterPimPid()
{
  heater = powerOn;
  if (heater)
  {
    double sum= 0;
    for (byte i = 0; i<AVERAGES_COUNT; i++)
     sum = sum + measures[i];
    PIDInput = sum / AVERAGES_COUNT;
    if (debugTemp && measureN == 0)
    {
      char bbuf[5];
      response("measured temperature: ");
      dtostrf(PIDInput, 2, 1, bbuf);
      responseln(bbuf);
    }
    pid.Compute();
    manPimSetting = PIDOutput;
    phaseTime = phasePowerMap[manPimSetting];
  }
}

void handleHeaterPimMan()
{
  heater = powerOn;
  phaseTime = phasePowerMap[manPimSetting];//19400 - manPimSetting * 193; //(100-19400 is working range)
}


void handleHeaterRelay()
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
    if (powerOn && bitRead(btnState, 1))
    {
      if (mode == MODE_PIM_MAN)
      {
        manPimSetting++;
        //if (manPimSetting >= 100) manPimSetting = 100;
      }
      else
      {
        tempSetting++;
        if (tempSetting >= MAX_TEMP_SETTING)
          tempSetting = MAX_TEMP_SETTING;
      }
    }
    if (powerOn && bitRead(btnState, 2))
    {
      if (mode == MODE_PIM_MAN)
      {
        manPimSetting--;
        //if (manPimSetting <= 1) manPimSetting = 1;
      }
      else
      {
        tempSetting--; 
        if (tempSetting <= MIN_TEMP_SETTING)
          tempSetting = MIN_TEMP_SETTING;
      }
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
      if (true)//error == "")
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
      
      if (mode == MODE_RELAY)
      {
        if (heater)
        {
          sprintf(buf, "\x2F");// heater symbol
          u8g.setFont(u8g_font_10x20_67_75);
          u8g.drawStr( 117, 64, buf);
        }
      }
      else
      {
        dtostrf(manPimSetting, 3, 0, charbuf); //pim setting
        sprintf(buf, "%s%%", charbuf); 
        u8g.setFont(u8g_font_8x13);
        u8g.drawStr(96, 60, buf);
      }
    }
    else
    {
      analogWrite(BACKLIGHT_PIN, STANDBY_BRIGHTNESS);
      //if (error == "")
      {
        sprintf(buf, "%d:%02d", hour(), minute()); //time
        u8g.setFont(u8g_font_fub25);
        u8g.drawStr( 19, 42, buf);
        char bbuf[11];
        sprintf(bbuf, "%d.%02d.%d", year(), month(), day()); //date
        u8g.setFont(u8g_font_8x13);
        u8g.drawStr( 24, 63, bbuf);
      }
      /*else
      {
        sprintf(buf, "ERROR!", charbuf);
        u8g.setFont(u8g_font_8x13);
        u8g.drawStr( 25, 10, buf);
        
        error.toCharArray(charbuf, 25);
        sprintf(buf, "%s", charbuf);
        u8g.setFont(u8g_font_8x13);
        u8g.drawStr( 5, 25, buf);
      }*/
    }
   } 
   while( u8g.nextPage() );
}

void setPIDParams()
{
  consKp = (double)Kp / 10; 
  consKi = (double)Ki / 100; 
  consKd = (double)Kd / 10;
  pid.SetTunings(consKp, consKi, consKd);
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

void writeRTC()
{
  tmp.d = tempSetting;
  for (int i = 0; i < 4; i++) rtc.poke(RAM_TEMP_SETTING_START_ADDR + i, tmp.b[i]);
}

void readRTC()
{
  for (int i = 0; i < 4; i++) tmp.b[i] = rtc.peek(RAM_TEMP_SETTING_START_ADDR + i);
  tempSetting = tmp.d; //get long value of counter
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
      byte errorRate = 0;
      for (byte i = 0; i < 100; i++)
        if (errors[i]) errorRate++;
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
      responseln("sensor error rate: "+ String(errorRate));
      response("power: ");
      responseln(powerOn?"ON":"OFF");
      response("heater: ");
      responseln(heater?"ON":"OFF");
      responseln("PIM setting: "+ String(manPimSetting));
      responseln("Ki: "+ String(Ki));
      responseln("Kp: "+ String(Kp));
      responseln("Kd: "+ String(Kd));

      response("mode: ");
      switch (mode)
      {
        case MODE_RELAY:
          responseln("RELAY");
        break;
        case MODE_PIM_PID:
          responseln("PID");
        break;
        case MODE_PIM_MAN:
          responseln("MANUAL");
        break;
      }    
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
  else if (textBuffer.substring(0,4) == "smp=")
  {
    textBuffer = textBuffer.substring(4);
    int value = parseDigits();
    if (value >= 0 && value <= 255)
    {
      manPimSetting = value;
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
  else if (textBuffer.substring(0,4) == "sdt0")
  {
    debugTemp = false;
    printOK();
  }
  else if (textBuffer.substring(0,4) == "sdt1")
  {
    debugTemp = true;
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
  else if (textBuffer.substring(0,2) == "sp")
  {
    textBuffer = textBuffer.substring(2);
    doRegulatorSettings();
    writeRegulatorSettings();
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
 
void doRegulatorSettings()
{
  int value = 0;
  if (textBuffer.substring(0,3) == "ki=")
  {
    textBuffer = textBuffer.substring(3);
    int value = parseDigits();
    if (value >= 0 && value <= 255)
    {
      Ki = value;
      setPIDParams();
      printOK();
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,3) == "kp=")
  {
    textBuffer = textBuffer.substring(3);
    int value = parseDigits();
    if (value >= 0 && value <= 255)
    {
      Kp = value;
      setPIDParams();
      printOK();
      
    }
    else
     printErrorMessage();
  }
  else if (textBuffer.substring(0,3) == "kd=")
  {
    textBuffer = textBuffer.substring(3);
    int value = parseDigits();
    if (value >= 0 && value <= 255)
    {
      Kd = value;
      setPIDParams();
      printOK();
    }
    else
     printErrorMessage();
  }  
  else if (textBuffer.substring(0,6) == "mrelay")
  {
    mode = MODE_RELAY;
    printOK();
  }
  else if (textBuffer.substring(0,7) == "mpimman")
  {
    mode = MODE_PIM_MAN;
    printOK();
  }
  else if (textBuffer.substring(0,7) == "mpimpid")
  {
    mode = MODE_PIM_PID;
    printOK();
  }
  else
   printErrorMessage(); 
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

void readRegulatorSettings()
{
  mode = EEPROM.read(EEPROM_MODE_ADDR);
  Ki = EEPROM.read(EEPROM_KI_ADDR);
  Kp = EEPROM.read(EEPROM_KP_ADDR);
  Kd = EEPROM.read(EEPROM_KD_ADDR);
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

void writeRegulatorSettings()
{
  EEPROM.write(EEPROM_MODE_ADDR, mode);
  EEPROM.write(EEPROM_KI_ADDR, Ki);
  EEPROM.write(EEPROM_KP_ADDR, Kp);
  EEPROM.write(EEPROM_KD_ADDR, Kd);
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
