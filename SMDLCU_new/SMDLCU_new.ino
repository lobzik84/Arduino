#include "U8glib.h"
#include <EEPROM.h>
#include <SPI.h>
#include <Ethernet.h>

//EEPROM data map
#define EEPROM_MAC_START_ADDR 0x00   //6 bytes
#define EEPROM_IP_START_ADDR 0x06    //4 bytes
#define EEPROM_MASK_START_ADDR 0x0a  //4 bytes
#define EEPROM_GW_START_ADDR 0x0e    //4 bytes
#define EEPROM_CONFIG_BYTE_ADDR 0x12 //1 byte
#define EEPROM_CONTROL_VALUE_ADDR 0xEE

#define EEPROM_DAC_VALUES_START_ADDR 0x20 //16 bytes

#define EEPROM_CONTROL_VALUE 0xEB

//Pins definition. Pins 4,10,11,12,13 used by ethernet-shield and should not be used. Pins 50,51,52,53 is SPI

/*
  Board	          int.0	  int.1	  int.2	  int.3	  int.4	  int.5
 Mega2560	  2	  3	  21	  20	  19	  18

 */
 
#define STEP1_EN_PIN A1 // stepper motor enable, active low pin (X)
#define STEP2_EN_PIN 8  // stepper motor enable, active low pin (Y)
#define STEP3_EN_PIN 9  // stepper motor enable, active low pin (Z)
#define STEP4_EN_PIN A2 // stepper motor enable, active low pin (A)

#define STEP1_STP_PIN 2 // CONNECTED stepper control pin
#define STEP2_STP_PIN 3 // CONNECTED stepper control pin
#define STEP3_STP_PIN A3 // stepper control pin
#define STEP4_STP_PIN A4 // stepper control pin

#define STEP1_DIR_PIN 5 // CONNECTED stepper motor direction control pin
#define STEP2_DIR_PIN 6 // CONNECTED stepper motor direction control pin
#define STEP3_DIR_PIN 7 // CONNECTED stepper motor direction control pin
#define STEP4_DIR_PIN A0 // stepper motor direction control pin

#define ENC_B_PIN 20 //YELLOW
#define ENC_A_PIN 21 //GREEN
#define ENC_A_INT 2

#define DAC1_BTN_PIN 24
#define DAC2_BTN_PIN 26
#define DAC3_BTN_PIN 28
#define DAC4_BTN_PIN 30
#define STEPPER_MODE_PIN 22

#define DAC1_LED_PIN 25
#define DAC2_LED_PIN 27
#define DAC3_LED_PIN 29
#define DAC4_LED_PIN 31

#define DAC1_DATA_PIN 33
#define DAC2_DATA_PIN 35
#define DAC3_DATA_PIN 37
#define DAC4_DATA_PIN 39
#define DAC_STB_PIN 41
#define DAC_LD_PIN 43

#define LCD_RS_PIN 32
#define LCD_RW_PIN 34
#define LCD_E_PIN 36


#define BACKLIGHT_PIN 44

// preload timer 65536-16MHz/8/4000Hz
#define THR_CNT_PRELOAD 65336

#define SERVER_PORT 23 //telnet server port
#define TEXT_BUFFER_SIZE 39 //length of longest command string plus two spaces for CR + LF

#define DISPLAY_REFRESH_PERIOD 200 //ms

#define STEPPER_TIMEOUT 500 //ms

#define BUTTON_DISACT_TIMEOUT 20000 //ms

U8GLIB_ST7920_128X64_1X u8g(LCD_E_PIN, LCD_RW_PIN, LCD_RS_PIN);

byte MAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte IP[4] = {192, 168, 0, 10};
byte MASK[4] = {255, 255, 255, 0};
byte GW[4] = {192, 168, 0, 1};
boolean DHCP = false;

EthernetServer server(SERVER_PORT); // Telnet listens on port 23
//EthernetClient client = 0; // Client needs to have global scope so it can be called

//buffers
String textBuffer = "";  
byte parsedBytesBuffer[12];


byte btnState = 0;
long btnChanged = 0;
long updated = 0;
long displayRefreshed = 0;
volatile long lastStep = 0;
volatile boolean stepDir = false;
volatile long angleChanged = 0;

boolean updateDisplay = true;

byte selectedStepperIndex = 0;
byte selectedDACindex = 0;
int DAC1out = 0;
int DAC2out = 0;
int DAC3out = 0;
int DAC4out = 0;

volatile byte cycle = 0;
volatile byte bitN = 0;

volatile int angle = 0;
volatile int prevAngle = 0;

volatile int toStep = 0;

boolean commandDone = true;


union mytype
{
  int i;
  byte b[2];
} tmp; //for int to byte[2] conversion and back

void encA() 
{
  angleChanged = millis();
  if (digitalRead(ENC_B_PIN)) 
  { 
    angle++; 
    if (!digitalRead(STEPPER_MODE_PIN)) 
    {
      lastStep = millis(); 
      toStep++;
    }
  }
  else 
  { 
    angle--; 
    if (!digitalRead(STEPPER_MODE_PIN)) 
    {
      lastStep = millis(); 
      toStep--;
    }
  }
}

ISR(TIMER1_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  if (cycle < 24)
  {
    if (cycle%2 == 0) 
    {
      digitalWrite(DAC_STB_PIN, false);

      bitN = 11 - (cycle / 2);
      digitalWrite(DAC1_DATA_PIN, bitRead(DAC1out, bitN));
      digitalWrite(DAC2_DATA_PIN, bitRead(DAC2out, bitN));
      digitalWrite(DAC3_DATA_PIN, bitRead(DAC3out, bitN));
      digitalWrite(DAC4_DATA_PIN, bitRead(DAC4out, bitN));
    }
    else 
    {
      digitalWrite(DAC_STB_PIN, true);      
    }
    cycle++;
  }
  else if (cycle < 26)
  {
    if (cycle == 24) 
    {
      digitalWrite(STEP1_STP_PIN, false);
      digitalWrite(STEP2_STP_PIN, false);
      digitalWrite(STEP3_STP_PIN, false);
      digitalWrite(STEP4_STP_PIN, false);
      digitalWrite(DAC1_DATA_PIN, false);
      digitalWrite(DAC2_DATA_PIN, false);
      digitalWrite(DAC3_DATA_PIN, false);
      digitalWrite(DAC4_DATA_PIN, false);
      digitalWrite(DAC_STB_PIN, false);
      digitalWrite(DAC_LD_PIN, false);
    }
    else if (cycle == 25) 
    {
      digitalWrite(DAC_LD_PIN, true);
      
      if (toStep > 0) 
      {
        digitalWrite(STEP1_DIR_PIN, true);
        digitalWrite(STEP2_DIR_PIN, true);
        digitalWrite(STEP3_DIR_PIN, true);
        digitalWrite(STEP4_DIR_PIN, true);
        digitalWrite(STEP1_STP_PIN, true);
        digitalWrite(STEP2_STP_PIN, true);
        digitalWrite(STEP3_STP_PIN, true);
        digitalWrite(STEP4_STP_PIN, true);
        toStep--;
      }
      else if (toStep < 0) 
      {
        digitalWrite(STEP1_DIR_PIN, false);
        digitalWrite(STEP2_DIR_PIN, false);
        digitalWrite(STEP3_DIR_PIN, false);
        digitalWrite(STEP4_DIR_PIN, false);
        digitalWrite(STEP1_STP_PIN, true);
        digitalWrite(STEP2_STP_PIN, true);
        digitalWrite(STEP3_STP_PIN, true);
        digitalWrite(STEP4_STP_PIN, true);
        toStep++;
      }

    }
    cycle++;
  }
  else
   cycle = 0;
   
  TCNT1 = THR_CNT_PRELOAD;   
}


void setup()
{
  pinMode(STEP1_EN_PIN, OUTPUT);
  pinMode(STEP2_EN_PIN, OUTPUT);
  pinMode(STEP3_EN_PIN, OUTPUT);
  pinMode(STEP4_EN_PIN, OUTPUT);

  digitalWrite(STEP1_EN_PIN, true);
  digitalWrite(STEP2_EN_PIN, true);
  digitalWrite(STEP3_EN_PIN, true);
  digitalWrite(STEP4_EN_PIN, true);

  pinMode(STEP1_STP_PIN, OUTPUT);
  pinMode(STEP2_STP_PIN, OUTPUT);
  pinMode(STEP3_STP_PIN, OUTPUT);
  pinMode(STEP4_STP_PIN, OUTPUT);
  
  digitalWrite(STEP1_STP_PIN, false);
  digitalWrite(STEP2_STP_PIN, false);
  digitalWrite(STEP3_STP_PIN, false);
  digitalWrite(STEP4_STP_PIN, false);

  pinMode(STEP1_DIR_PIN, OUTPUT);
  pinMode(STEP2_DIR_PIN, OUTPUT);
  pinMode(STEP3_DIR_PIN, OUTPUT);
  pinMode(STEP4_DIR_PIN, OUTPUT);
  
  digitalWrite(STEP1_DIR_PIN, false);
  digitalWrite(STEP2_DIR_PIN, false);
  digitalWrite(STEP3_DIR_PIN, false);
  digitalWrite(STEP4_DIR_PIN, false);
  
  pinMode(DAC1_BTN_PIN, INPUT_PULLUP);
  pinMode(DAC2_BTN_PIN, INPUT_PULLUP);
  pinMode(DAC3_BTN_PIN, INPUT_PULLUP);
  pinMode(DAC4_BTN_PIN, INPUT_PULLUP);
  
  pinMode(STEPPER_MODE_PIN, INPUT_PULLUP);
  
  pinMode(DAC1_LED_PIN, OUTPUT);
  pinMode(DAC2_LED_PIN, OUTPUT);
  pinMode(DAC3_LED_PIN, OUTPUT);
  pinMode(DAC4_LED_PIN, OUTPUT);

  digitalWrite(DAC1_LED_PIN, false);
  digitalWrite(DAC2_LED_PIN, false);
  digitalWrite(DAC3_LED_PIN, false);
  digitalWrite(DAC4_LED_PIN, false);
  
  pinMode(DAC1_DATA_PIN, OUTPUT);
  pinMode(DAC2_DATA_PIN, OUTPUT);
  pinMode(DAC3_DATA_PIN, OUTPUT);
  pinMode(DAC4_DATA_PIN, OUTPUT);
  
  pinMode(DAC_STB_PIN, OUTPUT);
  pinMode(DAC_LD_PIN, OUTPUT);
  
  digitalWrite(DAC_LD_PIN, true);
  
  analogWrite(BACKLIGHT_PIN, 255);
  
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  digitalWrite(ENC_A_PIN, HIGH);
  digitalWrite(ENC_B_PIN, HIGH);
  
   // initialize timer1 
  noInterrupts();           // disable all interrupts
  attachInterrupt(ENC_A_INT, encA, RISING);
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = THR_CNT_PRELOAD;            
  TCCR1B |= (1 << CS11);    // 8 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts(); 

  Serial.begin(115200);
  if (checkEEPROM())
    readNetworkSettings();
  initNetwork();
  if (checkEEPROM())
    readNetworkSettings();
  readDACvalues();
}

void loop()
{
  if (updateDisplay && millis() - displayRefreshed > DISPLAY_REFRESH_PERIOD )
  {
    draw();
    displayRefreshed = millis();
  }
  if (millis() - btnChanged > BUTTON_DISACT_TIMEOUT && millis() - angleChanged > BUTTON_DISACT_TIMEOUT ) 
  {
    selectedDACindex = 0;
  }
  
  handleButtons();
  handleEncoder();

  if (toStep == 0) 
  {
    if (!commandDone)  
      printOK();
    commandDone = true;
    if (millis() - lastStep > STEPPER_TIMEOUT)
    {
      selectedStepperIndex = 0;
      updateDisplay = true;
    }
  }
  handleStepper();

 // if (server.available())
  for (int sock=0; sock < 4; sock++) 
  {
    EthernetClient cli = server.available_(sock);
    if (cli) {
        if (cli.connected()) readCommand(&cli);
    }  
  }
  //client = server.available();
  
  //if (client.connected() && client.available()) readCommand(&client);
  if (Serial.available()) readCommand(&Serial);

}

void handleStepper()
{
 /* if (toStep > 0)
  {
    digitalWrite(STEP1_DIR_PIN, true);
    digitalWrite(STEP2_DIR_PIN, true);
    digitalWrite(STEP3_DIR_PIN, true);
    digitalWrite(STEP4_DIR_PIN, true);
  }
  else if (toStep < 0)
  {
    digitalWrite(STEP1_DIR_PIN, false);
    digitalWrite(STEP2_DIR_PIN, false);
    digitalWrite(STEP3_DIR_PIN, false);
    digitalWrite(STEP4_DIR_PIN, false);
  }*/
  
  switch (selectedStepperIndex)
  {
        case 1:
          digitalWrite(STEP1_EN_PIN, false);
          digitalWrite(STEP2_EN_PIN, true);
          digitalWrite(STEP3_EN_PIN, true);
          digitalWrite(STEP4_EN_PIN, true);
        break;  

        case 2:
          digitalWrite(STEP1_EN_PIN, true);
          digitalWrite(STEP2_EN_PIN, false);
          digitalWrite(STEP3_EN_PIN, true);
          digitalWrite(STEP4_EN_PIN, true);
        break;  

        case 3:
          digitalWrite(STEP1_EN_PIN, true);
          digitalWrite(STEP2_EN_PIN, true);
          digitalWrite(STEP3_EN_PIN, false);
          digitalWrite(STEP4_EN_PIN, true);
        break;  

        case 4:
          digitalWrite(STEP1_EN_PIN, true);
          digitalWrite(STEP2_EN_PIN, true);
          digitalWrite(STEP3_EN_PIN, true);
          digitalWrite(STEP4_EN_PIN, false);
        break;  
        
        default:
        case 0:
          digitalWrite(STEP1_EN_PIN, true);
          digitalWrite(STEP2_EN_PIN, true);
          digitalWrite(STEP3_EN_PIN, true);
          digitalWrite(STEP4_EN_PIN, true);
        break;
  }
}

void handleEncoder()
{
  if (angle != prevAngle) 
  {
    if (digitalRead(STEPPER_MODE_PIN))
    {
      switch (selectedDACindex)
      {
        case 1:
          if (DAC1out + (angle-prevAngle) > 4095)
            DAC1out = 4095;
          else if (DAC1out + (angle-prevAngle) < 0)
            DAC1out = 0;
          else 
            DAC1out = DAC1out + (angle-prevAngle);
        break;
  
  
        case 2:
          if (DAC2out + (angle-prevAngle) > 4095)
            DAC2out = 4095;
          else if (DAC2out + (angle-prevAngle) < 0)
            DAC2out = 0;
          else 
            DAC2out = DAC2out + (angle-prevAngle);
        break;
  
  
        case 3:
          if (DAC3out + (angle-prevAngle) > 4095)
            DAC3out = 4095;
          else if (DAC3out + (angle-prevAngle) < 0)
            DAC3out = 0;
          else 
            DAC3out = DAC3out + (angle-prevAngle);
        break;
  
        case 4:
          if (DAC4out + (angle-prevAngle) > 4095)
            DAC4out = 4095;
          else if (DAC4out + (angle-prevAngle) < 0)
            DAC4out = 0;
          else 
            DAC4out = DAC4out + (angle-prevAngle);
        break;               
      }
    }  
    prevAngle = angle;
    updateDisplay = true;
    displayRefreshed = 0;
  }
}

void handleButtons()
{

  byte state = 0;
  if (!digitalRead(DAC1_BTN_PIN))
    bitSet(state, 0);
  if (!digitalRead(DAC2_BTN_PIN))
    bitSet(state, 1);
  if (!digitalRead(DAC3_BTN_PIN))
    bitSet(state, 2);
  if (!digitalRead(DAC4_BTN_PIN))
    bitSet(state, 3);
    
  if (state != btnState)
  {
    btnState = state;
    btnChanged = millis();
  }

  if (state != 0 && millis() - btnChanged > 50)
  {
    switch(state) {
      case 1:
        selectedDACindex = 1;
        digitalWrite(DAC1_LED_PIN, true);
        digitalWrite(DAC2_LED_PIN, false);
        digitalWrite(DAC3_LED_PIN, false);
        digitalWrite(DAC4_LED_PIN, false);
      break; 
      
      case 2:
        selectedDACindex = 2;      
        digitalWrite(DAC1_LED_PIN, false);
        digitalWrite(DAC2_LED_PIN, true);
        digitalWrite(DAC3_LED_PIN, false);
        digitalWrite(DAC4_LED_PIN, false);
      break;
        
      case 4:
        selectedDACindex = 3;       
        digitalWrite(DAC1_LED_PIN, false);
        digitalWrite(DAC2_LED_PIN, false);
        digitalWrite(DAC3_LED_PIN, true);
        digitalWrite(DAC4_LED_PIN, false);
      break;
      
      case 8:
        selectedDACindex = 4;      
        digitalWrite(DAC1_LED_PIN, false);
        digitalWrite(DAC2_LED_PIN, false);
        digitalWrite(DAC3_LED_PIN, false);
        digitalWrite(DAC4_LED_PIN, true);
      break;
       
    }
    updateDisplay = true;
    displayRefreshed = 0;
    storeDACvalues();
  }

  if (!digitalRead(STEPPER_MODE_PIN) && selectedStepperIndex == 0) 
    selectedStepperIndex = selectedDACindex;

}

void draw() 
{
  //char charbuf[6];
  char buf[15];
  u8g.firstPage();  
  do 
  {   
      sprintf(buf, "DAC1: %d", DAC1out);
      u8g.setFont(u8g_font_5x8);
      u8g.drawStr(3, 8, buf);
      byte barL = (DAC1out/33) + 1;
      u8g.drawBox(3, 10, barL, 4);
      
      sprintf(buf, "DAC2: %d", DAC2out);
      u8g.setFont(u8g_font_5x8);
      u8g.drawStr(3, 24, buf);
      barL = (DAC2out/33) + 1;
      u8g.drawBox(3, 26, barL, 4);

      sprintf(buf, "DAC3: %d", DAC3out);
      u8g.setFont(u8g_font_5x8);
      u8g.drawStr(3, 40, buf);
      barL = (DAC3out/33) + 1;
      u8g.drawBox(3, 42, barL, 4);
      
      sprintf(buf, "DAC4: %d", DAC4out);
      u8g.setFont(u8g_font_5x8);
      u8g.drawStr(3, 56, buf);
      barL = (DAC4out/33) + 1;
      u8g.drawBox(3, 58, barL, 4);

      byte frameY = 16*(selectedDACindex-1);
      u8g.drawFrame(0,frameY,128,16);
      
      if (selectedStepperIndex > 0)
      {
        frameY = 16*(selectedStepperIndex-1);
        u8g.setFont(u8g_font_6x12);
        u8g.drawStr(68, frameY+9, "STEPPER");
        if (toStep > 0)
        {
          u8g.drawStr(111, frameY+9, ">>");
        }
        else if (toStep < 0)
        {
          u8g.drawStr(55, frameY+9, "<<");      
        }
      }  

   } 
   while( u8g.nextPage() );
   updateDisplay = false;
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
       if (textBuffer.substring(0,3) != "dac") printEcho();
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
      sprintf(bbuf, "DAC1: %d", DAC1out);
      responseln(bbuf);
      sprintf(bbuf, "DAC2: %d", DAC2out);
      responseln(bbuf);
      sprintf(bbuf, "DAC3: %d", DAC3out);
      responseln(bbuf);
      sprintf(bbuf, "DAC4: %d", DAC4out);
      responseln(bbuf);
      sprintf(bbuf, "Selected DAC: %d", selectedDACindex);
      responseln(bbuf);
      printOK();
  }
  else if (textBuffer.substring(0,5) == "dac1=")
  {
    textBuffer = textBuffer.substring(5);
    int value = parseDigits();
    if (value >=0 && value < 4096)  
    {
      DAC1out = value; 
//      commandDone = false;
      updateDisplay = true;
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "dac2=")
  {
    textBuffer = textBuffer.substring(5);
    int value = parseDigits();
    if (value >=0 && value < 4096)  
    {
      DAC2out = value; 
//      commandDone = false;
      updateDisplay = true;
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "dac3=")
  {
    textBuffer = textBuffer.substring(5);
    int value = parseDigits();
    if (value >=0 && value < 4096)  
    {
      DAC3out = value; 
//      commandDone = false;
      updateDisplay = true;
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,5) == "dac4=")
  {
    textBuffer = textBuffer.substring(5);
    int value = parseDigits();
    if (value >=0 && value < 4096)  
    {
      DAC4out = value; 
//      commandDone = false;
      updateDisplay = true;
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,4) == "step")
  {
    textBuffer = textBuffer.substring(4);
    int index = parseDigits();
    if (index > 0 && index <= 4)  
    {
      selectedStepperIndex = index;
      if (textBuffer.substring(0,1) == "+")
      {
        textBuffer = textBuffer.substring(1);
        int steps = parseDigits();
        toStep = steps;
        commandDone = false;
        updateDisplay = true;
      } 
      else if (textBuffer.substring(0,1) == "-")
      {
        textBuffer = textBuffer.substring(1);
        int steps = parseDigits();
        toStep = -1 *steps;
        commandDone = false;
        updateDisplay = true;
      }
      else 
        printErrorMessage();
    }
    else
      printErrorMessage();
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
//  client.stop();
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

void readDACvalues()
{
  for (int i = 0; i < 2; i++) tmp.b[i] = EEPROM.read(EEPROM_DAC_VALUES_START_ADDR + i);
  if (tmp.i >= 0 && tmp.i < 4096) DAC1out = tmp.i;
  for (int i = 0; i < 2; i++) tmp.b[i] = EEPROM.read(EEPROM_DAC_VALUES_START_ADDR + i+2);
  if (tmp.i >= 0 && tmp.i < 4096) DAC2out = tmp.i;
  for (int i = 0; i < 2; i++) tmp.b[i] = EEPROM.read(EEPROM_DAC_VALUES_START_ADDR + i+4);
  if (tmp.i >= 0 && tmp.i < 4096) DAC3out = tmp.i;
  for (int i = 0; i < 2; i++) tmp.b[i] = EEPROM.read(EEPROM_DAC_VALUES_START_ADDR + i+6);
  if (tmp.i >= 0 && tmp.i < 4096) DAC4out = tmp.i;
  
}

void storeDACvalues()
{
  for (int i = 0; i < 2; i++) tmp.b[i] = EEPROM.read(EEPROM_DAC_VALUES_START_ADDR + i);
  if (tmp.i != DAC1out)
  {
    tmp.i = DAC1out;
    for (int i = 0; i < 2; i++) EEPROM.write(EEPROM_DAC_VALUES_START_ADDR + i, tmp.b[i]);
  }
  for (int i = 0; i < 2; i++) tmp.b[i] = EEPROM.read(EEPROM_DAC_VALUES_START_ADDR + i+2);
  if (tmp.i != DAC2out)
  {
    tmp.i = DAC2out;
    for (int i = 0; i < 2; i++) EEPROM.write(EEPROM_DAC_VALUES_START_ADDR + i+2, tmp.b[i]);
  }
  for (int i = 0; i < 2; i++) tmp.b[i] = EEPROM.read(EEPROM_DAC_VALUES_START_ADDR + i+4);
  if (tmp.i != DAC3out)
  {
    tmp.i = DAC3out;
    for (int i = 0; i < 2; i++) EEPROM.write(EEPROM_DAC_VALUES_START_ADDR + i+4, tmp.b[i]);
  }
  for (int i = 0; i < 2; i++) tmp.b[i] = EEPROM.read(EEPROM_DAC_VALUES_START_ADDR + i+6);
  if (tmp.i != DAC4out)
  {
    tmp.i = DAC4out;
    for (int i = 0; i < 2; i++) EEPROM.write(EEPROM_DAC_VALUES_START_ADDR + i+6, tmp.b[i]);
  }
  
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
