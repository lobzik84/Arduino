#include <SPI.h>
#include <Ethernet.h>
#include <OneWire.h>
#include <EEPROM.h>
#include <DallasTemperature.h>

#define TEXT_BUFFER_SIZE 539 //length of longest command string plus two spaces for CR + LF
#define SERVER_PORT 23 //telnet server port
#define DIGITAL_OUTPUT_PINS_COUNT 9
#define DIGITAL_INPUT_PINS_COUNT 2
#define ANALOG_OUTPUT_PINS_COUNT 2
#define ANALOG_INPUT_PINS_COUNT 7

//Pins definition. Pins 4,10,11,12,13 used by ethernet-shield and should not be used. Pins 50,51,52,53 is SPI

#define ONE_WIRE_BUS 23 //pin for 1-wire
#define WATCHDOG_PIN 5
int digitalOutputPins[DIGITAL_OUTPUT_PINS_COUNT]={25,27,29,31,33,35,37};
int digitalInputPins[DIGITAL_INPUT_PINS_COUNT]={41,42}; 
int analogOutputPins[ANALOG_OUTPUT_PINS_COUNT]={7,8}; //2 to 9 (5,6 are strange), 44 to 46
int analogInputPins[ANALOG_INPUT_PINS_COUNT]={0,1,2,3,4,5,6}; //0 to 15 analog pins

#define ANALOG_REFERENCE INTERNAL1V1 //Analog refenernce

#define EEPROM_MAC_START_ADDR 0x00 //EEPROM data map
#define EEPROM_IP_START_ADDR 0x06
#define EEPROM_MASK_START_ADDR 0x0a
#define EEPROM_GW_START_ADDR 0x0e
#define EEPROM_CONFIG_BYTE_ADDR 0x12
#define EEPROM_PINS_CONFIG_ADDR 

//   pin  mode
//    1    0
//    2    1
//    3    2
//    4    3
//    5    1
// mode 0 - always off 
//      1 - always on (reboot not allowed)
//      2 - on  (reboot allowed)
//      3 - any (reboot allowed and poweroff allowed)
// reboot parameters (for each pin)
//      powerdown_time - на сколько отрубать (0..255 c)
//      timer - через сколько после команды отрубать (0..255 c)
// *** 3 bytes for each pin = 24 bytes for 8 pins
//watchdogs
// ip port? period timeout pin_to_reboot+enabled bit
// 4   2      1     1                     1           //9 bytes for each watchdog
// 72 bytes for 8 watchdog
// 96 bytes total

boolean WATCHDOG_ENABLED = false;

String textBuffer = "";
int commandId = 0;
//int charsInBuffer = 0;
byte parsedBytesBuffer[12];
boolean watchdogPin = false;
EthernetServer server(SERVER_PORT); // Telnet listens on port 23
EthernetClient client = 0; // Client needs to have global scope so it can be called
                   // from functions outside of loop, but we don't know
                   // what client is yet, so creating an empty object

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);

void setup()
{
  pinMode(WATCHDOG_PIN, INPUT); //disable watchdog
  
  for (int  i= 0; i < DIGITAL_OUTPUT_PINS_COUNT; i++)  pinMode(digitalOutputPins[i], OUTPUT); 
  for (int  i= 0; i < DIGITAL_OUTPUT_PINS_COUNT; i++) digitalWrite(digitalOutputPins[i], LOW);
  for (int  i= 0; i < ANALOG_OUTPUT_PINS_COUNT; i++) analogWrite(analogOutputPins[i], 0);
  for (int  i= 0; i < DIGITAL_INPUT_PINS_COUNT; i++)  pinMode(digitalInputPins[i], INPUT);

  analogReference(ANALOG_REFERENCE);
 // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {;} // wait for serial port to connect. Needed for Leonardo only
  
  
  byte mac[6];
  for (int i = 0; i < 6; i++)
    mac[i] = EEPROM.read(EEPROM_MAC_START_ADDR + i); //reading 6 bytes of MAC from EEPROM
  if (bitRead(EEPROM.read(EEPROM_CONFIG_BYTE_ADDR), 0)) //if DHCP is enabled
  {
     // start the Ethernet connection using just MAC, with DHCP:
    if (Ethernet.begin(mac) == 0)  
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
    }
  }
  else //if DHCP disabled
  {
    //configuring static IP and start the Ethernet connection
    byte ip[4];
    byte mask[4];
    byte gw[4];
    for(int i = 0; i < 4; i++) 
       ip[i] = EEPROM.read(EEPROM_IP_START_ADDR + i);
    for(int i = 0; i < 4; i++) 
       mask[i] = EEPROM.read(EEPROM_MASK_START_ADDR + i);
    for(int i = 0; i < 4; i++) 
       gw[i] = EEPROM.read(EEPROM_GW_START_ADDR + i);  
    Ethernet.begin(mac, ip, gw, mask);     
  }

  Serial.println();
  server.begin();
  if (WATCHDOG_ENABLED) pinMode(WATCHDOG_PIN, OUTPUT);//enable watchdog
  resetWatchDog();
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
}

void loop()
{
  if (server.available()) 
    client = server.available();
  if (client.connected() && client.available()) 
    readCommand(&client);
  if (Serial.available()) 
    readCommand(&Serial);
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
        if (textBuffer.substring(0,3) != "GET") printEcho();
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
  if (textBuffer.substring(0,2) == "id")
  {
    textBuffer = textBuffer.substring(2); //remove "id"
    commandId = parseDigits(); //getting commandId and removing it from buffer
  }
  else commandId = 0;
  if (textBuffer.substring(0,3) == "get") //special case for routeros
  {
    client.println("HTTP/1.1 200 OK");
    client.println("Connection: close");  // the connection will be closed after completion of the response    
    responseln ("INFO: Got command by HTTP: " + textBuffer);
    client.stop();
    digitalWrite(25, HIGH);
    delay(1500);
    digitalWrite(25, LOW);
  }
  else if (textBuffer.substring(0,2) == "gt")
  {
    int sensorsCnt = sensors.getDeviceCount();
    DeviceAddress ds18b20;
    response("Requesting temperatures from " + String(sensorsCnt, DEC) + " sensors...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    responseln("DONE");
    for (int i=0; i < sensorsCnt; i++)
    {
      sensors.getAddress(ds18b20, i);
      for (uint8_t j = 0; j < 8; j++)
      {
        if (ds18b20[j] < 16) response("0");
        response(String(ds18b20[j], HEX) + ":");
      }
      response(" TEMP" + String(i, DEC) + ": ");
      char bbuf[5];
      dtostrf(sensors.getTempCByIndex(i), 2, 1, bbuf);
      responseln(bbuf);
    }
    printOK();
  }
  else if (textBuffer.substring(0,2) == "dr")
  {
    textBuffer = textBuffer.substring(2);
    readDigitalPins();
  }
  else if (textBuffer.substring(0,2) == "dw")
  {
    textBuffer = textBuffer.substring(2);
    writeDigitalPin();
  }
  else if (textBuffer.substring(0,2) == "ar")
  {
    textBuffer = textBuffer.substring(2);
    readAnalogPins();
  }
  else if (textBuffer.substring(0,2) == "aw")
  {
    textBuffer = textBuffer.substring(2);
    writeAnalogPin();
  }
  else if (textBuffer.substring(0,2) == "ow")
  {
    textBuffer = textBuffer.substring(2);
    do1wire();
  }
  else if (textBuffer.substring(0,2) == "pc")
  {
    printSettings();
  }
  else if (textBuffer.substring(0,2) == "sp")
  {
    textBuffer = textBuffer.substring(2);
    doSettings();
  }
  else if (textBuffer.substring(0,2) == "we")
  {
    //enable watchdog
    WATCHDOG_ENABLED = true;
    pinMode(WATCHDOG_PIN, OUTPUT);
    resetWatchDog();
    printOK();    
  }
  else if (textBuffer.substring(0,2) == "wd")
  {
    //disable watchdog
    WATCHDOG_ENABLED = false;
    pinMode(WATCHDOG_PIN, INPUT);
    printOK();
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

void printSettings()
{
  response("MAC: ");
  for (int address = EEPROM_MAC_START_ADDR; address < EEPROM_MAC_START_ADDR + 6; address++)
  {
    byte data = EEPROM.read(address);
    response(String(data, HEX) + ":");
  }
  responseln("");
  response("DHCP: ");
  if (bitRead(EEPROM.read(EEPROM_CONFIG_BYTE_ADDR), 0))
    responseln("ON");
  else
    responseln("OFF");
  response("IP: ");
  for (int address = EEPROM_IP_START_ADDR; address < EEPROM_IP_START_ADDR + 4; address++)
  {
    byte data = EEPROM.read(address);
    response(String(data, DEC) + ".");
  }
  responseln("");
  response("MASK: ");
  for (int address = EEPROM_MASK_START_ADDR; address < EEPROM_MASK_START_ADDR + 4; address++)
  {
    byte data = EEPROM.read(address);
    response(String(data, DEC) + ".");
  }
  responseln("");
  response("GW: ");
  for (int address = EEPROM_GW_START_ADDR; address < EEPROM_GW_START_ADDR + 4; address++)
  {
    byte data = EEPROM.read(address);
    response(String(data, DEC) + ".");
  }
  responseln("");
  printOK();
}

void doSettings()
{
  if (textBuffer.substring(0,4) == "mac=")
  {
    //set mac
    textBuffer = textBuffer.substring(4);
    if (parseBytes(6))
    {
      for(int i = 0; i < 6; i++) 
        EEPROM.write(EEPROM_MAC_START_ADDR + i, parsedBytesBuffer[i]);
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
      byte conf = EEPROM.read(EEPROM_CONFIG_BYTE_ADDR);
      bitSet(conf, 0);
      EEPROM.write(EEPROM_CONFIG_BYTE_ADDR, conf);
      printOK();
    }
    else if (textBuffer == "off")
    {
      byte conf = EEPROM.read(EEPROM_CONFIG_BYTE_ADDR);
      bitClear(conf, 0);
      EEPROM.write(EEPROM_CONFIG_BYTE_ADDR, conf);
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
      for(int i = 0; i < 4; i++) 
        EEPROM.write(EEPROM_IP_START_ADDR + i, parsedBytesBuffer[i]);
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
      for(int i = 0; i < 4; i++) 
        EEPROM.write(EEPROM_MASK_START_ADDR + i, parsedBytesBuffer[i]);
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
      for(int i = 0; i < 4; i++) 
        EEPROM.write(EEPROM_GW_START_ADDR + i, parsedBytesBuffer[i]);
      printOK();
    }
    else
      printErrorMessage();
  }
  
  else
      printErrorMessage();
}


void do1wire()
{
  byte data[12];
  byte addr[8];
  if (textBuffer.substring(0,2) == "sr")
  {
    oneWire.reset_search(); //search the bus
    while (oneWire.search(addr))
    {
      for(int i = 0; i < 8; i++) 
      {
        response(String(addr[i], HEX) + ":");
      }
      responseln("");
    }
    printOK();
  }
  else if (textBuffer.substring(0,2) == "rs")
  {
    oneWire.reset(); //reset the bus
    printOK();
  }
  else if (textBuffer.substring(0,2) == "sl")
  {
    //select
    textBuffer = textBuffer.substring(2);
    if (parseBytes(8))
    {
      for(int i = 0; i < 8; i++) addr[i] = parsedBytesBuffer[i];
      oneWire.select(addr);
      printOK();
    }
    else
      printErrorMessage();
  }
  else if (textBuffer.substring(0,2) == "sk")
  {
    oneWire.skip(); //skip selection, if there is only 1 device
  }
  else if (textBuffer.substring(0,2) == "wr")
  {
    //write
    textBuffer = textBuffer.substring(2);
    int byteToWrite = parseByte(textBuffer);
    if (byteToWrite < 0)
      printErrorMessage();
    else
    {
      oneWire.write(byte(byteToWrite));
      printOK();
    }
  }
  else if (textBuffer.substring(0,2) == "wp")
  {
    //write with power on
    textBuffer = textBuffer.substring(2);
    int byteToWrite = parseByte(textBuffer);
    if (byteToWrite < 0)
      printErrorMessage();
    else
    {
      oneWire.write(byte(byteToWrite), 1);
      printOK();
    }
  }
  else if (textBuffer.substring(0,2) == "rd")
  {
    for (int i = 0; i < 12; i++)      //read bus
    {
      data[i] = oneWire.read();
      response(String(data[i], HEX) + ":");
    }
    float tempr = getTemperature(data);
    char tempStr[5];
    dtostrf(tempr, 2, 2, tempStr);
    responseln("  calculated temp: "  + String(tempStr));//); //TODO check if it is ds18b20
    printOK();
  }
  else
    printErrorMessage();
}


void readDigitalPins()
{
  for (int i = 0; i < DIGITAL_INPUT_PINS_COUNT; i++) 
  {
    response("digital pin " + String(i) + " is ");
    if (digitalRead(digitalInputPins[i])) {
      responseln("HIGH");
    }
    else
      responseln("LOW");
  }
  printOK();
}


void readAnalogPins()
{
  for (int i = 0; i < ANALOG_INPUT_PINS_COUNT; i++) 
    responseln("analog input " + String(i) +" is " + String(analogRead(analogInputPins[i])));
  printOK();
}


void writeDigitalPin()
{
  int pin = parseDigits();
  if (pin < 0 || pin >= DIGITAL_OUTPUT_PINS_COUNT || textBuffer.charAt(0) != '=' )
  {
    printErrorMessage();
    return;
  }
  pin = digitalOutputPins[pin];
  if (textBuffer.substring(1,3) == "hi")
    digitalWrite(pin, HIGH);
  else if (textBuffer.substring(1,3) == "lo")
    digitalWrite(pin, LOW);
  else 
  {
    printErrorMessage();
    return;
  }
  printOK();
}


void writeAnalogPin()
{
  int pin = parseDigits();
  if (pin < 0 || pin >= ANALOG_OUTPUT_PINS_COUNT || textBuffer.charAt(0) != '=' )
  {
    printErrorMessage();
    return;
  }
  textBuffer = textBuffer.substring(1);
  int pwmSetting = parseDigits();
  if (pwmSetting < 0 || pwmSetting > 255)
  {
    printErrorMessage();
    return;
  }
  analogWrite(analogOutputPins[pin],pwmSetting);
  printOK();
}


int parseDigit(char c)
{
  int digit = -1;
  digit = (int) c - 0x30; // subtracting 0x30 from ASCII code gives value
  if(digit < 0 || digit > 9) digit = -1;
  return digit;
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

void printOK()
{
  if (commandId > 0) 
    response(String(commandId));
  responseln(" OK");
  resetWatchDog();
}

void printEcho()
{
  responseln(textBuffer); //printing command back to all clients
}

void printErrorMessage()
{
  responseln("ERROR");
}

void closeConnection()
{
  responseln("\nBye.\n");
  client.stop();
}

void response(String text)
{
  server.print(text);
  Serial.print(text);
}

void responseln(String text)
{
  server.println(text);
  Serial.println(text);
}

float getTemperature(byte* data)
{
  int TReading, SignBit, Tc_100;
  TReading = (data[1] << 8) + data[0];
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit)  TReading = (TReading ^ 0xffff) + 1; // 2's comp
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25
  float temp = 0;
  temp = Tc_100 / 100;  // separate off the whole and fractional portions
  if (SignBit) temp = -1 * temp;
  return temp;
}

void printHelpMessage()
{
responseln("Supported commands:");
responseln("");
responseln(" dr                        - digital read, returns state of digital input");
responseln(" ar                        - analog read, returns all analog inputs");
responseln(" dw2=hi                    - digital write, turns pin 2 ON");
responseln(" dw3=lo                    - digital write, turns pin 3 OFF");
responseln(" aw1=132                   - analog write, sets analog output 1 to PWM 132");
responseln("                             allowed PWM range 0 to 255");
responseln("");
responseln(" ow                        - work with 1-wire bus");
responseln(" owrs                      - resets the bus");
responseln(" owsr                      - searches the bus and prints all device addresses");
responseln(" owsl28:40:69:F7:3:0:0:9A: - selects device by address");
responseln(" owsk                      - skips device selection ");
responseln("                            (useful if you have only one device connected)");
responseln(" owwrBE                    - writes byte BE to the bus");
responseln(" owwpBE                    - same but with bus power enabled");
responseln(" owrd                      - reads data from bus and prints it");
responseln("");
responseln(" we                        - enables hardware watchdog");
responseln(" wd                        - disables hardware watchdog");
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
responseln("  Every command may contain identity prefix like this");
responseln("	id12345ar");
responseln("  If ID is present and syntax is correct, response will conatin ID:");
responseln("	12345 OK");

}

void resetWatchDog()
{
  if (WATCHDOG_ENABLED) digitalWrite(WATCHDOG_PIN, !digitalRead(WATCHDOG_PIN));//trigger watchog
}
