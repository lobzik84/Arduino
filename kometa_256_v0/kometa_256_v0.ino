#include <SPI.h>
#include <Ethernet.h>
#include <OneWire.h>
#include <EEPROM.h>

#define TEXT_BUFFER_SIZE 39 //length of longest command string plus two spaces for CR + LF
#define SERVER_PORT 23 //telnet server port
#define DIGITAL_OUTPUT_PINS_COUNT 5
#define ANALOG_INPUT_PINS_COUNT 16

//Pins definition. Pins 4,10,11,12,13 used by ethernet-shield and should not be used. Pins 50,51,52,53 is SPI

int digitalOutputPins[DIGITAL_OUTPUT_PINS_COUNT]={33,31,29,27,25};
int analogInputPins[ANALOG_INPUT_PINS_COUNT]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; //0 to 15 analog pins

#define ANALOG_REFERENCE INTERNAL1V1 //Analog refenernce

#define EEPROM_MAC_START_ADDR 0x00 //EEPROM data map
#define EEPROM_IP_START_ADDR 0x06
#define EEPROM_MASK_START_ADDR 0x0a
#define EEPROM_GW_START_ADDR 0x0e
#define EEPROM_CONFIG_BYTE_ADDR 0x12

#define USB_SERIAL_BAUD_RATE 115200
#define MICRON1_SERIAL_BAUD_RATE 9600
#define MICRON2_SERIAL_BAUD_RATE 9600


String textBuffer = "";
int commandId = 0;
//int charsInBuffer = 0;
byte parsedBytesBuffer[12];

EthernetServer server(SERVER_PORT); // Telnet listens on port 23
EthernetClient client = 0; // Client needs to have global scope so it can be called
                   // from functions outside of loop, but we don't know
                   // what client is yet, so creating an empty object


void setup()
{
  
  for (int  i= 0; i < DIGITAL_OUTPUT_PINS_COUNT; i++)  pinMode(digitalOutputPins[i], OUTPUT); 
  for (int  i= 0; i < DIGITAL_OUTPUT_PINS_COUNT; i++) digitalWrite(digitalOutputPins[i], HIGH);

  analogReference(ANALOG_REFERENCE);
 // Open serial communications and wait for port to open:
  Serial.begin(USB_SERIAL_BAUD_RATE);
  while (!Serial) {;} // wait for serial port to connect. Needed for Leonardo only
  
  Serial1.begin(MICRON1_SERIAL_BAUD_RATE);
  Serial2.begin(MICRON2_SERIAL_BAUD_RATE);
  
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

}

void loop()
{
  if (server.available()) 
    client = server.available();
  if (client.connected() && client.available()) 
    readCommand(&client);
  if (Serial.available()) 
    readCommand(&Serial);
  if (Serial1.available())
    readSerial(&Serial1, 1);
  if (Serial2.available())
    readSerial(&Serial2, 2);

}

void readSerial(Stream *str, int portNum)
{
  char c;
  while (str->available())
  {
    c = str->read();
    responseln("Serial" + String(portNum) + ":" + String(c));
    //responseln( String(c));
    str->flush(); //flushing everything else   
  }
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
  if (textBuffer.substring(0,2) == "id")
  {
    textBuffer = textBuffer.substring(2); //remove "id"
    commandId = parseDigits(); //getting commandId and removing it from buffer
  }
  else commandId = 0;
  if (textBuffer.substring(0,2) == "dw")
  {
    textBuffer = textBuffer.substring(2);
    writeDigitalPin();
  }
  else if (textBuffer.substring(0,2) == "ar")
  {
    textBuffer = textBuffer.substring(2);
    readAnalogPins();
  }
  else if (textBuffer.substring(0,2) == "sw")
  {
    textBuffer = textBuffer.substring(2);
    writeSerial();
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


void readAnalogPins()
{
  for (int i = 0; i < ANALOG_INPUT_PINS_COUNT; i++) 
    responseln("IN" + String(i) +": " + String(analogRead(analogInputPins[i])));
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
  if (textBuffer.substring(1,2) == "1")
    digitalWrite(pin, LOW);
  else if (textBuffer.substring(1,2) == "0")
    digitalWrite(pin, HIGH);
  else 
  {
    printErrorMessage();
    return;
  }
  printOK();
}

void writeSerial()
{
  int port = parseDigits();
  if (!(port == 1 || port == 2) || textBuffer.charAt(0) != '=' )
  {
    printErrorMessage();
    return;
  }
  
  if (port == 1) {
    Serial1.print(textBuffer.substring(1));
  }
  else if (port == 2) {
    Serial2.print(textBuffer.substring(1));
  }
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


void printHelpMessage()
{
responseln("Supported commands:");
responseln("");
responseln(" ar                        - analog read, returns all analog inputs");
responseln(" dw2=1                     - digital write, turns pin 2 ON");
responseln(" dw3=0                     - digital write, turns pin 3 OFF");
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
