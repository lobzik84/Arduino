#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>


#define TEXT_BUFFER_SIZE 539 //length of longest command string plus two spaces for CR + LF
#define SERVER_PORT 23 //telnet server port

#define EEPROM_MAC_START_ADDR 0x00 //EEPROM data map
#define EEPROM_IP_START_ADDR 0x06
#define EEPROM_MASK_START_ADDR 0x0a
#define EEPROM_GW_START_ADDR 0x0e
#define EEPROM_CONFIG_BYTE_ADDR 0x12

#define EEPROM_DEV_IP_START_ADDR 0x13 //19 bytes,up to 0x26
#define EEPROM_DEV_CNT_ADDR 0x27

#define EEPROM_CONTROL_VALUE_ADDR 0xEE

#define EEPROM_CONTROL_VALUE 0xEB

//Pins definition. Pins 4,10,11,12,13 used by ethernet-shield and should not be used. Pins 50,51,52,53 is SPI

/*
  Board	          int.0	  int.1	  int.2	  int.3	  int.4	  int.5
 Mega2560	  2	  3	  21	  20	  19	  18
 */
 

#define ENC_A_PIN 2
#define ENC_A_INT 0 
#define ENC_B_PIN 3
#define BUTTON_PIN 44

#define MAX_DEVICES 19
#define CONNECT_PERIOD_MS 1000

// preload timer 65536-16MHz/256/4Hz
#define THR_CNT_PRELOAD 49911

byte MAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte IP[4] = {192, 168, 0, 10};
byte MASK[4] = {255, 255, 255, 0};
byte GW[4] = {192, 168, 0, 1};
boolean DHCP = false;

byte DEVS[MAX_DEVICES];
byte DEV_CNT = 0;
byte device = 0;
byte prevDevice = 0;
boolean axis = true;// X true

byte LED_DISP_PIN[11] = {22,24,26,28,30,32,34,36,38,40,42};
byte LED_DISP_SYNT[10] = {119,36,94,110,45,107,123,38,127,111}; 
/*
0 = 1110111 1110111= 119
1 = 0010010 0100100= 36
2 = 0111101 1011110= 94
3 = 0111011 1101110= 110
4 = 1011010 0101101= 45
5 = 1101011 1101011= 107
6 = 1101111 1111011= 123
7 = 0110010 0100110= 38
8 = 1111111 1111111= 127
9 = 1111011 1101111= 111
*/

/*byte LED_DISP_1[2] = {};
byte LED_DISP_2[5] = {};
byte LED_DISP_3[5] = {};
byte LED_DISP_4[4] = {};
byte LED_DISP_5[5] = {};
byte LED_DISP_6[6] = {};
byte LED_DISP_7[3] = {};
byte LED_DISP_8[7] = {};
byte LED_DISP_9[6] = {};
*/
String textBuffer = "";


byte parsedBytesBuffer[12];
//EthernetServer server(SERVER_PORT); // Telnet listens on port 23
EthernetClient client; // Client needs to have global scope so it can be called
                   // from functions outside of loop, but we don't know
                   // what client is yet, so creating an empty object

volatile boolean displayBlink = true;
volatile int displayState = 0;
volatile int angle = 0;
volatile int prevAngle = 0;

int steps = 0;
boolean stepping = false;
long lastGo = 0;
long lastMove = 0;
long lastConnAtt = 0;

ISR(TIMER1_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  TCNT1 = THR_CNT_PRELOAD;
  if (!displayBlink && displayState == 0) return;
  if (displayState > 0) {
    for (int i=0; i<11; i++) digitalWrite(LED_DISP_PIN[i], !bitRead(displayState, i));
    displayState = 0;
  }
  else  {
    for (int i=0; i<11; i++) {
      bitWrite(displayState, i, !digitalRead(LED_DISP_PIN[i])); 
      digitalWrite(LED_DISP_PIN[i], true);
    }
  }

}

void encA() 
{
  if (digitalRead(ENC_B_PIN) && digitalRead(ENC_A_PIN)) { angle--; }
  else if (!digitalRead(ENC_B_PIN) && digitalRead(ENC_A_PIN)) { angle++; }
}

void setup()
{

  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  digitalWrite(ENC_A_PIN, LOW);
  digitalWrite(ENC_B_PIN, LOW);
  digitalWrite(BUTTON_PIN, LOW);
  for (int i=0; i<11; i++) { pinMode(LED_DISP_PIN[i], OUTPUT); digitalWrite(LED_DISP_PIN[i], true); }
  noInterrupts();
  attachInterrupt(ENC_A_INT, encA, CHANGE);
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = THR_CNT_PRELOAD;            
  TCCR1B |= (1 << CS12);    // 8 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts(); 

 // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {;} // wait for serial port to connect. Needed for Leonardo only
    if (checkEEPROM())
  {
    readNetworkSettings();
    readDeviceSettings();
  }
  initNetwork();

//  Serial.println();
  displayInfo(device, axis);

}

void loop()
{
 // 
  displayBlink = !client.connected() && (device > 0) && !digitalRead(BUTTON_PIN);
  
  if (device != prevDevice && !digitalRead(BUTTON_PIN)) {
    displayInfo(device, axis);
    prevDevice = device;
    client.stop();
  }

  if (device > 0 && !client.connected() && millis() - lastConnAtt > CONNECT_PERIOD_MS && !digitalRead(BUTTON_PIN))  {

    client.stop();
    steps = 0;
    connectTo(DEVS[device-1]);
    lastConnAtt = millis();
  }
  
  
  if (angle != prevAngle) 
  {
    if (digitalRead(BUTTON_PIN)) {
      if (axis && angle > prevAngle) { 
        axis = false; 
        displayInfo(device, axis);
      }  else if (!axis && angle < prevAngle) { 
        axis = true; 
        displayInfo(device, axis);
      } else { 
      if (angle > prevAngle) {
        if (device <= DEV_CNT-1) { 
          device++; 
          axis = !axis; 
        }
      } else if (device > 0) {
        device--;
        axis = !axis;
      }
     
      displayInfo(device, axis);
      }
    }
    else {
      long ms = millis() - lastMove;
      if (ms > 50) ms = 50;
      if (ms < 10) ms = 10;
      steps = steps + (((long)(angle - prevAngle)*50)/ms);
      lastMove = millis();
    }
    prevAngle = angle;
  }
  
  if (!stepping && steps != 0) {
      if (client.connected()) {
        if (axis) {client.print("x");} else {client.print("y");}
        
        if (steps > 0) client.print("+");
        
        client.println(steps, DEC);
        stepping = true;
        lastGo = millis();
        steps = 0;
      }
  }
  if (stepping && millis() - lastGo > 5000)
    stepping = false;
    
  if (client.available()) {
    char c = client.read();
    if(c == 75) { delay(10); stepping = false;}// 'OK'
    Serial.print(c);
  }
  if (Serial.available()) 
    readCommand(&Serial);
}

void displayInfo(byte dev, boolean axis) {
  displayState = 0;
  for (int i=0; i<11; i++) pinMode(LED_DISP_PIN[i], OUTPUT);
  for (int i=0; i<11; i++) digitalWrite(LED_DISP_PIN[i], true);
  if (dev >= 10) 
  {
    for (int i=0; i<7; i++) digitalWrite(LED_DISP_PIN[i+1], !bitRead(LED_DISP_SYNT[dev-10], i));
    digitalWrite(LED_DISP_PIN[0], false);
  } 
  else 
  {
    for (int i=0; i<7; i++) digitalWrite(LED_DISP_PIN[i+1], !bitRead(LED_DISP_SYNT[dev], i));
  }
  if (dev > 0) {
     digitalWrite(LED_DISP_PIN[8], false);
     if (axis) { digitalWrite(LED_DISP_PIN[9], false); } else { digitalWrite(LED_DISP_PIN[10], false); }
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
  long steps = 0;
  textBuffer.trim();
  textBuffer.toLowerCase();
  if (textBuffer.substring(0,3) == "snp")
  {//set network parameters
    textBuffer = textBuffer.substring(3);
    doNetworkSettings();
    writeNetworkSettings();
  }
  else if (textBuffer.substring(0,2) == "sd")
  {
    textBuffer = textBuffer.substring(2);
    doDeviceSettings();
    writeDeviceSettings();
  }
  else if (textBuffer.substring(0,4) == "dev=")
  {
    textBuffer = textBuffer.substring(4);
    byte dev = parseDigits();
    if (dev >=0 && dev <= MAX_DEVICES) {
      device = dev; 
      printOK();
    } else {
      printErrorMessage();
    }
  }  
  else if (textBuffer.substring(0,3) == "pns")
  {//print 
    printNetworkSettings();
  }
  else if (textBuffer.substring(0,3) == "pds")
  {//print 
    printDeviceSettings();
  }
  else if (textBuffer.substring(0,2) == "cl")
  {
    closeConnection();
  }
  else if (textBuffer.substring(0,1) == "?")
  {
    printHelpMessage();
  }
  else 
  {
    if (client.connected()) {
      client.println(textBuffer); 
    }
  
  }
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
   delay(1000);

}

void connectTo(byte lastByte) 
{
    IPAddress server(Ethernet.localIP()[0],Ethernet.localIP()[1],Ethernet.localIP()[2],lastByte);
    Serial.print("connecting to ");
    for (byte thisByte = 0; thisByte < 4; thisByte++) 
    {
      // print the value of each byte of the IP address:
      Serial.print(server[thisByte], DEC);
      Serial.print("."); 
    }
    Serial.println(); 
  // if you get a connection, report back via serial:
  if (client.connect(server, 23)) {
    Serial.println("connected");
  }  else {
    // if you didn't get a connection to the server:
    Serial.println("connection failed");
  }
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

 void printDeviceSettings()
{
  response("Devices count: ");
  responseln(String(DEV_CNT, DEC));
  for (int i=0; i<DEV_CNT; i++) {
    response("DEV[");response(String(i+1, DEC));response("]: ip=");
    for (int j = 0; j < 3; j++) response(String(IP[j], DEC) + ".");
    responseln(String(DEVS[i], DEC));
  }
  response("Current device: ");
  responseln(String(device, DEC));
  printOK();
}

void readNetworkSettings()
{
  for (int i = 0; i < 6; i++) MAC[i] = EEPROM.read(EEPROM_MAC_START_ADDR + i); //reading 6 bytes of MAC from EEPROM
  for (int i = 0; i < 4; i++) IP[i] = EEPROM.read(EEPROM_IP_START_ADDR + i);
  for (int i = 0; i < 4; i++) MASK[i] = EEPROM.read(EEPROM_MASK_START_ADDR + i);
  for (int i = 0; i < 4; i++) GW[i] = EEPROM.read(EEPROM_GW_START_ADDR + i);  
  DHCP = bitRead(EEPROM.read(EEPROM_CONFIG_BYTE_ADDR), 0);
}

void readDeviceSettings()
{
  for (int i = 0; i < MAX_DEVICES; i++) DEVS[i] = EEPROM.read(EEPROM_DEV_IP_START_ADDR + i); 
  DEV_CNT = EEPROM.read(EEPROM_DEV_CNT_ADDR);
}

void doDeviceSettings()
{
  if (textBuffer.substring(0,4) == "cnt=") {
    textBuffer = textBuffer.substring(4);
    byte dev = parseDigits();
    if (dev >=0 && dev <= MAX_DEVICES) {
      DEV_CNT = dev; 
      printOK();
    } else {
      printErrorMessage();
    }
  }
  else if (textBuffer.substring(0,2) == "ip") {
    textBuffer = textBuffer.substring(2); 
    byte dev = parseDigits();
    if (dev > 0 && dev <= MAX_DEVICES && textBuffer.substring(0,1) == "=") {
       textBuffer = textBuffer.substring(1);
       byte ip = parseDigits();
       DEVS[dev-1] = ip;
      printOK();
    } else {
      printErrorMessage();
    }
  }
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
  Serial.print(text);
}

void responseln(String text)
{
  Serial.println(text);
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

void writeDeviceSettings()
{
  for (int i = 0; i < MAX_DEVICES; i++) EEPROM.write(EEPROM_DEV_IP_START_ADDR + i, DEVS[i]); 
  EEPROM.write(EEPROM_DEV_CNT_ADDR, DEV_CNT);
}

void printHelpMessage()
{/*
responseln("Supported commands:");
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
*/
}

