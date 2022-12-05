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

#define EEPROM_STEPBACK_THRESHOLD_ADDR 0x14
#define EEPROM_SIMULATION_TIME_ADDR 0x18
#define EEPROM_DEFLECTION_TIMEOUT_ADDR 0x1c

#define EEPROM_CONTROL_VALUE_ADDR 0xEE
#define EEPROM_CONTROL_VALUE 0xEB

//Pins definition. Pins 4,10,11,12,13 used by ethernet-shield and should not be used. Pins 50,51,52,53 is SPI

/*
  Board	          int.0	  int.1	  int.2	  int.3	  int.4	  int.5
 Mega2560	  2	  3	  21	  20	  19	  18
 */
 
#define ANALOG_REFERENCE INTERNAL1V1

#define X_IN_PIN A14  //X и Y немного перепутаны со здравым смыслом для введения в заблуждение врага )) в общем, так вышло, что X - сигнал, а Y - пила.
#define Y_IN_PIN A15
#define FILTER_RELAY_PIN 34
#define STEPBACK_LED_PIN 36

#define STEPBACK_FILTER 20

int STEPBACK_THRESHOLD = 500;
long SIMULATION_TIME = 2000;
long DEFLECTION_TIMEOUT = 10000; //TODO configurable via USB

int simulator = 0;

byte MAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
byte IP[4] = {192, 168, 0, 10};
byte MASK[4] = {255, 255, 255, 0};
byte GW[4] = {192, 168, 0, 1};
boolean DHCP = false;

String textBuffer = "";


byte parsedBytesBuffer[12];
EthernetServer server(SERVER_PORT); // Telnet listens on port 23
EthernetClient client; // Client needs to have global scope so it can be called
                   // from functions outside of loop, but we don't know
                   // what client is yet, so creating an empty object

long start = millis();
int cnt = 0;
int yArr[STEPBACK_FILTER];
int yArrInd = 0;
boolean prevStepBack = false;
unsigned long xArr[1024];
int xCntArr[1024];
long framePoints = 0;
long frameStartMs = 0;

boolean simulate = false;

union mytype
{
  long l;
  byte b[4];
} tmp; //for long to byte[4] conversion and back

void zeroXArr()
{
  for (int i=0; i<1024; i++)
  {
    xArr[i]=0;
    xCntArr[i]=0;  
  }
}
void setup()
{

 // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {;} // wait for serial port to connect. Needed for Leonardo only
    if (checkEEPROM())
  {
    readNetworkSettings();
    readDeviceSettings();
  }
  initNetwork();
  for (int i=0; i<STEPBACK_FILTER; i++) yArr[i] = 0;
  zeroXArr();
  pinMode(STEPBACK_LED_PIN, OUTPUT);
  digitalWrite(STEPBACK_LED_PIN, false);
  pinMode(FILTER_RELAY_PIN, OUTPUT);
  digitalWrite(FILTER_RELAY_PIN, false);
  analogReference(ANALOG_REFERENCE);
} 


void loop()
{
 // 
  int xAdc = analogRead(X_IN_PIN);
  int yAdc = analogRead(Y_IN_PIN);
  if (simulate) 
  {
    if (simulator >200 && simulator <400) xAdc = simulator/2; else xAdc = 10;
    yAdc = simulator;
    simulator++;
    if (simulator > 1023) simulator = 0;
  }
  yArr[yArrInd] = yAdc;
  xArr[yAdc] = xArr[yAdc] + xAdc;
  xCntArr[yAdc] = xCntArr[yAdc] + 1;
  framePoints++;
  
  long firstHalf = 0;
  long lastHalf = 0;
  for (int i=yArrInd-(STEPBACK_FILTER/2)+1; i <= yArrInd; i++) 
  {
    if (i < 0)
      lastHalf += yArr[i+STEPBACK_FILTER];
    else
      lastHalf += yArr[i];
  }
  
  for (int i=yArrInd-STEPBACK_FILTER+1; i <= yArrInd-(STEPBACK_FILTER/2); i++) 
  {
    if (i < 0)
      firstHalf += yArr[i+STEPBACK_FILTER];
    else
      firstHalf += yArr[i];
  }
  
  if (millis() - frameStartMs > DEFLECTION_TIMEOUT) 
  {
      for (int sock=0; sock < 4; sock++) 
  {
    EthernetClient cli = server.available_(sock);
    if (cli) {
        if (cli.connected()) cli.println(xAdc*8);
    }  
  }
    //server.println();
    digitalWrite(STEPBACK_LED_PIN, true);
  }
    
  if (firstHalf - lastHalf > STEPBACK_THRESHOLD ||(simulate && millis() - frameStartMs > SIMULATION_TIME))
  {
    digitalWrite(STEPBACK_LED_PIN, true);
    char buf[11];
    if (!prevStepBack)
    {
      Serial.print("stepback:");
      Serial.print(firstHalf, DEC);
      Serial.print(",");
      Serial.println(lastHalf, DEC);\
      for (int sock=0; sock < 4; sock++) 
      {
        EthernetClient cli = server.available_(sock);
        if (cli) {
            if (cli.connected()) {
                   cli.print("FRAME: ");
                  for (int i=0; i<1024; i++)
                  {
                    if (xCntArr[i] > 0)
                    {
                      int val = xArr[i]*8/xCntArr[i];
                      sprintf(buf, "%d,%d;", i, val); 
                      cli.print(buf);//1536 hz
                    }
                  }
                  cli.print("END FRAME: ");
                  cli.print(millis() - frameStartMs, DEC);
                  cli.print(" ms, ");
                  cli.print(framePoints, DEC);
                  cli.println(" points");                       
            }
        }  
      }

      zeroXArr();
    }
    prevStepBack = true; 
    framePoints = 0;
    frameStartMs = millis();;
  }
  else
  {
    prevStepBack = false; 
    if (millis() - frameStartMs < DEFLECTION_TIMEOUT) digitalWrite(STEPBACK_LED_PIN, false);
  }
  
  yArrInd++;
  if (yArrInd >= STEPBACK_FILTER) yArrInd = 0;
  
  /*server.print(xAdc, DEC);
  server.print(";");
  server.println(yAdc, DEC); */
  cnt++;
  if (millis() - start > 10000)
  {
    double freq = cnt/10;
    Serial.print(freq, DEC);
    Serial.println("Hz");
    start = millis();
    cnt = 0;
  }
  for (int sock=0; sock < 4; sock++) 
  {
    EthernetClient cli = server.available_(sock);
    if (cli) {
        if (cli.connected()) readCommand(&cli);
    }  
  }
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
  else if (textBuffer.substring(0,3) == "pns")
  {//print 
    printNetworkSettings();
  }
  else if (textBuffer.substring(0,3) == "pds")
  {//print 
    printDeviceSettings();
  }
  else if (textBuffer.substring(0,2) == "s1")
  {
    simulate = true;
  }  
  else if (textBuffer.substring(0,2) == "s0")
  {
    simulate = false;
  }
  else if (textBuffer.substring(0,2) == "f1")
  {
    digitalWrite(FILTER_RELAY_PIN, true);
  }
  else if (textBuffer.substring(0,2) == "f0")
  {
    digitalWrite(FILTER_RELAY_PIN, false);
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
  server.begin();
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
  response("Stepback Threshold (sdth): "); response(String(STEPBACK_THRESHOLD, DEC)); responseln("pts");
  response("Deflection Timeout (sddt): "); response(String(DEFLECTION_TIMEOUT, DEC)); responseln("ms");
  response("Simulation Time (sdst): "); response(String(SIMULATION_TIME, DEC)); responseln("ms");
  response("Simulation (s0/s1): ");
  if (simulate)
    responseln("ON");
  else
    responseln("OFF");
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
  for (int i = 0; i < 4; i++) tmp.b[i] = EEPROM.read(EEPROM_STEPBACK_THRESHOLD_ADDR + i);
  STEPBACK_THRESHOLD = tmp.l;
  for (int i = 0; i < 4; i++) tmp.b[i] = EEPROM.read(EEPROM_DEFLECTION_TIMEOUT_ADDR + i);
  DEFLECTION_TIMEOUT = tmp.l;
  for (int i = 0; i < 4; i++) tmp.b[i] = EEPROM.read(EEPROM_SIMULATION_TIME_ADDR + i);
  SIMULATION_TIME = tmp.l;
}

void doDeviceSettings()
{
    if (textBuffer.substring(0,3) == "th=")
    {
       textBuffer = textBuffer.substring(3);
       long value = parseLongDigits();
       STEPBACK_THRESHOLD = value;
       writeDeviceSettings();
       printOK();
    }
    else if (textBuffer.substring(0,3) == "dt=")
    {
       textBuffer = textBuffer.substring(3);
       long value = parseLongDigits();
       DEFLECTION_TIMEOUT = value;
       writeDeviceSettings();
       printOK();
    }
    else if (textBuffer.substring(0,3) == "st=")
    {
       textBuffer = textBuffer.substring(3);
       long value = parseLongDigits();
       SIMULATION_TIME = value;
       writeDeviceSettings();
       printOK();
    }
    else
      printErrorMessage();
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
  tmp.l = STEPBACK_THRESHOLD;
  for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_STEPBACK_THRESHOLD_ADDR + i, tmp.b[i]);
  tmp.l = DEFLECTION_TIMEOUT;
  for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_DEFLECTION_TIMEOUT_ADDR + i, tmp.b[i]);
  tmp.l = SIMULATION_TIME;
  for (int i = 0; i < 4; i++) EEPROM.write(EEPROM_SIMULATION_TIME_ADDR + i, tmp.b[i]);
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

