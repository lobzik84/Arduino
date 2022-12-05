#include <OneWire.h>
#include <DallasTemperature.h>

#define TACHO_INT 0
#define TACHO_PIN 2
#define PWN_PIN 11
#define ONE_WIRE_BUS 5

#define TEXT_BUFFER_SIZE 39 //length of longest command string plus two spaces for CR + LF

volatile long tachoCnt = 0;
long tachoMillis = 0;
long rpm = 0;
//buffers
String textBuffer = "";  
byte parsedBytesBuffer[12];

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);

void setup() {
  pinMode(TACHO_PIN, INPUT_PULLUP);
  
  pinMode(PWN_PIN, OUTPUT);
  attachInterrupt(TACHO_INT, tacho, CHANGE);
  analogWrite(PWN_PIN, 255);
  sensors.begin();
  Serial.begin(115200); 
}

void tacho() {
  tachoCnt++;  
}

// the loop function runs over and over again forever
void loop() {
  tachoMeter();
  if (Serial.available()) readCommand(&Serial);
}

void tachoMeter() {
    if (millis()  - tachoMillis > 1000) {
      rpm = tachoCnt * 30;
      tachoCnt = 0;
      tachoMillis = millis();
      Serial.print("RPM: ");
      Serial.println(rpm);
      printTemp();
  }

  
}

void printTemp() {
    Serial.print("TEMP:");
    if (sensors.getDeviceCount() == 1) {
      DeviceAddress ds18b20;
      sensors.requestTemperatures();
      char bbuf[5];
      dtostrf(sensors.getTempCByIndex(0), 2, 1, bbuf);
      Serial.println(bbuf);
    }
    else  Serial.println("error");
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
  if (textBuffer.substring(0,2) == "p=")
  {//set PWM
    textBuffer = textBuffer.substring(2);
    int value = 0;
    value = parseDigits();
    if (value > 60 && value < 256) {
        analogWrite(PWN_PIN, value);
        printOK();
    }
    
  }
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

int parseDigit(char c)
{
  int digit = -1;
  digit = (int) c - 0x30; // subtracting 0x30 from ASCII code gives value
  if(digit < 0 || digit > 9) digit = -1;
  return digit;
}

void printEcho()
{
  Serial.println(textBuffer); //printing command back to all clients
}

void printOK()
{
 Serial.println(" OK");
}


void printErrorMessage()
{
  Serial.println("ERROR: unrecognized command");
}

