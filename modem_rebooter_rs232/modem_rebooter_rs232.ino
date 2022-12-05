#include <OneWire.h>
#include <DallasTemperature.h>

#define POWEROFF_PIN 8
#define ONE_WIRE_BUS 9 //pin for 1-wire
#define POWER_METER_PIN 0 //ANALOG PIN

#define ANALOG_REFERENCE INTERNAL //Analog refenernce
#define ANALOG_CALIBRATION 192.3 //Let's say 2.56V = 5.632V, so 1023 is 5.5, so 1023/5.5 = 181.6, divider is 1:2.2 (e.g. 1200 Ohm + 1000 Ohm + 0.1uF)

#define RS232_BAUD 9600
#define TEXT_BUFFER_SIZE 39 //length of longest command string plus two spaces for CR + LF

String textBuffer = "";

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);

// the setup routine runs once when you press reset:
void setup() {                
  pinMode(POWEROFF_PIN, OUTPUT);
  digitalWrite(POWEROFF_PIN, LOW); 
  analogReference(ANALOG_REFERENCE);
  
  Serial.begin(RS232_BAUD);
  sensors.begin(); // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  analogRead(POWER_METER_PIN);
  

}

// the loop routine runs over and over again forever:
void loop() {
  
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
  if (textBuffer.substring(0,2) == "p0")
  {
     digitalWrite(POWEROFF_PIN, HIGH); 
     printOK();
  }  
  else if (textBuffer.substring(0,2) == "p1")
  {
     digitalWrite(POWEROFF_PIN, LOW); 
     printOK();
  }
  else if (textBuffer.substring(0,2) == "rs")
  {
     digitalWrite(POWEROFF_PIN, HIGH);
     delay(1500);
     digitalWrite(POWEROFF_PIN, LOW);     
     printOK();
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
  else if (textBuffer.substring(0,2) == "gv")
  {
     response("USB voltage:");
     float voltage = (float)analogRead(POWER_METER_PIN) / ANALOG_CALIBRATION;
     char bbuf[5];
     dtostrf(voltage, 2, 2, bbuf);
     responseln(bbuf);
     printOK();
  }
  else
    printErrorMessage();
  
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

void response(String text)
{
  Serial.print(text);
}

void responseln(String text)
{
  Serial.println(text);
}
