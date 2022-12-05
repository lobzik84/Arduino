#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include <RCSwitch.h>
#define LUMIN_SENSOR_PIN 0 //ANALOG PINS
#define VAC_SENSOR_PIN 6
#define VBAT_SENSOR_PIN 7

#define RX_433_INT 0
#define RX_433_PIN 2
#define GAS_SENSOR_PIN 3 //BOTH digital and analog
#define CHARGE_ENABLE_PIN 4
#define RELAY1_PIN 5
#define RELAY2_PIN 6
#define RELAY3_PIN 7
#define RELAY4_PIN 8
#define PIR_SENSOR_PIN 9
#define ONE_WIRE_BUS 10 //pin for 1-wire bus
#define DHT22_SENSOR_PIN 11
#define BATTERY_TEMP_SENSOR 12 //pin for DS18B20
#define TX_433_PIN 13 
#define LED1_PIN A1
#define LED2_PIN A2
#define MODEM_ENABLE_PIN A4
#define DISPLAY_ENABLE_PIN A5


#define DHTTYPE DHT22

#define ANALOG_REFERENCE DEFAULT //Analog refenernce

#define RS232_BAUD 57600
#define TEXT_BUFFER_SIZE 39 //length of longest command string plus two spaces for CR + LF


String textBuffer = "";
boolean prevPIRstate = false;
boolean prevGasSensorState = false;
DHT dht(DHT22_SENSOR_PIN, DHTTYPE);
RCSwitch mySwitch = RCSwitch();


OneWire internalOneWire(BATTERY_TEMP_SENSOR); // Setup a oneWire instance to communicate with internal DS18B20
DallasTemperature battTempSensor(&internalOneWire);

OneWire oneWire(ONE_WIRE_BUS); // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);

// the setup routine runs once when you press reset:
void setup() {                
  digitalWrite(RELAY1_PIN, true);
  digitalWrite(RELAY2_PIN, true);
  digitalWrite(RELAY3_PIN, true);
  digitalWrite(RELAY4_PIN, true);
  digitalWrite(LED1_PIN, false);
  digitalWrite(LED2_PIN, false);
  digitalWrite(MODEM_ENABLE_PIN, true);
  digitalWrite(DISPLAY_ENABLE_PIN, true);
  digitalWrite(CHARGE_ENABLE_PIN, false);

  analogReference(ANALOG_REFERENCE);
  pinMode(GAS_SENSOR_PIN, INPUT_PULLUP);
  digitalWrite(GAS_SENSOR_PIN, HIGH);
  
  Serial.begin(RS232_BAUD);
  dht.begin();
  sensors.begin();
  battTempSensor.begin();
  mySwitch.enableReceive(RX_433_INT);
  mySwitch.enableTransmit(TX_433_PIN);
  
  analogRead(LUMIN_SENSOR_PIN);
  
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(MODEM_ENABLE_PIN, OUTPUT);
  pinMode(DISPLAY_ENABLE_PIN, OUTPUT);
  pinMode(CHARGE_ENABLE_PIN, OUTPUT);
  

}

// the loop routine runs over and over again forever:
void loop() {
    boolean PIRstate = digitalRead(PIR_SENSOR_PIN);
    if (PIRstate != prevPIRstate) 
    {
      response("PIR_SENSOR:");
      responseln(PIRstate?"true":"false");
      prevPIRstate = PIRstate;
    } 
    boolean gasSensorState = !digitalRead(GAS_SENSOR_PIN);
    if (gasSensorState != prevGasSensorState) 
    {
      response("GAS_SENSOR:");
      responseln(gasSensorState?"true":"false");
      prevGasSensorState = gasSensorState;
    } 
    if (mySwitch.available()) 
    {
      response("433_RX:");
      responseln(String(mySwitch.getReceivedValue(), DEC));
      mySwitch.resetAvailable();
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
  textBuffer.toUpperCase();

  if (textBuffer.substring(0,2) == "GS") 
  {
    char bbuf[30];
    
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    
    response("INTERNAL_TEMP: ");
    dtostrf(t, 2, 1, bbuf);
    responseln(bbuf);
    
    response("INTERNAL_HUMIDITY: ");
    dtostrf(h, 2, 1, bbuf);
    responseln(bbuf);
    
    response("LUMIOSITY: ");
    dtostrf(analogRead(LUMIN_SENSOR_PIN), 2, 0, bbuf);
    responseln(bbuf);
    
    response("VBAT_SENSOR: ");
    dtostrf(analogRead(VBAT_SENSOR_PIN), 2, 0, bbuf);
    responseln(bbuf);
    
    response("VAC_SENSOR: ");
    dtostrf(analogRead(VAC_SENSOR_PIN), 2, 0, bbuf);
    responseln(bbuf);

    response("PIR_SENSOR: ");
    responseln(digitalRead(PIR_SENSOR_PIN)?"true":"false");
    
    response("GAS_SENSOR: ");
    responseln(!digitalRead(GAS_SENSOR_PIN)?"true":"false");
    
    response("GAS_SENSOR_ANALOG: ");
    dtostrf(analogRead(GAS_SENSOR_PIN), 2, 0, bbuf);
    responseln(bbuf);
    
    response("RELAY1: ");
    responseln(!digitalRead(RELAY1_PIN)?"true":"false");
    
    response("RELAY2: ");
    responseln(!digitalRead(RELAY2_PIN)?"true":"false");
    
    response("RELAY3: ");
    responseln(!digitalRead(RELAY3_PIN)?"true":"false");
    
    response("RELAY4: ");
    responseln(!digitalRead(RELAY4_PIN)?"true":"false");
    
    response("LED1: ");
    responseln(digitalRead(LED1_PIN)?"true":"false");
    
    response("LED2: ");
    responseln(digitalRead(LED2_PIN)?"true":"false");
    
    response("MODEM_ENABLED: ");
    responseln(digitalRead(MODEM_ENABLE_PIN)?"true":"false");
    
    response("DISPLAY_ENABLED: ");
    responseln(digitalRead(DISPLAY_ENABLE_PIN)?"true":"false");
    
    response("CHARGE_ENABLED: ");
    responseln(digitalRead(CHARGE_ENABLE_PIN)?"true":"false");
        
    response("BATT_TEMP: ");
    if (battTempSensor.getDeviceCount() == 1) {
      DeviceAddress ds18b20;
      battTempSensor.requestTemperatures();
      char bbuf[5];
      dtostrf(battTempSensor.getTempCByIndex(0), 2, 1, bbuf);
      responseln(bbuf);
    }
    else  responseln("error");

    
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
      response(" DS18B20_" + String(i, DEC) + ": ");
      char bbuf[5];
      dtostrf(sensors.getTempCByIndex(i), 2, 1, bbuf);
      responseln(bbuf);
    }
    printOK(); /*
    //emulation
    response("Requesting temperatures from 4 sensors...");
    delay(1500);
    responseln("DONE");
    response("28:80:a6:26:04:00:00:bc: DS18B20_0: ");
    dtostrf(((float)random(2200,2400))/100.0f, 2, 1, bbuf);
    responseln(bbuf);
    response("28:f4:a7:26:04:00:00:35: DS18B20_1: ");
    dtostrf(((float)random(3500,3800))/100.0f, 2, 1, bbuf);
    responseln(bbuf);
    response("28:99:d2:26:04:00:00:03: DS18B20_2: ");
    dtostrf(((float)random(1400,1700))/100.0f, 2, 1, bbuf);
    responseln(bbuf);
    response("28:a5:b7:26:04:00:00:50: DS18B20_3: ");
    dtostrf(((float)random(1500,1800))/100.0f, 2, 1, bbuf);
    responseln(bbuf);    
    printOK();*/
  }
  else if (textBuffer.substring(0,7) == "RELAY1=") 
  {
    textBuffer = textBuffer.substring(7);
    if (textBuffer.substring(0,2) == "ON")
      digitalWrite(RELAY1_PIN, LOW);
    else if (textBuffer.substring(0,3) == "OFF")
      digitalWrite(RELAY1_PIN, HIGH);
    else 
    {
      printErrorMessage();
      return;
    }
    printOK();
    
  }
  else if (textBuffer.substring(0,7) == "RELAY2=") 
  {
    textBuffer = textBuffer.substring(7);
    if (textBuffer.substring(0,2) == "ON")
      digitalWrite(RELAY2_PIN, LOW);
    else if (textBuffer.substring(0,3) == "OFF")
      digitalWrite(RELAY2_PIN, HIGH);
    else 
    {
      printErrorMessage();
      return;
    }
    printOK();    
  }
  else if (textBuffer.substring(0,7) == "RELAY3=") 
  {
    textBuffer = textBuffer.substring(7);
    if (textBuffer.substring(0,2) == "ON")
      digitalWrite(RELAY3_PIN, LOW);
    else if (textBuffer.substring(0,3) == "OFF")
      digitalWrite(RELAY3_PIN, HIGH);
    else 
    {
      printErrorMessage();
      return;
    }
    printOK();
  }
  else if (textBuffer.substring(0,7) == "RELAY4=") 
  {
    textBuffer = textBuffer.substring(7);
    if (textBuffer.substring(0,2) == "ON")
      digitalWrite(RELAY4_PIN, LOW);
    else if (textBuffer.substring(0,3) == "OFF")
      digitalWrite(RELAY4_PIN, HIGH);
    else 
    {
      printErrorMessage();
      return;
    }
    printOK();
  }
  else if (textBuffer.substring(0,5) == "LED1=") 
  {
    textBuffer = textBuffer.substring(5);
    if (textBuffer.substring(0,2) == "ON")
      digitalWrite(LED1_PIN, HIGH);
    else if (textBuffer.substring(0,3) == "OFF")
      digitalWrite(LED1_PIN, LOW);
    else 
    {
      printErrorMessage();
      return;
    }
    printOK();    
  }
  else if (textBuffer.substring(0,5) == "LED2=") 
  {
    textBuffer = textBuffer.substring(5);
    if (textBuffer.substring(0,2) == "ON")
      digitalWrite(LED2_PIN, HIGH);
    else if (textBuffer.substring(0,3) == "OFF")
      digitalWrite(LED2_PIN, LOW);
    else 
    {
      printErrorMessage();
      return;
    }
    printOK();     
  }
  else if (textBuffer.substring(0,6) == "MODEM=") 
  {
    textBuffer = textBuffer.substring(6);
    if (textBuffer.substring(0,2) == "ON")
      digitalWrite(MODEM_ENABLE_PIN, HIGH);
    else if (textBuffer.substring(0,3) == "OFF")
      digitalWrite(MODEM_ENABLE_PIN, LOW);
    else 
    {
      printErrorMessage();
      return;
    }
    printOK();      
  }
  else if (textBuffer.substring(0,8) == "DISPLAY=") 
  {
    textBuffer = textBuffer.substring(8);
    if (textBuffer.substring(0,2) == "ON")
      digitalWrite(DISPLAY_ENABLE_PIN, HIGH);
    else if (textBuffer.substring(0,3) == "OFF")
      digitalWrite(DISPLAY_ENABLE_PIN, LOW);
    else 
    {
      printErrorMessage();
      return;
    }
    printOK(); 
  }
  else if (textBuffer.substring(0,7) == "CHARGE=") 
  {
    textBuffer = textBuffer.substring(7);
    if (textBuffer.substring(0,2) == "ON")
      digitalWrite(CHARGE_ENABLE_PIN, HIGH);
    else if (textBuffer.substring(0,3) == "OFF")
      digitalWrite(CHARGE_ENABLE_PIN, LOW);
    else 
    {
      printErrorMessage();
      return;
    }
    printOK();     
  }
  else if (textBuffer.substring(0,7) == "433_TX=") 
  {
    textBuffer = textBuffer.substring(7);
    long val = parseLongDigits();
    mySwitch.setRepeatTransmit(10);
    if (textBuffer.substring(0,1) == ",") 
    {
      textBuffer = textBuffer.substring(1);
      int pulse = parseDigits();
      mySwitch.setPulseLength(pulse);
      if (textBuffer.substring(0,1) == ",") 
      {
        textBuffer = textBuffer.substring(1);
        int count = parseDigits();
        for (int i=0; i < count; i++) 
        {
          mySwitch.send(val, 24);
          delay(10);
        }
        responseln("sent:" + String(val, DEC) + " pulse: " + String(pulse, DEC) + " count: " + String(count, DEC));
      }
      else 
      {

        mySwitch.send(val, 24);
        responseln("sent:" + String(val, DEC) + " pulse: " + String(pulse, DEC));
      }
    }
    else
    {
      mySwitch.send(val, 24);
      responseln("sent:" + String(val, DEC));
    }

    printOK();     
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

void response(String text)
{
  Serial.print(text);
}

void responseln(String text)
{
  Serial.println(text);
}
