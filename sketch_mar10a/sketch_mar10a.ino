#include <stdlib.h>

#define RX_INT 0 //pin 2
#define MINPULSEWIDTH 400
#define MAXPULSEWIDTH 1000
#define SILENCEWIDTH 40000 //80ms
#define MANCHESTERBITS 160
#define MANCHESTERBYTES 20
#define HEADERONES 15


//Interface Definitions
int     rxPin      = 2;     //The number of signal from the Rx
int     ledPin     = 13;    //The number of the onboard LED pin

//Bank array for packet (at least one will be needed)
byte    manchester[MANCHESTERBYTES];      //Stores manchester pattern decoded on the fly
//Oregon bit pattern, causes nibble rotation to the right, ABCDabcd becomes DCBAdcba
byte    oregon[]   = {
//  8, 4, 2, 1, 128, 64, 32, 16
    16, 32, 64, 128, 1, 2, 4, 8
};

volatile long lastRxChangeMicros = 0l;
volatile bool silence = false;
volatile bool manchesterBits[MANCHESTERBITS] ;
volatile int mbIndex = 0;
volatile int onesCnt = 0;
volatile bool prev = false;

void rxInt() {
  long pulseTime = micros() - lastRxChangeMicros;
  lastRxChangeMicros = micros();
  bool val = digitalRead(rxPin);  
  if (pulseTime >= SILENCEWIDTH || silence) {
    silence = true;
    return;
  }
  if (pulseTime > MINPULSEWIDTH && pulseTime < MAXPULSEWIDTH) {
    manchesterBits[mbIndex] =  val ^ prev;
    prev = val;
    mbIndex++;
    if (mbIndex >= MANCHESTERBITS) {
      mbIndex = 0;
    }
  }
}


void parseManchesterBits() {
  {//process
    for (int i = 0; i < MANCHESTERBYTES; i++) {
      manchester[i] = 0;
    }

    for (int i=0; i < MANCHESTERBITS; i++) {
      
    }
    
    for (int i=0; i < MANCHESTERBITS; i = i + 8) {
        byte b = 0;
        for (int k = 0; k < 8; k++) {
          int j = i + k + mbIndex;
          if (j > MANCHESTERBITS) {
            j = j - MANCHESTERBITS;
          }
          if (manchesterBits[j]) {
            b |= oregon[k];            
            //bitSet(b, k);
          }
          manchester[i/8] = b;
        } 
    }
    

    Serial.println("data recieved ");
    
    for (int i = 0; i < MANCHESTERBYTES; i++) {
      if (manchester[i] < 0x10) {
        Serial.print("0");
      }
      Serial.print(manchester[i], HEX);
      Serial.print(" ");
    }
    Serial.println(" ---- ");
  }
  mbIndex = 0;
  for (int i=0; i < MANCHESTERBITS; i++) {
    manchesterBits[i] = false;

  }
}

void setup() {
  Serial.begin(115200);
  pinMode(rxPin, INPUT);
  pinMode(ledPin, OUTPUT);
  delay(1000);
  parseManchesterBits();
  noInterrupts();   
  attachInterrupt(RX_INT, rxInt, CHANGE);
  interrupts();  
}

void loop() {
  if (silence) {
    parseManchesterBits();
    silence = false;
  }
}


