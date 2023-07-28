#define PIN_AMPLITUDE 0
#define PIN_SPEED 1
#define PIN_FUCK 52
#define PIN_FORWARD 51
#define PIN_BACKWARD 53
#define PIN_PULSE A3
#define PIN_DIR A5
#define ADC_CYCLES_PER_CHANNEL 1000
#define PULSE_WIDTH_TIMER_PRELOAD 65400

long delayMicros = 1000; 
long currentMicros = 0; 
long previousMicros = 0;

//255 шагов - примерно 1см хода
//unsigned int speedSinMap[255] = {8148,4074,2716,2038,1630,1359,1165,1020,907,816,743,681,629,584,546,512,482,456,432,411,392,374,359,344,331,318,307,296,287,277,269,261,253,246,240,233,227,222,217,212,207,202,198,194,190,186,183,179,176,173,170,167,165,162,160,157,155,153,150,148,146,145,143,141,139,138,136,134,133,132,130,129,128,126,125,124,123,122,121,120,119,118,117,116,115,114,114,113,112,111,111,110,109,109,108,108,107,107,106,106,105,105,104,104,104,103,103,103,102,102,102,101,101,101,101,101,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,101,101,101,101,101,102,102,102,103,103,103,104,104,104,105,105,106,106,107,107,108,108,109,109,110,111,111,112,113,114,114,115,116,117,118,119,120,121,122,123,124,125,126,128,129,130,132,133,134,136,138,139,141,143,145,146,148,150,153,155,157,160,162,165,167,170,173,176,179,183,186,190,194,198,202,207,212,217,222,227,233,240,246,253,261,269,277,287,296,307,318,331,344,359,374,392,411,432,456,482,512,546,584,629,681,743,816,907,1020,1165,1359,1630,2038,2716,4074,8148};
unsigned int speedSinMap[99] = {3183,1592,1062,797,639,533,458,402,358,323,295,271,251,234,220,207,196,186,177,170,163,156,151,146,141,137,133,129,126,123,120,118,116,114,112,110,108,107,106,105,104,103,102,101,101,100,100,100,100,100,100,100,100,100,101,101,102,103,104,105,106,107,108,110,112,114,116,118,120,123,126,129,133,137,141,146,151,156,163,170,177,186,196,207,220,234,251,271,295,323,358,402,458,533,639,797,1062,1592,3183};


volatile int mapIndex = 0;
volatile int ovfCounter = 0;

volatile unsigned int preload = 0;
volatile unsigned int ovfNumber = 0;

volatile unsigned long MAX_ULONG =  0UL - 1UL;
volatile unsigned long adcBuffer = 0;
volatile unsigned long adcReading = MAX_ULONG;
volatile int bufferIndex = 0;
volatile byte channelIndex = 0;
volatile byte channelReading = 0;

volatile boolean doFuck = false;



ISR(TIMER1_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  if (doFuck) {
     ovfCounter++;
      digitalWrite(PIN_PULSE, false);
      TCNT1 = 1;
      if (ovfCounter >= ovfNumber) {
        mapIndex++;
        ovfCounter = 0;
      }
      if (mapIndex >= 99) {
        mapIndex = 0;
        digitalWrite(PIN_DIR, !digitalRead(PIN_DIR));
      }
    }
}

ISR(TIMER3_COMPA_vect)
{
  if (doFuck) {
    digitalWrite(PIN_PULSE, true);
    TCNT1 = PULSE_WIDTH_TIMER_PRELOAD;
    OCR3A = TCNT3 + preload +  speedSinMap[mapIndex];
  }  
}


ISR(ADC_vect)
{
  if (bufferIndex > 3) adcBuffer += (ADCL | ADCH<<8);  // read 10 bit value from ADC
  bufferIndex++;
  if (bufferIndex == ADC_CYCLES_PER_CHANNEL) {
    channelReading = channelIndex;
    adcReading = adcBuffer;
    adcBuffer = 0;
    channelIndex++;
    if (channelIndex >=2) channelIndex = 0;
    ADMUX = (channelIndex & 0x07) | 0xC0; //3 channel bits, add REFS1 and REFS0 bits, other is 0
    bufferIndex = 0;
  }
}


void setup() {
  pinMode(PIN_PULSE,OUTPUT);
  pinMode(PIN_DIR,OUTPUT);
  
  pinMode(PIN_FUCK,INPUT_PULLUP);
  pinMode(PIN_FORWARD,INPUT_PULLUP);
  pinMode(PIN_BACKWARD,INPUT_PULLUP);

  
  noInterrupts();           // disable all interrupts
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX = 0;
  ADMUX |= (1 << REFS1);  // set reference voltage 2.56V
  ADMUX |= (1 << REFS0);  // 

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz

  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements

  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 1;            
  TCCR1B |= (1 << CS11);    // 8 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt

  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 0;
  
  TCCR3B |= (0 << WGM12);
  TCCR3B |= (1 << CS11);    // 8 prescaler 

  TIMSK3 |= (1 << OCIE3A);   
  interrupts(); 
  
  Serial.begin(115200);
}

void loop() {
  
  if (adcReading != MAX_ULONG) {
    //Serial.println ("ADC" + String(channelReading) + ": " + String(adcReading)); 
    if (channelReading == PIN_AMPLITUDE) {
      ovfNumber = 1 + adcReading/ADC_CYCLES_PER_CHANNEL/100;
    } else if (channelReading == PIN_SPEED) {
      preload = 4320 - adcReading/ADC_CYCLES_PER_CHANNEL*4;
    }
    adcReading = MAX_ULONG;
  }
  
  if (millis() % 100 ==0 ) {
    Serial.print("Preload: ");
    Serial.println(preload);
    Serial.print("ovfNumber: ");
    Serial.println(ovfNumber);
  }

  
  if (!digitalRead(PIN_FUCK)) {
    doFuck = true;
  } else {
    doFuck = false;
    mapIndex = 0;
    delayMicros = 1000;
    if (!digitalRead(PIN_FORWARD)) {
      digitalWrite(PIN_DIR, HIGH);
      doStep();
    } else if (!digitalRead(PIN_BACKWARD)) {
      digitalWrite(PIN_DIR, LOW);
      doStep();
    } 
  }

}

void doStep() {
  
  currentMicros = micros();
  if(currentMicros - previousMicros >= delayMicros) {
    previousMicros = currentMicros;
    digitalWrite(PIN_PULSE,HIGH);
    delayMicroseconds(50);
    digitalWrite(PIN_PULSE,LOW);
  } 
}
