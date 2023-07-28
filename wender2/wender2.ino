#define ANALOG_PIN_AMPLITUDE 0
#define ANALOG_PIN_SPEED 1
#define PIN_WEND 52
#define PIN_FORWARD 51
#define PIN_BACKWARD 53
#define PIN_PULSE A3
#define PIN_DIR A5
#define ADC_CYCLES_PER_CHANNEL 1000
#define PULSE_WIDTH_TIMER_PRELOAD 65200

long delayMicros = 1000; 
long currentMicros = 0; 
long previousMicros = 0;

//массив значений Preload для таймера при проходе вблизи края
//первые 50 шагов - примерно 2мм хода - торможение, потом реверс на 50м шаге, ещё 50 шагов разгон, потом 150 шагов компенсация. среднее значение элементов массива - 1000, т.е. средняя скорость перемещения сохраняется
unsigned int edgeMap[250] = {1000,1000,1000,1000,1000,1001,1001,1002,1003,1004,1005,1006,1007,1008,1010,1012,1014,1016,1018,1020,1023,1026,1029,1033,1037,1041,1046,1051,1056,1063,1070,1077,1086,1096,1107,1120,1134,1151,1171,1195,1223,1258,1302,1358,1433,1539,1697,1962,2492,4083,5000,4083,2492,1962,1697,1539,1433,1358,1302,1258,1223,1195,1171,1151,1134,1120,1107,1096,1086,1077,1070,1063,1056,1051,1046,1041,1037,1033,1029,1026,1023,1020,1018,1016,1014,1012,1010,1008,1007,1006,1005,1004,1003,1002,1001,1001,1000,1000,1000,1000,1000,995,990,985,980,974,969,964,959,954,949,943,938,933,928,923,918,913,908,904,899,894,889,885,880,876,871,867,862,858,854,849,845,841,837,833,829,826,822,818,815,811,808,805,801,798,795,792,789,787,784,781,779,777,774,772,770,768,766,764,763,761,760,758,757,756,755,754,753,752,752,751,751,751,751,750,751,751,751,751,752,752,753,754,755,756,757,758,760,761,763,764,766,768,770,772,774,777,779,781,784,787,789,792,795,798,801,805,808,811,815,818,822,826,829,833,837,841,845,849,854,858,862,867,871,876,880,885,889,894,899,904,908,913,918,923,928,933,938,943,949,954,959,964,969,974,980,985,990,995};

volatile int mapIndex = 0;
volatile int wendIndex = 0;
volatile boolean onEdge = true;
volatile unsigned int preload = 0;
volatile unsigned int wendNumber = 0;

volatile unsigned long MAX_ULONG =  0UL - 1UL;
volatile unsigned long adcBuffer = 0;
volatile unsigned long adcReading = MAX_ULONG;
volatile int bufferIndex = 0;
volatile byte channelIndex = 0;
volatile byte channelReading = 0;

volatile boolean doWend = false;



ISR(TIMER1_OVF_vect)
{
  if (doWend) {
      digitalWrite(PIN_PULSE, false);
      TCNT1 = 1;
      if (onEdge) {
        mapIndex++;
      } else {
        wendIndex++;
      }
      
      if (wendIndex >= wendNumber) {
        wendIndex = 0;
        onEdge = true; 
      }
      
      if (mapIndex >= 250) {
        mapIndex = 0; 
        onEdge = false;       
      }
      if (mapIndex == 50 ) {
        digitalWrite(PIN_DIR, !digitalRead(PIN_DIR));
      }
    }
}

ISR(TIMER3_COMPA_vect)
{
  if (doWend) {
    digitalWrite(PIN_PULSE, true);
    TCNT1 = PULSE_WIDTH_TIMER_PRELOAD;
    OCR3A = TCNT3 + preload +  edgeMap[mapIndex];
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
    if (channelIndex >=2) channelIndex = 0; //считываем только два первых канала
    ADMUX = (channelIndex & 0x07) | 0xC0; //3 channel bits, add REFS1 and REFS0 bits, other is 0 - опорное напряжение АЦП 2.56В
    bufferIndex = 0;
  }
}


void setup() {
  pinMode(PIN_PULSE,OUTPUT);
  pinMode(PIN_DIR,OUTPUT);
  
  pinMode(PIN_WEND,INPUT_PULLUP);
  pinMode(PIN_FORWARD,INPUT_PULLUP);
  pinMode(PIN_BACKWARD,INPUT_PULLUP);

  
  noInterrupts();           // disable all interrupts
  //использую АЦП во free running mode, чтобы постоянно считывать положение регулировочных резисторов, 
  //усреднять значения и не тратить время основного цикла на это
  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX = 0;
  ADMUX |= (1 << REFS1);  // set reference voltage 2.56V
  ADMUX |= (1 << REFS0);  // 

  ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 38.5 KHz

  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete, free running mode
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
    if (channelReading == ANALOG_PIN_AMPLITUDE) {
      wendNumber = 1 + adcReading/ADC_CYCLES_PER_CHANNEL*3; // задаём количество шагов от края до края БЕЗ учёта торможения и разгона, т.е. реальное кол-во шагов будет больше 150 (разгон) + 50 (торможение) больше
    } else if (channelReading == ANALOG_PIN_SPEED) {
      preload = 4100 - adcReading/ADC_CYCLES_PER_CHANNEL*4; //дополнительное значение для таймера, чтобы иметь возможность линейно снижать скорость перемещения
    }
    adcReading = MAX_ULONG;
  }
  
  if (millis() % 100 ==0 ) {
    Serial.print("Preload: ");
    Serial.println(preload);
    Serial.print("wendNumber: ");
    Serial.println(wendNumber);
  }

  
  if (!digitalRead(PIN_WEND)) {
    doWend = true;
  } else {
    doWend = false;
    digitalWrite(PIN_DIR, false);
    mapIndex = 50;
    wendIndex = 0;
    onEdge = true; 
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
