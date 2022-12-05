#define SYNC_PIN 2
#define SYNC_INT 0
#define OUT_PIN 3
#define FREQ_REG_PIN 7 //analog in A7

#define TRIAC_PULSE_WIDTH_TIMER_PRELOAD 65200
#define PULSE_WIDTH_US 2

long start = 0l;
int freq =0;

volatile long lastSyncTime = 0l;
volatile boolean guardInterval = false;

void onSync() {
  if (!guardInterval) {
    digitalWrite(OUT_PIN, true);
    TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
    lastSyncTime = millis();
  }
}


ISR(TIMER1_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  digitalWrite(OUT_PIN, false);
  TCNT1 = 1;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  pinMode(SYNC_PIN, INPUT);
  pinMode(OUT_PIN, OUTPUT);
  digitalWrite(OUT_PIN, false);

  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  TCNT1 = 1;            
  TCCR1B |= (1 << CS11);    // 8 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt 

  interrupts(); 
  
  attachInterrupt(SYNC_INT, onSync, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - lastSyncTime < 2) { //2ms guard interval
    guardInterval = true;
  } else {
    guardInterval = false;
  
    if (millis() - lastSyncTime > 3000) {
      int val2 = 1365 - analogRead(FREQ_REG_PIN);
      delay(val2/68);
      //freq++;
      digitalWrite(OUT_PIN, true);
      TCNT1 = TRIAC_PULSE_WIDTH_TIMER_PRELOAD;
    
    //delayMicroseconds(PULSE_WIDTH_US);
   // digitalWrite(OUT_PIN, false);
    /*if (millis() - start > 1000) {
      start = millis();
      Serial.println( freq );
      freq = 0;
    }*/
    }  
  }
}
