#define PIN_AMPLITUDE A0
#define PIN_SPEED A1
#define PIN_FUCK 52
#define PIN_FORWARD 51
#define PIN_BACKWARD 53
#define PIN_PULSE 9
#define PIN_DIR 8
#define ANALOG_REFERENCE INTERNAL2V56
#define AVG_CNT 1024

//int period = 1200;

long delayMicros = 1000; 
long currentMicros = 0; 
long previousMicros = 0;
long lastMillis = 0;
int cnt = 0;
int us = 1623;
long period = 318319;
byte pinAmps[AVG_CNT];
byte pinSpds[AVG_CNT];


void setup() {
  analogReference(ANALOG_REFERENCE);
  pinMode(PIN_PULSE,OUTPUT);
  pinMode(PIN_DIR,OUTPUT);
  
  pinMode(PIN_FUCK,INPUT_PULLUP);
  pinMode(PIN_FORWARD,INPUT_PULLUP);
  pinMode(PIN_BACKWARD,INPUT_PULLUP);
  
  
  Serial.begin(115200);
}

void loop() {
  pinAmps[cnt] = analogRead(PIN_AMPLITUDE)/4;
  pinSpds[cnt] = analogRead(PIN_SPEED)/4;
  
  if (millis() % 500 == 0) {
    unsigned long sumAmps = 0;
    unsigned long sumSpds = 0;
    
    for (int i=0; i < AVG_CNT; i++) {
      sumAmps += pinAmps[i];
      sumSpds += pinSpds[i];
    }
    
    us = 3600 - (((sumAmps / AVG_CNT)*118)/10);
    period = 100000 + ((sumSpds / AVG_CNT)*30000)/10;
    /*Serial.print("Amplitude: ");
    Serial.println(period);
    Serial.print("Speed: ");
    Serial.println(us);*/
  }

  
  if (!digitalRead(PIN_FUCK)) {
    fuck(us, (float)period);
  } else {
    delayMicros = 1000;
    if (!digitalRead(PIN_FORWARD)) {
      digitalWrite(PIN_DIR, HIGH);
      doStep();
    } else if (!digitalRead(PIN_BACKWARD)) {
      digitalWrite(PIN_DIR, LOW);
      doStep();
    } 
  }
  cnt++;
  if (cnt >= AVG_CNT) {
    cnt = 0;
  }
}

void fuck(int usMin, float period) {

  float spd = sin((float)micros()/period);

  if (spd > 0) {
   digitalWrite(PIN_DIR, HIGH);
  } else {
    digitalWrite(PIN_DIR, LOW);
  }

  spd = abs(spd);
  if (spd < 0.005) {
    spd = 0.005;
  }

  delayMicros = usMin / spd;
  doStep();
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

