
#define OUTPUT_PIN 13
#define REMOTE_IN_PIN 2

#define DELAY 3000 //ms

void setup() {
  pinMode(OUTPUT_PIN, OUTPUT);
  digitalWrite(OUTPUT_PIN, LOW);
  pinMode(REMOTE_IN_PIN, INPUT_PULLUP);
  digitalWrite(REMOTE_IN_PIN, LOW);
  
  delay(DELAY);
  
}

void loop() {
  
  if (digitalRead(REMOTE_IN_PIN) && !digitalRead(OUTPUT_PIN)) 
  {
      delay(DELAY);
      digitalWrite(OUTPUT_PIN, HIGH);
   }

  if (!digitalRead(REMOTE_IN_PIN)) 
  {
    digitalWrite(OUTPUT_PIN, LOW);
  }  
  
  
}
