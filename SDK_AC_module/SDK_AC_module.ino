#include <IRremote.h>

#define IR_RECIEVER_INT 0
#define IR_RECIEVER_INT_PIN 2
#define IR_RECIEVER_PIN 11

#define CONTOL_LED_PIN 13
#define IR_LED_PIN 4

#define SERIAL_BAUD_RATE 57600

IRrecv irrecv(IR_RECIEVER_PIN);
decode_results results;


void irRecieverInt() {
  boolean recieverState = digitalRead(IR_RECIEVER_INT_PIN);
  digitalWrite(CONTOL_LED_PIN, !recieverState);

}

// the setup routine runs once when you press reset:
void setup() {                
  pinMode(IR_RECIEVER_INT_PIN, INPUT);     
  pinMode(IR_RECIEVER_PIN, INPUT); 
  
  pinMode(CONTOL_LED_PIN, OUTPUT); 
  pinMode(IR_LED_PIN, OUTPUT); 
  
  digitalWrite(CONTOL_LED_PIN, LOW);
  digitalWrite(IR_LED_PIN, LOW);

  noInterrupts();           // disable all interrupts
    
  attachInterrupt(IR_RECIEVER_INT, irRecieverInt, CHANGE);

  interrupts();

  Serial.begin(57600);
  irrecv.enableIRIn(); 
}

// the loop routine runs over and over again forever:
void loop() {
  if (irrecv.decode(&results)) {
      Serial.println(results.value, HEX);
      irrecv.resume(); // Receive the next value
  }
}
