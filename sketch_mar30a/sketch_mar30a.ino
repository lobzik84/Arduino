#define Pulse 9

#define Dir 8

long delay_Micros =1800; // Set value

long currentMicros = 0; long previousMicros = 0;

void setup()

{

pinMode(Pulse,OUTPUT);

pinMode(Dir,OUTPUT);

digitalWrite(Dir,LOW);

}

void loop()

{

currentMicros = micros();

if(currentMicros - previousMicros >= delay_Micros)

{

previousMicros = currentMicros;

digitalWrite(Pulse,HIGH);

delayMicroseconds(500); //Set Value

digitalWrite(Pulse,LOW);

} }
