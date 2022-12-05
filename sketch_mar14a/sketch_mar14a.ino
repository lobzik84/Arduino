#define Pulse 9

#define Dir 8

int period = 1200;

long delay_Micros =1000; // Set value

long currentMicros = 0; long previousMicros = 0;
long lastMillis = 0;

void setup()

{

pinMode(Pulse,OUTPUT);

pinMode(Dir,OUTPUT);

digitalWrite(Dir, HIGH);
Serial.begin(115200);
}

void loop()

{


if (millis() - lastMillis > period) {
  if (digitalRead (Dir)) {
   digitalWrite(Dir, LOW);
  } else {
    digitalWrite(Dir, HIGH);
  }
  lastMillis = millis();
}

float a  = ((float)(millis() - lastMillis) / (float) period) * 3.14;
//delay_Micros = 2000 - (1400 *sin (a));
//Serial.println(delay_Micros);

/*if ((millis() - lastMillis) > 20 && (millis() - lastMillis < 480)) {
  delay_Micros = 600;
} else {
  delay_Micros = 1600;
}*/

currentMicros = micros();

if(currentMicros - previousMicros >= delay_Micros)

{

previousMicros = currentMicros;

digitalWrite(Pulse,HIGH);

delayMicroseconds(100); //Set Value

digitalWrite(Pulse,LOW);

} }
