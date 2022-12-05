

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); 
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  int val2 = analogRead(2);
  int val3 = analogRead(3);
  boolean bool2 = !digitalRead(2);
  boolean bool3 = !digitalRead(3);
  
  Serial.print( val2 );
  Serial.print(",");
  Serial.print(val3);
  Serial.print(",");
  Serial.print(bool2?"1":"0");
  Serial.print(",");
  Serial.print(bool3?"1":"0");
  Serial.println("");
}
