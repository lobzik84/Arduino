void setup() {
  Serial.begin(1200);

}

void loop() {
  delay(1000);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0xAA);
Serial.write(0xB9);
Serial.write(0x4D);
Serial.write(0x5B);
Serial.write(0x00);
Serial.write(0x05);
  delay(1000);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0xAA);
Serial.write(0xBA);
Serial.write(0x4D);
Serial.write(0x5B);
Serial.write(0x00);
Serial.write(0x06);
  delay(500);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0x00);
Serial.write(0xAA);
Serial.write(0xBA);
Serial.write(0x4D);
Serial.write(0x5B);
Serial.write(0x00);
Serial.write(0x06);

}
