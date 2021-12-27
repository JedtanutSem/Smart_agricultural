void setup() {
 Serial.begin(9600);
}
void loop() {
 int analogVal = analogRead(A0);
 byte checksum = 0;
 checksum += (byte) analogVal;
 checksum += (byte) (analogVal >> 8);
 Serial.write(253); // start 
 Serial.write(255); // start 
 Serial.write(analogVal); // lower byte
 Serial.write(analogVal >> 8); // upper byte
 Serial.write(checksum); //checksum
 delay(250);
}
