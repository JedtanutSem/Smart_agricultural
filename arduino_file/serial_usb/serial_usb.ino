

void setup() {
 Serial.begin(9600);
}
void loop() {
 
 Serial.write(0x53); // start 
 delay(100);
 Serial.write(0x54); // start 

 delay(100);
}
