

void setup()
{
  Serial.begin(9600); // Initialize serial
  delay(2000); // Delay for 2 seconds.
  
  Serial.write(85); // Dummy byte for auto baudrate.
  delay(3000); // Delay for 3 seconds.
}

void loop()
{
  // Controlling left motor, direction CW
  Serial.write(85); // 1st byte: Header.
  Serial.write((byte)0); // 2nd byte: Channel = left & Address = 0.
  Serial.write(63); // 3rd byte: Speed = half & Direction = CW.
  Serial.write(85 + 63); // 4th byte: Checksum = 1st byte + 2nd byte + 3rd byte.
  Serial.write(0);
  Serial.write(0);
  Serial.write(0);
  delay(2000);
 
}
