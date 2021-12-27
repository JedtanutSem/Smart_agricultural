#include <SoftwareSerial.h>
int x_send;
//SoftwareSerial mySerial(2, 3); // RX, TX
int analogVal = 0;
int z_send = 0;
int x_plus = 0;
int z_plus = 0;
byte val2 = 0;
byte val3 = 0;
void setup() {
 //mySerial.begin(9600);
 Serial.begin(9600);
}
void loop() {
  analogVal = 256;
  val2++;
  val3++;
  int x_val = analogRead(1);
  int z_val = analogRead(0);
  
  if(x_val > 550)
  {
    x_send = map(x_val,550,1023,0,100);
    x_plus = 1;
  }
  else if(x_val <500)
  {
    x_send = map(x_val,500,0,0,100);
    x_plus = 0;
  }
  else 
  {
    x_send = 0;
    x_plus = 0;
  }
  
  if(z_val > 550)
  {
    z_send = map(z_val,550,1023,0,100);
    z_plus = 1;
  }
  else if(z_val <500)
  {
    z_send = map(z_val,500,0,0,100);
    z_plus = 0;
  }
  else
  {
    
    z_send = 0;
    z_plus = 0;
    
  }
  
  //Serial.print(x_send);
  //Serial.print("\t");
  //Serial.println(z_send);
  //x_send = 
 //int analogVal = analogRead(A0);
 //analogVal = 520;
 //val2 = 255;
 //val3 = 254;
 //Serial.print(analogVal);
 int checksum = 0;
 //checksum = 255+254;
 checksum += (byte) x_send;
 checksum += (byte) z_send;
 checksum += (byte) x_plus;
 checksum += (byte) z_plus;
 //Serial.print((byte) analogVal);
 //Serial.print(checksum);
 Serial.write(253); // start 
 Serial.write(255); // start 
 Serial.write(x_send);
 Serial.write(z_send);
 //Serial.print(analogVal);
 Serial.write(x_plus);
 Serial.write(z_plus);
 //mySerial.write(analogVal); // lower byte
 //mySerial.write(analogVal >> 8); // upper byte
 Serial.write(checksum);
 Serial.write(checksum>>8);//checksum'
 //Serial.print(checksum);
 //Serial.write(checksum>>16);//checksum
 //Serial.write(checksum>>24);//checksum
 //Serial.print(checksum);
 delay(100);
}
