#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define M1A_PIN 5
#define M1B_PIN 6
#define M2A_PIN 10
#define M2B_PIN 11

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
String speed_left_str ;
String speed_right_str;
String read_enter ;
String max_pwm_str;
float speed_left_float;
float speed_right_float;
int speed_M1A = 0;
int speed_M1B = 0;
int speed_M2A = 0;
int speed_M2B = 0;
float max_pwm_int;



void setup()
{
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  Serial.begin(115200);
  pinMode(M1A_PIN,OUTPUT);
  pinMode(M2A_PIN,OUTPUT);
  pinMode(M1B_PIN,OUTPUT);
  pinMode(M2B_PIN,OUTPUT);
}

void loop()
{
  // when characters arrive over the serial port...
  if (Serial.available()) {
    if (Serial.available() > 0) {
    
      
      
      speed_left_str = Serial.readStringUntil(',');
      speed_left_float = speed_left_str.toFloat();
      Serial.read();
      speed_right_str = Serial.readStringUntil(',');
      speed_right_float = speed_right_str.toFloat();
      Serial.read();
      max_pwm_str = Serial.readStringUntil('\n');
      max_pwm_int = max_pwm_str.toFloat();
     
      

    }
  }
 
  if (speed_left_float >= 0)
  {
     speed_M2A = max_pwm_int*speed_left_float;
     speed_M2B = 0;
     analogWrite(M2A_PIN,speed_M2A);
     analogWrite(M2B_PIN,speed_M2B);
      
  }
  else if(speed_left_float < 0)
  {
    speed_M2B = abs(max_pwm_int*speed_left_float);
    speed_M2A = 0;
    analogWrite(M2A_PIN,speed_M2A);
    analogWrite(M2B_PIN,speed_M2B);
  }
  
  if(speed_right_float >= 0)
  {
     speed_M1A = max_pwm_int*speed_right_float; 
     speed_M1B = 0;
     analogWrite(M1A_PIN,speed_M1A);
     analogWrite(M1B_PIN,speed_M1B);
  }
  else if(speed_right_float < 0)
  {
    speed_M1B = abs(max_pwm_int*speed_right_float);
    speed_M1A = 0;
    analogWrite(M1A_PIN,speed_M1A);
    analogWrite(M1B_PIN,speed_M1B);
  }

  
  
  
  
  Serial.print("vel_left:");
  Serial.print(speed_left_float);
  Serial.print(",");
  Serial.print("vel_right:");
  Serial.print(speed_right_float);
  Serial.print(","); 
  Serial.print("pwm:");
  Serial.print(max_pwm_int);
  Serial.println("");
  
}
