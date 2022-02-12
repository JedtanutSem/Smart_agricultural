String incoming_data;
String pwm_left_string;
String direction_left_string;
String pwm_right_string;
String direction_right_string;

int pwm_left = 0;
int direction_left = 0;
int pwm_right = 0;
int direction_right = 0;

int motor_left_pin = 7;
int motor_right_pin = 8;

int pwm_left_pin = 5;
int pwm_right_pin = 6;

int direction_left_pin = LOW ;
int direction_right_pin = LOW ;


void setup() {
  Serial.begin(9600);
  pinMode(motor_left_pin,OUTPUT);
  pinMode(motor_right_pin,OUTPUT);
  pinMode(pwm_left_pin,OUTPUT);
  pinMode(pwm_right_pin,OUTPUT);
}

void loop() {
  while(Serial.available()) {
    //incoming_data = Serial.readString();
    pwm_left_string = Serial.readStringUntil(' ,');
    direction_left_string = Serial.readStringUntil(' ,');
    pwm_right_string = Serial.readStringUntil(' ,');
    direction_right_string = Serial.readStringUntil(' \n');

    pwm_left = pwm_left_string.toInt();
    direction_left = direction_left_string.toInt();
    pwm_right = pwm_right_string.toInt();
    direction_right = direction_right_string.toInt();

    if(direction_left == 0){
      direction_left_pin = HIGH ;
    }
    else if(direction_left == 1){
      direction_left_pin = LOW ;
    }

    if(direction_right == 1){
      direction_right_pin = HIGH;
    }

    else if(direction_right == 0){
      direction_right_pin = LOW;
    }

    if (pwm_left == 0){
      direction_left_pin = LOW ;
    }

    if(pwm_right == 0){
      direction_right_pin = LOW;
    }

    analogWrite(pwm_left_pin, pwm_left); //ENA pin
    analogWrite(pwm_right_pin, pwm_right); //ENB pin

    digitalWrite(motor_left_pin, direction_left_pin);
    digitalWrite(motor_right_pin, direction_right_pin);

    //Serial.println(incoming_data);
    
    Serial.print(pwm_left);
    Serial.print(":");
    Serial.print(direction_left);
    Serial.print(":");
    Serial.print(pwm_right);
    Serial.print(":");
    Serial.println(direction_right);
    
    
  }

}
