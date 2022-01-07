int x_anaPin = 0;
int y_anaPin = 1;
int x_val;
int z_val;
int x_send;
int z_send;


void setup()
{
  Serial.begin(9600);
}

void loop()
{
  z_val = analogRead(x_anaPin);
  x_val = analogRead(y_anaPin);
  if(x_val < 500)
  {
    x_send = map(x_val,520,0,0,100);
    //x_plus = 1;
  }
  else if(x_val>500)
  {
    x_send = map(x_val,520,1023,0,-100);
    //x_plus = 0;
  }
  else
  {
    x_send = 0;
   // x_plus = 0;
  }

  if(z_val > 550)
  {
    z_send = map(z_val,550,1023,0,-100);
    //z_plus = 1;
  }
  else if(z_val <500)
  {
    z_send = map(z_val,520,0,0,100);
    //z_plus = 0;
  }
  else
  {

    z_send = 0;
    //z_plus = 0;

  }
  Serial.print(x_send);
  Serial.print(",");
  Serial.print(z_send);
  //Serial.print(",");
  //Serial.print(x_val);
  //Serial.print(",");
  //Serial.print(z_val);
  Serial.println("");
  delay(10);

}
