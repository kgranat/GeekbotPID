



#include <Servo.h>

void setup()
{
  Servo servoLeft;
  Servo servoRight;

  servoLeft.attach(9); 
  servoRight.attach(10);

  servoLeft.write(100);
  servoRight.write(80);
}

void loop()
{
  
}
