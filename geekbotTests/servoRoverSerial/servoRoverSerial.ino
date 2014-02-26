
#define LEFT_SERVO_PIN 9
#define RIGHT_SERVO_PIN 10





   
   


#include <Servo.h>

void setup()
{
  //initilaize left and right servo objects
  Servo servoLeft;
  Servo servoRight;

  //Attach the servo pins to the servo objects
  servoLeft.attach(LEFT_SERVO_PIN); 
  servoRight.attach(RIGHT_SERVO_PIN);

  //Set the servos to 'stopped'
  servoLeft.write(90);
  servoRight.write(90);
  
    //open serial port
   Serial.begin(9600);
   delay (500);   
    Serial.println("###########################");    
   Serial.println("Serial Communication Established.");  
   
   int servoToSet = 0;
}

void loop()
{
  if (Serial.available() > 0) 
  {
    int inByte = Serial.read();
    
    
    
    switch (inByte) 
    {
      case 'l':    
        servoToSet = LEFT_SERVO_PIN;
        break;
  
      case 'r':    
        servoToSet = RIGHT_SERVO_PIN;
        break;  
    }
    
    
    while(servoToSet != 0)
    {
      if (Serial.available() > 0) 
      {
        int numChars = 
        int inByte = Serial.read();
      
      }
  
      
      
    }
  }
  
  // read the sensor:
  
    int inByte = Serial.read();

    switch (inByte) {

    case '1':    
      ScanServo();
      break;

    case '2':    
      MoveCenter();
      break;
      
     case '3':    
      RelaxServos();
      break;     

    case '4':    
      LeftLegTest();
      break;
      
    case '5':    
      RightLegTest();
      break;
      
    case '6':    
      CheckVoltage();
      break;
      
    case '7':
      LEDTest();
      break;
      

    } 
  
}



void setServoSpeed(int servoPin)
{
  if (Serial.available() > 0) 
  {
    int inByte = Serial.read();
  
  }
  
}
