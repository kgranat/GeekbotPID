
/***********************************************************************************
 *    _                     RobotGeek Geekbot Rover
 *  _|_|______________      Tick Counter
 *   |___         |
 *  _/ . \ _______|_
 *   \___/      \_/
 *
 *  The following sketch will assist you in finsing which PWM pulses correspond 
 *  to which speeds for the two RobotGeek Continuous Servros. This will help you
 *  to drive the servo at the correct orientation
 *    
 *  Wiring
 *    Right Wheel IR Sensor - Digital Pin 2
 *    Left Wheel IR Sensor - Digital Pin 3
 *    Right Servo - Digital Pin 10
 *    Left Servo - Digital Pin 11
 *    Jumper for pins 9/10/11 should be set to 'VIN'
 *    Jumper for pins 3/5/6 should be set to '5V'
 *
 *  Control Behavior:
 *
 *  External Resources
 *
 ***********************************************************************************/
//Includes
#include <Servo.h> 


//include the I2C Wire library - needed for communication with the I2C chip attached to the LCD manual 
#include <Wire.h> 
// include the RobotGeekLCD library
#include <RobotGeekLCD.h>



//Defines

#define RIGHT_SERVO_PIN 10
#define LEFT_SERVO_PIN 9


#include <Servo.h>

  Servo servoLeft;
  Servo servoRight;

  int leftTicks;
  int rightTicks;
  long rightTime;
  long rightCycle =0;
  boolean firstRightTick = false;
boolean ledState = false;
long previousMillis = 0;        // will store last time LED was updated

int rightSpeed = 90;
int rightIncrease = 5;

RobotGeekLCD lcd;



void setup()
{


  // initlaize the lcd object - this sets up all the variables and IIC setup for the LCD object to work
  lcd.init();
  // Print a message to the LCD.
  lcd.print("0");

 /* servoLeft.attach(LEFT_SERVO_PIN); 
  servoRight.attach(RIGHT_SERVO_PIN);

  servoLeft.write(rightSpeed);
  servoRight.write(rightSpeed);
  */
  
  attachInterrupt(1, rightChange, CHANGE);
  //attachInterrupt(0, leftChange, CHANGE);
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
 //   Serial.begin(9600); 

}

void loop()
{
  
  // if(rightTicks >= 9)
  // {
    
  // Serial.print(rightSpeed);  
  // Serial.print(" cycle = " );                       
  // Serial.println(rightCycle);  
  // rightTicks = 0;
  
  // if(rightSpeed+rightIncrease <180)
  // {
  //   rightSpeed = rightSpeed+10;
  //   servoRight.write(rightSpeed);
  // }
  // else
  // {
  //   rightSpeed = 0;
  // }
  
  // }
  
  //two cycles of the wheel hwlps immensly - with one cycle the program has a lot of bad data (there is still potential for bad data, but not nearlt as much)
  /*else if(rightTicks > 9)
  {
    //error
    rightTicks = 0;
    firstRightTick = false;
  Serial.println("error missed " );    
  }*/
  
  
}

void rightChange()
{
  

 rightTicks++; 
 // Serial.println(rightTicks);  
  
 //lcd.print(rightTicks);
  

}
void leftChange()
{

 leftTicks++; 

 if(leftTicks == 1)
 {
   // rightTime = millis();
 } 
 
 if(leftTicks == 8)
 {

  servoLeft.write(90);

 }
}
