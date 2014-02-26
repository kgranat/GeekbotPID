#include <PID_v1.h>
#include <Commander.h>

#include <Servo.h> 

 
//include the I2C Wire library - needed for communication with the I2C chip attached to the LCD manual 
#include <Wire.h> 
// include the RobotGeekLCD library
#include <RobotGeekLCD.h>



// Encoder Pin Positions.
// Right IR Sensor
#define RightEncoderPin 2

// Left IR Sensor
#define LeftEncoderPin  3

//Left Servo Motor
#define PWM1            9                       // PWM motor pin

//Right Servo Motor
#define PWM2            10                       // PWM motor pin



#define RIGHT_SERVO_PIN 10
#define LEFT_SERVO_PIN 9

double L_Setpoint, L_Input, L_Output, R_Setpoint, R_Input, R_Output;

PID L_PID(&L_Input, &L_Output, &L_Setpoint,.4,1,.05, DIRECT);
PID R_PID(&R_Input, &R_Output, &R_Setpoint,.4,1,.05, DIRECT);
 
// Loop time for calculating velocity
#define LOOPTIME        100                     // PID loop time
 
// Define physical constants of the wheel for wheel encoding
#define WHEEL_CIRCUMFERENCE 0.283 // 90.23mm d wheel x Pi = .283m circumference wheel
#define WHEEL_TICKS 32         // The number of 'ticks' for a full wheel cycle
#define WHEEL_DIST .183           // The distance between wheels in meters
 
// Variables for storing the calculated velocity in m/s
double leftvelocity;
double rightvelocity;


//Variables for storing RPM
double L_RPM;
double R_RPM;

int L_Temp;
int R_Temp;

// Variables for storing position info
long OldLPos;
long NewLPos;
long OldRPos;
long NewRPos;

unsigned long lastMilli = 0;                    // loop timing
unsigned long lastMilliPrint = 0;               // loop timing

int rightSpeed = 1500;
int leftSpeed = 1500;


  Servo servoLeft;
  Servo servoRight;

 
// Volatile variables that can be changed inside of interrupt functions
volatile unsigned long RightEncoderPos = 0;
volatile unsigned long LeftEncoderPos = 0;

Commander command = Commander();

double SpeedDial;
 
 //right debouncing
long previousMillisr = 0;        // will store last time LED was 
long intervalr = 15;           // interval at which to blink (milliseconds)

//left debouncing
long previousMillisl = 0;        // will store last time LED was 
long intervall = 15;           // interval at which to blink (milliseconds)

//lcd update variables
long previousMillisLCD = 0;        // will store last time LED was 
long intervalLCD = 1000;           // interval at which to blink (milliseconds)


RobotGeekLCD lcd;


void setup()
{
  //pinMode(6,OUTPUT);  
  Serial.begin(38400);
  delay(1000);
  pinMode(LeftEncoderPin, INPUT);

  pinMode(RightEncoderPin, INPUT);

  
  //digitalWrite(LeftEncoderPinA, HIGH);

  //digitalWrite(RightEncoderPinA, HIGH);
  

  attachInterrupt(0, RightEncoderEvent, CHANGE);
  attachInterrupt(1, LeftEncoderEvent, CHANGE);

//// hbridge IO 
//  pinMode(InA1, OUTPUT);
//  pinMode(InB1, OUTPUT);
//  pinMode(PWM1, OUTPUT);
//  analogWrite(PWM1, 0);
//  pinMode(InA3, OUTPUT);
//  pinMode(InB4, OUTPUT);
//  pinMode(PWM2, OUTPUT);
  //analogWrite(PWM1, 0);
  //analogWrite(PWM2, 0);
//  digitalWrite(InA1, HIGH);
//  digitalWrite(InB1, LOW);
//  digitalWrite(InA3, HIGH);
//  digitalWrite(InB4, LOW);
//  
  L_PID.SetMode(AUTOMATIC);
  R_PID.SetMode(AUTOMATIC);
  
  


  servoLeft.attach(LEFT_SERVO_PIN); 
  servoRight.attach(RIGHT_SERVO_PIN);

  servoLeft.writeMicroseconds(leftSpeed);
  servoRight.writeMicroseconds(rightSpeed);
  
  
 L_PID.SetOutputLimits(0,200);
 R_PID.SetOutputLimits(0,200);
 
   lcd.init();

    lcd.print("");

}
 
void loop(){
  
if(command.ReadMsgs() > 0){

 if((command.walkV) > 10){        
    L_Temp = (4*(command.walkV));
    R_Temp = (4*(command.walkV));
    if((command.lookH) > 10){
      L_Temp = L_Temp + (4*(command.lookH));
    }
    if((command.lookH) < -10){
      R_Temp = R_Temp - (4*(command.lookH));
    }
        
}
else{
  L_Temp = 0;
  R_Temp = 0;
}

    L_Setpoint = L_Temp;
    R_Setpoint = R_Temp;
    GetSpeeds();
    L_PID.Compute();
    R_PID.Compute();
    LmotorForward(L_Output);
    RmotorForward(R_Output);  
  
  
}

//LCD Update
  unsigned long currentMillisLCD = millis();
  if(currentMillisLCD - previousMillisLCD > intervalLCD) 
  {
    // save the last time you blinked the LED 
    previousMillisLCD = currentMillisLCD;   
    lcd.setCursor(0, 0);
    lcd.print("R:");
    lcd.print(RightEncoderPos);   
    lcd.setCursor(0, 1);
    lcd.print("L:");
    lcd.print(LeftEncoderPos);

  }


//  getParam();       

  // Calculate current speeds

//  printMotorInfo();
  
}
 
// Return the real-world vehicle speed
void GetSpeeds()
{
  if((millis()-lastMilli) >= LOOPTIME)   {                                          // enter timed loop
    lastMilli = millis();
    
        // Find the old and new encoder positions
	NewLPos = LeftEncoderPos;
        NewRPos = RightEncoderPos;
        L_RPM = ((NewLPos - OldLPos)*(60*(1000/LOOPTIME)))/(WHEEL_TICKS);                  // 13 pulses X 40 gear ratio = 520 counts per output shaft rev
        R_RPM = ((NewRPos - OldRPos)*(60*(1000/LOOPTIME)))/(WHEEL_TICKS);                  // 13 pulses X 40 gear ratio = 520 counts per output shaft rev
 	OldLPos = NewLPos;
        OldRPos = NewRPos;

        L_Input = L_RPM;
        R_Input = R_RPM;
        	// Convert between angular velocity to velocity
	double LangVel = (L_RPM/60);
	leftvelocity = (LangVel * WHEEL_CIRCUMFERENCE);
 
        double RangVel = (R_RPM/60);
	rightvelocity = (RangVel * WHEEL_CIRCUMFERENCE);

  }
} 
 


//void LeftEncoderEvent()  {                                    // pulse and direction, direct port reading to save cycles
//  if (PINB & 0b00000001)    LeftEncoderPos++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
//  else                      LeftEncoderPos++;                // if (digitalRead(encodPinB1)==LOW)   count --;
//}
//
//void RightEncoderEvent()  {                                    // pulse and direction, direct port reading to save cycles
//  if (PINA & 0b00000001)    RightEncoderPos++;                // if(digitalRead(encodPinB1)==HIGH)   count ++;
//  else                      RightEncoderPos++;                // if (digitalRead(encodPinB1)==LOW)   count --;
//}
 
// Encoder event for the interrupt call
void LeftEncoderEvent()
{
   unsigned long currentMillisl = millis();
 
  if(currentMillisl - previousMillisl > intervall) 
  {
    // save the last time you blinked the LED 
    previousMillisl = currentMillisl;   
    LeftEncoderPos++;

  }
  
}
 
void RightEncoderEvent()
{
       unsigned long currentMillisr = millis();
 
  if(currentMillisr - previousMillisr > intervalr) 
  {
    // save the last time you blinked the LED 
    previousMillisr = currentMillisr;   
    RightEncoderPos++;

  }
  
  
  
}
 


 void RmotorForward(int PWM_val)  {
 
  servoRight.writeMicroseconds(1500 - PWM_val);
  
  
  
  }
  
 void LmotorForward(int PWM_val)  {
  servoLeft.writeMicroseconds(1500 + PWM_val); 
  }  
  
// void LmotorOdometry(int spd, int distance)  {
//  while(LeftEncoderPos < distance){
//  L_Setpoint = spd;
//  digitalWrite(InA1, LOW);
//  digitalWrite(InB1, HIGH);  
//  }
//  }


void LmotorBackward(int PWM_val)  {
  servoLeft.write(90 - PWM_val); 
  
}


void LmotorStop()  {
  servoLeft.write(90); 
  
}

// Print a double value onto the serial stream
// This is from the Arduino.cc forum
void PrintDouble( double val, byte precision)
{
  Serial.print (int(val));  //prints the int part
  if( precision > 0) {
    Serial.print("."); // print the decimal point
    unsigned long frac;
    unsigned long mult = 1;
    byte padding = precision -1;
    while(precision--)
	 mult *=10;
 
    if(val >= 0)
	frac = (val - int(val)) * mult;
    else
	frac = (int(val)- val ) * mult;
    unsigned long frac1 = frac;
    while( frac1 /= 10 )
	padding--;
    while(  padding--)
	Serial.print("0");
    Serial.println(frac,DEC) ;
  }
}


void printMotorInfo()  {                                                      // display data
  if((millis()-lastMilliPrint) >= 500)   {                    
    lastMilliPrint = millis();
//    Serial.print("Left Velocity: ");
//    PrintDouble(leftvelocity, 4);
    Serial.print("L_SP:");             Serial.print(L_Setpoint);  
    Serial.print("  L_RPM:");          Serial.print(L_Input);
    Serial.print("  L_PWM:");          Serial.print(L_Output);      
    Serial.print("  L_Count:");        Serial.println(LeftEncoderPos);    
    
    Serial.print("R_SP:");             Serial.print(R_Setpoint);  
    Serial.print("  R_RPM:");          Serial.print(R_Input);
    Serial.print("  R_PWM:");          Serial.print(R_Output);      
    Serial.print("  R_Count:");        Serial.println(RightEncoderPos);        
  }
}




//###################################################################################
//SERIAL COMMANDS
//
//int getParam()  {                                     // 
//char param, cmd;
//  if(!Serial.available())    return 0;
//  delay(10);                  
//  param = Serial.read();                              // get parameter byte
//  if(!Serial.available())    return 0;
//  cmd = Serial.read();                                // get command byte
//  Serial.flush();
//  switch (param) {
//    case 'v':                                         // adjust speed
//      if(cmd=='+')  {
//        L_Setpoint += 10;
//        if(L_Setpoint>200)   L_Setpoint=200;
//      }
//      if(cmd=='-')    {
//        L_Setpoint -= 10;
//        if(L_Setpoint<0)   L_Setpoint=0;
//      }
//      break;
//    case 'd':                                        // adjust direction
//      if(cmd=='+'){
//        digitalWrite(InA1, LOW);
//        digitalWrite(InB1, HIGH);
//      }
//      if(cmd=='-')   {
//        digitalWrite(InA1, HIGH);
//        digitalWrite(InB1, LOW);
//      }
//      break;
//    case 's':                                        // user should type "oo"
//      digitalWrite(InA1, LOW);
//      digitalWrite(InB1, LOW);
//      L_Setpoint = 0;
//      break;
//    default:
//      Serial.println("???");
//    }
//}
