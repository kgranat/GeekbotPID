/***********************************************************************************
 *    _                     RobotGeek Geekbot Rover
 *  _|_|______________      RPM Meter
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
 *    Right Servo - Digital Pin 11
 *    Left Servo - Digital Pin 10
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

//Defines

#define RIGHT_SERVO_PIN 10
#define LEFT_SERVO_PIN 9


// Loop time for calculating velocity
#define LOOPTIME        500                     // PID loop time
 
// Define physical constants of the wheel for wheel encoding
#define WHEEL_CIRCUMFERENCE 0.283 // 90.23mm d wheel x Pi = .283m circumference wheel
#define WHEEL_TICKS 32.0         // The number of 'ticks' for a full wheel cycle
#define WHEEL_DIST .183           // The distance between wheels in meters
 
 int meters = 0;
 
long previousMillisi = 0;        // will store last time LED was 
long intervali = 15;           // interval at which to blink (milliseconds)

long previousMillisil = 0;        // will store last time LED was 
long intervalil = 15;           // interval at which to blink (milliseconds)



double L_Setpoint, L_Input, L_Output, R_Setpoint, R_Input, R_Output;



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

int rightSpeed = 107;
int leftSpeed = 90;



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


int rightIncrease = 1;

 int GetSpeedCall = 0;
// Volatile variables that can be changed inside of interrupt functions
volatile unsigned long RightEncoderPos = 0;
volatile unsigned long LeftEncoderPos = 0;

         // The distance between wheels in meters
 
// Variables for storing the calculated velocity in m/s
double leftvelocity;
double rightvelocity;

void setup()
{

  pinMode(13,OUTPUT);
  pinMode(2,INPUT);
  pinMode(3,INPUT);
  
  

  servoLeft.attach(LEFT_SERVO_PIN); 
  servoRight.attach(RIGHT_SERVO_PIN);

  servoLeft.write(leftSpeed);
  servoRight.write(rightSpeed);
  
  
  attachInterrupt(0, rightChange, CHANGE);
  attachInterrupt(1, leftChange, CHANGE);
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
    Serial.begin(9600); 

}

void loop()
{
  /*
  if(meters !=0)
  {
    for(int x = 0; x< meters; x++)
    {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
   meters = 0;
    
  }*/
  
  
  


    servoRight.write(rightSpeed);
    GetSpeeds();
//printMotorInfo();
  
}
void GetSpeeds()
{  
  if((millis()-lastMilli) >= LOOPTIME)   {                                          // enter timed loop
    lastMilli = millis();
    
  GetSpeedCall = GetSpeedCall+1;
        // Find the old and new encoder positions
	NewLPos = LeftEncoderPos;
        NewRPos = RightEncoderPos;
       // L_RPM = ((NewLPos - OldLPos)*(60*(1000/LOOPTIME)))/(WHEEL_TICKS);                  // 13 pulses X 40 gear ratio = 520 counts per output shaft rev
        //R_RPM = (NewRPos - OldRPos)*((60*(1000/LOOPTIME)))/(WHEEL_TICKS);                  // 13 pulses X 40 gear ratio = 520 counts per output shaft rev
        R_RPM = ((NewRPos - OldRPos)/WHEEL_TICKS) * (60.0*(1000.0/LOOPTIME));
        L_RPM = ((NewLPos - OldLPos)/WHEEL_TICKS) * (60.0*(1000.0/LOOPTIME));
        
        //R_RPM = ((NewRPos - OldRPos)/(WHEEL_TICKS))/(LOOPTIME/(60.0*1000.0));
        
            Serial.print("R");       // Serial.print(RightEncoderPos);        
            Serial.print(",");          Serial.print(rightSpeed);      
            Serial.print(",");          Serial.println(R_RPM);
            
            
//            Serial.print(",");          Serial.print(NewRPos - OldRPos);
//            Serial.print(", L,");        Serial.print(LeftEncoderPos);        
//            Serial.print(",");          Serial.print(L_RPM);
//            Serial.print(",");          Serial.println(NewLPos - OldLPos);
           // Serial.print(",");          Serial.println(60.0*(1000.0/LOOPTIME));
            //Serial.print(",");          Serial.println((NewRPos - OldRPos)/WHEEL_TICKS);
            
            
 	OldLPos = NewLPos;
        OldRPos = NewRPos;

        L_Input = L_RPM;
        R_Input = R_RPM;   
       
            //Serial.print("  R_RPM:");          Serial.print(R_Input);

        	// Convert between angular velocity to velocity
	double LangVel = (L_RPM/60);
	leftvelocity = (LangVel * WHEEL_CIRCUMFERENCE);
 
        double RangVel = (R_RPM/60);
	rightvelocity = (RangVel * WHEEL_CIRCUMFERENCE);

  }
} 
 
 
void rightChange()
{

     // digitalWrite(13,HIGH-digitalRead(13));

      
      
      
       unsigned long currentMillisi = millis();
 
  if(currentMillisi - previousMillisi > intervali) 
  {
    // save the last time you blinked the LED 
    previousMillisi = currentMillisi;   
    RightEncoderPos++;

  }
  
  if(RightEncoderPos%113 ==0)
  {
      meters = RightEncoderPos/113;
  }
    
//RightEncoderPos++;

  
  
}
void leftChange()
{
      //LeftEncoderPos++;
   //   digitalWrite(13,HIGH-digitalRead(13));

      
      
      
       unsigned long currentMillisil = millis();
 
  if(currentMillisil - previousMillisil > intervalil) 
  {
    // save the last time you blinked the LED 
    previousMillisil = currentMillisil;   
    LeftEncoderPos++;

  }

}


void printMotorInfo()  {                                                      // display data
  if((millis()-lastMilliPrint) >= 500)   {                    
    lastMilliPrint = millis();
//    Serial.print("Left Velocity: ");
//    PrintDouble(leftvelocity, 4);
//    Serial.print("L_SP:");             Serial.print(L_Setpoint);  
//    Serial.print("  L_RPM:");          Serial.print(L_Input);
//    Serial.print("  L_PWM:");          Serial.print(L_Output);      
//    Serial.print("  L_Count:");        Serial.println(LeftEncoderPos);    
//    
//    Serial.print("R_SP:");             Serial.print(R_Setpoint);  
    Serial.print("  R_RPM:");          Serial.print(R_Input);
//    Serial.print("  R_PWM:");          Serial.print(R_Output);      
    Serial.print("  R_Count:");        Serial.println(RightEncoderPos);        
  }
}

