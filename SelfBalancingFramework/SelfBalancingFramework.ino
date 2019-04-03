// LIBRARY CALLS

// Include libraries from .zip file:
//----------------------------------

#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//----------------------------------

#include <Wire.h>
#include "I2Cdev.h"
#include <utility/imumaths.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// GYROSCOPE VALUES
//----------------------------------------------------------------------------------

// Create new gyroscope instance
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Orientation/motion vars to be read from Gyroscope
imu::Quaternion q; // [w, x, y, z] quaternion container
imu::Vector<3> gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//----------------------------------------------------------------------------------

// PID VALUES:
// -------------------------------------------------------------------------------------------------------------------------------

// orignalSetPoint is angle at which we want the robot to balance at; angle of 0 is upside down; angle of 180 is upside up
double originalSetpoint = 180;

// setPoint variable used in case of sampling for setPoint at the first run of the code
double setPoint = originalSetpoint;

// movingAngleOffset used in case of slight angle drift of gyroscope
double movingAngleOffset = 0;

// variables used for input and output of PID
double input, output;

/*---------------------------------------------------------------------------------------------------------
We need to tune PID values manually: 
1. Make Kp, Ki and Kd equal to zero.

2. Adjust Kp:
	- too little Kp will make robot fall over, Because there's not enough correction.
	- too much Kp will make robot go back and forth wildly.
	- A good enough Kp will make the robot go slightly back and forth
----------------------------------------------------------------------------------------------------------*/
double Kp = 58;  // 58 

/*----------------------------------------------------------------------------------------------------------
3. Once Kp is set, adjust Kd.
	- A good Kd value will lessen the oscillations until the robot is steady.
	- Also, the right amount of Kd will keep the robot standing, even if pushed.
----------------------------------------------------------------------------------------------------------*/
double Kd = 2.5; // 2.5

/*----------------------------------------------------------------------------------------------------------
4. Set Ki. The robot will oscillate when turned on, even if Kp and Kd are set but will stabilize in time. 
	- The correct Ki value will shorten the time it takes for the robot to stabilize.
----------------------------------------------------------------------------------------------------------*/
double Ki = 60; // 60

// PID initialization
PID pid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

//---------------------------------------------------------------------------------------------------------------------------------

// MOTOR AND MOTOR DRIVER VALUES:
//-------------------------------------------------------------------------

// Pin definitions for motors; each pin must be a PWM pin on the Arduino
#define MOTOR_A_PIN_A 11
#define MOTOR_A_PIN_B 5
#define MOTOR_B_PIN_A 9
#define MOTOR_B_PIN_B 10

// Minimum speed at which motors consistently spin
#define MIN_ABS_SPEED 128

//Speed factors to match non-precise motor speeds to each other
double globalMotorSpeedFactor = 1;
double motorSpeedFactorLeft = 1 ;
double motorSpeedFactorRight = 1;

//-------------------------------------------------------------------------

// TESTING AND VERIFICATION VALUES:
//-----------------------------------------

bool testMode = false;
double angle;
#define threshold 0
bool firstSample =  true;

//-----------------------------------------

void setup()
{
  // join I2C bus 000000000000(I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  // BNO INITIALIZATION:
  //----------------------------------------------------------------------------------

  // Serial initialization and printing for testing
	Serial.begin(9600);
	Serial.println("Connecting Orientation Sensor"); Serial.println("");
	if(!bno.begin())
	{
	  // BNO not detected, check connection
	  Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}
	delay(1000);
	bno.setExtCrystalUse(true);

  //----------------------------------------------------------------------------------
  // PID INITIALIZATION:
  //-------------------------------------------------------------------------------------------

  // AUTOMATIC sets turns the PID on
  pid.SetMode(AUTOMATIC);
  // Sets the time period of the PID evaluation in milliseconds
  pid.SetSampleTime(10);
  // Sets the range of the PID output to be from -255 to 255 to drive the motors through PWM
  pid.SetOutputLimits(-255, 255); 

  //--------------------------------------------------------------------------------------------
}


void loop()
{
  // Loop won't run if BNO could not be initialized

  // Request quaternion data from BNO055
	q = bno.getQuat();
  // Request Gravity vector from BNO055
  gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  // SERIAL PRINT STATEMENTS FOR TESTING:
  //-----------------------------------------

  if (testMode) {
    Serial.print("QUATERNION = w: ");
    Serial.print(q.w());
    Serial.print(" x: ");
    Serial.print(q.x());
    Serial.print(" y: ");
    Serial.print(q.y());
    Serial.print(" z: ");
    Serial.print(q.z());
    Serial.println();
  
    Serial.print("GRAVITY = x: ");
    Serial.print(gravity[0]);
    Serial.print(" y: ");
    Serial.print(gravity[1]);
    Serial.print(" z: ");
    Serial.print(gravity[2]);
    Serial.println();
  }

  //-----------------------------------------

  // CALCULATING THE YAW, PITCH, AND ROLL:
  //-----------------------------------------------------------------------------------------

  // yaw: (about Z axis)
  ypr[0] = atan2(2*q.x()*q.y() - 2*q.w() *q.z(), 2*q.w() *q.w() + 2*q.x()*q.x() - 1);
  // pitch: (nose up/down, about Y axis)
  ypr[1] = atan2(gravity[0] , sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
  // roll: (tilt left/right, about X axis)
  ypr[2] = atan2(gravity[1] , gravity[2]);

  // SERIAL PRINT STATEMENTS FOR TESTING:
  //-----------------------------------------

  if (testMode) {
    Serial.print("ypr = YAW: ");
    Serial.print(ypr[0]);
    Serial.print(" PITCH: ");
    Serial.print(ypr[1]);
    Serial.print(" ROLL: ");
    Serial.print(ypr[2]);
    Serial.println();
  }

  //-----------------------------------------
    
  if(gravity[2]<0) {
    if(ypr[1]>0) {
      ypr[1] = PI - ypr[1]; 
    } else { 
      ypr[1] = -PI - ypr[1];
    }
  }

  //-------------------------------------------------------------------------------------------

  // Calculate input for PID in degrees
  input = ypr[1]* 180/M_PI + 180;

  // SERIAL PRINT STATEMENTS FOR TESTING:
  //-----------------------------------------

  if (testMode) {
    Serial.print("input to PID: ");
    Serial.print(input);
    Serial.println();
  }

  //-----------------------------------------

  // UNUSED SAMPLING CODE TO DETERMINE SETPOINT ON FIRST LOOP:
  //-----------------------------------------------------------

//if(firstSample == true){
//  setPoint = input;
//  firstSample = false;
//}

  //-----------------------------------------------------------

  // BALANCING LOOP USING PID AND MOTOR MOVEMENT
  //--------------------------------------------------------------------------------------------------------------------
  
  // no bno data - performing PID calculations and output to motors
  // computes the error difference between input and setPoint, and produce output calculation to minimize the error.
  pid.Compute();

  // SERIAL PRINT STATEMENTS FOR TESTING:
  //-----------------------------------------
  
  if (testMode) {
    Serial.print("output: ");
    Serial.print(output);
    Serial.println();
  }

  //-----------------------------------------

  // Move motors  with the speed of the output given by the PID to counteract the system's forces
  move(output);

  //--------------------------------------------------------------------------------------------------------------------

}

// Move function for motor driver
void move(int Speed)
{
  // Uses motor factor to scale output if needed; default is 1
  Speed = Speed * globalMotorSpeedFactor;

  // Output is given between -255 and 255,
  // If output is negative, take absolute value and reverse drive
  if (Speed < 0){
    Speed = -Speed;
    backward(Speed, MIN);
  }
  // Else if output is positive, forward drive
  else if(Speed > 0) {
    forward(Speed, MIN);
  }
  // Else if output is zero, robot is balanced, stop motors 
  else {
    stopMotor();
  }
}

// Move forward function for motor driver
void forward(int Speed, int MIN)
{
  // If speed is less than minimum speed, the motors will not move; so set speed to minimum speed
  if(Speed < MIN){
    Speed = MIN;
  }
  // Run motor A at speed
  analogWrite(MOTOR_A_PIN_A, 0);
  analogWrite(MOTOR_A_PIN_B, Speed);
  // Run motor B at speed
  analogWrite(MOTOR_B_PIN_A, Speed);
  analogWrite(MOTOR_B_PIN_B, 0);
}

// Move forward function for motor driver
void backward(int Speed, int MIN)
{
  // If speed is less than minimum speed, the motors will not move; so set speed to minimum speed
  if(Speed < MIN){
    Speed = MIN;
  }
  // Run motor A at speed
  analogWrite(MOTOR_A_PIN_B, 0);
  analogWrite(MOTOR_A_PIN_A, Speed);
  // Run motor B at speed
  analogWrite(MOTOR_B_PIN_B, Speed);
  analogWrite(MOTOR_B_PIN_A, 0);
}

// Stop function for motor driver
void stopMotor(void)
{
  // Set all motor pins to zero to stop all motors
  analogWrite(MOTOR_B_PIN_B, 0);
  analogWrite(MOTOR_B_PIN_A, 0);
  analogWrite(MOTOR_A_PIN_B, 0);
  analogWrite(MOTOR_A_PIN_A, 0);
}
