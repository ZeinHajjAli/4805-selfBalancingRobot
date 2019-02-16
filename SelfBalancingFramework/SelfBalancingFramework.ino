#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include <Wire.h>
#include "PID_v1.h"
//TODO1: look into making this custom
//#include <LMotorController.h>
#include "I2Cdev.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//TODO2: Change value
#define MIN_ABS_SPEED 20

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
//TODO3: set up interrupt from bno
uint8_t bnoIntStatus; // holds actual interrupt status byte from bno
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 173;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//TODO2: find values for variables below
/* ------------- Hussain --------------------
We need to tune PID values manually: ( we could use the autoTuner for PID but may not be accurate )
1. Make Kp, Ki and Kd equal to zero.
2. Adjust Kp:
	- too little Kp will make robot fall over, Because there's not enough correction.
	- too much Kp will make robot go back and forth wildly.
	- A good enough Kp will make the robot go slightly back and forth
  
3. Once Kp is set, adjust Kd.
	- A good Kd value will lessen the oscillations until the robot is steady.
	- Also, the right amount of Kd will keep the robot standing, even if pushed.
  
4. Set Ki. The robot will oscillate when turned on, even if Kp and Kd are set but will stabilize in time. 
	- The correct Ki value will shorten the time it takes for the robot to stabilize.
---------------------------------------------*/
double Kp = 50;   
double Kd = 1.4;
double Ki = 60;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.6;
double motorSpeedFactorRight = 0.5;
//end of custom values


/*
//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);
*/

volatile bool bnoInterrupt = false; // indicates whether bno interrupt pin has gone high

void dmpDataReady()
{
  bnoInterrupt = true;
}


void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

//TODO4: research bno intialization
/*
  //Initialize bno
  mpu.initialize();
  
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

*/

//TODO5: find bno replacment for DMP
/*
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255); 
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
 */
}


void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  
  // wait for bno interrupt or extra packet(s) available
  while (!bnoInterrupt && fifoCount < packetSize)
  {
    //no bno data - performing PID calculations and output to motors 
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
  
  }
  
  // reset interrupt flag and get INT_STATUS byte
  bnoInterrupt = false;
  bnoIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((bnoIntStatus & 0x10) || fifoCount == 1024)
  {
    //TODO6: find alternative to mpu.resetFIFO();
    /*
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    */
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (bnoIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    //TODO6
    /*
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    */
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    //TODO7: find functions for bno
    /*
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180/M_PI + 180;
    */
  }
}
