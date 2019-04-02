#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>

//TODO1: look into making this custom
//#include <LMotorController.h>
#include "I2Cdev.h"
#include <utility/imumaths.h> // from Adafruit_BNO055 Library

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//TODO2: Change value
#define MIN_ABS_SPEED 128

Adafruit_BNO055 bno = Adafruit_BNO055(55);
/*
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error) (Used just after dmp Initialization)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
*/

// orientation/motion vars
imu::Quaternion q; // [w, x, y, z] quaternion container
imu::Vector<3> gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 180;    /*-----------
				OriginalSetPoint is the angle which we want the robot to stay at*/
double setpoint = originalSetpoint;
double movingAngleOffset = 0;
double input, output;

//TODO2: find values for variables below
/* ------------------------------------------
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
double Kp = 58;  // 58 
double Kd = 2; // 2.5
double Ki = 60; // 
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactor =1 ;
double motorSpeedFactorRight = 0.5;



#define MOTORA_PINA 11
#define MOTORA_PINB 5
#define MOTORB_PINA 9
#define MOTORB_PINB 10


byte byteSpeed;
double angle;
#define threshold 0
bool firstSample;
void setup()
{
  // join I2C bus 000000000000(I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  
  /*
		BNO intialization
  */
	Serial.begin(9600);
	Serial.println("Orientation Sensor Test"); Serial.println("");
	/* Initialise the sensor */
	if(!bno.begin())
	{
	/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}
	delay(1000);
	bno.setExtCrystalUse(true);
  
  /*   PID setup   */
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255); 
  
//TODO4: research bno intialization       // The bno should be placed in 0 offset.
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
 firstSample = true;
}


void loop()
{
  // if programming failed, don't try to do anything

	 q = bno.getQuat(); // Request quaternion data from BNO055
   gravity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);// Request Gravity vector from BNO055
//   Serial.print("q = w: ");
//   Serial.print(q.w());
//   Serial.print(" x: ");
//   Serial.print(q.x());
//   Serial.print(" y: ");
//   Serial.print(q.y());
//   Serial.print(" z: ");
//   Serial.print(q.z());
//   Serial.println();

//   Serial.print("g = x: ");
//   Serial.print(gravity[0]);
//   Serial.print(" y: ");
//   Serial.print(gravity[1]);
//   Serial.print(" z: ");
//   Serial.print(gravity[2]);
//   Serial.println();

	 /* --------------------------------------------------------------------------------
							
							Calculating the YAW PITCH ROLL
	
	 -----------------------------------------------------------------------------------*/
	     // yaw: (about Z axis)
    ypr[0] = atan2(2*q.x()*q.y() - 2*q.w() *q.z(), 2*q.w() *q.w() + 2*q.x()*q.x() - 1);
    // pitch: (nose up/down, about Y axis)
    ypr[1] = atan2(gravity[0] , sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
    // roll: (tilt left/right, about X axis)
    ypr[2] = atan2(gravity[1] , gravity[2]);

//    Serial.print("ypr = yaw: ");
//    Serial.print(ypr[0]);
//    Serial.print(" pitch: ");
//    Serial.print(ypr[1]);
//    Serial.print(" roll: ");
//    Serial.print(ypr[2]);
//    Serial.println();
    
    if(gravity[2]<0) {
        if(ypr[1]>0) {
            ypr[1] = PI - ypr[1]; 
        } else { 
            ypr[1] = -PI - ypr[1];
        }
    }
//

	/* ------------------------------------------------------------------------------------*/
//
	 input = ypr[1]* 180/M_PI + 180;
//    if(firstSample == true){
//      setpoint = input;
//      firstSample = false;
//    }   
    
   Serial.print("input to PID: ");
   Serial.print(input);
   Serial.println();
	 
/* ------------------ Balancing Loop--------------------- */
    //no bno data - performing PID calculations and output to motors 
    pid.Compute();// computes the error difference between input and setPoint, and produce output calculation to minimize the error.
    //output = -output;
    Serial.print("output: ");
    Serial.print(output);
    Serial.println();
    angle = ypr[1]-movingAngleOffset;
    if(angle<threshold && angle>-threshold){
      move(0, MIN_ABS_SPEED);

    }else{
      move(output, MIN_ABS_SPEED);
    }
//    } else {
//      if(angle<0){
//        move(-output, MIN_ABS_SPEED);
//      }else{
//        move(output, MIN_ABS_SPEED);     
//      }
//    }


/* ------------------------------------------------------*/

}

void move(int Speed, int MIN)
{
  Speed = Speed * motorSpeedFactor;

  if (Speed < 0){
    Speed = -Speed;
    backward(Speed, MIN);
  } else if(Speed > 0) {
    forward(Speed, MIN);
  } else {
    stopMotor();
  }
}

void forward(int Speed, int MIN)
{
//  if(Speed < MIN){
//    Speed = MIN;
//  }
   
  //byteSpeed = map(Speed, 0, 100, 0, 255);
  analogWrite(MOTORB_PINA, Speed);
  analogWrite(MOTORB_PINB, 0);
  analogWrite(MOTORA_PINA, 0);
  analogWrite(MOTORA_PINB, Speed);
}

void backward(int Speed, int MIN)
{
//    if(Speed < MIN){
//    Speed = MIN;
//  }
  //byteSpeed = map(Speed, 0, 100, 0, 255);
  analogWrite(MOTORB_PINB, Speed);
  analogWrite(MOTORB_PINA, 0);
  analogWrite(MOTORA_PINB, 0);
  analogWrite(MOTORA_PINA, Speed);
}

void stopMotor(void)
{
  analogWrite(MOTORB_PINB, 0);
  analogWrite(MOTORB_PINA, 0);
  analogWrite(MOTORA_PINB, 0);
  analogWrite(MOTORA_PINA, 0);
}
