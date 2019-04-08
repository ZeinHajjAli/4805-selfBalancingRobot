#include <Adafruit_Sensor.h>

#include <Adafruit_BNO055.h>

#include <Wire.h>


  
Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define aPWM 11
#define aDIR 12
#define bPWM 9
#define bDIR 10

#define PWM_SLOW 5
#define PWM_FAST 128
#define DIR_DELAY 1000
 
void setup(void) 
{
  //start serial transmission
  Serial.begin(9600);
  Serial.println("Starting demo code"); 
  Serial.println("");

  //------------------------------------------------------------------------------------
  //Start of BNO initialization
  /* Initialise the sensor */
  
  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
  //End of BNO initialization
  //------------------------------------------------------------------------------------
  //Start of Motor driver Initialization
  pinMode(aPWM, OUTPUT);
  pinMode(aDIR, OUTPUT);
  pinMode(bPWM, OUTPUT);
  pinMode(bDIR, OUTPUT);
  
  digitalWrite(aPWM, LOW);
  digitalWrite(aDIR, LOW);
  digitalWrite(bPWM, HIGH);
  digitalWrite(bDIR, HIGH);
  //End of Motor Driver Intitialization
  //------------------------------------------------------------------------------------
}
 
void loop(void) 
{
  //------------------------------------------------------------------------------------
  //BNO sensor serial test
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  
  delay(100);
  //------------------------------------------------------------------------------------
  //Motor Test
  /*
  goForward(PWM_FAST);
  delay(5000);
  digitalWrite(aPWM, LOW);
  digitalWrite(aDIR, LOW);
  digitalWrite(bPWM, HIGH);
  digitalWrite(bDIR, HIGH);
  goForward(PWM_SLOW);
  delay(5000);
  digitalWrite(aPWM, LOW);
  digitalWrite(aDIR, LOW);
  digitalWrite(bPWM, HIGH);
  digitalWrite(bDIR, HIGH);
  delay(1000);
  */
  digitalWrite(aPWM, LOW);
  digitalWrite(aDIR, HIGH);
  digitalWrite(bPWM, HIGH);
  digitalWrite(bDIR, LOW);
  delay(5000);
  digitalWrite(aPWM, LOW);
  digitalWrite(aDIR, LOW);
  digitalWrite(bPWM, HIGH);
  digitalWrite(bDIR, HIGH);
  delay(1000);
  digitalWrite(aPWM, HIGH);
  digitalWrite(aDIR, LOW);
  analogWrite(bPWM, 50);  
  digitalWrite(bDIR, HIGH);
  delay(5000);
  digitalWrite(aPWM, LOW);
  digitalWrite(aDIR, LOW);
  digitalWrite(bPWM, HIGH);
  digitalWrite(bDIR, HIGH);
  
  
}

void goForward(int motorSpeed)
{
  Serial.println("started 'goForward'");

  digitalWrite(aPWM, LOW);
  digitalWrite(aDIR, LOW);
  digitalWrite(bPWM, HIGH);
  digitalWrite(bDIR, HIGH);

  delay(DIR_DELAY);

  digitalWrite(aDIR, HIGH);
  analogWrite(aPWM, 128-motorSpeed);
  digitalWrite(bDIR, HIGH);
  analogWrite(bPWM, motorSpeed);
}

