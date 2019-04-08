#include "HCMotor.h"

#define MOTORA_PINA 11
#define MOTORA_PINB 12
#define MOTORB_PINA 9
#define MOTORB_PINB 10

HCMotor HCMotor;
int Speed;
 
void setup(void) 
{
  HCMotor.Init();


  HCMotor.attach(1, DCMOTOR_H_BRIDGE, MOTORB_PINA, MOTORB_PINB);

  HCMotor.DutyCycle(0, 100);

}
 
void loop(void) 
{

  HCMotor.Direction(1, REVERSE);

  Speed = 100;
  HCMotor.OnTime(1, Speed);


  delay(5000);

  HCMotor.Direction(1, REVERSE);

  Speed = 25;
  HCMotor.OnTime(1, Speed);


  delay(5000);
//  HCMotor.Direction(0, FORWARD);
//  Speed = 0;
//  HCMotor.OnTime(0, Speed);
//
//  HCMotor.Direction(0, FORWARD);
//  Speed = 25;
//  HCMotor.OnTime(0, Speed);
//
//  HCMotor.Direction(0, FORWARD);
//  Speed = 50;
//  HCMotor.OnTime(0, Speed);
//
//  HCMotor.Direction(0, FORWARD);
//  Speed = 75;
//  HCMotor.OnTime(0, Speed);
//
//  HCMotor.Direction(0, FORWARD);
//  Speed = 100;
//  HCMotor.OnTime(0, Speed);
  
}

