#include "LMotorController.h"
#include "Arduino.h"


LMotorController::LMotorController(int a1, int a2, int b1, int b2, double motorAConst, double motorBConst)
{
    _motorAConst = motorAConst;
    _motorBConst = motorBConst;

	_a1 = a1;
  _a2 = a2;
  _b1 = b1;
  _b2 = b2;

	pinMode(_a1, OUTPUT);
  pinMode(_a2, OUTPUT);
  pinMode(_b1, OUTPUT);
  pinMode(_b2, OUTPUT);
}


void LMotorController::move(int leftSpeed, int rightSpeed, int minAbsSpeed)
{
    if (rightSpeed < 0)
    {
        rightSpeed = min(rightSpeed, -1*minAbsSpeed);
        rightSpeed = max(rightSpeed, -255);
    }
    else if (rightSpeed > 0)
    {
        rightSpeed = max(rightSpeed, minAbsSpeed);
        rightSpeed = min(rightSpeed, 255);
    }

    int realRightSpeed = map(abs(rightSpeed), 0, 255, minAbsSpeed, 255);

    if (leftSpeed < 0)
    {
        leftSpeed = min(leftSpeed, -1*minAbsSpeed);
        leftSpeed = max(leftSpeed, -255);
    }
    else if (leftSpeed > 0)
    {
        leftSpeed = max(leftSpeed, minAbsSpeed);
        leftSpeed = min(leftSpeed, 255);
    }

    int realLeftSpeed = map(abs(leftSpeed), 0, 255, minAbsSpeed, 255);

    digitalWrite(_a1, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(_a2, rightSpeed > 0 ? LOW : HIGH);
    digitalWrite(_b1, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(_b2, leftSpeed > 0 ? LOW : HIGH);
}


void LMotorController::move(int speed, int minAbsSpeed)
{
    int direction = 1;

    if (speed < 0)
    {
        direction = -1;

        speed = min(speed, -1*minAbsSpeed);
        speed = max(speed, -255);
    }
    else
    {
        speed = max(speed, minAbsSpeed);
        speed = min(speed, 255);
    }

    if (speed == _currentSpeed) return;

    int realSpeed = max(minAbsSpeed, abs(speed));

    digitalWrite(_a1, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(_a2, rightSpeed > 0 ? LOW : HIGH);
    digitalWrite(_b1, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(_b2, leftSpeed > 0 ? LOW : HIGH);

    _currentSpeed = direction * realSpeed;
}


void LMotorController::move(int speed)
{
    if (speed == _currentSpeed) return;

    if (speed > 255) speed = 255;
    else if (speed < -255) speed = -255;

    digitalWrite(_a1, rightSpeed > 0 ? HIGH : LOW);
    digitalWrite(_a2, rightSpeed > 0 ? LOW : HIGH);
    digitalWrite(_b1, leftSpeed > 0 ? HIGH : LOW);
    digitalWrite(_b2, leftSpeed > 0 ? LOW : HIGH);

    _currentSpeed = speed;
}


void LMotorController::turnLeft(int speed, bool kick)
{
    digitalWrite(_a1, HIGH);
    digitalWrite(_a2, LOW);
    digitalWrite(_b1, LOW);
    digitalWrite(_b2, HIGH);

    if (kick)
    {
        analogWrite(_ena, 255);
        analogWrite(_enb, 255);

        delay(100);
    }

    analogWrite(_ena, speed * _motorAConst);
    analogWrite(_enb, speed * _motorBConst);
}


void LMotorController::turnRight(int speed, bool kick)
{
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, HIGH);
    digitalWrite(_in3, HIGH);
    digitalWrite(_in4, LOW);

    if (kick)
    {
        analogWrite(_ena, 255);
        analogWrite(_enb, 255);

        delay(100);
    }

    analogWrite(_ena, speed * _motorAConst);
    analogWrite(_enb, speed * _motorBConst);
}


void LMotorController::stopMoving()
{
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
    digitalWrite(_in3, LOW);
    digitalWrite(_in4, LOW);
    digitalWrite(_ena, HIGH);
    digitalWrite(_enb, HIGH);

    _currentSpeed = 0;
}
