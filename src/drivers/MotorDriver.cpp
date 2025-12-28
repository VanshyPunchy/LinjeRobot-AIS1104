#include "MotorDriver.h"
#include "config/pins.h"

#include <Arduino.h>


float clampf(float v, float lo, float hi)
{
  if (v < lo)
    return lo;
  if (v > hi)
    return hi;
  return v;
}

int toPwm(float v)
{
  float mag = v >= 0.0f ? v : -v;
  int pwm = int(mag * 255.0f + 0.5f);
  if (pwm < 0)
    pwm = 0;
  if (pwm > 255)
    pwm = 255;
  return pwm;
}

static void setOneMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t cmd)
{
  cmd = clampf(cmd, -1.0f, 1.0f);

  const bool forward = (cmd >= 0.0f);
  const int pwm = toPwm(cmd);

  // Direction
  digitalWrite(in1, forward ? HIGH : LOW);
  digitalWrite(in2, forward ? LOW : HIGH);

  // Speed
  analogWrite(pwmPin, pwm);
}

void MotorDriver::begin()
{
  pinMode(MOTOR_STBY, OUTPUT);

  pinMode(MOTOR_A_AIN1, OUTPUT);
  pinMode(MOTOR_A_AIN2, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);

  pinMode(MOTOR_B_AIN1, OUTPUT);
  pinMode(MOTOR_B_AIN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);

  digitalWrite(MOTOR_STBY, HIGH);

  analogWrite(MOTOR_A_PWM, 0);
  analogWrite(MOTOR_B_PWM, 0);
  digitalWrite(MOTOR_A_AIN1, LOW);
  digitalWrite(MOTOR_A_AIN2, LOW);
  digitalWrite(MOTOR_B_AIN1, LOW);
  digitalWrite(MOTOR_B_AIN2, LOW);
}

void MotorDriver::set(float left, float right)
{
  setOneMotor(MOTOR_A_AIN1, MOTOR_A_AIN2, MOTOR_A_PWM, left);
  setOneMotor(MOTOR_B_AIN1, MOTOR_B_AIN2, MOTOR_B_PWM, right);
}

void MotorDriver::brake()
{
  digitalWrite(MOTOR_A_AIN1, HIGH);
  digitalWrite(MOTOR_A_AIN2, HIGH);
  analogWrite(MOTOR_B_PWM, 255);

  digitalWrite(MOTOR_B_AIN1, HIGH);
  digitalWrite(MOTOR_B_AIN2, HIGH);
  analogWrite(MOTOR_B_PWM, 255);
}