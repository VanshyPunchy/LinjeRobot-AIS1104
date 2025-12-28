#include "PID.h"

#include "config/RobotConfig.h"

float PID::update(float setpoint, float measurement, float dt)
{
  float error = setpoint - measurement;
  integral_+= dt * error;

  if (integral_ > maxI_)
    integral_ = maxI_;
  if (integral_ < minI_)
    integral_ = minI_;

  float derivative = (error - prevError_) / dt;
  float output = kp_ * error + ki_ * integral_ + kd_ * derivative;

  if (output > maxOut_)
    output = maxOut_;
  if (output < minOut_)
    output = minOut_;

  prevError_ = error;
  return output;
}


void PID::setGain(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
