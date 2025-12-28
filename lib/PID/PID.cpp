#include "PID.h"

#include "config/RobotConfig.h"

void PID::setGain(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
