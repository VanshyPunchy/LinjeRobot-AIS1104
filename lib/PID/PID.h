#ifndef LINJEROBOT_AIS1104_PID_H
#define LINJEROBOT_AIS1104_PID_H

#pragma once

#include "config/RobotConfig.h"

class PID {
  public:
  struct Limits {
    float minOut = -1.0f;
    float maxOut = 1.0f;
  };

  PID(
    float kp = RobotConfig::kp,
    float ki = RobotConfig::ki,
    float kd = RobotConfig::kd
  );

  void setGains(float kp, float ki, float kd);
  void setOutputLim(float minOut, maxOut);
  void setIntegralLim(float minI, float maxI);

  void reset(float integral = RobotConfig::integral);
};

#endif // LINJEROBOT_AIS1104_PID_H
