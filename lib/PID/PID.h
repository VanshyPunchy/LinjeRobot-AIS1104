#ifndef LINJEROBOT_AIS1104_PID_H
#define LINJEROBOT_AIS1104_PID_H

#pragma once

class PID
{
  public:
  PID
  (
    float kp,
    float ki,
    float kd
  );

  void update
  (
    float setpoint,
    float measurement,
    float dt
    );

  void setGain
  (
    float kp,
    float ki,
    float kd
    );

  void reset();

private:
  float kp_, ki_, kd_;
  float integral_{0.0f};
  float prevError_{0.0f};
};

#endif // LINJEROBOT_AIS1104_PID_H
