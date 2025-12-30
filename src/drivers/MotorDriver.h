#ifndef LINJEROBOT_AIS1104_MOTORDRIVER_H
#define LINJEROBOT_AIS1104_MOTORDRIVER_H

#pragma once

class MotorDriver {
public:
  void begin();

  void set(
    float left,
    float right
    );

  void brake();
  void stop();

private:
  static float clamp_(float v, float lo, float hi);
  static int toPwm_(float v);
};

#endif // LINJEROBOT_AIS1104_MOTORDRIVER_H