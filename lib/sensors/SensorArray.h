#ifndef LINJEROBOT_AIS1104_ARRAY_H
#define LINJEROBOT_AIS1104_ARRAY_H

#pragma once

#include "config/pins.h"

class SensorArray {
  public:
  void begin();
  void calibrateStep();
  float readError();
};

#endif // LINJEROBOT_AIS1104_ARRAY_H
