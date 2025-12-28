#ifndef LINJEROBOT_AIS1104_ARRAY_H
#define LINJEROBOT_AIS1104_ARRAY_H

#pragma once

#include "config/pins.h"

class SensorArray {
  public:
  void begin();

  void calibration();

  void loop();
};

#endif // LINJEROBOT_AIS1104_ARRAY_H
