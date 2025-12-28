#ifndef LINJEROBOT_AIS1104_ENCODER_H
#define LINJEROBOT_AIS1104_ENCODER_H

#pragma once

#include "config/pins.h"

class Encoder {
  public:
  Encoder
  (
    uint8_t pinA,
    uint8_t pinB
    );
  void begin();
  int32_t getTicks() const;
  int32_t getDelta();
  float getVelocity(float dt);

private:
  uint8_t pinA_, pinB_;
  int32_t tickCount_{0};
  int32_t lastTick_{0};


  // ISR = Interrupt Service Routine
  // IRAM = internal ram on ESP32, such that function is in RAM, not flash memory
  static void IRAM_ATTR isrA(void* arg);
  void handleA_();
};

#endif // LINJEROBOT_AIS1104_ENCODER_H
