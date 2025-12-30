#ifndef LINJEROBOT_AIS1104_ROBOTCONFIG_H
#define LINJEROBOT_AIS1104_ROBOTCONFIG_H

#pragma once

namespace Config {

// Speed PID (tick/s -> motor cmd)
constexpr float SP_kp = 0.003f;
constexpr float SP_ki = 0.0008f;
constexpr float SP_kd = 0.0f;

// Line PID (line error -> steer)
constexpr float LN_kp = 1.2f;
constexpr float LN_ki = 0.0f;
constexpr float LN_kd = 0.08f;

constexpr float targetLeftSpeed = 100.f;
constexpr float targetRightSpeed = 100.f;

// Calibration
constexpr float WigglePWR = 0.25f; // How hard to turn
constexpr uint32_t FlipMS = 200; // Change direction every 200 ms
constexpr uint32_t CalTime = 8000; // Calibrate for 8 seconds
}

#endif // LINJEROBOT_AIS1104_ROBOTCONFIG_H
