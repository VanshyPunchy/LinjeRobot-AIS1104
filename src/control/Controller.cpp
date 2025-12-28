#include "../lib/Encoder/Encoder.h"
#include "../lib/PID/PID.h"
#include "config/RobotConfig.h"
#include "config/pins.h"
#include "drivers/MotorDriver.h"

Encoder leftEncoder(ENC_L_A, ENC_L_B);
Encoder rightEncoder(ENC_R_A, ENC_R_B);

static PID leftSpeedPID
(
  Config::SP_kp,
  Config::SP_ki,
  Config::SP_kd
  );

static PID rightSpeedPID
(
  Config::SP_kp,
  Config::SP_ki,
  Config::SP_kd
  );

static PID linePID
(
  Config::LN_kp,
  Config::LN_ki,
  Config::LN_kd
  );

MotorDriver motors;