#include "control/Controller.cpp"

void setup() {
  leftEncoder.begin();
  rightEncoder.begin();
}

void loop() {
  static unsigned long lastTime = millis();
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0f;
  lastTime = currentTime;

  float leftSpeed = leftEncoder.getVelocity(dt);
  float rightSpeed = rightEncoder.getVelocity(dt);

  float leftCmd = leftSpeedPID.update(Config::targetLeftSpeed, leftSpeed, dt);
  float rightCmd = rightSpeedPID.update(Config::targetRightSpeed, rightSpeed, dt);

  motors.set(leftCmd, rightCmd);

  // 5 = run at approx 200Hz
  delay(5);
}