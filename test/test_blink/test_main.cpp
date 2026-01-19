#include <Arduino.h>
#include <unity.h>

void test_simple_pass() {
  TEST_ASSERT_EQUAL(33, 33);
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && (millis() - start < 5000)) { delay(10); }
  delay(200);

  UNITY_BEGIN();
  RUN_TEST(test_simple_pass);
  UNITY_END();
}

void loop() { delay(1000); }
