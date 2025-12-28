#include "Encoder.h"

 Encoder::Encoder(uint8_t pinA, uint8_t pinB)
   : pinA_(pinA), pinB_(pinB) {}

void Encoder::begin() {
   pinMode(pinA_, INPUT_PULLUP);
   pinMode(pinB_, INPUT_PULLUP);

   // attach interrupt on A, pass "this" as arg
   attachInterruptArg(digitalPinToInterrupt(pinA_), &Encoder::isrA, this, CHANGE);
 }

void Encoder::isrA(void* arg) {
   static_cast<Encoder*>(arg)->handleA_();
 }

void Encoder::handleA_() {
   // Read both signals
   bool a = digitalRead(pinA_);
   bool b = digitalRead(pinB_);

   if (a == b )
     tickCount_++;
   else
     tickCount_--;
 }

int32_t Encoder::getTicks() const {
   return tickCount_;
 }

int32_t Encoder::getDelta() {
   int32_t now = tickCount_;
   int32_t diff = now - lastTick_;
   lastTick_ = now;
   return diff;
 }

float Encoder::getVelocity(float dt) {
   if (dt <= 0.0f)
     return 0.0f;
   return getDelta() / dt;
 }
