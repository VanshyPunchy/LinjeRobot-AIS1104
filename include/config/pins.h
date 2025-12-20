#ifndef LINJEROBOT_AIS1104_PINS_H
#define LINJEROBOT_AIS1104_PINS_H

#pragma once
#include <Arduino.h>

// Motordriver

constexpr uint8_t MOTOR_STBY = D2;

// Motor A
constexpr uint8_t MOTOR_A_AIN1 = D4;
constexpr uint8_t MOTOR_A_AIN2 = D5;
constexpr uint8_t MOTOR_A_PWM = D6;

// Motor B
constexpr uint8_t MOTOR_B_AIN1 = D7;
constexpr uint8_t MOTOR_B_AIN2 = D8;
constexpr uint8_t MOTOR_B_PWM = D9;

// Encoder
constexpr uint8_t ENC_L_A = D18;
constexpr uint8_t ENC_L_B = D17;
constexpr uint8_t ENC_R_A = D16;
constexpr uint8_t ENC_R_B = D15;

// Sensor Array
constexpr uint8_t OUT1 = A0;
constexpr uint8_t OUT3 = A1;
constexpr uint8_t OUT5 = A2;
constexpr uint8_t OUT7 = A3;
constexpr uint8_t OUT9 = A4;
constexpr uint8_t OUT11 = A5;
constexpr uint8_t OUT13 = A6;
constexpr uint8_t OUT15 = A7;
constexpr uint8_t OUT17 = A8;
constexpr uint8_t OUT19 = A9;
constexpr uint8_t OUT21 = A10;
constexpr uint8_t OUT23 = A11;
constexpr uint8_t OUT25 = A12;

constexpr uint8_t LEDON = D10;

#endif // LINJEROBOT_AIS1104_PINS_H
