#ifndef LINJEROBOT_AIS1104_PINS_H
#define LINJEROBOT_AIS1104_PINS_H

#pragma once
#include <Arduino.h>

// Motordriver

constexpr uint8_t MOTOR_STBY = D2;

// Motor A
constexpr uint8_t MOTOR_A_AIN1 = D3;
constexpr uint8_t MOTOR_A_AIN2 = D4;
constexpr uint8_t MOTOR_A_PWM = D5;

// Motor B
constexpr uint8_t MOTOR_B_AIN1 = D6;
constexpr uint8_t MOTOR_B_AIN2 = D7;
constexpr uint8_t MOTOR_B_PWM = D8;

// Encoder
constexpr uint8_t ENC_L_A = D9;
constexpr uint8_t ENC_L_B = D11;
constexpr uint8_t ENC_R_A = D12;
constexpr uint8_t ENC_R_B = D13;

// Sensor Array with 8 sensors from the 25 sensor array
constexpr uint8_t OUT9 = A0;
constexpr uint8_t OUT11 = A1;
constexpr uint8_t OUT13 = A2;
constexpr uint8_t OUT15 = A3;
constexpr uint8_t OUT17 = A4;
constexpr uint8_t OUT19 = A5;
constexpr uint8_t OUT21 = A6;
constexpr uint8_t OUT23 = A7;

constexpr uint8_t LEDON = D10;

#endif // LINJEROBOT_AIS1104_PINS_H
