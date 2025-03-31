/*
IMPORTANT:
Certain Timers on the MEGA correspond to PWM capabilities on certain pins.
We have three encoders, and thus are planning on using Timer1, Timer3, and Timer4t to handle them.
This leaves the following pins for PWM:

Timer0: 4, 13 (used for millis and such)
Timer2: 9, 10.
Timer5: 44, 45, 46.

The plan is to use Timer 5 for motor PWM control, as we can change the PWM freq. for that
timer if we want.




*/


#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <Arduino.h>

// Encoders
const float ENCODER_PPR = 211.2;
const uint16_t ENCODER_VEL_UPDATE_PERIOD_US = 1000; // 1ms


// Pins

// Motor 1
const uint8_t MOTOR1_PWM_PIN = 44;
const uint8_t MOTOR1_ENCODER_PIN_A = 22;
const uint8_t MOTOR1_ENCODER_PIN_B = 23;
const uint8_t MOTOR1_CW_A_PIN = 24;
const uint8_t MOTOR1_CCW_B_PIN = 25;

// Motor 2
const uint8_t MOTOR2_PWM_PIN = 45;
const uint8_t MOTOR2_ENCODER_PIN_A = 26;
const uint8_t MOTOR2_ENCODER_PIN_B = 27;
const uint8_t MOTOR2_CW_A_PIN = 28;
const uint8_t MOTOR2_CCW_B_PIN = 29;

// LED Strip 1
const uint8_t LED_STRIP1_DATA_PIN = 6;
const uint16_t LED_STRIP1_NUM_PIXELS = 300;


#endif