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
const float ENCODER_PPR = 48;
const uint16_t CONTROL_FUNCTION_PERIOD_US = 1000; // 1ms

// Controllers
const float MOTORA_KP = 0.4;
const float MOTORA_KI = 0.0;
const float MOTORA_KD = 0.0;

const float MOTORB_KP = 0.4;
const float MOTORB_KI = 0.0;
const float MOTORB_KD = 0.0;

// Pins

// Motor A
const uint8_t MOTORA_PWM_PIN = 44;
const uint8_t MOTORA_ENCODER_PIN_A = 2;
const uint8_t MOTORA_ENCODER_PIN_B = 4;
const uint8_t MOTORA_CW_A_PIN = 24;
const uint8_t MOTORA_CCW_B_PIN = 25;

// Motor B
const uint8_t MOTORB_PWM_PIN = 45;
const uint8_t MOTORB_ENCODER_PIN_A = 3;
const uint8_t MOTORB_ENCODER_PIN_B = 5;
const uint8_t MOTORB_CW_A_PIN = 28;
const uint8_t MOTORB_CCW_B_PIN = 29;

// LED Strip 1
const uint8_t LED_STRIP1_DATA_PIN = 6;
const uint16_t LED_STRIP1_NUM_PIXELS = 300;


#endif