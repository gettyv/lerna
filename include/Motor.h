#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

class Motor {
public:
    Motor(uint8_t pwmPin, uint8_t encoderPinA, uint8_t encoderPinB);
    void setTargetSpeed(float speed);
    void update();
    float getCurrentSpeed();

private:
    void updateEncoder();
    void updatePWM();

    uint8_t pwmPin;
    uint8_t encoderPinA;
    uint8_t encoderPinB;

    float targetSpeed;
    float currentSpeed;
    float pwmOutput;

    float kp;
    float ki;
    float kd;

    float integral;
    float previousError;
    unsigned long lastUpdateTime;
};

#endif // MOTOR_H