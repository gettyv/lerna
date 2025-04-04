#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

class Motor {
public:
    Motor(uint8_t pwmPin);
    void setPWM(float pwmSignal);
private:
    uint8_t pwmPin;
};

#endif // MOTOR_H