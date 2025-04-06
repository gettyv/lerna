#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

class Motor {
    public:
        Motor(uint8_t pwmPin, uint8_t cwPin, uint8_t ccwPin) : pwmPin(pwmPin), cwPin(cwPin), ccwPin(ccwPin) {
            pinMode(pwmPin, OUTPUT);
            pinMode(cwPin, OUTPUT);
            pinMode(ccwPin, OUTPUT);
        }
        void setSpeed(float command);

    private:
        uint8_t pwmPin;
        uint8_t cwPin;
        uint8_t ccwPin;

};

#endif // MOTOR_H