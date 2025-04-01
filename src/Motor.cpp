#include <Arduino.h>
#include <Motor.h>

Motor::Motor(uint8_t pwmPinIn) {
    pwmPin = pwmPinIn;
}

void Motor::setPWM(float pwmSignal) {
    pwmSignal = map(pwmSignal, 0.0, 1.0, 0.0, 255.0);
    analogWrite(pwmPin, pwmSignal);
}