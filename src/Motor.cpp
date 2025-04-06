#include <Arduino.h>
#include <Motor.h>

void Motor::setSpeed(float speed) {
    float pwmSignal = map(speed, -1.0, 1.0, -255.0, 255.0);
    if (pwmSignal > 0) {
        // Counter-clockwise is positive
        digitalWrite(cwPin, LOW);
        digitalWrite(ccwPin, HIGH);
    } else {
        digitalWrite(cwPin, HIGH);
        digitalWrite(ccwPin, LOW);
    }
    analogWrite(pwmPin, abs(pwmSignal));
}