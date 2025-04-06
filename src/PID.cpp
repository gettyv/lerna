#include <Arduino.h>
#include <PID.h>

PID::PID(float Kp, float Ki, float Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::setRef(float ref) {
    this->ref = ref;
}

float PID::step(float reading) {
    long currentTime = micros();
    float deltaTime;

    // handle first call
    if (lastUpdateTime < 0) {
        deltaTime = 0;
        lastUpdateTime = currentTime;
    } else {
        deltaTime = (currentTime - lastUpdateTime) / 1.0e6; // Convert to seconds
        lastUpdateTime = currentTime;
    }

    error = ref - reading;

    // Proportional term
    float Pout = Kp * error;

    // Integral term
    accumulator += error * deltaTime;
    float Iout = Ki * accumulator;

    // Derivative term (skip on the first call)
    float Dout = 0;
    if (deltaTime > 0) {
        float derivative = (error - lastError) / deltaTime;
        Dout = Kd * derivative;
    }

    lastError = error;

    return Pout + Iout + Dout;
}