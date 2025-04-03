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
    long currentTime = micros(); // Get the current time in microseconds
    float deltaTime = currentTime - lastUpdateTime;

    if (lastUpdateTime < 0) {
        deltaTime = 1e16; // Set a large deltaTime for the first call
    }
    
    lastUpdateTime = currentTime;

    error = ref - reading;

    // Proportional term
    float Pout = Kp * error;

    // Integral term
    accumulator += error * deltaTime;
    float Iout = Ki * accumulator;

    // Derivative term
    float derivative = (error - lastError) / deltaTime;
    float Dout = Kd * derivative;

    lastError = error;

    return Pout + Iout + Dout;
}