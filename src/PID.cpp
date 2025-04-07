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
    // Serial.print(Pout);
    // Serial.print(", ");
    // Serial.print(Iout);
    // Serial.print(", ");
    // Serial.print(Dout);
    // Serial.print(", ");
    // Serial.print(error);
    // Serial.print(", ");
    // Serial.print(accumulator);
    // Serial.print(", ");

    // 0.5 is added to start in the middle range of the motor, which will then 
    // be adjusted by the PID controller
    float output = Pout + Iout + Dout;
    // Clamp the output to a range of -1.0 to 1.0
    output = constrain(output, -1.0, 1.0);

    return output;
}