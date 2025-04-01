#include <Arduino.h>
#include <PID.h>

PID::PID(float Kp, float Ki, float Kd) {
    Kp = Kp;
    Ki = Ki;
    Kd = Kd;
}