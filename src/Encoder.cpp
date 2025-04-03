#include <Arduino.h>
#include <Encoder.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <Constants.h>

Encoder::Encoder(){
    return;
}

void Encoder::begin() {return;}

int Encoder::encoderUpdateInterrupt(bool lastStateA, bool currentStateA, bool currentStateB) {
    int valToIncrement = NULL;
    if (currentStateA && !lastStateA) {
        if (currentStateB) valToIncrement = 1; else valToIncrement = -1;
    }
    return valToIncrement;
}

void Encoder::updateVelocity(long currentPosition,
    long lastPosition, uint32_t currentUpdateTime, uint32_t lastUpdateTime) {
    long deltaPosition = currentPosition - lastPosition;
    // float deltaTime = ENCODER_VEL_UPDATE_PERIOD_US / 1.0e6;
    float deltaTime = (currentUpdateTime - lastUpdateTime) / 1.0e6;
    float pulsesPerSec = deltaPosition / deltaTime;
    velocity = pulsesPerSec / ENCODER_PPR;
}

float Encoder::getVelocity() {
    noInterrupts();
    float currentVelocity = velocity;
    interrupts();
    return currentVelocity;
}