#include <Arduino.h>
#include <Encoder.h>
#include <TimerOne.h>
#include <TimerThree.h>

Encoder::Encoder(uint8_t encoderPinA, uint8_t encoderPinB, uint16_t ppr, unsigned long update_period_us,bool useTimer3) 
    : pinA(encoderPinA), pinB(encoderPinB), pulsesPerRev(ppr), useTimerThree(useTimer3), velocity_update_period_us(update_period_us){
    instance = this;
    }

void Encoder::begin() {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    lastStateA = digitalRead(pinA);
    
    // Attach pin interrupt
    Serial.println("Attaching interrupt");
    attachInterrupt(digitalPinToInterrupt(pinA), Encoder::isrWrapper, CHANGE);
    
    // Configure timer based on flag
    if (useTimerThree) {
        Timer3.initialize(velocity_update_period_us);
        Timer3.attachInterrupt(timer3IsrWrapper);
    } else {
        Timer1.initialize(velocity_update_period_us);
        Timer1.attachInterrupt(timer1IsrWrapper);
    }
}

void Encoder::handleInterrupt() {
    bool currentStateA = digitalRead(pinA);
    bool currentStateB = digitalRead(pinB);
    if (currentStateA && !lastStateA) {
        if (currentStateB) position++; else position--;
    }
    lastStateA = currentStateA;
}

void Encoder::updateVelocity() {
    long currentPosition = position;
    long deltaPosition = currentPosition - lastPosition;
    float deltaTime = velocity_update_period_us / 1.0e6;
    float pulsesPerSec = deltaPosition / deltaTime;
    velocity = pulsesPerSec / pulsesPerRev;
    lastPosition = currentPosition;
}

float Encoder::getVelocity() {
    noInterrupts();
    float currentVelocity = velocity;
    interrupts();
    return currentVelocity;
}

long Encoder::getPosition() {
    noInterrupts();
    long currentPosition = position;
    interrupts();
    return currentPosition;
}

void Encoder::isrWrapper() {
    Serial.println("A/B state interrupt");
    instance->handleInterrupt();
}

void Encoder::timer1IsrWrapper() {
    // Serial.println("Timer1 interrupt");
    instance->updateVelocity();
}

void Encoder::timer3IsrWrapper() {
    Serial.println("Timer3 interrupt");
    instance->updateVelocity();
}

Encoder* Encoder::instance = nullptr;