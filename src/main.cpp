#include <Arduino.h>
#include <ArduinoLog.h>
#include <Encoder.h>
#include <constants.h>

// Encoders and state
uint32_t lastUpdateTime = NULL;
uint32_t currentUpdateTime = NULL;

Encoder encoderMotorA;
Encoder encoderMotorB;

bool encoderALastStateA = NULL;
bool encoderBLastStateA = NULL;

long encoderAPosition = 0;
long encoderBPosition = 0;

long encoderALastPosition = 0;
long encoderBLastPosition = 0;

// ISR that should be called routinely by the timer
void timerUpdateEncoderVelocity() {
  currentUpdateTime = micros();

  encoderMotorA.updateVelocity(encoderAPosition, 
    encoderALastPosition, currentUpdateTime, lastUpdateTime);
  encoderMotorB.updateVelocity(encoderBPosition, 
    encoderBLastPosition, currentUpdateTime, lastUpdateTime);

  encoderALastPosition = encoderAPosition;
  encoderBLastPosition = encoderBPosition;
  lastUpdateTime = currentUpdateTime;

}

void interruptUpdateEncoderA() {
  return
}

void interruptUpdateEncoderB() {
  return
}


void setup() {
  Serial.begin(115200);
  Log.begin(LOG_LEVEL_VERBOSE, &Serial, true);
  Log.noticeln("Setup");

  Log.noticeln("Encoder setup");
  encoder.begin();
  Log.noticeln("Encoder setup complete.");

  Log.noticeln("Setup complete.");
  pinMode(36, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(36), []() {
    Serial.println("Interrupt triggered on pin 36");
  }, CHANGE);
}

void loop() {
  Log.noticeln("Loop");
  delay(1000);
  Serial.println("Position: " + String(encoder.getPosition()));
  Serial.println("Velocity: " + String(encoder.getVelocity()));
  Serial.println("DigitalRead: " + String(digitalRead(MOTOR1_ENCODER_PIN_A)));


  // Log.noticeln("Position: %ld", encoder.getPosition());
  // Log.noticeln("Velocity: %f", encoder.getVelocity());
  
}