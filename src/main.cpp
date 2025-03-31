#include <Arduino.h>
#include <ArduinoLog.h>
#include <Encoder.h>
#include <constants.h>

Encoder encoder(MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B, ENCODER_PPR, ENCODER_VEL_UPDATE_PERIOD_US, false);

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