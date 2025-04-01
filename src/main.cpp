#include <Arduino.h>
#include <ArduinoLog.h>
#include <Encoder.h>
#include <Constants.h>
#include <TimerOne.h>

TimerOne timer;

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
void updateEncoderVelocity() {
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
  bool currentStateA = digitalRead(MOTOR1_ENCODER_PIN_A);
  bool currentStateB = digitalRead(MOTOR1_ENCODER_PIN_B);

  encoderAPosition += encoderMotorA.encoderUpdateInterrupt(
    encoderALastStateA, currentStateA, currentStateB);

  encoderALastStateA = currentStateA;
}

void interruptUpdateEncoderB() {
  bool currentStateA = digitalRead(MOTOR2_ENCODER_PIN_A);
  bool currentStateB = digitalRead(MOTOR2_ENCODER_PIN_B);

  encoderBPosition += encoderMotorB.encoderUpdateInterrupt(
    encoderBLastStateA, currentStateA, currentStateB);

  encoderBLastStateA = currentStateA;
}

// Where the magic happens!
void ctrlf() {
  Serial.println("Control Function Called");
  updateEncoderVelocity();
  Serial.println("Updated Velocities");
}


void setup() {
  Serial.begin(115200);
  Serial.println("Beginning Setup");

  // Encoders should likely be setup last due to their use of interrupts
  Serial.println("Starting Up Encoders, interrupts, and timer");
  encoderMotorA.begin();
  encoderMotorB.begin();

  attachInterrupt(MOTOR1_ENCODER_PIN_A, interruptUpdateEncoderA, CHANGE);
  attachInterrupt(MOTOR2_ENCODER_PIN_A, interruptUpdateEncoderB, CHANGE);

  // Attach control function to timer
  Serial.println("Attaching Timer");
  timer.initialize(CONTROL_FUNCTION_PERIOD_US);
  timer.attachInterrupt(ctrlf);

  Serial.println("Begin the timer");
  timer.start();
}

void loop() {
  // Nothing to see here :0  
}