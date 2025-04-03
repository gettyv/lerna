#include <Arduino.h>
#include <ArduinoLog.h>
#include <Encoder.h>
#include <Constants.h>
#include <TimerOne.h>
#include <Motor.h>
#include <PID.h>

// TimerOne timer;
// Motor motorA(MOTORA_PWM_PIN);
// Motor motorB(MOTORB_PWM_PIN);
// PID pidA(MOTORA_KP, MOTORA_KI, MOTORA_KD);
// PID pidB(MOTORB_KP, MOTORB_KI, MOTORB_KD);

// Encoders and state
uint32_t lastUpdateTime = NULL;
uint32_t currentUpdateTime = NULL;

Encoder encoderMotorA;
// Encoder encoderMotorB;

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
  // encoderMotorB.updateVelocity(encoderBPosition, 
  //   encoderBLastPosition, currentUpdateTime, lastUpdateTime);

  encoderALastPosition = encoderAPosition;
  // encoderBLastPosition = encoderBPosition;
  lastUpdateTime = currentUpdateTime;

}

void interruptUpdateEncoderA() {
  Serial.println("Interrupt Update Encoder A Called");
  bool currentStateA = digitalRead(MOTORA_ENCODER_PIN_A);
  bool currentStateB = digitalRead(MOTORA_ENCODER_PIN_B);
  Serial.print("Current State A: ");
  Serial.println(currentStateA);
  Serial.print("Current State B: ");
  Serial.println(currentStateB);
  encoderAPosition += encoderMotorA.encoderUpdateInterrupt(
    encoderALastStateA, currentStateA, currentStateB);

  encoderALastStateA = currentStateA;
}

// void interruptUpdateEncoderB() {
//   bool currentStateA = digitalRead(MOTORB_ENCODER_PIN_A);
//   bool currentStateB = digitalRead(MOTORB_ENCODER_PIN_B);

//   encoderBPosition += encoderMotorB.encoderUpdateInterrupt(
//     encoderBLastStateA, currentStateA, currentStateB);

//   encoderBLastStateA = currentStateA;
// }

// Where the magic happens!
// void ctrlf() {
//   Serial.println("Control Function Called");

//   updateEncoderVelocity();
//   Serial.println("Updated Velocities");

//   float motorAVelocity = encoderMotorA.getVelocity();
//   float pidACommand = pidA.step(motorAVelocity);
//   motorA.setPWM(pidACommand);
//   Serial.println("Updated Motor A Command");

//   float motorBVelocity = encoderMotorB.getVelocity();
//   float pidBCommand = pidB.step(motorBVelocity);
//   motorB.setPWM(pidBCommand);
//   Serial.println("Updated Motor B Command");
// }


void setup() {
  Serial.begin(9600);
  Serial.println("Beginning Setup");

  // Encoders should likely be setup last due to their use of interrupts
  Serial.println("Starting Up Encoders, interrupts, and timer");
  encoderMotorA.begin();
  // encoderMotorB.begin();

  attachInterrupt(MOTORA_ENCODER_PIN_A, interruptUpdateEncoderA, CHANGE);
  // attachInterrupt(MOTORB_ENCODER_PIN_A, interruptUpdateEncoderB, CHANGE);

  // // Attach control function to timer
  // Serial.println("Attaching Timer");
  // timer.initialize(CONTROL_FUNCTION_PERIOD_US);
  // timer.attachInterrupt(ctrlf);

  Serial.println("Prepare Global State");
  lastUpdateTime = micros();

  // Serial.println("Begin the timer");
  // timer.start();
}

void loop() {
  // Nothing to see here :0  
  updateEncoderVelocity();
  delay(1000);
  Serial.print("Encoder A Position: ");
  Serial.println(encoderAPosition);
  Serial.print("Encoder A Velocity: ");
  Serial.println(encoderMotorA.getVelocity());

}