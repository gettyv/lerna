#include <Arduino.h>
#include <ArduinoLog.h>
#include <Encoder.h>
#include <Constants.h>
#include <TimerOne.h>
#include <Motor.h>
#include <PID.h>
#include <Led_strip.h>
#include <Panel.h>

Panel panel(9600);

// TimerOne timer;
Motor motorA(MOTORA_PWM_PIN, MOTORA_CW_A_PIN, MOTORA_CCW_B_PIN);
Motor motorB(MOTORB_PWM_PIN, MOTORB_CW_A_PIN, MOTORB_CCW_B_PIN);

LedStrip* foxTowerLeds[FOX_TOWER_STRIPS];

int ledPattern = 0;

PID pidA(MOTORA_KP, MOTORA_KI, MOTORA_KD);
PID pidB(MOTORB_KP, MOTORB_KI, MOTORB_KD);

// Encoders and state
volatile uint32_t lastUpdateTime = NULL;
volatile uint32_t currentUpdateTime = NULL;

// This is zero, so we will almost certainly immediately run motor update
uint32_t lastLoopTime = 0;
uint32_t currentLoopTime = 0;

Encoder encoderMotorA;
Encoder encoderMotorB;

volatile bool encoderALastStateA = NULL;
volatile bool encoderBLastStateA = NULL;

volatile long encoderAPosition = 0;
volatile long encoderBPosition = 0;

volatile long encoderALastPosition = 0;
volatile long encoderBLastPosition = 0;

// ISR that should be called routinely by the timer
void updateEncoderVelocity() {

  noInterrupts();
  long currentEncoderAPosition = encoderAPosition;
  long currentEncoderBPosition = encoderBPosition;
  interrupts();

  currentUpdateTime = micros();

  encoderMotorA.updateVelocity(currentEncoderAPosition, 
    encoderALastPosition, currentUpdateTime, lastUpdateTime);
  encoderMotorB.updateVelocity(currentEncoderBPosition, 
    encoderBLastPosition, currentUpdateTime, lastUpdateTime);

  encoderALastPosition = currentEncoderAPosition;
  encoderBLastPosition = currentEncoderBPosition;
  lastUpdateTime = currentUpdateTime;

}

void interruptUpdateEncoderA() {
  bool currentStateA = digitalRead(MOTORA_ENCODER_PIN_A);
  bool currentStateB = digitalRead(MOTORA_ENCODER_PIN_B);

  encoderAPosition += encoderMotorA.encoderUpdateInterrupt(
    encoderALastStateA, currentStateA, currentStateB);

  encoderALastStateA = currentStateA;
}

void interruptUpdateEncoderB() {
  bool currentStateA = digitalRead(MOTORB_ENCODER_PIN_A);
  bool currentStateB = digitalRead(MOTORB_ENCODER_PIN_B);

  encoderBPosition += encoderMotorB.encoderUpdateInterrupt(
    encoderBLastStateA, currentStateA, currentStateB);

  encoderBLastStateA = currentStateA;
}

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
  panel.begin();
  panel.sendMessage("Waiting for panel");
  if (panel.waitReady(100e3) == 0) {
    panel.sendMessage("Panel not ready, holding forever");
    while (true) {
          // Wait indefinitely
    }
  }
  panel.sendMessage("Beginning Setup");

  panel.sendMessage("Starting FOX LED Strips");
  // foxTowerLeds[0] = new LedStrip(FOX_TOWER_LEDS, FOX_1_LEDS_DATA_PIN);
  // foxTowerLeds[1] = new LedStrip(FOX_TOWER_LEDS, FOX_2_LEDS_DATA_PIN);

  // for (int i = 0; i < FOX_TOWER_STRIPS; i++) {
  //   foxTowerLeds[i]->begin();
  // }

  // Encoders should likely be setup last due to their use of interrupts
  panel.sendMessage("Starting Up Encoders, interrupts, and timer");
  pinMode(MOTORA_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(MOTORA_ENCODER_PIN_B, INPUT_PULLUP);
  encoderMotorA.begin();
  encoderMotorB.begin();

  encoderALastStateA = digitalRead(MOTORA_ENCODER_PIN_A);
  encoderBLastStateA = digitalRead(MOTORB_ENCODER_PIN_A);
  attachInterrupt(digitalPinToInterrupt(MOTORA_ENCODER_PIN_A), interruptUpdateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTORB_ENCODER_PIN_A), interruptUpdateEncoderB, CHANGE);

  // Setting up PID
  panel.sendMessage("Setting up PID Controllers");
  pidA.setRef(MOTORA_VEL_SETPOINT);
  pidB.setRef(MOTORB_VEL_SETPOINT);


  panel.sendMessage("Prepare Global State");
  lastUpdateTime = micros();

  // Set initial states of motors
  digitalWrite(MOTORA_CW_A_PIN, LOW);
  digitalWrite(MOTORA_CCW_B_PIN, HIGH);
  analogWrite(MOTORA_PWM_PIN, 0);

  digitalWrite(MOTORB_CW_A_PIN, LOW);
  digitalWrite(MOTORB_CCW_B_PIN, HIGH);
  analogWrite(MOTORB_PWM_PIN, 0);
  panel.sendMessage("Setup Complete");

  // Log Header
  panel.sendMessage(
    "time, motorAPosition, motorBPosition, motorAVelocity, motorBVelocity, motorACommand, motorBCommand");
}

void loop() {
  currentLoopTime = micros();
  // Serial.println(currentLoopTime);

  // LED update
  switch (ledPattern) {
  case 0:
    // EVERY_N_MILLISECONDS(LED_RAINBOW_DELAY_US / 1000) {
    //   for (int i = 0; i < FOX_TOWER_STRIPS; i++) {
    //     foxTowerLeds[i]->updateRainbow();
    //   }
    // }
    break;
  case 1:
    // EVERY_N_MILLISECONDS(LED_NEW_SPARKLE_US / 1000) {
    //   leds.updateSparkle();
    // }
    // EVERY_N_MILLISECONDS(LED_SPARKLE_DECAY_US){
    //   leds.fadeLeds(LED_SPARKLE_DECAY_AMOUNT);
    // }

    break;
  default:
    break;
  }
  
  // for (int i = 0; i < FOX_TOWER_STRIPS; i++) {
  //   foxTowerLeds[i]->showLeds();
  // }

  // EVERY_N_SECONDS(LED_ANIMATION_SWITCH_S) {
  //   ledPattern++;
  //   if (ledPattern > (LED_NUM_ANIMATIONS - 1)) {
  //     ledPattern = 0;
  //   }
  // }
 


  if (currentLoopTime - lastLoopTime > CONTROL_FUNCTION_PERIOD_US) {
    updateEncoderVelocity();

    noInterrupts();
    long currentEncoderAPosition = encoderAPosition;
    long currentEncoderBPosition = encoderBPosition;
    interrupts();

    float motorAVelocity = encoderMotorA.getVelocity();
    float pidACommand = pidA.step(motorAVelocity);
    motorA.setSpeed(pidACommand);

    float motorBVelocity = encoderMotorB.getVelocity();
    float pidBCommand = pidB.step(motorBVelocity);
    motorB.setSpeed(pidBCommand);

    // Logging
    String logMessage = String(micros()) + ", " +
              String(currentEncoderAPosition) + ", " +
              String(currentEncoderBPosition) + ", " +
              String(motorAVelocity) + ", " +
              String(motorBVelocity) + ", " +
              String(pidACommand) + ", " +
              String(pidBCommand);

    panel.sendMessage(logMessage);
    lastLoopTime = currentLoopTime;
  }

  // Handle incoming messages from the panel
  int command = panel.handleCommand();
  switch (command) {
  case 0: // End command
    motorA.setSpeed(0);
    motorB.setSpeed(0);
    
    // Wait indefinitely
    while (true) {
      // Wait indefinitely
    }
    break;

  case 1: // Pause command
    motorA.setSpeed(0);
    motorB.setSpeed(0);
    panel.sendMessage("Motors paused, waiting for resume ('R')");
    while (true) {
      while (!panel.messageAvailable()) {
      // Wait for a message from the panel to resume
      }
      String resumeMessage = panel.readMessage();
      if (resumeMessage.charAt(0) == 'R') {
      panel.sendMessage("Motors resumed");
      break;
      } else {
      panel.sendMessage("Invalid resume command received, waiting for valid command");
      }
    }
    break;

  case -2: // Panel switch fallthrough
    panel.sendMessage("Panel command fallthrough");
    break;

  default:
    // Unknown command or error
    break;
  }

  
}