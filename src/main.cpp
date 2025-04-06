#include <Arduino.h>
#include <ArduinoLog.h>
#include <Encoder.h>
#include <Constants.h>
#include <TimerOne.h>
#include <Motor.h>
#include <PID.h>
#include <Led_strip.h>

// TimerOne timer;
Motor motorA(MOTORA_PWM_PIN, MOTORA_CW_A_PIN, MOTORA_CCW_B_PIN);
Motor motorB(MOTORB_PWM_PIN, MOTORB_CW_A_PIN, MOTORB_CCW_B_PIN);

LedStrip* foxTowerLeds[4];

int ledPattern = 0;

PID pidA(MOTORA_KP, MOTORA_KI, MOTORA_KD);
PID pidB(MOTORB_KP, MOTORB_KI, MOTORB_KD);

// Encoders and state
volatile uint32_t lastUpdateTime = NULL;
volatile uint32_t currentUpdateTime = NULL;

// This is zero, so we will almost certainly immediately run motor update
uint32_t lastLoopTime = 0;

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
  Serial.begin(9600);
  Serial.println("Beginning Setup");

  Serial.println("Starting FOX LED Strips");
  foxTowerLeds[0] = new LedStrip(FOX_TOWER_LEDS, FOX_1_LEDS_DATA_PIN);
  foxTowerLeds[1] = new LedStrip(FOX_TOWER_LEDS, FOX_2_LEDS_DATA_PIN);
  foxTowerLeds[2] = new LedStrip(FOX_TOWER_LEDS, FOX_3_LEDS_DATA_PIN);
  foxTowerLeds[3] = new LedStrip(FOX_TOWER_LEDS, FOX_4_LEDS_DATA_PIN);

  for (int i = 0; i < 4; i++) {
    foxTowerLeds[i]->begin();
  }

  // Encoders should likely be setup last due to their use of interrupts
  Serial.println("Starting Up Encoders, interrupts, and timer");
  pinMode(MOTORA_ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(MOTORA_ENCODER_PIN_B, INPUT_PULLUP);
  encoderMotorA.begin();
  encoderMotorB.begin();

  encoderALastStateA = digitalRead(MOTORA_ENCODER_PIN_A);
  encoderBLastStateA = digitalRead(MOTORB_ENCODER_PIN_A);
  attachInterrupt(digitalPinToInterrupt(MOTORA_ENCODER_PIN_A), interruptUpdateEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTORB_ENCODER_PIN_A), interruptUpdateEncoderB, CHANGE);

  Serial.println("Prepare Global State");
  lastUpdateTime = micros();

  // Set initial states of motors
  digitalWrite(MOTORA_CW_A_PIN, LOW);
  digitalWrite(MOTORA_CCW_B_PIN, HIGH);
  analogWrite(MOTORA_PWM_PIN, 0);

  digitalWrite(MOTORB_CW_A_PIN, LOW);
  digitalWrite(MOTORB_CCW_B_PIN, HIGH);
  analogWrite(MOTORB_PWM_PIN, 0);
  Serial.println("Setup Complete");

  // Log Header
  Serial.println(
    "time, motorAPosition, motorBPosition, motorAVelocity, motorBVelocity, motorACommand, motorBCommand");
}

void loop() {
  unsigned long currentLoopTime = micros();

  // LED update
  switch (ledPattern) {
  case 0:
    EVERY_N_MILLISECONDS(LED_RAINBOW_DELAY_US / 1000) {
      for (int i = 0; i < 4; i++) {
        foxTowerLeds[i]->updateRainbow();
      }
    }
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
  
  for (int i = 0; i < 4; i++) {
    foxTowerLeds[i]->showLeds();
  }

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
    Serial.print(micros());
    Serial.print(", ");
    Serial.print(currentEncoderAPosition);
    Serial.print(", ");
    Serial.print(currentEncoderBPosition);
    Serial.print(", ");
    Serial.print(motorAVelocity);
    Serial.print(", ");
    Serial.print(motorBVelocity);
    Serial.print(", ");
    Serial.print(pidACommand);
    Serial.print(", ");
    Serial.print(pidBCommand);
    Serial.println("");
  }

  lastLoopTime = currentLoopTime;
}