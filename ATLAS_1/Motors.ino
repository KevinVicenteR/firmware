/*
  Motors.ino - module code for ATLAS series

    Created on: Mar, 2026
    Edited by Mauricio Tovar
    
    ATLAS 1.4.3 © 2026 by EXOTIC TEAM MX is licensed under Creative Commons
    Attribution-NonCommercial-NoDerivatives 4.0 International. To view a copy of
    this license, visit https://creativecommons.org/licenses/by-nc-nd/4.0/

  This code is part of the ATLAS_1.4.3.ino original firmware.
*/

#define INH 12
#define INL1 6
#define INL2 9
#define INR1 5
#define INR2 10
#define PWMC 11

#define T_WAKEUP_US 5
#define T_STBY_US 300

#define NOT_INVERTED 1
#define INVERTED (-1)

#define MOTORS_PWM_PULSE_MAX (400 - 1)
#define MOTORS_PWM_RESOLUTION 255

#define IMP_PWM_PULSE_MAX 0xFF
#define IMP_PWM_RESOLUTION 0xFF

void motorsInit() {
  // motor drivers inhibit
  pinMode(INH, OUTPUT);

  // motor A driver
  pinMode(INL1, OUTPUT);
  pinMode(INL2, OUTPUT);

  // motor B driver
  pinMode(INR1, OUTPUT);
  pinMode(INR2, OUTPUT);

  // impeller
  pinMode(PWMC, OUTPUT);

  // digital outputs
  digitalWrite(INH, 0);
  digitalWrite(INL1, 0);
  digitalWrite(INR1, 0);

  // Timer 1 configuration
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);

  ICR1 = MOTORS_PWM_PULSE_MAX;
  OCR1A = 0;
  OCR1B = 0;

  // Timer 2 configuration
  TCCR2A = _BV(COM2A1) | _BV(WGM20);
  TCCR2B = _BV(CS20);

  OCR2A = 0;
}

void enableMotorDrivers() {
  digitalWrite(INH, 1);
  delayMicroseconds(T_WAKEUP_US);
}

void disableMotorDrivers() {
  digitalWrite(INH, 0);
  digitalWrite(INL1, 0);
  digitalWrite(INR1, 0);
  OCR1A = 0;
  OCR1B = 0;
  delayMicroseconds(T_STBY_US);
}

void setPWM_Impeller(int val) {
  if (!areMotorsEnabled) return;

  uint16_t pulse = (uint32_t)abs(val) * IMP_PWM_PULSE_MAX / IMP_PWM_RESOLUTION;
  if (pulse > IMP_PWM_PULSE_MAX) pulse = IMP_PWM_PULSE_MAX;

  OCR2A = pulse;
}

void setPWM_MotorL(int val) {
  if (!areMotorsEnabled) return;

  uint16_t pulse = (uint32_t)abs(val) * MOTORS_PWM_PULSE_MAX / MOTORS_PWM_RESOLUTION;
  if (pulse > MOTORS_PWM_PULSE_MAX) pulse = MOTORS_PWM_PULSE_MAX;

  if (val * MOTOR_L_SIGN > 0) {
    digitalWrite(INL1, 0);
    OCR1A = pulse;
  } else if (val * MOTOR_L_SIGN < 0) {
    digitalWrite(INL1, 1);
    OCR1A = MOTORS_PWM_PULSE_MAX - pulse;
  } else {
    digitalWrite(INL1, 0);
    OCR1A = 0;
  }
}

void setPWM_MotorR(int val) {
  if (!areMotorsEnabled) return;

  uint16_t pulse = (uint32_t)abs(val) * MOTORS_PWM_PULSE_MAX / MOTORS_PWM_RESOLUTION;
  if (pulse > MOTORS_PWM_PULSE_MAX) pulse = MOTORS_PWM_PULSE_MAX;

  if (val * MOTOR_R_SIGN > 0) {
    digitalWrite(INR1, 1);
    OCR1B = MOTORS_PWM_PULSE_MAX - pulse;
  } else if (val * MOTOR_R_SIGN < 0) {
    digitalWrite(INR1, 0);
    OCR1B = pulse;
  } else {
    digitalWrite(INR1, 0);
    OCR1B = 0;
  }
}

void motorTest() {
  delay(500);

  int ON_TIME_MS = 400;
  int OFF_TIME_MS = 1000;
  int MOTOR_PWM_TEST = 60;

  enableMotorDrivers();

  // motor A
  setLED_0(1);
  setPWM_MotorL(MOTOR_PWM_TEST);
  delay(ON_TIME_MS);
  setLED_0(0);
  setPWM_MotorL(0);
  delay(OFF_TIME_MS);
  setLED_0(1);
  setPWM_MotorL(-MOTOR_PWM_TEST);
  delay(ON_TIME_MS);
  setLED_0(0);
  setPWM_MotorL(0);
  delay(OFF_TIME_MS);

  // motor B
  setLED_1(1);
  setPWM_MotorR(MOTOR_PWM_TEST);
  delay(ON_TIME_MS);
  setLED_1(0);
  setPWM_MotorR(0);
  delay(OFF_TIME_MS);
  setLED_1(1);
  setPWM_MotorR(-MOTOR_PWM_TEST);
  delay(ON_TIME_MS);
  setLED_1(0);
  setPWM_MotorR(0);
  delay(OFF_TIME_MS);

  disableMotorDrivers();
}