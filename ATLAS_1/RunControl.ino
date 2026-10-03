/*
  RunController.ino - module code for ATLAS series

    Created on: Mar, 2026
    Edited by Mauricio Tovar
    
    ATLAS 1.4.3 © 2026 by EXOTIC TEAM MX is licensed under Creative Commons
    Attribution-NonCommercial-NoDerivatives 4.0 International. To view a copy of
    this license, visit https://creativecommons.org/licenses/by-nc-nd/4.0/

  This code is part of the ATLAS_1.4.3.ino original firmware.
*/

#define LEFT 0
#define RIGHT 1

// run-state
bool isRunning;

// run time
unsigned long previousLooptime_us;
unsigned long startTime;
unsigned long elapsedTime;

// line control
int previousError;
bool lastDetectedSide;
bool isOnLine;
unsigned long offlineStartTime;

// PWM
int velocityPWM = BASE_VELOCITY_PWM;
int velPWMDecrement;
int impPWMDecrement;
int outputPWML;
int outputPWMR;
int outputPWMImp;

// stop phase
bool stopBrake_flag = false;
unsigned long stopBrakeStartTime;

void runInit() {
  enableMotorDrivers();

  previousError = 0;

  velPWMDecrement = velocityPWM;
  impPWMDecrement = IMPELLER_PWM;

  // start timming
  elapsedTime = 0;
  startTime = millis();
  currentTime = startTime;
  offlineStartTime = startTime;

  previousLooptime_us = micros();
}

void updatePWMDecrementRamps() {
  // stop velocity PWM brake phase
  if (stopBrake_flag) {
    if (currentTime - stopBrakeStartTime >= STOP_BRAKE_TIME_MS) {
      isRunning = false;
    } else {
      velPWMDecrement = velocityPWM * (currentTime - stopBrakeStartTime) / STOP_BRAKE_TIME_MS;
    }
  }

  // velocity PWM rise phase
  if (!stopBrake_flag && velPWMDecrement) {
    if (elapsedTime >= RUN_VELOCITY_RAMP_TIME_MS) {
      velPWMDecrement = 0;
    } else velPWMDecrement = velocityPWM - (velocityPWM * elapsedTime / RUN_VELOCITY_RAMP_TIME_MS);
  }

  // impeller PWM rise phase
  if (impPWMDecrement) {
    if (elapsedTime >= RUN_IMPELLER_RAMP_TIME_MS) {
      impPWMDecrement = 0;
    } else impPWMDecrement = IMPELLER_PWM - (IMPELLER_PWM * elapsedTime / RUN_IMPELLER_RAMP_TIME_MS);
  }
}

void run() {
  //  =========================
  //  R E A D Y    T O    R U N
  //  =========================

  int impRampDuty = 0;

  startTime = millis();  // (only for impeller ramp)

  //  start trigger
  while (!readGo()) {
    // impeller ready control
    if (readReady()) {
      // get ready elapsed time
      elapsedTime = millis() - startTime;

      // get impeller pwm ramp
      if (elapsedTime >= START_IMPELLER_RAMP_TIME_MS) {
        impRampDuty = IMPELLER_PWM;
        setLED_2(1);
      } else {
        impRampDuty = IMPELLER_PWM * elapsedTime / START_IMPELLER_RAMP_TIME_MS;
      }

      // set impeller
      setPWM_Impeller(impRampDuty);

    } else {
      startTime = millis();
      impRampDuty = 0;
      setLED_2(0);
      setPWM_Impeller(0);
    }

    // velocity PWM selector
    if (readButton_1() && velocityPWM >= MIN_VELOCITY_PWM + VELOCITY_PWM_STEP) {
      setLED_0(1);
      delay(250);
      setLED_0(0);
      delay(250);
      velocityPWM -= VELOCITY_PWM_STEP;
    } else if (readButton_2() && velocityPWM <= MAX_VELOCITY_PWM - VELOCITY_PWM_STEP) {
      setLED_1(1);
      delay(250);
      setLED_1(0);
      delay(250);
      velocityPWM += VELOCITY_PWM_STEP;
    }
  }

  setPWM_Impeller(0);
  setLEDS(1);

  isRunning = true;

  runInit();

  while (isRunning) {
    if (micros() - previousLooptime_us >= CONTROL_LOOP_PERIOD_US) {
      previousLooptime_us = micros();

      // update time measurements
      currentTime = millis();
      elapsedTime = currentTime - startTime;

      updatePeriod();
    }

    // stop trigger
    if (!readGo() && !stopBrake_flag) {
      setLEDS(0);
      stopBrakeStartTime = currentTime;
      stopBrake_flag = true;
    }
  }

  // end of run
  disableRobot();
}

void updatePeriod() {

  updatePWMDecrementRamps();

  // get line position
  const unsigned int position = getLinePosition();

  if (isOnLine && position && position != (TOTAL_SENSORS - 1) * 1000) {

    // check and update last detected side
    if (sensorValues[0] || sensorValues[1]) lastDetectedSide = LEFT;
    if (sensorValues[TOTAL_SENSORS - 2] || sensorValues[TOTAL_SENSORS - 1]) lastDetectedSide = RIGHT;

    // get line error
    const int error = position - (TOTAL_SENSORS - 1) * 1000 / 2;

    // get line delta error
    const int deltaError = error - previousError;
    previousError = error;  //  update previous error

    // compute and clamp line control term PWM value
    long linePWM = ((float)(error)*KP) + ((float)(deltaError)*KD);

    if (linePWM > 1000) linePWM = 1000;
    else if (linePWM < -1000) linePWM = -1000;

    // get motor PWM values
    outputPWML = velocityPWM - velPWMDecrement + linePWM;
    outputPWMR = velocityPWM - velPWMDecrement - linePWM;
    outputPWMImp = IMPELLER_PWM - impPWMDecrement;

    // update last time online
    offlineStartTime = currentTime;

  } else {
    //  ===============================
    //    O F F L I N E   M E T H O D
    //  ===============================

    // get offline elapsed time
    const unsigned long offlineElapsedTime = currentTime - offlineElapsedTime;

    // set motors PWM values in function of last detected side before out
    if (lastDetectedSide == LEFT) {
      outputPWML = offlineElapsedTime <= OFFLINE_INNER_MOTOR_BRAKE_TIME_MS ? OFFLINE_INNER_MOTOR_PWM : 0;
      outputPWMR = OFFLINE_OUTTER_MOTOR_PWM;
    } else {
      outputPWML = OFFLINE_OUTTER_MOTOR_PWM;
      outputPWMR = offlineElapsedTime <= OFFLINE_INNER_MOTOR_BRAKE_TIME_MS ? OFFLINE_INNER_MOTOR_PWM : 0;
    }
  }

  // clamp motors PWM values
  if (outputPWML > CONTROL_MAX_PWM_FORWARD) outputPWML = CONTROL_MAX_PWM_FORWARD;
  else if (outputPWML < CONTROL_MAX_PWM_BACKWARD) outputPWML = CONTROL_MAX_PWM_BACKWARD;

  if (outputPWMR > CONTROL_MAX_PWM_FORWARD) outputPWMR = CONTROL_MAX_PWM_FORWARD;
  else if (outputPWMR < CONTROL_MAX_PWM_BACKWARD) outputPWMR = CONTROL_MAX_PWM_BACKWARD;

  //  update motors PWM
  setPWM_MotorL(outputPWML);
  setPWM_MotorR(outputPWMR);
  setPWM_Impeller(outputPWMImp);
}

void disableRobot() {
  // motors off
  setPWM_MotorL(0);
  setPWM_MotorR(0);
  setPWM_Impeller(0);
  disableMotorDrivers();

  delay(2000);

  // infinite standby loop
  while (1) {
    setLED_0(1);
    setLED_1(1);
    delay(50);
    setLED_0(0);
    setLED_1(0);
    delay(1000);
  }
}

void impellerRamp() {
  int rampPWM = 0;
  startTime = millis();  // impeller PWM ramp start time

  while (rampPWM != IMPELLER_PWM) {
    currentTime = millis();

    if (currentTime - startTime >= START_IMPELLER_RAMP_TIME_MS) {
      rampPWM = IMPELLER_PWM;
    } else {
      rampPWM = IMPELLER_PWM * (currentTime - startTime) / START_IMPELLER_RAMP_TIME_MS;
    }

    setPWM_Impeller(rampPWM);
  }
}