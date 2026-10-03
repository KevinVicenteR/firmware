/*
  ATLAS_1.4.3.ino - main code for ATLAS series

    Created on: Mar, 2026
    Edited by Mauricio Tovar

      DRIVERS:            BTN9960LV
      NUMBER OF SENSORS:  16A-4S1C
      MICROCONTROLLER:    ATmega328P-AU
      BATTERY:            2-3S 300mAh 80C
      CONTROL:            PD
      MOTORS:             Coreless 1023 7.4V

  This code is the original firmware used for the commercial ATLAS Rev. 1.4.3
  Conceived and designed to facilitate the reading and understanding of the
  fundamental aspects of the algorithm. We appreciate any confidentiality and
  privacy provided to this code.

    ATLAS 1.4.3 © 2026 by EXOTIC TEAM MX is licensed under Creative Commons
    Attribution-NonCommercial-NoDerivatives 4.0 International. To view a copy of
    this license, visit https://creativecommons.org/licenses/by-nc-nd/4.0/

  This code is part of the ATLAS series. See more
  in the link below:

  https://exoticteam.com.mx/atlas
*/

//  =============================
//  U S E R   P A R A M E T E R S
//  =============================

#define MOTOR_L_SIGN NOT_INVERTED  // left motor direction
#define MOTOR_R_SIGN NOT_INVERTED  // right motor direction

#define KP 0.05f  // line control term proportional component coefficient
#define KD 0.35f  // line control term derivative component coefficient

#define BASE_VELOCITY_PWM 100  // velocity selection base PWM
#define MIN_VELOCITY_PWM 60    // velocity selection min PWM
#define MAX_VELOCITY_PWM 250   // velocity selection max PWM
#define VELOCITY_PWM_STEP 10   // velocity selection PWM increment/decrement step

#define IMPELLER_PWM 200  // suction impeller fan PWM value

#define RUN_VELOCITY_RAMP_TIME_MS 150  // velocity term PWM rise time in milliseconds
#define RUN_IMPELLER_RAMP_TIME_MS 250  // impeller PWM rise time in milliseconds

//  =====================================
//  A D V A N C E D   P A R A M E T E R S
//  =====================================

const bool areMotorsEnabled = true;

// TIME
#define CONTROL_LOOP_PERIOD_US 750
#define START_IMPELLER_RAMP_TIME_MS 1000
#define STOP_BRAKE_TIME_MS 500
#define OFFLINE_INNER_MOTOR_BRAKE_TIME_MS 35

// PWM
#define CONTROL_MAX_PWM_FORWARD 200
#define CONTROL_MAX_PWM_BACKWARD (-120)

#define OFFLINE_OUTTER_MOTOR_PWM CONTROL_MAX_PWM_FORWARD
#define OFFLINE_INNER_MOTOR_PWM CONTROL_MAX_PWM_BACKWARD

// OTHERS
#define SENSORS_THRESHOLD_PCT 50

//  =====================================================
//  G L O B A L   V A R I A B L E S  /  C O N S T A N T S
//  =====================================================

// sensors
#define TOTAL_SENSORS 16
byte sensorValues[TOTAL_SENSORS];
bool invertSensorReads = false;
byte ADC6_value;
byte ADC7_value;

// time
unsigned long currentTime;

void setup() {
  cli();  // disable global interrupts

  UIInit();
  sensorsInit();
  motorsInit();

  sei();  // enable global interrupts

  bootAnimation();  // boot display animation

  // debug mode trigger
  if (readButton_1() || readButton_2()) {
    confirmAnimation(500, 1);
    debugMode();
  }

  //  ===================
  //  I D L E   S T A T E
  //  ===================

  while (!readButton_1() && !readButton_2())
    ;

  invertSensorReads = readButton_2();

  calibrateSensors();
  delay(250);

  run();
}

void loop() {
}

/*
  [ WARNING ] - debugMode()

  This function uses about 5520 bytes (18%) of program storage space and 
  1270 bytes (62%) of dynamic memory.

  If more memory is needed for other applications, consider removing or
  modifying it as necessary.

*/
void debugMode() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("======================================================");
  Serial.println(" ATLAS REV 1.4.3    Copyright (C) 2026 EXOTIC TEAM MX");
  Serial.println("======================================================");
  Serial.println("[ TEST MODE ENABLED ]");

  while (1) {
    Serial.println("[0] WHEELS MOTORS TEST");
    Serial.println("[1] IMPELLER MOTOR TEST");
    Serial.println("[2] SENSORS TEST");
    Serial.println("[3] UI TEST");
    Serial.println("[4] YOUR TEST NAME");
    Serial.println("SELECT OPERATION (CONFIRM [1] / CHANGE [2])");
    Serial.println("> 0");

    uint8_t selection = 0;

    while (!readButton_1()) {
      if (readButton_2()) {
        selection = selection < 4 ? selection + 1 : 0;
        displayNumber(selection);
        Serial.println("> " + String(selection));
        delay(250);
      }
    }

    Serial.println("SELECTED> " + String(selection));

    confirmAnimation(250, 2);

    switch (selection) {
      case 0:
        Serial.println("*WHEELS MOTORS TEST*");
        delay(250);

        Serial.println("\tPRESS ANY BUTTON TO START MOTORS TEST");
        Serial.println("\t[ WARNING ] - Battery must be connected.");
        Serial.println("\t[ WARNING ] - Keep the wheels off the ground and away from anything.");

        while (!readButton_1() && !readButton_2()) {
        }
        confirmAnimation(250, 1);

        Serial.println("\t*WHEEL MOTOR TEST STARTED*");
        motorTest();
        Serial.println("\t*WHEEL MOTOR TEST ENDED*");

        delay(250);
        break;

      case 1:
        {
          Serial.println("*IMPELLER MOTOR TEST*");
          delay(250);

          Serial.println("\tPRESS ANY BUTTON TO TOGGLE IMPELLER FAN");
          Serial.println("\t[ WARNING ] - Impeller connector and battery must be connected.");
          Serial.println("\t[ WARNING ] - Place the robot horizontally and on a completely clear flat surface.");

          bool imp_flag = false;


          Serial.println("\tIMP OFF");

          // impeller test infinite loop
          while (1) {
            if (readButton_2()) {
              if (!imp_flag) {
                Serial.println("\tIMP ON");
                imp_flag = true;
                impellerRamp();
                setLED_2(1);
              } else {
                Serial.println("\tIMP OFF");
                imp_flag = false;
                setPWM_Impeller(0);
                setLED_2(0);
                delay(250);
              }
            }
          }
        }
        break;

      case 2:
        {
          Serial.println("*SENSORS TEST*");
          delay(250);

          // sensors test mode selector
          Serial.println("\tSELECT MODE> CALIBRATED [SW1] / RAW [SW2]");
          while (!readButton_1() && !readButton_2()) {
          }
          const bool calibratedMode = readButton_1();
          Serial.println(String("\t") + (calibratedMode ? "*CALIBRATED*" : "*RAW*"));
          confirmAnimation(250, 1);

          // sensors reading mode selector
          Serial.println("\tSELECT READING MODE> NOT INVERTED [SW1] / INVERTED [SW2]");
          while (!readButton_1() && !readButton_2())
            ;
          invertSensorReads = readButton_2();
          Serial.println(String("\t") + (invertSensorReads ? "*INVERTED*" : "NOT INVERTED*"));
          confirmAnimation(250, 1);

          if (calibratedMode) {
            Serial.println("\tPRESS ANY BUTTON TO START SENSORS CALIBRATION");
            while (!readButton_1() && !readButton_2())
              ;
            Serial.println("\tPRESS ANY BUTTON TO STOP SENSORS CALIBRATION");
            calibrateSensors();
            printCalibrationValues();

            Serial.println("\tPRESS ANY BUTTON TO START TEST");
            Serial.println("\t[ WHILE READING TEST PRESS ANY BUTTON TO SHOW RAW VALUES ]");
            while (!readButton_1() && !readButton_2())
              ;
            delay(250);
          }

          // sensors test infinite loop
          while (1) {
            if (calibratedMode && !readButton_1() && !readButton_2()) {
              readCalibratedSensors();
            } else {
              readRawSensors();
            }

            printSensorValues();
            delayMicroseconds(CONTROL_LOOP_PERIOD_US);
          }
        }
        break;

      case 3:
        Serial.println("*UI TEST*");
        delay(250);

        Serial.println("\t- LED0 [SW1]");
        Serial.println("\t- LED1 [SW2]");
        Serial.println("\t- LED2 [SW1 + SW2]");
        Serial.println("\t PRESS ANY BUTTON TO START TEST");

        while (!readButton_1() && !readButton_2())
          ;
        confirmAnimation(250, 1);

        // user interface test infinite loop
        while (1) {
          setLED_0(readButton_1());
          setLED_1(readButton_2());
          setLED_2(readButton_1() && readButton_2());

          Serial.print("SW1:" + String(readButton_1()) + "   ");
          Serial.print("SW2:" + String(readButton_2()) + "   ");

          Serial.print("READY:" + String(readReady()) + "   ");
          Serial.println("GO:" + String(readGo()));
        }
        break;

      case 4:
        Serial.println("*YOUR TEST NAME*");
        delay(250);

        // place your test code here
        // (do not exceed 80% of the dynamic memory limit)

        delay(250);
        break;

      default:
        Serial.println("*NO OPTION AVAILABLE*");
        break;
    }
  }
}