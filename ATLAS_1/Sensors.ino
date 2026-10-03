/*
  Sensors.ino - module code for ATLAS series

    Created on: Mar, 2026
    Edited by Mauricio Tovar
    
    ATLAS 1.4.3 © 2026 by EXOTIC TEAM MX is licensed under Creative Commons
    Attribution-NonCommercial-NoDerivatives 4.0 International. To view a copy of
    this license, visit https://creativecommons.org/licenses/by-nc-nd/4.0/

  This code is part of the ATLAS_1.4.3.ino original firmware.
*/

#define ADC_CHANNEL_4 0b0100
#define ADC_CHANNEL_6 0b0110
#define ADC_CHANNEL_7 0b0111
#define LINE_SENSORS_ADC_CHANNEL ADC_CHANNEL_4

#define MUX_SWITCH_MASK 0xF
#define ADC_ADMUX_CONFIG_MASK 0xE0
#define ADC_CHANNEL_MASK 0xF
#define ADC_PRESCALER_ADPS_MASK 0x7
#define ADC_MAX_VALUE 0xFF

// calibration parameters
#define CALIBRATION_AVG_SAMPLES 8
#define CALIBRATION_COLLECTOR_SAMPLES 10

byte maxSensorValues[TOTAL_SENSORS];
byte minSensorValues[TOTAL_SENSORS];
byte sensorThreshold[TOTAL_SENSORS];

void sensorsInit() {
  // set sensor MUX switch pins to output
  DDRC |= MUX_SWITCH_MASK;

  // set AVcc as ADC voltage reference and right adjusted bit
  ADMUX = (ADMUX & ~ADC_ADMUX_CONFIG_MASK) | (1 << REFS0) | (1 << ADLAR);

  // set ADC prescaler (/16)
  ADCSRA = (ADCSRA & ~ADC_PRESCALER_ADPS_MASK) | (1 << ADPS2);

  // set ADC channel to line sensors
  setADCChannel(LINE_SENSORS_ADC_CHANNEL);
}

void setADCChannel(byte channel) {
  ADMUX = (ADMUX & ~ADC_CHANNEL_MASK) | channel;
}

byte readADC() {
  ADCSRA |= (1 << ADEN) | (1 << ADSC);  // enable ADC and start conversion
  while (bit_is_set(ADCSRA, ADSC))      // wait for conversion
    ;
  ADCSRA &= ~(1 << ADEN);  // disable ADC

  return ADCH;  // return ADC conversion
}

void readRawSensors() {
  // line sensors reading loop
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    // set sensors MUX switch output configuration
    PORTC = (PORTC & ~MUX_SWITCH_MASK) | i;

    const byte ADCValue = readADC();

    sensorValues[i] = invertSensorReads ? ADC_MAX_VALUE - ADCValue : ADCValue;
  }
}

void readSampledSensors() {
  unsigned int sumSensorValues[TOTAL_SENSORS];  // temporal sum of each sensor samples

  // set sum values to zero
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    sumSensorValues[i] = 0;
  }

  for (byte j = 0; j < CALIBRATION_AVG_SAMPLES; j++) {
    // update raw sensor values
    readRawSensors();

    // add readings to sum of each sensor
    for (byte i = 0; i < TOTAL_SENSORS; i++) {
      sumSensorValues[i] += sensorValues[i];
    }
  }

  // update sensor values with sum average
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    sensorValues[i] = sumSensorValues[i] / CALIBRATION_AVG_SAMPLES;
  }
}

void updateMaxMinSensorValues() {
  byte highestSample[TOTAL_SENSORS];  // temporal highest collector sample for each sensor
  byte lowestSample[TOTAL_SENSORS];   // temporal lowest collector sample for each sensor

  // samples collector
  for (byte j = 0; j < CALIBRATION_COLLECTOR_SAMPLES; j++) {
    readSampledSensors();

    for (byte i = 0; i < TOTAL_SENSORS; i++) {
      // update maximum collected value
      if ((j == 0) || (sensorValues[i] > highestSample[i])) {
        highestSample[i] = sensorValues[i];
      }

      // update minimum collected value
      if ((j == 0) || (sensorValues[i] < lowestSample[i])) {
        lowestSample[i] = sensorValues[i];
      }
    }
  }

  // update the min and max values
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    // update maximum only if the min of all collected readings was still higher than it
    // (we got all collected readings in a row higher than the existing maximum)
    if (lowestSample[i] > maxSensorValues[i]) {
      maxSensorValues[i] = lowestSample[i];
    }

    // update minimum only if the max of all collected readings was still lower than it
    // (we got all collected readings in a row lower than the existing minimum)
    if (highestSample[i] < minSensorValues[i]) {
      minSensorValues[i] = highestSample[i];
    }
  }
}

void resetCalibrationValues() {
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    maxSensorValues[i] = 0;
    minSensorValues[i] = ADC_MAX_VALUE;
  }
}

void calibrateSensors() {
  setLED_2(1);
  delay(250);
  setLED_2(0);
  delay(250);

  resetCalibrationValues();

  bool led_flag = true;

  //  end calibration trigger
  while (!readButton_1() && !readButton_2()) {
    setLED_0(invertSensorReads ? led_flag : 0);
    setLED_2(invertSensorReads ? 0 : led_flag);
    setLED_1(invertSensorReads ? led_flag : 0);
    led_flag = !led_flag;

    updateMaxMinSensorValues();
  }

  getSensorThreshold();
  setLEDS(0);
  delay(250);

  confirmAnimation(250, 2);
}

void getSensorThreshold() {
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    sensorThreshold[i] = (maxSensorValues[i] - minSensorValues[i]) * ((float)(SENSORS_THRESHOLD_PCT) / 100) + minSensorValues[i];
  }
}

void readCalibratedSensors() {
  // update actual raw sensor readings
  readRawSensors();

  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    byte range = maxSensorValues[i] - sensorThreshold[i];

    if (sensorValues[i] > sensorThreshold[i]) {
      if (sensorValues[i] < maxSensorValues[i]) {
        // scale value to fixed range if raw sensor value is higher than threshold and lower than maximum
        sensorValues[i] = ((uint32_t)(sensorValues[i] - sensorThreshold[i]) * ADC_MAX_VALUE) / range;
      } else {
        sensorValues[i] = ADC_MAX_VALUE;  // set to 255 if raw sensor value is higher than maximum calibrated value
      }
    } else {
      sensorValues[i] = 0;  // set to zero if raw sensor value is lower than threshold
    }
  }
}

unsigned int getLinePosition() {
  uint32_t wtd = 0;  // temporal sum of weighted sensor values
  uint32_t sum = 0;  // temporal sum of sensor values
  isOnLine = false;  // set line is not on sensors range at the start of line position calculation

  // get calibrated sensor read values
  readCalibratedSensors();

  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    // add to sum if actual sensor value is higher to zero
    if (sensorValues[i]) {
      isOnLine = true;  // updates to true if at least one sensor detects the line
      wtd += (uint32_t)sensorValues[i] * i * 1000;
      sum += sensorValues[i];
    }
  }

  // if weighted sum is higher to zero return weighted average
  return wtd ? wtd / sum : 0;
}

void printCalibrationValues() {
  Serial.print("MAX:\t");
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    Serial.print(maxSensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.print("MIN:\t");
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    Serial.print(minSensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
  Serial.print("TH:\t");
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    Serial.print(sensorThreshold[i]);
    Serial.print("\t");
  }
  Serial.println();
}

void readOtherADCs() {
  setADCChannel(ADC_CHANNEL_6);
  ADC6_value = readADC();

  setADCChannel(ADC_CHANNEL_7);
  ADC7_value = readADC();

  setADCChannel(LINE_SENSORS_ADC_CHANNEL);
}

//  ======================
//  D E B U G   T O O L S
//  ======================

void printSensorValues() {
  for (byte i = 0; i < TOTAL_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    Serial.print("\t");
  }
  Serial.println();
}