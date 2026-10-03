/*
  UI.ino - module code for ATLAS series

    Created on: Mar, 2026
    Edited by Mauricio Tovar
    
    ATLAS 1.4.3 © 2026 by EXOTIC TEAM MX is licensed under Creative Commons
    Attribution-NonCommercial-NoDerivatives 4.0 International. To view a copy of
    this license, visit https://creativecommons.org/licenses/by-nc-nd/4.0/

  This code is part of the ATLAS_1.4.3.ino original firmware.
*/

void UIInit() {
  DDRD |= 1 << DDD2;     //  set PD2 as LED_0 digital output
  DDRB |= 1 << DDB0;     //  set PB0 as LED_1 digital output
  DDRB |= 1 << DDB5;     //  set PB5 as LED_2 digital output
  PORTC |= 1 << PORTC5;  //  enable PC5 as button_1 pull-up input
  PORTD |= 1 << PORTD7;  //  enable PD7 as button_2 pull-up input
}

bool readButton_1() {
  return !(PINC & (1 << PINC5));
}

bool readButton_2() {
  return !(PIND & (1 << PIND7));
}

bool readReady() {
  return PIND & (1 << PIND3);
}

bool readGo() {
  return PIND & (1 << PIND4);
}

void setLED_0(bool _state) {
  PORTD = (PORTD & ~(1 << PORTD2)) | (_state << PORTD2);
}

void setLED_1(bool _state) {
  PORTB = (PORTB & ~(1 << PORTB0)) | (_state << PORTB0);
}

void setLED_2(bool _state) {
  PORTB = (PORTB & ~(1 << PORTB5)) | (_state << PORTB5);
}

void setLEDS(bool _state) {
  PORTD = (PORTD & ~(1 << PORTD2)) | (_state << PORTD2);
  PORTB = (PORTB & ~(1 << PORTB0)) | (_state << PORTB0);
  PORTB = (PORTB & ~(1 << PORTB5)) | (_state << PORTB5);
}

void displayNumber(uint8_t number) {
  uint8_t config = number > 7 ? 0 : number;

  setLED_0(config & 0b100);
  setLED_2(config & 0b10);
  setLED_1(config & 0b1);
}

void toggleLED_2() {
  PINB |= 1 << PORTB5;
}

void confirmAnimation(int _time, byte _cycles) {
  for (byte i = 0; i < _cycles; i++) {
    setLEDS(1);
    delay(_time);
    setLEDS(0);
    delay(_time);
  }
}

void bootAnimation() {
  for (byte i = 0; i < 20; i++) {
    toggleLED_2();
    delay(25);
  }
}