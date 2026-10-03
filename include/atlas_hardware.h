#pragma once
#include <Arduino.h>

// Conexion ATLAS Rev. 1.4.3 obtenida de los modulos del fabricante.
// Referencia: Mauricio Tovar / EXOTIC TEAM MX, archivos .ino de ATLAS_1 (2026).
// Los archivos originales se conservan con sus avisos y licencia.
//   SW1 A5 (PC5) / SW2 D7 (PD7), activos bajos con pull-up
//   READY D3 (PD3) / GO D4 (PD4), activos altos
//   LED0 D2 (PD2) / LED1 D8 (PB0) / LED2 D13 (PB5)
//   INH D12 (PB4); motor izq. D6 (PD6) + D9 (OC1A); motor der. D5 (PD5) + D10 (OC1B)
//   Succion D11 (OC2A); multiplexor A0..A3 (PC0..PC3), lectura A4 (ADC4)
constexpr bool HARDWARE_VERIFIED = true; // Mapa conocido; pendiente prueba fisica.
constexpr uint8_t SENSOR_COUNT = 16;
namespace Hardware {
constexpr uint16_t MOTOR_TOP = 399; // Timer1 fase correcta: 16 MHz / (2*399) = 20 kHz.
constexpr bool BLACK_IS_HIGH = false;

inline void begin() {
  const uint8_t sreg = SREG;
  cli();
  // Inhibir drivers y poner salidas de motor en bajo antes de configurar PWM.
  PORTB &= ~_BV(PB4); DDRB |= _BV(PB4);
  PORTD &= ~(_BV(PD5) | _BV(PD6)); DDRD |= _BV(PD5) | _BV(PD6);
  PORTB &= ~(_BV(PB1) | _BV(PB2) | _BV(PB3)); DDRB |= _BV(PB1) | _BV(PB2) | _BV(PB3);
  // Timer1: PWM fase correcta, TOP=ICR1, sin prescaler (como el original).
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = MOTOR_TOP; OCR1A = 0; OCR1B = 0;
  // Timer2: succion en D11, ~31 kHz. Timer0 se conserva para millis/micros.
  TCCR2A = _BV(COM2A1) | _BV(WGM20);
  TCCR2B = _BV(CS20); OCR2A = 0;
  // LEDs, botones con pull-up y entradas del modulo de arranque.
  PORTD &= ~_BV(PD2); DDRD |= _BV(PD2);
  PORTB &= ~(_BV(PB0) | _BV(PB5)); DDRB |= _BV(PB0) | _BV(PB5);
  DDRC &= ~_BV(PC5); PORTC |= _BV(PC5);
  DDRD &= ~(_BV(PD3) | _BV(PD4) | _BV(PD7));
  PORTD = (PORTD & ~(_BV(PD3) | _BV(PD4))) | _BV(PD7);
  // A0..A3: direccion multiplexor. A4: entrada analogica.
  DDRC |= 0x0F; PORTC &= ~0x0F;
  DDRC &= ~_BV(PC4); PORTC &= ~_BV(PC4);
  DIDR0 |= _BV(ADC4D);
  // ADC de 8 bits (ADLAR), AVcc, reloj /16 = 1 MHz como el original: ~15 us por canal.
  ADMUX = _BV(REFS0) | _BV(ADLAR) | 4;
  ADCSRA = _BV(ADEN) | _BV(ADPS2);
  SREG = sreg;
}
inline bool sw1() { return !(PINC & _BV(PINC5)); }
inline bool sw2() { return !(PIND & _BV(PIND7)); }
inline bool ready() { return PIND & _BV(PIND3); }
inline bool go() { return PIND & _BV(PIND4); }
inline void leds(bool a, bool b, bool c) {
  PORTD = (PORTD & ~_BV(PD2)) | (a ? _BV(PD2) : 0);
  PORTB = (PORTB & ~(_BV(PB0) | _BV(PB5))) | (b ? _BV(PB0) : 0) | (c ? _BV(PB5) : 0);
}
inline uint16_t pulse(int16_t value) {
  auto magnitude = static_cast<uint16_t>(value < 0 ? -value : value);
  if (magnitude > 255) magnitude = 255;
  return static_cast<uint16_t>(uint32_t(magnitude) * MOTOR_TOP / 255);
}
// PWM con signo -255..255; positivo = avance. Habilita los drivers si estaban inhibidos.
inline void motors(int16_t left, int16_t right) {
  const uint16_t l = pulse(left);
  const uint16_t r = pulse(right);
  if (left < 0) { PORTD |= _BV(PD6); OCR1A = MOTOR_TOP - l; }
  else { PORTD &= ~_BV(PD6); OCR1A = l; }
  // El motor derecho tiene polaridad electrica opuesta al izquierdo.
  if (right > 0) { PORTD |= _BV(PD5); OCR1B = MOTOR_TOP - r; }
  else { PORTD &= ~_BV(PD5); OCR1B = r; }
  if (!(PORTB & _BV(PB4))) {
    PORTB |= _BV(PB4); delayMicroseconds(5); // t_wakeup BTN9960LV.
  }
}
// Salidas a cero e inhibicion de drivers.
inline void disable() {
  OCR1A = 0; OCR1B = 0;
  PORTD &= ~(_BV(PD5) | _BV(PD6));
  PORTB &= ~_BV(PB4);
}
inline void suction(uint8_t pwm) { OCR2A = pwm; }
inline void readSensors(uint8_t (&values)[SENSOR_COUNT]) {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    PORTC = (PORTC & 0xF0) | i; // Preservar A4/A5 (pull-up del boton).
    delayMicroseconds(2);       // Asentamiento del multiplexor.
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {
      // Esperar fin de conversion (~13 us).
    }
    values[i] = ADCH;
  }
}
}
