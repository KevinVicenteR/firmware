"""Pruebas del control con entradas simuladas; no prueban electronica ni traccion."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
arduino = r'''
#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <string>
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define constrain(x,a,b) ((x)<(a)?(a):((x)>(b)?(b):(x)))
struct __FlashStringHelper {};
#define F(x) reinterpret_cast<const __FlashStringHelper*>(x)
uint32_t mockUs = 0;
uint32_t micros() { return mockUs; }
uint32_t millis() { return mockUs / 1000; }
void delay(uint32_t ms) { mockUs += ms * 1000; }
void delayMicroseconds(uint32_t us) { mockUs += us; }
struct Console {
 std::string input, output;
 void begin(long) {}
 int available() { return input.size(); }
 int read() { char c = input[0]; input.erase(0, 1); return c; }
 void print(const __FlashStringHelper *s) { output += reinterpret_cast<const char*>(s); }
 void println(const __FlashStringHelper *s) { print(s); output += '\n'; }
 void print(char c) { output += c; }
 template<class T> void print(T v) { output += std::to_string(v); }
 template<class T> void print(T v, int) { output += std::to_string(v); }
 template<class T> void println(T v) { print(v); output += '\n'; }
 template<class T> void println(T v, int) { print(v); output += '\n'; }
} Serial;
'''
eeprom = r'''
#pragma once
#include <string.h>
struct EEPROMClass {
 uint8_t data[1024];
 EEPROMClass() { memset(data, 0xFF, sizeof data); }
 template<class T> T &get(int a, T &t) { memcpy(&t, data + a, sizeof t); return t; }
 template<class T> const T &put(int a, const T &t) { memcpy(data + a, &t, sizeof t); return t; }
} EEPROM;
'''
hardware = r'''
constexpr bool HARDWARE_VERIFIED = true;
constexpr uint8_t SENSOR_COUNT = 16;
namespace Hardware {
constexpr bool BLACK_IS_HIGH = false;
bool goValue = true, readyValue = true, enabled = false;
int16_t left = 0, right = 0;
uint8_t fan = 0;
uint8_t samples[16];
void begin() {}
bool sw1() { return false; }
bool sw2() { return false; }
bool ready() { return readyValue; }
bool go() { return goValue; }
void leds(bool,bool,bool) {}
void motors(int16_t l,int16_t r) { left=l; right=r; enabled=true; }
void disable() { left=right=0; enabled=false; }
void suction(uint8_t p) { fan=p; }
void readSensors(uint8_t (&v)[16]) { for (int i=0;i<16;++i) v[i]=samples[i]; }
}
'''
checks = r'''
#include <cassert>
// Linea negra (ADC bajo) bajo los sensores 7 y 8, fondo blanco (ADC alto).
void centerLine() {
 for(auto &v:Hardware::samples) v=255;
 Hardware::samples[7]=Hardware::samples[8]=0;
}
void resetControl(uint8_t turbo = 0) {
 defaultParams(); params.turbo=turbo; applyParams();
 state=State::Running; blackLine=true;
 previousError=0; recovering=false; lineRight=false; turboQ=0; errorSide=0;
 stats=Stats(); sampleCount=0;
 runAt=lastSeen=tickAt=0; mockUs=0; suctionFrom=0; suctionPwm=0;
 Hardware::goValue=Hardware::readyValue=true;
 Hardware::left=Hardware::right=0; Hardware::fan=0;
 for(int i=0;i<16;++i) {low[i]=0; high[i]=255; scale[i]=256000UL/255;}
 centerLine();
}
void step(uint32_t ms) {mockUs=ms*1000; follow(ms);}
void send(const char *text) { Serial.input += text; Serial.input += '\n'; pollSerial(); }
int main() {
 // Rampa de arranque simetrica con la linea centrada.
 resetControl(); step(10);
 assert(Hardware::left>0 && Hardware::left<10 && Hardware::left==Hardware::right);
 step(150); assert(Hardware::left==100 && Hardware::right==100);
 // Succion: rampa de 0 a SUCTION_PWM en RUN_SUCTION_RAMP_MS.
 step(250); assert(Hardware::fan==SUCTION_PWM);
 // Linea a la derecha: giro a la derecha, rueda interior en reversa limitada.
 Hardware::samples[7]=Hardware::samples[8]=255; Hardware::samples[15]=0;
 step(255); assert(Hardware::left==MAX_MOTOR_PWM && Hardware::right==MAX_REVERSE_PWM && lineRight);
 // Perdida breve: girar hacia el ultimo lado, frenar la interior y luego soltarla.
 for(auto &v:Hardware::samples) v=255;
 step(260); assert(state==State::Running && recovering && stats.losses==1);
 assert(Hardware::left==OFFLINE_OUTER_PWM && Hardware::right==OFFLINE_INNER_PWM);
 step(300); assert(Hardware::left==OFFLINE_OUTER_PWM && Hardware::right==0 && stats.losses==1);
 // Recuperacion: control normal sin golpe derivativo.
 centerLine(); step(310);
 assert(state==State::Running && !recovering && Hardware::left==100);
 // Perdida por la izquierda: gira a la izquierda.
 Hardware::samples[7]=Hardware::samples[8]=255; Hardware::samples[0]=0;
 step(320); assert(!lineRight && Hardware::left<Hardware::right);
 for(auto &v:Hardware::samples) v=255;
 step(325); assert(Hardware::left==OFFLINE_INNER_PWM && Hardware::right==OFFLINE_OUTER_PWM);
 // Recuperacion tardia: parada, motores inhibidos, succion apagada y carrera guardada.
 step(320 + LOST_LINE_MS);
 assert(state==State::Stopped && reason==Reason::LineLost && !Hardware::enabled && Hardware::fan==0);
 RunLog log; assert(readLog(log) && log.count==sampleCount && log.stats.losses==2);
 assert(Serial.output.find("Se salio de la linea") != std::string::npos);
 // Telemetria CSV: cabecera, muestras y fin.
 Serial.output.clear(); send("d");
 assert(Serial.output.find("t_ms,error,izq,der,en_linea\n0,0,") == 0);
 assert(Serial.output.find("FIN") != std::string::npos);
 // Turbo: sube en recta hasta V+TURBO y se anula en curva.
 resetControl(30);
 for (uint32_t t=1; t<=600; ++t) step(t);
 assert(Hardware::left==130 && Hardware::right==130);
 Hardware::samples[7]=Hardware::samples[8]=255; Hardware::samples[12]=0;
 step(601); assert(turboQ==0);
 // Comandos serie: ajustar, limitar y guardar en EEPROM.
 resetControl(); state=State::Speed; Serial.output.clear();
 send("KP 0.08"); send("v 999"); send("turbo 50"); send("save");
 assert(fabsf(params.kp-0.08f)<1e-6 && params.speed==MAX_SPEED && params.turbo==50);
 params.kp=0; loadParams(); assert(fabsf(params.kp-0.08f)<1e-6 && kpQ==82);
 // GO ya activo al terminar la calibracion no arranca; exige flanco.
 resetControl(); state=State::Speed; goArmed=false; Hardware::goValue=true;
 mockUs=1000000; loop(); assert(state==State::Speed);
 Hardware::goValue=false; mockUs+=1000; loop(); assert(goArmed);
 Hardware::goValue=true; mockUs+=1000; loop(); assert(state==State::Running);
 // GO sin linea bajo la barra: no arranca y exige otro flanco.
 resetControl(); state=State::Speed; goArmed=true;
 for(auto &v:Hardware::samples) v=255;
 mockUs=1000000; loop(); assert(state==State::Speed && !goArmed);
 // Quitar GO detiene la carrera.
 resetControl(); step(10); Hardware::goValue=false; mockUs=20000; loop();
 assert(state==State::Stopped && reason==Reason::Request && !Hardware::enabled);
 // Ambas polaridades y extremos de barra producen posiciones validas.
 resetControl(); for(int i=0;i<16;++i) raw[i]=255;
 raw[0]=0; int16_t e; assert(position(e) && e==-7500);
 blackLine=false; for(int i=0;i<16;++i) raw[i]=0;
 raw[15]=255; assert(position(e) && e==7500);
 // Sin linea visible: posicion invalida.
 for(int i=0;i<16;++i) raw[i]=0; assert(!position(e));
}
'''
with tempfile.TemporaryDirectory(prefix='atlas-check-') as directory:
    tmp = Path(directory)
    (tmp/'Arduino.h').write_text(arduino)
    (tmp/'EEPROM.h').write_text(eeprom)
    source = (root/'src/main.cpp').read_text().replace('#include "atlas_hardware.h"', hardware)
    (tmp/'checks.cpp').write_text(source + checks)
    subprocess.run(['c++', '-std=c++11', '-Wall', '-Wextra', '-I'+str(tmp),
                    '-I'+str(root/'include'), str(tmp/'checks.cpp'), '-o', str(tmp/'checks')], check=True)
    subprocess.run([str(tmp/'checks')], check=True)
print('OK: rampas, giro, reversa, recuperacion, telemetria, turbo, comandos, armado de GO y polaridades.')
