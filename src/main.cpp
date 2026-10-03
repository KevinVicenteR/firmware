#include <Arduino.h>
#include <EEPROM.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "atlas_hardware.h"
#include "atlas_config.h"

namespace {
using namespace Config;
enum class State : uint8_t { Locked, Diagnostic, Ready, Countdown, Color, Calibration, Speed, Running, Stopped };
enum class Gesture : uint8_t { None, Single, Both };
enum class Reason : uint8_t { Request, LineLost, RunLimit, Session };

// Parametros ajustables por serie; "save" los guarda en EEPROM.
struct Params {
  uint16_t magic;
  float kp;
  float kd;
  uint8_t speed;
  uint8_t turbo;
  uint8_t suction;
};
struct Stats {
  uint32_t durationMs;
  uint32_t cycles;
  uint32_t offCycles;
  uint32_t errSum;
  uint16_t losses;
  uint16_t maxErr;
  uint16_t maxLoopUs;
  uint16_t crossings;
};
// error/100 y PWM/2; error=-128 fuera de linea.
struct Sample {
  int8_t error;
  int8_t left;
  int8_t right;
};
struct RunLog {
  uint16_t magic;
  Stats stats;
  Params params;
  uint16_t first;
  uint16_t count;
  Reason reason;
};
struct Command {
  int16_t left;
  int16_t right;
};
constexpr uint16_t PARAMS_MAGIC = 0xA7A1;
constexpr uint16_t LOG_MAGIC = 0xA7B1;
constexpr int PARAMS_ADDR = 0;
constexpr int LOG_ADDR = 32;
constexpr int SAMPLES_ADDR = 96;
static_assert(sizeof(Params) <= LOG_ADDR - PARAMS_ADDR, "Params no cabe en EEPROM");
static_assert(sizeof(RunLog) <= SAMPLES_ADDR - LOG_ADDR, "RunLog no cabe en EEPROM");
static_assert(SAMPLES_ADDR + TELEMETRY_SAMPLES * sizeof(Sample) <= 1024, "Telemetria no cabe en EEPROM");

// Estado del firmware: el lazo de un microcontrolador necesita estado global mutable.
State state = State::Locked;
Reason reason = Reason::Request;
Params params;
int16_t kpQ;        // Ganancias en Q10 para el lazo entero.
int16_t kdQ;
uint16_t turboStep; // Incremento de turbo por ciclo, Q8.
Stats stats;
Sample samples[TELEMETRY_SAMPLES];
uint16_t sampleCount;
uint8_t low[SENSOR_COUNT];
uint8_t high[SENSOR_COUNT];
uint8_t raw[SENSOR_COUNT];
uint16_t scale[SENSOR_COUNT]; // 256000 / contraste: evita divisiones en el lazo.
uint8_t suctionPwm = 0;
uint8_t suctionFrom = 0;
bool blackLine = true;
bool rampActive = false;
bool goArmed = false;
bool recovering = false;
bool lineRight = false;
int8_t errorSide = 0;
int16_t previousError = 0;
uint16_t turboQ = 0;
uint32_t stateAt;
uint32_t rampAt;
uint32_t runAt;
uint32_t tickAt;
uint32_t lastSeen;
uint32_t reportAt;
uint32_t calFailAt;     // Momento del ultimo intento de calibracion fallido.
bool calFailed = false;
bool autoMode = false; // Modo botones: sin GO y sin menu.
bool gestureHeld = false;
bool gestureBoth = false;
bool gestureSkip = false; // Ignorar el gesto de la pulsacion que detuvo la carrera.
bool calibrated = false;
bool viewBar = true;    // Imprimir lo que ve la barra de sensores.
uint32_t viewAt;
char command[24];
uint8_t commandLength = 0;

struct Button {
  bool last = false;
  bool stable = false;
  uint32_t changed = 0;
  bool update(bool value, uint32_t now) {
    if (value != last) { last = value; changed = now; }
    if (value != stable && now - changed >= 25) {
      stable = value;
      return stable;
    }
    return false;
  }
};
Button b1;
Button b2;
// Gesto completo al soltar todos los botones: uno solo o los dos a la vez en algun momento.
Gesture readGesture() {
  if (b1.stable || b2.stable) {
    gestureHeld = true;
    if (b1.stable && b2.stable) gestureBoth = true;
    return Gesture::None;
  }
  if (!gestureHeld) return Gesture::None;
  const Gesture g = gestureBoth ? Gesture::Both : Gesture::Single;
  gestureHeld = gestureBoth = false;
  if (gestureSkip) { gestureSkip = false; return Gesture::None; }
  return g;
}

// ===== Parametros =====
void applyParams() {
  params.speed = constrain(params.speed, MIN_SPEED, MAX_SPEED);
  params.turbo = min(params.turbo, MAX_TURBO_PWM);
  params.suction = min(params.suction, MAX_SUCTION_PWM);
  params.kp = constrain(params.kp, 0.0f, MAX_KP);
  params.kd = constrain(params.kd, 0.0f, MAX_KD);
  kpQ = static_cast<int16_t>(params.kp * 1024 + 0.5f);
  kdQ = static_cast<int16_t>(params.kd * 1024 + 0.5f);
  turboStep = static_cast<uint16_t>((uint32_t(params.turbo) << 8) * CONTROL_US / (TURBO_RAMP_MS * 1000UL));
}
void defaultParams() {
  params = {PARAMS_MAGIC, KP, KD, BASE_SPEED, TURBO_PWM, SUCTION_PWM};
  applyParams();
}
void loadParams() {
  EEPROM.get(PARAMS_ADDR, params);
  if (params.magic != PARAMS_MAGIC || isnan(params.kp) || isnan(params.kd)) defaultParams();
  else applyParams();
}
void printParams(const Params &p) {
  Serial.print(F("KP ")); Serial.print(p.kp, 3);
  Serial.print(F("  KD ")); Serial.print(p.kd, 3);
  Serial.print(F("  V ")); Serial.print(p.speed);
  Serial.print(F("  TURBO ")); Serial.print(p.turbo);
  Serial.print(F("  SUCCION ")); Serial.println(p.suction);
}

// ===== Telemetria y resumen =====
const __FlashStringHelper *reasonText(Reason r) {
  switch (r) {
    case Reason::LineLost: return F("linea perdida");
    case Reason::RunLimit: return F("limite de carrera");
    case Reason::Session: return F("limite de sesion");
    default: return F("GO / boton");
  }
}
bool readLog(RunLog &log) {
  EEPROM.get(LOG_ADDR, log);
  return log.magic == LOG_MAGIC && log.count <= TELEMETRY_SAMPLES;
}
// Escribe solo los bytes que cambian (~3.3 ms por byte): unos segundos tras cada carrera.
void saveRun() {
  RunLog log;
  log.magic = LOG_MAGIC;
  log.stats = stats;
  log.params = params;
  log.reason = reason;
  log.count = min(sampleCount, TELEMETRY_SAMPLES);
  log.first = sampleCount - log.count;
  EEPROM.put(LOG_ADDR, log);
  for (uint16_t k = 0; k < log.count; ++k)
    EEPROM.put(SAMPLES_ADDR + k * sizeof(Sample), samples[(log.first + k) % TELEMETRY_SAMPLES]);
}
void printSuggestions(const Stats &s, uint32_t meanErr, uint32_t oscHz10) {
  bool stable = true;
  if (s.maxLoopUs > CONTROL_US) { stable = false; Serial.println(F("> El ciclo supera CONTROL_US: subirlo y reescalar KD.")); }
  if (s.losses) { stable = false; Serial.println(F("> Se salio de la linea: bajar V/TURBO o subir KD.")); }
  if (oscHz10 > OSCILLATION_HZ * 10UL) { stable = false; Serial.println(F("> Oscila: bajar KP o subir KD.")); }
  else if (meanErr > HIGH_MEAN_ERROR) { stable = false; Serial.println(F("> Error medio alto: subir KP.")); }
  if (stable) Serial.println(F("> Estable: probar +10 de V o de TURBO."));
}
void printSummary() {
  RunLog log;
  if (!readLog(log)) { Serial.println(F("Sin carrera registrada.")); return; }
  const Stats &s = log.stats;
  const uint32_t onCycles = s.cycles - s.offCycles;
  const uint32_t meanErr = onCycles ? s.errSum / onCycles : 0;
  const uint32_t oscHz10 = s.durationMs ? uint32_t(s.crossings) * 5000UL / s.durationMs : 0;
  Serial.println(F("--- Ultima carrera ---"));
  Serial.print(F("Parada: ")); Serial.print(reasonText(log.reason));
  Serial.print(F("  Duracion (ms): ")); Serial.println(s.durationMs);
  printParams(log.params);
  Serial.print(F("Error medio / maximo (milesimas de sensor): "));
  Serial.print(meanErr); Serial.print(F(" / ")); Serial.println(s.maxErr);
  Serial.print(F("Fuera de linea (%): ")); Serial.print(s.cycles ? s.offCycles * 100 / s.cycles : 0);
  Serial.print(F("  Salidas: ")); Serial.println(s.losses);
  Serial.print(F("Oscilacion (Hz): ")); Serial.println(static_cast<float>(oscHz10) / 10.0f, 1);
  Serial.print(F("Ciclo de control maximo (us): ")); Serial.println(s.maxLoopUs);
  printSuggestions(s, meanErr, oscHz10);
  Serial.println(F("'d' telemetria CSV | '?' comandos | SW1/SW2 nueva carrera"));
}
void dumpTelemetry() {
  RunLog log;
  if (!readLog(log)) { Serial.println(F("Sin carrera registrada.")); return; }
  Serial.println(F("t_ms,error,izq,der,en_linea"));
  for (uint16_t k = 0; k < log.count; ++k) {
    Sample s;
    EEPROM.get(SAMPLES_ADDR + k * sizeof(Sample), s);
    const bool online = s.error != INT8_MIN;
    Serial.print(uint32_t(log.first + k) * TELEMETRY_MS); Serial.print(',');
    if (online) Serial.print(s.error * 100);
    Serial.print(','); Serial.print(s.left * 2);
    Serial.print(','); Serial.print(s.right * 2);
    Serial.print(','); Serial.println(online);
  }
  Serial.println(F("FIN"));
}

// ===== Consola serie (fuera de carrera) =====
void printHelp() {
  Serial.println(F("Comandos: kp <x> | kd <x> | v <pwm> | turbo <pwm> | succion <pwm> | save | defaults"));
  Serial.println(F("          r (resumen) | d (telemetria CSV) | ver (barra de sensores) | ?"));
  printParams(params);
}
// Ajusta un parametro; devuelve false si el comando no es de ajuste.
bool setParam(const char *name, const char *arg) {
  if (!arg) return false;
  const auto value = static_cast<float>(atof(arg));
  const auto pwm = static_cast<uint8_t>(constrain(value, 0.0f, 255.0f));
  if (!strcmp(name, "kp")) params.kp = value;
  else if (!strcmp(name, "kd")) params.kd = value;
  else if (!strcmp(name, "v")) params.speed = pwm;
  else if (!strcmp(name, "turbo")) params.turbo = pwm;
  else if (!strcmp(name, "succion")) params.suction = pwm;
  else return false;
  return true;
}
void handleCommand(char *text) {
  char *arg = strchr(text, ' ');
  if (arg) {
    *arg = '\0';
    ++arg;
    while (*arg == ' ') ++arg;
  }
  if (setParam(text, arg)) applyParams();
  else if (!strcmp(text, "defaults")) defaultParams();
  else if (!strcmp(text, "save")) { EEPROM.put(PARAMS_ADDR, params); Serial.println(F("Parametros guardados.")); return; }
  else if (!strcmp(text, "r")) { printSummary(); return; }
  else if (!strcmp(text, "d")) { dumpTelemetry(); return; }
  else if (!strcmp(text, "ver")) { viewBar = !viewBar; return; }
  else { printHelp(); return; }
  printParams(params);
}
void pollSerial() {
  while (Serial.available()) {
    const auto c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (!commandLength) continue;
      command[commandLength] = '\0';
      commandLength = 0;
      handleCommand(command);
    } else if (commandLength < sizeof(command) - 1) {
      command[commandLength] = static_cast<char>(tolower(static_cast<unsigned char>(c)));
      ++commandLength;
    }
  }
}

// ===== Control =====
void drive(int16_t left, int16_t right) { Hardware::motors(left * MOTOR_L_SIGN, right * MOTOR_R_SIGN); }
void setSuction(uint8_t pwm) { suctionPwm = pwm; Hardware::suction(pwm); }
void off() { Hardware::disable(); setSuction(0); rampActive = false; }
void stop(Reason why, const __FlashStringHelper *message) {
  const bool wasRunning = state == State::Running;
  off();
  state = State::Stopped;
  reason = why;
  Serial.println(message);
  if (!wasRunning) return;
  stats.durationMs = millis() - runAt;
  Hardware::leds(true, true, true); // Guardando telemetria.
  saveRun();
  printSummary();
}
void enterSpeed() {
  state = State::Speed;
  goArmed = false;
  Serial.print(F("Velocidad: ")); Serial.println(params.speed);
  Serial.println(F("SW1 -10 / SW2 +10. READY prepara succion; GO arranca (debe pasar por bajo)."));
}
void calibrateStart(bool black, uint32_t now) {
  blackLine = black;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) { low[i] = 255; high[i] = 0; }
  state = State::Calibration;
  stateAt = now;
  calFailed = false;
  Serial.println(autoMode ? F("Calibracion: mover toda la barra sobre linea y fondo. SW1+SW2 para salir.")
                          : F("Mueve manualmente toda la barra sobre linea y fondo. Pulsa para terminar."));
}
// Promedio de 4 barridos para que el ruido no ensanche el minimo/maximo de calibracion.
void readAveraged() {
  uint16_t sum[SENSOR_COUNT] = {};
  for (uint8_t n = 0; n < 4; ++n) {
    Hardware::readSensors(raw);
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) sum[i] += raw[i];
  }
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) raw[i] = static_cast<uint8_t>(sum[i] / 4);
}
void trackMinMax() {
  readAveraged();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    low[i] = min(low[i], raw[i]);
    high[i] = max(high[i], raw[i]);
  }
}
// Error en milesimas de sensor: -7500 (sensor 0, izquierda) .. 7500 (sensor 15).
// Cuanto ve la linea un sensor: 0 (fondo) .. 1000 (linea), segun la calibracion.
uint16_t lineValue(uint8_t i) {
  uint16_t value = 0;
  if (raw[i] > low[i]) value = static_cast<uint16_t>(min(uint32_t(raw[i] - low[i]) * scale[i] >> 8, 1000UL));
  if (blackLine != Hardware::BLACK_IS_HIGH) value = 1000 - value;
  return value;
}
bool position(int16_t &error) {
  uint32_t sum = 0;
  uint32_t weighted = 0;
  uint16_t peak = 0;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    uint16_t value = lineValue(i);
    if (value > peak) peak = value;
    if (value < 150) value = 0;
    sum += value;
    weighted += uint32_t(value) * i;
  }
  if (peak < 300 || sum < 400) return false;
  error = static_cast<int16_t>(static_cast<int16_t>(weighted * 1000 / sum) - 7500);
  return true;
}
void startRun(uint32_t now, int16_t initial) {
  previousError = initial;
  recovering = false;
  lineRight = initial > 0;
  errorSide = 0;
  turboQ = 0;
  sampleCount = 0;
  stats = Stats();
  suctionFrom = suctionPwm;
  runAt = now;
  lastSeen = now;
  tickAt = micros();
  state = State::Running;
  Hardware::leds(true, true, true);
}
void updateRunSuction(uint32_t elapsed) {
  const uint8_t target = autoMode && !AUTO_SUCTION ? 0 : params.suction;
  if (elapsed >= RUN_SUCTION_RAMP_MS) { setSuction(target); return; }
  const int32_t delta = int32_t(target) - suctionFrom;
  setSuction(static_cast<uint8_t>(suctionFrom + delta * int32_t(elapsed) / int32_t(RUN_SUCTION_RAMP_MS)));
}
// Fuera de linea: girar hacia el lado por donde salio; exterior a fondo, interior frena.
Command offlineCommand(uint32_t now) {
  if (!recovering) { recovering = true; ++stats.losses; }
  ++stats.offCycles;
  turboQ = 0;
  const int16_t inner = now - lastSeen <= OFFLINE_BRAKE_MS ? OFFLINE_INNER_PWM : 0;
  if (lineRight) return {OFFLINE_OUTER_PWM, inner};
  return {inner, OFFLINE_OUTER_PWM};
}
// Turbo en rectas: sube mientras el error es pequeno y se anula en curvas.
void updateTurbo(int16_t magnitude) {
  if (magnitude <= STRAIGHT_ERROR) {
    turboQ = static_cast<uint16_t>(min(uint32_t(turboQ) + turboStep, uint32_t(params.turbo) << 8));
  } else if (magnitude >= CURVE_ERROR) {
    turboQ = 0;
  }
}
// Cruces del centro con histeresis para estimar oscilacion.
void trackOscillation(int16_t error) {
  int8_t side = errorSide;
  if (error > OSC_HYSTERESIS) side = 1;
  else if (error < -OSC_HYSTERESIS) side = -1;
  if (side == errorSide) return;
  if (errorSide) ++stats.crossings;
  errorSide = side;
}
Command onlineCommand(uint32_t now, uint32_t elapsed, int16_t error) {
  // Al recuperar la linea, reiniciar la derivada para evitar un golpe de giro.
  if (recovering) { previousError = error; recovering = false; }
  lastSeen = now;
  const int16_t magnitude = abs(error);
  if (error >= EDGE_ERROR) lineRight = true;
  else if (error <= -EDGE_ERROR) lineRight = false;
  updateTurbo(magnitude);
  trackOscillation(error);
  stats.errSum += magnitude;
  if (uint16_t(magnitude) > stats.maxErr) stats.maxErr = magnitude;

  const int32_t correction = (int32_t(kpQ) * error + int32_t(kdQ) * (error - previousError)) >> 10;
  previousError = error;
  int32_t base = params.speed + (turboQ >> 8);
  if (CURVE_SLOWDOWN_PWM) base = max(int32_t(MIN_SPEED), base - int32_t(CURVE_SLOWDOWN_PWM) * magnitude / 1000);
  // Error positivo: linea a la derecha -> rueda izquierda mas rapida.
  int32_t left = base + correction;
  int32_t right = base - correction;
  if (elapsed < DRIVE_RAMP_MS) { // La rampa incluye la correccion: sin golpe de giro al arrancar.
    left = left * int32_t(elapsed) / int32_t(DRIVE_RAMP_MS);
    right = right * int32_t(elapsed) / int32_t(DRIVE_RAMP_MS);
  }
  return {static_cast<int16_t>(constrain(left, MAX_REVERSE_PWM, MAX_MOTOR_PWM)),
          static_cast<int16_t>(constrain(right, MAX_REVERSE_PWM, MAX_MOTOR_PWM))};
}
void recordSample(uint32_t elapsed, bool online, int16_t error, const Command &out) {
  if (elapsed < uint32_t(sampleCount) * TELEMETRY_MS) return;
  Sample &s = samples[sampleCount % TELEMETRY_SAMPLES];
  ++sampleCount;
  s.error = online ? static_cast<int8_t>(error / 100) : static_cast<int8_t>(INT8_MIN);
  s.left = static_cast<int8_t>(out.left / 2);
  s.right = static_cast<int8_t>(out.right / 2);
}
void follow(uint32_t now) {
  const uint32_t us = micros();
  if (us - tickAt < CONTROL_US) return;
  tickAt = us;
  const uint32_t elapsed = now - runAt;
  updateRunSuction(elapsed);
  Hardware::readSensors(raw);
  ++stats.cycles;
  int16_t error = 0;
  const bool online = position(error);
  if (!online && now - lastSeen >= LOST_LINE_MS) { stop(Reason::LineLost, F("Linea perdida.")); return; }
  const Command out = online ? onlineCommand(now, elapsed, error) : offlineCommand(now);
  drive(out.left, out.right);
  recordSample(elapsed, online, error, out);
  const uint32_t loopUs = micros() - us;
  if (loopUs > stats.maxLoopUs) stats.maxLoopUs = static_cast<uint16_t>(loopUs);
}

// ===== Vista de la barra =====
// Calibrado: [..##....] con # = linea, + = borde, . = fondo, y el error.
// Sin calibrar: valores ADC crudos. En carrera no bloquea: omite la linea si el buffer esta lleno.
void printBar(uint32_t now) {
  if (!viewBar || state == State::Diagnostic || now - viewAt < VIEW_MS) return;
  if (state == State::Running && Serial.availableForWrite() < 32) return;
  viewAt = now;
  if (state != State::Running) Hardware::readSensors(raw);
  if (!calibrated) {
    for (const uint8_t value : raw) { Serial.print(value); Serial.print(' '); }
    Serial.println(F("(sin calibrar)"));
    return;
  }
  Serial.print('[');
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint16_t value = lineValue(i);
    Serial.print(value >= 500 ? '#' : value >= 150 ? '+' : '.');
  }
  Serial.print(F("] "));
  int16_t error;
  if (position(error)) { Serial.print(F("e=")); Serial.println(error); }
  else Serial.println(F("sin linea"));
}

// ===== Diagnostico =====
void motorTest() {
  Serial.println(F("Prueba de motores: ruedas en el aire. Izq +, izq -, der +, der -."));
  const Command steps[] = {{60, 0}, {-60, 0}, {0, 60}, {0, -60}};
  for (const auto &s : steps) {
    Hardware::leds(s.left, s.right, false);
    drive(s.left, s.right);
    delay(400);
    Hardware::disable();
    delay(600);
  }
  Hardware::leds(false, false, false);
}
void diagnostic(uint32_t now, bool p1, bool p2) {
  if (p1) { setSuction(0); motorTest(); }
  if (p2) {
    setSuction(suctionPwm ? 0 : params.suction);
    Serial.println(suctionPwm ? F("Succion ON (SW2 apaga).") : F("Succion OFF."));
  }
  Hardware::leds(Hardware::ready(), Hardware::go(), suctionPwm);
  if (now - reportAt < 200) return;
  reportAt = now;
  Hardware::readSensors(raw);
  for (const uint8_t value : raw) { Serial.print(value); Serial.print(' '); }
  Serial.print(F("| READY ")); Serial.print(Hardware::ready());
  Serial.print(F(" GO ")); Serial.println(Hardware::go());
}

// ===== Estados =====
// Imprime min/max/contraste de cada sensor, valida el contraste y precalcula su escala.
bool finishCalibration() {
  bool valid = true;
  Serial.println(F("Sensor  min  max  contraste"));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const auto contrast = static_cast<uint8_t>(high[i] - low[i]);
    const bool ok = high[i] >= low[i] && contrast >= MIN_CONTRAST;
    Serial.print(F("  ")); Serial.print(i);
    Serial.print(F("     ")); Serial.print(low[i]);
    Serial.print(F("  ")); Serial.print(high[i]);
    Serial.print(F("  ")); Serial.print(high[i] >= low[i] ? contrast : 0);
    Serial.println(ok ? F("  OK") : F("  BAJO"));
    if (ok) scale[i] = static_cast<uint16_t>(256000UL / contrast);
    else valid = false;
  }
  calibrated = valid;
  if (valid) Serial.println(F("Calibracion correcta."));
  else Serial.println(F("Contraste insuficiente: pasar TODOS los sensores sobre linea y fondo y pulsar otra vez."));
  return valid;
}
void enterReady(uint32_t now);
void handleCalibration(uint32_t now, bool pressed) {
  if (calFailed && now - calFailAt < 1000) {
    // Calibracion fallida: los tres LEDs parpadean rapido durante 1 s.
    const bool fast = (now / 50) % 2;
    Hardware::leds(fast, fast, fast);
  } else {
    calFailed = false;
    const bool blink = (now / 200) % 2;
    Hardware::leds(blackLine && blink, blackLine && blink, !blackLine && blink);
  }
  trackMinMax();
  if (!pressed || now - stateAt < 500) return;
  if (!finishCalibration()) {
    calFailed = true;
    calFailAt = now;
    if (!autoMode) return;
    Serial.println(F("Se calibrara quieto sobre la linea al arrancar."));
  }
  Hardware::leds(false, false, false);
  if (autoMode) enterReady(now);
  else enterSpeed();
}
void enterReady(uint32_t now) {
  Hardware::disable();
  blackLine = AUTO_BLACK_LINE;
  if (!suctionPwm) rampAt = now;
  state = State::Ready;
  Serial.println(F("Listo. SW1 o SW2: arrancar. SW1+SW2: calibrar."));
}
// Calibracion sin giro: con el robot quieto sobre la linea, unos sensores ven linea y otros
// fondo. El minimo y el maximo de toda la barra se usan como referencia comun.
bool calibrateOnLine() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) { low[i] = 255; high[i] = 0; }
  for (uint8_t n = 0; n < 8; ++n) trackMinMax();
  uint8_t lo = 255;
  uint8_t hi = 0;
  Serial.print(F("Lectura: "));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(raw[i]); Serial.print(' ');
    lo = min(lo, low[i]);
    hi = max(hi, high[i]);
  }
  Serial.println();
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) { low[i] = lo; high[i] = hi; }
  return finishCalibration();
}
// La succion sube desde el encendido y sigue activa mientras espera el arranque.
void spinSuction(uint32_t now) {
  const uint8_t target = AUTO_SUCTION ? params.suction : 0;
  const uint32_t t = now - rampAt;
  setSuction(t >= SUCTION_RAMP_MS ? target : static_cast<uint8_t>(t * target / SUCTION_RAMP_MS));
}
void handleReady(uint32_t now, Gesture g) {
  spinSuction(now);
  if (g == Gesture::Both) { setSuction(0); calibrateStart(AUTO_BLACK_LINE, now); return; }
  if (g == Gesture::Single) {
    state = State::Countdown;
    stateAt = now;
    Serial.println(F("Arranque en 1 s. Boton cancela."));
    return;
  }
  // Tras un intento fallido, los tres LEDs parpadean rapido 1 s; si no, lento.
  const bool blink = calFailed && now - calFailAt < 1000 ? (now / 50) % 2 : (now / 500) % 2;
  Hardware::leds(blink, blink, blink);
}
void handleCountdown(uint32_t now, bool pressed) {
  spinSuction(now);
  if (pressed) {
    gestureSkip = true;
    Serial.println(F("Arranque cancelado."));
    enterReady(now);
    return;
  }
  const bool fast = (now / 100) % 2;
  Hardware::leds(fast, fast, fast);
  if (now - stateAt < START_DELAY_MS) return;
  int16_t initial = 0;
  bool online;
  if (calibrated) { readAveraged(); online = position(initial); }
  else online = calibrateOnLine() && position(initial);
  if (!online) {
    Serial.println(F("No se ve la linea: centrar el robot y pulsar otra vez."));
    calFailed = true;
    calFailAt = now;
    enterReady(now);
    return;
  }
  calFailed = false;
  startRun(now, initial);
}
void updateReadySuction(uint32_t now) {
  if (!Hardware::ready()) {
    setSuction(0);
    rampActive = false;
    return;
  }
  if (!rampActive) { rampAt = now; rampActive = true; }
  const uint32_t elapsed = now - rampAt;
  setSuction(elapsed >= SUCTION_RAMP_MS ? params.suction
                                        : static_cast<uint8_t>(elapsed * params.suction / SUCTION_RAMP_MS));
}
void handleSpeed(uint32_t now, bool p1, bool p2) {
  if (p1) params.speed = max(int(MIN_SPEED), params.speed - SPEED_STEP);
  if (p2) params.speed = min(int(MAX_SPEED), params.speed + SPEED_STEP);
  if (p1 || p2) { Serial.print(F("V: ")); Serial.println(params.speed); }
  updateReadySuction(now);
  Hardware::leds(b1.stable, b2.stable, suctionPwm == params.suction);
  // Solo arranca con un flanco de GO; si ya estaba activo al terminar la calibracion, no.
  if (!Hardware::go()) { goArmed = true; return; }
  if (!goArmed) return;
  goArmed = false;
  Hardware::readSensors(raw);
  int16_t initial;
  if (!position(initial)) { Serial.println(F("Colocar sobre la linea y repetir GO.")); return; }
  startRun(now, initial);
}
void handleRunning(uint32_t now, bool pressed) {
  // Como el original, solo GO detiene la carrera; READY puede caer al arrancar segun el modulo.
  if (pressed) gestureSkip = true; // Soltar el boton de parada no rearranca.
  if ((!autoMode && !Hardware::go()) || pressed) stop(Reason::Request, F("Parada solicitada."));
  else if (now - runAt >= MAX_RUN_MS) stop(Reason::RunLimit, F("Limite de carrera alcanzado."));
  else follow(now);
}
void handleStopped(uint32_t now, bool pressed, Gesture g) {
  // Codigo de LEDs segun el motivo de parada.
  const bool fast = (now / 125) % 2;
  const bool slow = (now / 500) % 2;
  switch (reason) {
    case Reason::LineLost: Hardware::leds(fast, false, false); break;
    case Reason::RunLimit: Hardware::leds(false, fast, false); break;
    case Reason::Session: Hardware::leds(false, false, fast); break;
    default: Hardware::leds(slow, slow, slow); break;
  }
  // Nueva carrera sin RESET, conservando calibracion (no tras limite de sesion).
  if (reason == Reason::Session) return;
  if (autoMode && g != Gesture::None) { enterReady(now); handleReady(now, g); }
  else if (!autoMode && pressed) enterSpeed();
}
}

void setup() {
  Serial.begin(115200);
  if (!HARDWARE_VERIFIED) {
    Serial.println(F("ATLAS bloqueado: completar y verificar atlas_hardware.h."));
    return; // No tocar pines desconocidos.
  }
  Hardware::begin();
  off();
  loadParams();
  for (uint8_t i = 0; i < 20; ++i) { Hardware::leds(false, false, i % 2 == 0); delay(25); }
  Serial.println(F("ATLAS listo. '?' comandos, 'r' resumen de la ultima carrera."));
  printParams(params);
  if (Hardware::sw1() || Hardware::sw2()) {
    state = State::Diagnostic;
    Serial.println(F("Diagnostico: sensores crudos 0..15 (izq. a der.). SW1 motores, SW2 succion."));
    while (Hardware::sw1() || Hardware::sw2()) {
      // Esperar a soltar el boton para que no cuente como pulsacion.
    }
    delay(30);
    return;
  }
  if (AUTO_START) {
    autoMode = true;
    enterReady(millis());
    return;
  }
  state = State::Color;
  Serial.println(F("SW1: linea blanca. SW2: linea negra."));
}

void loop() {
  if (state == State::Locked) return;
  const uint32_t now = millis();
  const bool p1 = b1.update(Hardware::sw1(), now);
  const bool p2 = b2.update(Hardware::sw2(), now);
  const Gesture g = readGesture();
  if (state != State::Running) pollSerial();
  if (state != State::Stopped && now >= MAX_SESSION_MS) {
    stop(Reason::Session, F("Limite de sesion: apagar y dejar enfriar."));
  }
  switch (state) {
    case State::Diagnostic: diagnostic(now, p1, p2); break;
    case State::Ready: handleReady(now, g); break;
    case State::Countdown: handleCountdown(now, p1 || p2); break;
    case State::Color: if (p1 || p2) calibrateStart(!p1, now); break;
    case State::Calibration: handleCalibration(now, autoMode ? g == Gesture::Both : p1 || p2); break;
    case State::Speed: handleSpeed(now, p1, p2); break;
    case State::Running: handleRunning(now, p1 || p2); break;
    case State::Stopped: handleStopped(now, p1 || p2, g); break;
    case State::Locked: break;
  }
  printBar(millis());
}
