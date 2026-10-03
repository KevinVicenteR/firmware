#pragma once
#include <stdint.h>

// Valores por defecto. KP, KD, V, TURBO y SUCCION tambien se ajustan por serie
// sin reprogramar ("?" muestra los comandos) y se guardan en EEPROM con "save".
namespace Config {
constexpr int8_t MOTOR_L_SIGN = 1; // -1 invierte el sentido de avance.
constexpr int8_t MOTOR_R_SIGN = 1;

// Control PD en unidades del original: error en milesimas de sensor (-7500..7500),
// KD por ciclo de control. Si se cambia CONTROL_US, reescalar KD en la misma proporcion.
constexpr uint32_t CONTROL_US = 750;
constexpr float KP = 0.05f;
constexpr float KD = 0.35f;
constexpr float MAX_KP = 2.0f;
constexpr float MAX_KD = 10.0f;

// Velocidad base seleccionable con SW1/SW2 (manual: maximo recomendado 160).
constexpr uint8_t MIN_SPEED = 40;
constexpr uint8_t BASE_SPEED = 50;
constexpr uint8_t MAX_SPEED = 100;
constexpr uint8_t SPEED_STEP = 10;
constexpr int16_t MAX_MOTOR_PWM = 100;
constexpr int16_t MAX_REVERSE_PWM = -80;

// Turbo en rectas: suma hasta TURBO_PWM mientras |error| <= STRAIGHT_ERROR,
// creciendo en TURBO_RAMP_MS; se anula al superar CURVE_ERROR o al perder la linea.
constexpr uint8_t TURBO_PWM = 0;
constexpr uint8_t MAX_TURBO_PWM = 80;
constexpr int16_t STRAIGHT_ERROR = 800;
constexpr int16_t CURVE_ERROR = 2000;
constexpr uint32_t TURBO_RAMP_MS = 300;
constexpr uint16_t CURVE_SLOWDOWN_PWM = 0; // PWM por sensor de error (0 = desactivado).

constexpr uint8_t SUCTION_PWM = 200; // Manual: maximo 215.
constexpr uint8_t MAX_SUCTION_PWM = 215;
constexpr uint8_t MIN_CONTRAST = 30; // Unidades ADC de 8 bits.
constexpr uint32_t DRIVE_RAMP_MS = 150;
constexpr uint32_t SUCTION_RAMP_MS = 1000;
constexpr uint32_t RUN_SUCTION_RAMP_MS = 250;
constexpr uint32_t MAX_RUN_MS = 8000;
constexpr uint32_t MAX_SESSION_MS = 45000;

// Modo botones (sin modulo GO): la succion sube al encender. SW1 o SW2 arranca tras
// START_DELAY_MS y detiene la carrera; SW1+SW2 juntos entran o salen de calibracion.
// Sin calibracion manual, se calibra quieto sobre la linea al arrancar.
constexpr bool AUTO_START = true;
constexpr bool AUTO_BLACK_LINE = true; // Linea negra sobre fondo blanco.
constexpr bool AUTO_SUCTION = true;
constexpr uint32_t START_DELAY_MS = 1000;

// Fuera de linea: rueda exterior a fondo, interior en reversa breve y luego detenida.
constexpr uint32_t LOST_LINE_MS = 500;
constexpr uint32_t OFFLINE_BRAKE_MS = 35;
constexpr int16_t OFFLINE_OUTER_PWM = MAX_MOTOR_PWM;
constexpr int16_t OFFLINE_INNER_PWM = MAX_REVERSE_PWM;
constexpr int16_t EDGE_ERROR = 6000; // |error| que marca el lado por donde salio la linea.

// Vista de la barra de sensores por serie ("ver" la activa o desactiva).
constexpr uint32_t VIEW_MS = 100;

// Telemetria: una muestra cada TELEMETRY_MS; se conservan las ultimas en EEPROM.
constexpr uint32_t TELEMETRY_MS = 25;
constexpr uint16_t TELEMETRY_SAMPLES = 300; // 7.5 s
// Sugerencias del resumen (orientativas).
constexpr int16_t OSC_HYSTERESIS = 500;
constexpr uint8_t OSCILLATION_HZ = 4;
constexpr uint16_t HIGH_MEAN_ERROR = 1500;

static_assert(MIN_SPEED <= BASE_SPEED && BASE_SPEED <= MAX_SPEED, "Rango de velocidad invalido");
static_assert(MAX_SPEED <= MAX_MOTOR_PWM && MAX_MOTOR_PWM <= 255, "Velocidad supera limite de motores");
static_assert(MAX_REVERSE_PWM <= 0 && MAX_REVERSE_PWM >= -255, "Limite de reversa invalido");
static_assert(SUCTION_PWM <= MAX_SUCTION_PWM && MAX_SUCTION_PWM <= 215, "Succion por encima del maximo recomendado");
static_assert(TURBO_PWM <= MAX_TURBO_PWM, "Turbo invalido");
static_assert(STRAIGHT_ERROR < CURVE_ERROR, "Umbrales de turbo invalidos");
static_assert(DRIVE_RAMP_MS > 0 && SUCTION_RAMP_MS > 0 && RUN_SUCTION_RAMP_MS > 0 && TURBO_RAMP_MS > 0,
              "Rampa debe ser positiva");
static_assert(CONTROL_US >= 600, "Reservar tiempo para ADC de 16 canales y calculo");
}
