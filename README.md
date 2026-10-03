# Firmware ATLAS Rev. 1.4

Firmware Arduino/PlatformIO para ATmega328P. El mapa de conexiones se obtuvo de los archivos ATLAS 1.4.3 proporcionados; el control y la interfaz estan en `src/main.cpp`. Los originales en `ATLAS_1/` se conservan intactos con sus avisos de autoria y licencia, y no se compilan junto al programa nuevo.

## Conexiones integradas

| Funcion | Pines Arduino |
| --- | --- |
| SW1 / SW2 (activos bajos) | A5 / D7 |
| READY / GO (activos altos) | D3 / D4 |
| LED0 / LED1 / LED2 | D2 / D8 / D13 |
| Inhibicion comun de drivers | D12 |
| Motor izquierdo: direccion / PWM | D6 / D9 |
| Motor derecho: direccion / PWM | D5 / D10 |
| Succion PWM | D11 |
| Direccion del multiplexor | A0, A1, A2, A3 |
| Lectura del multiplexor | A4 |

Se conserva la configuracion de Timer1 y Timer2 del fabricante; Timer0 mantiene millis/micros. El motor derecho usa polaridad opuesta al izquierdo. La linea negra corresponde a ADC bajo, segun la inversion del original. El ADC trabaja como el original: 8 bits, reloj /16, ~0.3 ms para los 16 sensores.

`HARDWARE_VERIFIED=true` habilita el mapa identificado en los archivos; no significa que se haya probado fisicamente. Antes de correr, usar el modo diagnostico para verificar el avance de las ruedas y el orden de sensores.

## Uso

1. SW1 selecciona linea blanca; SW2, negra.
2. Desplazar manualmente todos los sensores sobre fondo y linea. Pulsar nuevamente para finalizar. Se exige contraste de al menos 30 unidades ADC (8 bits) en cada canal; los sensores que fallan se listan por serie.
3. SW1 disminuye y SW2 aumenta la velocidad base (V) en pasos de 10, rango 60..160.
4. READY inicia la rampa de succion de un segundo (LED2 encendido al completar). GO arranca solo con un flanco (debe haber estado en bajo) y con la linea bajo la barra. Si GO llega antes de completar la rampa, la succion termina de subir en 250 ms.
5. Desactivar GO o pulsar un boton detiene el robot. READY no se vigila durante la carrera, como en el original.
6. Al detenerse, el robot guarda la carrera en EEPROM (~3 s con los tres LEDs fijos, no apagar) e imprime el resumen. Pulsar SW1 o SW2 prepara otra carrera **sin RESET**, conservando la calibracion.

LEDs tras la parada: LED0 rapido = linea perdida; LED1 rapido = limite de carrera; LED2 rapido = limite de sesion (exige RESET); los tres lentos = GO o boton.

Se limita la carrera a 8 s y la sesion a 45 s desde el encendido, segun el manual. No se mide temperatura ni voltaje: apagar y dejar enfriar entre sesiones. Alimentar la succion desde 2S.

## Velocidad

- Lazo PD entero cada 750 us, como el original. KP=0.05 y KD=0.35 son las ganancias del fabricante (error en milesimas de sensor, KD por ciclo).
- **Turbo en rectas**: mientras `|error| <= 0.8` sensores, la velocidad sube hasta `V + TURBO` en 300 ms; se anula al superar 2 sensores de error o al perder la linea. Por defecto TURBO=30.
- Ruedas con PWM con signo: avance hasta 200, reversa hasta -120.
- Al perder la linea gira hacia el ultimo lado donde la vio: exterior a 200 e interior en reversa 35 ms. Es el metodo del original, con la correccion del calculo de tiempo fuera de linea que en `RunControl.ino` usaba una variable sin inicializar. Tras 500 ms sin linea, se detiene.
- `CURVE_SLOWDOWN_PWM` (en `include/atlas_config.h`) reduce la velocidad en curvas; desactivado por defecto.

## Retroalimentacion

Conectar el convertidor USB-TTL (primero al robot, despues a la PC) y abrir la consola:

```sh
PLATFORMIO_CORE_DIR="$PWD/.pio-core" pio device monitor
```

El monitor esta configurado para no reiniciar el robot (DTR/RTS en 0). Comandos (fuera de carrera):

| Comando | Efecto |
| --- | --- |
| `kp 0.06`, `kd 0.4` | Ganancias PD |
| `v 120` | Velocidad base |
| `turbo 40` | Velocidad extra en rectas (0..80) |
| `succion 210` | PWM de succion (maximo 215) |
| `save` / `defaults` | Guardar en EEPROM / volver a los valores de `atlas_config.h` |
| `r` | Resumen de la ultima carrera con sugerencias |
| `d` | Telemetria CSV (cada 25 ms: error, PWM izquierdo/derecho, en linea) |
| `?` | Ayuda y parametros actuales |

El resumen incluye duracion, motivo de parada, error medio y maximo, porcentaje fuera de linea, numero de salidas, frecuencia de oscilacion y tiempo maximo del ciclo de control, con sugerencias orientativas (por ejemplo "Oscila: bajar KP o subir KD"). La ultima carrera queda en EEPROM: se puede consultar despues de RESET o apagado.

Para descargar y graficar en la PC (`pip install pyserial matplotlib`):

```sh
python3 tools/atlas_telemetria.py /dev/cu.usbserial-XXXX
python3 tools/atlas_telemetria.py --archivo carreras/carrera_20261003_110000.csv
```

Guarda CSV y resumen en `carreras/` y muestra el error y el PWM de cada rueda, con los tramos fuera de linea en rojo.

Ciclo de ajuste sugerido: empezar con V=80 y TURBO=0; subir V de 10 en 10 mientras el resumen diga "Estable"; si oscila, subir KD; luego subir TURBO. Guardar con `save`.

## Modo diagnostico

Mantener SW1 o SW2 pulsado durante el parpadeo inicial del LED2 y soltar. Por serie se imprimen cada 200 ms los 16 sensores crudos (0 = izquierda) y READY/GO; LED0/LED1 reflejan READY/GO. SW1 ejecuta la prueba de motores con las ruedas en el aire (izquierdo adelante, atras; derecho adelante, atras, PWM 60). SW2 enciende o apaga la succion. Si una rueda gira al reves, cambiar `MOTOR_L_SIGN`/`MOTOR_R_SIGN`.

## Compilar

```sh
pio run -e atlas_usbasp
```

Programacion por USBasp:

```sh
pio run -e atlas_usbasp -t upload
```

Compilacion verificada con PlatformIO para ATmega328P. No se modifican fuses ni se graba automaticamente el robot. Si el fusible EESAVE no esta activo, grabar el firmware borra la EEPROM: los parametros vuelven a los valores por defecto.

Si el entorno restringe escrituras en la instalacion global de PlatformIO, usar el core local ya preparado:

```sh
PLATFORMIO_CORE_DIR="$PWD/.pio-core" pio run -e atlas_usbasp
```

## Pruebas

```sh
python3 tests/run_control_checks.py
```

Compila el control para la PC con hardware simulado y comprueba rampas, sentido de correccion, reversa, recuperacion por lado, parada tardia, telemetria en EEPROM, turbo, comandos serie, armado de GO y polaridades. No verifica conexiones fisicas, tiempos reales del ADC ni estabilidad en pista.
