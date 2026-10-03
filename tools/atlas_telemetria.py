"""Descarga la ultima carrera del ATLAS por serie, la guarda en CSV y la grafica.

Uso:
  python3 tools/atlas_telemetria.py /dev/cu.usbserial-XXXX   # descargar y graficar
  python3 tools/atlas_telemetria.py --archivo carrera.csv     # solo graficar

Requiere: pip install pyserial matplotlib
La telemetria vive en la EEPROM del robot: se puede descargar despues de RESET o apagado.
"""
import argparse
import csv
import datetime
import time
from pathlib import Path


def descargar(puerto, carpeta):
    import serial

    conexion = serial.Serial()
    conexion.port, conexion.baudrate, conexion.timeout = puerto, 115200, 3
    conexion.dtr = conexion.rts = False  # No reiniciar el robot al abrir el puerto.
    conexion.open()
    with conexion:
        time.sleep(0.3)
        conexion.reset_input_buffer()
        conexion.write(b'r\n')
        resumen = []
        while (linea := conexion.readline().decode(errors='replace').rstrip()):
            resumen.append(linea)
            if linea.startswith("'d'") or linea.startswith('Sin carrera'):
                break
        print('\n'.join(resumen))
        conexion.write(b'd\n')
        filas = []
        while True:
            linea = conexion.readline().decode(errors='replace').strip()
            if not linea:
                raise SystemExit('Sin respuesta del robot (revisar puerto y que no este en carrera).')
            if linea == 'FIN':
                break
            if linea.startswith('t_ms') or filas or linea[0].isdigit():
                filas.append(linea)
    carpeta.mkdir(exist_ok=True)
    nombre = carpeta / datetime.datetime.now().strftime('carrera_%Y%m%d_%H%M%S')
    nombre.with_suffix('.csv').write_text('\n'.join(filas) + '\n')
    nombre.with_suffix('.txt').write_text('\n'.join(resumen) + '\n')
    print(f'Guardado: {nombre}.csv / .txt')
    return nombre.with_suffix('.csv')


def graficar(archivo):
    import matplotlib.pyplot as plt

    t, error, izq, der, fuera = [], [], [], [], []
    with open(archivo) as f:
        for fila in csv.DictReader(f):
            t.append(int(fila['t_ms']) / 1000)
            error.append(int(fila['error']) / 1000 if fila['error'] else float('nan'))
            izq.append(int(fila['izq']))
            der.append(int(fila['der']))
            fuera.append(fila['en_linea'] == '0')

    figura, (eje_error, eje_pwm) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    eje_error.plot(t, error, color='tab:blue', marker='.', markersize=3)
    eje_error.axhline(0, color='gray', linewidth=0.8)
    eje_error.set_ylabel('Error (sensores)')
    eje_error.set_ylim(-8, 8)
    eje_pwm.plot(t, izq, label='Izquierdo')
    eje_pwm.plot(t, der, label='Derecho')
    eje_pwm.set_ylabel('PWM')
    eje_pwm.set_xlabel('Tiempo (s)')
    eje_pwm.legend(loc='upper right')
    for eje in (eje_error, eje_pwm):
        for instante, sin_linea in zip(t, fuera):
            if sin_linea:
                eje.axvspan(instante, instante + 0.025, color='tab:red', alpha=0.2, linewidth=0)
        eje.grid(alpha=0.3)
    figura.suptitle(f'{Path(archivo).name}  (rojo: fuera de linea)')
    figura.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('puerto', nargs='?', help='Puerto serie del convertidor USB-TTL')
    parser.add_argument('--archivo', help='CSV ya descargado para graficar')
    parser.add_argument('--carpeta', default='carreras', help='Donde guardar las descargas')
    parser.add_argument('--sin-grafica', action='store_true')
    args = parser.parse_args()
    if not args.puerto and not args.archivo:
        parser.error('indicar un puerto o --archivo')
    archivo = args.archivo or descargar(args.puerto, Path(args.carpeta))
    if not args.sin_grafica:
        graficar(archivo)


if __name__ == '__main__':
    main()
