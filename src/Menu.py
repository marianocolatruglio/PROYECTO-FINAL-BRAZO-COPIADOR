#!/usr/bin/env python3
import serial
import time
import sys
import threading

# CONFIGURA ESTO según tu sistema
PUERTO = 'COM6'    # o 'COM3' en Windows
BAUDRATE = 115200
TIEMPO_RESET = 0.1           # segundos
FICHERO_STREAM = 'posiciones.txt'

# Variables globales para streaming
en_thread = None
stop_event = threading.Event()

# Funciones de puerto serie
def abrir_puerto():
    try:
        ser = serial.Serial(PUERTO, BAUDRATE, timeout=1)
        time.sleep(TIEMPO_RESET)
        return ser
    except serial.SerialException as e:
        print(f"No pude abrir {PUERTO}: {e}")
        sys.exit(1)

def enviar(ser, cmd):
    """Envía comando + '\n' y lee respuesta hasta timeout."""
    ser.write((cmd + '\n').encode())
    lines = []
    start = time.time()
    while True:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            lines.append(line)
        if time.time() - start > 0.2:
            break
    return lines

# Worker para streaming
def streaming_worker(ser):
    with open(FICHERO_STREAM, 'w', encoding='utf-8') as f:
        f.write('posacum1\tposacum2\n')
        while not stop_event.is_set():
            # Solicita posiciones al Arduino
            respuestas = enviar(ser, 'o')
            for texto in respuestas:
                try:
                    partes = texto.split()
                    pos1 = float(partes[0])
                    pos2 = float(partes[1])
                except (IndexError, ValueError):
                    continue
                f.write(f"{pos1}\t{pos2}\n")
                f.flush()
                print(f"pos1: {pos1}, pos2: {pos2}")
            time.sleep(0.1)  # frecuencia de muestreo
    print(f"\nStreaming detenido. Archivo cerrado: {FICHERO_STREAM}\n")

# Menú principal
def menu():
    print("""
=== MENÚ PRINCIPAL ===
 h  : Homing
 p  : Leer ángulos (θ1 y θ2)
 k  : Cinemática directa
 m  : Cinemática inversa (no mueve)
 i  : Mover a (X,Y)
 o  : Reportar posiciones (motor1, motor2)
 e  : Habilitar motores
 d  : Deshabilitar motores
 s  : Mover N pasos en motor 2
 1  : Mover motor 1 en pasos fijos
 2  : Mover motor 2 en pasos fijos
 b  : Iniciar streaming de posiciones
 f  : Detener streaming de posiciones
 q  : Salir
""")

# Función principal
def main():
    global en_thread, stop_event
    ser = abrir_puerto()
    pasos_aux1 = 100
    pasos_aux2 = 100

    try:
        while True:
            menu()
            cmd = input("Ingresa opción: ").strip().lower()

            match cmd:
                case 'q':
                    print("Saliendo.")
                    break

                case 'h' | 'p' | 'k' | 'o' | 'e' | 'd':
                    respuesta = enviar(ser, cmd)
                    for line in respuesta:
                        print(line)

                case '1':
                    respuesta = enviar(ser, cmd)
                    for line in respuesta:
                        print(line)
                    pasos_aux1 += 100

                case '2':
                    respuesta = enviar(ser, cmd)
                    for line in respuesta:
                        print(line)
                    pasos_aux2 += 100

                case 's':
                    n = input("¿Cuántos pasos mover en motor 2? ").strip()
                    respuesta = enviar(ser, f"s {n}")
                    for line in respuesta:
                        print(line)

                case 'm':
                    x = input("X objetivo [mm]: ").strip()
                    y = input("Y objetivo [mm]: ").strip()
                    respuesta = enviar(ser, f"m {x} {y}")
                    for line in respuesta:
                        print(line)

                case 'i':
                    x = input("X destino [mm]: ").strip()
                    y = input("Y destino [mm]: ").strip()
                    respuesta = enviar(ser, f"i {x} {y}")
                    for line in respuesta:
                        print(line)

                case 'b':
                    if en_thread and en_thread.is_alive():
                        print("Streaming ya está activo.")
                    else:
                        stop_event.clear()
                        en_thread = threading.Thread(target=streaming_worker, args=(ser,), daemon=True)
                        en_thread.start()
                        print(f"Streaming iniciado. Guardando en '{FICHERO_STREAM}'.")

                case 'f':
                    if en_thread and en_thread.is_alive():
                        stop_event.set()
                        en_thread.join()
                    else:
                        print("No hay streaming activo.")

                case _:
                    print("Opción no reconocida. Intenta de nuevo.")

    except KeyboardInterrupt:
        print("\nInterrumpido por usuario.")
    finally:
        if en_thread and en_thread.is_alive():
            stop_event.set()
            en_thread.join()
        ser.close()
        print("Puerto serie cerrado.")

if __name__ == "__main__":
    main()
