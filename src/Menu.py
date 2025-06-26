#!/usr/bin/env python3
import serial
import time
import sys

# CONFIGURA ESTO según tu sistema
PUERTO = 'COM6'    # o 'COM3' en Windows
BAUDRATE = 115200
TIEMPO_RESET = 2.5           # segundos

def abrir_puerto():
    try:
        ser = serial.Serial(PUERTO, BAUDRATE, timeout=1)
        time.sleep(TIEMPO_RESET)
        return ser
    except serial.SerialException as e:
        print(f"No pude abrir {PUERTO}: {e}")
        sys.exit(1)

def enviar(ser, cmd):
    """Envía comando + '\\n' y lee respuesta hasta timeout."""
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
q  : Salir
""")

def main():
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
                # case 'b':
                #     respuesta = enviar(ser, cmd)
                #     for line in respuesta:
                #         print(line)

                case _:
                    print("Opción no reconocida. Intenta de nuevo.")

    except KeyboardInterrupt:
        print("\nInterrumpido por usuario.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
