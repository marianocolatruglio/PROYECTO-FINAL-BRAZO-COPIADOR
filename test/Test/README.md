# Proyecto: Brazo Copiador con AS5600L (Multi-vuelta)

Este proyecto implementa un sistema de brazo robot copiador controlado por Arduino, con lectura de posición absoluta multivuelta mediante sensores AS5600L, utilizando la librería desarrollada por Rob Tillaart. El objetivo es grabar trayectorias desde los encoders y luego reproducirlas con interpolación de movimiento en ambos motores.

---

## ⚙️ Características principales

- **Lectura multivuelta acumulativa** con `getCumulativePosition()` del AS5600L.
- **Multiplexor analógico** para selección de encoder (hasta 2 canales).
- **Interpolación Bresenham** para movimientos suaves entre puntos.
- **Homing automático** con finales de carrera y reseteo de posición.
- **Interfaz serie** simple para comandos de grabación, reproducción y consulta.
- **Velocidad configurable** mediante factor de escala (`speedFactor`).

---

## 📡 Comandos disponibles

- `h` → Ejecuta homing simultáneo.
- `r` → Reproduce trayectoria recibida por puerto serie.
- `g` → Graba trayectoria en tiempo real desde los encoders.
- `p` → Muestra la posición actual de ambos encoders.
- `e` / `d` → Habilita / deshabilita motores.
- `x` → Detiene la grabación.

---

## 🧠 Librerías utilizadas

- [`AS5600`](https://github.com/RobTillaart/AS5600): Librería para sensores AS5600 y AS5600L, con soporte de posición acumulativa.
- `Wire.h`: Para comunicación I2C.

---

## 📷 Ejemplo de salida

```plaintext
INICIO_TRAYECTORIA
123.45,67.89
124.23,69.02
FIN_TRAYECTORIA
```

---

## 🛠️ Hardware utilizado

- Arduino UNO.
- Sensores magnéticos AS5600L (x2).
- Multiplexor 4051 para selección de canal I2C.
- Motores paso a paso + drivers.
- Finales de carrera (x2).
- Fuente de alimentación externa 12 V.

---

## 📌 Notas

- Este proyecto está orientado al control de un brazo copiador de 2 GDL, donde ambos motores están alineados en el mismo eje de origen.
- La reproducción y grabación se realiza desde un script Python que envía comandos por puerto serie.

---

## 📎 Créditos

- Desarrollo: [Mariano Colatruglio](https://github.com/mcolatruglio)
- Librería AS5600L: Rob Tillaart ([Repositorio original](https://github.com/RobTillaart/AS5600))