<img width="1920" height="850" alt="image" src="https://github.com/user-attachments/assets/06df05e8-b3f3-4d3c-b7fd-119643758fde" />

# Sistema de Digitalizacion y Reproduccion de Contornos
Desarrollado por Colatruglio M. - Dziubek N.
## Introduccion

La evolución tecnológica en el ámbito industrial ha llevado a una creciente necesidad de soluciones automatizadas que permitan a pequeñas y medianas empresas mejorar su competitividad. En particular, el desarrollo de herramientas capaces de reproducir trayectorias manuales con precisión resulta de gran interés para sectores como la metalurgia, carpintería o manufactura personalizada.
La automatización industrial ha impulsado el desarrollo de dispositivos capaces de ejecutar tareas con precisión creciente. En este contexto, los brazos articulados han demostrado ser herramientas fundamentales para reproducir trayectorias definidas, tanto en entornos remotos como en talleres locales.
Este tipo de tecnología también resulta útil en talleres metalúrgicos de pequeña escala, donde un operario puede escanear una pieza irregular guiando manualmente el brazo y luego reproducir la misma trayectoria para realizar cortes sobre placas de metal. Esto permite fabricar tanto piezas únicas como series reducidas de forma eficiente.
Frente a la necesidad de una alternativa nacional y de bajo costo a los sistemas importados, se desarrolló un prototipo funcional que demuestra la viabilidad técnica de capturar y reproducir trayectorias con buena precisión, empleando componentes accesibles como Arduino y piezas impresas en 3D.
El proyecto consistió en desarrollar un brazo articulado de dos grados de libertad, capaz de grabar trayectorias guiadas por el usuario y luego reproducirlas automáticamente. Este tipo de soluciones se observa en la industria metalúrgica internacional, especialmente en tareas de corte térmico.
Se planteó una alternativa local escalable, partiendo de un prototipo a pequeña escala como prueba de concepto. Si bien este no incluye herramientas de corte reales, sí valida el sistema de guiado, almacenamiento y reproducción.

![RENDER1](https://github.com/user-attachments/assets/f081308e-e7b5-4d39-92d0-e8df6d7e722d)

# Objetivo
* Proveer a la industria de un sistema de posicionamiento robotico para operaciones de corte de metales in-situ de bajo costo , modular y de facil operacion, utilizando una estructura abierta.
* Sustituir equipos importados de alto costo.


# Alcance

# Problematica

# Componentes Utilizados

Como parte del Hardware se utilizaron los siguientes componentes:
NEMA23 Stepper Motor
Description: A high-torque stepper motor used for precise control of CNC axes.
Pins: A+, A-, B+, B-
CNC Shield V3 Engraving Machine Expansion Board
Description: An expansion board that interfaces with the Arduino to control stepper motors and other peripherals.
Pins: EN, GND, x.Step, DIR, Y.Step, Z.Step, A.Step, 5V, COM, V+, End Stop X-, End Stop Z+, End Stop Z-, End Stop Y+, End Stop Y-, SpnEN, SpnDir, CoolEn, Abort, Hold, Resume, E-Stop, Y.Motor A+, Y.Motor A-, Y.Motor B+, Y.Motor B-, RST, SDA, SCL, RX, TX, 3V3, X.Motor A+, X.Motor A-, X.Motor B+, X.Motor B-, Z.Motor A+, Z.Motor A-, Z.Motor B+, Z.Motor B-, A.Motor A+, A.Motor A-, A.Motor B+, A.Motor B-, Enable, MS1, MS2, MS3, RESET, Sleep, Step, Direction, VDD, 1B, 1A, 2A, 2B, VMOT, M0, M1, M2, d12, A Drive Step and Direction, Z Drive Module Setp D4, Y Drive Module Setp D3, X Drive Module Setp D2, Set X Drive Module Dir D5, Set Y Drive Module Dir D6, Set Z Aquí tenés la traducción al español y el formato compatible con GitHub Markdown, ideal para un `README.md` o documentación técnica clara y ordenada:

---

## Componentes del Sistema CNC

### 🔹 Motor Paso a Paso NEMA 23

* **Descripción**: Motor paso a paso de alto torque utilizado para el control preciso de los ejes en máquinas CNC.
* **Pines**: `A+`, `A-`, `B+`, `B-`

---

### 🔹 Placa de Expansión CNC Shield V3 para Grabado

* **Descripción**: Placa de expansión que se conecta al Arduino para controlar motores paso a paso y otros periféricos.
* **Pines**:
  `EN`, `GND`, `X.Step`, `DIR`, `Y.Step`, `Z.Step`, `A.Step`, `5V`, `COM`, `V+`
  `End Stop X-`, `End Stop Z+`, `End Stop Z-`, `End Stop Y+`, `End Stop Y-`
  `SpnEN`, `SpnDir`, `CoolEn`, `Abort`, `Hold`, `Resume`, `E-Stop`
  `Y.Motor A+`, `A-`, `B+`, `B-`
  `X.Motor A+`, `A-`, `B+`, `B-`
  `Z.Motor A+`, `A-`, `B+`, `B-`
  `A.Motor A+`, `A-`, `B+`, `B-`
  `RST`, `SDA`, `SCL`, `RX`, `TX`, `3V3`
  `Enable`, `MS1`, `MS2`, `MS3`, `RESET`, `Sleep`, `Step`, `Direction`, `VDD`, `1B`, `1A`, `2A`, `2B`, `VMOT`, `M0`, `M1`, `M2`
  `D12`, `D13`, `D7`, `D6`, `D5`, `D4`, `D3`, `D2`

---

### 🔹 Fuente de Alimentación

* **Descripción**: Proporciona energía eléctrica a la CNC Shield y los componentes conectados.
* **Pines**: `+`, `-`

---

### 🔹 Driver A4988

* **Descripción**: Controlador para motores paso a paso, utilizado para manejar los motores NEMA 23.
* **Pines**:
  `VMOT`, `GND`, `2B`, `2A`, `1A`, `1B`, `VDD`, `EN`, `MS1`, `MS2`, `MS3`,
  `RST`, `SLP`, `STEP`, `DIR`

---

### 🔹 Arduino UNO

* **Descripción**: Microcontrolador principal que gestiona la lógica de control de la máquina CNC.
* **Pines**:
  `IOREF`, `Reset`, `3.3V`, `5V`, `GND`, `Vin`
  `A0` a `A5`, `SCL`, `SDA`, `AREF`
  `D0` a `D13`

---

### 🔹 Sensor AS5600

* **Descripción**: Sensor magnético de posición rotativa, utilizado como realimentación tipo encoder.
* **Pines**: `VCC`, `OUT`, `GND`, `GPO`, `SDA`, `SCL`, `DIR`

---

### 🔹 Multiplexor CD74HC4067

* **Descripción**: Multiplexor/demultiplexor analógico de 16 canales.
* **Pines**:
  `SIG`, `S3`, `S2`, `S1`, `S0`, `EN`, `VCC`, `GND`
  `C0` a `C15`

---

### 🔹 Finales de Carrera (Limit Switch)

* **Descripción**: Detectan el fin de recorrido de los ejes del sistema CNC.
* **Pines**: `C` (común), `NO` (normalmente abierto), `NC` (normalmente cerrado)

---


# Circuitos
<img width="3000" height="2700" alt="circuit_image" src="https://github.com/user-attachments/assets/87880af7-bc9d-4427-abf0-7493b8a86297" />



🗓️ Cronograma Mensual del Proyecto Final (Marzo–Julio 2025)

| Mes           | Investigación (h) | Aplicación (h) | Programación (h) | Total mensual (h) | Actividades principales |
|----------------|--------------------|----------------|-------------------|--------------------|---------------------------|
| March 2025     |               1.6 |            4.7 |               4.2 |               10.5 | Inicio del proyecto, selección de plataforma, motores y encoders AS5600 |
| April 2025     |               1.1 |            3.4 |               3.0 |                7.5 | Implementación multiplexor, pruebas PWM/I2C, brazo paralelo plano |
| May 2025       |               1.1 |            3.4 |               3.0 |                7.5 | Homing, control lazo cerrado, sincronización motores |
| June 2025      |               1.1 |            3.4 |               3.0 |                7.5 | Grabación de trayectorias, visualización en Python, debugging |
| July 2025      |               3.4 |           10.1 |               9.0 |               22.5 | Validación de cinemática , pruebas finales, afinación |

