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

## Componentes del Sistema 

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

# Principio de Funcionamiento 

Se seleccionó como accionamiento 2 motores paso a paso, por su simplicidad y presición de posicionamiento, utilizando una configuracion de multistepping 1/16. Ademas se agregaron encoders magneticos AS5600 que tienen una 
precision de 4096 steps/rev. Estos se utilizaran para obtener la posicion angular de los brazos. Con esta informacion crearemos un streaming de datos se transmitirá via serial mediante pares de coordenadas que se almacenaran en un archivo .TXT a traves de un script de Python. 

## Morfologia del robot.
  Se utilizo una configuracion de brazos planos paralelos con ambos accionamientos sobre el mismo eje.
  Esto se decidió con el objetivo de reducior el peso en el extremo del robot, con el ojetivo de alcanzar extensiones largas con una inercia minima.
  La reduccion de los motores es de 88/16, mediante correas GT2, las cuales se imprimieron en 3D con filamento flexible TPU.
  De esta forma se agrupo toda la transmision en la base del dispositivo donde se concentra la mayor parte del peso.
  

## Encoders AS5600

Estos encoders utilizan una comunicacion I2C, con el problema que tienen una direccion estatica 0x36. lo cual nos obligo a utilizar un multiplexor, segun recomendacion de del creador de la Libreria (https://github.com/RobTillaart/AS5600) utilizamos el siguiente : CD74HC4067. De esta forma podemos obtener de ambos encoders casi simultaneamente.

## CNC Shield Protoneer v3.0

La placa de expansión para máquina de grabado CNC Shield V3 de Protoneer es una placa de expansión versátil y potente diseñada para controlar motores paso a paso en máquinas CNC. Se utiliza comúnmente en routers CNC, cortadoras láser y máquinas de grabado DIY. La placa es compatible con Arduino UNO y utiliza firmware GRBL para controlar hasta cuatro motores paso a paso, lo que permite un funcionamiento preciso y eficiente de la máquina.
En nuestro proyecto la utilizamos con una configuracion personalizada para utilizar los pines disponibles para conectar los encoders, el multiplexor y los fines de carrera.

## Homing

Es necesario darle un origen de referencia al sistema, para inicializar la posicion, entonces se debio programar una rutina de homing. Utilizamos 2 microswitchs para ello, los cuales se accionan al llevar los brazos al origen.






























🗓️ Cronograma Mensual del Proyecto Final (Marzo–Julio 2025)

| Mes           | Investigación (h) | Aplicación (h) | Programación (h) | Total mensual (h) | Actividades principales |
|----------------|--------------------|----------------|-------------------|--------------------|---------------------------|
| March 2025     |               1.6 |            4.7 |               4.2 |               10.5 | Inicio del proyecto, selección de plataforma, motores y encoders AS5600 |
| April 2025     |               1.1 |            3.4 |               3.0 |                7.5 | Implementación multiplexor, pruebas PWM/I2C, brazo paralelo plano |
| May 2025       |               1.1 |            3.4 |               3.0 |                7.5 | Homing, control lazo cerrado, sincronización motores |
| June 2025      |               1.1 |            3.4 |               3.0 |                7.5 | Grabación de trayectorias, visualización en Python, debugging |
| July 2025      |               3.4 |           10.1 |               9.0 |               22.5 | Validación de cinemática , pruebas finales, afinación |

