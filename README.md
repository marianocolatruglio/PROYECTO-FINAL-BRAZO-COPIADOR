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

# Principio de Funcionamiento 

Se seleccionó como accionamiento 2 motores paso a paso, por su simplicidad y presición de posicionamiento, utilizando una configuracion de multistepping 1/16. Ademas se agregaron encoders magneticos AS5600 que tienen una 
precision de 4096 steps/rev. Estos se utilizaran para obtener la posicion angular de los brazos. Con esta informacion crearemos un streaming de datos se transmitirá via serial mediante pares de coordenadas que se almacenaran en un archivo .TXT a traves de un script de Python. 

## Arduino UNO 

## Resumen del funcionamiento

Este código controla un **brazo copiador** con dos motores paso a paso y encoders magnéticos **AS5600** (lectura multivuelta). Los puntos más relevantes:

1. **Lectura de encoders (AS5600):**

   * Se obtienen ángulos absolutos en crudo (RAW ANGLE) a través de I²C.
   * Se implementa un contador manual de vueltas para cada encoder usando un multiplexor 4051.

2. **Control de motores:**

   * Los motores paso a paso se manejan mediante drivers tipo A4988/TMC, con microstepping configurado a **1/8** (1600 pasos/rev, 0.225°/paso).
   * Se implementa un **movimiento interpolado** con velocidad constante de **45°/s**, ajustable con `speedFactor`.
   * Al finalizar un segmento, se aplica **corrección de posición** usando el feedback de los encoders (anti-drift).

3. **Homing automático:**

   * Rutina `homingSimultaneo()` para buscar finales de carrera, retroceder y re-inicializar el conteo de ángulos.

4. **Grabación y reproducción de trayectorias:**

   * **Grabación:** lee continuamente los ángulos de los encoders y los envía por puerto serie (`g`).
   * **Reproducción:** recibe una secuencia de ángulos (`INICIO_TRAYECTORIA` ... `FIN_TRAYECTORIA`) por serie y ejecuta los movimientos de forma sincronizada (`r`).
   * Comunicación con **handshake**: el Arduino responde con `RDY`, `OK` por cada segmento y `DONE` al finalizar.

5. **Comandos por serie:**

   * `h`: homing
   * `g`: iniciar grabación
   * `r`: reproducir trayectoria
   * `x`: detener grabación
   * `p`: imprimir posición actual
   * `e` / `d`: habilitar o deshabilitar drivers

6. **GUI - INTERFAZ GRAFICA**
   
   ![ ](https://github.com/user-attachments/assets/4d090cfe-190c-4049-bfff-628752775015)

8. **Detalles técnicos:**

   * Corrección de errores por redondeo en pasos mediante acumulación de restos.
   * Tiempo por paso calculado dinámicamente para mantener la velocidad angular deseada.
   * Tolerancia de corrección final de \~0.1° (medio micropaso).

## Morfologia del robot.
  Se utilizo una configuracion de brazos planos paralelos con ambos accionamientos sobre el mismo eje.
  Esto se decidió con el objetivo de reducir el peso en el extremo del robot, con el ojetivo de alcanzar extensiones largas con una inercia minima.
  La reduccion de los motores es de 88/16, mediante correas GT2, las cuales se imprimieron en 3D con filamento flexible TPU.
  De esta forma se agrupo toda la transmision en la base del dispositivo donde se concentra la mayor parte del peso.
  

## Encoders AS5600

Estos encoders utilizan una comunicacion I2C, con el problema que tienen una direccion estatica 0x36. lo cual nos obligo a utilizar un multiplexor, segun recomendacion del creador de la Libreria (https://github.com/RobTillaart/AS5600) utilizamos el siguiente : CD74HC4067. De esta forma podemos obtener datos de ambos encoders simultaneamente.

## CNC Shield Protoneer v3.0

La placa de expansión para máquina de grabado CNC Shield V3 de Protoneer es una placa de expansión versátil y potente diseñada para controlar motores paso a paso en máquinas CNC. Se utiliza comúnmente en routers CNC, cortadoras láser y máquinas de grabado DIY. La placa es compatible con Arduino UNO y utiliza firmware GRBL para controlar hasta cuatro motores paso a paso, lo que permite un funcionamiento preciso y eficiente de la máquina.
En nuestro proyecto la utilizamos con una configuracion personalizada para utilizar los pines disponibles para conectar los encoders, el multiplexor y los fines de carrera.

## Homing

Es necesario darle un origen de referencia al sistema, para inicializar la posicion, entonces se debio programar una rutina de homing. Utilizamos 2 microswitchs para ello, los cuales se accionan al llevar los brazos al origen.


# Componentes del Sistema 

### 🔹 Motor Paso a Paso NEMA 23

* **Descripción**: Motor paso a paso de alto torque utilizado para el control preciso de los ejes en máquinas CNC.
---

### 🔹 Placa de Expansión CNC Shield V3 para Grabado

* **Descripción**: Placa de expansión que se conecta al Arduino para controlar motores paso a paso y otros periféricos.
---

### 🔹 Fuente de Alimentación

* **Descripción**: Proporciona energía eléctrica a la CNC Shield y los componentes conectados.
---

### 🔹 Driver A4988

* **Descripción**: Controlador para motores paso a paso, utilizado para manejar los motores NEMA 23.
---

### 🔹 Arduino UNO

* **Descripción**: Microcontrolador principal que gestiona la lógica de control de la máquina CNC.
---

### 🔹 Sensor AS5600

* **Descripción**: Sensor magnético de posición rotativa, utilizado como realimentación tipo encoder.
---

### 🔹 Multiplexor CD74HC4067

* **Descripción**: Multiplexor/demultiplexor analógico de 16 canales.
---

### 🔹 Finales de Carrera (Limit Switch)

* **Descripción**: Detectan el fin de recorrido de los ejes del sistema CNC.
---


# Circuitos
<img width="3000" height="2700" alt="circuit_image" src="https://github.com/user-attachments/assets/87880af7-bc9d-4427-abf0-7493b8a86297" />


# Fabricacion del prototipo

Para la fabricacion del prototipo utilizamos el software de modelado parametrico Solidworks, donde se modelaron integramente las piezas mecanicas, para luego ser impresas mediante el proceso de FDM.
Se utilizó una impresora Anycubic Kobra Neo 2 y las piezas se imprimieron con filamento PLA. Las correas GT2 se imprimieron con filamente FLEX - TPU.

# Prueba - VIDEO

https://github.com/user-attachments/assets/5e62bf16-36d7-4c9c-b1ba-a70892847485


# Despiece - VIDEO


https://github.com/user-attachments/assets/c534c029-4edf-490a-99b5-7e7f0a055c2a



# Vistas del Modelo

<p align="center">
  <img src="https://github.com/user-attachments/assets/7a986e9b-1f47-4f09-8889-d5921a0ae0a2" width="400" alt="image 1"/>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/3eb8262e-3cf1-4530-b942-628630ac113b" width="400" alt="image 2"/>
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/e660af4a-de8a-4e27-9187-dc3bb7d53a65" width="400" alt="image 3"/>
</p>




# 🗓️ Cronograma Mensual del Proyecto Final (Marzo–Julio 2025)

| Mes           | Investigación (h) | Aplicación (h) | Programación (h) | Total mensual (h) | Actividades principales |
|----------------|--------------------|----------------|-------------------|--------------------|---------------------------|
| March 2025     |               1.6 |            4.7 |               4.2 |               10.5 | Inicio del proyecto, selección de plataforma, motores y encoders AS5600 |
| April 2025     |               1.1 |            3.4 |               3.0 |                7.5 | Implementación multiplexor, pruebas PWM/I2C, brazo paralelo plano |
| May 2025       |               1.1 |            3.4 |               3.0 |                7.5 | Homing, control lazo cerrado, sincronización motores |
| June 2025      |               1.1 |            3.4 |               3.0 |                7.5 | Grabación de trayectorias, visualización en Python, debugging |
| July 2025      |               3.4 |           10.1 |               9.0 |               22.5 | Validación de cinemática , pruebas finales, afinación |

