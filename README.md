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


🗓️ Cronograma Mensual del Proyecto Final (Marzo–Julio 2025)

| Mes           | Investigación (h) | Aplicación (h) | Programación (h) | Total mensual (h) | Actividades principales |
|----------------|--------------------|----------------|-------------------|--------------------|---------------------------|
| March 2025     |               1.6 |            4.7 |               4.2 |               10.5 | Inicio del proyecto, selección de plataforma, motores y encoders AS5600 |
| April 2025     |               1.1 |            3.4 |               3.0 |                7.5 | Implementación multiplexor, pruebas PWM/I2C, brazo paralelo plano |
| May 2025       |               1.1 |            3.4 |               3.0 |                7.5 | Homing, control lazo cerrado, sincronización motores |
| June 2025      |               1.1 |            3.4 |               3.0 |                7.5 | Grabación de trayectorias, visualización en Python, debugging |
| July 2025      |               3.4 |           10.1 |               9.0 |               22.5 | Validación de cinemática , pruebas finales, afinación |

