#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <AS5600.h>

// Pines de conexión
#define STEP1 2
#define DIR1 5
#define STEP2 3
#define DIR2 6
#define ENABLE_PIN 8
#define LIMIT1_PIN 10
#define LIMIT2_PIN 9
#define S0 7

// Canales del multiplexor
#define CH_ENCODER1 0
#define CH_ENCODER2 1

AS5600 encoder;
AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);

// Homing
const long distanciaBusqueda = -10000;
const int velocidadHoming = 300;
const int aceleracionHoming = 5000;
const int retrocesoHoming = 200;

// Encoders multivuelta
int32_t acc1 = 0, acc2 = 0;
uint16_t prevRaw1 = 0, prevRaw2 = 0;

// Conversión pasos ↔ grados
const float pasosPorRevSalida = 200.0 * 88.0;
const float pasosPorDeg = pasosPorRevSalida / 360.0;

// Geometría articulada
const float L1 = 50.0;
const float L2 = 150.0;
const float L3 = 100.0;
float t1a, t2a, t1b, t2b;
// Coordenadas home
float xHome = 0, yHome = 0;

// Utilidades
float degToRad(float d) { return d * PI / 180.0; }
float radToDeg(float r) { return r * 180.0 / PI; }

// Selección de canal en el multiplexor
void seleccionarCanal(uint8_t canal)
{
  digitalWrite(S0, canal & 0x01);
  delay(2);
}

// Calibración cero de encoders
void calibrarZeroEncoders()
{
  seleccionarCanal(CH_ENCODER1);
  delay(5);
  prevRaw1 = encoder.readAngle();
  acc1 = 0;
  seleccionarCanal(CH_ENCODER2);
  delay(5);
  prevRaw2 = encoder.readAngle();
  acc2 = 0;
}

// Lee la posición acumulada de un encoder
float leerPosicionAcumulada(uint8_t canal)
{
  uint16_t raw, prev;
  int32_t *acc;
  if (canal == CH_ENCODER1)
  {
    seleccionarCanal(CH_ENCODER1);
    delay(5);
    raw = encoder.readAngle();
    prev = prevRaw1;
    acc = &acc1;
  }
  else
  {
    seleccionarCanal(CH_ENCODER2);
    delay(5);
    raw = encoder.readAngle();
    prev = prevRaw2;
    acc = &acc2;
  }
  int16_t delta = int(raw) - int(prev);
  if (delta > 2048)
    delta -= 4096;
  if (delta < -2048)
    delta += 4096;
  *acc += delta;
  if (canal == CH_ENCODER1)
    prevRaw1 = raw;
  else
    prevRaw2 = raw;
  float gradosEncoder = (*acc) * (360.0f / 4096.0f);
  return gradosEncoder / (88.0f / 16.0f);
}

// Rutina de homing simultáneo
void homingSimultaneo()
{
  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  motor1.setMaxSpeed(velocidadHoming);
  motor1.setAcceleration(aceleracionHoming);
  motor2.setMaxSpeed(velocidadHoming);
  motor2.setAcceleration(aceleracionHoming);

  motor1.moveTo(distanciaBusqueda);
  motor2.moveTo(distanciaBusqueda);

  bool m1 = false, m2 = false;
  while (!m1 || !m2)
  {
    if (!m1)
    {
      if (digitalRead(LIMIT1_PIN))
        motor1.run();
      else
      {
        motor1.stop();
        m1 = true;
      }
    }
    if (!m2)
    {
      if (digitalRead(LIMIT2_PIN))
        motor2.run();
      else
      {
        motor2.stop();
        m2 = true;
      }
    }
  }
  delay(100);
  motor1.move(retrocesoHoming);
  motor2.move(retrocesoHoming);
  while (motor1.distanceToGo() || motor2.distanceToGo())
  {
    motor1.run();
    motor2.run();
  }
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  calibrarZeroEncoders();
}

// Cinemática directa
bool cinematicaDirecta(float th1_deg, float th2_deg, float &x, float &y)
{
  float t1 = degToRad(th1_deg);
  float t2 = degToRad(th2_deg + 21.2);
  x = -(L2 * cos(t2) + L3 * cos(t1 - PI));
  y = L2 * sin(t2) + L3 * sin(t1 - PI);
  return true;
}

bool cinematicaInversa(float x, float y,
                       float L2, float L3,
                       float &theta1_sol1, float &theta2_sol1,
                       float &theta1_sol2, float &theta2_sol2)
{
  // Distancia radial al punto objetivo
  float r = sqrt(x * x + y * y);
  if (r < 1e-6)
    return false; // Evitar división por cero

  // Parámetro K según deducción:
  //    K = (L2^2 – L3^2 – r^2) / (2⋅L3)
  float K = (L2 * L2 - L3 * L3 - r * r) / (2.0f * L3);

  // cos(φ) = K / r
  float cosPhi = K / r;
  if (cosPhi < -1.0f || cosPhi > 1.0f)
    return false; // Fuera de alcance

  // Ángulo auxiliar φ y δ
  float phi = acos(cosPhi);
  float delta = atan2(y, x);

  // Solución “codo arriba”
  theta1_sol1 = delta + phi;
  // Solución “codo abajo”
  theta1_sol2 = delta - phi;

  // Ahora t2 = atan2(y + L3⋅sin(t1),  x + L3⋅cos(t1))
  theta2_sol1 = atan2(y + L3 * sin(theta1_sol1),
                      x + L3 * cos(theta1_sol1));
  theta2_sol2 = atan2(y + L3 * sin(theta1_sol2),
                      x + L3 * cos(theta1_sol2));

  return true;
}

/**
 * Mueve el brazo hasta la posición (x, y).
 * @param x  Coordenada X deseada [mm]
 * @param y  Coordenada Y deseada [mm]
 * @return   true si el movimiento se completó, false si fuera de alcance
 */
bool moverA(float x, float y) {
  // 1) Calcular cinemática inversa
  float t1_sol1, t2_sol1, t1_sol2, t2_sol2;
  if (!cinematicaInversa(x, y, L2, L3, t1_sol1, t2_sol1, t1_sol2, t2_sol2)) {
    return false;  // fuera de alcance
  }

  // 2) Elegir la solución deseada (aquí “solución 1” codo arriba)
  float t1 = t1_sol1;
  float t2 = t2_sol1;

  // float t1 = t1_sol2;
  // float t2 = t2_sol2;

  // 3) Convertir radianes a grados
  float t1_deg = radToDeg(t1);
  float t2_deg = radToDeg(t2);

  // 4) Calcular pasos absolutos para cada motor
  long pasos1 = lround(t1_deg * pasosPorDeg);
  long pasos2 = lround(t2_deg * pasosPorDeg);

  // 5) Ordenar movimiento absoluto
  motor1.moveTo(pasos1);
  motor2.moveTo(pasos2);

  // 6) Ejecutar hasta llegar
  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
  }

  return true;
}


// ———————— Gestión de muestreo periódico ————————
const uint16_t TIEMPO_MUESTREO_MS = 10;
unsigned long tUltimaLectura = 0;
float posAcum1 = 0, posAcum2 = 0;
void msg()
{
  Serial.println(F("----------------------------------------------------------------------------------------------------------------"));
  Serial.println(F("Comandos disponibles:"));
  Serial.println(F("Listo. 'h'=homing | 'p'=leer áng | 'k'=cin.dir | 'm'=cin.inv | 'e'=habilitar motores | 'd'=deshabilitar motores"));
  Serial.println(F(" 'i X Y'=mover a (X,Y) | 's N'=mover N pasos | 'q'=salir"));
  Serial.println(F(" '1'=mover motor 1 | '2'=mover motor 2"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------"));
  return;
}
void setup()
{
  Serial.begin(115200);
  Wire.begin();
  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE);

  pinMode(S0, OUTPUT);
  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, HIGH);

  motor1.setMaxSpeed(5000);
  motor1.setAcceleration(500);
  motor2.setMaxSpeed(5000);
  motor2.setAcceleration(500);

  xHome = yHome = 0;
  calibrarZeroEncoders();
  msg();
}

  int pasosaux1 = 100;
  int pasosaux2 = 100;
void loop() {
  unsigned long ahora = millis();

  // 1) Muestreo periódico de encoders
  if (ahora - tUltimaLectura >= TIEMPO_MUESTREO_MS) {
    tUltimaLectura = ahora;
    posAcum1 = leerPosicionAcumulada(CH_ENCODER1);
    posAcum2 = leerPosicionAcumulada(CH_ENCODER2);
  }

  // 2) Procesar comandos Serial
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'e': {
        digitalWrite(ENABLE_PIN, LOW);
        Serial.println(F("Motores habilitados"));
        msg();
        break;
      }
      case 'd': {
        digitalWrite(ENABLE_PIN, HIGH);
        Serial.println(F("Motores deshabilitados"));
        msg();
        break;
      }
      case 'h': {
        Serial.println(F("Homing..."));
        homingSimultaneo();
        Serial.println(F("¡Hecho!"));
        msg();
        break;
      }
      case 'p': {
        Serial.print(F("θ1=")); Serial.print(posAcum1, 2);
        Serial.print(F("° θ2=")); Serial.print(posAcum2, 2);
        Serial.println(F("°"));
        msg();
        break;
      }
      case 'k': {
        float xr, yr;
        if (cinematicaDirecta(posAcum1, posAcum2, xr, yr)) {
          Serial.print(F("X_rel=")); Serial.print(xr+39.8, 1);
          Serial.print(F(" mm, Y_rel=")); Serial.print(yr-54.2, 1);
          Serial.println(F(" mm"));
          Serial.print(F("Distancia al origen="));
          Serial.print(sqrt(pow(xr + 39.8, 2) + pow(yr - 54.2, 2)), 1);
          Serial.println(F(" mm"));
        } else {
          Serial.println(F("Error cin.dir"));
        }
        msg();
        break;
      }
      case 'm': {
        String cmd = Serial.readStringUntil('\n');
        float x = cmd.substring(2).toFloat();
        int sep = cmd.indexOf(' ', 3);
        float y = cmd.substring(sep + 1).toFloat();
        // if (cinematicaInversa(x, y, L2, L3, t1a, t2a, t1b, t2b)) {
          Serial.print("Solución 1: t1="); Serial.print(t1a);
          Serial.print("  t2="); Serial.println(t2a);
          Serial.print("Solución 2: t1="); Serial.print(t1b);
          Serial.print("  t2="); Serial.println(t2b);
        // } else {
          // Serial.println("Objetivo fuera de alcance");
        
        msg();
        break;
      }
      case 'i': {
        // Comando “i X Y” para mover a (x,y)
        String cmd = Serial.readStringUntil('\n');
        float x = cmd.substring(2).toFloat();
        int sep = cmd.indexOf(' ', 3);
        float y = cmd.substring(sep + 1).toFloat();
        if (moverA(x, y)) {
          Serial.println("Movimiento completado");
        } else {
          Serial.println("Objetivo fuera de alcance");
        }
        msg();
        break;
      }
      case 's': { 
        digitalWrite(ENABLE_PIN, LOW);
        Serial.println(F("Motores habilitados"));
        Serial.println(F("Mover Cantidad de Pasos:"));
        Serial.print(F("N° de posicion actual:")); Serial.println(motor2.currentPosition());
        int pasos=3200;
        Serial.print(F("Moviendo:  "));
        Serial.print(pasos);
        motor2.moveTo(pasos);
        while (motor2.distanceToGo() != 0) {
          motor2.run();
        }
        
        msg();
        break;
      }
      case '1': {
        digitalWrite(ENABLE_PIN, LOW);
        Serial.println(F("Motores habilitados"));
        Serial.println(F("Mover Motor 1:"));
        Serial.print(F("N° de posicion actual:")); 
        Serial.println(motor1.currentPosition());
        Serial.print(F("Moviendo:  "));
        Serial.print(pasosaux1);
        motor1.moveTo(pasosaux1);
        while (motor1.distanceToGo() != 0) {
          motor1.run();
        }
        pasosaux1=pasosaux1 + 100;

        msg();
        break;  
      }
      case '2': {
        digitalWrite(ENABLE_PIN, LOW);
        Serial.println(F("Motores habilitados"));
        Serial.println(F("Mover Motor 2:"));
        Serial.print(F("N° de posicion actual:")); Serial.println(motor2.currentPosition());
        Serial.print(F("Moviendo:  "));
        Serial.print(pasosaux2);
        motor2.moveTo(pasosaux2);
        while (motor2.distanceToGo() != 0) {
          motor2.run();
        }
        pasosaux2=pasosaux2 + 100;

        msg();
        break;
      }
    } // fin switch
  }
} // fin loop
