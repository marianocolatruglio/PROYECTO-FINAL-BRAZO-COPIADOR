#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <AS5600.h>

// Pines de conexión
#define STEP1         2
#define DIR1          5
#define STEP2         3
#define DIR2          6
#define ENABLE_PIN    8
#define LIMIT1_PIN    10
#define LIMIT2_PIN    9
#define S0            7

// Canales del multiplexor
#define CH_ENCODER1   0
#define CH_ENCODER2   1

AS5600 encoder;
AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);

// Homing
const long   distanciaBusqueda  = -10000;
const int    velocidadHoming    = 300;
const int    aceleracionHoming  = 5000;
const int    retrocesoHoming    = 200;

// Encoders multivuelta
int32_t  acc1 = 0, acc2 = 0;
uint16_t prevRaw1 = 0, prevRaw2 = 0;

// Conversión pasos ↔ grados
const float pasosPorRevSalida = 200.0 * 92.0;
const float pasosPorDeg       = pasosPorRevSalida / 360.0;

// Geometría articulada
const float L1 = 50.0;
const float L2 = 150.0;
const float L3 = 100.0;

// Coordenadas home
float xHome = 0, yHome = 0;

// Utilidades
float degToRad(float d){ return d * PI / 180.0; }
float radToDeg(float r){ return r * 180.0 / PI; }

// Selección de canal en el multiplexor
void seleccionarCanal(uint8_t canal) {
  digitalWrite(S0, canal & 0x01);
  delay(2);
}

// Calibración cero de encoders
void calibrarZeroEncoders() {
  seleccionarCanal(CH_ENCODER1); delay(5);
  prevRaw1 = encoder.readAngle(); acc1 = 0;
  seleccionarCanal(CH_ENCODER2); delay(5);
  prevRaw2 = encoder.readAngle(); acc2 = 0;
}

// Lee la posición acumulada de un encoder
float leerPosicionAcumulada(uint8_t canal) {
  uint16_t raw, prev;
  int32_t* acc;
  if (canal == CH_ENCODER1) {
    seleccionarCanal(CH_ENCODER1);
    delay(5);
    raw = encoder.readAngle();
    prev = prevRaw1;
    acc = &acc1;
  } else {
    seleccionarCanal(CH_ENCODER2);
    delay(5);
    raw = encoder.readAngle();
    prev = prevRaw2;
    acc = &acc2;
  }
  int16_t delta = int(raw) - int(prev);
  if (delta >  2048) delta -= 4096;
  if (delta < -2048) delta += 4096;
  *acc += delta;
  if (canal == CH_ENCODER1) prevRaw1 = raw; else prevRaw2 = raw;
  float gradosEncoder = (*acc) * (360.0f / 4096.0f);
  return gradosEncoder / (92.0f / 16.0f);
}

// Rutina de homing simultáneo
void homingSimultaneo() {
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
  while (!m1 || !m2) {
    if (!m1) {
      if (digitalRead(LIMIT1_PIN)) motor1.run();
      else { motor1.stop(); m1 = true; }
    }
    if (!m2) {
      if (digitalRead(LIMIT2_PIN)) motor2.run();
      else { motor2.stop(); m2 = true; }
    }
  }
  delay(100);
  motor1.move(retrocesoHoming);
  motor2.move(retrocesoHoming);
  while (motor1.distanceToGo() || motor2.distanceToGo()) {
    motor1.run();
    motor2.run();
  }
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  calibrarZeroEncoders();
}

// Cinemática directa
bool cinematicaDirecta(float th1_deg, float th2_deg, float &x, float &y) {
  float t1 = degToRad(th1_deg);
  float t2 = degToRad(th2_deg + 20);
  x = L2 * cos(t2) + L3 * cos(PI - t1);
  y = L2 * sin(t2) - L3 * sin(PI - t1);
  return true;
}

// ———————— Gestión de muestreo periódico ————————
const uint16_t TIEMPO_MUESTREO_MS = 10;
unsigned long tUltimaLectura = 0;
float posAcum1 = 0, posAcum2 = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE);

  pinMode(S0, OUTPUT);
  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  motor1.setMaxSpeed(5000);
  motor1.setAcceleration(500);
  motor2.setMaxSpeed(5000);
  motor2.setAcceleration(500);

  xHome = yHome = 0;
  calibrarZeroEncoders();
  Serial.println(F("Listo. 'h'=homing | 'p'=leer áng | 'k'=cin.dir"));
}

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
      case 'h':
        Serial.println(F("Homing..."));
        homingSimultaneo();
        // tras homing, posAcum1/2 se recalcularán en la siguiente iteración
        Serial.println(F("¡Hecho!"));
        break;

      case 'p':
        Serial.print(F("θ1=")); Serial.print(posAcum1,2);
        Serial.print(F("° θ2=")); Serial.print(posAcum2,2);
        Serial.println(F("°"));
        break;

      case 'k': {
        float xr, yr;
        if (cinematicaDirecta(posAcum1, posAcum2, xr, yr)) {
          Serial.print(F("X_rel="));
          Serial.print(xr - xHome,1);
          Serial.print(F(" mm, Y_rel="));
          Serial.print(yr - yHome,1);
          Serial.println(F(" mm"));
        } else {
          Serial.println(F("Error cin.dir"));
        }
        break;
      }
      // puedes desbloquear 'm' aquí si lo necesitas
    }
  }
}
