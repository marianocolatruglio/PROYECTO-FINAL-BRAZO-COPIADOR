// ----------------------------------------------------------------
//  Proyecto Brazo Copiador con AS5600L multi-vuelta
//  - Lectura RAW ANGLE por I2C (registros 0x0C/0x0D)
//  - Conteo multivuelta manual por canal (con multiplexor)
//  - Reproducción por serie con handshake: RDY / OK / DONE
//  - Los ángulos recibidos/guardados son de MOTOR (no aplicar relación)
//  - Velocidad constante objetivo: 45 deg/s
//  - Microstepping: 1/8  -> PASOS_REV = 1600  (0.225°/paso)
//  - Anti-drift: acumulación de resto de pasos + corrección final con encoder
// ----------------------------------------------------------------

#include <Wire.h>
#include "AS5600.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Dirección I2C del AS5600
#define AS5600_ADDR 0x36

// Registros RAW ANGLE (12 bits)
#define RAW1_REG 0x0C  // MSB (bits 11:8)
#define RAW2_REG 0x0D  // LSB (bits 7:0)

// Pines de conexión motor y enable
#define STEP1       2
#define DIR1        5
#define STEP2       3
#define DIR2        6
#define ENABLE_PIN  8   // Activo en LOW en A4988/TMC típicos

// Pines multiplexor (4051)
#define S0 7           // S1/S2 cableadas según tu diseño (si no se usan, a GND/VCC fijas)

// Canales multiplexor
#define CH_ENCODER1  0
#define CH_ENCODER2  1
#define NUM_ENCODERS 2

// Pines finales de carrera
#define LIMIT1_PIN 10
#define LIMIT2_PIN  9

// ---------------------- Parámetros de movimiento -----------------
// Microstepping 1/8  -> 200 * 8 = 1600 pasos/rev
#define PASOS_REV           1600
#define V_DEG_S             45.0f   // velocidad angular objetivo (deg/s)
#define VELOCIDAD_HOMING    3000    // µs entre pasos en homing
#define RETROCESO_HOMING    22
#define DELAY_INTERPOLADO   50      // (no se usa en modo vel. constante)

// Derivados (se calculan en setup)
static float   STEP_ANGLE_DEG   = 360.0f / PASOS_REV;   // 0.225°
static uint32_t MICROS_POR_PASO = 5000UL;               // ~5 ms por paso para 45°/s

// Escalado runtime (1.0 = 45°/s; 2.0 = 90°/s; 0.5 = 22.5°/s)
float  speedFactor = 1.0f;

AS5600 as5600;
volatile bool detenerGrabacion = false;

// --------------------- Estado multivuelta manual -----------------
int32_t  cum[NUM_ENCODERS]     = {0, 0};
uint16_t prevRaw[NUM_ENCODERS] = {0, 0};

// ---------------- Acumulación de resto (anti-redondeo) -----------
static float resto1 = 0.0f, resto2 = 0.0f;
static const float TOL = 0.45f * (360.0f / PASOS_REV);  // ~0.101°: ~½ micro-paso

long pasosDesdeDeltaConResto(float delta_deg, float &resto) {
  float stepsF = delta_deg / STEP_ANGLE_DEG + resto; // = delta_deg * PASOS_REV / 360
  long  steps  = lroundf(stepsF);
  resto = stepsF - (float)steps;                      // guarda resto p/ próximo segmento
  return steps;
}

// ------------------------ Multiplexor ----------------------------
void setCanalMultiplexor(int canal) {
  digitalWrite(S0, canal & 1);
  // Si tuvieras S1/S2: digitalWrite(S1,(canal>>1)&1); digitalWrite(S2,(canal>>2)&1);
  delayMicroseconds(100);
}

// -------------------------- Encoders -----------------------------
uint16_t getRaw(int canal) {
  setCanalMultiplexor(canal);
  delay(1);  // estabilizar línea

  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(RAW1_REG);
  Wire.endTransmission(false);              // mantener el bus
  Wire.requestFrom(AS5600_ADDR, 2);
  while (Wire.available() < 2);
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return ((uint16_t(msb) << 8) | lsb) & 0x0FFF;  // 0..4095
}

float getAngleMulti(int canal) {
  uint16_t raw = getRaw(canal);
  int32_t delta = int32_t(raw) - int32_t(prevRaw[canal]);
  if (delta >  2048) delta -= 4096;   // wrap positivo
  else if (delta < -2048) delta += 4096; // wrap negativo
  cum[canal]     += delta;
  prevRaw[canal]  = raw;
  return cum[canal] * (360.0f / 4096.0f); // grados de MOTOR acumulados
}

// ------------------ Corrección de fin de segmento ----------------
void corregirEjeHasta(float tgt_deg, int ch, int stepPin, int dirPin, uint32_t dt_us) {
  for (uint16_t guard = 0; guard < 2 * PASOS_REV; guard++) {
    float err = tgt_deg - getAngleMulti(ch);
    if (fabsf(err) <= TOL) break;            // dentro de tolerancia
    bool sentido = (err > 0);
    digitalWrite(dirPin, sentido ? HIGH : LOW);
    digitalWrite(stepPin, HIGH); delayMicroseconds(10);
    digitalWrite(stepPin, LOW);  delayMicroseconds(dt_us);
  }
}

// --------------------------- Movimiento --------------------------
void moverInterpolado(float a1_ini, float a2_ini, float a1_fin, float a2_fin, int /*delay_ms*/) {
  long pasos1 = pasosDesdeDeltaConResto(a1_fin - a1_ini, resto1);
  long pasos2 = pasosDesdeDeltaConResto(a2_fin - a2_ini, resto2);

  digitalWrite(DIR1, pasos1 >= 0 ? HIGH : LOW);
  digitalWrite(DIR2, pasos2 >= 0 ? HIGH : LOW);

  pasos1 = labs(pasos1);
  pasos2 = labs(pasos2);
  long maxPasos = (pasos1 > pasos2) ? pasos1 : pasos2;
  if (maxPasos == 0) return;

  // dt ajustado por speedFactor
  uint32_t dt = (uint32_t)( (float)MICROS_POR_PASO / (speedFactor > 0 ? speedFactor : 1.0f) );

  long err1 = 0, err2 = 0;
  for (long i = 0; i < maxPasos; i++) {
    err1 += pasos1;
    if (err1 >= maxPasos) {
      digitalWrite(STEP1, HIGH); delayMicroseconds(10);
      digitalWrite(STEP1, LOW);  err1 -= maxPasos;
    }
    err2 += pasos2;
    if (err2 >= maxPasos) {
      digitalWrite(STEP2, HIGH); delayMicroseconds(10);
      digitalWrite(STEP2, LOW);  err2 -= maxPasos;
    }
    delayMicroseconds(dt);
  }

  // Corrección final guiada por encoder para aterrizar sobre el objetivo
  corregirEjeHasta(a1_fin, CH_ENCODER1, STEP1, DIR1, dt);
  corregirEjeHasta(a2_fin, CH_ENCODER2, STEP2, DIR2, dt);
}

// ---------------------------- Homing -----------------------------
void moverPaso(int stepPin, int dirPin, bool sentido, int pasos, int delay_us) {
  digitalWrite(dirPin, sentido ? HIGH : LOW);
  for (int i = 0; i < pasos; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    delayMicroseconds((uint32_t)delay_us);
  }
}

void homingSimultaneo() {
  digitalWrite(ENABLE_PIN, LOW); // activar drivers
  Serial.println("Iniciando homing...");

  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);

  bool listo1 = false, listo2 = false;
  while (!listo1 || !listo2) {
    if (!listo1 && digitalRead(LIMIT1_PIN)) {
      digitalWrite(STEP1, HIGH); delayMicroseconds(10);
      digitalWrite(STEP1, LOW);  delayMicroseconds((uint32_t)VELOCIDAD_HOMING);
    } else listo1 = true;

    if (!listo2 && digitalRead(LIMIT2_PIN)) {
      digitalWrite(STEP2, HIGH); delayMicroseconds(10);
      digitalWrite(STEP2, LOW);  delayMicroseconds((uint32_t)VELOCIDAD_HOMING);
    } else listo2 = true;
  }

  moverPaso(STEP1, DIR1, HIGH, RETROCESO_HOMING, VELOCIDAD_HOMING);
  moverPaso(STEP2, DIR2, HIGH, RETROCESO_HOMING, VELOCIDAD_HOMING);

  Serial.println("Homing completado.");

  // Re-inicializar nuestro conteo manual
  for (int ch = 0; ch < NUM_ENCODERS; ch++) {
    prevRaw[ch] = getRaw(ch);
    cum[ch]     = 0;
  }

  // Reset de restos para reproducción limpia
  resto1 = resto2 = 0.0f;
}

// --------------- Recepción y reproducción (con handshake) --------
void recibirYReproducirSerial() {
  // Limpiar residuo y anunciar disponibilidad
  while (Serial.available()) Serial.read();
  Serial.println("RDY");

  bool reproduciendo = false;
  const int BUF = 48;
  char buf[BUF];

  float a1_ant = getAngleMulti(CH_ENCODER1);
  float a2_ant = getAngleMulti(CH_ENCODER2);

  // reset de restos al iniciar una reproducción
  resto1 = resto2 = 0.0f;

  uint32_t t0 = millis();
  const uint32_t TO = 30000; // 30 s

  for (;;) {
    if (millis() - t0 > TO) break;

    int len = Serial.readBytesUntil('\n', buf, BUF - 1);
    if (len <= 0) { delay(1); continue; }
    buf[len] = '\0';
    if (len > 0 && buf[len - 1] == '\r') buf[len - 1] = '\0';

    t0 = millis();

    if (!reproduciendo) {
      if (strcmp(buf, "INICIO_TRAYECTORIA") == 0) reproduciendo = true;
      continue;
    }

    if (strcmp(buf, "FIN_TRAYECTORIA") == 0) {
      Serial.println("DONE");
      break;
    }

    // Parsear manual "a1,a2"
    char *comma = strchr(buf, ',');
    if (comma) {
      *comma = '\0';
      float a1 = atof(buf);
      float a2 = atof(comma + 1);
      moverInterpolado(a1_ant, a2_ant, a1, a2, DELAY_INTERPOLADO);
      a1_ant = a1; a2_ant = a2;
      Serial.println("OK"); // ACK por segmento
    }
  }

  Serial.println("Reproducción finalizada.");
  while (Serial.available()) Serial.read();
}

// ---------------------------- Utilidades -------------------------
void grabarDesdeEncoders() {
  detenerGrabacion = false;
  Serial.println("INICIO_TRAYECTORIA");
  while (!detenerGrabacion) {
    if (Serial.available() && Serial.read() == 'x') break;
    float a1 = getAngleMulti(CH_ENCODER1);
    float a2 = getAngleMulti(CH_ENCODER2);
    Serial.print(a1); Serial.print(","); Serial.println(a2);
    delay(50);
  }
  Serial.println("FIN_TRAYECTORIA");
}

void printPosition() {
  float p1 = getAngleMulti(CH_ENCODER1);
  float p2 = getAngleMulti(CH_ENCODER2);
  Serial.print(p1, 2); Serial.print(","); Serial.println(p2, 2);
}

// ------------------------------ Setup/Loop -----------------------
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50); // para readBytesUntil

  Wire.begin();
  as5600.begin();
  as5600.setDirection(AS5600_CLOCK_WISE);
  as5600.resetPosition();

  pinMode(STEP1, OUTPUT); pinMode(DIR1, OUTPUT);
  pinMode(STEP2, OUTPUT); pinMode(DIR2, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);  digitalWrite(ENABLE_PIN, LOW); // habilitar
  pinMode(S0, OUTPUT);

  // Inicializar acumuladores
  for (int ch = 0; ch < NUM_ENCODERS; ch++) {
    prevRaw[ch] = getRaw(ch);
    cum[ch]     = 0;
  }

  // Calcular parámetros de velocidad (1/8 microstep, 45°/s)
  STEP_ANGLE_DEG   = 360.0f / PASOS_REV;                  // 0.225°
  MICROS_POR_PASO  = (uint32_t)((360.0f * 1000000.0f) / (PASOS_REV * V_DEG_S)); // ~5000 µs

  Serial.print("STEP_ANGLE_DEG= "); Serial.println(STEP_ANGLE_DEG, 6);
  Serial.print("MICROS_POR_PASO= "); Serial.println(MICROS_POR_PASO);
  Serial.println("Listo. Esperando comando...");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if      (c == 'h') homingSimultaneo();
  else if (c == 'r') recibirYReproducirSerial();
  else if (c == 'g') grabarDesdeEncoders();
  else if (c == 'x') detenerGrabacion = true;
  else if (c == 'e') digitalWrite(ENABLE_PIN, LOW);
  else if (c == 'd') digitalWrite(ENABLE_PIN, HIGH);
  else if (c == 'p') printPosition();
}
