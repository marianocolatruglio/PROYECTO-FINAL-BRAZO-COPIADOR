#include <Wire.h>
#include "AS5600.h"
#include <AccelStepper.h>

#define S0 7
#define CH_ENCODER1 0
#define CH_ENCODER2 1

#define DIR1 5
#define STEP1 2
#define DIR2 6
#define STEP2 3

AS5600 encoder;

// Motores
AccelStepper motor1(AccelStepper::DRIVER, STEP1, DIR1);
AccelStepper motor2(AccelStepper::DRIVER, STEP2, DIR2);

int16_t angulo_prev[2] = {0, 0};
int32_t vueltas[2] = {0, 0};
int16_t offset[2] = {0, 0};

const int16_t LIMITE = 4096;
const int16_t UMBRAL = 2048;

const float TICKS_A_GRADOS = 360.0 / 4096.0;
const float RELACION_TRANSMISION = 92.0 / 16.0;

void seleccionarCanal(uint8_t canal) {
  digitalWrite(S0, canal & 0x01);
  delay(2);
}

int16_t leerAngulo(uint8_t canal) {
  seleccionarCanal(canal);
  return encoder.readAngle();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(S0, OUTPUT);

  encoder.begin();
  encoder.setDirection(AS5600_COUNTERCLOCK_WISE);

  for (uint8_t i = 0; i < 2; i++) {
    offset[i] = leerAngulo(i);
    angulo_prev[i] = 0;
    vueltas[i] = 0;
  }

  // Configuración de motores
  motor1.setMaxSpeed(1000);
  motor1.setAcceleration(500);
  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(500);

  Serial.println("Haciendo HOME...");
  Serial.println("Listo para recibir comandos. Ej: MOVER 1 200");
}

void loop() {
  // Ejecutar motores si tienen movimiento pendiente
  motor1.run();
  motor2.run();

  // Leer comandos del Serial
  if (Serial.available()) {
    String linea = Serial.readStringUntil('\n');
    linea.trim();
    if (linea.startsWith("MOVER")) {
      int numMotor, pasos, grados;
      sscanf(linea.c_str(), "MOVER %d %d", &numMotor, &grados);
      pasos=grados*(3200/360)*RELACION_TRANSMISION;
      if (numMotor == 1) {
        motor1.move(-pasos);
      } else if (numMotor == 2) {
        motor2.move(-pasos);
      }
      Serial.print("Moviendo motor ");
      Serial.print(numMotor);
      Serial.print(" a ");
      Serial.print(pasos);
      Serial.println(" pasos.");
    }
  }

  // Mostrar lectura de encoders
  static uint32_t t_prev = 0;
  if (millis() - t_prev >= 500) {
    t_prev = millis();

    for (uint8_t i = 0; i < 2; i++) {
      int16_t angulo_bruto = leerAngulo(i);
      int16_t angulo_actual = (angulo_bruto - offset[i] + LIMITE) % LIMITE;

      int16_t delta = angulo_actual - angulo_prev[i];
      if (delta > UMBRAL) vueltas[i]--;
      else if (delta < -UMBRAL) vueltas[i]++;

      angulo_prev[i] = angulo_actual;
      int32_t ticks_acumulados = vueltas[i] * LIMITE + angulo_actual;
      float angulo_salida = (ticks_acumulados * TICKS_A_GRADOS) / RELACION_TRANSMISION;

      Serial.print("Encoder ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(angulo_salida, 2);
      Serial.println("°");
    }

    Serial.println("-----------------------------");
  }
}
