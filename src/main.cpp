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
const float L1 = 150.0;
const float L2 = 100.0;
float t1a, t2a;
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



// Lee la posición acumulada de un encoder y la devuelve en pasos
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
  float t2 = degToRad(th2_deg+21.2);
  x = (L1 * cos(t2) + L2 * sin(t1 - (PI / 2))) - 39.8;
  y = (L1 * sin(t2) - L2 * cos(t1 - (PI / 2))) - 54.2;
  return true;
}

// void cinematicaInversa(
//   float x, float y,
//   float l2, float l3,
//   float &q2, float &q3
// ) {
//   // 1) Distancia al punto
//   Serial.print(F("Coordenadas del punto (x,y): "));
//   Serial.print(x, 1);
//   Serial.print(F(", "));
//   Serial.println(y, 1);
//   float r = sqrt(x*x + y*y);
//   Serial.print(F("Distancia al punto r: "));
//   Serial.println(r, 1);
  
//   // 2) Ángulo interno en el codo (ley de cosenos)
//   float cosPhi = ( r*r - l2*l2 - l3*l3) / (2 * l2 * l3);
//   Serial.print(F("Ángulo interno en el codo cosPhi: "));
//   Serial.println(cosPhi, 4);
//   if (isnan(cosPhi)) {
//     Serial.println(F("Error: cosPhi es NaN, punto fuera de alcance."));
//     q2 = q3 = 0; // Asignar valores por defecto
//     return;

//   }
  // Asegurar rango válido para acos()
//   cosPhi = constrain(cosPhi, -1.0, 1.0);
//   float phi = acos(cosPhi);

//   // 3) Ángulo de codo (ángulo exterior)
//   q3 = atan2(-sqrt(1 - cosPhi*cosPhi),(cosPhi*cosPhi));
//   Serial.print(F("Ángulo de codo (q3): ")); Serial.println(radToDeg(q3), 2);

//   // 4) Ángulo polar al punto
//   float beta = atan2(y, x);
//   Serial.print(F("Ángulo polar al punto beta: "));
//   Serial.println(radToDeg(beta), 2);
//   // 5) Desfase producido por el segundo eslabón
//   float alpha = atan2(l3 * sin(q3),
//                       l2 + l3 * cos(q3));
 
//   // 6) Ángulo de hombro
//   q2 = beta + alpha ;
//   Serial.print(F("Ángulo de hombro (q2): "));
//   Serial.println(radToDeg(q2), 2);
// }
#include <math.h>



// Función de cinemática inversa
void cinematicaInversa(float x, float y, float& alpha, float& beta) {
  float r2 = x*x + y*y;
  float r = sqrt(r2);

  // Verifica si el punto es alcanzable
  float alcanceMaximo = L1 + L2;
  float alcanceMinimo = fabs(L1 - L2);
  // if (r > alcanceMaximo || r < alcanceMinimo) {
  //   return false; // No alcanzable
  // }

  // Codo hacia arriba (convención)
  float cosBeta = (L1*L1 + L2*L2 - r2) / (-2 * L1 * L2);
  beta = PI - acos(constrain(cosBeta, -1.0, 1.0)); // importante: limitar dominio

  float cosGamma = (L1*L1 + r2 - L2*L2) / (2 * L1 * r);
  float gamma = acos(constrain(cosGamma, -1.0, 1.0));
  float phi = atan2(y, x);

  alpha = phi + gamma;

  // return true; // Éxito
}


void moverA(float x, float y) {
  float q2, q3;
  // 1) calcular ángulos
  cinematicaInversa(x, y, q2, q3);

  // 2) convertir radianes a pasos
  long steps1 = q2 * (3200.0 * 88.0 / 16.0) / 360.0;
  long steps2 = q3 * (3200.0 * 88.0 / 16.0) / 360.0;

  // 3) planificar objetivo
  motor1.moveTo(steps1);
  motor2.moveTo(steps2);
  
  // 4) ejecutar movimiento 
  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 ) {
    motor1.run();
    motor2.run();
  }

}

bool streaming = false;                  // bandera de transmisión
const uint16_t STREAM_INTERVAL = 10;     // ms entre muestras

void streamEncoderPositions(uint16_t intervalo) {
  Serial.println(F("STREAM_START"));     // marcador de inicio
  unsigned long tPrev = millis();
  while (true) {
    // 1) Leer si llegó la orden de fin ('f')
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'f') {
        Serial.println(F("STREAM_END")); // marcador de fin
        break;
      }
    }
    // 2) Cada intervalo ms, muestro posiciones
    unsigned long tNow = millis();
    if (tNow - tPrev >= intervalo) {
      tPrev = tNow;
      float p1 = leerPosicionAcumulada(CH_ENCODER1);
      float p2 = leerPosicionAcumulada(CH_ENCODER2);
      // Envia "p1 \t p2\n"
      Serial.print(p1, 2);
      Serial.print('\t');
      Serial.println(p2, 2);
      
    }
  }
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
  delay(50);
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
          Serial.print(F("X_rel=")); Serial.print(xr+0,1);
          Serial.print(F(" mm, Y_rel=")); Serial.print(yr, 1);
          Serial.println(F(" mm"));
          Serial.print(F("Distancia al origen="));
          Serial.print(sqrt(pow(xr, 2) + pow(yr, 2)), 1);
          Serial.println(F(" mm"));
        } else {
          Serial.println(F("Error cin.dir"));
        }
        msg();
        break;
      }
      case 'm': {
        String cmd = Serial.readStringUntil('\n');
        float x = cmd.substring(0).toFloat();
        int sep = cmd.indexOf(' ', 3);
        float y = cmd.substring(sep + 1).toFloat();

        (cinematicaInversa(x, y, t1a, t2a));
        Serial.print("Solución 1: theta1="); Serial.print(radToDeg(t1a)+0, 2);
        Serial.print("Solución 2: theta2="); Serial.println(radToDeg(t2a), 2);
        break;
      }
      case 'i': {
        // Comando “i X Y” para mover a (x,y)
        String cmd = Serial.readStringUntil('\n');
        float x = cmd.substring(0).toFloat() ;
        int sep = cmd.indexOf(' ', 3);
        float y = cmd.substring(sep + 1).toFloat() ;
        Serial.print(F("Mover a (X,Y)=(")); Serial.print(x, 1); Serial.print(F(", ")); Serial.print(y, 1); Serial.println(F(")"));   
        moverA(x,y);
        Serial.print(F("Moviendo a (X,Y)=(")); Serial.print(x, 1); Serial.print(F(", ")); Serial.print(y, 1); Serial.println(F(")"));

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

      case 'b': {  // 'b' = begin streaming
        Serial.println(F("Iniciando streaming de posiciones..."));
        streamEncoderPositions(STREAM_INTERVAL);
        msg();
        break;
      }

    } // fin switch
  }
} // fin loop
