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
#define S1            8
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
const float L1 = 50.0;     // actuadorLen1
const float L2 = 150.0;     // actuadorLen2
const float L3 = 100.0;     // couplerLen1 (eslabón final)


// Coordenadas home
float xHome = 0, yHome = 0;

// Utilidades
float degToRad(float d){ return d * PI / 180.0; }
float radToDeg(float r){ return r * 180.0 / PI; }

void seleccionarCanal(uint8_t canal) {
  digitalWrite(S0, (canal >> 0) & 1);
  digitalWrite(S1, (canal >> 1) & 1);
}

void calibrarZeroEncoders() {
  seleccionarCanal(CH_ENCODER1); delay(5);
  prevRaw1 = encoder.readAngle(); acc1 = 0;
  seleccionarCanal(CH_ENCODER2); delay(5);
  prevRaw2 = encoder.readAngle(); acc2 = 0;
}

float leerPosicionAcumulada(uint8_t canal) {
  uint16_t raw, prev;
  int32_t* acc;
  if (canal==CH_ENCODER1) {
    seleccionarCanal(CH_ENCODER1); 
    delay(5);
    raw = encoder.readAngle(); prev = prevRaw1; acc=&acc1;
  } else {
    seleccionarCanal(CH_ENCODER2); 
    delay(5);
    raw = encoder.readAngle(); prev = prevRaw2; acc=&acc2;
  }
  int16_t delta = int(raw) - int(prev);
  if (delta>2048)  delta -= 4096;
  if (delta<-2048) delta += 4096;
  *acc += delta;
  if (canal==CH_ENCODER1) prevRaw1=raw; else prevRaw2=raw;
  float motorDeg = (*acc) * (360.0f/4096.0f);
  return motorDeg / (92.0f/16.0f);
}

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
  bool m1=false, m2=false;
  while(!m1||!m2){
    if(!m1){
      if(digitalRead(LIMIT1_PIN)) motor1.run();
      else { motor1.stop(); m1=true; }
    }
    if(!m2){
      if(digitalRead(LIMIT2_PIN)) motor2.run();
      else { motor2.stop(); m2=true; }
    }
  }
  delay(100);
  motor1.move(retrocesoHoming);
  motor2.move(retrocesoHoming);
  while(motor1.distanceToGo()||motor2.distanceToGo()){
    motor1.run(); motor2.run();
  }
  motor1.setCurrentPosition(0);
  motor2.setCurrentPosition(0);
  calibrarZeroEncoders();
}

// Cinemática directa th1, th2 → X, Y
bool cinematicaDirecta(float th1_deg, float th2_deg, float &x, float &y) {
  float t1 = degToRad(th1_deg);
  float t2 = degToRad(th2_deg);
  x = L2 * cos(t2) + (L3) * cos(PI-t1);
  y = L2 * sin(t2) + (L3) * sin(PI-t1);
  return true;
}

// // Cinemática inversa X, Y → th1, ht2
// bool cinematicaInversa(float x, float y, float &th1_deg, float &th2_deg) {
//   float D = (sq(x) + sq(y) - sq(L1) - sq(L23)) / (2 * L1 * L23);
//   if (D < -1.0 || D > 1.0) return false;
//   float t2 = acos(D);
//   float k1 = L1 + L23 * cos(t2);
//   float k2 = L23 * sin(t2);
//   float t1 = atan2(y, x) - atan2(k2, k1);
//   th1_deg = radToDeg(t1);
//   th2_deg = radToDeg(t2);
//   return true;
// }

// void moveToXY(float Xrel, float Yrel) {
//   float xt = xHome + Xrel;
//   float yt = yHome + Yrel;
//   float t1, t2;
//   if(!cinematicaInversa(xt, yt, t1, t2)) {
//     Serial.println(F("Error: fuera de alcance"));
//     return;
//   }
//   long s1 = lround(t1 * pasosPorDeg);
//   long s2 = lround(t2 * pasosPorDeg);
//   motor1.moveTo(s1);
//   motor2.moveTo(s2);
//   while(motor1.distanceToGo()||motor2.distanceToGo()){
//     motor1.run(); 
//     motor2.run();
//   }
// }

void setup() {
  Serial.begin(115200);
  Wire.begin();          
  encoder.begin();
  encoder.setDirection(AS5600_CLOCK_WISE);
  pinMode(S0,OUTPUT);    
  pinMode(S1,OUTPUT);
  pinMode(LIMIT1_PIN,INPUT_PULLUP);
  pinMode(LIMIT2_PIN,INPUT_PULLUP);
  pinMode(ENABLE_PIN,OUTPUT); 
  digitalWrite(ENABLE_PIN,LOW);

  motor1.setMaxSpeed(5000);  
  motor1.setAcceleration(500);
  motor2.setMaxSpeed(5000);  
  motor2.setAcceleration(500);

  xHome = yHome = 0;
  Serial.print(F("Home→ X=")); Serial.print(xHome,1);
  Serial.print(F(" mm, Y=")); Serial.print(yHome,1);
  Serial.println(F(" mm"));
  Serial.println(F("Listo. 'h'=homing | 'p'=leer áng | 'k'=cin.dir | 'm X Y'=mover"));
}

void loop() {
  if(!Serial.available()) return;
  char c = Serial.read();
  if(c=='h') {
    Serial.println(F("Homing..."));
    homingSimultaneo();
    float a1 = leerPosicionAcumulada(CH_ENCODER1);
    float a2 = leerPosicionAcumulada(CH_ENCODER2);
    if (cinematicaDirecta(a1, a2, xHome, yHome)) {
      Serial.print(F("Home calibrado → X="));
      Serial.print(xHome,1);
      Serial.print(F(" mm, Y="));
      Serial.print(yHome,1);
      Serial.println(F(" mm"));
    } else {
      Serial.println(F("ERROR calibrar home: fuera de rango"));
    }
    Serial.println(F("¡Hecho!"));
  }
  if(c=='p') {
    float a1 = leerPosicionAcumulada(CH_ENCODER1);
    float a2 = leerPosicionAcumulada(CH_ENCODER2);
    Serial.print(F("θ1=")); Serial.print(a1,2);
    Serial.print(F("° θ2=")); Serial.print(a2,2);
    Serial.println(F("°"));
  }
  if(c=='k') {
    float a1 = leerPosicionAcumulada(CH_ENCODER1);
    float a2 = leerPosicionAcumulada(CH_ENCODER2);
    float xr, yr;
    if(cinematicaDirecta(a1,a2,xr,yr)) {
      Serial.print(F("X_rel=")); 
      Serial.print(xr - xHome,1);
      Serial.print(F(" mm, Y_rel=")); 
      Serial.print(yr - yHome,1);
      Serial.println(F(" mm"));
    } else {
      Serial.println(F("Error cin.dir"));
    }
  }
// if (c == 'm') {
//   while (!Serial.available());  // Espera que haya datos

//   String linea = Serial.readStringUntil('\n');  // Lee toda la línea enviada
//   float Xr = 0, Yr = 0;
//   int sep = linea.indexOf(' ');
//   if (sep != -1) {
//     Xr = linea.substring(0, sep).toFloat();
//     Yr = linea.substring(sep + 1).toFloat();
//     moveToXY(Xr, Yr);
//     Serial.print(F("Moviendo a X=")); Serial.print(Xr,1);
//     Serial.print(F(" mm, Y=")); Serial.print(Yr,1);
//     Serial.println(F(" → Movimiento completado"));
//   } else {
//     Serial.println(F("Error: formato inválido. Usar 'm 100 50'"));
//   }

// }
}
