#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <math.h>

/*
  ZO-ROS - Firmware ESP32-S3 integrado corregido

  Incluye:
  - Comunicación USB Serial con Raspberry/ROS2.
  - Control manual por joystick: V <linear> <angular>.
  - Modo automático con bumpers y ultrasonidos como el código que funcionaba.
  - ESC1/ESC2 para brushless inferiores.
  - ADS1015 para batería.
  - IMU LSM6DSO32TR para alarma de levantamiento y avance recto.
  - JSON periódico para /robot/status.

  Correcciones principales:
  - Ultrasonidos sí participan directamente en AUTO: frontal, izquierdo y derecho.
  - La detección de levantamiento en AUTO es menos sensible para evitar falsos positivos por vibración.
  - El joystick no se corta tan fácil: timeout mayor y lectura serial dentro de delays.

  Pinout según esquemático enviado:
  I2C: SDA GPIO1, SCL GPIO2
  BUMPER1 GPIO8  = derecho
  BUMPER2 GPIO3  = frontal
  BUMPER3 GPIO9  = izquierdo
  BUMPER4 GPIO10 = trasero
  HCSR04_1 izquierdo = TRIG GPIO21 / ECHO GPIO45
  HCSR04_2 frontal   = TRIG GPIO13 / ECHO GPIO14
  HCSR04_3 derecho   = TRIG GPIO11 / ECHO GPIO12
  DIR1/PWM1 motor derecho   = GPIO15 / GPIO16
  DIR2/PWM2 motor izquierdo = GPIO17 / GPIO18
  ESC1 izquierdo = GPIO7
  ESC2 derecho   = GPIO6
  ESC3 bordeadora = GPIO5, no usado aquí
  Buzzer = GPIO48
*/

// =========================================================
// PINOUT
// =========================================================
#define I2C_SDA 1
#define I2C_SCL 2

#define BUMPER_DER    8
#define BUMPER_FRONT  3
#define BUMPER_IZQ    9
#define BUMPER_TRAS   10

#define TRIG_IZQ    21
#define ECHO_IZQ    47
#define TRIG_FRONT  13
#define ECHO_FRONT  14
#define TRIG_DER    11
#define ECHO_DER    12

#define DIR_DER 15
#define PWM_DER 16
#define DIR_IZQ 17
#define PWM_IZQ 18

#define DIR_DER_ADELANTE HIGH
#define DIR_DER_ATRAS    LOW
#define DIR_IZQ_ADELANTE HIGH
#define DIR_IZQ_ATRAS    LOW

#define ESC_IZQ        7
#define ESC_DER        6
#define ESC_BORDEADORA 5

#define BUZZER 48

// =========================================================
// ADS1015 - BATERÍA
// =========================================================
Adafruit_ADS1015 ads;
#define ADS_ADDR 0x48

const float R18_SUP = 100000.0f;
const float R19_INF = 22000.0f;
const float FACTOR_DIVISOR = (R18_SUP + R19_INF) / R19_INF;
const int NUM_MUESTRAS_ADC = 20;
const float ADS1015_MV_POR_BIT = 2.0f;

const float BATTERY_LOW_VOLTAGE = 13.0f;
const float BATTERY_CRITICAL_VOLTAGE = 12.8f;
const float BATTERY_HYSTERESIS = 0.25f;
const float BATTERY_EMPTY_VOLTAGE = 13.0f;
const float BATTERY_FULL_VOLTAGE = 16.8f;

bool adsOk = false;
float batteryVoltage = 0.0f;
float batteryPercent = 0.0f;
bool batteryLow = false;
bool batteryCritical = false;

unsigned long lastBatteryBeep = 0;
const unsigned long BATTERY_BEEP_PERIOD_MS = 5000;

// =========================================================
// IMU LSM6DSO32TR
// =========================================================
#define IMU_ADDR 0x6A

#define REG_WHO_AM_I  0x0F
#define REG_CTRL1_XL  0x10
#define REG_CTRL2_G   0x11
#define REG_CTRL3_C   0x12
#define REG_OUTX_L_G  0x22
#define REG_OUTX_L_A  0x28

#define ACC_SENS_MG_PER_LSB      0.122f
#define MG_TO_MS2                0.00980665f
#define GYR_SENS_DPS_PER_LSB     0.0175f

bool imuOk = false;

float ax_ms2 = 0.0f;
float ay_ms2 = 0.0f;
float az_ms2 = 0.0f;
float gx_dps = 0.0f;
float gy_dps = 0.0f;
float gz_dps = 0.0f;
float gzBias_dps = 0.0f;

// Si en reposo AZ está cerca de +9.8, deja 1. Si está cerca de -9.8, cambia a 0.
#define DETECT_LIFT_POSITIVE_Z 1

// Umbrales corregidos. El modo AUTO es menos sensible por vibraciones.
const float LIFT_Z_THRESHOLD_MANUAL = 14.0f;
const float LIFT_Z_THRESHOLD_AUTO = 20.0f;
const int LIFT_CONFIRM_MANUAL = 4;
const int LIFT_CONFIRM_AUTO = 14;

const unsigned long IGNORE_LIFT_AFTER_AUTO_START_MS = 2500;
const unsigned long IGNORE_LIFT_AFTER_MANEUVER_MS = 1200;

int liftCounter = 0;
bool liftDetected = false;
bool resumeAutoWhenStable = false;

unsigned long lastAutoStartMs = 0;
unsigned long lastManeuverMs = 0;

const float STABLE_Z_TARGET = 9.81f;
const float STABLE_Z_TOL = 2.8f;
const float STABLE_GYRO_MAX_DPS = 10.0f;
const unsigned long STABLE_REQUIRED_MS = 5000;
unsigned long stableStartMs = 0;

float yawDeg = 0.0f;
float yawRefDeg = 0.0f;
unsigned long lastImuUpdateMicros = 0;

// Si al avanzar recto corrige hacia el lado equivocado, cambia a -1.
#define IMU_STEER_SIGN 1

const float YAW_KP = 2.2f;
const float YAW_KD = 0.30f;
const int MAX_IMU_CORRECTION = 25;

// =========================================================
// ESC / MOVIMIENTO
// =========================================================
const int ESC_STOP_US = 1000;
const int ESC_RUN_US  = 1350;
bool cortadoresActivos = false;

const int PWM_MAX_MANUAL = 150;

const float APP_MAX_LINEAR = 0.30f;
const float APP_MAX_ANGULAR = 0.85f;

int velocidadAvance = 150;
int velocidadLenta = 110;
int velocidadGiro = 150;
int velocidadRetroceso = 140;

const int distanciaFrontalParada = 25;
const int distanciaLateralAlerta = 22;
const int distanciaPrecaucion = 40;

const unsigned long tiempoRetroceso = 700;
const unsigned long tiempoGiro = 650;
const unsigned long tiempoCorreccionLateral = 280;

const unsigned long TIEMPO_ARMADO_ESC = 8000;
const unsigned long DURACION_AUTO = 60000;

const unsigned long TIMEOUT_MANUAL_MS = 1400;
const unsigned long STATUS_PERIOD_MS = 250;
const unsigned long SENSOR_PERIOD_AUTO_MS = 180;
const unsigned long SENSOR_PERIOD_MANUAL_MS = 900;
const unsigned long BATTERY_PERIOD_MS = 1000;
const unsigned long IMU_PERIOD_MS = 50;

float dIzq = -1.0f;
float dFront = -1.0f;
float dDer = -1.0f;

enum RobotMode {
  MODE_MANUAL,
  MODE_AUTO,
  MODE_STOPPED
};

RobotMode modoActual = MODE_MANUAL;

bool autoActivo = false;
bool autoTerminado = false;

unsigned long inicioAuto = 0;
unsigned long ultimoComandoManual = 0;
unsigned long ultimoStatus = 0;
unsigned long ultimoSensor = 0;
unsigned long ultimoBattery = 0;
unsigned long ultimoImu = 0;

String lineaSerial = "";

// =========================================================
// DECLARACIONES
// =========================================================
void servicioESC();
void delayConESC(unsigned long tiempoMs);
void detenerMotoresDC();
void detenerTodo();
void iniciarAutomatico();
void detenerAutomatico();
void actualizarIMU();
void actualizarBateria();
void alarmaLevantamiento();
void beepBateriaBaja();
void beepBateriaCritica();
void resetYawReferencia();
void leerSerial();
void actualizarDistancias();

// =========================================================
// UTILIDADES I2C
// =========================================================
void scanI2C() {
  Serial.println("Escaneando bus I2C...");

  int devices = 0;

  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("Dispositivo I2C encontrado en 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      devices++;
    }
  }

  if (devices == 0) {
    Serial.println("No se encontraron dispositivos I2C.");
  }

  Serial.println();
}

// =========================================================
// IMU
// =========================================================
uint8_t imuReadRegister(uint8_t reg) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(IMU_ADDR, (uint8_t)1);

  if (Wire.available()) {
    return Wire.read();
  }

  return 0xFF;
}

void imuWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t imuReadInt16(uint8_t regLow) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(regLow);
  Wire.endTransmission(false);

  Wire.requestFrom(IMU_ADDR, (uint8_t)2);

  uint8_t low = 0;
  uint8_t high = 0;

  if (Wire.available()) low = Wire.read();
  if (Wire.available()) high = Wire.read();

  return (int16_t)((high << 8) | low);
}

bool setupIMU() {
  uint8_t whoami = imuReadRegister(REG_WHO_AM_I);

  Serial.print("IMU WHO_AM_I = 0x");
  if (whoami < 16) Serial.print("0");
  Serial.println(whoami, HEX);

  if (whoami == 0x00 || whoami == 0xFF) {
    Serial.println("ERROR: IMU no responde. Seguridad y control recto por IMU desactivados.");
    return false;
  }

  imuWriteRegister(REG_CTRL3_C, 0x01);
  delay(100);

  imuWriteRegister(REG_CTRL3_C, 0x44);
  delay(20);

  imuWriteRegister(REG_CTRL1_XL, 0x48);
  delay(20);

  imuWriteRegister(REG_CTRL2_G, 0x44);
  delay(100);

  lastImuUpdateMicros = micros();

  Serial.println("IMU configurado correctamente.");
  return true;
}

void leerIMUCrudoYConvertir() {
  int16_t gx_raw = imuReadInt16(REG_OUTX_L_G);
  int16_t gy_raw = imuReadInt16(REG_OUTX_L_G + 2);
  int16_t gz_raw = imuReadInt16(REG_OUTX_L_G + 4);

  int16_t ax_raw = imuReadInt16(REG_OUTX_L_A);
  int16_t ay_raw = imuReadInt16(REG_OUTX_L_A + 2);
  int16_t az_raw = imuReadInt16(REG_OUTX_L_A + 4);

  ax_ms2 = ax_raw * ACC_SENS_MG_PER_LSB * MG_TO_MS2;
  ay_ms2 = ay_raw * ACC_SENS_MG_PER_LSB * MG_TO_MS2;
  az_ms2 = az_raw * ACC_SENS_MG_PER_LSB * MG_TO_MS2;

  gx_dps = gx_raw * GYR_SENS_DPS_PER_LSB;
  gy_dps = gy_raw * GYR_SENS_DPS_PER_LSB;
  gz_dps = (gz_raw * GYR_SENS_DPS_PER_LSB) - gzBias_dps;
}

void calibrarGyroZ() {
  if (!imuOk) return;

  Serial.println("Calibrando bias del giroscopio Z. No mover el robot...");

  const int N = 120;
  float suma = 0.0f;

  for (int i = 0; i < N; i++) {
    int16_t gz_raw = imuReadInt16(REG_OUTX_L_G + 4);
    suma += gz_raw * GYR_SENS_DPS_PER_LSB;
    delay(8);
  }

  gzBias_dps = suma / N;

  Serial.print("Bias GZ = ");
  Serial.print(gzBias_dps, 4);
  Serial.println(" °/s");
}

float normalizarAngulo(float ang) {
  while (ang > 180.0f) ang -= 360.0f;
  while (ang < -180.0f) ang += 360.0f;
  return ang;
}

void resetYawReferencia() {
  yawRefDeg = yawDeg;
}

bool imuEstaEnVentanaIgnorada() {
  unsigned long now = millis();

  if (modoActual == MODE_AUTO || autoActivo) {
    if (now - lastAutoStartMs < IGNORE_LIFT_AFTER_AUTO_START_MS) {
      return true;
    }

    if (now - lastManeuverMs < IGNORE_LIFT_AFTER_MANEUVER_MS) {
      return true;
    }
  }

  return false;
}

void actualizarIMU() {
  if (!imuOk) return;

  unsigned long nowMicros = micros();
  float dt = (nowMicros - lastImuUpdateMicros) / 1000000.0f;
  lastImuUpdateMicros = nowMicros;

  if (dt <= 0.0f || dt > 0.5f) {
    dt = IMU_PERIOD_MS / 1000.0f;
  }

  leerIMUCrudoYConvertir();

  yawDeg += gz_dps * dt;
  yawDeg = normalizarAngulo(yawDeg);

#if DETECT_LIFT_POSITIVE_Z
  float zLift = az_ms2;
#else
  float zLift = -az_ms2;
#endif

  float threshold = (modoActual == MODE_AUTO || autoActivo) ?
                    LIFT_Z_THRESHOLD_AUTO :
                    LIFT_Z_THRESHOLD_MANUAL;

  int confirmSamples = (modoActual == MODE_AUTO || autoActivo) ?
                       LIFT_CONFIRM_AUTO :
                       LIFT_CONFIRM_MANUAL;

  bool liftNow = false;

  if (!imuEstaEnVentanaIgnorada()) {
    liftNow = zLift > threshold;
  }

  if (liftNow) {
    liftCounter++;
  } else {
    liftCounter = 0;
  }

  if (liftCounter >= confirmSamples && !liftDetected) {
    liftDetected = true;

    if (modoActual == MODE_AUTO || autoActivo) {
      resumeAutoWhenStable = true;
    }

    modoActual = MODE_STOPPED;
    autoActivo = false;

    detenerTodo();
    alarmaLevantamiento();

    stableStartMs = 0;

    Serial.println("SAFETY: ROBOT LEVANTADO. Motores DC y cortadores detenidos.");
  }

  if (liftDetected) {
    float absZError = fabsf(fabsf(az_ms2) - STABLE_Z_TARGET);

    bool stableNow =
      absZError < STABLE_Z_TOL &&
      fabsf(gx_dps) < STABLE_GYRO_MAX_DPS &&
      fabsf(gy_dps) < STABLE_GYRO_MAX_DPS &&
      fabsf(gz_dps) < STABLE_GYRO_MAX_DPS;

    if (stableNow) {
      if (stableStartMs == 0) {
        stableStartMs = millis();
      }

      if (millis() - stableStartMs >= STABLE_REQUIRED_MS) {
        liftDetected = false;
        liftCounter = 0;
        stableStartMs = 0;
        resetYawReferencia();

        Serial.println("IMU: robot estable durante 5 segundos. Seguridad liberada.");

        if (resumeAutoWhenStable && !autoTerminado) {
          resumeAutoWhenStable = false;
          Serial.println("Reanudando AUTO porque estaba activo antes del levantamiento.");
          iniciarAutomatico();
        } else {
          resumeAutoWhenStable = false;
          modoActual = MODE_MANUAL;
          autoActivo = false;
        }
      }
    } else {
      stableStartMs = 0;
    }
  }
}

// =========================================================
// ADC BATERÍA
// =========================================================
bool setupADC() {
  if (!ads.begin(ADS_ADDR)) {
    Serial.println("WARN: ADS1015 no detectado. Medición de batería desactivada.");
    return false;
  }

  ads.setGain(GAIN_ONE);

  Serial.println("ADS1015 detectado correctamente.");
  Serial.print("Factor divisor: ");
  Serial.println(FACTOR_DIVISOR, 3);

  return true;
}

float leerVoltajeAIN0() {
  if (!adsOk) return 0.0f;

  long suma = 0;

  for (int i = 0; i < NUM_MUESTRAS_ADC; i++) {
    int16_t lectura = ads.readADC_SingleEnded(0);
    suma += lectura;
    delay(2);
  }

  float promedio = suma / (float)NUM_MUESTRAS_ADC;
  return promedio * ADS1015_MV_POR_BIT / 1000.0f;
}

void actualizarBateria() {
  if (!adsOk) {
    batteryVoltage = 0.0f;
    batteryPercent = 0.0f;
    batteryLow = false;
    batteryCritical = false;
    return;
  }

  float v_adc = leerVoltajeAIN0();
  batteryVoltage = v_adc * FACTOR_DIVISOR;

  batteryPercent = (batteryVoltage - BATTERY_EMPTY_VOLTAGE) * 100.0f /
                   (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE);

  batteryPercent = constrain(batteryPercent, 0.0f, 100.0f);

  if (!batteryLow && batteryVoltage <= BATTERY_LOW_VOLTAGE) {
    batteryLow = true;
  }

  if (batteryLow && batteryVoltage >= BATTERY_LOW_VOLTAGE + BATTERY_HYSTERESIS) {
    batteryLow = false;
  }

  if (!batteryCritical && batteryVoltage <= BATTERY_CRITICAL_VOLTAGE) {
    batteryCritical = true;
  }

  if (batteryCritical && batteryVoltage >= BATTERY_CRITICAL_VOLTAGE + BATTERY_HYSTERESIS) {
    batteryCritical = false;
  }

  if (batteryCritical && millis() - lastBatteryBeep > BATTERY_BEEP_PERIOD_MS) {
    beepBateriaCritica();
    lastBatteryBeep = millis();
  } else if (batteryLow && millis() - lastBatteryBeep > BATTERY_BEEP_PERIOD_MS) {
    beepBateriaBaja();
    lastBatteryBeep = millis();
  }

  if (batteryLow && modoActual == MODE_AUTO) {
    Serial.println("AUTO detenido por batería baja.");
    modoActual = MODE_STOPPED;
    autoActivo = false;
    detenerTodo();
  }
}

// =========================================================
// ESC
// =========================================================
void enviarPulsoESC(int pin, int pulsoUs) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulsoUs);
  digitalWrite(pin, LOW);
}

void servicioESC() {
  int pulso = cortadoresActivos ? ESC_RUN_US : ESC_STOP_US;

  enviarPulsoESC(ESC_IZQ, pulso);
  enviarPulsoESC(ESC_DER, pulso);

  digitalWrite(ESC_BORDEADORA, LOW);

  int restante = 20000 - (2 * pulso);
  if (restante > 0) {
    delayMicroseconds(restante);
  }
}

void delayConESC(unsigned long tiempoMs) {
  unsigned long inicio = millis();

  while (millis() - inicio < tiempoMs) {
    servicioESC();

    leerSerial();

    if (millis() - ultimoImu >= IMU_PERIOD_MS) {
      actualizarIMU();
      ultimoImu = millis();
    }

    if (liftDetected) {
      return;
    }
  }
}

void armarESCs() {
  Serial.println("Armando ESC1 y ESC2 con 1000 us...");
  cortadoresActivos = false;
  delayConESC(TIEMPO_ARMADO_ESC);
  Serial.println("ESC1 y ESC2 armados.");
}

void encenderCortadores() {
  if (liftDetected) {
    Serial.println("Cortadores bloqueados: robot levantado.");
    cortadoresActivos = false;
    return;
  }

  if (batteryLow) {
    Serial.println("Cortadores bloqueados: batería baja.");
    beepBateriaBaja();
    cortadoresActivos = false;
    return;
  }

  if (cortadoresActivos) return;

  cortadoresActivos = true;
  Serial.println("Cortadores ESC1/ESC2 encendidos.");
}

void detenerCortadores() {
  cortadoresActivos = false;

  for (int i = 0; i < 20; i++) {
    servicioESC();
  }

  Serial.println("Cortadores ESC1/ESC2 detenidos.");
}

// =========================================================
// BUZZER
// =========================================================
void beepCorto() {
  digitalWrite(BUZZER, HIGH);
  delayConESC(80);
  digitalWrite(BUZZER, LOW);
  delayConESC(80);
}

void beepLargo() {
  digitalWrite(BUZZER, HIGH);
  delayConESC(250);
  digitalWrite(BUZZER, LOW);
  delayConESC(120);
}

void beepBateriaBaja() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER, HIGH);
    delayConESC(120);
    digitalWrite(BUZZER, LOW);
    delayConESC(180);
  }
}

void beepBateriaCritica() {
  for (int i = 0; i < 5; i++) {
    digitalWrite(BUZZER, HIGH);
    delayConESC(80);
    digitalWrite(BUZZER, LOW);
    delayConESC(80);
  }
}

void alarmaLevantamiento() {
  for (int i = 0; i < 8; i++) {
    digitalWrite(BUZZER, HIGH);
    delayConESC(55);
    digitalWrite(BUZZER, LOW);
    delayConESC(55);
  }
}

void avisoMovimiento() {
  beepCorto();
  beepCorto();
  beepLargo();
}

void alarmaChoque() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(BUZZER, HIGH);
    delayConESC(60);
    digitalWrite(BUZZER, LOW);
    delayConESC(60);
  }
}

void alarmaObstaculo() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(BUZZER, HIGH);
    delayConESC(140);
    digitalWrite(BUZZER, LOW);
    delayConESC(100);
  }
}

void alarmaFinAuto() {
  beepLargo();
  beepLargo();
}

// =========================================================
// BUMPERS
// =========================================================
bool bumperDer() {
  return digitalRead(BUMPER_DER);
}

bool bumperFront() {
  return digitalRead(BUMPER_FRONT);
}

bool bumperIzq() {
  return digitalRead(BUMPER_IZQ);
}

bool bumperTras() {
  return digitalRead(BUMPER_TRAS);
}

bool hayBumperPresionado() {
  return bumperDer() || bumperFront() || bumperIzq() || bumperTras();
}

// =========================================================
// MOTORES
// =========================================================
void detenerMotoresDC() {
  analogWrite(PWM_DER, 0);
  analogWrite(PWM_IZQ, 0);
}

void detenerTodo() {
  detenerMotoresDC();
  cortadoresActivos = false;

  for (int i = 0; i < 25; i++) {
    servicioESC();
  }
}

void moverMotorDerecho(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    digitalWrite(DIR_DER, DIR_DER_ADELANTE);
    analogWrite(PWM_DER, pwm);
  } else if (pwm < 0) {
    digitalWrite(DIR_DER, DIR_DER_ATRAS);
    analogWrite(PWM_DER, -pwm);
  } else {
    analogWrite(PWM_DER, 0);
  }
}

void moverMotorIzquierdo(int pwm) {
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    digitalWrite(DIR_IZQ, DIR_IZQ_ADELANTE);
    analogWrite(PWM_IZQ, pwm);
  } else if (pwm < 0) {
    digitalWrite(DIR_IZQ, DIR_IZQ_ATRAS);
    analogWrite(PWM_IZQ, -pwm);
  } else {
    analogWrite(PWM_IZQ, 0);
  }
}

void aplicarMovimientoManual(float linear, float angular) {
  if (modoActual != MODE_MANUAL) return;

  if (liftDetected) {
    detenerTodo();
    Serial.println("Movimiento bloqueado: robot levantado.");
    return;
  }

  if (hayBumperPresionado()) {
    detenerMotoresDC();
    Serial.println("Movimiento bloqueado: bumper presionado.");
    return;
  }

  float linearNorm = linear / APP_MAX_LINEAR;
  float angularNorm = angular / APP_MAX_ANGULAR;

  linearNorm = constrain(linearNorm, -1.0f, 1.0f);
  angularNorm = constrain(angularNorm, -1.0f, 1.0f);

  float derecha = constrain(linearNorm + angularNorm, -1.0f, 1.0f);
  float izquierda = constrain(linearNorm - angularNorm, -1.0f, 1.0f);

  int pwmDer = (int)(derecha * PWM_MAX_MANUAL);
  int pwmIzq = (int)(izquierda * PWM_MAX_MANUAL);

  moverMotorDerecho(pwmDer);
  moverMotorIzquierdo(pwmIzq);
}

void avanzar(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);

  digitalWrite(DIR_DER, DIR_DER_ADELANTE);
  digitalWrite(DIR_IZQ, DIR_IZQ_ADELANTE);

  analogWrite(PWM_DER, velocidad);
  analogWrite(PWM_IZQ, velocidad);
}

void avanzarRectoIMU(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);

  if (!imuOk) {
    avanzar(velocidad);
    return;
  }

  float errorYaw = normalizarAngulo(yawDeg - yawRefDeg);

  float correctionFloat =
      IMU_STEER_SIGN * ((YAW_KP * errorYaw) + (YAW_KD * gz_dps));

  int correction =
      constrain((int)correctionFloat, -MAX_IMU_CORRECTION, MAX_IMU_CORRECTION);

  int pwmDer = constrain(velocidad - correction, 0, 255);
  int pwmIzq = constrain(velocidad + correction, 0, 255);

  digitalWrite(DIR_DER, DIR_DER_ADELANTE);
  digitalWrite(DIR_IZQ, DIR_IZQ_ADELANTE);

  analogWrite(PWM_DER, pwmDer);
  analogWrite(PWM_IZQ, pwmIzq);
}

void retroceder(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);

  digitalWrite(DIR_DER, DIR_DER_ATRAS);
  digitalWrite(DIR_IZQ, DIR_IZQ_ATRAS);

  analogWrite(PWM_DER, velocidad);
  analogWrite(PWM_IZQ, velocidad);
}

void girarIzquierda(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);

  digitalWrite(DIR_DER, DIR_DER_ADELANTE);
  digitalWrite(DIR_IZQ, DIR_IZQ_ATRAS);

  analogWrite(PWM_DER, velocidad);
  analogWrite(PWM_IZQ, velocidad);
}

void girarDerecha(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);

  digitalWrite(DIR_DER, DIR_DER_ATRAS);
  digitalWrite(DIR_IZQ, DIR_IZQ_ADELANTE);

  analogWrite(PWM_DER, velocidad);
  analogWrite(PWM_IZQ, velocidad);
}

// =========================================================
// ULTRASONIDOS - LÓGICA COMO EL CÓDIGO QUE FUNCIONABA
// =========================================================
float medirDistanciaCM(int trigPin, int echoPin) {
  servicioESC();
  leerSerial();

  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duracion = pulseIn(echoPin, HIGH, 20000);

  servicioESC();
  leerSerial();

  if (duracion == 0) {
    return -1.0f;
  }

  return duracion * 0.0343f / 2.0f;
}

void actualizarDistancias() {
  dIzq = medirDistanciaCM(TRIG_IZQ, ECHO_IZQ);
  delayConESC(20);

  dFront = medirDistanciaCM(TRIG_FRONT, ECHO_FRONT);
  delayConESC(20);

  dDer = medirDistanciaCM(TRIG_DER, ECHO_DER);
  delayConESC(20);
}

// =========================================================
// STATUS JSON
// =========================================================
const char* modoTexto() {
  if (modoActual == MODE_MANUAL) return "MANUAL";
  if (modoActual == MODE_AUTO) return "AUTO";
  return "STOPPED";
}

int stableCountdown() {
  if (!liftDetected || stableStartMs == 0) return 0;

  unsigned long elapsed = millis() - stableStartMs;
  int remaining = 5 - (elapsed / 1000);

  if (remaining < 0) remaining = 0;

  return remaining;
}

void enviarStatus() {
  Serial.print("{\"type\":\"status\"");

  Serial.print(",\"mode\":\"");
  Serial.print(modoTexto());
  Serial.print("\"");

  Serial.print(",\"bumper_der\":");
  Serial.print(bumperDer() ? 1 : 0);

  Serial.print(",\"bumper_front\":");
  Serial.print(bumperFront() ? 1 : 0);

  Serial.print(",\"bumper_izq\":");
  Serial.print(bumperIzq() ? 1 : 0);

  Serial.print(",\"bumper_tras\":");
  Serial.print(bumperTras() ? 1 : 0);

  Serial.print(",\"cutters_on\":");
  Serial.print(cortadoresActivos ? 1 : 0);

  Serial.print(",\"d_izq\":");
  Serial.print(dIzq, 1);

  Serial.print(",\"d_front\":");
  Serial.print(dFront, 1);

  Serial.print(",\"d_der\":");
  Serial.print(dDer, 1);

  Serial.print(",\"auto_elapsed\":");
  if (autoActivo) {
    Serial.print((millis() - inicioAuto) / 1000);
  } else {
    Serial.print(0);
  }

  Serial.print(",\"ads_ok\":");
  Serial.print(adsOk ? 1 : 0);

  Serial.print(",\"battery_v\":");
  Serial.print(batteryVoltage, 2);

  Serial.print(",\"battery_percent\":");
  Serial.print(batteryPercent, 0);

  Serial.print(",\"battery_low\":");
  Serial.print(batteryLow ? 1 : 0);

  Serial.print(",\"battery_critical\":");
  Serial.print(batteryCritical ? 1 : 0);

  Serial.print(",\"imu_ok\":");
  Serial.print(imuOk ? 1 : 0);

  Serial.print(",\"ax_ms2\":");
  Serial.print(ax_ms2, 3);

  Serial.print(",\"ay_ms2\":");
  Serial.print(ay_ms2, 3);

  Serial.print(",\"az_ms2\":");
  Serial.print(az_ms2, 3);

  Serial.print(",\"gx_dps\":");
  Serial.print(gx_dps, 3);

  Serial.print(",\"gy_dps\":");
  Serial.print(gy_dps, 3);

  Serial.print(",\"gz_dps\":");
  Serial.print(gz_dps, 3);

  Serial.print(",\"yaw_deg\":");
  Serial.print(yawDeg, 2);

  Serial.print(",\"yaw_ref\":");
  Serial.print(yawRefDeg, 2);

  Serial.print(",\"lift_detected\":");
  Serial.print(liftDetected ? 1 : 0);

  Serial.print(",\"stable_countdown\":");
  Serial.print(stableCountdown());

  Serial.println("}");
}

// =========================================================
// AUTOMÁTICO - USA ULTRASONIDOS COMO EL QUE FUNCIONABA
// =========================================================
void iniciarAutomatico() {
  if (modoActual == MODE_AUTO) return;

  if (liftDetected) {
    Serial.println("AUTO no inicia: robot levantado.");
    detenerTodo();
    return;
  }

  if (batteryLow) {
    Serial.println("AUTO no inicia: batería baja.");
    beepBateriaBaja();
    detenerTodo();
    return;
  }

  detenerMotoresDC();
  actualizarDistancias();

  if (hayBumperPresionado()) {
    Serial.println("AUTO no inicia: bumper presionado.");
    alarmaChoque();
    modoActual = MODE_STOPPED;
    autoActivo = false;
    detenerTodo();
    return;
  }

  if (dFront > 0 && dFront < distanciaFrontalParada) {
    Serial.println("AUTO no inicia: obstáculo frontal cercano.");
    alarmaObstaculo();
    modoActual = MODE_STOPPED;
    autoActivo = false;
    detenerTodo();
    return;
  }

  resetYawReferencia();

  modoActual = MODE_AUTO;
  autoActivo = true;
  autoTerminado = false;
  inicioAuto = millis();
  lastAutoStartMs = millis();

  avisoMovimiento();
  encenderCortadores();

  Serial.println("AUTO iniciado.");
}

void detenerAutomatico() {
  autoActivo = false;
  autoTerminado = false;
  resumeAutoWhenStable = false;

  modoActual = MODE_MANUAL;

  detenerMotoresDC();
  detenerCortadores();

  Serial.println("AUTO detenido. Volviendo a MANUAL.");
}

void finalizarAutomatico() {
  Serial.println("AUTO finalizado: 60 segundos cumplidos.");

  detenerTodo();

  autoActivo = false;
  autoTerminado = true;
  modoActual = MODE_STOPPED;
  resumeAutoWhenStable = false;

  alarmaFinAuto();
}

void maniobraPorBumper() {
  detenerTodo();

  Serial.println("AUTO EVENTO: bumper presionado.");
  alarmaChoque();

  lastManeuverMs = millis();

  if (bumperFront()) {
    retroceder(velocidadRetroceso);
    delayConESC(tiempoRetroceso);

    detenerMotoresDC();
    delayConESC(150);

    actualizarDistancias();

    if (dIzq > dDer) {
      girarIzquierda(velocidadGiro);
    } else {
      girarDerecha(velocidadGiro);
    }

    delayConESC(tiempoGiro);
  } else if (bumperDer()) {
    retroceder(velocidadRetroceso);
    delayConESC(500);

    girarIzquierda(velocidadGiro);
    delayConESC(tiempoGiro);
  } else if (bumperIzq()) {
    retroceder(velocidadRetroceso);
    delayConESC(500);

    girarDerecha(velocidadGiro);
    delayConESC(tiempoGiro);
  } else if (bumperTras()) {
    avanzar(velocidadAvance);
    delayConESC(600);
  }

  detenerMotoresDC();
  delayConESC(200);

  actualizarDistancias();
  resetYawReferencia();

  lastManeuverMs = millis();

  if (autoActivo &&
      !hayBumperPresionado() &&
      !(dFront > 0 && dFront < distanciaFrontalParada) &&
      !liftDetected &&
      !batteryLow) {
    avisoMovimiento();
    encenderCortadores();
  }
}

void maniobraFrontal() {
  detenerTodo();

  Serial.println("AUTO EVENTO: obstáculo frontal.");
  alarmaObstaculo();

  lastManeuverMs = millis();

  retroceder(velocidadRetroceso);
  delayConESC(tiempoRetroceso);

  detenerMotoresDC();
  delayConESC(150);

  actualizarDistancias();

  if (dIzq > 0 && dDer > 0) {
    if (dIzq > dDer) {
      girarIzquierda(velocidadGiro);
    } else {
      girarDerecha(velocidadGiro);
    }
  } else if (dIzq > 0 && dDer < 0) {
    girarIzquierda(velocidadGiro);
  } else if (dDer > 0 && dIzq < 0) {
    girarDerecha(velocidadGiro);
  } else {
    girarDerecha(velocidadGiro);
  }

  delayConESC(tiempoGiro);

  detenerMotoresDC();
  delayConESC(200);

  actualizarDistancias();
  resetYawReferencia();

  lastManeuverMs = millis();

  if (autoActivo &&
      !hayBumperPresionado() &&
      !(dFront > 0 && dFront < distanciaFrontalParada) &&
      !liftDetected &&
      !batteryLow) {
    avisoMovimiento();
    encenderCortadores();
  }
}

void correccionLateralIzquierda() {
  Serial.println("AUTO: obstáculo izquierda. Corrige derecha.");

  lastManeuverMs = millis();

  girarDerecha(velocidadGiro);
  delayConESC(tiempoCorreccionLateral);

  detenerMotoresDC();
  delayConESC(80);

  resetYawReferencia();
  lastManeuverMs = millis();
}

void correccionLateralDerecha() {
  Serial.println("AUTO: obstáculo derecha. Corrige izquierda.");

  lastManeuverMs = millis();

  girarIzquierda(velocidadGiro);
  delayConESC(tiempoCorreccionLateral);

  detenerMotoresDC();
  delayConESC(80);

  resetYawReferencia();
  lastManeuverMs = millis();
}

void ejecutarAutomatico() {
  if (!autoActivo || modoActual != MODE_AUTO) return;

  if (liftDetected) {
    Serial.println("AUTO detenido por levantamiento.");
    modoActual = MODE_STOPPED;
    autoActivo = false;
    detenerTodo();
    return;
  }

  if (batteryLow) {
    Serial.println("AUTO detenido por batería baja.");
    modoActual = MODE_STOPPED;
    autoActivo = false;
    detenerTodo();
    beepBateriaBaja();
    return;
  }

  if (millis() - inicioAuto >= DURACION_AUTO) {
    finalizarAutomatico();
    return;
  }

  // Esta llamada es deliberada: AUTO siempre decide con ultrasonidos frescos.
  actualizarDistancias();

  // Prioridad 1: bumpers.
  if (hayBumperPresionado()) {
    maniobraPorBumper();
    return;
  }

  // Prioridad 2: obstáculo frontal por ultrasonido.
  if (dFront > 0 && dFront < distanciaFrontalParada) {
    maniobraFrontal();
    return;
  }

  // Prioridad 3: obstáculo lateral izquierdo.
  if (dIzq > 0 && dIzq < distanciaLateralAlerta) {
    correccionLateralIzquierda();
    return;
  }

  // Prioridad 4: obstáculo lateral derecho.
  if (dDer > 0 && dDer < distanciaLateralAlerta) {
    correccionLateralDerecha();
    return;
  }

  if (!cortadoresActivos) {
    encenderCortadores();
  }

  // Sin obstáculos: avance recto asistido por IMU.
  if (dFront > 0 && dFront < distanciaPrecaucion) {
    avanzarRectoIMU(velocidadLenta);
  } else {
    avanzarRectoIMU(velocidadAvance);
  }
}

// =========================================================
// SERIAL DESDE RASPBERRY
// =========================================================
void procesarLinea(String linea) {
  linea.trim();

  if (linea.length() == 0) return;

  if (linea == "STOP") {
    modoActual = MODE_STOPPED;
    autoActivo = false;
    resumeAutoWhenStable = false;
    detenerTodo();
    Serial.println("STOP recibido. Todo detenido.");
    return;
  }

  if (linea == "MODE MANUAL") {
    if (liftDetected) {
      Serial.println("No se puede pasar a MANUAL: robot levantado.");
      detenerTodo();
      return;
    }

    resumeAutoWhenStable = false;
    modoActual = MODE_MANUAL;
    autoActivo = false;

    detenerMotoresDC();
    resetYawReferencia();

    Serial.println("Modo MANUAL activado.");
    return;
  }

  if (linea == "MODE AUTO") {
    iniciarAutomatico();
    return;
  }

  if (linea == "AUTO STOP") {
    detenerAutomatico();
    return;
  }

  if (linea == "CUTTERS ON") {
    encenderCortadores();
    Serial.println("CUTTERS ON recibido.");
    return;
  }

  if (linea == "CUTTERS OFF") {
    detenerCortadores();
    Serial.println("CUTTERS OFF recibido.");
    return;
  }

  if (linea.startsWith("V ")) {
    if (modoActual != MODE_MANUAL) {
      return;
    }

    int primerEspacio = linea.indexOf(' ');
    int segundoEspacio = linea.indexOf(' ', primerEspacio + 1);

    if (segundoEspacio < 0) {
      Serial.println("ERROR: comando V inválido.");
      return;
    }

    float linear = linea.substring(primerEspacio + 1, segundoEspacio).toFloat();
    float angular = linea.substring(segundoEspacio + 1).toFloat();

    ultimoComandoManual = millis();

    aplicarMovimientoManual(linear, angular);
    return;
  }

  Serial.print("Comando desconocido: ");
  Serial.println(linea);
}

void leerSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      procesarLinea(lineaSerial);
      lineaSerial = "";
    } else {
      lineaSerial += c;
    }
  }
}

// =========================================================
// SETUP / LOOP
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(BUMPER_DER, INPUT);
  pinMode(BUMPER_FRONT, INPUT);
  pinMode(BUMPER_IZQ, INPUT);
  pinMode(BUMPER_TRAS, INPUT);

  pinMode(TRIG_IZQ, OUTPUT);
  pinMode(ECHO_IZQ, INPUT);

  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);

  pinMode(TRIG_DER, OUTPUT);
  pinMode(ECHO_DER, INPUT);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  pinMode(DIR_DER, OUTPUT);
  pinMode(PWM_DER, OUTPUT);

  pinMode(DIR_IZQ, OUTPUT);
  pinMode(PWM_IZQ, OUTPUT);

  pinMode(ESC_IZQ, OUTPUT);
  pinMode(ESC_DER, OUTPUT);
  pinMode(ESC_BORDEADORA, OUTPUT);

  digitalWrite(ESC_IZQ, LOW);
  digitalWrite(ESC_DER, LOW);
  digitalWrite(ESC_BORDEADORA, LOW);

  digitalWrite(TRIG_IZQ, LOW);
  digitalWrite(TRIG_FRONT, LOW);
  digitalWrite(TRIG_DER, LOW);

  detenerMotoresDC();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  Serial.println("====================================");
  Serial.println(" ZO-ROS ESP32 INTEGRADO CON ULTRASONIDOS");
  Serial.println(" Manual + AUTO + US + ESC + ADC + IMU + recto");
  Serial.println("====================================");

  scanI2C();

  adsOk = setupADC();
  imuOk = setupIMU();
  calibrarGyroZ();

  armarESCs();

  actualizarBateria();
  actualizarIMU();
  resetYawReferencia();
  actualizarDistancias();

  modoActual = MODE_MANUAL;
  autoActivo = false;
  autoTerminado = false;
  cortadoresActivos = false;
  liftDetected = false;
  resumeAutoWhenStable = false;

  ultimoComandoManual = millis();
  ultimoStatus = millis();
  ultimoSensor = millis();
  ultimoBattery = millis();
  ultimoImu = millis();

  beepCorto();

  Serial.println("Sistema listo en modo MANUAL.");
}

void loop() {
  leerSerial();

  if (millis() - ultimoImu >= IMU_PERIOD_MS) {
    actualizarIMU();
    ultimoImu = millis();
  }

  if (millis() - ultimoBattery >= BATTERY_PERIOD_MS) {
    actualizarBateria();
    ultimoBattery = millis();
  }

  if (liftDetected) {
    detenerTodo();
  }

  if (modoActual == MODE_MANUAL) {
    if (millis() - ultimoComandoManual > TIMEOUT_MANUAL_MS) {
      detenerMotoresDC();
    }

    if (hayBumperPresionado()) {
      detenerMotoresDC();
    }

    if (millis() - ultimoSensor > SENSOR_PERIOD_MANUAL_MS) {
      actualizarDistancias();
      ultimoSensor = millis();
    }
  }

  if (modoActual == MODE_AUTO) {
    ejecutarAutomatico();
  }

  if (millis() - ultimoStatus > STATUS_PERIOD_MS) {
    enviarStatus();
    ultimoStatus = millis();
  }

  servicioESC();
}
