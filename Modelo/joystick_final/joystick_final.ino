#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <math.h>
#include <ArduinoOTA.h>  // <--- Librería para actualización inalámbrica

// =========================
// Wi-Fi
// =========================

const char* ssid = "Guante_IMU";
const char* password = "12345678";

const IPAddress pcIP(192, 168, 4, 2);
const unsigned int udpPort = 5000;

WiFiUDP udp;

// =========================
// I2C
// =========================

#define SDA_PIN 13
#define SCL_PIN 14

// =========================
// Sensores
// =========================

LSM6 imu;
LIS3MDL mag;

// =========================
// Muestreo
// =========================

const unsigned long sampleInterval = 10;   // 100 Hz -> lectura del sensor y cálculo
const unsigned long sendInterval   = 150;  // Envío por UDP

unsigned long lastSample = 0;
unsigned long lastSend = 0;

// =========================================================
// CONSTANTES DEL MODELO DE MACHINE LEARNING
// =========================================================

const float ACCEL_SCALE = 0.000061;   // g/LSB
const float GYRO_SCALE  = 0.00875;    // grados/s / LSB
const float GRAVITY     = 9.80665;    // m/s^2 por g

const float MEAN_1[3]  = {38.10096507, 7.73562638, -1.76439703};
const float SCALE_1[3] = {25.89619806, 19.42472526, 44.5293814};

const float MEAN_2[9]  = {38.1009651, 7.73562638, -1.76439703, 2122.29661, 448.215522, -143.665112, 437.159867, -274.597072, 1985.97891};
const float SCALE_2[9] = {25.89619806, 19.42472526, 44.5293814, 2189.85809529, 1286.45872902, 2274.87676336, 1313.35371098, 1229.23159976, 3294.56183187};

const float THETA[10] = {0.66657489, 0.80386927, -0.01309359, -0.14718238, -0.46418315, 0.01453122, 0.11901392, 0.00294823, 0.02212228, 0.0322368};

// =========================
// Enviar mensaje UDP
// =========================

void sendMessage(const char* message)
{
  udp.beginPacket(pcIP, udpPort);
  udp.print(message);
  udp.endPacket();
}

// =========================================================
// FUNCIONES DEL MODELO
// =========================================================

void calcularCaracteristicas(int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                             int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
                             float &angulo_abs, float &w_x, float &w_y)
{
  float a_x = (ax_raw * ACCEL_SCALE) * GRAVITY;
  float a_y = (az_raw * ACCEL_SCALE) * GRAVITY; 
  float a_z = (ay_raw * ACCEL_SCALE) * GRAVITY; 

  w_x = gx_raw * GYRO_SCALE;
  w_y = gz_raw * GYRO_SCALE;

  // Ángulo de inclinación absoluto
  float angulo_roll = atan2(a_z, sqrt(a_x * a_x + a_y * a_y)) * 180.0 / M_PI;
  angulo_abs = fabs(angulo_roll);
}

float normalizar(float x, float media, float escala)
{
  return (x - media) / escala;
}

float calcularVelocidad(float angulo_abs, float w_x, float w_y)
{
  float f1[3];
  f1[0] = normalizar(angulo_abs, MEAN_1[0], SCALE_1[0]);
  f1[1] = normalizar(w_x,        MEAN_1[1], SCALE_1[1]);
  f1[2] = normalizar(w_y,        MEAN_1[2], SCALE_1[2]);

  float poly[9];
  poly[0] = f1[0];
  poly[1] = f1[1];
  poly[2] = f1[2];
  poly[3] = f1[0] * f1[0];
  poly[4] = f1[0] * f1[1];
  poly[5] = f1[0] * f1[2];
  poly[6] = f1[1] * f1[1];
  poly[7] = f1[1] * f1[2];
  poly[8] = f1[2] * f1[2];

  float f2[9];
  for (int i = 0; i < 9; i++) {
    f2[i] = normalizar(poly[i], MEAN_2[i], SCALE_2[i]);
  }

  float velocidad = THETA[0];
  for (int i = 0; i < 9; i++) {
    velocidad += THETA[i + 1] * f2[i];
  }

  if (velocidad < 0.0) velocidad = 0.0;
  if (velocidad > 1.0) velocidad = 1.0;

  return velocidad;
}

// =========================
// SETUP
// =========================

void setup()
{
  // -------------------------
  // Crear punto de acceso Wi-Fi
  // -------------------------
  WiFi.softAP(ssid, password);
  delay(500);

  // Inicializar UDP
  udp.begin(udpPort);
  sendMessage("STATUS,ESP32_INICIADA");

  // -------------------------
  // Configuración de Arduino OTA
  // -------------------------
  ArduinoOTA.setHostname("GuanteIMU-ESP32");
  
  // Opcional: Establecer una contraseña para poder actualizar el código
  // ArduinoOTA.setPassword("admin123");

  ArduinoOTA.onStart([]() {
    sendMessage("STATUS,INICIANDO_ACTUALIZACION_OTA");
  });
  ArduinoOTA.onEnd([]() {
    sendMessage("STATUS,ACTUALIZACION_COMPLETADA");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    sendMessage("ERROR,FALLO_OTA");
  });

  ArduinoOTA.begin();
  sendMessage("STATUS,OTA_LISTO");

  // -------------------------
  // Inicializar I2C y Sensores
  // -------------------------
  Wire.begin(SDA_PIN, SCL_PIN);
  sendMessage("STATUS,I2C_INICIADO");

  if (!imu.init()) {
    sendMessage("ERROR,LSM6,no_detectado");
  } else {
    imu.enableDefault();
    sendMessage("STATUS,LSM6_OK");
  }

  if (!mag.init()) {
    sendMessage("ERROR,LIS3MDL,no_detectado");
  } else {
    mag.enableDefault();
    sendMessage("STATUS,LIS3MDL_OK");
  }

  sendMessage("STATUS,IMU_INICIADA");
}

// =========================
// LOOP
// =========================

void loop()
{
  // Escuchar y procesar peticiones de actualización inalámbrica
  ArduinoOTA.handle();

  unsigned long currentTime = millis();

  if (currentTime - lastSample >= sampleInterval)
  {
    lastSample = currentTime;

    // Leer sensores
    imu.read();
    mag.read();

    // Calcular características
    float angulo_abs, w_x, w_y;
    calcularCaracteristicas(
      imu.a.x, imu.a.y, imu.a.z,
      imu.g.x, imu.g.y, imu.g.z,
      angulo_abs, w_x, w_y
    );

    // Calcular velocidad
    float velocidad = calcularVelocidad(angulo_abs, w_x, w_y);

    // Enviar velocidad por UDP
    if (currentTime - lastSend >= sendInterval)
    {
      lastSend = currentTime;

      char mensaje[64];
      snprintf(mensaje, sizeof(mensaje), "VEL,%.3f,%.2f,%.2f,%.2f", velocidad, angulo_abs, w_x, w_y);
      sendMessage(mensaje);
    }
  }
}