#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <math.h>

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

const unsigned long sampleInterval = 10;   // 100 Hz -> lectura del sensor y calculo del modelo
const unsigned long sendInterval   = 150;  // cada cuanto se ENVIA la velocidad por WiFi (para no saturar la red)

unsigned long lastSample = 0;
unsigned long lastSend = 0;


// =========================================================
// CONSTANTES DEL MODELO DE MACHINE LEARNING
// (obtenidas del entrenamiento en Google Colab)
// =========================================================

// --- Conversion de unidades crudas a unidades fisicas ---
// (mismos factores que usaba el script receptor en Python)
const float ACCEL_SCALE = 0.000061;   // g/LSB
const float GYRO_SCALE  = 0.00875;    // grados/s / LSB
const float GRAVITY     = 9.80665;    // m/s^2 por g

// --- Parametros del primer escalador (3 features: angulo_abs, w_x, w_y) ---
const float MEAN_1[3]  = {38.10096507, 7.73562638, -1.76439703};
const float SCALE_1[3] = {25.89619806, 19.42472526, 44.5293814};

// --- Parametros del segundo escalador (9 features polinomicas, grado 2) ---
// Orden: angulo_abs, w_x, w_y, angulo_abs^2, angulo_abs*w_x, angulo_abs*w_y, w_x^2, w_x*w_y, w_y^2
const float MEAN_2[9]  = {38.1009651, 7.73562638, -1.76439703, 2122.29661, 448.215522, -143.665112, 437.159867, -274.597072, 1985.97891};
const float SCALE_2[9] = {25.89619806, 19.42472526, 44.5293814, 2189.85809529, 1286.45872902, 2274.87676336, 1313.35371098, 1229.23159976, 3294.56183187};

// --- Theta del modelo polinomico entrenado (10 valores: intercepto + 9 pesos) ---
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

// Convierte las lecturas crudas del sensor a las 3 caracteristicas del modelo
// (angulo_abs, w_x, w_y), replicando exactamente la misma conversion y el mismo
// intercambio de ejes Y<->Z que hacia el script de Python durante el entrenamiento.
void calcularCaracteristicas(int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                              int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
                              float &angulo_abs, float &w_x, float &w_y)
{
  // Conversion a unidades fisicas + intercambio de ejes Y <-> Z
  // (igual que hacia procesar_paquete() en receptorDB2.py)
  float a_x = (ax_raw * ACCEL_SCALE) * GRAVITY;
  float a_z = (ay_raw * ACCEL_SCALE) * GRAVITY;   // usa el eje Y crudo (intercambio)

  w_x = gx_raw * GYRO_SCALE;
  w_y = gz_raw * GYRO_SCALE;                       // usa el eje Z crudo (intercambio)

  // Angulo de inclinacion (roll), en grados, y su valor absoluto
  float angulo_roll = atan2(a_z, -a_x) * 180.0 / PI;
  angulo_abs = fabs(angulo_roll);
}

// Normaliza un valor: z = (x - promedio) / desviacion
float normalizar(float x, float media, float escala)
{
  return (x - media) / escala;
}

// Calcula la velocidad predicha por el modelo polinomico a partir de las
// 3 caracteristicas ya calculadas (angulo_abs, w_x, w_y)
float calcularVelocidad(float angulo_abs, float w_x, float w_y)
{
  // --- Paso 1: normalizar las 3 caracteristicas originales ---
  float f1[3];
  f1[0] = normalizar(angulo_abs, MEAN_1[0], SCALE_1[0]);
  f1[1] = normalizar(w_x,        MEAN_1[1], SCALE_1[1]);
  f1[2] = normalizar(w_y,        MEAN_1[2], SCALE_1[2]);

  // --- Paso 2: generar las 9 caracteristicas polinomicas (grado 2) ---
  // [angulo_abs, w_x, w_y, angulo_abs^2, angulo_abs*w_x, angulo_abs*w_y, w_x^2, w_x*w_y, w_y^2]
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

  // --- Paso 3: normalizar OTRA VEZ, ahora las 9 caracteristicas polinomicas ---
  float f2[9];
  for (int i = 0; i < 9; i++) {
    f2[i] = normalizar(poly[i], MEAN_2[i], SCALE_2[i]);
  }

  // --- Paso 4: aplicar la ecuacion del modelo (Y = A*X + B) ---
  float velocidad = THETA[0];  // intercepto
  for (int i = 0; i < 9; i++) {
    velocidad += THETA[i + 1] * f2[i];
  }

  // --- Paso 5: limitar el resultado al rango valido [0, 1] ---
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
  // Crear punto de acceso
  // -------------------------

  WiFi.softAP(ssid, password);

  delay(500);

  // Inicializar UDP
  udp.begin(udpPort);

  sendMessage("STATUS,ESP32_INICIADA");


  // -------------------------
  // Inicializar I2C
  // -------------------------

  Wire.begin(SDA_PIN, SCL_PIN);

  sendMessage("STATUS,I2C_INICIADO");


  // -------------------------
  // Inicializar LSM6DS33
  // -------------------------

  if (!imu.init())
  {
    sendMessage("ERROR,LSM6,no_detectado");

    while (true)
    {
      delay(1000);
    }
  }

  imu.enableDefault();

  sendMessage("STATUS,LSM6_OK");


  // -------------------------
  // Inicializar LIS3MDL
  // -------------------------

  if (!mag.init())
  {
    sendMessage("ERROR,LIS3MDL,no_detectado");

    while (true)
    {
      delay(1000);
    }
  }

  mag.enableDefault();

  sendMessage("STATUS,LIS3MDL_OK");


  // -------------------------
  // Todo listo
  // -------------------------

  sendMessage("STATUS,IMU_INICIADA");
}


// =========================
// LOOP
// =========================

void loop()
{
  unsigned long currentTime = millis();

  if (currentTime - lastSample >= sampleInterval)
  {
    lastSample = currentTime;

    // Leer sensores
    imu.read();
    mag.read();

    // ---- Calcular las caracteristicas del modelo ----
    float angulo_abs, w_x, w_y;
    calcularCaracteristicas(
      imu.a.x, imu.a.y, imu.a.z,
      imu.g.x, imu.g.y, imu.g.z,
      angulo_abs, w_x, w_y
    );

    // ---- Calcular la velocidad con el modelo entrenado ----
    float velocidad = calcularVelocidad(angulo_abs, w_x, w_y);

    // ---- Enviar la velocidad por WiFi cada 'sendInterval' ms ----
    if (currentTime - lastSend >= sendInterval)
    {
      lastSend = currentTime;

      char mensaje[64];
      snprintf(mensaje, sizeof(mensaje), "VEL,%.3f,%.2f,%.2f,%.2f", velocidad, angulo_abs, w_x, w_y);
      sendMessage(mensaje);
    }
  }
}
