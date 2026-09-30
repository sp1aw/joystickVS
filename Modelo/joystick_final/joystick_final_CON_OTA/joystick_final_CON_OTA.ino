#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <math.h>
#include <WebServer.h>
#include <Update.h>

// =========================
// Wi-Fi & Red
// =========================

const char* ssid = "Guante_IMU";
const char* password = "12345678";

IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

const IPAddress pcIP(192, 168, 4, 2);
const unsigned int udpPort = 5000;

WiFiUDP udp;

// =========================
// Web OTA & Versión
// =========================

WebServer server(80);

const char* FIRMWARE_VERSION = "2.0 (MODO RESILIENTE)";

const char* paginaUpdate =
  "<!DOCTYPE html><html><head><meta charset='UTF-8'></head><body>"
  "<h2>Actualizar firmware - Joystick Inteligente</h2>"
  "<p style='color: green; font-weight: bold;'>Versión cargada actual: v1.1 (MODO RESILIENTE)</p>"
  "<form method='POST' action='/update' enctype='multipart/form-data'>"
  "<input type='file' name='update'>"
  "<input type='submit' value='Actualizar'>"
  "</form>"
  "</body></html>";

void configurarWebOTA()
{
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", paginaUpdate);
  });

  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "ERROR: fallo la actualizacion" : "OK: actualizado, reiniciando...");
    delay(1000);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START)
    {
      Serial.printf("Actualizando: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN))
      {
        Update.printError(Serial);
      }
    }
    else if (upload.status == UPLOAD_FILE_WRITE)
    {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
      {
        Update.printError(Serial);
      }
    }
    else if (upload.status == UPLOAD_FILE_END)
    {
      if (Update.end(true))
      {
        Serial.printf("Actualizacion completa: %u bytes\n", upload.totalSize);
      }
      else
      {
        Update.printError(Serial);
      }
    }
  });

  server.begin();
}

// =========================
// I2C
// =========================

#define SDA_PIN 13
#define SCL_PIN 14

// =========================
// Sensores & Banderas de estado
// =========================

LSM6 imu;
LIS3MDL mag;

bool imuOk = false;
bool magOk = false;

// =========================
// Muestreo
// =========================

const unsigned long sampleInterval = 10;
const unsigned long sendInterval   = 150;

unsigned long lastSample = 0;
unsigned long lastSend = 0;

// =========================================================
// CONSTANTES DEL MODELO
// =========================================================

const float ACCEL_SCALE = 0.000061;
const float GYRO_SCALE  = 0.00875;
const float GRAVITY     = 9.80665;

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
  float a_z = (ay_raw * ACCEL_SCALE) * GRAVITY;

  w_x = gx_raw * GYRO_SCALE;
  w_y = gz_raw * GYRO_SCALE;

  float angulo_roll = atan2(a_z, -a_x) * 180.0 / PI;
  angulo_abs = fabs(angulo_roll);
}

float normalizar(float x, float media, float escala)
{
  return (x - media) / escala;
}

float calcularVelocidad(float angulo_abs, float w_x, float w_y)
{
  float poly[9];
  poly[0] = angulo_abs;
  poly[1] = w_x;
  poly[2] = w_y;
  poly[3] = angulo_abs * angulo_abs;
  poly[4] = angulo_abs * w_x;
  poly[5] = angulo_abs * w_y;
  poly[6] = w_x * w_x;
  poly[7] = w_x * w_y;
  poly[8] = w_y * w_y;

  float f[9];
  for (int i = 0; i < 9; i++) {
    f[i] = normalizar(poly[i], MEAN_2[i], SCALE_2[i]);
  }

  float velocidad = THETA[0];
  for (int i = 0; i < 9; i++) {
    velocidad += THETA[i + 1] * f[i];
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
  Serial.begin(115200);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  WiFi.setSleep(false);
  delay(500);

  udp.begin(udpPort);
  sendMessage("STATUS,ESP32_INICIADA_v2.0");

  configurarWebOTA();
  sendMessage("STATUS,WEB_OTA_LISTA_EN_192.168.4.1");

  Wire.begin(SDA_PIN, SCL_PIN);
  sendMessage("STATUS,I2C_INICIADO");

  // Verificación no bloqueante del LSM6
  if (!imu.init())
  {
    sendMessage("ERROR,LSM6,no_detectado");
    Serial.println("LSM6 no detectado. Continuando sin acelerometro/giroscopio...");
    imuOk = false;
  }
  else
  {
    imu.enableDefault();
    sendMessage("STATUS,LSM6_OK");
    imuOk = true;
  }

  // Verificación no bloqueante del LIS3MDL
  if (!mag.init())
  {
    sendMessage("ERROR,LIS3MDL,no_detectado");
    Serial.println("LIS3MDL no detectado. Continuando sin magnetometro...");
    magOk = false;
  }
  else
  {
    mag.enableDefault();
    sendMessage("STATUS,LIS3MDL_OK");
    magOk = true;
  }
}

// =========================
// LOOP
// =========================

void loop()
{
  server.handleClient();
  yield();

  unsigned long currentTime = millis();

  if (currentTime - lastSample >= sampleInterval)
  {
    lastSample = currentTime;

    float velocidad = 0.0;
    float angulo_abs = 0.0;
    float w_x = 0.0;
    float w_y = 0.0;

    // Solo leer el sensor si está disponible físicamente
    if (imuOk)
    {
      imu.read();
      calcularCaracteristicas(
        imu.a.x, imu.a.y, imu.a.z,
        imu.g.x, imu.g.y, imu.g.z,
        angulo_abs, w_x, w_y
      );
      velocidad = calcularVelocidad(angulo_abs, w_x, w_y);
    }

    if (magOk)
    {
      mag.read();
    }

    if (currentTime - lastSend >= sendInterval)
    {
      lastSend = currentTime;
      char mensaje[64];

      if (imuOk)
      {
        snprintf(mensaje, sizeof(mensaje), "VEL[v2.0],%.3f,%.2f,%.2f,%.2f", velocidad, angulo_abs, w_x, w_y);
      }
      else
      {
        snprintf(mensaje, sizeof(mensaje), "STATUS,SIN_SENSORES_WEB_OTA_ACTIVA");
      }

      sendMessage(mensaje);
    }
  }
}