#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

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

const unsigned long sampleInterval = 10;  // 100 Hz

unsigned long lastSample = 0;


// =========================
// Enviar mensaje UDP
// =========================

void sendMessage(const char* message)
{
  udp.beginPacket(pcIP, udpPort);
  udp.print(message);
  udp.endPacket();
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


    // Crear paquete
    char data[180];

    snprintf(
      data,
      sizeof(data),

      "%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d",

      currentTime,

      imu.a.x,
      imu.a.y,
      imu.a.z,

      imu.g.x,
      imu.g.y,
      imu.g.z,

      mag.m.x,
      mag.m.y,
      mag.m.z
    );


    // Enviar datos al PC
    udp.beginPacket(pcIP, udpPort);
    udp.print(data);
    udp.endPacket();
  }
}