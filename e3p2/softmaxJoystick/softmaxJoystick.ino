#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <math.h>

// ==============================================================================
// 1. CONFIGURACIÓN WI-FI Y UDP
// ==============================================================================

const char* ssid = "Guante_IMU";
const char* password = "12345678";

const unsigned int udpPort = 5000;
WiFiUDP udp;

IPAddress pcIP(192, 168, 4, 255); 
unsigned int pcPort = udpPort;
bool pcConnected = false;

// Estado de los sensores
bool lsm6_ok = false;
bool lis3mdl_ok = false;

void sendMessage(const char* message) {
  udp.beginPacket(pcIP, pcPort);
  udp.print(message);
  udp.endPacket();
}

void actualizarIPCliente() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    pcIP = udp.remoteIP();
    pcPort = udp.remotePort();
    pcConnected = true;
  }
}

// ==============================================================================
// 2. PARÁMETROS DEL MODELO Y PREDICCIÓN
// ==============================================================================

const int NUM_FEATURES = 76;
const int NUM_CLASSES = 5;

const char* CLASS_NAMES[NUM_CLASSES] = {
  "N", "adelante", "atras", "izquierda", "derecha"
};

const float SCALER_MEAN[NUM_FEATURES] = { -8.843102f, -0.025414f, 0.421956f, 5.172899f, 2.269647f, -6.094260f, -55.872810f, -63.972409f, 10.063589f, 9.699118f, 57.044426f, 234.748204f, -0.403386f, 1.289833f, -0.936661f, 0.216854f, 0.810340f, -4.791716f, 1.673755f, 0.297028f, 0.427257f, 0.374194f, 6.893499f, 12.927133f, 13.755418f, 6.335418f, 3.576971f, 3.739085f, 0.233794f, 18.586306f, 5.116888f, 0.458050f, 0.053482f, 0.131066f, 0.025257f, 3.638327f, 3.106577f, 1.069220f, -9.346291f, -0.774264f, -0.206230f, -6.024267f, -18.181639f, -28.186434f, -64.956884f, -68.629484f, 5.021558f, 9.291558f, 29.578999f, 227.692869f, -1.081678f, 1.196933f, -1.100644f, 0.182737f, -5.546886f, -10.011796f, -0.107553f, -8.334696f, 0.728316f, 1.059468f, 16.610068f, 22.794442f, 16.008466f, -47.153216f, -59.139402f, 15.293478f, 10.123795f, 88.243844f, 241.976622f, 0.432860f, 1.375746f, -0.776056f, 0.253100f, 7.233305f, 0.531588f, 3.677197f };
const float SCALER_SCALE[NUM_FEATURES] = { 1.830464f, 2.206545f, 2.620323f, 13.227621f, 53.470074f, 48.364095f, 78.828713f, 40.726824f, 208.002869f, 0.241892f, 49.136463f, 58.654277f, 1.137813f, 0.334768f, 1.621160f, 0.286057f, 12.957087f, 15.931982f, 4.838368f, 0.323647f, 0.386128f, 0.346983f, 6.505030f, 12.675452f, 14.657004f, 7.188953f, 3.148248f, 5.320749f, 0.252309f, 15.808170f, 6.077801f, 0.605583f, 0.048266f, 0.537852f, 0.032140f, 3.121255f, 2.858980f, 1.558862f, 1.563260f, 2.255472f, 2.727070f, 16.130904f, 56.232956f, 53.535960f, 82.727759f, 40.015360f, 208.440915f, 0.491059f, 35.419093f, 57.350532f, 1.294286f, 0.374051f, 1.620579f, 0.278994f, 14.038081f, 15.759791f, 3.917317f, 2.127033f, 2.341283f, 2.613305f, 17.810160f, 55.232535f, 53.921277f, 75.382323f, 41.560804f, 207.735028f, 0.507230f, 65.483400f, 60.227697f, 1.416809f, 0.294392f, 1.787719f, 0.298053f, 14.479670f, 17.480592f, 7.019462f };

const float THETA_W[5][76] = {
    { -1.773457f, -0.255190f, 0.746362f, 0.055974f, -0.388088f, 0.786395f, 0.159556f, -0.119267f, 0.261996f, -3.678699f, -4.996797f, -0.541191f, 0.252799f, 1.849189f, -1.160640f, -0.247956f, 0.195882f, -0.863346f, -1.177081f, -2.379232f, -1.756658f, -2.281898f, -0.566122f, -3.843086f, -2.914969f, -1.374070f, -0.109944f, -1.925742f, -1.975380f, -4.698447f, -1.457212f, -0.206953f, -2.408865f, -1.247647f, -0.918896f, -1.327819f, -1.947292f, -2.608264f, -1.573262f, 0.361719f, 1.178907f, 0.111715f, 0.910383f, 1.894803f, 0.452740f, -0.079666f, 0.345950f, 0.141153f, -3.863163f, -0.263019f, 0.415313f, 2.111369f, -0.634220f, -0.158229f, 0.024390f, -0.494862f, 0.132736f, -2.159979f, -0.333232f, 0.386112f, -0.488695f, -1.797384f, -0.597452f, 0.001750f, -0.250625f, 0.189016f, -2.681684f, -5.636264f, -0.859813f, -0.814006f, 1.603726f, -1.514057f, -0.550330f, -0.483014f, -1.283661f, -1.930355f },
    { 0.389119f, -0.323640f, 1.084502f, -0.016284f, -1.443672f, 0.225293f, 0.022443f, 0.144234f, 0.200123f, 2.300863f, -1.039868f, 0.281743f, -0.323127f, 0.308581f, 0.224026f, -0.254363f, -0.446101f, 1.285718f, -0.187617f, 1.438223f, -0.645929f, 0.854634f, -0.243330f, 2.048105f, -0.600373f, 1.325628f, -0.050960f, -0.631945f, 0.278995f, -1.791242f, -0.083082f, -0.090100f, 0.552709f, -0.078528f, -0.423021f, -0.290912f, -0.389939f, -0.293198f, -0.186546f, 0.140976f, 1.585351f, 0.820885f, 0.001989f, 0.055563f, -0.317195f, 0.143528f, 0.200267f, 0.930793f, -0.144089f, 0.279705f, -1.174254f, 0.234440f, 0.008196f, 0.056579f, -0.333269f, -0.060149f, 0.015295f, 0.511303f, -0.492929f, 2.122941f, 0.512312f, 1.910898f, -0.462172f, -0.017871f, 0.072322f, 0.120082f, 0.202381f, -1.259594f, 0.164843f, -0.157833f, 0.683062f, -0.127668f, -0.065848f, -0.925936f, -0.108381f, -0.263159f },
    { -1.338391f, 0.387010f, -0.878890f, -0.539859f, 1.095390f, -0.503788f, -0.084459f, -0.217710f, -0.022143f, -1.471341f, 0.039519f, -0.092315f, -0.367228f, -0.875329f, -0.277388f, 0.237293f, 0.115984f, -1.666531f, -0.503187f, -0.814877f, -0.755399f, 0.654889f, 0.269127f, 0.737758f, -1.166865f, -1.527202f, -0.345982f, 1.731342f, 0.292392f, -0.378566f, -0.898812f, 0.171595f, -0.695307f, -0.012269f, 0.607725f, -0.333059f, 0.860418f, 0.171640f, -0.840588f, 0.141266f, -2.036449f, -0.652716f, -1.776456f, 0.623061f, 0.266723f, 0.047636f, -0.098352f, -1.129672f, 0.472224f, -0.098471f, 0.229079f, -0.707303f, -0.092479f, -0.218674f, 0.604609f, -0.532025f, -0.780081f, -1.193477f, -0.095242f, -1.689454f, -0.433281f, -0.550514f, -0.324750f, 0.090192f, -0.112533f, 0.055372f, -0.526574f, 0.238751f, -0.125521f, 0.225157f, -1.062996f, -0.118458f, 0.116244f, 0.622032f, 0.221378f, -0.069107f },
    { -0.572996f, -1.474396f, 0.024638f, -0.760520f, 0.199537f, -0.024660f, -0.114823f, -0.164123f, -0.426089f, 1.760987f, -1.586359f, 0.245554f, 1.527683f, -0.223944f, 0.982766f, 0.015505f, -1.069192f, 0.094761f, -0.306957f, 0.591875f, 0.639080f, -1.645731f, -0.052357f, -1.990108f, 3.031491f, 0.287977f, -0.455426f, -0.741208f, 0.213126f, -1.894107f, 0.316589f, -0.605681f, 0.069091f, 0.223907f, -0.586061f, 0.827332f, -1.211002f, -0.573468f, -0.865107f, -2.307509f, 0.354727f, -0.959847f, 0.853034f, -0.923243f, -0.075992f, -0.109338f, -0.377644f, 0.497482f, -0.732273f, 0.140571f, 0.778912f, -0.217482f, 0.465390f, 0.113580f, -0.325800f, 0.418631f, -0.118346f, -0.438379f, -1.540100f, -0.291191f, -0.662203f, -0.490334f, 2.489117f, -0.034138f, -0.166231f, -0.449676f, 0.762378f, -1.109012f, 0.134305f, 0.776955f, 0.122295f, 0.808700f, -0.115873f, 0.607440f, -0.167298f, -0.525992f },
    { -0.394624f, 1.148066f, 0.688174f, 0.742733f, -0.615823f, -0.129209f, 0.104374f, 0.368974f, 0.165355f, -0.942371f, 0.604788f, -0.108855f, -0.815853f, 0.722784f, -0.591789f, -0.534028f, 1.050449f, 0.105552f, 0.781680f, -0.288015f, 1.193133f, -1.204380f, 0.022490f, -1.120544f, 1.376086f, -1.244577f, 0.189277f, -0.970203f, -0.562080f, -0.353149f, -0.678041f, 0.283409f, -0.029885f, 0.298835f, -0.953954f, 0.532535f, -1.192094f, 0.338601f, -0.175608f, 1.023617f, 0.880083f, 1.026954f, 0.040345f, -1.802444f, 0.294744f, 0.272499f, 0.211525f, -0.269022f, 1.071202f, 0.005592f, -0.315854f, 0.470960f, -1.019262f, -0.358180f, -0.268033f, 0.282532f, 0.984338f, -0.036479f, 1.932370f, 0.525890f, 0.975600f, -0.875086f, -0.528221f, -0.070866f, 0.413629f, 0.131748f, -0.996913f, 0.500913f, -0.206611f, -0.305333f, 0.767605f, -0.882369f, -0.709054f, 0.546696f, -0.260072f, 0.869856f }
};

const float THETA_B[NUM_CLASSES] = { -54.798575f, -3.417022f, -3.626348f, -9.029684f, -5.959866f };

int predecir_direccion(float x_raw[]) {
  float x_scaled[NUM_FEATURES];
  for (int j = 0; j < NUM_FEATURES; j++) {
    x_scaled[j] = (x_raw[j] - SCALER_MEAN[j]) / SCALER_SCALE[j];
  }

  float max_z = -1e9;
  int mejor_clase = 0;

  for (int k = 0; k < NUM_CLASSES; k++) {
    float z_k = THETA_B[k];
    for (int j = 0; j < NUM_FEATURES; j++) {
      z_k += THETA_W[k][j] * x_scaled[j];
    }
    if (z_k > max_z) {
      max_z = z_k;
      mejor_clase = k;
    }
  }
  return mejor_clase;
}

// ==============================================================================
// 3. I2C, SENSORES Y FEATURE ENGINEERING
// ==============================================================================

#define SDA_PIN 13
#define SCL_PIN 14

LSM6 imu;
LIS3MDL mag;

const unsigned long sampleInterval = 10; 
unsigned long lastSample = 0;

const int WINDOW_SIZE = 20;
const int STEP_SIZE = 5;
const int RAW_FEATURES = 19;

float window_buffer[WINDOW_SIZE][RAW_FEATURES];
int sample_count = 0;
int step_counter = 0;

void calcular_features_instantaneas(float ax, float ay, float az, 
                                     float wx, float wy, float wz, 
                                     float mx, float my, float mz, 
                                     float out_feat[19]) {
  float a_mag = sqrt(ax*ax + ay*ay + az*az);
  float w_mag = sqrt(wx*wx + wy*wy + wz*wz);
  float m_mag = sqrt(mx*mx + my*my + mz*mz);

  float roll_acc  = atan2(ay, az);
  float pitch_acc = atan2(-ax, sqrt(ay*ay + az*az));
  float roll_mag  = atan2(my, mz);
  float pitch_mag = atan2(-mx, sqrt(my*my + mz*mz));

  float ax_ay = ax * ay;
  float ax_az = ax * az;
  float ay_az = ay * az;

  out_feat[0] = ax;       out_feat[1] = ay;       out_feat[2] = az;
  out_feat[3] = wx;       out_feat[4] = wy;       out_feat[5] = wz;
  out_feat[6] = mx;       out_feat[7] = my;       out_feat[8] = mz;
  out_feat[9] = a_mag;    out_feat[10] = w_mag;   out_feat[11] = m_mag;
  out_feat[12] = roll_acc; out_feat[13] = pitch_acc;
  out_feat[14] = roll_mag; out_feat[15] = pitch_mag;
  out_feat[16] = ax_ay;   out_feat[17] = ax_az;   out_feat[18] = ay_az;
}

void extraer_estadisticas_ventana(float x_raw[76]) {
  for (int col = 0; col < RAW_FEATURES; col++) {
    float sum = 0.0f;
    float vmin = window_buffer[0][col];
    float vmax = window_buffer[0][col];

    for (int i = 0; i < WINDOW_SIZE; i++) {
      float val = window_buffer[i][col];
      sum += val;
      if (val < vmin) vmin = val;
      if (val > vmax) vmax = val;
    }

    float mean = sum / WINDOW_SIZE;

    float sq_diff_sum = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      float diff = window_buffer[i][col] - mean;
      sq_diff_sum += diff * diff;
    }
    float std_dev = sqrt(sq_diff_sum / WINDOW_SIZE);

    x_raw[col]                  = mean;
    x_raw[col + RAW_FEATURES]   = std_dev;
    x_raw[col + 2*RAW_FEATURES] = vmin;
    x_raw[col + 3*RAW_FEATURES] = vmax;
  }
}

void procesar_lectura_e_inferir(float ax, float ay, float az, 
                                float wx, float wy, float wz, 
                                float mx, float my, float mz) {
  float feat_19[RAW_FEATURES];
  calcular_features_instantaneas(ax, ay, az, wx, wy, wz, mx, my, mz, feat_19);

  if (sample_count >= WINDOW_SIZE) {
    for (int i = 0; i < WINDOW_SIZE - 1; i++) {
      for (int j = 0; j < RAW_FEATURES; j++) {
        window_buffer[i][j] = window_buffer[i + 1][j];
      }
    }
    for (int j = 0; j < RAW_FEATURES; j++) {
      window_buffer[WINDOW_SIZE - 1][j] = feat_19[j];
    }
  } else {
    for (int j = 0; j < RAW_FEATURES; j++) {
      window_buffer[sample_count][j] = feat_19[j];
    }
    sample_count++;
  }

  step_counter++;

  if (sample_count == WINDOW_SIZE && step_counter >= STEP_SIZE) {
    step_counter = 0;

    float x_raw_76[76];
    extraer_estadisticas_ventana(x_raw_76);

    int clase_id = predecir_direccion(x_raw_76);

    char mensaje_prediccion[64];
    snprintf(mensaje_prediccion, sizeof(mensaje_prediccion), "PRED,%s", CLASS_NAMES[clase_id]);
    sendMessage(mensaje_prediccion);
  }
}

// ==============================================================================
// 4. SETUP Y LOOP PRINCIPAL
// ==============================================================================

unsigned long lastHeartbeat = 0;
unsigned long loopCount = 0;

void setup() {
  // Inicialización de punto de acceso y UDP de forma rápida
  WiFi.softAP(ssid, password);
  delay(100);
  udp.begin(udpPort);

  // Intentar inicializar I2C sin bloquer el microcontrolador
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(100); // Evitar cuelgue indefinido si las líneas SDA/SCL fallan

  // Probar LSM6
  if (imu.init()) {
    imu.enableDefault();
    lsm6_ok = true;
  }

  // Probar LIS3MDL
  if (mag.init()) {
    mag.enableDefault();
    lis3mdl_ok = true;
  }
}

void loop() {
  actualizarIPCliente();

  unsigned long currentTime = millis();
  loopCount++;

  // 1. Mensaje de Latido (Heartbeat) enviado cada 1 segundo por Wi-Fi
  if (currentTime - lastHeartbeat >= 1000) {
    lastHeartbeat = currentTime;

    char diagBuf[128];
    snprintf(diagBuf, sizeof(diagBuf), "DIAG,loop=%lu,LSM6=%s,LIS3MDL=%s,PC_IP=%d.%d.%d.%d",
             loopCount,
             lsm6_ok ? "OK" : "FAIL",
             lis3mdl_ok ? "OK" : "FAIL",
             pcIP[0], pcIP[1], pcIP[2], pcIP[3]);
             
    sendMessage(diagBuf);
  }

  // 2. Procesar lectura de sensores solo si ambos inicializaron bien
  if (lsm6_ok && lis3mdl_ok) {
    if (currentTime - lastSample >= sampleInterval) {
      lastSample = currentTime;

      imu.read();
      mag.read();

      procesar_lectura_e_inferir(
        imu.a.x, imu.a.y, imu.a.z,
        imu.g.x, imu.g.y, imu.g.z,
        mag.m.x, mag.m.y, mag.m.z
      );
    }
  }
}