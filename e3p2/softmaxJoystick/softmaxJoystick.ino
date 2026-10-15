#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <math.h>

// ==============================================================================
// PARÁMETROS DEL MODELO (theta y scaler)
// Pegar aquí el bloque que imprime la última celda del notebook,
// reemplazando todo lo que hay entre INICIO y FIN.
// ==============================================================================
// ===== INICIO MODELO =====
// Generado por el notebook de entrenamiento. No editar a mano.
const int N_BASE = 17;
const int NUM_FEATURES = 68;
const int NUM_CLASSES = 5;
const int WINDOW_SIZE = 20;
const int STEP_SIZE = 5;
const char* CLASS_NAMES[NUM_CLASSES] = { "N", "adelante", "atras", "derecha", "izquierda" };
const float SCALER_MEAN[NUM_FEATURES] = { -8.8880698f, 0.0618243f, 0.6728556f, 4.7847005f, 3.1937240f, -6.3983187f, -9.1250506f, -0.7806896f, 3.3694630f, 9.6981245f, 55.4759026f, 0.0078084f, 0.0683373f, 0.1012723f, 0.4144720f, -6.2152048f, 1.6042182f, 0.2838521f, 0.4180212f, 0.3547435f, 6.5101255f, 12.2089357f, 13.6060418f, 6.3866096f, 3.4325007f, 3.2463014f, 0.2193496f, 17.8697589f, 0.0435803f, 0.0393311f, 0.0247125f, 3.6052230f, 2.9520912f, 0.9764010f, -9.3680052f, -0.6715716f, 0.0779175f, -5.9469053f, -16.0903620f, -28.1810125f, -18.3780847f, -5.2463029f, -0.9391683f, 9.3126165f, 28.9523314f, -0.0676441f, 0.0035305f, 0.0696440f, -5.8335688f, -11.2019123f, -0.0144678f, -8.3993591f, 0.7915077f, 1.2782804f, 15.4225811f, 22.6072191f, 15.4666315f, -0.3808890f, 3.8101259f, 7.8827318f, 10.0969405f, 85.3771991f, 0.0838269f, 0.1359754f, 0.1387906f, 6.7707411f, -1.2048537f, 3.4163506f };
const float SCALER_SCALE[NUM_FEATURES] = { 1.8070504f, 2.2434805f, 2.3934430f, 12.4057141f, 50.8696941f, 48.9430651f, 43.6741834f, 21.3318681f, 34.0019277f, 0.2386272f, 48.4642463f, 0.2748214f, 0.2916036f, 0.1287402f, 12.9012225f, 14.8998839f, 4.9279304f, 0.3133088f, 0.3826814f, 0.3359197f, 5.9822083f, 12.2990571f, 14.5126336f, 7.3308227f, 2.9232867f, 4.2269162f, 0.2479460f, 15.4799875f, 0.0396084f, 0.0391501f, 0.0280619f, 3.1016678f, 2.7725931f, 1.4119975f, 1.5478159f, 2.2619842f, 2.4455518f, 15.5687429f, 53.5896735f, 53.4886970f, 49.0498182f, 22.7837109f, 30.9365408f, 0.4874693f, 35.2284719f, 0.2779573f, 0.2895378f, 0.1116660f, 14.0482818f, 15.0457985f, 3.8701169f, 2.0979466f, 2.3942261f, 2.4442207f, 15.9540948f, 52.4460192f, 54.5195987f, 40.0788993f, 20.2920710f, 37.3813150f, 0.4933062f, 64.3221662f, 0.2798232f, 0.3033298f, 0.1523093f, 14.3183302f, 15.9996650f, 6.8570177f };
const float THETA_B[NUM_CLASSES] = { 3.3147326f, -0.9487036f, -0.9507933f, -0.3353751f, -1.0798607f };
const float THETA_W[NUM_CLASSES][NUM_FEATURES] = {
  { 0.0190006f, -0.0205482f, -0.1584592f, 0.0452066f, -0.0905829f, 0.0613590f, -0.2501907f, 0.2597367f, -0.0758832f, 0.3964913f, 0.2410129f, 0.0467126f, -0.1756326f, 0.2661861f, 0.3362321f, 0.1021904f, 0.1338929f, -0.1004965f, -0.1839093f, -0.2361110f, 0.4139554f, -0.2138732f, -0.1206647f, 0.1160153f, -0.4567969f, 0.0071990f, -0.1565403f, 0.9042333f, -0.2113689f, -0.4133700f, -0.0471498f, -0.4429267f, 0.3120495f, -0.0266241f, 0.0765278f, 0.1717500f, -0.0729649f, 0.0074435f, 0.0491933f, 0.1114154f, -0.1283445f, 0.1496719f, -0.0855999f, -0.0457935f, -0.3694999f, 0.1831050f, 0.0059305f, 0.0692159f, 0.0612108f, 0.1463518f, 0.1867715f, -0.0734986f, 0.2040535f, -0.2135247f, 0.1315508f, -0.1713566f, -0.1412802f, -0.0517797f, -0.0527308f, 0.0302174f, -0.0638786f, 0.1877928f, 0.2373857f, -0.2153633f, -0.0687909f, -0.0474289f, 0.1849679f, 0.0950081f },  // N
  { 0.0070479f, 0.0661564f, 0.1832320f, -0.0405012f, 0.3017893f, 0.1494868f, -0.2924113f, 0.1366719f, 0.0706468f, 0.2140977f, -0.1961022f, 0.0749003f, 0.1838414f, 0.1537638f, -0.1049123f, -0.2131352f, 0.0331160f, 0.0851007f, -0.1036656f, 0.1874218f, -0.1258497f, 0.3105289f, -0.1414705f, -0.0398369f, -0.2160703f, -0.2546377f, -0.0741682f, 0.0772507f, -0.1172088f, 0.2770648f, 0.2202536f, -0.1063756f, 0.0144996f, 0.0978378f, -0.0768823f, 0.0797592f, 0.1807751f, 0.0408995f, 0.1955612f, 0.1953227f, -0.4702904f, 0.1864930f, 0.0341874f, 0.2380031f, -0.1919862f, 0.0897340f, 0.1522894f, 0.1618946f, -0.0596219f, -0.3588349f, -0.0179779f, -0.0197621f, 0.0407753f, 0.2819072f, -0.1898748f, 0.5603779f, 0.0405833f, -0.4474527f, 0.1099977f, -0.0040920f, 0.0506956f, 0.0297073f, 0.0575582f, 0.2717240f, 0.4258728f, -0.1080331f, -0.2791494f, 0.0725415f },  // adelante
  { 0.1963630f, -0.0427197f, -0.0690778f, -0.1481516f, -0.0841743f, -0.1921871f, 0.6070882f, -0.2208169f, 0.2709934f, -0.4222654f, 0.0742838f, -0.0086657f, -0.0299175f, -0.4308724f, 0.2989076f, 0.2175137f, -0.2598715f, -0.1604933f, -0.1717457f, 0.3720643f, 0.1453375f, 0.2051273f, -0.4100272f, 0.7398149f, -0.1123431f, 0.8448747f, 0.0427908f, -0.3180886f, -0.2185043f, 0.4090780f, 0.1246283f, -0.0400935f, 0.0638838f, -0.2732804f, 0.2671969f, 0.0013233f, -0.2139879f, -0.1575807f, -0.2110812f, 0.0569359f, 0.5234274f, -0.0160700f, 0.2128453f, -0.3535154f, 0.3381216f, 0.0501324f, -0.2108932f, -0.4549615f, 0.2405436f, 0.2522102f, -0.2368723f, 0.1953169f, -0.0433008f, -0.0525107f, -0.0147247f, -0.0433124f, -0.3490992f, 0.8169001f, -0.1838702f, 0.2853385f, -0.1463519f, -0.0495804f, -0.0193110f, -0.0133427f, -0.4100425f, 0.2315271f, 0.3006434f, -0.1856830f },  // atras
  { 0.0142978f, -0.0629136f, -0.0523159f, -0.4080136f, -0.0154796f, 0.2539397f, -0.2016077f, 0.1024418f, -0.1186247f, 0.7375052f, -0.3476406f, -0.2138709f, -0.0591493f, 0.0410486f, -0.3000514f, 0.0695665f, -0.3318615f, 0.3127775f, 0.1329709f, -0.2825962f, -0.4714834f, -0.1847407f, 0.6013606f, -0.2427719f, 0.5924882f, -0.5382881f, 0.2819712f, -0.3597660f, 0.2461522f, -0.2336528f, 0.0023102f, 0.0420636f, -0.3195488f, -0.1098790f, -0.0995448f, -0.2374914f, 0.0696888f, -0.1417869f, 0.0430195f, -0.4601735f, -0.1316242f, 0.0554905f, 0.0227057f, 0.2807701f, -0.0796003f, -0.3268189f, 0.0347911f, 0.1718048f, 0.0642180f, 0.0838957f, -0.4015042f, 0.0635685f, -0.2683670f, -0.0731682f, -0.4351620f, -0.1627465f, 0.3020258f, -0.3015926f, 0.3736522f, -0.1749595f, 0.6545451f, -0.0485055f, -0.3251985f, -0.0848626f, 0.1304721f, -0.0867418f, -0.1233566f, -0.4013097f },  // derecha
  { -0.2367092f, 0.0600251f, 0.0966209f, 0.5514598f, -0.1115524f, -0.2725984f, 0.1371215f, -0.2780335f, -0.1471323f, -0.9258289f, 0.2284461f, 0.1009237f, 0.0808581f, -0.0301261f, -0.2301760f, -0.1761354f, 0.4247241f, -0.1368883f, 0.3263498f, -0.0407790f, 0.0380402f, -0.1170423f, 0.0708018f, -0.5732215f, 0.1927220f, -0.0591479f, -0.0940536f, -0.3036294f, 0.3009298f, -0.0391200f, -0.3000423f, 0.5473322f, -0.0708840f, 0.3119457f, -0.1672975f, -0.0153411f, 0.0364890f, 0.2510247f, -0.0766928f, 0.0964994f, 0.2068317f, -0.3755854f, -0.1841384f, -0.1194643f, 0.3029647f, 0.0038475f, 0.0178821f, 0.0520461f, -0.3063505f, -0.1236228f, 0.4695828f, -0.1656247f, 0.0668391f, 0.0572964f, 0.5082108f, -0.1829623f, 0.1477702f, -0.0160750f, -0.2470488f, -0.1365045f, -0.4950103f, -0.1194142f, 0.0495657f, 0.0418446f, -0.0775115f, 0.0106766f, -0.0831053f, 0.4194432f },  // izquierda
};
// ===== FIN MODELO =====

// ===================== WI-FI / UDP =====================
const char* ssid = "Guante_IMU";
const char* password = "12345678";
const unsigned int udpPort = 5000;
WiFiUDP udp;
IPAddress pcIP(192, 168, 4, 255);   // broadcast hasta que el PC escriba
unsigned int pcPort = udpPort;

void sendMessage(const char* msg) {
  udp.beginPacket(pcIP, pcPort);
  udp.print(msg);
  udp.endPacket();
}

// ===================== SENSORES =====================
#define SDA_PIN 13
#define SCL_PIN 14
LSM6 imu;
LIS3MDL mag;
bool sensores_ok = false;

// Mismas conversiones que receptorDB2.py
const float K_ACC = 0.000061f * 9.80665f;  // cuentas -> m/s^2
const float K_GYR = 0.00875f;              // cuentas -> °/s
const float K_MAG = 0.15f;                 // cuentas -> unidades del CSV

const unsigned long sampleInterval = 10;   // 100 Hz, igual que la grabación
unsigned long lastSample = 0;

// Referencia del magnetómetro (posición neutra al encender)
const int N_REF = 100;                     // 1 s quieto
float mref[3] = {0, 0, 0};
float mref_acc[3] = {0, 0, 0};
int ref_count = 0;
bool ref_lista = false;

// BEGIN_CORE (C puro: igual a features_base() y ventanas del notebook)
float ventana[WINDOW_SIZE][N_BASE];
int idx_ventana = 0, llenas = 0, paso = 0;

void features_base(const float a[3], const float w[3], const float m[3],
                   const float ref[3], float f[N_BASE]) {
  float ax = a[0], ay = a[1], az = a[2];
  float wx = w[0], wy = w[1], wz = w[2];
  float nm = sqrtf(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
  float nr = sqrtf(ref[0]*ref[0] + ref[1]*ref[1] + ref[2]*ref[2]);
  float c = (m[0]*ref[0] + m[1]*ref[1] + m[2]*ref[2]) / (nm * nr + 1e-6f);
  if (c > 1) c = 1;
  if (c < -1) c = -1;
  f[0] = ax;  f[1] = ay;  f[2] = az;
  f[3] = wx;  f[4] = wy;  f[5] = wz;
  f[6] = m[0] - ref[0];  f[7] = m[1] - ref[1];  f[8] = m[2] - ref[2];  // mag relativo
  f[9]  = sqrtf(ax*ax + ay*ay + az*az);
  f[10] = sqrtf(wx*wx + wy*wy + wz*wz);
  f[11] = atan2f(ay, sqrtf(ax*ax + az*az));   // inclinación y
  f[12] = atan2f(az, sqrtf(ax*ax + ay*ay));   // inclinación z
  f[13] = acosf(c);                           // giro respecto a la referencia
  f[14] = ax*ay;  f[15] = ax*az;  f[16] = ay*az;
}

// media, desviación (ddof=0), mínimo y máximo de la ventana
void estadisticas(float x[NUM_FEATURES]) {
  for (int j = 0; j < N_BASE; j++) {
    float s = 0, vmin = ventana[0][j], vmax = ventana[0][j];
    for (int i = 0; i < WINDOW_SIZE; i++) {
      float v = ventana[i][j];
      s += v;
      if (v < vmin) vmin = v;
      if (v > vmax) vmax = v;
    }
    float media = s / WINDOW_SIZE, sq = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      float d = ventana[i][j] - media;
      sq += d * d;
    }
    x[j] = media;
    x[j + N_BASE] = sqrtf(sq / WINDOW_SIZE);
    x[j + 2*N_BASE] = vmin;
    x[j + 3*N_BASE] = vmax;
  }
}

// Softmax: z_k = theta_k0 + sum(theta_kj * x~_j),  P_k = exp(z_k) / sum(exp(z_i))
int predecir(const float x[NUM_FEATURES], float prob[NUM_CLASSES]) {
  float z[NUM_CLASSES], zmax = -1e30f;
  for (int k = 0; k < NUM_CLASSES; k++) {
    z[k] = THETA_B[k];
    for (int j = 0; j < NUM_FEATURES; j++)
      z[k] += THETA_W[k][j] * (x[j] - SCALER_MEAN[j]) / SCALER_SCALE[j];  // normaliza
    if (z[k] > zmax) zmax = z[k];
  }
  float suma = 0;
  int mejor = 0;
  for (int k = 0; k < NUM_CLASSES; k++) {
    prob[k] = expf(z[k] - zmax);
    suma += prob[k];
  }
  for (int k = 0; k < NUM_CLASSES; k++) {
    prob[k] /= suma;
    if (prob[k] > prob[mejor]) mejor = k;
  }
  return mejor;
}

// Agrega una muestra; devuelve la clase o -1 si aún no toca predecir
int nueva_muestra(const float a[3], const float w[3], const float m[3],
                  const float ref[3], float prob[NUM_CLASSES]) {
  features_base(a, w, m, ref, ventana[idx_ventana]);   // buffer circular
  idx_ventana = (idx_ventana + 1) % WINDOW_SIZE;
  if (llenas < WINDOW_SIZE) llenas++;
  paso++;
  if (llenas < WINDOW_SIZE || paso < STEP_SIZE) return -1;
  paso = 0;
  float x[NUM_FEATURES];
  estadisticas(x);
  return predecir(x, prob);
}
// END_CORE

void reiniciar_referencia() {
  ref_count = 0;
  ref_lista = false;
  llenas = 0;
  mref_acc[0] = mref_acc[1] = mref_acc[2] = 0;
  sendMessage("STATUS,CALIBRANDO_NEUTRO_1s");
  Serial.println("Calibrando neutro: deje el joystick quieto 1 s");
}

// Guarda la IP del PC y atiende el comando ZERO
void revisarUDP() {
  int n = udp.parsePacket();
  if (!n) return;
  pcIP = udp.remoteIP();
  pcPort = udp.remotePort();
  char buf[16] = {0};
  udp.read(buf, sizeof(buf) - 1);
  if (strncmp(buf, "ZERO", 4) == 0) reiniciar_referencia();
}

void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  delay(100);
  udp.begin(udpPort);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(100);
  bool ok_imu = imu.init();
  bool ok_mag = mag.init();
  if (ok_imu) imu.enableDefault();   // ±2 g, 245 °/s (igual que al grabar)
  if (ok_mag) mag.enableDefault();
  sensores_ok = ok_imu && ok_mag;

  Serial.println(sensores_ok ? "Sensores OK" : "ERROR: sensor no detectado");
  reiniciar_referencia();
}

void loop() {
  revisarUDP();
  if (!sensores_ok) {
    sendMessage("ERROR,SENSOR");
    delay(1000);
    return;
  }
  unsigned long t = millis();
  if (t - lastSample < sampleInterval) return;
  lastSample = t;

  imu.read();
  mag.read();

  // Unidades + intercambio Y<->Z, exactamente como receptorDB2.py
  float a[3] = { imu.a.x * K_ACC, imu.a.z * K_ACC, imu.a.y * K_ACC };
  float w[3] = { imu.g.x * K_GYR, imu.g.z * K_GYR, imu.g.y * K_GYR };
  float m[3] = { mag.m.x * K_MAG, mag.m.z * K_MAG, mag.m.y * K_MAG };

  // Primer segundo: referencia del magnetómetro en neutro
  if (!ref_lista) {
    for (int i = 0; i < 3; i++) mref_acc[i] += m[i];
    if (++ref_count >= N_REF) {
      for (int i = 0; i < 3; i++) mref[i] = mref_acc[i] / N_REF;
      ref_lista = true;
      sendMessage("STATUS,LISTO");
      Serial.println("Listo");
    }
    return;
  }

  float prob[NUM_CLASSES];
  int k = nueva_muestra(a, w, m, mref, prob);
  if (k < 0) return;

  char msg[64];
  snprintf(msg, sizeof(msg), "PRED,%s,%.2f", CLASS_NAMES[k], prob[k]);
  sendMessage(msg);
  Serial.println(msg);
}
