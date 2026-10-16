// Guante IMU - MODO BÁSICO y MODO AVANZADO
// Básico:   velocidad = regresión polinómica | dirección = regresión softmax
// Avanzado: velocidad = MLP 3-8-8-1          | dirección = MLP 52-8-5
// En cada modo los dos modelos corren a la vez. Cambio de modo: tecla M en la plataforma.
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>
#include <math.h>

// ===================== RED =====================
const char* ssid = "Guante_IMU";
const char* password = "12345678";
const unsigned int udpPort = 5000;
IPAddress destino(192, 168, 4, 255);   // broadcast: carrito y PC reciben
WiFiUDP udp;

// IP del PC (la aprende cuando la plataforma envía HOLA)
IPAddress pcIP;
uint16_t pcPort = 0;
unsigned long t_pc = 0;

void enviar(const char* msg) {
  udp.beginPacket(destino, udpPort);   // broadcast (carrito)
  udp.print(msg);
  udp.endPacket();
  if (pcPort && millis() - t_pc < 5000) {   // directo al PC
    udp.beginPacket(pcIP, pcPort);
    udp.print(msg);
    udp.endPacket();
  }
}

// ===================== SENSORES =====================
#define SDA_PIN 13
#define SCL_PIN 14
LSM6 imu;
LIS3MDL mag;
bool sensores_ok = false;

const float K_ACC = 0.000061f * 9.80665f;  // m/s^2
const float K_GYR = 0.00875f;              // °/s
const float K_MAG = 0.15f;                 // unidades del CSV
const unsigned long T_MUESTRA = 10;        // 100 Hz

// IMU girada 180° (eje vertical) respecto a la grabación del CSV:
// true = el software la devuelve a la orientación de entrenamiento
const bool ROTAR_180 = false;

// (convención de qué lado del joystick es el frente). true = se intercambian.
const bool INTERCAMBIAR_ADEL_ATRAS = true;
unsigned long t_ultima = 0;

// Modo de funcionamiento: false = básico, true = avanzado (tecla M en la plataforma)
bool avanzado = false;

// ===================== MODELO 1: VELOCIDAD (polinómica) =====================
// Escalador y theta del notebook Modelo_Joystick_Velocidad.ipynb
const float MEAN_P[9]  = {38.1009651f, 7.73562638f, -1.76439703f, 2122.29661f, 448.215522f, -143.665112f, 437.159867f, -274.597072f, 1985.97891f};
const float SCALE_P[9] = {25.89619806f, 19.42472526f, 44.5293814f, 2189.85809529f, 1286.45872902f, 2274.87676336f, 1313.35371098f, 1229.23159976f, 3294.56183187f};
const float THETA_P[10] = {0.66657489f, 0.80386927f, -0.01309359f, -0.14718238f, -0.46418315f, 0.01453122f, 0.11901392f, 0.00294823f, 0.02212228f, 0.0322368f};

// Polinomio con valores crudos y un solo escalado 
float velocidad_poly(const float a[3], const float w[3]) {
  float ang = fabsf(atan2f(a[2], -a[0]) * 180.0f / PI);   // angulo_abs
  float x0 = ang, x1 = w[0], x2 = w[1];                  // angulo_abs, w_x, w_y
  float p[9] = {x0, x1, x2, x0*x0, x0*x1, x0*x2, x1*x1, x1*x2, x2*x2};
  float v = THETA_P[0];
  for (int i = 0; i < 9; i++) v += THETA_P[i + 1] * (p[i] - MEAN_P[i]) / SCALE_P[i];
  if (v < 0) v = 0;
  if (v > 1) v = 1;
  return v;
}

// ===================== MODELO 2: DIRECCIÓN (softmax) =====================
// Bloque generado: softmax sin magnetómetro + posturas sostenidas 
const int N_BASE = 13;
const int NUM_FEATURES = 52;
const int NUM_CLASSES = 5;
const int WINDOW_SIZE = 20;
const int STEP_SIZE = 5;
const char* CLASS_NAMES[NUM_CLASSES] = { "N", "adelante", "atras", "derecha", "izquierda" };
const float SCALER_MEAN[NUM_FEATURES] = { -8.8292568f, 0.0626529f, 0.8564949f, 4.6620553f, 2.6033190f, -6.2295737f, 9.7346594f, 52.3517115f, 0.0078081f, 0.0890936f, 0.3102645f, -7.5642447f, 1.7420291f, 0.2974682f, 0.4509511f, 0.3809389f, 6.8444259f, 13.7248831f, 15.3927960f, 0.2697148f, 20.2103969f, 0.0460244f, 0.0394091f, 3.8236058f, 3.1337992f, 1.1239990f, -9.3112849f, -0.7190907f, 0.2375195f, -6.6979597f, -18.8269732f, -30.4871117f, 9.3016196f, 25.0114998f, -0.0708589f, 0.0247238f, -6.2522891f, -12.8614099f, -0.0166599f, -8.3162729f, 0.8369918f, 1.5022828f, 15.8660420f, 24.9584913f, 19.0293022f, 10.2285696f, 88.5354016f, 0.0870023f, 0.1561782f, 7.0160437f, -2.4097057f, 3.9056475f };
const float SCALER_SCALE[NUM_FEATURES] = { 1.7790979f, 2.2935256f, 2.6287183f, 11.7301602f, 46.9164089f, 45.4724595f, 0.2619869f, 45.2128633f, 0.2739465f, 0.3140506f, 14.4107584f, 16.6207200f, 4.8870022f, 0.3850416f, 0.4665839f, 0.4442042f, 5.8677349f, 13.8783152f, 16.7303810f, 0.3523501f, 16.1104241f, 0.0484423f, 0.0437537f, 3.5423389f, 3.3424092f, 1.5798880f, 1.5509351f, 2.4061512f, 2.7867034f, 15.4469870f, 52.2146946f, 52.8687379f, 0.5120786f, 32.9930735f, 0.2825122f, 0.3215475f, 16.1196632f, 17.4563790f, 4.0956867f, 2.1156534f, 2.5066935f, 2.7263024f, 15.6085771f, 50.9216579f, 54.3555925f, 0.6957697f, 61.4652602f, 0.2870522f, 0.3237326f, 16.1907419f, 17.8974204f, 7.0923594f };
const float THETA_B[NUM_CLASSES] = { 3.2288540f, -0.9176064f, -1.0030050f, -0.7959783f, -0.5122643f };
const float THETA_W[NUM_CLASSES][NUM_FEATURES] = {
  { 0.0542465f, -0.0217304f, -0.3666406f, 0.0415954f, -0.1412930f, -0.0902908f, 0.1288378f, 1.1413350f, -0.0132066f, -0.3724298f, 0.1526154f, 0.3120785f, 0.0935067f, 0.0498691f, -0.0520027f, -0.1422001f, 0.2511093f, -0.0239032f, -0.4279977f, -0.2409227f, 0.8065680f, -0.1142950f, -0.4716553f, -0.3738608f, 0.1629072f, -0.1780929f, 0.0686464f, 0.1808845f, -0.0580196f, -0.0417111f, 0.2482889f, 0.4132111f, -0.4401021f, -0.5723780f, 0.1604152f, 0.0814068f, 0.0513440f, 0.0569038f, 0.3685005f, 0.0895013f, 0.1672986f, -0.1788586f, 0.0509809f, 0.0121362f, -0.2076496f, -0.3827540f, 0.2062072f, 0.1458458f, -0.1856873f, -0.2314097f, 0.0182261f, 0.0209974f },  // N
  { 0.1148025f, 0.0047793f, 0.4613084f, 0.0937447f, 0.0603077f, 0.1766537f, 0.4250883f, -0.3624007f, -0.0133035f, 0.4265499f, -0.1228335f, -0.5834946f, 0.1855458f, 0.1798271f, -0.0833516f, 0.0823605f, -0.1648551f, 0.2442411f, -0.2567615f, 0.2502558f, -0.1976627f, -0.0856099f, 0.0507012f, -0.1474246f, 0.0997748f, 0.2159159f, -0.0679644f, -0.0136381f, 0.4727156f, 0.2220144f, 0.2476746f, 0.2526179f, 0.1676366f, 0.0848352f, -0.0272925f, 0.4351483f, -0.0189631f, -0.7848073f, 0.0139822f, 0.0896867f, -0.0464959f, 0.5589729f, -0.0009255f, 0.6387713f, 0.0589918f, 0.6508361f, -0.0189613f, -0.0585198f, 0.4705827f, -0.0968849f, -0.6305046f, 0.1838274f },  // adelante
  { 0.1129363f, -0.0146889f, -0.2092869f, -0.3964208f, 0.3267881f, -0.2542666f, -0.4644986f, -0.0836211f, 0.0825846f, -0.1511441f, 0.4845629f, 0.4782586f, -0.4065641f, -0.1616005f, -0.3598409f, 0.5534646f, 0.2072227f, 0.5578889f, -0.5684742f, -0.2966917f, -0.0893393f, -0.3576956f, 0.9331856f, -0.3471596f, 0.3712436f, -0.0330713f, 0.2694212f, 0.1317019f, -0.5803909f, -0.1803105f, -0.5548591f, 0.1544771f, 0.0198411f, 0.2424425f, 0.2125190f, -0.6429273f, 0.5848283f, 0.6735901f, -0.4798771f, 0.0794271f, -0.1308167f, -0.3045062f, -0.1594611f, -0.0006225f, -0.4541470f, -0.5937418f, 0.0265802f, -0.0354839f, -0.1939328f, 0.2533330f, 0.8953870f, -0.1908138f },  // atras
  { -0.0950256f, -0.0568714f, 0.0093679f, -0.0229489f, -0.0456609f, 0.1614945f, 0.7710564f, -0.3622768f, -0.1230935f, -0.0013735f, 0.0576543f, -0.0223049f, -0.1446461f, 0.1490157f, 0.1487390f, -0.2179330f, -0.3088098f, -0.4405518f, 0.9378321f, 0.2094573f, -0.4338353f, 0.2221706f, -0.3143212f, -0.0159137f, -0.2920100f, -0.4411130f, -0.0429456f, -0.4574821f, 0.0785706f, -0.3108347f, 0.1528588f, -0.6109443f, 0.1508451f, 0.2600097f, -0.4910574f, 0.0772933f, 0.4290698f, 0.0613399f, -0.0098452f, -0.0135560f, -0.3880440f, -0.0697388f, -0.5676842f, -0.2945101f, 0.3357432f, 0.2434814f, 0.0373460f, -0.4339030f, -0.0805277f, 0.4520049f, -0.1329937f, -0.4054616f },  // derecha
  { -0.1869598f, 0.0885114f, 0.1052513f, 0.2840296f, -0.2001419f, 0.0064091f, -0.8604839f, -0.3330364f, 0.0670190f, 0.0983975f, -0.5719991f, -0.1845376f, 0.2721577f, -0.2171114f, 0.3464563f, -0.2756920f, 0.0153328f, -0.3376750f, 0.3154013f, 0.0779013f, -0.0857307f, 0.3354299f, -0.1979102f, 0.8843587f, -0.3419156f, 0.4363614f, -0.2271576f, 0.1585338f, 0.0871243f, 0.3108418f, -0.0939633f, -0.2093618f, 0.1017794f, -0.0149095f, 0.1454157f, 0.0490788f, -1.0462791f, -0.0070264f, 0.1072397f, -0.2450591f, 0.3980579f, -0.0058693f, 0.6770898f, -0.3557750f, 0.2670615f, 0.0821784f, -0.2511720f, 0.3820609f, -0.0104349f, -0.3770433f, -0.1501149f, 0.3914506f },  // izquierda
};

float ventana[WINDOW_SIZE][N_BASE];
int idx_ventana = 0, llenas = 0, paso = 0;

// 13 features por muestra (igual que features_base del notebook)
// Sin magnetómetro: el modelo no depende de hacia dónde mire el usuario
void features_base(const float a[3], const float w[3], const float m[3],
                   const float ref[3], float* f) {
  float ax = a[0], ay = a[1], az = a[2], wx = w[0], wy = w[1], wz = w[2];
  f[0] = ax;  f[1] = ay;  f[2] = az;
  f[3] = wx;  f[4] = wy;  f[5] = wz;
  f[6] = sqrtf(ax*ax + ay*ay + az*az);
  f[7] = sqrtf(wx*wx + wy*wy + wz*wz);
  f[8] = atan2f(ay, sqrtf(ax*ax + az*az));
  f[9] = atan2f(az, sqrtf(ax*ax + ay*ay));
  f[10] = ax*ay;  f[11] = ax*az;  f[12] = ay*az;
}

// Media, desviación, mínimo y máximo de la ventana -> 52 features
void estadisticas(float* x) {
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

// z_k = theta_k0 + sum(theta_kj * x~_j), P_k = softmax(z)
int predecir(const float* x, float* prob) {
  float z[NUM_CLASSES], zmax = -1e30f;
  for (int k = 0; k < NUM_CLASSES; k++) {
    z[k] = THETA_B[k];
    for (int j = 0; j < NUM_FEATURES; j++)
      z[k] += THETA_W[k][j] * (x[j] - SCALER_MEAN[j]) / SCALER_SCALE[j];
    if (z[k] > zmax) zmax = z[k];
  }
  float suma = 0;
  int mejor = 0;
  for (int k = 0; k < NUM_CLASSES; k++) { prob[k] = expf(z[k] - zmax); suma += prob[k]; }
  for (int k = 0; k < NUM_CLASSES; k++) {
    prob[k] /= suma;
    if (prob[k] > prob[mejor]) mejor = k;
  }
  return mejor;
}

// Agrega una muestra; devuelve la clase o -1 si aún no toca
int nueva_muestra(const float a[3], const float w[3], const float m[3],
                  const float ref[3], float* prob) {
  features_base(a, w, m, ref, ventana[idx_ventana]);
  idx_ventana = (idx_ventana + 1) % WINDOW_SIZE;
  if (llenas < WINDOW_SIZE) llenas++;
  paso++;
  if (llenas < WINDOW_SIZE || paso < STEP_SIZE) return -1;
  paso = 0;
  float x[NUM_FEATURES];
  estadisticas(x);
  return avanzado ? predecir_mlp(x, prob) : predecir(x, prob);
}

// ===================== MODELO 3: VELOCIDAD (MLP, modo avanzado) =====================
const float MV_MEAN[3] = { 33.0035241f, 4.9183367f, -1.2166216f };
const float MV_SCALE[3] = { 25.9972127f, 11.8104556f, 43.3867043f };
const float MV_W1[3][8] = {
  { -0.2782979f, 0.5123194f, 1.1202710f, -0.1665422f, -0.2856760f, 0.4578103f, 1.0957142f, 0.7193523f },
  { -0.4004219f, -1.0177004f, -0.0186806f, 0.0386293f, -1.2906139f, 0.0780947f, -0.5815039f, -0.3002945f },
  { -0.2288336f, -0.4481216f, -0.0052602f, 0.9904262f, -0.0013556f, 0.4801722f, -1.2721490f, -0.0128005f }
};
const float MV_B1[8] = { 0.2143528f, 0.0300331f, 1.0588534f, 0.2090460f, -0.1482803f, 0.2062779f, 0.8332457f, 0.3083929f };
const float MV_W2[8][8] = {
  { 0.4676847f, -0.1954897f, -0.2961956f, -0.4628561f, -0.0970948f, 0.1720092f, -0.4752605f, -0.2332240f },
  { -0.0513270f, 0.1007299f, 0.0225894f, 0.1397097f, -0.2885314f, -0.0387251f, 0.2579943f, 0.7255877f },
  { -0.5252213f, 1.1269696f, 0.6433520f, 0.3611448f, -0.0289191f, -0.2161609f, 0.6588279f, 1.4802867f },
  { 0.6114478f, 0.4350555f, 0.2178825f, -0.4950946f, 0.0841838f, 0.2995493f, -0.5127576f, -0.3559045f },
  { 0.1096268f, 0.1006044f, -0.2771840f, -0.1764933f, -0.1669126f, -0.3549309f, 0.6684977f, -0.2937242f },
  { -0.0017954f, 0.1559364f, 0.6112781f, 0.5597628f, 0.1824391f, -0.8318276f, 0.0574835f, 0.2442411f },
  { 0.3679540f, 0.1101779f, 0.6703573f, -0.3814459f, -0.3849651f, 0.2533169f, 0.0690110f, 1.5198137f },
  { -0.0666747f, -0.0212879f, -0.1533696f, -0.2328979f, -0.5424233f, 0.1628048f, 0.2147004f, 0.6277849f }
};
const float MV_B2[8] = { 0.3540387f, 0.0793609f, 0.2436730f, -0.3568107f, 0.1465969f, 0.1178647f, -0.0208628f, 0.6161859f };
const float MV_W3[8][1] = {
  { -0.8219573f },
  { 1.0020963f },
  { 0.6399582f },
  { 1.0643839f },
  { -0.3192111f },
  { -0.4532523f },
  { 0.6021494f },
  { 1.6656770f }
};
const float MV_B3[1] = { -0.2839865f };

// x~ = (x - mu)/sigma, h1 = tanh(x~ W1 + b1), h2 = tanh(h1 W2 + b2), v = sigmoide(h2 W3 + b3)
float velocidad_mlp(const float a[3], const float w[3]) {
  float x[3] = { fabsf(atan2f(a[2], -a[0]) * 180.0f / PI), w[0], w[1] };   // angulo_abs, w_x, w_y
  for (int i = 0; i < 3; i++) x[i] = (x[i] - MV_MEAN[i]) / MV_SCALE[i];
  float h1[8], h2[8];
  for (int j = 0; j < 8; j++) {
    float z = MV_B1[j];
    for (int i = 0; i < 3; i++) z += x[i] * MV_W1[i][j];
    h1[j] = tanhf(z);
  }
  for (int j = 0; j < 8; j++) {
    float z = MV_B2[j];
    for (int i = 0; i < 8; i++) z += h1[i] * MV_W2[i][j];
    h2[j] = tanhf(z);
  }
  float z = MV_B3[0];
  for (int i = 0; i < 8; i++) z += h2[i] * MV_W3[i][0];
  return 1.0f / (1.0f + expf(-z));
}

// ===================== MODELO 4: DIRECCIÓN (MLP, modo avanzado) =====================
const int MD_OCULTAS = 8;
const float MD_MEAN[52] = { -8.8292568f, 0.0626529f, 0.8564949f, 4.6620553f, 2.6033190f, -6.2295737f, 9.7346594f, 52.3517115f, 0.0078081f, 0.0890936f, 0.3102645f, -7.5642447f, 1.7420291f, 0.2974682f, 0.4509511f, 0.3809389f, 6.8444259f, 13.7248831f, 15.3927960f, 0.2697148f, 20.2103969f, 0.0460244f, 0.0394091f, 3.8236058f, 3.1337992f, 1.1239990f, -9.3112849f, -0.7190907f, 0.2375195f, -6.6979597f, -18.8269732f, -30.4871117f, 9.3016196f, 25.0114998f, -0.0708589f, 0.0247238f, -6.2522891f, -12.8614099f, -0.0166599f, -8.3162729f, 0.8369918f, 1.5022828f, 15.8660420f, 24.9584913f, 19.0293022f, 10.2285696f, 88.5354016f, 0.0870023f, 0.1561782f, 7.0160437f, -2.4097057f, 3.9056475f };
const float MD_SCALE[52] = { 1.7790979f, 2.2935256f, 2.6287183f, 11.7301602f, 46.9164089f, 45.4724595f, 0.2619869f, 45.2128633f, 0.2739465f, 0.3140506f, 14.4107584f, 16.6207200f, 4.8870022f, 0.3850416f, 0.4665839f, 0.4442042f, 5.8677349f, 13.8783152f, 16.7303810f, 0.3523501f, 16.1104241f, 0.0484423f, 0.0437537f, 3.5423389f, 3.3424092f, 1.5798880f, 1.5509351f, 2.4061512f, 2.7867034f, 15.4469870f, 52.2146946f, 52.8687379f, 0.5120786f, 32.9930735f, 0.2825122f, 0.3215475f, 16.1196632f, 17.4563790f, 4.0956867f, 2.1156534f, 2.5066935f, 2.7263024f, 15.6085771f, 50.9216579f, 54.3555925f, 0.6957697f, 61.4652602f, 0.2870522f, 0.3237326f, 16.1907419f, 17.8974204f, 7.0923594f };
const float MD_W1[52][8] = {
  { 0.0161417f, 0.0649908f, 0.0560379f, 0.0429556f, 0.0417001f, 0.0811443f, 0.0025310f, 0.0485241f },
  { -0.0233959f, -0.0773044f, 0.0855210f, 0.1238800f, -0.1278820f, -0.0773062f, -0.0817035f, 0.0372883f },
  { 0.0279980f, 0.1530938f, 0.1617671f, -0.1013429f, 0.1642744f, -0.0761225f, -0.0610124f, -0.0185757f },
  { -0.0407142f, 0.0039558f, 0.1331153f, -0.0776136f, 0.0376048f, -0.1551734f, -0.3479325f, -0.0298545f },
  { -0.0705944f, -0.0164339f, -0.0391303f, -0.0622946f, 0.1151467f, 0.0099261f, -0.0879997f, -0.0109715f },
  { -0.0416096f, -0.0402960f, 0.1037506f, -0.0631737f, 0.0313301f, -0.0507549f, 0.0753420f, -0.0233723f },
  { 0.1791253f, 0.4412273f, -0.2505398f, 0.2572877f, -0.1872265f, -0.0256256f, 0.0484990f, -0.1313940f },
  { 0.0332781f, -0.0835029f, -0.1629780f, -0.1383195f, -0.1934071f, -0.1452405f, 0.0654768f, 0.0159099f },
  { -0.0144144f, -0.1084123f, 0.0958188f, 0.1195941f, -0.1261286f, -0.0707179f, -0.0819599f, 0.0633479f },
  { 0.0169072f, 0.1558495f, 0.1467124f, -0.0936434f, 0.1494513f, -0.0748280f, -0.0694917f, -0.0088062f },
  { 0.0352201f, 0.0042051f, -0.0796367f, -0.1526468f, 0.1041268f, 0.1373236f, 0.1078085f, 0.1042420f },
  { -0.0774432f, -0.1134975f, -0.2195484f, 0.1464456f, -0.2336823f, 0.0879609f, 0.0282475f, 0.0430426f },
  { 0.0153347f, 0.0214130f, 0.0466201f, 0.2102426f, 0.0378511f, -0.0402612f, -0.1691250f, -0.0405971f },
  { -0.0613323f, 0.2043950f, -0.0483846f, -0.0870518f, 0.0967456f, 0.0444942f, -0.0310790f, -0.0665804f },
  { -0.2276975f, -0.0037805f, 0.0818747f, 0.0604311f, 0.0854088f, -0.0175664f, -0.0559867f, -0.2596664f },
  { 0.3313535f, -0.1329044f, -0.0209640f, 0.0854073f, 0.0335009f, 0.0423363f, 0.0740396f, 0.2526495f },
  { -0.0476558f, -0.1544195f, 0.0328546f, 0.0451190f, -0.1412967f, -0.0695655f, 0.1351202f, 0.0078620f },
  { 0.4242392f, -0.1274832f, -0.1079590f, -0.1251451f, -0.0054802f, -0.0978831f, 0.0469359f, 0.2925505f },
  { -0.2252476f, 0.0908994f, -0.0200496f, -0.0039624f, 0.0466764f, 0.0267551f, 0.1425937f, -0.5178302f },
  { -0.0677429f, 0.1418659f, 0.0792832f, 0.0767264f, 0.1940347f, 0.0822458f, -0.0240255f, -0.0503574f },
  { 0.0120949f, -0.1821955f, -0.1420263f, -0.1863290f, -0.1815904f, -0.2444380f, 0.0893814f, -0.1216063f },
  { -0.2461943f, -0.0042994f, 0.0272323f, 0.0266493f, 0.1131024f, 0.0044344f, -0.0240550f, -0.3084740f },
  { 0.3740898f, -0.0696988f, 0.0060573f, 0.1002361f, 0.0259319f, 0.0447719f, 0.0919105f, 0.2580588f },
  { -0.2428335f, -0.0034766f, 0.1530351f, 0.1003045f, 0.1398604f, -0.0728682f, -0.1015603f, -0.3444792f },
  { 0.3478909f, -0.1787035f, -0.0320725f, 0.1206846f, 0.0156736f, 0.0360933f, 0.0475058f, 0.2757239f },
  { 0.0111979f, 0.0002452f, 0.1497548f, 0.3354617f, 0.1252390f, 0.0436399f, -0.1247972f, 0.0666159f },
  { 0.0369611f, -0.0572560f, 0.0188420f, 0.0774228f, -0.0307203f, 0.1224595f, 0.0509667f, 0.0935266f },
  { 0.0375197f, -0.1631978f, 0.0859825f, 0.1373370f, -0.2043448f, -0.1132619f, -0.1106582f, 0.1327508f },
  { -0.0789087f, 0.2034407f, 0.1482337f, -0.1969558f, 0.1307258f, -0.1186824f, -0.1135689f, -0.0683082f },
  { 0.0290788f, -0.0020230f, 0.1558475f, -0.0095064f, 0.0509413f, -0.1083153f, -0.4078782f, 0.0310674f },
  { -0.2788831f, 0.0650650f, 0.0904351f, -0.1083414f, 0.1822124f, 0.0478882f, -0.1649786f, -0.1268430f },
  { 0.0909364f, -0.1491058f, 0.0903904f, -0.1044395f, -0.0032665f, -0.1098456f, -0.0474033f, 0.2689989f },
  { 0.2085728f, 0.1722815f, -0.0604627f, 0.1105103f, -0.1399957f, -0.1916636f, 0.0012709f, -0.0618251f },
  { 0.1074963f, 0.0956212f, 0.0829953f, 0.0961593f, 0.0632159f, 0.0034269f, 0.0754145f, 0.0164649f },
  { 0.0450998f, -0.1712057f, 0.1225211f, 0.1465437f, -0.1977371f, -0.1175145f, -0.1257964f, 0.1610199f },
  { -0.0956346f, 0.1865087f, 0.1260643f, -0.1902290f, 0.1152155f, -0.1174081f, -0.1319048f, -0.0548086f },
  { 0.1149308f, 0.0985012f, -0.1602689f, -0.2046876f, 0.1494495f, 0.2322426f, 0.1946901f, 0.1920472f },
  { -0.1778873f, -0.1302698f, -0.2260303f, 0.2292768f, -0.2449979f, 0.1610679f, 0.0561700f, -0.0497946f },
  { -0.0597607f, -0.0533721f, -0.0208462f, 0.0782819f, -0.0351261f, -0.0846887f, -0.1957195f, -0.0892312f },
  { 0.0025303f, 0.0858447f, 0.0065820f, 0.0221789f, 0.0298148f, 0.1023820f, 0.0072994f, 0.0373840f },
  { -0.0828976f, -0.1298427f, 0.1430856f, 0.1623906f, -0.1624675f, -0.1451981f, -0.1461897f, -0.0299756f },
  { 0.1018322f, 0.1618665f, 0.1615797f, -0.1658810f, 0.1591883f, -0.1227827f, -0.0722840f, 0.0525121f },
  { -0.0411065f, -0.1646320f, 0.2614427f, 0.0602984f, -0.0920508f, -0.2025704f, -0.2800536f, 0.0003822f },
  { 0.1213545f, -0.0174247f, 0.0522319f, -0.1779999f, 0.2229329f, -0.0562873f, -0.1336035f, 0.1534636f },
  { -0.1196937f, 0.0155564f, 0.0993541f, -0.1336906f, 0.0938115f, -0.1071784f, 0.0627912f, -0.2443743f },
  { 0.0790101f, 0.3727802f, 0.1403346f, 0.1891806f, 0.1969285f, -0.0438939f, -0.0499573f, -0.1064166f },
  { 0.0889613f, 0.0019170f, -0.0817785f, -0.1373052f, -0.0638232f, -0.1821882f, 0.1341147f, -0.0953163f },
  { -0.0746852f, -0.1452865f, 0.1436738f, 0.1511222f, -0.1494988f, -0.1364773f, -0.1437581f, -0.0054568f },
  { 0.0743817f, 0.1715612f, 0.1382113f, -0.1530811f, 0.1275067f, -0.1129605f, -0.0817634f, 0.0459883f },
  { -0.0474463f, 0.1380704f, -0.0469935f, -0.1418221f, 0.2376459f, 0.1569547f, 0.1174385f, -0.0500548f },
  { 0.0510556f, -0.2061198f, -0.2068090f, 0.2827575f, -0.2021549f, 0.1469375f, 0.0918493f, 0.1014471f },
  { 0.0427859f, -0.0152550f, 0.0928651f, 0.2767389f, 0.0668305f, -0.0119517f, -0.1773839f, 0.0084632f }
};
const float MD_B1[8] = { 0.0433367f, -0.3310922f, -0.6558600f, -0.6624830f, -0.5490401f, -0.1073819f, -0.4449404f, -0.1246498f };
const float MD_W2[8][5] = {
  { -0.4893875f, 0.7716399f, 0.8747265f, -0.4917089f, -0.6652827f },
  { -0.3563854f, 0.6642874f, -0.7137727f, 0.8960356f, -0.4899958f },
  { -0.6634526f, 0.7129554f, -0.3039737f, -0.5247603f, 0.7792657f },
  { -0.7937134f, -0.4327387f, 0.7575287f, -0.3240587f, 0.7930479f },
  { -1.0411344f, 0.6189733f, 0.0400455f, 0.3112085f, 0.0709071f },
  { -0.2217105f, -0.6283370f, 0.8167990f, 0.5733416f, -0.5398265f },
  { -0.3870214f, -0.5196122f, 0.8226240f, 0.5791719f, -0.4951445f },
  { 0.5860233f, 0.5794523f, 0.6436451f, -0.8727835f, -0.9363376f }
};
const float MD_B2[5] = { 0.4405338f, -0.2945067f, -0.0871846f, -0.2004588f, 0.1416162f };

// h = tanh(x~ W1 + b1), P = softmax(h W2 + b2)
int predecir_mlp(const float* x, float* prob) {
  float h[MD_OCULTAS];
  for (int j = 0; j < MD_OCULTAS; j++) {
    float z = MD_B1[j];
    for (int i = 0; i < NUM_FEATURES; i++) z += (x[i] - MD_MEAN[i]) / MD_SCALE[i] * MD_W1[i][j];
    h[j] = tanhf(z);
  }
  float z[NUM_CLASSES], zmax = -1e30f;
  for (int k = 0; k < NUM_CLASSES; k++) {
    z[k] = MD_B2[k];
    for (int j = 0; j < MD_OCULTAS; j++) z[k] += h[j] * MD_W2[j][k];
    if (z[k] > zmax) zmax = z[k];
  }
  float suma = 0;
  int mejor = 0;
  for (int k = 0; k < NUM_CLASSES; k++) { prob[k] = expf(z[k] - zmax); suma += prob[k]; }
  for (int k = 0; k < NUM_CLASSES; k++) {
    prob[k] /= suma;
    if (prob[k] > prob[mejor]) mejor = k;
  }
  return mejor;
}

// ===================== MODO =====================
void cambiar_modo() {
  avanzado = !avanzado;
  enviar(avanzado ? "STATUS,MODO AVANZADO" : "STATUS,MODO BASICO");
  Serial.println(avanzado ? "Modo AVANZADO" : "Modo BASICO");
}

// ===================== FILTROS DE SALIDA =====================
const float UMBRAL_PROB = 0.60f;   // menos confianza -> N
const int   N_VOTOS = 3;           // mayoría de las últimas 3 predicciones
const float VEL_GIRO = 0.60f;      // velocidad fija para izquierda/derecha
const float ALFA_VEL = 0.20f;      // suavizado de la velocidad

int votos[N_VOTOS] = {0, 0, 0};
int i_voto = 0;
int dir_actual = 0;                // 0 = N
float vel_suave = 0;

int votar(int k) {
  votos[i_voto] = k;
  i_voto = (i_voto + 1) % N_VOTOS;
  for (int c = 0; c < NUM_CLASSES; c++) {
    int n = 0;
    for (int i = 0; i < N_VOTOS; i++) if (votos[i] == c) n++;
    if (n >= 2) return c;
  }
  return dir_actual;               // sin mayoría: se mantiene
}

// ===================== REFERENCIA DEL MAGNETÓMETRO =====================
// 1 s quieto al encender; se actualiza sola tras 2 s quieto en N
const int N_REF = 100;
float mref[3] = {0, 0, 0}, mref_acc[3] = {0, 0, 0};
int ref_count = 0;
bool ref_lista = false;
int quieto = 0;
const float W_QUIETO = 15.0f;      // °/s
const int N_QUIETO = 200;          // 2 s

void iniciar_referencia() {
  ref_count = 0; ref_lista = false; llenas = 0;
  mref_acc[0] = mref_acc[1] = mref_acc[2] = 0;
  enviar("STATUS,CALIBRANDO");
}

// ===================== PROGRAMA =====================
void setup() {
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  WiFi.setSleep(false);
  delay(100);
  udp.begin(udpPort);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setTimeOut(100);
  bool ok_imu = imu.init(), ok_mag = mag.init();
  if (ok_imu) imu.enableDefault();   // ±2 g, 245 °/s
  if (ok_mag) mag.enableDefault();
  sensores_ok = ok_imu && ok_mag;
  Serial.println(sensores_ok ? "Sensores OK" : "ERROR sensores");
  iniciar_referencia();
}

void loop() {
  // Mensajes del PC: HOLA (registro) o ZERO (recalibrar)
  if (udp.parsePacket()) {
    char buf[8] = {0};
    udp.read(buf, sizeof(buf) - 1);
    pcIP = udp.remoteIP();
    pcPort = udp.remotePort();
    t_pc = millis();
    if (strncmp(buf, "ZERO", 4) == 0) iniciar_referencia();
    if (strncmp(buf, "MODO", 4) == 0) cambiar_modo();
  }
  if (Serial.available() && Serial.read() == 'm') cambiar_modo();   // también desde el monitor serie
  if (!sensores_ok) { enviar("CMD,N,0.00"); delay(500); return; }

  unsigned long t = millis();
  if (t - t_ultima < T_MUESTRA) return;
  t_ultima = t;

  imu.read();
  mag.read();
  // Unidades + intercambio Y<->Z (igual que receptorDB2.py)
  float a[3] = { imu.a.x * K_ACC, imu.a.z * K_ACC, imu.a.y * K_ACC };
  float w[3] = { imu.g.x * K_GYR, imu.g.z * K_GYR, imu.g.y * K_GYR };
  float m[3] = { mag.m.x * K_MAG, mag.m.z * K_MAG, mag.m.y * K_MAG };
  if (ROTAR_180) {   // giro de 180° sobre x: y,z cambian de signo
    a[1] = -a[1]; a[2] = -a[2];
    w[1] = -w[1]; w[2] = -w[2];
    m[1] = -m[1]; m[2] = -m[2];
  }

  if (!ref_lista) {
    for (int i = 0; i < 3; i++) mref_acc[i] += m[i];
    if (++ref_count >= N_REF) {
      for (int i = 0; i < 3; i++) mref[i] = mref_acc[i] / N_REF;
      ref_lista = true;
      enviar("STATUS,LISTO");
    }
    return;
  }

  // Velocidad en cada muestra (polinómica o MLP según el modo)
  float v = avanzado ? velocidad_mlp(a, w) : velocidad_poly(a, w);
  vel_suave += ALFA_VEL * (v - vel_suave);

  // Recalibración automática en reposo
  float wmag = sqrtf(w[0]*w[0] + w[1]*w[1] + w[2]*w[2]);
  quieto = (dir_actual == 0 && wmag < W_QUIETO) ? quieto + 1 : 0;
  if (quieto >= N_QUIETO) {
    for (int i = 0; i < 3; i++) mref[i] = 0.98f * mref[i] + 0.02f * m[i];
  }

  // Dirección cada 5 muestras (20 Hz) (softmax o MLP según el modo)
  float prob[NUM_CLASSES];
  int k = nueva_muestra(a, w, m, mref, prob);
  if (k < 0) return;
  if (prob[k] < UMBRAL_PROB) k = 0;
  if (INTERCAMBIAR_ADEL_ATRAS && (k == 1 || k == 2)) k = 3 - k;   // 1 adelante <-> 2 atras
  dir_actual = votar(k);

  // Velocidad final según la dirección
  float vel_cmd = 0;
  if (dir_actual == 1 || dir_actual == 2) vel_cmd = vel_suave;   // adelante / atrás
  else if (dir_actual == 3 || dir_actual == 4) vel_cmd = VEL_GIRO;

  // Diagnóstico: lecturas del sensor (las graba la plataforma con G)
  char raw[96];
  snprintf(raw, sizeof(raw), "RAW,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f,%s,%.2f",
           a[0], a[1], a[2], w[0], w[1], w[2], CLASS_NAMES[k], prob[k]);
  enviar(raw);

  char msg[64];
  snprintf(msg, sizeof(msg), "CMD,%s,%.2f,%s", CLASS_NAMES[dir_actual], vel_cmd, avanzado ? "AVANZADO" : "BASICO");
  enviar(msg);
  Serial.println(msg);
}
