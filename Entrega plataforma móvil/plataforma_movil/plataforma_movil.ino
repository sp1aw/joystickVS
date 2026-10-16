// (canal A = lado izquierdo, canal B = lado derecho)
#include <WiFi.h>
#include <WiFiUdp.h>

// ===================== RED =====================
const char* ssid = "Guante_IMU";       
const char* password = "12345678";
const unsigned int udpPort = 5000;
WiFiUDP udp;

// ===================== PINES TB6612 =====================
const int PIN_STBY = 33;
const int PIN_PWMA = 25;
const int PIN_AIN2 = 26;
const int PIN_AIN1 = 27;
const int PIN_PWMB = 14;
const int PIN_BIN1 = 12;   // GPIO12 es pin de arranque: el TB6612 lo deja en bajo
const int PIN_BIN2 = 13;

// Si un lado gira al revés, cambiar a true
const bool INVERTIR_A = false;
const bool INVERTIR_B = false;

// Si gira al lado contrario del comando (derecha <-> izquierda), cambiar a true
const bool INTERCAMBIAR_GIROS = true;

// Compensación si el carrito se desvía al ir derecho (1.0 = sin cambio)
// Ej.: si se va hacia la derecha, bajar GANANCIA_A a 0.85
const float GANANCIA_A = 1.0f;   // lado izquierdo
const float GANANCIA_B = 1.0f;   // lado derecho

// ===================== PWM =====================
const int PWM_FREQ = 20000;    // 20 kHz, sin ruido audible
const int PWM_RES = 8;         // 0..255
const int PWM_MIN = 90;        // por debajo de esto los motores no arrancan (ajustar)
const int PWM_MAX = 255;
const int RAMPA = 15;          // cambio máximo de PWM por ciclo (evita tirones)

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void pwmIniciar(int pin, int canal) { ledcAttach(pin, PWM_FREQ, PWM_RES); }
void pwmEscribir(int pin, int canal, int duty) { ledcWrite(pin, duty); }
#else
void pwmIniciar(int pin, int canal) { ledcSetup(canal, PWM_FREQ, PWM_RES); ledcAttachPin(pin, canal); }
void pwmEscribir(int pin, int canal, int duty) { ledcWrite(canal, duty); }
#endif

// ===================== SEGURIDAD =====================
const unsigned long TIMEOUT_MS = 300;   // sin comandos -> se detiene
unsigned long t_ultimo_cmd = 0;

// ===================== ESTADO =====================
int obj_a = 0, obj_b = 0;      // objetivo -255..255
int act_a = 0, act_b = 0;      // valor aplicado (con rampa)
String dir_rx = "N";
float vel_rx = 0;

// Aplica velocidad con signo a un canal (-255..255)
void motor(int in1, int in2, int pwm, int canal, int valor, bool invertir) {
  if (invertir) valor = -valor;
  valor = (int)(valor * (canal == 0 ? GANANCIA_A : GANANCIA_B));
  if (valor > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (valor < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else                { digitalWrite(in1, LOW);  digitalWrite(in2, LOW); }   // libre
  pwmEscribir(pwm, canal, abs(valor));
}

// Velocidad 0..1 -> PWM útil
int aPWM(float v) {
  if (v < 0.05f) return 0;
  if (v > 1) v = 1;
  return PWM_MIN + (int)(v * (PWM_MAX - PWM_MIN));
}

// Dirección + velocidad -> objetivo de cada lado
void fijarObjetivo(const String& d, float v) {
  int p = aPWM(v);
  int g = INTERCAMBIAR_GIROS ? -p : p;
  if (d == "adelante")      { obj_a =  p; obj_b =  p; }
  else if (d == "atras")     { obj_a = -p; obj_b = -p; }
  else if (d == "izquierda") { obj_a = -g; obj_b =  g; }   // gira sobre su eje
  else if (d == "derecha")   { obj_a =  g; obj_b = -g; }
  else                       { obj_a =  0; obj_b =  0; }   // N
}

int acercar(int actual, int objetivo) {
  if (objetivo > actual) return min(actual + RAMPA, objetivo);
  if (objetivo < actual) return max(actual - RAMPA, objetivo);
  return actual;
}

// Lee "CMD,adelante,0.73"
void leerUDP() {
  int n = udp.parsePacket();
  if (!n) return;
  char buf[64] = {0};
  udp.read(buf, sizeof(buf) - 1);
  if (strncmp(buf, "CMD,", 4) != 0) return;   // ignora STATUS y otros
  char* d = strtok(buf + 4, ",");
  char* v = strtok(NULL, ",");
  if (!d || !v) return;
  dir_rx = String(d);
  vel_rx = atof(v);
  fijarObjetivo(dir_rx, vel_rx);
  t_ultimo_cmd = millis();
}

// Prueba sin guante por monitor serie: w s a d (espacio = parar)
void leerSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'w') fijarObjetivo("adelante", 0.6f);
  else if (c == 's') fijarObjetivo("atras", 0.6f);
  else if (c == 'a') fijarObjetivo("izquierda", 0.6f);
  else if (c == 'd') fijarObjetivo("derecha", 0.6f);
  else if (c == ' ') fijarObjetivo("N", 0);
  else return;
  t_ultimo_cmd = millis() + 1700;   // la prueba dura 2 s
  Serial.printf("Prueba: %c\n", c);
}

// Conexión sin bloquear: el loop sigue atendiendo el monitor serie
bool udp_iniciado = false;

void iniciarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  Serial.println("Buscando la red Guante_IMU (encienda el guante)...");
}

void revisarWiFi() {
  bool ok = WiFi.status() == WL_CONNECTED;
  if (ok && !udp_iniciado) {
    udp.begin(udpPort);
    udp_iniciado = true;
    Serial.print("Conectado a Guante_IMU, IP: ");
    Serial.println(WiFi.localIP());
  } else if (!ok && udp_iniciado) {
    udp.stop();
    udp_iniciado = false;
    Serial.println("Conexión perdida, reintentando...");
  }
}

void setup() {
  Serial.begin(115200);
  int pines[] = {PIN_AIN1, PIN_AIN2, PIN_BIN1, PIN_BIN2, PIN_STBY};
  for (int p : pines) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
  pwmIniciar(PIN_PWMA, 0);
  pwmIniciar(PIN_PWMB, 1);
  digitalWrite(PIN_STBY, HIGH);   // habilita el driver
  iniciarWiFi();
}

void loop() {
  revisarWiFi();
  if (udp_iniciado) leerUDP();
  leerSerial();

  // Seguridad: sin comandos recientes -> detener
  if ((long)(millis() - t_ultimo_cmd) > (long)TIMEOUT_MS) fijarObjetivo("N", 0);

  // Rampa cada 10 ms
  static unsigned long t_ctrl = 0;
  if (millis() - t_ctrl >= 10) {
    t_ctrl = millis();
    act_a = acercar(act_a, obj_a);
    act_b = acercar(act_b, obj_b);
    motor(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0, act_a, INVERTIR_A);
    motor(PIN_BIN1, PIN_BIN2, PIN_PWMB, 1, act_b, INVERTIR_B);
  }

  // Estado por serie cada 0.5 s
  static unsigned long t_log = 0;
  if (millis() - t_log > 500) {
    t_log = millis();
    Serial.printf("dir=%s vel=%.2f | A=%d B=%d\n", dir_rx.c_str(), vel_rx, act_a, act_b);
  }
}
