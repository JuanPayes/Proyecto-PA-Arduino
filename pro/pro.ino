#include <Servo.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//********DELAY PARA MOSTRAR ESTADO DEL WIFI*********
unsigned long interval = 30000;
unsigned long previousMillis = 0;

WiFiClient wlanclient;
PubSubClient mqttClient(wlanclient);

//***************CONFIGURACIÓN DE RED****************
const char *ssid = "Dark knight";
const char *passwd = "TheFlash2022";

//***************CONFIGURACIÓN DE MQTT***************
char *server = "192.168.0.7";
int port = 1883;

const char *mqtt_user = "Artefactos";
const char *mqtt_password = "Repito1234";

// Definir todos los "topics" a los cuales se desea suscribir
const char* topics[] = {
  "/lata/comando",        // Para recibir comandos remotos
  "/lata/calibrar"        // Para calibrar remotamente
};

// Topics para publicar (siguiendo estructura de tu API)
const char* TOPIC_COLOR = "/smartbin/color";
const char* TOPIC_PROXIMITY = "/smartbin/proximity";
const char* TOPIC_LEVEL = "/smartbin/level";

// ===== CONFIGURACIÓN DE PINES =====
const int PIN_PROXIMIDAD = D1;
const int PIN_SERVO = D2;
const int PIN_TRIG = D3;
const int PIN_ECHO = D4;

#define S0 D5
#define S1 D6
#define S2 D7
#define S3 D0
#define OUT D8

// ===== CONFIGURACIÓN SERVO =====
Servo miServo;
const int ANGULO_DETECTADO = 180;  
const int ANGULO_REPOSO = 0;

// ===== CONFIGURACIÓN HC-SR04 =====
const unsigned long TIMEOUT_US = 25000; // 25ms timeout
const int READS_AVG = 3; // lecturas para promediar
const float LIMITE_CM = 18.0; // 18 cm = límite
const float MIN_DISTANCIA_CM = 2.0; // distancia mínima confiable

// ===== UMBRALES PARA DETECTAR GRIS METÁLICO (LATA) =====
int ROJO_MIN_GRIS = 60;      
int ROJO_MAX_GRIS = 120;     
int VERDE_MIN_GRIS = 60;     
int VERDE_MAX_GRIS = 120;    
int AZUL_MIN_GRIS = 60;      
int AZUL_MAX_GRIS = 120;     
int DIFERENCIA_MAX_GRIS = 30;

// ===== VARIABLES DE ESTADO =====
bool objetoDetectado = false;
bool esColorGris = false;
bool estadoAnterior = false;
int estadoProximidad = HIGH;
bool servoMovido = false;

// Variables RGB
int redFreq = 0;
int greenFreq = 0;
int blueFreq = 0;

// Variables para control de publicación MQTT
unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 2000; // Publicar cada 2 segundos
String lastColorPublished = "";
float lastDistancePublished = -1;

//***************CALLBACK DE RESOLUCIÓN A SUBSCRIBES***************
void mqttCallback(char *topicChar, byte *payload, unsigned int length) {
  Serial.println();
  String topic = String(topicChar);
  Serial.print("Message arrived on Topic: ");
  Serial.println(topic);

  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  if (topic == "/lata/comando") {
    Serial.print("Comando recibido: ");
    Serial.println(message);
    if (message == "reset") {
      servoMovido = false;
      miServo.write(ANGULO_REPOSO);
      Serial.println("Sistema reseteado remotamente");
    }
  }
}

//***************FUNCIONES PARA PUBLICAR Y SUSCRIBIR***************
boolean publishToTopic(char *topic, char *message) {
  return mqttClient.publish(topic, message);
}

void subscribeToTopics() {
  const int numTopics = sizeof(topics) / sizeof(topics[0]);
  for (int i = 0; i < numTopics; i++) {
    mqttClient.subscribe(topics[i]);
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);
  
  // Configurar pines sensores
  pinMode(PIN_PROXIMIDAD, INPUT_PULLUP);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  
  // Configurar pines TCS3200
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  
  // Configurar escala de frecuencia a 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  // Inicializar servo
  miServo.attach(PIN_SERVO);
  miServo.write(ANGULO_REPOSO);
  
  //*********Configurar conexión wifi*************
  WiFi.begin(ssid, passwd);
  Serial.println();
  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  Serial.print("Conectado a WiFi. IP: ");
  Serial.println(WiFi.localIP());

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  //*********Configurar conexión MQTT*************
  mqttClient.setServer(server, port);
  mqttClient.setCallback(mqttCallback);

  // Realizar conexión al MQTT Broker
  if (mqttClient.connect("ESP-Detector-Latas", mqtt_user, mqtt_password)) {
    Serial.println("Conectado al Broker MQTT");
  } else {
    Serial.println("Fallo conexión MQTT");
    Serial.println(mqttClient.state());
  }

  subscribeToTopics();
  
  Serial.println();
  Serial.println("============================================");
  Serial.println("Sistema Multi-Sensor + MQTT");
  Serial.println("============================================");
  Serial.println("Proximidad + Color + Servo + Ultrasónico");
  Serial.println("Servo se activa solo si: OBJETO + COLOR GRIS (LATA)");
  Serial.println();
}

// ===== FUNCIONES TCS3200 =====
void leerColorRGB() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFreq = pulseIn(OUT, LOW);
  delay(50);
  
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFreq = pulseIn(OUT, LOW);
  delay(50);
  
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFreq = pulseIn(OUT, LOW);
  delay(50);
}

bool detectarGrisMetalico(int r, int g, int b) {
  if (r < ROJO_MIN_GRIS || r > ROJO_MAX_GRIS) return false;
  if (g < VERDE_MIN_GRIS || g > VERDE_MAX_GRIS) return false;
  if (b < AZUL_MIN_GRIS || b > AZUL_MAX_GRIS) return false;
  
  int maxVal = max(r, max(g, b));
  int minVal = min(r, min(g, b));
  int diferencia = maxVal - minVal;
  
  if (diferencia > DIFERENCIA_MAX_GRIS) return false;
  
  return true;
}

// ===== FUNCIONES HC-SR04 =====
float medirDistanciaCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duracion = pulseIn(PIN_ECHO, HIGH, TIMEOUT_US);
  if (duracion == 0) return -1.0;
  
  float distancia = (float)duracion / 58.0;
  return distancia;
}

float medirPromedioCm(int n) {
  float sum = 0;
  int validas = 0;
  
  for (int i = 0; i < n; i++) {
    float d = medirDistanciaCm();
    if (d > 0 && d >= MIN_DISTANCIA_CM) {
      sum += d;
      validas++;
    }
    delay(60);
  }
  
  if (validas == 0) return -1.0;
  return sum / (float)validas;
}

float distanciaAPorcentaje(float distanciaCm) {
  if (distanciaCm < 0) return -1.0;
  if (distanciaCm >= LIMITE_CM) return 0.0;
  if (distanciaCm <= MIN_DISTANCIA_CM) return 100.0;
  
  float porcentaje = ((LIMITE_CM - distanciaCm) / LIMITE_CM) * 100.0;
  if (porcentaje < 0.0) porcentaje = 0.0;
  if (porcentaje > 100.0) porcentaje = 100.0;
  
  return porcentaje;
}

//***************FUNCIÓN DE RECONEXIÓN MQTT***************
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Intentando conectar al broker MQTT...");
    if (mqttClient.connect("ESP-Detector-Latas", mqtt_user, mqtt_password)) {
      Serial.println("Conectado al Broker MQTT");
      subscribeToTopics();
    } else {
      Serial.print("Fallo, rc=");
      Serial.println(mqttClient.state());
      Serial.println("Reintentando en 5 segundos");
      delay(5000);
    }
  }
}

void printWifiStatus() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    Serial.println("");
    Serial.println("----------------------------------------");
    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL:
        Serial.println("SSID no disponible");
        break;
      case WL_CONNECTED:
        Serial.println("WiFi conectado OK");
        break;
      case WL_CONNECT_FAILED:
        Serial.println("WiFi fallo conexión");
        break;
    }
    Serial.printf("Estado conexión: %d\n", WiFi.status());
    Serial.print("RSSI: ");
    Serial.println(WiFi.RSSI());
    Serial.println("----------------------------------------");
    previousMillis = currentMillis;
  }
}

//***************FUNCIONES AUXILIARES DE CONVERSIÓN***************
char* floatToChar(float number, int precision) {
    int length = snprintf(NULL, 0, "%.*f", precision, number);
    char* result = (char*)malloc(length + 1);
    if (result != NULL) {
        snprintf(result, length + 1, "%.*f", precision, number);
    }
    return result;
}

char* intToChar(int number) {
    int length = snprintf(NULL, 0, "%d", number);
    char* result = (char*)malloc(length + 1);
    if (result != NULL) {
        snprintf(result, length + 1, "%d", number);
    }
    return result;
}

// Función para crear JSON de color
// Formato: {"classification":"GRIS_METALICO","confidence":95,"rgb":[85,90,88]}
void publishColorData(String classification, int confidence) {
  String json = "{\"classification\":\"" + classification + "\",";
  json += "\"confidence\":" + String(confidence) + ",";
  json += "\"rgb\":[" + String(redFreq) + "," + String(greenFreq) + "," + String(blueFreq) + "]}";
  
  char jsonChar[200];
  json.toCharArray(jsonChar, 200);
  publishToTopic((char*)TOPIC_COLOR, jsonChar);
  
  Serial.print("📤 MQTT Color: ");
  Serial.println(json);
}

// Función para crear JSON de proximidad
// Formato: {"distance_cm":15.5,"trigger":true}
void publishProximityData(float distance, bool trigger) {
  char* distString = floatToChar(distance, 2);
  String json = "{\"distance_cm\":" + String(distString) + ",";
  json += "\"trigger\":" + String(trigger ? "true" : "false") + "}";
  free(distString);
  
  char jsonChar[100];
  json.toCharArray(jsonChar, 100);
  publishToTopic((char*)TOPIC_PROXIMITY, jsonChar);
  
  Serial.print("📤 MQTT Proximidad: ");
  Serial.println(json);
}

// Función para crear JSON de nivel
// Formato: {"bin_id":"BIN001","level_percent":31.4}
void publishLevelData(float levelPercent) {
  char* levelString = floatToChar(levelPercent, 1);
  String json = "{\"bin_id\":\"BIN001\",";  // ⬅️ CAMBIAR bin_id según tu sistema
  json += "\"level_percent\":" + String(levelString) + "}";
  free(levelString);
  
  char jsonChar[100];
  json.toCharArray(jsonChar, 100);
  publishToTopic((char*)TOPIC_LEVEL, jsonChar);
  
  Serial.print("📤 MQTT Nivel: ");
  Serial.println(json);
}

void loop() {
  // Verificar conexión WiFi y MQTT
  printWifiStatus();
  if (!mqttClient.connected()) {
    reconnect(); 
  }
  mqttClient.loop();

  // ===== 1. LEER SENSOR DE PROXIMIDAD =====
  estadoProximidad = digitalRead(PIN_PROXIMIDAD);
  objetoDetectado = (estadoProximidad == LOW);
  
  // ===== 2. LEER COLOR RGB =====
  leerColorRGB();
  esColorGris = detectarGrisMetalico(redFreq, greenFreq, blueFreq);
  
  // ===== 3. CONTROLAR SERVO =====
  bool activarServo = (objetoDetectado && esColorGris);
  
  if (activarServo && !servoMovido) {
    miServo.write(ANGULO_DETECTADO);
    servoMovido = true;
    Serial.println();
    Serial.println("🔘 LATA DETECTADA - SERVO ACTIVADO");
    Serial.println("   Color: GRIS METÁLICO           ");
    Serial.println("   Servo: 180° (BLOQUEADO)        ");
    Serial.println();
    
    // Publicar color detectado (GRIS_METALICO con 95% confianza)
    publishColorData("GRIS_METALICO", 95);
    
    delay(15);
  }
  
  if (!activarServo && servoMovido) {
    miServo.write(ANGULO_REPOSO);
    servoMovido = false;
    Serial.println("❌ Sin lata detectada - Servo vuelve a 0° (LISTO PARA NUEVO CICLO)");
    
    // Publicar que no hay color detectado
    publishColorData("NINGUNO", 0);
    
    delay(15);
  }
  
  // ===== 4. MEDIR DISTANCIA CON HC-SR04 =====
  float distancia = medirPromedioCm(READS_AVG);
  
  // ===== 5. MOSTRAR INFORMACIÓN Y PUBLICAR A MQTT =====
  Serial.print("👁️ Prox: ");
  Serial.print(objetoDetectado ? "✓ SÍ" : "✗ NO");
  
  Serial.print("\t🎨 RGB: R=");
  Serial.print(redFreq);
  Serial.print(" G=");
  Serial.print(greenFreq);
  Serial.print(" B=");
  Serial.print(blueFreq);
  
  Serial.print("\t│ ");
  if (esColorGris) {
    Serial.print("🔘 GRIS (LATA)");
  } else {
    Serial.print("⚪ Otro color");
  }
  
  Serial.print("\t│ 🔧 Servo: ");
  if (servoMovido) {
    Serial.print("180° ⚠️ BLOQUEADO");
  } else {
    Serial.print("0° ✓ LISTO");
  }
  
  Serial.print("\t│ Estado: ");
  if (objetoDetectado && esColorGris) {
    Serial.print("✓✓ ACTIVADO");
  } else if (objetoDetectado && !esColorGris) {
    Serial.print("✓✗ Objeto NO es lata");
  } else if (!objetoDetectado && esColorGris) {
    Serial.print("✗✓ Sin objeto");
  } else {
    Serial.print("✗✗ Esperando...");
  }
  
  Serial.print("\t│ ");
  
  if (distancia < 0) {
    Serial.println("❌ HC-SR04: Sin señal");
  } else {
    float porcentaje = distanciaAPorcentaje(distancia);
    
    Serial.print("📏 ");
    Serial.print(distancia, 2);
    Serial.print("cm");
    Serial.print("\t📊 ");
    Serial.print(porcentaje, 1);
    Serial.print("%");
    
    Serial.print("\t[");
    int barras = (int)(porcentaje / 10);
    for (int i = 0; i < 10; i++) {
      if (i < barras) Serial.print("█");
      else Serial.print("·");
    }
    Serial.println("]");
    
    // Publicar datos periódicamente (cada PUBLISH_INTERVAL ms)
    unsigned long currentMillis = millis();
    if (currentMillis - lastPublish >= PUBLISH_INTERVAL) {
      
      // 1. Publicar proximidad (distance_cm y trigger boolean)
      publishProximityData(distancia, objetoDetectado);
      
      // 2. Publicar nivel de llenado (bin_id y level_percent)
      publishLevelData(porcentaje);
      
      // 3. Publicar color si cambió
      String currentColor = esColorGris ? "GRIS_METALICO" : "OTRO";
      if (currentColor != lastColorPublished) {
        int confidence = esColorGris ? 95 : 0;
        publishColorData(currentColor, confidence);
        lastColorPublished = currentColor;
      }
      
      lastPublish = currentMillis;
    }
  }
  
  delay(500);
}
