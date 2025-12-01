#include <Servo.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//********DELAY PARA MOSTRAR ESTADO DEL WIFI*********
unsigned long interval = 30000;
unsigned long previousMillis = 0;

WiFiClient wlanclient;
PubSubClient mqttClient(wlanclient);

//***************CONFIGURACIÓN DE RED****************
const char *ssid = "Dark Knigth"; // <--------------- CAMBIAR
const char *passwd = "TheFlash2022"; //<--------------- CAMBIAR

//***************CONFIGURACIÓN DE MQTT***************
char *server = "192.168.0.5"; // <--------------- CAMBIAR
int port = 1884;

const char *mqtt_user = "Artefactos";
const char *mqtt_password = "Repito1234";

const char* topics[] = {
  "/lata/comando",
  "/lata/calibrar"
};

// Topics
const char* TOPIC_COLOR = "/smartbin/color";
const char* TOPIC_PROXIMITY = "/smartbin/proximity";
const char* TOPIC_LEVEL = "/smartbin/level";

const int PIN_PROXIMIDAD = D1;
const int PIN_SERVO = D2;
const int PIN_TRIG = D3;
const int PIN_ECHO = D4;

#define S0 D5
#define S1 D6
#define S2 D7
#define S3 D0
#define OUT D8

Servo miServo;
const int ANGULO_DETECTADO = 0;
const int ANGULO_REPOSO = 180;

const char* CLIENT_MQTT_ID = "ESP8266_LATA_01";  // <----------- ID único para MQTT (client_id_mqtt) NO OLVIDAR
const unsigned long TIMEOUT_US = 25000;
const int READS_AVG = 3;
const float LIMITE_CM = 18.0;
const float MIN_DISTANCIA_CM = 2.0;

int ROJO_MIN_GRIS = 60;      
int ROJO_MAX_GRIS = 120;     
int VERDE_MIN_GRIS = 60;     
int VERDE_MAX_GRIS = 120;    
int AZUL_MIN_GRIS = 60;      
int AZUL_MAX_GRIS = 120;     
int DIFERENCIA_MAX_GRIS = 30;

bool objetoDetectado = false;
bool esColorGris = false;
bool estadoAnterior = false;
int estadoProximidad = HIGH;
bool servoMovido = false;

int redFreq = 0;
int greenFreq = 0;
int blueFreq = 0;

unsigned long lastPublish = 0;
const unsigned long PUBLISH_INTERVAL = 2000; 
String lastColorPublished = "";
float lastDistancePublished = -1;

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
      delay(5000);
      Serial.println("Sistema reseteado remotamente");
    }
  }
}

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
  
  pinMode(PIN_PROXIMIDAD, INPUT_PULLUP);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  miServo.attach(PIN_SERVO);
  miServo.write(ANGULO_REPOSO);
  
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

  mqttClient.setServer(server, port);
  mqttClient.setCallback(mqttCallback);

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

void publishColorData(String classification, int confidence) {
  String json = "{\"client_id_mqtt\":\"" + String(CLIENT_MQTT_ID) + "\",";
  json += "\"classification\":\"" + classification + "\",";
  json += "\"confidence\":" + String(confidence) + ",";
  json += "\"rgb\":[" + String(redFreq) + "," + String(greenFreq) + "," + String(blueFreq) + "]}";
  
  char jsonChar[250];
  json.toCharArray(jsonChar, 250);
  publishToTopic((char*)TOPIC_COLOR, jsonChar);
  
  Serial.print("📤 MQTT Color: ");
  Serial.println(json);
}

void publishProximityData(float distance, bool trigger) {
  char* distString = floatToChar(distance, 2);
  String json = "{\"client_id_mqtt\":\"" + String(CLIENT_MQTT_ID) + "\",";
  json += "\"distance_cm\":" + String(distString) + ",";
  json += "\"trigger\":" + String(trigger ? "true" : "false") + "}";
  free(distString);
  
  char jsonChar[150];
  json.toCharArray(jsonChar, 150);
  publishToTopic((char*)TOPIC_PROXIMITY, jsonChar);
  
  Serial.print("📤 MQTT Proximidad: ");
  Serial.println(json);
}

void publishLevelData(float levelPercent) {
  char* levelString = floatToChar(levelPercent, 1);
  String json = "{\"client_id_mqtt\":\"" + String(CLIENT_MQTT_ID) + "\",";
  json += "\"level_percent\":" + String(levelString) + "}";
  free(levelString);
  
  char jsonChar[150];
  json.toCharArray(jsonChar, 150);
  publishToTopic((char*)TOPIC_LEVEL, jsonChar);
  
  Serial.print("📤 MQTT Nivel: ");
  Serial.println(json);
}

void loop() {
  printWifiStatus();
  if (!mqttClient.connected()) {
    reconnect(); 
  }
  mqttClient.loop();

  estadoProximidad = digitalRead(PIN_PROXIMIDAD);
  objetoDetectado = (estadoProximidad == LOW);
  
  leerColorRGB();
  esColorGris = detectarGrisMetalico(redFreq, greenFreq, blueFreq);
  
  bool activarServo = (objetoDetectado && esColorGris);
  
  if (activarServo && !servoMovido) {
    miServo.write(ANGULO_DETECTADO);
    servoMovido = true;
    Serial.println();
    Serial.println("🔘 LATA DETECTADA - SERVO ACTIVADO");
    Serial.println("   Color: GRIS METÁLICO           ");
    Serial.println("   Servo: 0° (ABIERTO/PERPENDICULAR)");
    Serial.println("   Manteniéndose abierto por 10 segundos...");
    Serial.println();
    
    publishColorData("GRIS_METALICO", 95);
    
    for (int i = 0; i < 100; i++) {
      mqttClient.loop();
      delay(100);
    }
    
    Serial.println("✅ Tiempo de apertura completado. Esperando retiro de objeto...");
    return;
  }
  
  if (!activarServo && servoMovido) {
    miServo.write(ANGULO_REPOSO);
    delay(5000);
    servoMovido = false;
    Serial.println("❌ Sin lata detectada - Servo vuelve a 180° (CERRADO/PARALELO - LISTO PARA NUEVO CICLO)");
    
    publishColorData("NINGUNO", 0);
    
    delay(15);
  }
  
  float distancia = medirPromedioCm(READS_AVG);
  
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
    Serial.print("⚪ No se detecto un color de aluminio");
  }
  
  Serial.print("\t│ 🔧 Servo: ");
  if (servoMovido) {
    Serial.print("0° ⚠️ ABIERTO");
  } else {
    Serial.print("180° ✓ CERRADO");
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
    
    unsigned long currentMillis = millis();
    if (currentMillis - lastPublish >= PUBLISH_INTERVAL) {
      
      publishProximityData(distancia, objetoDetectado);
      
      publishLevelData(porcentaje);
      
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