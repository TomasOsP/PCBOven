#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Adafruit_MAX31856.h>

// PCBOven - ESP32 oven controller
// Implements a web-controlled heating routine using a MAX31856 thermocouple,
// an SSR for heating (PWM), a PWM-driven fan, and a simple WebSocket UI.

// Valores controlador
#define MAX_CS 5
#define INTERVAL 600000
#define freq 120
#define resolution 8
#define SSR 13
#define LED_PIN 2  // Ajusta si usas otro pin; el pin 2 suele tener un LED en muchas placas ESP32



// WiFi credentials - change for your network before flashing
const char* ssid = "iPhone TK";
const char* password = "1234567890";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
Adafruit_MAX31856 max31856 = Adafruit_MAX31856(MAX_CS);

// Main control variables
// `desiredTemp` is the target temperature in °C
float desiredTemp = 80.0;
// `maxTime` is the maximum heating time in milliseconds
unsigned long maxTime = 5 * 60000UL; // en milisegundos
unsigned long startTime = 0;
// `functionActive` toggled from the web UI to enable heating routine
bool functionActive = false;
// `timerRunning` indicates the runtime timer is active
bool timerRunning = false;

/////////////////// Controller / timing variables /////////////////////////

// `previousMillis` used for periodic serial/log updates
unsigned long previousMillis = 0;  // Store last time event occurred
// Typical 1 second interval for printing/logging
const long interval = 1000;  // Delay time in milliseconds (1 second)
// Control output written to SSR PWM (0-255)
float Output = 0;
unsigned long currentTime, previousTime;
double elapsedTime;
unsigned int millis_before, millis_before_2;    // timing helpers
unsigned int millis_now = 0;
float pid_refresh_rate  = 50;                   // placeholder PID refresh rate
float seconds = 0;                              // elapsed seconds placeholder
// Latest temperature reading from thermocouple
float temperature = 0.0;
float MIN_VALUE = 0;
float MAX_VALUE = 255;                      // Max PWM value for SSR
unsigned long previousMillis2 = 0; // used for long-interval fan decision


//////////////////////// Funciones Ventilador /////////////////////

void fanSetup() {
  // Configure PWM channel for the fan on pin 27
  // Note: confirm channel/pin mapping for your ESP32 core
  ledcAttach(27,25000, 8);
  // Start with fan off
  ledcWrite(27, 0);
}

void fanStatus(int vel) {
  // `vel` range 0-255; writes PWM duty to fan channel
  ledcWrite(27, vel);
}

/////////////////////// Funciones controlador /////////////////////


void inicializarControlador() {
    Serial.begin(115200);
    fanSetup();
    
    if (!max31856.begin()) {
        Serial.println("Could not initialize MAX31856. Check wiring!");
        while (1);
    }
    ledcAttach(SSR, freq, resolution);

    max31856.setNoiseFilter(MAX31856_NOISE_FILTER_60HZ);

    // Set thermocouple type to J 
    max31856.setThermocoupleType(MAX31856_TCTYPE_J);
    
    Serial.println("MAX31856 Ready!");
    SPISettings settings(500000, MSBFIRST, SPI_MODE1);
    //SPI.beginTransaction(settings);
    
    millis_before = millis();
    millis_now = millis();
}



void ejecutarControlador(float Setpoint) {

    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; // Update the last event time
    Serial.print(max31856.readThermocoupleTemperature());
    Serial.print(" - ");
    Serial.println(Output);
    } 

    temperature = max31856.readThermocoupleTemperature();
    unsigned long currentMillis2 = millis();
    
    if (Setpoint > 210){
      if (currentMillis2 - previousMillis2 >= INTERVAL) {//If para prender el ventilador
        Serial.print(" If Ventilador ");
        Output = 0;
        fanStatus(255);
      }
      else if(temperature > (Setpoint -5.0) && temperature < (Setpoint - 1.0)){
        Output = Setpoint*(78.0/190.0);
        fanStatus(0);
      }
      else if (temperature > (Setpoint - 1.5)){
        Output = 0.0;
        fanStatus(205);
      }
      else if (temperature < (Setpoint -5) && (temperature > Setpoint*(150.0/190.0))){
        Output = Setpoint*(90.0/190.0);
        fanStatus(0);
      }
      else{
        Output = Setpoint*(150.0/190.0);
        fanStatus(0);
      }
      
    }
    else {
      if (currentMillis2 - previousMillis2 >= INTERVAL) {//If para prender el ventilador
        Serial.print(" If Ventilador ");
        Output = 0;
        fanStatus(255);
      }
      else if(temperature > (Setpoint -5.0) && temperature < (Setpoint - 1.0)){
        Output = Setpoint*(69.0/190.0);
        fanStatus(0);
      }
      else if (temperature > (Setpoint - 1.5)){
        Output = 0.0;
        fanStatus(155);
      }
      else if (temperature < (Setpoint -5) && (temperature > Setpoint*(150.0/190.0))){
        Output = Setpoint*(74.0/190.0);
        fanStatus(0);
      }
      else{
        Output = Setpoint*(100.0/190.0);
        fanStatus(0);
      }
    }

      if (Output > MAX_VALUE){Output=MAX_VALUE;}
      else if (Output < MIN_VALUE){Output=MIN_VALUE;}
      else {Output=Output;}

      ledcWrite(SSR,Output);
}

/////////////////////////////////////////////////////////////////

void notifyClients() {
  DynamicJsonDocument doc(256);
  doc["temperature"] = temperature;
  doc["functionActive"] = functionActive;
  doc["remainingTime"] = timerRunning ? maxTime - (millis() - startTime) : maxTime;
  doc["desiredTemp"] = desiredTemp;
  doc["fanActive"] = (temperature > 30.0 && !functionActive);

  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    notifyClients();
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  inicializarControlador();
  // Configurar IP estática (opcional)
  IPAddress local_IP(172, 20, 10, 2);
  IPAddress gateway(172, 20, 10, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(local_IP, gateway, subnet);

  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, HIGH);
    delay(300);
    digitalWrite(LED_PIN, LOW);
    delay(300);
    Serial.println("Connecting to WiFi...");
  }
  digitalWrite(LED_PIN, HIGH);
  Serial.println("\n✅ Conectado a WiFi");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  if (!SPIFFS.begin(true)) {
    Serial.println("Error mounting SPIFFS");
    return;
  }

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

  server.on("/toggle", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      DynamicJsonDocument doc(256);
      DeserializationError error = deserializeJson(doc, data);
      if (!error) {
        functionActive = doc["state"].as<bool>();
        desiredTemp = doc["desiredTemp"].as<float>();
        maxTime = doc["maxTime"].as<unsigned long>() * 60000UL;
        if (functionActive) {
          startTime = millis();
          timerRunning = true;
        } else {
          timerRunning = false;
        }
      }
      request->send(200, "text/plain", "OK");
    });

  server.begin();
}

void loop() {
  temperature = max31856.readThermocoupleTemperature();
  Serial.print("Temperatura deseada: ");
  Serial.println(desiredTemp);
  Serial.print("Funcion activa: ");
  Serial.println(functionActive);
  Serial.print("Tiempo corriendo: ");
  Serial.println(timerRunning);
  Serial.print("Temperatura actual: ");
  Serial.println(temperature);
  if (functionActive) {
    // Control del calentamiento si temperatura < deseada
    if (temperature < desiredTemp) {
      ejecutarControlador(desiredTemp);

    }

    // Comprobar si estamos dentro del rango de activación del temporizador
    if (!timerRunning && temperature >= (desiredTemp - 5.0)) {
      timerRunning = true;
      startTime = millis();  // Iniciar temporizador
      Serial.println("Temporizador iniciado - Temperatura dentro del rango");
    }

    // Si el temporizador ya está corriendo, verificar si se acabó
    if (timerRunning && millis() - startTime >= maxTime) {
      functionActive = false;
      timerRunning = false;
      Serial.println("Temporizador completado - Función desactivada");
    }
  }
  else {
    ledcWrite(SSR,0);
  }
  // Si ya terminó o se apagó, activar ventilador
  if (!functionActive && temperature > 35.0) {
    fanStatus(255);  // Enciende el ventilador
    ledcWrite(SSR,0);
  } else {
    fanStatus(0);   // Apágalo cuando baje a <30 °C
  }

  static unsigned long lastNotify = 0;
  if (millis() - lastNotify >= 1000) {
    notifyClients();
    lastNotify = millis();
  }
}
