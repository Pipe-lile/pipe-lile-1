/*
   -----------------------------------------------------------------------------
   PROYECTO: TELEMETRÍA SÍSMICA/VIBRACIONES IOT SOBRE 4G (V3.2 - GPS)
   PLATAFORMA: ESP32 + A7670SA
   -----------------------------------------------------------------------------

   OBJETIVO DE ESTE ARCHIVO
   -----------------------------------------------------------------------------
   Versión reducida que conserva solo la inicialización del módem 4G,
   la conexión a red/GPRS y la transmisión MQTT. Se eliminaron tareas,
   sensores y GPS para dejar exclusivamente la ruta de datos 4G.
*/

// --- 1. Bibliotecas ---
#include <Arduino.h>

#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// --- 2. Definiciones del Módem y Pines ---
#define MODEM_TX_PIN 17
#define MODEM_RX_PIN 16

// --- 3. Configuración de Red/MQTT ---
const char apn[] = "wap.tmovil.cl";
const char gprsUser[] = "wap";
const char gprsPass[] = "wap";
const char* broker = "lorawansvr02.wisensor.cl";
const char* topic = "/mcutestlab/";

#define MQTT_BUFFER_SIZE 10240

// --- 4. Objetos Globales ---
HardwareSerial SerialAT(2);
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// --- 5. Prototipos ---
void reconnect();
void publishPayload(const char* payload);
void callback(char* topic, byte* payload, unsigned int length);

// --- 6. Setup ---
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("--- Inicio: Transmisión 4G MQTT (solo red) ---");

  Serial.println("Iniciando Módem...");
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
  if (!modem.restart()) {
    Serial.println("Fallo Módem. Reiniciando...");
    delay(1000);
    ESP.restart();
  }

  Serial.println("Conectando Red...");
  if (!modem.waitForNetwork() || !modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println("Fallo Red/GPRS");
    delay(5000);
    return;
  }

  mqtt.setServer(broker, 1883);
  mqtt.setCallback(callback);
  mqtt.setBufferSize(MQTT_BUFFER_SIZE);

  Serial.println("Setup finalizado.");
}

// --- 7. Loop principal ---
void loop() {
  if (!mqtt.connected()) {
    reconnect();
  }
  mqtt.loop();

  // Envío de ejemplo cada 5 segundos.
  static unsigned long lastSend = 0;
  if (millis() - lastSend >= 5000) {
    publishPayload("{\"status\":\"ok\",\"msg\":\"4G activo\"}");
    lastSend = millis();
  }
}

// --- 8. Envío MQTT ---
void publishPayload(const char* payload) {
  size_t payloadLen = strlen(payload);
  Serial.print(">> Enviando payload MQTT (bytes: ");
  Serial.print(payloadLen);
  Serial.println(")");

  if (mqtt.beginPublish(topic, payloadLen, false)) {
    size_t sent = 0;
    while (sent < payloadLen) {
      size_t chunk = (payloadLen - sent > 1024) ? 1024 : (payloadLen - sent);
      client.write((const uint8_t*)(payload + sent), chunk);
      sent += chunk;
    }
    mqtt.endPublish();
  } else {
    Serial.println("Error MQTT Publish");
  }
}

// --- 9. Reconexión robusta ---
void reconnect() {
  int retries = 0;
  while (!mqtt.connected()) {
    Serial.print("Reconectando... ");

    if (!modem.isNetworkConnected()) {
      Serial.println("¡Red celular perdida! Esperando señal...");
      if (!modem.waitForNetwork(10000L)) {
        Serial.println("Fallo al recuperar red. Reintentando...");
        delay(1000);
        continue;
      }
      Serial.println("Red recuperada.");
    }

    if (!modem.isGprsConnected()) {
      Serial.println("¡GPRS caído! Reconectando datos...");
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println("Fallo al reconectar GPRS.");
        delay(2000);
        continue;
      }
      Serial.println("GPRS recuperado.");
    }

    String clientId = "esp32-A7670SA-" + String(random(0xffff), HEX);
    if (mqtt.connect(clientId.c_str())) {
      Serial.println("¡MQTT Conectado!");
      mqtt.publish(topic, "ESP32 4G reconectado");
      retries = 0;
    } else {
      Serial.print("Fallo MQTT, rc=");
      Serial.print(mqtt.state());
      Serial.println(" reintentando en 5 segundos");
      delay(5000);

      retries++;
      if (retries > 12) {
        Serial.println("¡Demasiados fallos de conexión! Reiniciando sistema...");
        delay(1000);
        ESP.restart();
      }
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  (void)topic;
  (void)payload;
  (void)length;
}
