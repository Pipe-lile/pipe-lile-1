/*
   -----------------------------------------------------------------------------
   PROYECTO: TELEMETRÍA IOT SOBRE 4G (BASE)
   PLATAFORMA: ESP32 + A7670SA
   -----------------------------------------------------------------------------

   AUTOR:    Gabriel Bravo
   EMAIL:    gabriel.bravo.v@usach.cl
   FECHA:    27-NOV-2025
   VERSIÓN:  V4.1 (Base MQTT 4G)

   -----------------------------------------------------------------------------
   DESCRIPCIÓN DEL SISTEMA
   -----------------------------------------------------------------------------
   Firmware base para transmisión de datos por red celular 4G.
   Incluye conexión MQTT y reconexión robusta.
  
   Nota: Este archivo elimina toda la lógica del acelerómetro para que puedas
   integrar directamente tu sensor radar más adelante.

   -----------------------------------------------------------------------------
   DEPENDENCIAS CRÍTICAS & CONFIGURACIÓN DE LIBRERÍAS (¡LEER!)
   -----------------------------------------------------------------------------
   Si enviarás payloads grandes, modifica PubSubClient para permitir tamaños
   mayores a 256 bytes:

   [ARCHIVO A MODIFICAR]
   Ruta típica: .pio/libdeps/esp32dev/PubSubClient/src/PubSubClient.h

   [CAMBIO REQUERIDO]
   Buscar:   #define MQTT_MAX_PACKET_SIZE 256
   Cambiar:  #define MQTT_MAX_PACKET_SIZE 10240
*/

// --- 1. Bibliotecas ---
#include <Arduino.h>

#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// --- 2. Definiciones del Módem y Pines ---
#define MODEM_TX_PIN 17
#define MODEM_RX_PIN 16

// --- 3. CONFIGURACIÓN ---
#define MQTT_BUFFER_SIZE 512

// --- 4. Objetos Globales ---
HardwareSerial SerialAT(2);
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// --- 5. Variables Globales ---
const char apn[] = "wap.tmovil.cl";
const char gprsUser[] = "wap";
const char gprsPass[] = "wap";
const char* broker = "lorawansvr02.wisensor.cl";
const char* topic = "/mcutestlab/";

char jsonBuffer[MQTT_BUFFER_SIZE];

// --- Funciones ---
void TaskPublisher(void *pvParameters);
void reconnect();

// --- 6. Setup ---
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("--- Inicio: Base MQTT 4G ---");

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
  mqtt.setBufferSize(MQTT_BUFFER_SIZE);

  // --- Lanzar tarea de publicación ---
  xTaskCreatePinnedToCore(
    TaskPublisher,
    "Publisher_MQTT",
    8192,
    NULL,
    1,
    NULL,
    1
  );

  Serial.println("¡Tarea MQTT iniciada! Setup finalizado.");
}

void loop() {
  vTaskDelete(NULL);
}

// --- 7. TAREA PUBLICADORA (CORE 1) ---
void TaskPublisher(void *pvParameters) {
  while (true) {
    if (!mqtt.connected()) reconnect();
    mqtt.loop();

    int len = snprintf(
      jsonBuffer,
      MQTT_BUFFER_SIZE,
      "{\"status\": \"ok\"}"
    );

    if (len > 0) {
      size_t payloadLen = static_cast<size_t>(len);
      Serial.print(">> Publicando: ");
      Serial.println(jsonBuffer);

      if (!mqtt.publish(topic, jsonBuffer, payloadLen, false)) {
        Serial.println("Error MQTT Publish");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// --- 8. Función de Reconexión MQTT (ROBUSTA / AUTO-REPARABLE) ---
void reconnect() {
  int retries = 0;
  while (!mqtt.connected()) {
    Serial.print("Reconectando... ");

    if (!modem.isNetworkConnected()) {
      Serial.println("¡Red celular perdida! Esperando señal...");
      if (!modem.waitForNetwork(10000L)) {
        Serial.println("Fallo al recuperar red. Reintentando...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      Serial.println("Red recuperada.");
    }

    if (!modem.isGprsConnected()) {
      Serial.println("¡GPRS caído! Reconectando datos...");
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
        Serial.println("Fallo al reconectar GPRS.");
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
      Serial.println("GPRS recuperado.");
    }

    String clientId = "esp32-A7670SA-" + String(random(0xffff), HEX);
    if (mqtt.connect(clientId.c_str())) {
      Serial.println("¡MQTT Conectado!");
      mqtt.publish(topic, "ESP32 Base Re-Conectado");
      retries = 0;
    } else {
      Serial.print("Fallo MQTT, rc=");
      Serial.print(mqtt.state());
      Serial.println(" reintentando en 5 segundos");

      vTaskDelay(pdMS_TO_TICKS(5000));

      retries++;
      if (retries > 12) {
        Serial.println("¡Demasiados fallos de conexión! Reiniciando sistema...");
        delay(1000);
        ESP.restart();
      }
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {}
