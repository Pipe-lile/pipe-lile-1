/*
  Ejemplo para ESP32 + sensor de presencia por radar LD2410B.
  Variante con filtrado temporal, suavizado de señal y
  confirmación por tiempo para reducir falsos positivos.
  Envio por 4G con MQTT a lorawansvr02.wisensor.cl.
  Basado en la librería: MyLD2410 (https://github.com/iavorvel/MyLD2410)

  Requiere:
  - TinyGSM (https://github.com/vshymanskyy/TinyGSM)
  - PubSubClient (https://github.com/knolleary/pubsubclient)
*/

// Definir el modelo del módem antes de incluir TinyGSM
#define TINY_GSM_MODEM_SIM7600

#include "MyLD2410.h"
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// -----------------------------------------------------------------------------
// Configuración de hardware
// -----------------------------------------------------------------------------
#define SENSOR_SERIAL Serial1
#define RX_PIN 26
#define TX_PIN 27

// Módem 4G (ajusta pines/serial según tu módulo)
#define MODEM_SERIAL Serial2
#define MODEM_RX_PIN 16
#define MODEM_TX_PIN 17
#define MODEM_PWR_PIN 4

// Velocidades de los puertos serie
#define SERIAL_BAUD_RATE 115200
#define LD2410_BAUD_RATE 256000
#define MODEM_BAUD_RATE 115200

// Activa modo mejorado del sensor (comentarlo para modo básico)
#define ENHANCED_MODE

// -----------------------------------------------------------------------------
// Configuración MQTT / 4G
// -----------------------------------------------------------------------------
const char *apn = "internet";      // Reemplazar por APN del operador
const char *gprsUser = "";
const char *gprsPass = "";

const char *mqttServer = "lorawansvr02.wisensor.cl";
const uint16_t mqttPort = 1883;
const char *topic = "/mcutestlab/";

// -----------------------------------------------------------------------------
// Parámetros ajustables de filtrado y temporización
// -----------------------------------------------------------------------------
struct UserConfig {
  uint16_t minSignalMoving   = 80;   // Señal mínima para objetivo en movimiento
  uint16_t minSignalStatic   = 50;   // Señal mínima para objetivo estacionario
  uint16_t maxDistanceCm     = 300;  // Distancia máxima a considerar
  uint8_t  framesToConfirm   = 10;   // Tramas consecutivas para confirmar presencia
  uint8_t  framesToRelease   = 3;    // Tramas consecutivas sin presencia para liberar
  uint8_t  framesToForget    = 2;    // Reducción gradual cuando hay ruido
  uint8_t  smoothingPercent  = 10;   // Porcentaje (0-100) para suavizado exponencial
  uint16_t minPresenceMs     = 200;  // Tiempo mínimo continuo para confirmar presencia
  uint16_t minAbsenceMs      = 200;  // Tiempo mínimo continuo sin señal para liberar
  uint16_t printIntervalMs   = 2000; // Intervalo de impresión en loop
  uint16_t publishIntervalMs = 5000; // Intervalo de envío por MQTT
};

UserConfig cfg;
MyLD2410 sensor(SENSOR_SERIAL);

unsigned long nextPrint = 0;
unsigned long nextPublish = 0;

// -----------------------------------------------------------------------------
// Estado interno del filtro
// -----------------------------------------------------------------------------
struct PresenceState {
  uint8_t goodFrames = 0;
  uint8_t badFrames = 0;
  uint16_t movingFiltered = 0;
  uint16_t staticFiltered = 0;
  unsigned long candidateStartMs = 0;
  unsigned long lastSeenMs = 0;
  bool presenceLatched = false;
};

PresenceState presenceState;

// -----------------------------------------------------------------------------
// Cliente 4G + MQTT
// -----------------------------------------------------------------------------
TinyGsm modem(MODEM_SERIAL);
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);

// -----------------------------------------------------------------------------
// Utilidades de impresión
// -----------------------------------------------------------------------------
void printValue(const byte &val) {
  Serial.print(' ');
  Serial.print(val);
}

void printConfig(const UserConfig &c) {
  Serial.println(F("====== Configuracion aplicada ======"));
  Serial.print(F("RX/TX pins            : ")); Serial.print(RX_PIN); Serial.print('/'); Serial.println(TX_PIN);
  Serial.print(F("Modo mejorado         : ")); Serial.println(sensor.inEnhancedMode() ? F("SI") : F("NO"));
  Serial.print(F("Min señal movimiento  : ")); Serial.println(c.minSignalMoving);
  Serial.print(F("Min señal estatico    : ")); Serial.println(c.minSignalStatic);
  Serial.print(F("Max distancia (cm)    : ")); Serial.println(c.maxDistanceCm);
  Serial.print(F("Tramas confirmar      : ")); Serial.println(c.framesToConfirm);
  Serial.print(F("Tramas soltar         : ")); Serial.println(c.framesToRelease);
  Serial.print(F("Tramas olvidar        : ")); Serial.println(c.framesToForget);
  Serial.print(F("Suavizado (%)         : ")); Serial.println(c.smoothingPercent);
  Serial.print(F("Min presencia (ms)    : ")); Serial.println(c.minPresenceMs);
  Serial.print(F("Min ausencia (ms)     : ")); Serial.println(c.minAbsenceMs);
  Serial.print(F("Intervalo impresion   : ")); Serial.print(c.printIntervalMs); Serial.println(F(" ms"));
  Serial.print(F("Intervalo publicacion : ")); Serial.print(c.publishIntervalMs); Serial.println(F(" ms"));
  Serial.println(F("===================================="));
  Serial.println();
}

// Helper genérico para arrays de la librería
template <typename ArrayT>
void printSignals(const char *title,
                  const char *distanceLabel,
                  bool detected,
                  uint16_t signalStrength,
                  uint16_t distance,
                  const ArrayT &signals,
                  const ArrayT &thresholds) {
  Serial.print(F("- "));
  Serial.print(title);
  Serial.print(F(": "));
  if (!detected) {
    Serial.println(F("sin deteccion"));
    return;
  }

  Serial.print(signalStrength);
  Serial.print(F(" @ "));
  Serial.print(distance);
  Serial.print(distanceLabel);
  Serial.println();

  if (sensor.inEnhancedMode()) {
    Serial.print(F("  senales : ["));
    signals.forEach(printValue);
    Serial.println(F(" ]"));

    Serial.print(F("  umbrales: ["));
    thresholds.forEach(printValue);
    Serial.println(F(" ]"));
  }
}

// -----------------------------------------------------------------------------
// Filtro de presencia configurable con suavizado y desacople de ruido
// -----------------------------------------------------------------------------
uint16_t smoothSignal(uint16_t previous, uint16_t current, uint8_t smoothingPercent) {
  const uint8_t clamp = smoothingPercent > 100 ? 100 : smoothingPercent;
  // Promedio exponencial: mayor porcentaje = más peso al valor previo
  return (previous * clamp + current * (100 - clamp)) / 100;
}

bool presenceFiltered(MyLD2410 &ld, PresenceState &state, const UserConfig &c) {
  const unsigned long now = millis();

  // Suavizamos las señales para amortiguar picos aislados
  state.movingFiltered = smoothSignal(state.movingFiltered, ld.movingTargetSignal(), c.smoothingPercent);
  state.staticFiltered = smoothSignal(state.staticFiltered, ld.stationaryTargetSignal(), c.smoothingPercent);

  const bool movOK = ld.movingTargetDetected() &&
                     state.movingFiltered >= c.minSignalMoving &&
                     ld.movingTargetDistance() <= c.maxDistanceCm;

  const bool staOK = ld.stationaryTargetDetected() &&
                     state.staticFiltered >= c.minSignalStatic &&
                     ld.stationaryTargetDistance() <= c.maxDistanceCm;

  const bool presenceCandidate = movOK || staOK;

  if (presenceCandidate) {
    if (state.goodFrames == 0) {
      state.candidateStartMs = now;
    }
    if (state.goodFrames < c.framesToConfirm) {
      state.goodFrames += 1;
    }
    state.badFrames = 0;
    state.lastSeenMs = now;
  } else {
    if (state.badFrames < c.framesToRelease) {
      state.badFrames += 1;
    }
    if (state.goodFrames > 0 && state.badFrames >= c.framesToForget) {
      state.goodFrames -= 1;
      if (state.goodFrames == 0) {
        state.candidateStartMs = 0;
      }
    }
  }

  const bool timeConfirmed = state.goodFrames >= c.framesToConfirm &&
                             (now - state.candidateStartMs) >= c.minPresenceMs;
  if (timeConfirmed) {
    state.presenceLatched = true;
  }

  const bool absenceConfirmed = state.badFrames >= c.framesToRelease &&
                                (now - state.lastSeenMs) >= c.minAbsenceMs;
  if (absenceConfirmed) {
    state.presenceLatched = false;
  }

  return state.presenceLatched;
}

// -----------------------------------------------------------------------------
// MQTT helpers
// -----------------------------------------------------------------------------
void modemPowerOn() {
  pinMode(MODEM_PWR_PIN, OUTPUT);
  digitalWrite(MODEM_PWR_PIN, HIGH);
  delay(1000);
  digitalWrite(MODEM_PWR_PIN, LOW);
}

bool ensureMqttConnected() {
  if (mqtt.connected()) {
    return true;
  }

  Serial.println(F("Conectando a red 4G..."));
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(F("Fallo en GPRS."));
    return false;
  }

  Serial.println(F("Conectando a MQTT..."));
  const String clientId = String(F("esp32-ld2410-")) + String(random(0xFFFF), HEX);
  if (!mqtt.connect(clientId.c_str())) {
    Serial.print(F("Fallo MQTT, rc="));
    Serial.println(mqtt.state());
    return false;
  }

  Serial.println(F("MQTT conectado."));
  return true;
}

void publishPresence(bool presenceSoft) {
  if (!ensureMqttConnected()) {
    return;
  }

  const uint16_t distance = sensor.detectedDistance();
  const uint16_t movSignal = presenceState.movingFiltered;
  const uint16_t staSignal = presenceState.staticFiltered;

  char payload[256];
  snprintf(payload, sizeof(payload),
           "{\"presence\":%s,\"distance_cm\":%u,\"moving_signal\":%u,\"static_signal\":%u}",
           presenceSoft ? "true" : "false",
           distance,
           movSignal,
           staSignal);

  if (!mqtt.publish(topic, payload)) {
    Serial.println(F("Error publicando MQTT."));
  } else {
    Serial.print(F("Publicado MQTT en "));
    Serial.print(topic);
    Serial.print(F(": "));
    Serial.println(payload);
  }
}

// -----------------------------------------------------------------------------
// Impresión de datos del sensor
// -----------------------------------------------------------------------------
void printData(bool presenceSoft) {
  Serial.println(F("================ LD2410 ================"));

  Serial.print(F("Estado           : "));
  Serial.println(sensor.statusString());

  const bool presenceRaw = sensor.presenceDetected();
  Serial.print(F("Presencia (raw)  : "));
  Serial.println(presenceRaw ? F("SI") : F("NO"));

  Serial.print(F("Presencia filtra.: "));
  Serial.println(presenceSoft ? F("SI") : F("NO"));

  if (presenceRaw) {
    Serial.print(F("Distancia obj.   : "));
    Serial.print(sensor.detectedDistance());
    Serial.println(F(" cm"));
  }

  printSignals("Movimiento", " cm", sensor.movingTargetDetected(),
               sensor.movingTargetSignal(), sensor.movingTargetDistance(),
               sensor.getMovingSignals(), sensor.getMovingThresholds());

  printSignals("Estacionario", " cm", sensor.stationaryTargetDetected(),
               sensor.stationaryTargetSignal(), sensor.stationaryTargetDistance(),
               sensor.getStationarySignals(), sensor.getStationaryThresholds());

  if (sensor.inEnhancedMode() && (sensor.getFirmwareMajor() > 1)) {
    Serial.print(F("Nivel de luz     : "));
    Serial.println(sensor.getLightLevel());

    Serial.print(F("Nivel de salida  : "));
    Serial.println(sensor.getOutLevel() ? F("ALTO") : F("BAJO"));
  }

  Serial.println(F("========================================"));
  Serial.println();
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  SENSOR_SERIAL.begin(LD2410_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  MODEM_SERIAL.begin(MODEM_BAUD_RATE, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  delay(2000);
  Serial.println(__FILE__);

  if (!sensor.begin()) {
    Serial.println(F("No se pudo comunicar con el sensor."));
    while (true) {}
  }

#ifdef ENHANCED_MODE
  sensor.enhancedMode();
#else
  sensor.enhancedMode(false);
#endif

  modemPowerOn();
  modem.restart();

  mqtt.setServer(mqttServer, mqttPort);

  // Imprimir la configuración antes de iniciar el loop
  printConfig(cfg);

  nextPrint = millis() + cfg.printIntervalMs;
  nextPublish = millis() + cfg.publishIntervalMs;
}

// -----------------------------------------------------------------------------
// Loop principal
// -----------------------------------------------------------------------------
void loop() {
  const auto resp = sensor.check();
  if (resp == MyLD2410::Response::DATA && millis() > nextPrint) {
    nextPrint = millis() + cfg.printIntervalMs;

    const bool presenceSoft = presenceFiltered(sensor, presenceState, cfg);
    printData(presenceSoft);
  }

  if (millis() > nextPublish) {
    nextPublish = millis() + cfg.publishIntervalMs;
    const bool presenceSoft = presenceFiltered(sensor, presenceState, cfg);
    publishPresence(presenceSoft);
  }

  mqtt.loop();
}
