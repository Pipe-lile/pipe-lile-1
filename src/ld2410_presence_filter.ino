/*
  Ejemplo para ESP32 + sensor de presencia por radar LD2410B.
  Variante con filtrado temporal y suavizado de señal para reducir
  falsos positivos cuando no hay presencia real.

  Basado en la librería: MyLD2410 (https://github.com/iavorvel/MyLD2410)
*/

#include "MyLD2410.h"

// -----------------------------------------------------------------------------
// Configuración de hardware
// -----------------------------------------------------------------------------
#define SENSOR_SERIAL Serial1
#define RX_PIN 16
#define TX_PIN 17

// Velocidades de los puertos serie
#define SERIAL_BAUD_RATE 115200
#define LD2410_BAUD_RATE 256000

// Activa modo mejorado del sensor (comentarlo para modo básico)
#define ENHANCED_MODE

// -----------------------------------------------------------------------------
// Parámetros ajustables de filtrado y temporización
// -----------------------------------------------------------------------------
struct UserConfig {
  uint16_t minSignalMoving   = 80;   // Señal mínima para objetivo en movimiento
  uint16_t minSignalStatic   = 50;   // Señal mínima para objetivo estacionario
  uint16_t maxDistanceCm     = 600;  // Distancia máxima a considerar
  uint8_t  framesToConfirm   = 10;    // Tramas consecutivas para confirmar presencia
  uint8_t  framesToRelease   = 1;    // Tramas consecutivas sin presencia para limpiar estado
  uint8_t  framesToForget    = 1;    // Reducción gradual cuando hay ruido
  uint8_t  smoothingPercent  = 0;   // Porcentaje (0-100) para suavizado exponencial
  uint16_t printIntervalMs   = 1500; // Intervalo de impresión en loop
};

UserConfig cfg;
MyLD2410 sensor(SENSOR_SERIAL);

unsigned long nextPrint = 0;

// Estado interno del filtro
struct PresenceState {
  uint8_t goodFrames = 0;
  uint8_t badFrames = 0;
  uint16_t movingFiltered = 0;
  uint16_t staticFiltered = 0;
};

PresenceState presenceState;

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
  Serial.print(F("Intervalo impresion   : ")); Serial.print(c.printIntervalMs); Serial.println(F(" ms"));
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
    if (state.goodFrames < c.framesToConfirm) {
      state.goodFrames += 1;
    }
    state.badFrames = 0; // reiniciar ausencia cuando hay señal válida
  } else {
    if (state.badFrames < c.framesToRelease) {
      state.badFrames += 1;
    }
    // Reducimos lentamente la confianza para evitar falsos "apagados"
    if (state.goodFrames > 0 && state.badFrames >= c.framesToForget) {
      state.goodFrames -= 1;
    }
  }

  // Solo consideramos ausencia real tras varias tramas sin señal
  const bool confirmedPresence = state.goodFrames >= c.framesToConfirm;
  const bool confirmedAbsence = state.badFrames >= c.framesToRelease;

  return confirmedPresence && !confirmedAbsence;
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

  // Imprimir la configuración antes de iniciar el loop
  printConfig(cfg);

  nextPrint = millis() + cfg.printIntervalMs;
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
}
