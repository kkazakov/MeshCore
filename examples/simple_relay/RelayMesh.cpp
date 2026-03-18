#include "RelayMesh.h"

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pin-table helpers — resolve RELAY_PIN_* / INPUT_PIN_* build flags at
// compile time into a small runtime array so the rest of the code is clean.
// ---------------------------------------------------------------------------

static int8_t s_relay_pins[MAX_RELAY_PINS] = {
#ifdef RELAY_PIN_0
  RELAY_PIN_0,
#else
  -1,
#endif
#ifdef RELAY_PIN_1
  RELAY_PIN_1,
#else
  -1,
#endif
#ifdef RELAY_PIN_2
  RELAY_PIN_2,
#else
  -1,
#endif
#ifdef RELAY_PIN_3
  RELAY_PIN_3,
#else
  -1,
#endif
};

static int8_t s_input_pins[MAX_INPUT_PINS] = {
#ifdef INPUT_PIN_0
  INPUT_PIN_0,
#else
  -1,
#endif
#ifdef INPUT_PIN_1
  INPUT_PIN_1,
#else
  -1,
#endif
#ifdef INPUT_PIN_2
  INPUT_PIN_2,
#else
  -1,
#endif
#ifdef INPUT_PIN_3
  INPUT_PIN_3,
#else
  -1,
#endif
};

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

RelayMesh::RelayMesh(mesh::MainBoard &board, mesh::Radio &radio, mesh::MillisecondClock &ms, mesh::RNG &rng,
                     mesh::RTCClock &rtc, mesh::MeshTables &tables)
    : SensorMesh(board, radio, ms, rng, rtc, tables), relay_count(0), input_count(0), relay_active_low(false),
      input_active_low(false), relay_state(0) {
#ifdef RELAY_ACTIVE_LOW
  relay_active_low = (RELAY_ACTIVE_LOW != 0);
#endif
#ifdef INPUT_ACTIVE_LOW
  input_active_low = (INPUT_ACTIVE_LOW != 0);
#endif

  // Copy compile-time tables, count valid pins
  for (int i = 0; i < MAX_RELAY_PINS; i++) {
    relay_pins[i] = s_relay_pins[i];
    if (relay_pins[i] >= 0) relay_count = i + 1;
  }
  for (int i = 0; i < MAX_INPUT_PINS; i++) {
    input_pins[i] = s_input_pins[i];
    if (input_pins[i] >= 0) input_count = i + 1;
  }
}

// ---------------------------------------------------------------------------
// Pin initialisation — called from main.cpp setup() before the_mesh.begin()
// ---------------------------------------------------------------------------

void RelayMesh::initPins() {
  for (int i = 0; i < MAX_RELAY_PINS; i++) {
    if (relay_pins[i] >= 0) {
      pinMode(relay_pins[i], OUTPUT);
    }
  }
  for (int i = 0; i < MAX_INPUT_PINS; i++) {
    if (input_pins[i] >= 0) {
      pinMode(input_pins[i], INPUT_PULLUP);
    }
  }
  // Ensure all relays start in OFF state
  applyRelayState(0);
}

// ---------------------------------------------------------------------------
// Low-level relay write
// ---------------------------------------------------------------------------

void RelayMesh::applyRelayState(uint8_t new_state) {
  relay_state = new_state;
  for (int i = 0; i < MAX_RELAY_PINS; i++) {
    if (relay_pins[i] < 0) continue;
    bool on = (new_state >> i) & 1;
    if (relay_active_low) on = !on;
    digitalWrite(relay_pins[i], on ? HIGH : LOW);
  }
}

// ---------------------------------------------------------------------------
// Low-level input read
// ---------------------------------------------------------------------------

uint8_t RelayMesh::readInputState() const {
  uint8_t state = 0;
  for (int i = 0; i < MAX_INPUT_PINS; i++) {
    if (input_pins[i] < 0) continue;
    bool raw = digitalRead(input_pins[i]);
    // active-low means pin reads LOW when active
    bool active = input_active_low ? !raw : raw;
    if (active) state |= (1 << i);
  }
  return state;
}

// ---------------------------------------------------------------------------
// SensorMesh::onSensorDataRead override
// Called periodically (SENSOR_READ_INTERVAL_SECS) to update telemetry.
// ---------------------------------------------------------------------------

void RelayMesh::onSensorDataRead() {
  // Report each relay output state as a CayenneLPP digital output channel
  for (int i = 0; i < MAX_RELAY_PINS; i++) {
    if (relay_pins[i] < 0) continue;
    bool on = (relay_state >> i) & 1;
    // CayenneLPP digital output: channel, value (0 or 1)
    telemetry.addDigitalOutput(RELAY_TELEM_CH_BASE + i, on ? 1 : 0);
  }

  // Report each digital input state as a CayenneLPP digital input channel
  uint8_t inputs = readInputState();
  for (int i = 0; i < MAX_INPUT_PINS; i++) {
    if (input_pins[i] < 0) continue;
    bool active = (inputs >> i) & 1;
    telemetry.addDigitalInput(INPUT_TELEM_CH_BASE + i, active ? 1 : 0);
  }
}

// ---------------------------------------------------------------------------
// SensorMesh::querySeriesData override (relays have no historical series)
// ---------------------------------------------------------------------------

int RelayMesh::querySeriesData(uint32_t /*start_secs_ago*/, uint32_t /*end_secs_ago*/, MinMaxAvg /*dest*/[],
                               int /*max_num*/) {
  return 0; // no time-series data for relay node
}

// ---------------------------------------------------------------------------
// Helper: parse a relay index from a string token ("0".."3")
// ---------------------------------------------------------------------------

bool RelayMesh::parseRelayIndex(const char *str, uint8_t &idx) const {
  if (str == NULL || str[0] < '0' || str[0] > '3' || str[1] != '\0') return false;
  idx = str[0] - '0';
  if (relay_pins[idx] < 0) return false; // pin not configured
  return true;
}

// ---------------------------------------------------------------------------
// handleCustomCommand — relay-specific CLI commands
//
// Syntax:
//   relay on {n}       turn relay n ON
//   relay off {n}      turn relay n OFF
//   relay toggle {n}   toggle relay n
//   relay set {hex}    set all relays by bitmask  (e.g. "relay set f")
//   relay get          print current relay states as hex
//   input get          print current input states as hex
// ---------------------------------------------------------------------------

bool RelayMesh::handleCustomCommand(uint32_t /*sender_timestamp*/, char *command, char *reply) {
  // Skip leading spaces
  while (*command == ' ')
    command++;

  // ---- relay commands ----
  if (memcmp(command, "relay ", 6) == 0) {
    const char *sub = command + 6;
    while (*sub == ' ')
      sub++;

    if (memcmp(sub, "on ", 3) == 0) {
      // relay on {n}
      uint8_t idx;
      if (!parseRelayIndex(sub + 3, idx)) {
        strcpy(reply, "Err - bad relay index (0-3)");
      } else {
        applyRelayState(relay_state | (1 << idx));
        sprintf(reply, "relay %d ON, state=%x", (int)idx, (unsigned)relay_state);
      }
      return true;
    }

    if (memcmp(sub, "off ", 4) == 0) {
      // relay off {n}
      uint8_t idx;
      if (!parseRelayIndex(sub + 4, idx)) {
        strcpy(reply, "Err - bad relay index (0-3)");
      } else {
        applyRelayState(relay_state & ~(1 << idx));
        sprintf(reply, "relay %d OFF, state=%x", (int)idx, (unsigned)relay_state);
      }
      return true;
    }

    if (memcmp(sub, "toggle ", 7) == 0) {
      // relay toggle {n}
      uint8_t idx;
      if (!parseRelayIndex(sub + 7, idx)) {
        strcpy(reply, "Err - bad relay index (0-3)");
      } else {
        applyRelayState(relay_state ^ (1 << idx));
        bool now_on = (relay_state >> idx) & 1;
        sprintf(reply, "relay %d %s, state=%x", (int)idx, now_on ? "ON" : "OFF", (unsigned)relay_state);
      }
      return true;
    }

    if (memcmp(sub, "set ", 4) == 0) {
      // relay set {hex}
      unsigned int val = 0;
      if (sscanf(sub + 4, "%x", &val) != 1) {
        strcpy(reply, "Err - expected hex bitmask");
      } else {
        // Mask to only configured relays
        uint8_t mask = 0;
        for (int i = 0; i < MAX_RELAY_PINS; i++) {
          if (relay_pins[i] >= 0) mask |= (1 << i);
        }
        applyRelayState((uint8_t)(val & mask));
        sprintf(reply, "state=%x", (unsigned)relay_state);
      }
      return true;
    }

    if (strcmp(sub, "get") == 0) {
      // relay get
      sprintf(reply, "state=%x (%d relay(s))", (unsigned)relay_state, (int)relay_count);
      return true;
    }

    // Unknown relay sub-command — fall through to show help
    strcpy(reply, "Usage: relay on {n} | relay off {n} | relay toggle {n} | "
                  "relay set {hex} | relay get");
    return true;
  }

  // ---- input commands ----
  if (strcmp(command, "input get") == 0) {
    uint8_t inputs = readInputState();
    sprintf(reply, "inputs=%x (%d input(s))", (unsigned)inputs, (int)input_count);
    return true;
  }

  return false; // not handled here — pass to CommonCLI
}
