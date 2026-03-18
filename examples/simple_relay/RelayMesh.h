#pragma once

#include "../simple_sensor/SensorMesh.h"

// ---------------------------------------------------------------------------
// Build-flag relay/input pin declarations
//
// Define up to 4 relay output pins and 4 digital input pins via platformio.ini:
//
//   -D RELAY_PIN_0=5     ; GPIO pin for relay 0 (OUTPUT, active-HIGH by default)
//   -D RELAY_PIN_1=6     ; GPIO pin for relay 1
//   -D RELAY_PIN_2=7     ; GPIO pin for relay 2
//   -D RELAY_PIN_3=8     ; GPIO pin for relay 3
//
//   -D RELAY_ACTIVE_LOW=1   ; set if relay board uses active-LOW logic
//
//   -D INPUT_PIN_0=4     ; GPIO pin for digital input 0 (INPUT_PULLUP by default)
//   -D INPUT_PIN_1=3     ; GPIO pin for digital input 1
//   -D INPUT_PIN_2=2     ; GPIO pin for digital input 2
//   -D INPUT_PIN_3=1     ; GPIO pin for digital input 3
//
//   -D INPUT_ACTIVE_LOW=1   ; set if inputs are active-LOW (pulled low when active)
//
// Relay outputs are reported on CayenneLPP channels 10-13 (LPP_DIGITAL_OUTPUT).
// Digital inputs are reported on CayenneLPP channels 20-23 (LPP_DIGITAL_INPUT).
//
// CLI commands (sent from companion app as CLI data after login):
//
//   relay on {n}          - turn relay N (0-3) on
//   relay off {n}         - turn relay N (0-3) off
//   relay toggle {n}      - toggle relay N (0-3)
//   relay set {hex}       - set all relays at once (bitmask, e.g. 'relay set f' = all on)
//   relay get             - get current relay state as hex bitmask
//   input get             - get current digital input states as hex bitmask
// ---------------------------------------------------------------------------

#ifndef FIRMWARE_BUILD_DATE
#define FIRMWARE_BUILD_DATE "18 Mar 2026"
#endif

#ifndef FIRMWARE_VERSION
#define FIRMWARE_VERSION "v1.0.0"
#endif

#define FIRMWARE_ROLE       "relay"

// CayenneLPP channels for relay state reporting
#define RELAY_TELEM_CH_BASE 10 // channels 10..13 for relay outputs
#define INPUT_TELEM_CH_BASE 20 // channels 20..23 for digital inputs

// Maximum relays and inputs supported
#define MAX_RELAY_PINS      4
#define MAX_INPUT_PINS      4

// CayenneLPP type codes used for GPIO reporting
#define LPP_DIGITAL_INPUT   0x00
#define LPP_DIGITAL_OUTPUT  0x01

class RelayMesh : public SensorMesh {
public:
  RelayMesh(mesh::MainBoard &board, mesh::Radio &radio, mesh::MillisecondClock &ms, mesh::RNG &rng,
            mesh::RTCClock &rtc, mesh::MeshTables &tables);

  const char *getFirmwareVer() override { return FIRMWARE_VERSION; }
  const char *getBuildDate() override { return FIRMWARE_BUILD_DATE; }
  const char *getRole() override { return FIRMWARE_ROLE; }

  // Called from main.cpp setup() to configure GPIO directions and initial state.
  void initPins();

protected:
  // SensorMesh overrides
  void onSensorDataRead() override;
  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override;
  bool handleCustomCommand(uint32_t sender_timestamp, char *command, char *reply) override;

private:
  // Pin tables (populated from build flags, -1 = unused)
  int8_t relay_pins[MAX_RELAY_PINS];
  int8_t input_pins[MAX_INPUT_PINS];
  uint8_t relay_count;
  uint8_t input_count;
  bool relay_active_low;
  bool input_active_low;

  // Current relay output state bitmask (bit N = relay N)
  uint8_t relay_state;

  // Helpers
  void applyRelayState(uint8_t new_state);
  uint8_t readInputState() const;
  bool parseRelayIndex(const char *str, uint8_t &idx) const;
};
