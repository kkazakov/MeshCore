#include "RelayMesh.h"

#ifdef DISPLAY_CLASS
// Include display UI task if a display is configured for the target hardware
#include "../simple_sensor/UITask.h"
static UITask ui_task(display);
#endif

// ---------------------------------------------------------------------------
// Application-level MyMesh subclass
//
// Put any project-specific customisation here. Trigger-based alerts and
// time-series data are inherited from SensorMesh; relay commands are
// handled by RelayMesh::handleCustomCommand().
// ---------------------------------------------------------------------------

class MyMesh : public RelayMesh {
public:
  MyMesh(mesh::MainBoard &board, mesh::Radio &radio, mesh::MillisecondClock &ms, mesh::RNG &rng,
         mesh::RTCClock &rtc, mesh::MeshTables &tables)
      : RelayMesh(board, radio, ms, rng, rtc, tables) {}

protected:
  /* =====================  custom alerts / logic here  ==================== */
  // Example: alert when an input goes active (e.g. door sensor, PIR)
  //   Trigger door_open;
  //
  //   void onSensorDataRead() override {
  //     RelayMesh::onSensorDataRead(); // IMPORTANT: call base to update telemetry
  //     bool door = (readInputState() & 0x01) != 0;
  //     alertIf(door, door_open, HIGH_PRI_ALERT, "Door opened!");
  //   }
  /* ======================================================================= */
};

// ---------------------------------------------------------------------------
// Global objects (platform-specific board/radio/sensors are in target.cpp)
// ---------------------------------------------------------------------------

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

// ---------------------------------------------------------------------------

void halt() {
  while (1)
    ;
}

static char command[160];

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.print("Relay Node");
    display.endFrame();
  }
#endif

  if (!radio_init()) {
    halt();
  }

  fast_rng.begin(radio_get_rng_seed());

  // Initialise relay output and digital input pins BEFORE mesh begin
  the_mesh.initPins();

  FILESYSTEM *fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
#error "need to define filesystem for this platform"
#endif

  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {
      the_mesh.self_id = radio_new_identity();
      count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Relay ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE);
  Serial.println();

  command[0] = 0;

  sensors.begin();
  the_mesh.begin(fs);

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  // Send initial zero-hop advertisement
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, false);
#endif
}

void loop() {
  // Handle local serial CLI input (useful for debug / initial config)
  int len = strlen(command);
  while (Serial.available() && len < (int)sizeof(command) - 1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len == (int)sizeof(command) - 1) {
    command[sizeof(command) - 1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {
    command[len - 1] = 0;
    char reply[160];
    the_mesh.handleCommand(0, command, reply);
    if (reply[0]) {
      Serial.print("  -> ");
      Serial.println(reply);
    }
    command[0] = 0;
  }

  the_mesh.loop();
  sensors.loop();

#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif

  rtc_clock.tick();
}
