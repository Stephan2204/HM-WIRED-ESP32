#include <Arduino.h>
#include <WiFi.h>
#include <ETH.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <Update.h>
#include <nvs.h>
#include <esp_system.h>

HardwareSerial HM485(2);
WebServer webServer(80);
Preferences preferences;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ============================================================
// HM485 PROJECT NOTES / RESEARCH LOG (v0.7.45)
//
// v0.7.43 NEW EVIDENCE / PURPOSE
// - v0.7.42b with A/B connected: every active DBA7/FB probe produced one 0x00
//   roughly one UART character time (~590 us) after DIR was released to RX.
// - v0.7.42b with A/B physically disconnected: 0 RX bytes in all hold-high and
//   turnaround tests. Therefore the 0x00 is bus-side dependent, not a purely local
//   ESP32 UART FIFO echo.
// - Turnaround delays 0..100 us and a 1500 us TX hold did not change the result.
// - v0.7.43 therefore observes GPIO35 (SP485 RO -> ESP32 RX) directly after DIR LOW.
//   It records logic transitions and longest LOW/HIGH runs for 1800 us, then drains
//   the UART FIFO. This distinguishes a real serial byte from a BREAK/long-low or
//   other electrical waveform without depending on HardwareSerial error reporting.
// - IMPORTANT: GPIO sampling is diagnostic only; normal parser/network/MQTT behavior
//   remains based on the stable v0.7.40 lineage.
// ============================================================
//
// IMPORTANT BASELINE
// ------------------
// v0.7.40 is the last known-good normal gateway baseline. It added the
// permanent web-switchable RAW RX ONLY mode. In that mode DIR is forced
// LOW (receive), all WT32 HM485 TX is inhibited, the protocol parser is
// bypassed and every UART byte is logged with timing. KEEP THIS FEATURE
// in all future versions: it is our built-in passive bus analyzer.
//
// HARDWARE / ELECTRICAL FINDINGS
// ------------------------------
// - WT32-ETH01: RX GPIO35, TX GPIO17, DIR GPIO33.
// - Current transceiver is SP485 with DE and /RE controlled together by
//   DIR. The earlier MAX485 setup was replaced during troubleshooting.
// - RX is proven good on the real production bus. v0.7.39 RX-only saw
//   reproducible structured HM485 traffic from real switches and hm485d.
// - Therefore A/B polarity, RX wiring, UART RX pin and 19200 baud work.
// - An isolated self-TX often leaves one 0x00 after TX->RX; treat it as a
//   local turnaround/transceiver artifact unless proven otherwise.
//
// DISCOVERY FINDINGS SO FAR
// -------------------------
// 1. hm485d discovery is a 32-bit tree/prefix search, not a conventional
//    brute-force scan. Passive capture saw 00000000/03, /0B ... /7B, then
//    branches such as 00008000/83, 0000C000/8B and finally
//    the final 32-bit branch split for a known device.
// 2. The user's subnetting analogy is a very useful model: DST is the
//    address/prefix under test and CTRL carries the prefix depth (valid
//    bits). A positive response means at least one device exists below
//    that branch. Exact bit mapping still has to be documented/proven.
// 3. Working hm485d discovery repeats negative/ambiguous probes up to three
//    times. Passive captures show ~20 ms retry timing and progressive tree
//    descent after positive responses.
// 4. Exact hm485d-style discovery frame generation is implemented here:
//    start FD, 4-byte DST, CTRL, no sender for discovery, length 02, CRC16,
//    FC escaping. Known captured frames validated the CRC implementation.
//    FD 00 00 DB A7 FB 02 F8 1A.
// 5. Earlier low-level observations saw raw 0xF8 around known positive
//    prefixes all the way to complete 32-bit device addresses.
// 6. v0.7.41 replayed the complete captured hm485d branch while A/B were
//    still wired with the wrong polarity. All 73 probes produced only 0x00.
//    This result is now INVALID as a protocol conclusion because the later
//    electrical tests proved the bus polarity was wrong for active TX/RX.
// 7. v0.7.42b/.43 isolated that 0x00 electrically. With A/B disconnected
//    it vanished; GPIO35 showed a long invalid level rather than a UART
//    discovery byte. After swapping A/B, the normal active read-only scan
//    works completely: TYPE, SERIAL, FW, EEPROM and STATUS replies are valid
//    for all three known devices. Therefore the current A/B polarity and the
//    normal bidirectional RS485 path are now proven.
// 8. v0.7.44 deliberately returns to the deterministic captured hm485d
//    prefix replay, now with the CORRECT proven A/B polarity. It returned
//    valid F8/F0 positive reactions and 00 negative reactions all the way to
//    a known full-address leaf. v0.7.45 replaced replay with the native full tree.
//    Any non-zero raw reaction is logged; F8 remains counted separately.
//
// v0.7.45 DISCOVERY MILESTONE (PROVEN)
// ------------------------------------
// - Replace the deterministic DBA7 replay with the complete native hm485d
//   32-bit prefix/tree traversal.
// - Start at address 00000000 with validBits=1 and 3 tries per negative prefix.
// - Use CTRL=((validBits-1)<<3)|0x03 exactly like HM485_Protocol.pm.
// - Decide each probe only from the FIRST raw RX byte: 00 = negative; any
//   non-zero byte (normally F8, occasionally observed F0) = positive.
// - Preserve the 20 ms hm485d discovery timeout/step cadence and 600 ms
//   continuous idle guard before starting.
// - On validBits==33, record the full 32-bit device address and continue the
//   tree exactly as hm485d does.
// - After discovery finishes, automatically run the proven READ-ONLY
//   TYPE/SERIAL/FW/EEPROM/STATUS scan against the discovered address list.
// - v0.7.46 retires the fixed TEST_DEVICES runtime fallback. Native discovery
//   is now the normal boot path and the authoritative live address source.

// v0.7.46 CLEANUP / NORMAL-OPERATION STEP
// ----------------------------------------
// - Native discovery starts automatically at boot after 600 ms continuous idle.
// - Normal Serial output is compact: start, FOUND addresses, finish summary,
//   TYPE, SERIAL and FW. Per-prefix discovery, EEPROM dumps and per-channel
//   STATUS lines are disabled by default to reduce jitter and console flooding.
// - Detailed discovery/file and scan logging can be re-enabled with the
//   compile-time flags DISCOVERY_VERBOSE_* / SCAN_VERBOSE_*.
// - EEPROM and STATUS are still queried and processed; only their Serial
//   presentation is quiet. MQTT/state processing remains unchanged.
// - Runtime TEST_DEVICES fallback is retired; discovery results are authoritative.
//
// v0.7.50a POLISH / OPERATIONS
// ---------------------------
// - Persistent German/English selection for the primary web UI. Language never
//   changes MQTT topics, serial-number based object identity or HA unique_id.
// - Common top navigation reduces duplicated buttons and separates normal
//   operation from diagnostics/firmware pages.
// - Authenticated backup export/import for gateway configuration and currently
//   known device metadata (friendly names, profiles, inversion). Backup contains
//   credentials and therefore must be handled as a secret. Discovery remains
//   authoritative after restore.
//
// v0.8.0 OUTPUT CONTROL MILESTONE
// --------------------------------
// - First deliberately write-capable release. The proven v0.7.51 read path,
//   native discovery, RAW RX ONLY hard inhibit and address-conflict guard remain.
// - HMW-IO-12-Sw14-DR digital outputs can be exposed as HA/MQTT Switch or Button.
// - Wire command confirmed from passive FHEM captures: 73 <bus-channel> <value16>,
//   with OFF=0x0000 and ON=0x03FF. Device INFO_LEVEL reply is 69 <ch> <value>.
// - Output writes use bus-idle arbitration and up to three attempts total. A matching
//   INFO_LEVEL reply confirms state; an ACK-only reply is accepted and followed by
//   a status refresh. Failed writes never publish an optimistic state.
// - Button profile supports configurable pulse duration and resting state OFF/ON.
// - Analog/frequency 0..1023 encoding has been observed but is intentionally NOT
//   exposed for writing yet. EEPROM writes remain disabled.
// - Coexistence with another central (for example FHEM 00000001 while this gateway
//   uses 00000002) remains experimental and must be tested carefully.
//
// v0.8.1 WRITE/ACK FIX
// --------------------
// - Fix critical output transaction bug: an active output write no longer blocks
//   sendAck(). Device INFO_LEVEL I-frames are ACKed immediately in the normal
//   parser before output application logic runs. This prevents the device from
//   retransmitting the same 0x69 frame after the missing ACK timeout.
// - AUTO now resolves to OUTPUT_SWITCH for physically known digital outputs.
//   Existing persisted AUTO profiles therefore become switches without NVS migration.

// GENERAL PROJECT ROADMAP (KEEP THIS SECTION IN FUTURE VERSIONS)
// --------------------------------------------------------------
// DISCOVERY / BUS CORE
// [x] Prove electrical TX/RX path and correct A/B polarity; active reads now work.
// [x] DST+CTRL prefix/valid-bit mapping proven by native full-tree discovery.
// [x] Positive discovery reaction: first RX byte != 0 (hm485d semantics; v0.7.44 verified F8/F0).
// [x] v0.7.45: native hm485d-compatible 32-bit discovery tree implemented.
// [x] v0.7.46: discovered addresses exclusively drive scan/poll; fixed runtime list retired.
// [x] Native discovery automatically hands off to TYPE/SERIAL/FW/EEPROM/STATUS scan.
// [ ] Detect devices appearing/disappearing and expose online/last-seen state.
//
// PASSIVE OPERATION / LEARNING
// [ ] Preserve passive event snooping as the preferred fast state-update path.
// [ ] Passively recognize previously unknown device addresses seen on the bus.
// [ ] Learn/validate channel activity and useful telegram types from traffic.
// [ ] Keep periodic active read-only polling only as a safety/fallback refresh.
// [ ] Keep permanent RAW RX ONLY hard-TX-inhibit mode in every future build.
//
// DEVICE / PORT PROFILE MODEL
// [ ] Build semantic per-channel profiles on top of physical HM485 channels.
// [ ] Profiles planned/needed include at least: Window/door contact, shutter/
//     roller shutter (cover), alarm/contact, switch/light, push button, generic
//     binary input, analog/frequency sensor and generic/raw fallback.
// [ ] Where possible derive electrical/channel capability automatically from
//     device type + EEPROM; allow user override of semantic purpose.
// [ ] Map profiles to correct Home Assistant entity domain/device_class, e.g.
//     binary_sensor, switch, cover, sensor rather than exposing raw channels.
//
// NAMING / CONFIGURATION / PERSISTENCE
// [ ] User-editable friendly name for every device.
// [x] v0.7.47: user-editable friendly name and semantic profile per channel/port.
// [ ] Store names, profile selections, discovered-device metadata and relevant
//     gateway settings persistently in ESP32 NVS (Preferences).
// [ ] Keep configuration across reboot/OTA; design versioned/migratable NVS
//     schema before large numbers of installations/config entries exist.
// [x] v0.7.47: web UI for device/channel naming and profile assignment.
// [x] v0.7.48: per-channel NO/NC inversion, status page, configurable own address.
// [x] v0.7.49: passive address-1 detection + duplicate-own-address TX lock; status shows inversion.
// [x] v0.7.50a: persistent German / English web UI selection.
// [x] v0.7.50a: authenticated config/profile backup export + restore import.
//
// MQTT / HOME ASSISTANT
// [ ] Retained state topics + LWT/availability for gateway and devices.
// [ ] Generate HA MQTT Discovery from the semantic channel profile and names.
// [ ] Update HA immediately from passive bus events; polling is fallback only.
// [ ] Republish discovery/state after MQTT reconnect without duplicate entities.
// [ ] Keep stable unique_ids even if friendly names are changed later.
//
// WRITE / CONTROL - ONLY AFTER READ PATH IS PROVEN
// [ ] Stay READ-ONLY during discovery/profile development.
// [ ] Later add explicit safe switching/control for supported output profiles.
// [ ] Add cover/shutter command semantics only after telegrams are understood.
// [ ] EEPROM writes/configuration only with explicit safeguards, verification,
//     backups and no accidental writes during discovery or passive learning.
// [ ] Separate read-only diagnostics from write-capable operating mode clearly.
//
// PLATFORM / OPERATIONS (ALREADY PRESENT; MUST NOT REGRESS)
// [ ] Ethernet primary on WT32-ETH01; Wi-Fi fallback/setup AP.
// [ ] Web configuration in NVS, HTTP Basic Auth, diagnostics/log page.
// [ ] MQTT, Home Assistant Discovery, retained states and gateway diagnostics.
// [ ] Web OTA firmware update; keep a recoverable serial-flash path.
// [ ] Preserve bus statistics: RX/TX, CRC errors, timeouts, retries, last-seen.
// [ ] Keep experimental/research notes in source until native discovery and
//     stable production operation are proven, so findings are not forgotten.
//
// ============================================================
// HM485 Gateway v0.7.8
//
// Base: v0.5.3a HM485 READ-ONLY core
//
// New:
// - v0.7.46: native discovery runs automatically at boot.
// - Quiet normal logging: discovery probe-by-probe output disabled; status and
//   EEPROM data no longer flood Serial. TYPE/SERIAL/FW remain visible.
// - Fixed TEST_DEVICES list retired from runtime device selection.
//   - WiFi station + fallback setup AP
//   - Web configuration stored in NVS
//   - MQTT + retained states + LWT
//   - Home Assistant MQTT Discovery
//   - Passive HM485 status/event snooping -> immediate MQTT
//   - Periodic read-only status refresh (5 min safety net)
//   - Gateway itself appears as a Home Assistant MQTT device
//   - Gateway diagnostics: firmware, IP, RSSI, uptime, devices,
//     RX/TX frames, CRC errors, timeouts and retries
//   - WT32-ETH01 Ethernet primary + WiFi fallback
//   - Web firmware update (OTA)
//   - HTTP Basic Auth for configuration / OTA
//   - Protected Web Log / Diagnosis page
//   - Optional raw HM485 RX/TX logging in RAM
//   - Native HM485 Discovery diagnostics
//   - Discovery probes grouped as xN
//   - Reactions / CRC errors / incomplete frames correlated to last probe
//   - WT32 sends no HM485 requests while analyzer is active
//
// WT32-ETH01 HM485 wiring:
// GPIO35 = RX <- RO
// GPIO17 = TX -> DI
// GPIO33 = DIR -> RTS / DE+/RE
//
// WT32 Ethernet/LAN8720:
// GPIO16 = PHY oscillator enable
// GPIO18 = MDIO
// GPIO23 = MDC
// GPIO0  = 50 MHz REFCLK input
//
// ESP HM485 address : 00000002
// FHEM central      : 00000001
//
// IMPORTANT:
//   HM485 is still READ ONLY.
//   No output switching and no EEPROM writes.
// ============================================================

// v0.8.2:
// - Data-driven device profile registry for official HMW and selected HBW devices.
// - Explicit support state: tested / known-untested / experimental.
// - Untested/HBW profiles are recognition-only by default: no write enable.
// - Existing tested HMW-IO-12-Sw14-DR write path remains enabled by default.
// v0.9.2:
// - Passive discovery: a CRC-valid frame from a previously unknown device address
//   (> 0x000000FF) schedules automatic identification at the next safe bus-idle slot.
// - Optional experimental runtime writes for documented classic/HBW switch actors.
// v0.9.3g2: HBW-Sen-EP (0x84) real/source-verified counter payload is
// 69 <channel> <uint16_be>. Counter channels are exposed to HA as numeric
// sensors with state_class=total_increasing and unit 'impulses'. Active polling
// remains disabled for now; passive broadcasts are sufficient for this test.
// v0.9.3g1: compile-order fix: forgetDevice() moved behind transaction
// globals and MQTT forward declarations. No functional changes.
// v0.9.3g: persistent device registry + boot restore/verification, explicit
// 'forget device' action with retained HA discovery cleanup, and extended ESP32
// system diagnostics (chip/RAM/flash/sketch/reset reason). Existing hm485meta
// records are auto-migrated into the registry by enumerating NVS once.
// v0.9.3f4: HBW MQTT resync fix. Every complete 0x41 identity announcement
// now refreshes HA discovery/output subscriptions even when metadata is unchanged.
// A completed targeted HBW status poll republishes retained states as a final
// synchronization pass. This fixes stale HA state and missing /set subscription
// after gateway/HBW restart ordering.
// v0.9.3f3: compile fix: forward declaration of
// processPassiveIdentityStatusPoll() now matches its static definition.
// v0.9.3f2: every HBW 0x41 identity/boot announcement now queues a targeted
// one-shot status poll for that device when activeReadSafe=true. This also
// runs when identity metadata is unchanged, fixing stale HA states after an
// HBW-LC-Sw8 reboot. The queued poll waits for a safe idle bus slot.
// v0.9.3f1: compile/link fix for Arduino auto-prototype const mismatch in
// runtimePollDeviceCount()/runtimePollDeviceAddress(). No protocol changes.
// v0.9.3f: status polling now iterates the runtime device database instead of
// native-discovery-only addresses, so passive HBW devices can be polled when
// their profile explicitly marks active reads safe. HBW-LC-Sw8 is polled via
// 53 ch immediately after its 0x41 identity announcement. HBW-Sen-EP (0x84)
// remains passive-only with eight UINT24 counter channels prepared for testing.
// v0.9.3e: HBW-LC-Sw8 original source verified: LEVEL_SET 78 ch 00/C8,
// LEVEL_GET 53 ch, INFO_LEVEL 69 ch level 00. Active polling enabled.
// 0.9.3d's incorrect 0x69 write path has been removed.
// v0.9.3d: HBW-LC-Sw8 real protocol: 69 ch 00/C8, ACK confirms write;
// overview now shows known state values / nominal channel count.
// v0.9.3c: nominal HBW channel count is independent of active-read safety;
// fixes Web UI channel count and MQTT command subscriptions for HBW actors.
// v0.9.3b: HBW 0x41 identity is learned only in the existing passive
// broadcast path; native discovery remains byte-for-byte/timing unchanged.
// v0.9.3: real HBW support: HBW-1W-T10 (0x81) temperatures and
// HBW-LC-Sw8 (0x83) eight switch channels enabled for hardware verification.
//   Disabled by default and deliberately separate from EEPROM/config support.
//
static constexpr const char *FW_VERSION = "0.9.3g2";

// ============================================================
// Hardware / HM485
// ============================================================

static constexpr int HM485_RX_PIN  = 35;
static constexpr int HM485_TX_PIN  = 17;
static constexpr int HM485_DIR_PIN = 33;
static constexpr uint32_t HM485_BAUD = 19200;

static constexpr uint32_t DEFAULT_LOCAL_ADDRESS = 0x00000001;
static constexpr uint32_t FHEM_ADDRESS  = 0x00000001;

// v0.7.47: no fixed HM485 device addresses are compiled into normal runtime.
// The live device list comes exclusively from native HM485 discovery.

static constexpr uint8_t FRAME_START_LONG  = 0xFD;
static constexpr uint8_t FRAME_START_SHORT = 0xFE;
static constexpr uint8_t ESCAPE_CHAR       = 0xFC;

static constexpr uint32_t BUS_IDLE_US             = 10000;
static constexpr uint32_t RESPONSE_TIMEOUT_MS     = 500;
static constexpr uint32_t REQUEST_GAP_MS          = 100;
static constexpr uint8_t  MAX_REQUEST_RETRIES     = 2;
static constexpr uint32_t STATUS_REFRESH_MS       = 300000;
static constexpr uint32_t MQTT_RECONNECT_MS       = 5000;
static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static constexpr uint32_t ETHERNET_FALLBACK_DELAY_MS = 8000;

// v0.7.49 address-collision guard. On every boot the gateway listens only
// before its first HM485 transmission. A source address equal to our own is
// latched as a conflict and keeps DE/TX disabled until the address is changed
// and the gateway is rebooted. Address 00000001 is also observed separately
// so parallel-test installations can see whether the regular central address
// is already active on the bus.
static constexpr uint32_t ADDRESS_GUARD_BOOT_LISTEN_MS = 8000;
static constexpr uint32_t ADDRESS_GUARD_OWN_TX_SUPPRESS_MS = 100;

// v0.7.47: normal operation is intentionally quiet. Detailed per-probe
// discovery logging was invaluable during reverse engineering, but Serial I/O
// and String formatting add jitter. Enable only for diagnostics.
static constexpr bool DISCOVERY_VERBOSE_SERIAL = false;
static constexpr bool DISCOVERY_VERBOSE_FILE   = false;
static constexpr bool SCAN_VERBOSE_STATUS      = false;
static constexpr bool SCAN_VERBOSE_EEPROM      = false;

static constexpr uint8_t  MAX_DEVICES      = 16;
static constexpr uint8_t  MAX_CHANNELS     = 32;
static constexpr uint16_t EEPROM_CACHE_SIZE = 64;

// ============================================================
// Network / MQTT configuration
// ============================================================

struct GatewayConfig
{
  String wifiSsid;
  String wifiPassword;

  String mqttHost;
  uint16_t mqttPort = 1883;
  String mqttUser;
  String mqttPassword;

  String mqttClientId = "hm485-gateway";
  String mqttBaseTopic = "hm485";
  String discoveryPrefix = "homeassistant";
  bool discoveryEnabled = true;

  // v0.9.2: allow runtime writes for documented but locally untested actuators.
  // Default OFF. EEPROM/config writes remain restricted independently.
  bool experimentalWritesEnabled = false;

  String hostname = "hm485-gateway";

  // HM485 address of this gateway/central. 00000001 is the regular central
  // address. Other addresses are intended for parallel operation/tests.
  uint32_t localAddress = DEFAULT_LOCAL_ADDRESS;

  // Web UI language. Values: "de" or "en".
  String uiLanguage = "de";

  // HTTP Basic Auth for configuration/OTA.
  String webUser = "admin";
  String webPassword = "hm485setup";
};

GatewayConfig config;

bool setupApActive = false;
String setupApSsid;

bool ethernetStarted = false;
bool wifiFallbackStarted = false;
bool lastEthernetReady = false;
uint32_t ethernetStartedMs = 0;

uint32_t wifiConnectStartedMs = 0;
uint32_t lastMqttReconnectMs = 0;
uint32_t lastGatewayDiagnosticsMs = 0;
static constexpr uint32_t GATEWAY_DIAGNOSTICS_MS = 30000;

// ============================================================
// HM485 address conflict protection (v0.7.49)
// ============================================================

bool addressGuardBootActive = false;
bool addressConflictLocked = false;
bool centralAddressOneSeen = false;

// Must be declared before web handlers that reference it.
static bool rawRxOnlyMode = false;
uint32_t addressGuardStartedMs = 0;
uint32_t addressConflictLastSeenMs = 0;
uint32_t centralAddressOneLastSeenMs = 0;
uint32_t lastLocalTxCompletedMs = 0;
uint32_t addressConflictSender = 0;

// v0.9.2 passive discovery. Unknown CRC-valid device senders are added as
// candidates and trigger a normal metadata scan once the bus is safely idle.
bool passiveDiscoveryScanPending = false;
uint32_t passiveDiscoveryLastCandidate = 0;

// ============================================================
// Channel model
// ============================================================

enum class ChannelType : uint8_t
{
  UNKNOWN,
  CONTACT,
  DIGITAL_OUTPUT,
  DIGITAL_ANALOG_OUTPUT,
  DIGITAL_INPUT,
  DIGITAL_FREQUENCY_INPUT,
  DIGITAL_ANALOG_INPUT,
  INPUT_OUTPUT,
  COVER,
  DIMMER,
  TEMPERATURE,
  COUNTER,
  KEY_INPUT,
  MOTION,
  RGB
};

enum class ChannelBehaviour : uint8_t
{
  UNKNOWN,
  CONTACT,
  DIGITAL_OUTPUT,
  ANALOG_OUTPUT,
  DIGITAL_INPUT,
  FREQUENCY_INPUT,
  ANALOG_INPUT
};

enum class SemanticProfile : uint8_t
{
  AUTO = 0,
  WINDOW,
  DOOR,
  ALARM,
  CONTACT,
  BINARY_INPUT,
  ANALOG_SENSOR,
  FREQUENCY_SENSOR,
  OUTPUT_MONITOR,
  SHUTTER_MONITOR,
  RAW_SENSOR,
  OUTPUT_SWITCH,
  OUTPUT_BUTTON,
  ANALOG_VOLTAGE_RAW,
  ANALOG_CURRENT_RAW,
  ANALOG_POWER_RAW,
  ANALOG_TEMPERATURE_RAW,
  ANALOG_RESISTANCE_RAW,
  ANALOG_PRESSURE_RAW
};

enum class ValueEncoding : uint8_t
{
  UNKNOWN,
  BOOLEAN_1BYTE,
  BOOLEAN_2BYTE,
  UINT16,
  UINT24
};

enum class PulseValueState : uint8_t
{
  UNAVAILABLE,
  VALID,
  SPECIAL
};

struct BehaviourEeprom
{
  bool available;
  uint16_t byteAddress;
  uint8_t firstBit;
  uint8_t bitStep;
};

struct ChannelProfile
{
  uint8_t first;
  uint8_t count;
  ChannelType type;
  ValueEncoding infoLevelEncoding;
  bool configurable;
  ChannelBehaviour defaultBehaviour;
  BehaviourEeprom behaviourEeprom;
};

enum class DeviceSupport : uint8_t
{
  TESTED,          // Real hardware tested with this gateway
  KNOWN_UNTESTED,  // Profile documented, but no real hardware test here yet
  EXPERIMENTAL     // Homebrew/partial profile; passive/basic recognition only
};

struct DeviceProfile
{
  uint16_t deviceType;
  const char *model;
  const char *family;
  uint16_t eepromSize;
  const ChannelProfile *channels;
  uint8_t channelGroupCount;
  uint8_t nominalChannelCount;
  DeviceSupport support;
  bool activeReadSafe;
  bool writeEnabled;
};

// ============================================================
// Profiles
// ============================================================

static constexpr ChannelProfile PROFILE_SEN_SC_12[] =
{
  { 0, 12, ChannelType::CONTACT, ValueEncoding::BOOLEAN_1BYTE, false,
    ChannelBehaviour::CONTACT, { false, 0, 0, 0 } }
};

static constexpr ChannelProfile PROFILE_IO12_SW14[] =
{
  { 0, 6,  ChannelType::DIGITAL_OUTPUT,          ValueEncoding::BOOLEAN_2BYTE, false, ChannelBehaviour::DIGITAL_OUTPUT, { false, 0, 0, 0 } },
  { 6, 8,  ChannelType::DIGITAL_ANALOG_OUTPUT,   ValueEncoding::UINT16,        true,  ChannelBehaviour::DIGITAL_OUTPUT, { true, 0x0007, 0, 1 } },
  { 14, 6, ChannelType::DIGITAL_FREQUENCY_INPUT, ValueEncoding::BOOLEAN_2BYTE, true,  ChannelBehaviour::DIGITAL_INPUT,  { true, 0x0009, 0, 1 } },
  { 20, 6, ChannelType::DIGITAL_ANALOG_INPUT,    ValueEncoding::UINT16,        true,  ChannelBehaviour::DIGITAL_INPUT,  { true, 0x0008, 0, 1 } }
};

// Generic/untested profile descriptions.  They intentionally do NOT enable
// active writing.  The purpose is recognition, sane HA defaults where safe,
// and a data-driven base for later verified implementations.
static constexpr ChannelProfile PROFILE_IO4_FM[] =
{
  { 0, 4, ChannelType::INPUT_OUTPUT, ValueEncoding::BOOLEAN_1BYTE, true,
    ChannelBehaviour::UNKNOWN, { false, 0, 0, 0 } }
};

static constexpr ChannelProfile PROFILE_SW2[] =
{
  { 0, 2, ChannelType::DIGITAL_OUTPUT, ValueEncoding::BOOLEAN_1BYTE, false,
    ChannelBehaviour::DIGITAL_OUTPUT, { false, 0, 0, 0 } }
};

static constexpr ChannelProfile PROFILE_IO12_SW7[] =
{
  { 0, 7,  ChannelType::DIGITAL_OUTPUT, ValueEncoding::BOOLEAN_1BYTE, false, ChannelBehaviour::DIGITAL_OUTPUT, { false, 0, 0, 0 } },
  { 7, 12, ChannelType::DIGITAL_INPUT,  ValueEncoding::UINT16, false, ChannelBehaviour::DIGITAL_INPUT,  { false, 0, 0, 0 } }
};

static constexpr ChannelProfile PROFILE_DIM1[] =
{
  { 0, 1, ChannelType::DIMMER, ValueEncoding::UINT16, false,
    ChannelBehaviour::UNKNOWN, { false, 0, 0, 0 } }
};

static constexpr ChannelProfile PROFILE_BL1[] =
{
  { 0, 1, ChannelType::COVER, ValueEncoding::UINT16, false,
    ChannelBehaviour::UNKNOWN, { false, 0, 0, 0 } }
};

static constexpr ChannelProfile PROFILE_IO12_FM[] =
{
  { 0, 12, ChannelType::INPUT_OUTPUT, ValueEncoding::UINT16, true,
    ChannelBehaviour::UNKNOWN, { false, 0, 0, 0 } }
};

// HBW-1W-T10, real hardware verified:
 // INFO_LEVEL: 69 <channel 0..9> <temperature MSB> <temperature LSB>
 // temperature is signed int16 big-endian in 0.01 degC.
 // 0x954D = -27315 = -273.15 degC marks an unassigned/missing 1-Wire sensor.
static constexpr ChannelProfile PROFILE_HBW_TEMP10[] =
{
  { 0, 10, ChannelType::TEMPERATURE, ValueEncoding::UINT16, false, ChannelBehaviour::UNKNOWN, { false, 0, 0, 0 } }
};
static constexpr ChannelProfile PROFILE_HBW_BL4[] =
{
  { 0, 4, ChannelType::COVER, ValueEncoding::UINT16, false, ChannelBehaviour::UNKNOWN, { false, 0, 0, 0 } }
};
static constexpr ChannelProfile PROFILE_HBW_BL8[] =
{
  { 0, 8, ChannelType::COVER, ValueEncoding::UINT16, false, ChannelBehaviour::UNKNOWN, { false, 0, 0, 0 } }
};
static constexpr ChannelProfile PROFILE_HBW_SW8[] =
{
  { 0, 8, ChannelType::DIGITAL_OUTPUT, ValueEncoding::BOOLEAN_1BYTE, false, ChannelBehaviour::DIGITAL_OUTPUT, { false, 0, 0, 0 } }
};
static constexpr ChannelProfile PROFILE_HBW_SW12[] =
{
  { 0, 12, ChannelType::DIGITAL_OUTPUT, ValueEncoding::BOOLEAN_1BYTE, false, ChannelBehaviour::DIGITAL_OUTPUT, { false, 0, 0, 0 } }
};
// HBW-Sen-EP (0x84): eight S0/pulse counters.
// Real bus traffic and the original source confirm a 16-bit counter value:
//   INFO_LEVEL: 69 <channel> <value_hi> <value_lo>
// Keep active polling disabled until separately tested on the real module.
static constexpr ChannelProfile PROFILE_HBW_COUNTER8[] =
{
  { 0, 8, ChannelType::COUNTER, ValueEncoding::UINT16, false, ChannelBehaviour::UNKNOWN, { false, 0, 0, 0 } }
};
static constexpr ChannelProfile PROFILE_HBW_KEY8[] =
{
  { 0, 8, ChannelType::KEY_INPUT, ValueEncoding::BOOLEAN_1BYTE, false, ChannelBehaviour::DIGITAL_INPUT, { false, 0, 0, 0 } }
};
static constexpr ChannelProfile PROFILE_HBW_KEY12[] =
{
  { 0, 12, ChannelType::KEY_INPUT, ValueEncoding::BOOLEAN_1BYTE, false, ChannelBehaviour::DIGITAL_INPUT, { false, 0, 0, 0 }
  }
};
static constexpr ChannelProfile PROFILE_HBW_MOTION2[] =
{
  { 0, 2, ChannelType::MOTION, ValueEncoding::BOOLEAN_1BYTE, false, ChannelBehaviour::DIGITAL_INPUT, { false, 0, 0, 0 } }
};
static constexpr ChannelProfile PROFILE_HBW_RGB[] =
{
  { 0, 3, ChannelType::RGB, ValueEncoding::UINT16, false, ChannelBehaviour::UNKNOWN, { false, 0, 0, 0 } }
};

static constexpr DeviceProfile DEVICE_PROFILES[] =
{
  // Official HMW device IDs from the classic HM485 protocol documentation.
  { 0x0010, "HMW-IO-4-FM",       "HMW", 1024, PROFILE_IO4_FM,      1,  4, DeviceSupport::KNOWN_UNTESTED, false, false },
  { 0x0011, "HMW-LC-Sw2-DR",     "HMW", 1024, PROFILE_SW2,         1,  2, DeviceSupport::KNOWN_UNTESTED, false, true  },
  { 0x0012, "HMW-IO-12-Sw7-DR",  "HMW", 1024, PROFILE_IO12_SW7,    2, 19, DeviceSupport::KNOWN_UNTESTED, false, true  },
  { 0x0014, "HMW-LC-Dim1L-DR",   "HMW", 1024, PROFILE_DIM1,        1,  1, DeviceSupport::KNOWN_UNTESTED, false, false },
  { 0x0015, "HMW-LC-Bl1-DR",     "HMW", 1024, PROFILE_BL1,         1,  1, DeviceSupport::KNOWN_UNTESTED, false, false },
  { 0x0016, "HMW-IO-SR-FM",      "HMW", 1024, nullptr,             0,  0, DeviceSupport::KNOWN_UNTESTED, false, false },
  { 0x0019, "HMW-Sen-SC-12-DR",  "HMW", 1024, PROFILE_SEN_SC_12,  1, 12, DeviceSupport::TESTED,         true,  false },
  { 0x001A, "HMW-Sen-SC-12-FM",  "HMW", 1024, PROFILE_SEN_SC_12,  1, 12, DeviceSupport::KNOWN_UNTESTED, false, false },
  { 0x001B, "HMW-IO-12-FM",      "HMW", 1024, PROFILE_IO12_FM,     1, 12, DeviceSupport::KNOWN_UNTESTED, false, false },
  { 0x001C, "HMW-IO-12-Sw14-DR", "HMW", 1024, PROFILE_IO12_SW14,   4, 26, DeviceSupport::TESTED,         true,  true  },

  // HBW/Homebrew IDs from the FHEM Wired overview. These entries are deliberately
  // passive/experimental until a real device or a verified XML profile is tested.
  { 0x0081, "HBW-1W-T10",       "HBW", 0, PROFILE_HBW_TEMP10,   1, 10, DeviceSupport::TESTED,         false, false },
  { 0x0082, "HBW-LC-Bl-4",      "HBW", 0, PROFILE_HBW_BL4,      1,  4, DeviceSupport::EXPERIMENTAL, false, false },
  { 0x0092, "HBW-LC-Bl-8",      "HBW", 0, PROFILE_HBW_BL8,      1,  8, DeviceSupport::EXPERIMENTAL, false, false },
  { 0x0083, "HBW-LC-Sw8",       "HBW", 0, PROFILE_HBW_SW8,      1,  8, DeviceSupport::KNOWN_UNTESTED, true,  true  },
  { 0x0093, "HBW-LC-Sw-12",     "HBW", 0, PROFILE_HBW_SW12,     1, 12, DeviceSupport::EXPERIMENTAL, false, true  },
  { 0x0084, "HBW-Sen-EP",       "HBW", 0, PROFILE_HBW_COUNTER8, 1,  8, DeviceSupport::KNOWN_UNTESTED, false, false },
  { 0x0085, "HBW-Sen-KEY",      "HBW", 0, PROFILE_HBW_KEY8,     1,  8, DeviceSupport::EXPERIMENTAL, false, false },
  { 0x0086, "HBW-Sen-SC8",      "HBW", 0, PROFILE_HBW_KEY8,     1,  8, DeviceSupport::EXPERIMENTAL, false, false },
  { 0x0095, "HBW-Sen-Key-12",   "HBW", 0, PROFILE_HBW_KEY12,    1, 12, DeviceSupport::EXPERIMENTAL, false, false },
  { 0x0091, "HBW-Sec-MDIR-2",   "HBW", 0, PROFILE_HBW_MOTION2,  1,  2, DeviceSupport::EXPERIMENTAL, false, false },
  { 0x00A0, "HBW-LC-RGB1-CV",   "HBW", 0, PROFILE_HBW_RGB,      1,  3, DeviceSupport::EXPERIMENTAL, false, false }
};

static constexpr size_t DEVICE_PROFILE_COUNT =
  sizeof(DEVICE_PROFILES) / sizeof(DEVICE_PROFILES[0]);

// ============================================================
// Runtime devices
// ============================================================

struct ChannelState
{
  bool known = false;
  uint32_t value = 0;
  uint8_t valueSize = 0;

  ChannelBehaviour behaviour = ChannelBehaviour::UNKNOWN;
  bool behaviourKnown = false;

  uint32_t lastUpdate = 0;
};

struct HM485Device
{
  bool used = false;
  uint32_t address = 0;

  bool typeKnown = false;
  uint16_t deviceType = 0;
  const DeviceProfile *profile = nullptr;

  bool serialKnown = false;
  char serialNumber[16] = {};

  bool firmwareKnown = false;
  uint16_t firmware = 0;

  uint8_t eeprom[EEPROM_CACHE_SIZE] = {};
  bool eepromValid[EEPROM_CACHE_SIZE] = {};

  ChannelState channel[MAX_CHANNELS];

  // User metadata is keyed by the dynamically discovered HM485 address and
  // persisted in NVS. It never replaces discovery; it only decorates it.
  bool metadataLoaded = false;
  char friendlyName[32] = {};
  char channelName[MAX_CHANNELS][24] = {};
  SemanticProfile semanticProfile[MAX_CHANNELS] = {};
  // Per-channel logical inversion for NO/NC contacts. This changes the
  // logical MQTT/HA state only; the raw HM485 value remains untouched.
  bool invertLogic[MAX_CHANNELS] = {};
  // v0.8.0 output-button settings. Duration is stored in milliseconds.
  uint32_t buttonPulseMs[MAX_CHANNELS] = {};
  bool buttonRestOn[MAX_CHANNELS] = {};

  uint32_t lastSeen = 0;

  // Restored from persistent metadata at gateway boot. This does NOT mean the
  // device has been seen on the bus in the current boot.
  bool restoredFromNvs = false;
};

HM485Device devices[MAX_DEVICES];

// Forward declaration: used by the web profile handlers before the
// Device DB implementation appears later in this translation unit.
HM485Device *getDevice(uint32_t address, bool create = true);

// Forward declarations used by persistent metadata/web profile helpers.
String hexAddress(uint32_t address);
uint8_t busToHmWiredChannel(uint8_t busChannel);
const DeviceProfile *findDeviceProfile(uint16_t type);
const ChannelProfile *findChannelProfile(const DeviceProfile *device, uint8_t busChannel);
const char *channelTypeName(ChannelType type);
const char *behaviourName(ChannelBehaviour behaviour);
uint8_t deviceChannelCount(const HM485Device *device);

// ============================================================
// Persistent device metadata / semantic channel profiles
// ============================================================

static constexpr uint16_t DEVICE_METADATA_SCHEMA = 3;
static constexpr const char *DEVICE_METADATA_NAMESPACE = "hm485meta";

static constexpr uint16_t DEVICE_REGISTRY_SCHEMA = 1;
static constexpr const char *DEVICE_REGISTRY_KEY = "registry";

struct PersistedDeviceRegistry
{
  uint16_t schema = DEVICE_REGISTRY_SCHEMA;
  uint8_t count = 0;
  uint8_t reserved = 0;
  uint32_t address[MAX_DEVICES] = {};
};

static bool initialNativeDiscoveryFinished = false;
static bool bootKnownDeviceVerifyPending = false;
static uint8_t bootKnownDeviceVerifyIndex = 0;

// v0.7.47 on-flash layout. Kept so existing friendly names/profile settings
// survive the schema extension in v0.7.48.
struct PersistedDeviceMetadataV1
{
  uint16_t schema = 1;
  uint16_t cachedType = 0;
  uint16_t cachedFirmware = 0;
  char cachedSerial[16] = {};
  char friendlyName[32] = {};
  char channelName[MAX_CHANNELS][24] = {};
  uint8_t semanticProfile[MAX_CHANNELS] = {};
};

struct PersistedDeviceMetadataV2
{
  uint16_t schema = 2;
  uint16_t cachedType = 0;
  uint16_t cachedFirmware = 0;
  char cachedSerial[16] = {};
  char friendlyName[32] = {};
  char channelName[MAX_CHANNELS][24] = {};
  uint8_t semanticProfile[MAX_CHANNELS] = {};
  uint8_t invertLogic[MAX_CHANNELS] = {};
};

struct PersistedDeviceMetadata
{
  uint16_t schema = DEVICE_METADATA_SCHEMA;
  uint16_t cachedType = 0;
  uint16_t cachedFirmware = 0;
  char cachedSerial[16] = {};
  char friendlyName[32] = {};
  char channelName[MAX_CHANNELS][24] = {};
  uint8_t semanticProfile[MAX_CHANNELS] = {};
  uint8_t invertLogic[MAX_CHANNELS] = {};
  uint32_t buttonPulseMs[MAX_CHANNELS] = {};
  uint8_t buttonRestOn[MAX_CHANNELS] = {};
};

static const char *semanticProfileName(SemanticProfile profile)
{
  switch (profile)
  {
    case SemanticProfile::WINDOW:           return "Fenster";
    case SemanticProfile::DOOR:             return "Tuer";
    case SemanticProfile::ALARM:            return "Alarm";
    case SemanticProfile::CONTACT:          return "Kontakt";
    case SemanticProfile::BINARY_INPUT:     return "Binaereingang";
    case SemanticProfile::ANALOG_SENSOR:    return "Analogsensor";
    case SemanticProfile::FREQUENCY_SENSOR: return "Frequenzsensor";
    case SemanticProfile::OUTPUT_MONITOR:   return "Ausgang (read-only)";
    case SemanticProfile::SHUTTER_MONITOR:  return "Rollladen (read-only)";
    case SemanticProfile::RAW_SENSOR:       return "Raw Sensor";
    case SemanticProfile::OUTPUT_SWITCH:    return "Ausgang Switch";
    case SemanticProfile::OUTPUT_BUTTON:    return "Ausgang Button";
    case SemanticProfile::ANALOG_VOLTAGE_RAW:     return "Spannung (Rohwert)";
    case SemanticProfile::ANALOG_CURRENT_RAW:     return "Strom (Rohwert)";
    case SemanticProfile::ANALOG_POWER_RAW:       return "Leistung (Rohwert)";
    case SemanticProfile::ANALOG_TEMPERATURE_RAW: return "Temperatur (Rohwert)";
    case SemanticProfile::ANALOG_RESISTANCE_RAW:  return "Widerstand (Rohwert)";
    case SemanticProfile::ANALOG_PRESSURE_RAW:    return "Druck (Rohwert)";
    case SemanticProfile::AUTO:
    default:                                return "Auto";
  }
}

static String deviceMetadataKey(uint32_t address)
{
  // NVS keys are limited to 15 characters. Eight hexadecimal digits fit.
  return hexAddress(address);
}

static void copyStringToBuffer(const String &value, char *buffer, size_t size)
{
  if (!buffer || size == 0)
    return;
  String v = value;
  v.trim();
  if (v.length() >= size)
    v.remove(size - 1);
  v.toCharArray(buffer, size);
}

static String effectiveDeviceName(const HM485Device *device)
{
  if (!device)
    return "HM485";
  if (device->friendlyName[0])
    return String(device->friendlyName);
  if (device->profile && device->serialKnown)
    return String(device->profile->model) + " " + String(device->serialNumber);
  if (device->profile)
    return String(device->profile->model) + " " + hexAddress(device->address);
  return String("HM485 ") + hexAddress(device->address);
}

static String effectiveChannelName(const HM485Device *device, uint8_t busChannel)
{
  if (device && busChannel < MAX_CHANNELS && device->channelName[busChannel][0])
    return String(device->channelName[busChannel]);
  return String("Kanal ") + String(busToHmWiredChannel(busChannel));
}

enum class RuntimeWriteProtocol : uint8_t
{
  NONE,
  SET_73_UINT16_03FF, // HMW-IO-12-Sw14-DR: 73 ch 0000/03FF
  SET_78_UINT8_C8     // classic/HBW switch: 78 ch 00/C8
};

// Explicit prototypes: Arduino's automatic prototype generator can place
// prototypes before this enum, which breaks custom return types.
static bool deviceRuntimeWriteAllowed(const HM485Device *device);
static RuntimeWriteProtocol runtimeWriteProtocol(const HM485Device *device, uint8_t busChannel);

static bool deviceRuntimeWriteAllowed(const HM485Device *device)
{
  if (!device || !device->profile || !device->profile->writeEnabled)
    return false;

  if (device->deviceType == 0x001C)
    return true; // locally tested HMW-IO-12-Sw14-DR

  // Real HBW-LC-Sw8 hardware (type 0x0083) has been physically identified.
  // The original HBW source and FHEM device definition confirm LEVEL_SET 0x78
  // with 0x00/0xC8, LEVEL_GET 0x53 and INFO_LEVEL 0x69. Keep the overall
  // support state KNOWN_UNTESTED until the no-debug hardware switching test
  // has confirmed the complete round trip on this physical module.
  if (device->deviceType == 0x0083)
    return true;

  return config.experimentalWritesEnabled;
}

static RuntimeWriteProtocol runtimeWriteProtocol(const HM485Device *device, uint8_t busChannel)
{
  if (!deviceRuntimeWriteAllowed(device) || !device->profile)
    return RuntimeWriteProtocol::NONE;

  const ChannelProfile *cp = findChannelProfile(device->profile, busChannel);
  if (!cp) return RuntimeWriteProtocol::NONE;

  if (device->deviceType == 0x001C &&
      (cp->type == ChannelType::DIGITAL_OUTPUT ||
       (cp->type == ChannelType::DIGITAL_ANALOG_OUTPUT &&
        device->channel[busChannel].behaviour == ChannelBehaviour::DIGITAL_OUTPUT)))
    return RuntimeWriteProtocol::SET_73_UINT16_03FF;

  // HBW-LC-Sw8 source and FHEM definition confirm:
  //   LEVEL_SET:  78 <channel> <00/C8>
  //   LEVEL_GET:  53 <channel>
  //   INFO_LEVEL: 69 <channel> <00/C8> 00
  switch (device->deviceType)
  {
    case 0x0011: // HMW-LC-Sw2-DR
    case 0x0012: // HMW-IO-12-Sw7-DR, relay outputs only
    case 0x0083: // HBW-LC-Sw8, source verified
    case 0x0093: // HBW-LC-Sw-12
      if (cp->type == ChannelType::DIGITAL_OUTPUT)
        return RuntimeWriteProtocol::SET_78_UINT8_C8;
      break;
    default:
      break;
  }

  return RuntimeWriteProtocol::NONE;
}

static SemanticProfile channelSemanticProfile(const HM485Device *device, uint8_t busChannel)
{
  if (!device || busChannel >= MAX_CHANNELS)
    return SemanticProfile::AUTO;

  uint8_t raw = static_cast<uint8_t>(device->semanticProfile[busChannel]);
  if (raw > static_cast<uint8_t>(SemanticProfile::ANALOG_PRESSURE_RAW))
    raw = static_cast<uint8_t>(SemanticProfile::AUTO);

  SemanticProfile configured = static_cast<SemanticProfile>(raw);
  if (configured != SemanticProfile::AUTO)
    return configured;

  // v0.8.1: AUTO means "Switch" for a channel that is physically known to
  // be a digital output. This also upgrades existing persisted AUTO profiles
  // without rewriting NVS. Configurable digital/analog outputs only become a
  // Switch while their EEPROM-derived behaviour is DIGITAL_OUTPUT.
  if (device->profile)
  {
    const ChannelProfile *cp = findChannelProfile(device->profile, busChannel);
    if (cp)
    {
      if (cp->type == ChannelType::DIGITAL_OUTPUT)
        return deviceRuntimeWriteAllowed(device)
          ? SemanticProfile::OUTPUT_SWITCH
          : SemanticProfile::OUTPUT_MONITOR;
      if (cp->type == ChannelType::DIGITAL_ANALOG_OUTPUT &&
          device->channel[busChannel].behaviour == ChannelBehaviour::DIGITAL_OUTPUT)
        return deviceRuntimeWriteAllowed(device)
          ? SemanticProfile::OUTPUT_SWITCH
          : SemanticProfile::OUTPUT_MONITOR;
    }
  }

  return SemanticProfile::AUTO;
}

static const char *semanticDeviceClass(SemanticProfile profile)
{
  switch (profile)
  {
    case SemanticProfile::WINDOW:  return "window";
    case SemanticProfile::DOOR:    return "door";
    case SemanticProfile::ALARM:   return "problem";
    case SemanticProfile::CONTACT: return "opening";
    default:                       return nullptr;
  }
}

static const char *semanticIcon(SemanticProfile profile)
{
  switch (profile)
  {
    case SemanticProfile::ANALOG_VOLTAGE_RAW:     return "mdi:flash";
    case SemanticProfile::ANALOG_CURRENT_RAW:     return "mdi:current-ac";
    case SemanticProfile::ANALOG_POWER_RAW:       return "mdi:lightning-bolt";
    case SemanticProfile::ANALOG_TEMPERATURE_RAW: return "mdi:thermometer";
    case SemanticProfile::ANALOG_RESISTANCE_RAW:  return "mdi:resistor";
    case SemanticProfile::ANALOG_PRESSURE_RAW:    return "mdi:gauge";
    default:                                       return nullptr;
  }
}

static void saveDeviceMetadata(HM485Device *device);
static void restoreKnownDevicesFromNvs();
static void processBootKnownDeviceVerification();
static bool forgetDevice(uint32_t address);

static void applyLoadedMetadata(HM485Device *device,
                                uint16_t cachedType,
                                uint16_t cachedFirmware,
                                const char *cachedSerial,
                                const char *friendlyName,
                                const char channelName[MAX_CHANNELS][24],
                                const uint8_t semanticProfile[MAX_CHANNELS],
                                const uint8_t *invertLogic,
                                const uint32_t *buttonPulseMs = nullptr,
                                const uint8_t *buttonRestOn = nullptr)
{
  memcpy(device->friendlyName, friendlyName, sizeof(device->friendlyName));
  device->friendlyName[sizeof(device->friendlyName) - 1] = 0;
  memcpy(device->channelName, channelName, sizeof(device->channelName));
  for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++)
  {
    device->channelName[ch][sizeof(device->channelName[ch]) - 1] = 0;
    uint8_t raw = semanticProfile[ch];
    device->semanticProfile[ch] = raw <= static_cast<uint8_t>(SemanticProfile::ANALOG_PRESSURE_RAW)
      ? static_cast<SemanticProfile>(raw)
      : SemanticProfile::AUTO;
    device->invertLogic[ch] = invertLogic ? (invertLogic[ch] != 0) : false;
    device->buttonPulseMs[ch] = buttonPulseMs ? buttonPulseMs[ch] : 2000;
    if (device->buttonPulseMs[ch] < 100) device->buttonPulseMs[ch] = 100;
    if (device->buttonPulseMs[ch] > 3600000UL) device->buttonPulseMs[ch] = 3600000UL;
    device->buttonRestOn[ch] = buttonRestOn ? (buttonRestOn[ch] != 0) : false;
  }

  // Cached identity is informational only; live scan remains authoritative.
  if (cachedType)
  {
    device->typeKnown = true;
    device->deviceType = cachedType;
    device->profile = findDeviceProfile(cachedType);
  }
  if (cachedFirmware)
  {
    device->firmwareKnown = true;
    device->firmware = cachedFirmware;
  }
  if (cachedSerial && cachedSerial[0])
  {
    device->serialKnown = true;
    strncpy(device->serialNumber, cachedSerial, sizeof(device->serialNumber) - 1);
    device->serialNumber[sizeof(device->serialNumber) - 1] = 0;
  }
}


static bool validPersistedDeviceKey(const char *key, uint32_t &address)
{
  if (!key || strlen(key) != 8)
    return false;

  for (uint8_t i = 0; i < 8; i++)
    if (!isxdigit((unsigned char)key[i]))
      return false;

  char *end = nullptr;
  address = strtoul(key, &end, 16);
  return end && *end == 0 && address > 0x000000FFUL && address != 0xFFFFFFFFUL;
}

static bool loadDeviceRegistry(PersistedDeviceRegistry &registry)
{
  registry = PersistedDeviceRegistry{};

  Preferences p;
  if (!p.begin(DEVICE_METADATA_NAMESPACE, true))
    return false;

  const size_t len = p.getBytesLength(DEVICE_REGISTRY_KEY);
  bool ok = false;
  if (len == sizeof(PersistedDeviceRegistry))
  {
    p.getBytes(DEVICE_REGISTRY_KEY, &registry, sizeof(registry));
    ok = registry.schema == DEVICE_REGISTRY_SCHEMA && registry.count <= MAX_DEVICES;
  }
  p.end();

  if (!ok)
    registry = PersistedDeviceRegistry{};

  return ok;
}

static void saveDeviceRegistry(const PersistedDeviceRegistry &registry)
{
  Preferences p;
  if (!p.begin(DEVICE_METADATA_NAMESPACE, false))
    return;

  p.putBytes(DEVICE_REGISTRY_KEY, &registry, sizeof(registry));
  p.end();
}

static void registryAddAddress(uint32_t address)
{
  if (address <= 0x000000FFUL || address == 0xFFFFFFFFUL)
    return;

  PersistedDeviceRegistry registry;
  loadDeviceRegistry(registry);

  for (uint8_t i = 0; i < registry.count; i++)
    if (registry.address[i] == address)
      return;

  if (registry.count >= MAX_DEVICES)
    return;

  registry.address[registry.count++] = address;
  saveDeviceRegistry(registry);
}

static void registryRemoveAddress(uint32_t address)
{
  PersistedDeviceRegistry registry;
  if (!loadDeviceRegistry(registry))
    return;

  uint8_t out = 0;
  for (uint8_t i = 0; i < registry.count; i++)
    if (registry.address[i] != address)
      registry.address[out++] = registry.address[i];

  for (uint8_t i = out; i < MAX_DEVICES; i++)
    registry.address[i] = 0;

  if (out != registry.count)
  {
    registry.count = out;
    saveDeviceRegistry(registry);
  }
}

static bool migrateDeviceRegistryFromMetadataKeys(PersistedDeviceRegistry &registry)
{
  registry = PersistedDeviceRegistry{};

  nvs_iterator_t it = nullptr;
  esp_err_t err = nvs_entry_find("nvs", DEVICE_METADATA_NAMESPACE, NVS_TYPE_BLOB, &it);
  while (err == ESP_OK && it != nullptr)
  {
    nvs_entry_info_t info{};
    nvs_entry_info(it, &info);

    uint32_t address = 0;
    if (validPersistedDeviceKey(info.key, address))
    {
      bool duplicate = false;
      for (uint8_t i = 0; i < registry.count; i++)
        if (registry.address[i] == address)
          duplicate = true;

      if (!duplicate && registry.count < MAX_DEVICES)
        registry.address[registry.count++] = address;
    }

    err = nvs_entry_next(&it);
  }
  nvs_release_iterator(it);

  if (registry.count > 0)
    saveDeviceRegistry(registry);

  return registry.count > 0;
}

static void restoreKnownDevicesFromNvs()
{
  PersistedDeviceRegistry registry;
  bool hadRegistry = loadDeviceRegistry(registry);

  if (!hadRegistry)
  {
    migrateDeviceRegistryFromMetadataKeys(registry);
    if (registry.count > 0)
      webLogAdd(String("[NVS] Migrated device registry entries=") + String(registry.count));
  }

  uint8_t restored = 0;
  for (uint8_t i = 0; i < registry.count && i < MAX_DEVICES; i++)
  {
    const uint32_t address = registry.address[i];
    if (address <= 0x000000FFUL || address == 0xFFFFFFFFUL)
      continue;

    HM485Device *d = getDevice(address, true);
    if (!d)
      continue;

    d->restoredFromNvs = true;
    d->lastSeen = 0; // persisted identity != live bus presence
    restored++;
  }

  if (restored > 0)
  {
    bootKnownDeviceVerifyPending = true;
    bootKnownDeviceVerifyIndex = 0;
    String msg = String("[NVS] Restored known devices=") + String(restored);
    Serial.println(msg);
    webLogAdd(msg);
  }
}

static void loadDeviceMetadata(HM485Device *device)
{
  if (!device || device->metadataLoaded)
    return;

  device->metadataLoaded = true;
  String key = deviceMetadataKey(device->address);
  bool migratedV1 = false;

  Preferences metaPrefs;
  if (!metaPrefs.begin(DEVICE_METADATA_NAMESPACE, true))
    return;

  size_t len = metaPrefs.getBytesLength(key.c_str());
  if (len == sizeof(PersistedDeviceMetadata))
  {
    PersistedDeviceMetadata meta;
    metaPrefs.getBytes(key.c_str(), &meta, sizeof(meta));
    if (meta.schema == DEVICE_METADATA_SCHEMA)
    {
      applyLoadedMetadata(device, meta.cachedType, meta.cachedFirmware,
                          meta.cachedSerial, meta.friendlyName,
                          meta.channelName, meta.semanticProfile,
                          meta.invertLogic, meta.buttonPulseMs, meta.buttonRestOn);
    }
  }
  else if (len == sizeof(PersistedDeviceMetadataV2))
  {
    PersistedDeviceMetadataV2 meta;
    metaPrefs.getBytes(key.c_str(), &meta, sizeof(meta));
    if (meta.schema == 2)
    {
      applyLoadedMetadata(device, meta.cachedType, meta.cachedFirmware,
                          meta.cachedSerial, meta.friendlyName,
                          meta.channelName, meta.semanticProfile, meta.invertLogic);
      migratedV1 = true;
    }
  }
  else if (len == sizeof(PersistedDeviceMetadataV1))
  {
    PersistedDeviceMetadataV1 meta;
    metaPrefs.getBytes(key.c_str(), &meta, sizeof(meta));
    if (meta.schema == 1)
    {
      applyLoadedMetadata(device, meta.cachedType, meta.cachedFirmware,
                          meta.cachedSerial, meta.friendlyName,
                          meta.channelName, meta.semanticProfile, nullptr);
      migratedV1 = true;
    }
  }
  metaPrefs.end();

  // Write the new schema only after the read handle is closed. Existing names
  // and semantic profiles are retained; inversion defaults to off.
  if (migratedV1)
    saveDeviceMetadata(device);
}

static void saveDeviceMetadata(HM485Device *device)
{
  if (!device)
    return;

  PersistedDeviceMetadata meta;
  meta.schema = DEVICE_METADATA_SCHEMA;
  meta.cachedType = device->typeKnown ? device->deviceType : 0;
  meta.cachedFirmware = device->firmwareKnown ? device->firmware : 0;
  if (device->serialKnown)
    memcpy(meta.cachedSerial, device->serialNumber, sizeof(meta.cachedSerial));
  memcpy(meta.friendlyName, device->friendlyName, sizeof(meta.friendlyName));
  memcpy(meta.channelName, device->channelName, sizeof(meta.channelName));
  for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++)
  {
    meta.semanticProfile[ch] = static_cast<uint8_t>(channelSemanticProfile(device, ch));
    meta.invertLogic[ch] = device->invertLogic[ch] ? 1 : 0;
    meta.buttonPulseMs[ch] = device->buttonPulseMs[ch] ? device->buttonPulseMs[ch] : 2000;
    meta.buttonRestOn[ch] = device->buttonRestOn[ch] ? 1 : 0;
  }

  String key = deviceMetadataKey(device->address);
  Preferences metaPrefs;
  if (!metaPrefs.begin(DEVICE_METADATA_NAMESPACE, false))
    return;

  // Avoid an unnecessary flash write if the persisted record is identical.
  PersistedDeviceMetadata oldMeta;
  bool same = false;
  size_t len = metaPrefs.getBytesLength(key.c_str());
  if (len == sizeof(oldMeta))
  {
    metaPrefs.getBytes(key.c_str(), &oldMeta, sizeof(oldMeta));
    same = memcmp(&oldMeta, &meta, sizeof(meta)) == 0;
  }
  if (!same)
    metaPrefs.putBytes(key.c_str(), &meta, sizeof(meta));
  metaPrefs.end();

  registryAddAddress(device->address);
}

static void saveAllDeviceMetadata()
{
  for (uint8_t i = 0; i < MAX_DEVICES; i++)
    if (devices[i].used)
      saveDeviceMetadata(&devices[i]);
}


// ============================================================
// Requests / scanners
// ============================================================

enum class RequestType : uint8_t
{
  NONE,
  REQ_DEVICE_TYPE,
  REQ_SERIAL_NUMBER,
  REQ_FIRMWARE,
  REQ_STATUS,
  REQ_EEPROM_READ
};

struct PendingRequest
{
  bool active = false;
  RequestType type = RequestType::NONE;

  uint32_t target = 0;
  uint8_t txCounter = 0;
  uint8_t channel = 0;

  uint16_t eepromAddress = 0;
  uint8_t eepromLength = 0;

  uint32_t sentAt = 0;
};

PendingRequest pending;

// ============================================================
// v0.8.0 output transaction state
// ============================================================
struct OutputWriteTransaction
{
  bool active = false;
  bool waitingAckOrInfo = false;
  bool verifyPending = false;
  uint32_t target = 0;
  uint8_t channel = 0;
  uint16_t value = 0;
  RuntimeWriteProtocol protocol = RuntimeWriteProtocol::NONE;
  uint8_t txCounter = 0;
  uint8_t attempts = 0;
  uint32_t sentAt = 0;
};

OutputWriteTransaction outputWrite;
static constexpr uint8_t OUTPUT_WRITE_MAX_ATTEMPTS = 3;
static constexpr uint32_t OUTPUT_WRITE_TIMEOUT_MS = 120;
static constexpr uint32_t OUTPUT_VERIFY_DELAY_MS = 40;
static uint32_t outputVerifyDueMs = 0;

struct ButtonPulseState
{
  bool active = false;
  uint32_t deviceAddress = 0;
  uint8_t channel = 0;
  uint32_t dueMs = 0;
  uint32_t durationMs = 2000;
  bool returnOn = false;
};
ButtonPulseState buttonPulse;

// ============================================================
// v0.9.0 safe EEPROM / I/O configuration transaction
// EEPROM/config writes remain enabled only for the real-hardware-tested HMW-IO-12-Sw14-DR (0x001C).
// The transaction is deliberately narrow: read/modify/write known bytes,
// ACK each write, issue 0x43 apply, then read back and verify.
// ============================================================
enum class IoConfigStage : uint8_t
{
  IDLE,
  WRITE_MODE,
  WRITE_PARAM,
  APPLY,
  VERIFY_MODE_START,
  VERIFY_MODE_WAIT,
  VERIFY_PARAM_START,
  VERIFY_PARAM_WAIT,
  DONE,
  ERROR
};

struct IoConfigTransaction
{
  bool active = false;
  uint32_t target = 0;
  uint8_t channel = 0;
  IoConfigStage stage = IoConfigStage::IDLE;
  bool waitingAck = false;
  uint8_t txCounter = 0;
  uint8_t attempts = 0;
  uint32_t sentAt = 0;

  uint16_t modeAddress = 0;
  uint8_t modeValue = 0;

  bool hasParam = false;
  uint16_t paramAddress = 0;
  uint8_t paramLen = 0;
  uint8_t paramData[2] = {};

  // Optional frequency output setpoint after a successful EEPROM commit.
  bool setFrequencyAfter = false;
  uint16_t frequencyMilliHz = 0;
};

IoConfigTransaction ioConfig;
static constexpr uint8_t IO_CONFIG_MAX_ATTEMPTS = 3;
static constexpr uint32_t IO_CONFIG_ACK_TIMEOUT_MS = 150;

enum class ScanStep : uint8_t
{
  IDLE,
  STEP_TYPE,
  STEP_SERIAL_NUMBER,
  STEP_FIRMWARE,
  STEP_EEPROM0,
  STEP_EEPROM1,
  STEP_STATUS,
  STEP_NEXT_DEVICE,
  DONE
};

bool scanActive = false;
size_t scanDeviceIndex = 0;
ScanStep scanStep = ScanStep::IDLE;
uint8_t scanBusChannel = 0;
uint8_t scanRetry = 0;
uint32_t lastScanActionMs = 0;

bool statusPollActive = false;
size_t statusPollDeviceIndex = 0;
uint8_t statusPollBusChannel = 0;
uint32_t lastStatusPollActionMs = 0;
uint32_t lastStatusRefreshCompletedMs = 0;

// Optional targeted one-device refresh. Manual/global refresh keeps this false.
bool statusPollSingleDevice = false;
uint32_t statusPollSingleAddress = 0;

// A complete HBW 0x41 announcement is also a useful boot/restart indication.
// Queue the refresh instead of transmitting from inside the RX parser.
uint32_t passiveIdentityStatusPollPendingAddress = 0;

// ============================================================
// Statistics
// ============================================================

uint8_t localTxCounter = 0;

uint32_t statRxBytes = 0;
uint32_t statFrames = 0;
uint32_t statCrcOk = 0;
uint32_t statCrcError = 0;
uint32_t statTxFrames = 0;
uint32_t statAckTx = 0;
uint32_t statAckRx = 0;
uint32_t statTimeouts = 0;
uint32_t statRetries = 0;
uint32_t statPassiveUpdates = 0;

volatile uint32_t lastBusActivityUs = 0;

// ============================================================
// Web diagnostic log
// RAM only: no flash wear
// ============================================================

static constexpr uint16_t WEB_LOG_LINES = 160;

String webLogLines[WEB_LOG_LINES];
uint16_t webLogHead = 0;
uint16_t webLogCount = 0;

bool hm485RawLogEnabled = false;

// Passive legacy Discovery Analyzer removed in v0.7.50a.
// Native HM485 Discovery remains available below.

static constexpr uint8_t ACTIVE_DISCOVERY_TRIES = 3;
static constexpr uint32_t ACTIVE_DISCOVERY_TIMEOUT_MS = 20;
static constexpr uint8_t ACTIVE_DISCOVERY_MAX_DEVICES = 32;

struct ActiveDiscoveryState
{
  bool running = false;
  bool waiting = false;
  bool foundAck = false;

  uint32_t address = 0;
  uint8_t validBits = 1;
  uint8_t tries = 0;

  uint32_t nextActionMs = 0;
  uint8_t ackByte = 0;

  uint32_t probeCount = 0;
  uint32_t ackCount = 0;

  // Bytes already sitting in UART RX immediately after our own
  // discovery TX. These are treated as local TX echo / turnaround
  // residue and discarded BEFORE DiscoveryWait starts.
  uint32_t discardedEchoBytes = 0;

  // Complete RX frames that are recognized as our own transmitted
  // HM485 traffic and therefore ignored for discovery.
  uint32_t ignoredOwnFrames = 0;

  // v0.7.8 raw TX-echo matcher. During a discovery probe we remember
  // the exact bytes written to UART. Matching RX bytes are suppressed;
  // the first byte that differs is passed to the hm485d-style raw ACK
  // evaluation.
  static constexpr uint8_t TX_ECHO_MAX = 32;
  uint8_t txEcho[TX_ECHO_MAX] = {};
  uint8_t txEchoLen = 0;
  uint8_t txEchoPos = 0;
  bool txEchoMatching = false;
  uint32_t ignoredEchoBytes = 0;
  uint32_t ignoredEchoFrames = 0;

  // v0.7.9 discovery turnaround diagnostics.  Timestamp is taken
  // immediately after DE is released.  Every first non-echo RX byte
  // is logged with its distance from that point.
  uint32_t rxArmUs = 0;
  uint32_t lastAckDeltaUs = 0;
  uint32_t minAckDeltaUs = 0xFFFFFFFFUL;
  uint32_t maxAckDeltaUs = 0;
  uint8_t diagLogged = 0;
  static constexpr uint8_t DIAG_LOG_MAX = 40;

  // v0.7.12 compact turnaround analyzer
  static constexpr uint8_t ANALYZER_MAX_PROBES = 10;
  static constexpr uint32_t ANALYZER_WINDOW_US = 3000;
  static constexpr uint8_t ANALYZER_MAX_EVENTS = 80;

  struct AnalyzerEvent
  {
    uint8_t probe = 0;
    uint32_t address = 0;
    uint8_t validBits = 0;
    uint8_t value = 0;
    uint32_t deltaUs = 0;
  };

  AnalyzerEvent analyzerEvents[ANALYZER_MAX_EVENTS] = {};
  uint8_t analyzerEventCount = 0;
  uint32_t analyzerDropped = 0;
  uint32_t analyzerPreDeDrained = 0;

  // v0.7.13: measured turnaround guard.
  // v0.7.12 showed local artifacts around 626..675 us and a
  // bus-dependent candidate at 1551 us.
  static constexpr uint32_t DISCOVERY_RX_GUARD_US = 0;
  uint32_t guardIgnored = 0;

  // v0.7.14: The protocol description states that a matching module
  // answers a Discovery probe with exactly one raw byte 0xF8.
  // Other raw bytes are ignored and counted for diagnostics.
  uint32_t nonF8Ignored = 0;

  // v0.7.25: fixed known-positive Discovery probe test.
  static constexpr uint32_t KNOWN_TARGET = 0x00000000UL;
  static constexpr uint8_t KNOWN_CTRL = 0x03;
  static constexpr uint16_t KNOWN_PROBES = 1;
  static constexpr uint8_t KNOWN_TRACE_MAX = 30;
  static constexpr uint8_t SWEEP_STEPS = 1;
  static constexpr uint8_t PROBES_PER_STEP = 1;
  uint16_t turnaroundDelayUs = 0;
  uint16_t sweepSent[SWEEP_STEPS] = {};
  uint16_t sweepF8[SWEEP_STEPS] = {};
  uint16_t sweepF0[SWEEP_STEPS] = {};
  uint16_t sweep00[SWEEP_STEPS] = {};
  uint16_t sweepOther[SWEEP_STEPS] = {};
  uint16_t sweepTimeout[SWEEP_STEPS] = {};

  bool currentProbeHadRx = false;
  bool currentProbeHadF8 = false;
  uint32_t probesWithRx = 0;
  uint32_t probesWithF8 = 0;
  uint32_t probesOtherOnly = 0;
  uint32_t probeTimeouts = 0;
  uint32_t rawByteCount = 0;
  uint16_t rxHistogram[256] = {};

  uint8_t firstRxByte[KNOWN_TRACE_MAX] = {};
  uint32_t firstRxDeltaUs[KNOWN_TRACE_MAX] = {};
  bool firstRxSeen[KNOWN_TRACE_MAX] = {};

  static constexpr uint8_t TREE_TRACE_MAX = 80;
  String treeTrace[TREE_TRACE_MAX];
  uint8_t treeTraceCount = 0;
  uint32_t treeTraceDropped = 0;

  uint32_t foundAddress[ACTIVE_DISCOVERY_MAX_DEVICES] = {};
  uint8_t foundCount = 0;
};

ActiveDiscoveryState activeDiscovery;

static String wireVerifyLastTx;
static String crcAbResultText;

struct PassiveCaptureEvent
{
  uint32_t tUs;
  uint8_t b;
};

static constexpr uint16_t PASSIVE_CAPTURE_MAX = 1536;
static PassiveCaptureEvent passiveCaptureBuf[PASSIVE_CAPTURE_MAX];
static volatile uint16_t passiveCaptureCount = 0;
static volatile uint32_t passiveCaptureDropped = 0;
static volatile bool passiveCaptureActive = false;
static volatile bool passiveCaptureFull = false;
static bool passiveCaptureArmRequested = false;
static uint32_t passiveCaptureArmSinceMs = 0;
static constexpr uint32_t PASSIVE_CAPTURE_ARM_DELAY_MS = 5000;
static uint32_t passiveCaptureStartUs = 0;

// Legacy name retained as generic active-discovery-test ownership gate.
static constexpr uint32_t DISCOVERY_REPLAY_IDLE_US = 600000UL;
static bool quietRootRequested = false;
static void wireVerifySetTx(const uint8_t *data, size_t len)
{
  wireVerifyLastTx = "";
  wireVerifyLastTx.reserve(len * 3 + 16);
  for (size_t i = 0; i < len; i++)
  {
    char b[4];
    snprintf(b, sizeof(b), "%02X", data[i]);
    if (i) wireVerifyLastTx += ' ';
    wireVerifyLastTx += b;
  }
}

// ============================================================
// Forward declarations for upper layers
// ============================================================

void mqttPublishChannel(HM485Device *device, uint8_t busChannel);
void mqttPublishAllStates();
void mqttPublishDiscoveryAll();
void mqttRemoveDeviceDiscovery(HM485Device *device);
void mqttPublishGatewayDiscovery();
void mqttPublishGatewayDiagnostics();
void mqttMessageCallback(char *topic, byte *payload, unsigned int length);
bool startDigitalOutputWrite(HM485Device *device, uint8_t busChannel, bool on);
bool startFrequencyOutputWrite(HM485Device *device, uint8_t busChannel, uint16_t milliHz);
void processOutputWrite();
void processButtonPulse();
void startStatusPoll();
bool startStatusPollForDevice(uint32_t address);
static void processPassiveIdentityStatusPoll();
void startScan();


void activeDiscoveryStart();
void activeDiscoveryStop();
void processActiveDiscovery();
void activeDiscoveryRawByte(uint8_t rawByte);
uint8_t activeDeviceCount();
uint32_t activeDeviceAddress(uint8_t index);
uint8_t runtimePollDeviceCount();
uint32_t runtimePollDeviceAddress(uint8_t index);

static bool forgetDevice(uint32_t address)
{
  HM485Device *device = getDevice(address, false);
  if (!device)
    return false;

  if ((pending.active && pending.target == address) ||
      (outputWrite.active && outputWrite.target == address) ||
      (ioConfig.active && ioConfig.target == address))
    return false;

  // Remove HA retained discovery/state while the serial/key is still known.
  mqttRemoveDeviceDiscovery(device);

  Preferences p;
  if (p.begin(DEVICE_METADATA_NAMESPACE, false))
  {
    p.remove(deviceMetadataKey(address).c_str());
    p.end();
  }
  registryRemoveAddress(address);

  // Remove from runtime DB. It may be learned again later from real bus traffic.
  *device = HM485Device{};

  String msg = String("[DEVICE] Forgotten: ") + hexAddress(address);
  Serial.println(msg);
  webLogAdd(msg);

  if (mqttClient.connected())
  {
    mqttPublishDiscoveryAll();
    mqttSubscribeOutputCommands();
    mqttPublishAllStates();
  }

  return true;
}

// v0.7.42a FIX: forward declaration required by HM485Parser::decodeFrame()
void activeDiscoveryForeignFrame(uint32_t target, uint32_t sender, uint8_t ctrl);
String activeDiscoveryResultText();

// ============================================================
// Web diagnostic log helpers
// ============================================================

String hexAddress(uint32_t address);

void webLogAdd(const String &message)
{
  uint32_t now = millis();

  char prefix[24];
  snprintf(
    prefix,
    sizeof(prefix),
    "%lu.%03lu ",
    now / 1000UL,
    now % 1000UL
  );

  webLogLines[webLogHead] =
    String(prefix) + message;

  webLogHead =
    (webLogHead + 1) %
    WEB_LOG_LINES;

  if (webLogCount < WEB_LOG_LINES)
    webLogCount++;
}

void webLogClear()
{
  for (uint16_t i = 0; i < WEB_LOG_LINES; i++)
    webLogLines[i] = String();

  webLogHead = 0;
  webLogCount = 0;
}

String webLogText()
{
  String out;
  out.reserve(webLogCount * 110);

  uint16_t start =
    (
      webLogHead +
      WEB_LOG_LINES -
      webLogCount
    ) %
    WEB_LOG_LINES;

  for (uint16_t i = 0; i < webLogCount; i++)
  {
    uint16_t index =
      (start + i) %
      WEB_LOG_LINES;

    out += webLogLines[index];
    out += '\n';
  }

  return out;
}

String hm485RawLine(
  const char *direction,
  uint32_t target,
  uint32_t sender,
  uint8_t ctrl,
  const uint8_t *payload,
  uint8_t payloadLen
)
{
  String line;
  line.reserve(380);

  line += "[RAW ";
  line += direction;
  line += "] DST=";
  line += hexAddress(target);
  line += " SRC=";
  line += hexAddress(sender);

  char tmp[48];
  snprintf(
    tmp,
    sizeof(tmp),
    " CTRL=%02X LEN=%u DATA=",
    ctrl,
    payloadLen
  );

  line += tmp;

  for (uint8_t i = 0; i < payloadLen; i++)
  {
    char hb[4];
    snprintf(
      hb,
      sizeof(hb),
      "%02X",
      payload[i]
    );

    if (i)
      line += ' ';

    line += hb;
  }

  return line;
}

// ============================================================
// Generic helpers
// ============================================================

String hexAddress(uint32_t address)
{
  char buffer[9];
  snprintf(buffer, sizeof(buffer), "%08lX", address);
  return String(buffer);
}

void printAddress(uint32_t address)
{
  Serial.printf("%08lX", address);
}

uint8_t busToHmWiredChannel(uint8_t busChannel)
{
  return busChannel + 1;
}

String twoDigit(uint8_t value)
{
  char b[4];
  snprintf(b, sizeof(b), "%02u", value);
  return String(b);
}

String htmlEscape(const String &src)
{
  String out;
  out.reserve(src.length() + 16);

  for (size_t i = 0; i < src.length(); i++)
  {
    char c = src[i];
    switch (c)
    {
      case '&': out += F("&amp;"); break;
      case '<': out += F("&lt;"); break;
      case '>': out += F("&gt;"); break;
      case '"': out += F("&quot;"); break;
      case '\'': out += F("&#39;"); break;
      default: out += c; break;
    }
  }
  return out;
}

String jsonEscape(const String &src)
{
  String out;
  out.reserve(src.length() + 16);

  for (size_t i = 0; i < src.length(); i++)
  {
    char c = src[i];
    switch (c)
    {
      case '\\': out += F("\\\\"); break;
      case '"':  out += F("\\\""); break;
      case '\n': out += F("\\n"); break;
      case '\r': out += F("\\r"); break;
      case '\t': out += F("\\t"); break;
      default: out += c; break;
    }
  }
  return out;
}

String normalizedTopic(String topic)
{
  topic.trim();

  while (topic.endsWith("/"))
    topic.remove(topic.length() - 1);

  while (topic.startsWith("/"))
    topic.remove(0, 1);

  return topic;
}

// ============================================================
// Configuration / NVS
// ============================================================

void loadConfig()
{
  preferences.begin("hm485gw", true);

  config.wifiSsid        = preferences.getString("wifi_ssid", "");
  config.wifiPassword    = preferences.getString("wifi_pass", "");

  config.mqttHost        = preferences.getString("mqtt_host", "");
  config.mqttPort        = preferences.getUShort("mqtt_port", 1883);
  config.mqttUser        = preferences.getString("mqtt_user", "");
  config.mqttPassword    = preferences.getString("mqtt_pass", "");

  config.mqttClientId    = preferences.getString("mqtt_id", "hm485-gateway");
  config.mqttBaseTopic   = preferences.getString("mqtt_base", "hm485");
  config.discoveryPrefix = preferences.getString("ha_prefix", "homeassistant");
  config.discoveryEnabled = preferences.getBool("ha_disc", true);
  config.experimentalWritesEnabled = preferences.getBool("exp_write", false);
  config.hostname        = preferences.getString("hostname", "hm485-gateway");
  config.localAddress    = preferences.getUInt("hm_addr", DEFAULT_LOCAL_ADDRESS);
  config.uiLanguage      = preferences.getString("ui_lang", "de");

  config.webUser         = preferences.getString("web_user", "admin");
  config.webPassword     = preferences.getString("web_pass", "hm485setup");

  preferences.end();

  config.mqttBaseTopic   = normalizedTopic(config.mqttBaseTopic);
  config.discoveryPrefix = normalizedTopic(config.discoveryPrefix);

  if (config.mqttBaseTopic.length() == 0)
    config.mqttBaseTopic = "hm485";

  if (config.discoveryPrefix.length() == 0)
    config.discoveryPrefix = "homeassistant";

  if (config.mqttClientId.length() == 0)
    config.mqttClientId = "hm485-gateway";

  if (config.hostname.length() == 0)
    config.hostname = "hm485-gateway";

  if (config.localAddress == 0 || config.localAddress == 0xFFFFFFFFUL)
    config.localAddress = DEFAULT_LOCAL_ADDRESS;

  config.uiLanguage.toLowerCase();
  if (config.uiLanguage != "de" && config.uiLanguage != "en")
    config.uiLanguage = "de";

  if (config.webUser.length() == 0)
    config.webUser = "admin";

  if (config.webPassword.length() == 0)
    config.webPassword = "hm485setup";
}

void saveConfig()
{
  preferences.begin("hm485gw", false);

  preferences.putString("wifi_ssid", config.wifiSsid);
  preferences.putString("wifi_pass", config.wifiPassword);

  preferences.putString("mqtt_host", config.mqttHost);
  preferences.putUShort("mqtt_port", config.mqttPort);
  preferences.putString("mqtt_user", config.mqttUser);
  preferences.putString("mqtt_pass", config.mqttPassword);

  preferences.putString("mqtt_id", config.mqttClientId);
  preferences.putString("mqtt_base", config.mqttBaseTopic);
  preferences.putString("ha_prefix", config.discoveryPrefix);
  preferences.putBool("ha_disc", config.discoveryEnabled);
  preferences.putBool("exp_write", config.experimentalWritesEnabled);
  preferences.putString("hostname", config.hostname);
  preferences.putUInt("hm_addr", config.localAddress);
  preferences.putString("ui_lang", config.uiLanguage);

  preferences.putString("web_user", config.webUser);
  preferences.putString("web_pass", config.webPassword);

  preferences.end();
}

// ============================================================
// WT32-ETH01 Ethernet / network selection
// ============================================================

bool ethernetReady()
{
  return
    ethernetStarted &&
    ETH.linkUp() &&
    ETH.localIP() != IPAddress(0, 0, 0, 0);
}

bool wifiReady()
{
  return WiFi.status() == WL_CONNECTED;
}

bool networkReady()
{
  return ethernetReady() || wifiReady();
}

// Stable local URL independent of the currently active LAN/WiFi IP.
bool mdnsStarted = false;
String mdnsNetwork;

String gatewayConfigurationUrl()
{
  return String("http://") + config.hostname + ".local/";
}

void startMdnsIfNeeded()
{
  // Do not advertise the normal gateway hostname from the setup AP.
  if (!networkReady() || setupApActive)
  {
    if (mdnsStarted)
    {
      MDNS.end();
      mdnsStarted = false;
      mdnsNetwork = "";
    }
    return;
  }

  if (config.hostname.length() == 0)
    return;

  String currentNetwork = ethernetReady() ? "Ethernet" : "WiFi";

  // Re-register when the active transport changes. This keeps the same
  // hostname working after Ethernet <-> WiFi fallback transitions.
  if (mdnsStarted && mdnsNetwork == currentNetwork)
    return;

  if (mdnsStarted)
  {
    MDNS.end();
    mdnsStarted = false;
    mdnsNetwork = "";
    delay(2);
  }

  if (MDNS.begin(config.hostname.c_str()))
  {
    mdnsStarted = true;
    mdnsNetwork = currentNetwork;
    MDNS.addService("http", "tcp", 80);

    Serial.print(F("[mDNS] Ready on "));
    Serial.print(currentNetwork);
    Serial.print(F(": http://"));
    Serial.print(config.hostname);
    Serial.println(F(".local/"));
    webLogAdd(String("[mDNS] Ready on ") + currentNetwork + " http://" + config.hostname + ".local/");
  }
  else
  {
    Serial.println(F("[mDNS] Start failed"));
    webLogAdd("[mDNS] Start failed");
  }
}

String activeNetworkName()
{
  if (ethernetReady())
    return "Ethernet";

  if (wifiReady())
    return "WiFi";

  if (setupApActive)
    return "Setup-AP";

  return "offline";
}

IPAddress activeLocalIP()
{
  if (ethernetReady())
    return ETH.localIP();

  if (wifiReady())
    return WiFi.localIP();

  if (setupApActive)
    return WiFi.softAPIP();

  return IPAddress(0, 0, 0, 0);
}

void startEthernet()
{
  if (ethernetStarted)
    return;

  Serial.println(F("[ETH] Starting WT32-ETH01 LAN8720..."));

  // Arduino-ESP32 3.x signature:
  // ETH.begin(type, phy_addr, mdc, mdio, power, clk_mode)
  bool ok = ETH.begin(
    ETH_PHY_LAN8720,
    1,
    23,
    18,
    16,
    ETH_CLOCK_GPIO0_IN
  );

  ethernetStarted = ok;
  ethernetStartedMs = millis();

  if (!ok)
    Serial.println(F("[ETH] ETH.begin() failed"));
}

// ============================================================
// WiFi
// ============================================================

void startSetupAp()
{
  if (setupApActive)
    return;

  uint64_t chipId = ESP.getEfuseMac();
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%06llX",
           (unsigned long long)(chipId & 0xFFFFFFULL));

  setupApSsid = "HM485-Gateway-";
  setupApSsid += suffix;

  WiFi.mode(WIFI_AP_STA);

  // Setup-only password. Change later if desired.
  if (WiFi.softAP(setupApSsid.c_str(), "hm485setup"))
  {
    setupApActive = true;

    Serial.println();
    Serial.println(F("[WIFI] Setup AP active"));
    Serial.print(F("[WIFI] SSID : "));
    Serial.println(setupApSsid);
    Serial.println(F("[WIFI] PASS : hm485setup"));
    Serial.print(F("[WIFI] URL  : http://"));
    Serial.println(WiFi.softAPIP());
  }
}

void startWifi()
{
  if (wifiFallbackStarted && WiFi.getMode() != WIFI_OFF)
    return;

  wifiFallbackStarted = true;

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(config.hostname.c_str());

  if (config.wifiSsid.length() == 0)
  {
    startSetupAp();
    return;
  }

  WiFi.mode(WIFI_STA);

  Serial.print(F("[WIFI] Connecting to "));
  Serial.println(config.wifiSsid);

  WiFi.begin(
    config.wifiSsid.c_str(),
    config.wifiPassword.c_str()
  );

  wifiConnectStartedMs = millis();
}

void processWifi()
{
  if (!wifiFallbackStarted && !setupApActive)
    return;

  static wl_status_t lastStatus = WL_NO_SHIELD;
  wl_status_t current = WiFi.status();

  if (current != lastStatus)
  {
    lastStatus = current;

    if (current == WL_CONNECTED)
    {
      Serial.println();
      Serial.print(F("[WIFI] Connected, IP: "));
      Serial.println(WiFi.localIP());
      Serial.print(F("[WIFI] RSSI: "));
      Serial.println(WiFi.RSSI());
    }
  }

  if (
    config.wifiSsid.length() > 0 &&
    current != WL_CONNECTED &&
    !setupApActive &&
    millis() - wifiConnectStartedMs > WIFI_CONNECT_TIMEOUT_MS
  )
  {
    Serial.println(F("[WIFI] Station timeout, enabling setup AP."));
    startSetupAp();
  }
}

void processNetwork()
{
  bool ethNow = ethernetReady();

  if (ethNow != lastEthernetReady)
  {
    lastEthernetReady = ethNow;

    if (ethNow)
    {
      Serial.println();
      Serial.print(F("[ETH] Link/IP ready: "));
      Serial.println(ETH.localIP());

      webLogAdd(
        String("[NET] Ethernet ready IP=") +
        ETH.localIP().toString()
      );

      // Ethernet is primary. If WiFi was only a fallback, turn the
      // station radio back off. Any MQTT TCP session over WiFi will
      // reconnect automatically through Ethernet.
      if (wifiReady() && !setupApActive)
      {
        Serial.println(F("[NET] Ethernet active -> WiFi fallback off"));
        mqttClient.disconnect();
        WiFi.disconnect(false, false);
        WiFi.mode(WIFI_OFF);
        wifiFallbackStarted = false;
      }
    }
    else
    {
      Serial.println(F("[ETH] Link/IP unavailable"));
      webLogAdd("[NET] Ethernet unavailable");
    }
  }

  // Start WiFi only when Ethernet has not become usable.
  if (
    !ethNow &&
    !wifiFallbackStarted &&
    millis() - ethernetStartedMs >= ETHERNET_FALLBACK_DELAY_MS
  )
  {
    Serial.println(F("[NET] Ethernet unavailable -> starting WiFi fallback"));
    startWifi();
  }

  // Stable .local hostname for both Ethernet and WiFi.
  startMdnsIfNeeded();
}

// ============================================================
// MQTT topic helpers
// ============================================================

String mqttAvailabilityTopic()
{
  return config.mqttBaseTopic + "/gateway/status";
}

String mqttDeviceKey(const HM485Device *device)
{
  if (device && device->serialKnown && strlen(device->serialNumber) > 0)
    return String(device->serialNumber);

  if (device)
    return hexAddress(device->address);

  return "unknown";
}

String mqttStateTopic(const HM485Device *device, uint8_t busChannel)
{
  return config.mqttBaseTopic
       + "/"
       + mqttDeviceKey(device)
       + "/channel/"
       + twoDigit(busToHmWiredChannel(busChannel))
       + "/state";
}

String mqttUniqueId(const HM485Device *device, uint8_t busChannel)
{
  return "hm485_"
       + mqttDeviceKey(device)
       + "_ch"
       + twoDigit(busToHmWiredChannel(busChannel));
}

// ============================================================
// HA entity classification
// ============================================================

bool channelIsBinary(const HM485Device *device, uint8_t busChannel)
{
  if (!device)
    return false;

  switch (channelSemanticProfile(device, busChannel))
  {
    case SemanticProfile::WINDOW:
    case SemanticProfile::DOOR:
    case SemanticProfile::ALARM:
    case SemanticProfile::CONTACT:
    case SemanticProfile::BINARY_INPUT:
    case SemanticProfile::OUTPUT_MONITOR:
    case SemanticProfile::OUTPUT_SWITCH:
    case SemanticProfile::OUTPUT_BUTTON:
      return true;
    case SemanticProfile::ANALOG_SENSOR:
    case SemanticProfile::FREQUENCY_SENSOR:
    case SemanticProfile::SHUTTER_MONITOR:
    case SemanticProfile::RAW_SENSOR:
    case SemanticProfile::ANALOG_VOLTAGE_RAW:
    case SemanticProfile::ANALOG_CURRENT_RAW:
    case SemanticProfile::ANALOG_POWER_RAW:
    case SemanticProfile::ANALOG_TEMPERATURE_RAW:
    case SemanticProfile::ANALOG_RESISTANCE_RAW:
    case SemanticProfile::ANALOG_PRESSURE_RAW:
      return false;
    case SemanticProfile::AUTO:
    default:
      break;
  }

  if (!device->profile)
    return false;

  const ChannelProfile *cp =
    findChannelProfile(device->profile, busChannel);

  if (!cp)
    return false;

  switch (cp->type)
  {
    case ChannelType::CONTACT:
    case ChannelType::DIGITAL_OUTPUT:
      return true;

    case ChannelType::DIGITAL_ANALOG_OUTPUT:
      return device->channel[busChannel].behaviour ==
             ChannelBehaviour::DIGITAL_OUTPUT;

    case ChannelType::DIGITAL_FREQUENCY_INPUT:
      return device->channel[busChannel].behaviour ==
             ChannelBehaviour::DIGITAL_INPUT;

    case ChannelType::DIGITAL_ANALOG_INPUT:
      return device->channel[busChannel].behaviour ==
             ChannelBehaviour::DIGITAL_INPUT;

    case ChannelType::DIGITAL_INPUT:
    case ChannelType::KEY_INPUT:
    case ChannelType::MOTION:
      return true;

    default:
      return false;
  }
}

static bool channelIsWritableOutput(const HM485Device *device, uint8_t busChannel)
{
  if (!device || busChannel >= MAX_CHANNELS || !device->profile) return false;
  if (runtimeWriteProtocol(device, busChannel) == RuntimeWriteProtocol::NONE) return false;
  SemanticProfile sp = channelSemanticProfile(device, busChannel);
  return sp == SemanticProfile::OUTPUT_SWITCH || sp == SemanticProfile::OUTPUT_BUTTON;
}

static bool channelIsFrequencyOutput(const HM485Device *device, uint8_t busChannel)
{
  if (!device || busChannel >= MAX_CHANNELS || !device->profile) return false;
  if (!deviceRuntimeWriteAllowed(device)) return false;
  const ChannelProfile *cp = findChannelProfile(device->profile, busChannel);
  return cp &&
         cp->type == ChannelType::DIGITAL_ANALOG_OUTPUT &&
         device->channel[busChannel].behaviour == ChannelBehaviour::ANALOG_OUTPUT;
}

String mqttCommandTopic(const HM485Device *device, uint8_t busChannel)
{
  return config.mqttBaseTopic + "/" + mqttDeviceKey(device) + "/channel/" +
         twoDigit(busToHmWiredChannel(busChannel)) + "/set";
}

String discoveryComponent(const HM485Device *device, uint8_t busChannel)
{
  SemanticProfile sp = channelSemanticProfile(device, busChannel);
  if (channelIsFrequencyOutput(device, busChannel)) return "number";
  if (sp == SemanticProfile::OUTPUT_SWITCH && channelIsWritableOutput(device, busChannel)) return "switch";
  if (sp == SemanticProfile::OUTPUT_BUTTON && channelIsWritableOutput(device, busChannel)) return "button";
  return channelIsBinary(device, busChannel) ? "binary_sensor" : "sensor";
}

bool channelLogicalOn(const HM485Device *device, uint8_t busChannel)
{
  if (!device || busChannel >= MAX_CHANNELS)
    return false;
  bool on = device->channel[busChannel].value != 0;
  if (device->invertLogic[busChannel])
    on = !on;
  return on;
}

static bool channelIsTemperatureSensor(const HM485Device *device, uint8_t busChannel)
{
  if (!device || !device->profile || busChannel >= MAX_CHANNELS)
    return false;
  const ChannelProfile *cp = findChannelProfile(device->profile, busChannel);
  return cp && cp->type == ChannelType::TEMPERATURE;
}

static int16_t channelTemperatureCentiDegC(const HM485Device *device, uint8_t busChannel)
{
  return static_cast<int16_t>(
    static_cast<uint16_t>(device->channel[busChannel].value & 0xFFFFU)
  );
}

static bool channelTemperatureUnavailable(const HM485Device *device, uint8_t busChannel)
{
  // HBW-1W-T10 uses -273.15 degC as its "no sensor / invalid sensor" marker.
  return channelIsTemperatureSensor(device, busChannel) &&
         channelTemperatureCentiDegC(device, busChannel) == -27315;
}

// ============================================================
// MQTT publishing
// ============================================================

void mqttPublishAvailability()
{
  if (!mqttClient.connected())
    return;

  String topic = mqttAvailabilityTopic();
  mqttClient.publish(topic.c_str(), "online", true);
}

void mqttPublishChannel(HM485Device *device, uint8_t busChannel)
{
  if (
    !mqttClient.connected() ||
    !device ||
    busChannel >= MAX_CHANNELS ||
    !device->channel[busChannel].known
  )
  {
    return;
  }

  String topic = mqttStateTopic(device, busChannel);
  String payload;

  if (channelIsBinary(device, busChannel))
  {
    payload = channelLogicalOn(device, busChannel)
      ? "ON"
      : "OFF";
  }
  else if (channelIsFrequencyOutput(device, busChannel))
  {
    // HMW wire/status value is milli-Hertz. Home Assistant number uses Hz.
    payload = String(device->channel[busChannel].value / 1000.0f, 3);
  }
  else if (channelIsTemperatureSensor(device, busChannel))
  {
    if (channelTemperatureUnavailable(device, busChannel))
      payload = "unknown";
    else
      payload = String(channelTemperatureCentiDegC(device, busChannel) / 100.0f, 2);
  }
  else
  {
    payload = String(device->channel[busChannel].value);
  }

  mqttClient.publish(
    topic.c_str(),
    payload.c_str(),
    true
  );
}

void mqttPublishAllStates()
{
  if (!mqttClient.connected())
    return;

  for (uint8_t i = 0; i < MAX_DEVICES; i++)
  {
    HM485Device &d = devices[i];

    if (!d.used)
      continue;

    for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++)
    {
      if (d.channel[ch].known)
        mqttPublishChannel(&d, ch);
    }
  }
}

void mqttPublishDiscoveryChannel(
  HM485Device *device,
  uint8_t busChannel
)
{
  if (
    !mqttClient.connected() ||
    !config.discoveryEnabled ||
    !device ||
    !device->profile ||
    !device->serialKnown
  )
  {
    return;
  }

  const ChannelProfile *cp =
    findChannelProfile(device->profile, busChannel);

  if (!cp)
    return;

  String component =
    discoveryComponent(device, busChannel);

  String uniqueId =
    mqttUniqueId(device, busChannel);

  String discoveryTopic =
    config.discoveryPrefix
    + "/"
    + component
    + "/"
    + uniqueId
    + "/config";

  // A profile change can move the same stable unique_id between HA domains.
  // Remove retained discovery configs from the other domains first, otherwise
  // Home Assistant may keep an orphaned binary_sensor/sensor beside switch/button.
  const char *allComponents[] = {"binary_sensor", "sensor", "switch", "button", "number"};
  for (const char *oldComponent : allComponents)
  {
    if (component == oldComponent) continue;
    String oldTopic = config.discoveryPrefix + "/" + String(oldComponent) + "/" +
                      uniqueId + "/config";
    mqttClient.publish(oldTopic.c_str(), "", true);
  }

  String deviceName = effectiveDeviceName(device);
  String channelName = effectiveChannelName(device, busChannel);

  String payload;
  payload.reserve(900);

  payload += "{";

  payload += "\"name\":\"";
  payload += jsonEscape(channelName);
  payload += "\",";

  payload += "\"unique_id\":\"";
  payload += jsonEscape(uniqueId);
  payload += "\",";

  SemanticProfile semantic = channelSemanticProfile(device, busChannel);
  bool writableOutput = channelIsWritableOutput(device, busChannel);
  bool frequencyOutput = channelIsFrequencyOutput(device, busChannel);
  if (semantic != SemanticProfile::OUTPUT_BUTTON)
  {
    payload += "\"state_topic\":\"";
    payload += jsonEscape(mqttStateTopic(device, busChannel));
    payload += "\",";
  }
  if (writableOutput || frequencyOutput)
  {
    payload += "\"command_topic\":\"";
    payload += jsonEscape(mqttCommandTopic(device, busChannel));
    payload += "\",";
    if (semantic == SemanticProfile::OUTPUT_BUTTON) payload += "\"payload_press\":\"PRESS\",";
  }
  if (frequencyOutput)
  {
    payload += "\"min\":0,";
    payload += "\"max\":65.535,";
    payload += "\"step\":0.001,";
    payload += "\"mode\":\"box\",";
    payload += "\"unit_of_measurement\":\"Hz\",";
  }
  if (channelIsTemperatureSensor(device, busChannel))
  {
    payload += "\"device_class\":\"temperature\",";
    payload += "\"state_class\":\"measurement\",";
    payload += "\"unit_of_measurement\":\"°C\",";
  }
  else if (cp->type == ChannelType::COUNTER)
  {
    // HBW-Sen-EP counters are monotonic during one module runtime and reset
    // when the AVR restarts. Home Assistant's total_increasing state class
    // handles such counter resets.
    payload += "\"state_class\":\"total_increasing\",";
    payload += "\"unit_of_measurement\":\"impulses\",";
    payload += "\"icon\":\"mdi:counter\",";
  }

  payload += "\"availability_topic\":\"";
  payload += jsonEscape(mqttAvailabilityTopic());
  payload += "\",";

  if (channelIsBinary(device, busChannel) && semantic != SemanticProfile::OUTPUT_BUTTON)
  {
    payload += "\"payload_on\":\"ON\",";
    payload += "\"payload_off\":\"OFF\",";
  }

  const char *deviceClass = semanticDeviceClass(channelSemanticProfile(device, busChannel));
  if (deviceClass)
  {
    payload += "\"device_class\":\"";
    payload += deviceClass;
    payload += "\",";
  }

  const char *icon = semanticIcon(channelSemanticProfile(device, busChannel));
  if (icon)
  {
    payload += "\"icon\":\"";
    payload += icon;
    payload += "\",";
  }

  payload += "\"device\":{";

  payload += "\"identifiers\":[\"hm485_";
  payload += jsonEscape(String(device->serialNumber));
  payload += "\"],";

  payload += "\"name\":\"";
  payload += jsonEscape(deviceName);
  payload += "\",";

  payload += "\"model\":\"";
  payload += jsonEscape(String(device->profile->model));
  payload += "\",";

  payload += "\"manufacturer\":\"eQ-3 / Homematic Wired\",";

  payload += "\"configuration_url\":\"";
  payload += jsonEscape(gatewayConfigurationUrl());
  payload += "\",";

  payload += "\"sw_version\":\"";
  char fw[8];
  snprintf(fw, sizeof(fw), "%04X", device->firmware);
  payload += fw;
  payload += "\"";

  payload += "}";

  payload += "}";

  mqttClient.publish(
    discoveryTopic.c_str(),
    payload.c_str(),
    true
  );
}


void mqttRemoveDeviceDiscovery(HM485Device *device)
{
  if (!mqttClient.connected() || !device || !device->serialKnown)
    return;

  const char *allComponents[] = {"binary_sensor", "sensor", "switch", "button", "number"};

  for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++)
  {
    String uniqueId = mqttUniqueId(device, ch);

    for (const char *component : allComponents)
    {
      String topic = config.discoveryPrefix + "/" + String(component) + "/" +
                     uniqueId + "/config";
      mqttClient.publish(topic.c_str(), "", true);
    }

    // Also clear any retained state from this device/channel.
    mqttClient.publish(mqttStateTopic(device, ch).c_str(), "", true);
  }
}

void mqttPublishDiscoveryAll()
{
  if (
    !mqttClient.connected() ||
    !config.discoveryEnabled
  )
  {
    return;
  }

  for (uint8_t i = 0; i < MAX_DEVICES; i++)
  {
    HM485Device &d = devices[i];

    if (
      !d.used ||
      !d.profile ||
      !d.serialKnown
    )
    {
      continue;
    }

    uint8_t count = 0;

    for (uint8_t g = 0; g < d.profile->channelGroupCount; g++)
    {
      uint8_t end =
        d.profile->channels[g].first +
        d.profile->channels[g].count;

      if (end > count)
        count = end;
    }

    for (uint8_t ch = 0; ch < count; ch++)
      mqttPublishDiscoveryChannel(&d, ch);
  }
}


uint8_t detectedDeviceCount()
{
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_DEVICES; i++)
  {
    if (devices[i].used && devices[i].typeKnown)
      count++;
  }

  return count;
}

String mqttGatewayTopic(const char *name)
{
  return config.mqttBaseTopic + "/gateway/" + String(name);
}

void mqttPublishGatewaySensorDiscovery(
  const char *objectId,
  const char *name,
  const char *icon = nullptr,
  const char *unit = nullptr,
  const char *deviceClass = nullptr,
  const char *stateClass = nullptr
)
{
  if (!mqttClient.connected() || !config.discoveryEnabled)
    return;

  String uniqueId = "hm485_gateway_" + String(objectId);
  String topic =
    config.discoveryPrefix + "/sensor/" + uniqueId + "/config";

  String payload;
  payload.reserve(900);

  payload += "{";
  payload += "\"name\":\"" + jsonEscape(String(name)) + "\",";
  payload += "\"unique_id\":\"" + jsonEscape(uniqueId) + "\",";
  payload += "\"state_topic\":\"" +
             jsonEscape(mqttGatewayTopic(objectId)) + "\",";
  payload += "\"availability_topic\":\"" +
             jsonEscape(mqttAvailabilityTopic()) + "\",";

  if (icon && strlen(icon))
    payload += "\"icon\":\"" + jsonEscape(String(icon)) + "\",";

  if (unit && strlen(unit))
    payload += "\"unit_of_measurement\":\"" +
               jsonEscape(String(unit)) + "\",";

  if (deviceClass && strlen(deviceClass))
    payload += "\"device_class\":\"" +
               jsonEscape(String(deviceClass)) + "\",";

  if (stateClass && strlen(stateClass))
    payload += "\"state_class\":\"" +
               jsonEscape(String(stateClass)) + "\",";

  payload += "\"entity_category\":\"diagnostic\",";

  payload += "\"device\":{";
  payload += "\"identifiers\":[\"hm485_gateway\"],";
  payload += "\"name\":\"HM485 Gateway\",";
  payload += "\"model\":\"ESP32 HM485 Gateway\",";
  payload += "\"manufacturer\":\"DIY / Homematic Wired\",";
  payload += "\"configuration_url\":\"" + jsonEscape(gatewayConfigurationUrl()) + "\",";
  payload += "\"sw_version\":\"" + jsonEscape(String(FW_VERSION)) + "\"";
  payload += "}";

  payload += "}";

  mqttClient.publish(topic.c_str(), payload.c_str(), true);
}

void mqttPublishGatewayDiscovery()
{
  if (!mqttClient.connected() || !config.discoveryEnabled)
    return;

  mqttPublishGatewaySensorDiscovery(
    "firmware", "Firmware", "mdi:chip"
  );
  mqttPublishGatewaySensorDiscovery(
    "network", "Aktives Netzwerk", "mdi:lan-connect"
  );
  mqttPublishGatewaySensorDiscovery(
    "ip", "IP-Adresse", "mdi:ip-network"
  );
  mqttPublishGatewaySensorDiscovery(
    "rssi", "WLAN RSSI", "mdi:wifi", "dBm",
    "signal_strength", "measurement"
  );
  mqttPublishGatewaySensorDiscovery(
    "uptime", "Uptime", "mdi:timer-outline", "s",
    "duration", "total_increasing"
  );
  mqttPublishGatewaySensorDiscovery(
    "devices", "Erkannte HM-Wired-Geräte", "mdi:devices"
  );
  mqttPublishGatewaySensorDiscovery(
    "rx_frames", "RX Frames", "mdi:download-network-outline"
  );
  mqttPublishGatewaySensorDiscovery(
    "tx_frames", "TX Frames", "mdi:upload-network-outline"
  );
  mqttPublishGatewaySensorDiscovery(
    "crc_errors", "CRC-Fehler", "mdi:alert-circle-outline"
  );
  mqttPublishGatewaySensorDiscovery(
    "timeouts", "Timeouts", "mdi:timer-alert-outline"
  );
  mqttPublishGatewaySensorDiscovery(
    "retries", "Retries", "mdi:reload"
  );
  mqttPublishGatewaySensorDiscovery(
    "passive_updates", "Passive Updates", "mdi:access-point-network"
  );
}

void mqttPublishGatewayDiagnostics()
{
  if (!mqttClient.connected())
    return;

  mqttClient.publish(
    mqttGatewayTopic("firmware").c_str(),
    FW_VERSION,
    true
  );

  String networkName = activeNetworkName();
  mqttClient.publish(
    mqttGatewayTopic("network").c_str(),
    networkName.c_str(),
    true
  );

  String ip =
    networkReady()
      ? activeLocalIP().toString()
      : String("0.0.0.0");

  mqttClient.publish(
    mqttGatewayTopic("ip").c_str(),
    ip.c_str(),
    true
  );

  String rssi =
    wifiReady()
      ? String(WiFi.RSSI())
      : String("0");

  mqttClient.publish(
    mqttGatewayTopic("rssi").c_str(),
    rssi.c_str(),
    true
  );

  String uptime = String(millis() / 1000UL);
  String deviceCount = String(detectedDeviceCount());
  String rxFrames = String(statFrames);
  String txFrames = String(statTxFrames);
  String crcErrors = String(statCrcError);
  String timeouts = String(statTimeouts);
  String retries = String(statRetries);
  String passiveUpdates = String(statPassiveUpdates);

  mqttClient.publish(mqttGatewayTopic("uptime").c_str(), uptime.c_str(), true);
  mqttClient.publish(mqttGatewayTopic("devices").c_str(), deviceCount.c_str(), true);
  mqttClient.publish(mqttGatewayTopic("rx_frames").c_str(), rxFrames.c_str(), true);
  mqttClient.publish(mqttGatewayTopic("tx_frames").c_str(), txFrames.c_str(), true);
  mqttClient.publish(mqttGatewayTopic("crc_errors").c_str(), crcErrors.c_str(), true);
  mqttClient.publish(mqttGatewayTopic("timeouts").c_str(), timeouts.c_str(), true);
  mqttClient.publish(mqttGatewayTopic("retries").c_str(), retries.c_str(), true);
  mqttClient.publish(
    mqttGatewayTopic("passive_updates").c_str(),
    passiveUpdates.c_str(),
    true
  );

  lastGatewayDiagnosticsMs = millis();
}

void mqttSubscribeOutputCommands()
{
  if (!mqttClient.connected()) return;
  for (uint8_t i=0;i<MAX_DEVICES;i++)
  {
    HM485Device &d=devices[i]; if(!d.used || !d.profile || !d.serialKnown) continue;
    uint8_t count=deviceChannelCount(&d);
    for(uint8_t ch=0;ch<count;ch++)
      if(channelIsWritableOutput(&d,ch) || channelIsFrequencyOutput(&d,ch))
        mqttClient.subscribe(mqttCommandTopic(&d,ch).c_str());
  }
}

bool connectMqtt()
{
  if (
    !networkReady() ||
    config.mqttHost.length() == 0
  )
  {
    return false;
  }

  mqttClient.setServer(
    config.mqttHost.c_str(),
    config.mqttPort
  );

  String willTopic =
    mqttAvailabilityTopic();

  bool ok = false;

  if (config.mqttUser.length() > 0)
  {
    ok = mqttClient.connect(
      config.mqttClientId.c_str(),
      config.mqttUser.c_str(),
      config.mqttPassword.c_str(),
      willTopic.c_str(),
      0,
      true,
      "offline"
    );
  }
  else
  {
    ok = mqttClient.connect(
      config.mqttClientId.c_str(),
      willTopic.c_str(),
      0,
      true,
      "offline"
    );
  }

  if (ok)
  {
    Serial.println(F("[MQTT] Connected"));

    webLogAdd(
      String("[MQTT] Connected ") +
      config.mqttHost +
      ":" +
      String(config.mqttPort)
    );

    mqttPublishAvailability();
    mqttPublishGatewayDiscovery();
    mqttPublishGatewayDiagnostics();
    mqttPublishDiscoveryAll();
    mqttSubscribeOutputCommands();
    mqttPublishAllStates();
  }
  else
  {
    Serial.print(F("[MQTT] Connect failed, state="));
    Serial.println(mqttClient.state());
  }

  return ok;
}

void processMqtt()
{
  if (!networkReady())
    return;

  if (!mqttClient.connected())
  {
    if (
      millis() - lastMqttReconnectMs >=
      MQTT_RECONNECT_MS
    )
    {
      lastMqttReconnectMs = millis();
      connectMqtt();
    }

    return;
  }

  mqttClient.loop();

  if (
    millis() - lastGatewayDiagnosticsMs >=
    GATEWAY_DIAGNOSTICS_MS
  )
  {
    mqttPublishGatewayDiagnostics();
  }
}

// ============================================================
// Web authentication
// ============================================================

bool requireWebAuth()
{
  if (
    webServer.authenticate(
      config.webUser.c_str(),
      config.webPassword.c_str()
    )
  )
  {
    return true;
  }

  webServer.requestAuthentication();
  return false;
}

// ============================================================
// Web UI
// ============================================================

static bool uiGerman()
{
  return config.uiLanguage != "en";
}

static const char *T(const char *de, const char *en)
{
  return uiGerman() ? de : en;
}

static String webNav()
{
  String h;
  h.reserve(700);
  h += F("<nav class='topnav'>");
  h += F("<a href='/'>"); h += T("Übersicht", "Overview"); h += F("</a>");
  h += F("<a href='/config'>"); h += T("Konfiguration", "Configuration"); h += F("</a>");
  h += F("<a href='/backup'>"); h += T("Sicherung", "Backup"); h += F("</a>");
  h += F("<a href='/log'>"); h += T("Diagnose", "Diagnostics"); h += F("</a>");
  h += F("<a href='/update'>Firmware</a>");
  h += F("</nav>");
  return h;
}

static const char *semanticProfileLabel(SemanticProfile profile)
{
  switch (profile)
  {
    case SemanticProfile::WINDOW:           return T("Fenster", "Window");
    case SemanticProfile::DOOR:             return T("Tür", "Door");
    case SemanticProfile::ALARM:            return T("Alarm", "Alarm");
    case SemanticProfile::CONTACT:          return T("Kontakt", "Contact");
    case SemanticProfile::BINARY_INPUT:     return T("Binäreingang", "Binary input");
    case SemanticProfile::ANALOG_SENSOR:    return T("Analogsensor", "Analog sensor");
    case SemanticProfile::FREQUENCY_SENSOR: return T("Frequenzsensor", "Frequency sensor");
    case SemanticProfile::OUTPUT_MONITOR:   return T("Ausgang (nur lesen)", "Output (read-only)");
    case SemanticProfile::SHUTTER_MONITOR:  return T("Rollladen (nur lesen)", "Shutter (read-only)");
    case SemanticProfile::RAW_SENSOR:       return T("Raw Sensor", "Raw sensor");
    case SemanticProfile::OUTPUT_SWITCH:    return T("Ausgang: Switch", "Output: switch");
    case SemanticProfile::OUTPUT_BUTTON:    return T("Ausgang: Button/Impuls", "Output: button/pulse");
    case SemanticProfile::ANALOG_VOLTAGE_RAW:     return T("Spannung (Rohwert)", "Voltage (raw)");
    case SemanticProfile::ANALOG_CURRENT_RAW:     return T("Strom (Rohwert)", "Current (raw)");
    case SemanticProfile::ANALOG_POWER_RAW:       return T("Leistung (Rohwert)", "Power (raw)");
    case SemanticProfile::ANALOG_TEMPERATURE_RAW: return T("Temperatur (Rohwert)", "Temperature (raw)");
    case SemanticProfile::ANALOG_RESISTANCE_RAW:  return T("Widerstand (Rohwert)", "Resistance (raw)");
    case SemanticProfile::ANALOG_PRESSURE_RAW:    return T("Druck (Rohwert)", "Pressure (raw)");
    case SemanticProfile::AUTO:
    default:                                return "Auto";
  }
}

String htmlHeader(const String &title)
{
  String html;

  html += F("<!doctype html><html><head>");
  html += F("<meta charset='utf-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>");
  html += htmlEscape(title);
  html += F("</title>");
  html += F(
    "<style>"
    "body{font-family:system-ui,sans-serif;max-width:1000px;margin:32px auto;padding:0 16px;background:#f6f7f9;color:#222}"
    "h1,h2{margin-bottom:.4em}"
    ".topnav{display:flex;flex-wrap:wrap;gap:8px;margin:0 0 20px}"
    ".topnav a{background:#fff;border:1px solid #ccc;border-radius:7px;padding:8px 11px;text-decoration:none;color:#222}"
    ".card{background:white;border:1px solid #ddd;border-radius:12px;padding:18px;margin:16px 0}"
    "table{border-collapse:collapse;width:100%}"
    "td,th{text-align:left;padding:7px;border-bottom:1px solid #eee}"
    "input,select{width:100%;box-sizing:border-box;padding:9px;margin:4px 0 12px;border:1px solid #bbb;border-radius:6px;background:white}"
    "input[type=checkbox]{width:auto}"
    "button,.button{display:inline-block;background:#222;color:white;padding:9px 14px;border:0;border-radius:7px;text-decoration:none;cursor:pointer}"
    ".muted{color:#666}"
    ".ok{color:#087a2c;font-weight:600}.bad{color:#a00;font-weight:600}"
    "code{background:#eee;padding:2px 5px;border-radius:4px}"
    "</style>"
  );
  html += F("</head><body>");
  html += webNav();

  return html;
}

String htmlFooter()
{
  return F("</body></html>");
}

String webDeviceTable()
{
  String html;
  html.reserve(5000);

  html += F("<table><tr><th>"); html += T("Adresse", "Address");
  html += F("</th><th>"); html += T("Name", "Name");
  html += F("</th><th>"); html += T("Modell", "Model");
  html += F("</th><th>"); html += T("Seriennummer", "Serial number");
  html += F("</th><th>"); html += T("Statuswerte", "Known states");
  html += F("</th><th></th></tr>");

  for (uint8_t i = 0; i < MAX_DEVICES; i++)
  {
    HM485Device &d = devices[i];
    if (!d.used) continue;

    uint8_t known = 0;
    for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++)
      if (d.channel[ch].known) known++;

    html += F("<tr><td><code>"); html += hexAddress(d.address);
    html += F("</code></td><td>"); html += htmlEscape(effectiveDeviceName(&d));
    html += F("</td><td>"); html += d.profile ? htmlEscape(String(d.profile->model)) : String(T("unbekannt", "unknown"));
    if (d.profile) { html += F("<br><span class='muted'>"); html += htmlEscape(String(deviceSupportName(d.profile->support))); html += F("</span>"); }
    html += F("</td><td>"); html += d.serialKnown ? htmlEscape(String(d.serialNumber)) : String("-");
    html += F("</td><td>");
    html += String(known);
    html += F(" / ");
    html += String(deviceChannelCount(&d));
    html += F("</td><td><a class='button' href='/device?addr="); html += hexAddress(d.address);
    html += F("'>"); html += T("Profile", "Profiles");
    html += F("</a> <a class='button' href='/device_status?addr="); html += hexAddress(d.address);
    html += F("'>"); html += T("Status", "Status"); html += F("</a></td></tr>");
  }

  html += F("</table>");
  return html;
}

static HM485Device *deviceFromWebArg()
{
  if (!webServer.hasArg("addr"))
    return nullptr;
  String value = webServer.arg("addr");
  value.trim();
  if (value.length() != 8)
    return nullptr;
  char *end = nullptr;
  uint32_t address = strtoul(value.c_str(), &end, 16);
  if (!end || *end != 0)
    return nullptr;
  return getDevice(address, false);
}

static bool webChannelFromArgs(HM485Device *&device, uint8_t &ch)
{
  device = deviceFromWebArg();
  if (!device || !webServer.hasArg("ch"))
    return false;
  int ci = webServer.arg("ch").toInt();
  if (ci < 0 || ci >= MAX_CHANNELS)
    return false;
  ch = (uint8_t)ci;
  return findChannelProfile(device->profile, ch) != nullptr;
}

static bool semanticAllowedForBehaviour(SemanticProfile sp, ChannelBehaviour behaviour)
{
  if (sp == SemanticProfile::AUTO)
    return true;
  switch (behaviour)
  {
    case ChannelBehaviour::CONTACT:
    case ChannelBehaviour::DIGITAL_INPUT:
      return sp == SemanticProfile::WINDOW || sp == SemanticProfile::DOOR ||
             sp == SemanticProfile::ALARM || sp == SemanticProfile::CONTACT ||
             sp == SemanticProfile::BINARY_INPUT || sp == SemanticProfile::RAW_SENSOR;
    case ChannelBehaviour::FREQUENCY_INPUT:
      return sp == SemanticProfile::FREQUENCY_SENSOR || sp == SemanticProfile::RAW_SENSOR;
    case ChannelBehaviour::ANALOG_INPUT:
      return sp == SemanticProfile::ANALOG_SENSOR || sp == SemanticProfile::RAW_SENSOR ||
             sp == SemanticProfile::ANALOG_VOLTAGE_RAW ||
             sp == SemanticProfile::ANALOG_CURRENT_RAW ||
             sp == SemanticProfile::ANALOG_POWER_RAW ||
             sp == SemanticProfile::ANALOG_TEMPERATURE_RAW ||
             sp == SemanticProfile::ANALOG_RESISTANCE_RAW ||
             sp == SemanticProfile::ANALOG_PRESSURE_RAW;
    case ChannelBehaviour::DIGITAL_OUTPUT:
      return sp == SemanticProfile::OUTPUT_SWITCH || sp == SemanticProfile::OUTPUT_BUTTON ||
             sp == SemanticProfile::OUTPUT_MONITOR;
    case ChannelBehaviour::ANALOG_OUTPUT: // FHEM name; physically frequency/PWM output
      return sp == SemanticProfile::OUTPUT_MONITOR || sp == SemanticProfile::RAW_SENSOR;
    default:
      return true;
  }
}

static const char *behaviourModeToken(ChannelBehaviour b)
{
  switch (b)
  {
    case ChannelBehaviour::DIGITAL_OUTPUT:  return "do";
    case ChannelBehaviour::ANALOG_OUTPUT:   return "fo";
    case ChannelBehaviour::CONTACT:
    case ChannelBehaviour::DIGITAL_INPUT:   return "di";
    case ChannelBehaviour::FREQUENCY_INPUT: return "fi";
    case ChannelBehaviour::ANALOG_INPUT:    return "ai";
    default:                                return "";
  }
}

void handleWebDevice()
{
  if (!requireWebAuth()) return;
  HM485Device *device = deviceFromWebArg();
  if (!device)
  {
    webServer.send(404, "text/plain; charset=utf-8", T("Gerät nicht gefunden", "Device not found"));
    return;
  }

  String html = htmlHeader(T("HM485 Gerät", "HM485 device"));
  html += F("<h1>"); html += htmlEscape(effectiveDeviceName(device)); html += F("</h1>");
  html += F("<div class='card'><form method='post' action='/device_save'>");
  html += F("<input type='hidden' name='addr' value='"); html += hexAddress(device->address); html += F("'>");
  html += F("<p><strong>"); html += T("Adresse", "Address"); html += F(":</strong> <code>"); html += hexAddress(device->address); html += F("</code><br>");
  html += F("<strong>"); html += T("Modell", "Model"); html += F(":</strong> ");
  html += device->profile ? htmlEscape(String(device->profile->model)) : String(T("unbekannt", "unknown"));
  if (device->profile) { html += F(" <span class='muted'>["); html += htmlEscape(String(deviceSupportName(device->profile->support))); html += F("]</span>"); }
  html += F("<br><strong>"); html += T("Seriennummer", "Serial number"); html += F(":</strong> ");
  html += device->serialKnown ? htmlEscape(String(device->serialNumber)) : String("-"); html += F("</p>");
  html += F("<label>"); html += T("Gerätename", "Device name"); html += F("</label><input name='device_name' maxlength='31' value='");
  html += htmlEscape(String(device->friendlyName)); html += F("' placeholder='"); html += htmlEscape(effectiveDeviceName(device)); html += F("'>");
  html += F("<button type='submit'>"); html += T("Gerätename speichern", "Save device name"); html += F("</button></form></div>");

  uint8_t count = 0;
  if (device->profile)
    for (uint8_t g=0; g<device->profile->channelGroupCount; g++)
    { uint8_t e=device->profile->channels[g].first+device->profile->channels[g].count; if(e>count) count=e; }

  html += F("<div class='card'><h2>"); html += T("Kanäle", "Channels"); html += F("</h2><table><tr><th>");
  html += T("Kanal", "Channel"); html += F("</th><th>Name</th><th>"); html += T("Hardware-Modus", "Hardware mode");
  html += F("</th><th>"); html += T("HA-Profil", "HA profile"); html += F("</th><th>Status</th><th></th></tr>");
  for (uint8_t ch=0; ch<count; ch++)
  {
    const ChannelProfile *cp=findChannelProfile(device->profile,ch);
    if (!cp) continue;
    html += F("<tr><td>"); html += String(busToHmWiredChannel(ch)); html += F("</td><td>");
    html += htmlEscape(effectiveChannelName(device,ch)); html += F("</td><td>");
    if (device->channel[ch].behaviourKnown) html += htmlEscape(String(behaviourName(device->channel[ch].behaviour)));
    else html += htmlEscape(String(channelTypeName(cp->type)));
    html += F("</td><td>"); html += htmlEscape(String(semanticProfileLabel(channelSemanticProfile(device,ch)))); html += F("</td><td>");
    if (!device->channel[ch].known) html += F("-");
    else if (channelIsBinary(device,ch)) html += channelLogicalOn(device,ch) ? F("<span class='ok'>ON</span>") : F("<span class='bad'>OFF</span>");
    else { html += String(device->channel[ch].value); html += F(" <span class='muted'>(0x"); html += String(device->channel[ch].value,HEX); html += F(")</span>"); }
    html += F("</td><td><a class='button' href='/channel?addr="); html += hexAddress(device->address); html += F("&ch="); html += String(ch); html += F("'>");
    html += T("Konfigurieren", "Configure"); html += F("</a></td></tr>");
  }
  html += F("</table><p><a class='button' href='/device_status?addr="); html += hexAddress(device->address); html += F("'>"); html += T("Status", "Status");
  html += F("</a> <a class='button' href='/'>"); html += T("Zurück", "Back"); html += F("</a></p></div>");
  html += F("<div class='card muted'>");
  html += T("Jeder Port wird jetzt auf einer eigenen Seite konfiguriert. Es werden nur Einstellungen angezeigt, die zum gewählten Hardware-Modus passen.",
            "Each port is now configured on its own page. Only settings relevant to the selected hardware mode are shown.");
  html += F("</div>");

  html += F("<div class='card'><h2>"); html += T("Gerät vergessen", "Forget device"); html += F("</h2>");
  html += F("<p class='muted'>");
  html += T("Entfernt dieses Gerät aus der persistenten Gerätedatenbank und löscht seine retained Home-Assistant-Discovery-Einträge. Wenn das Gerät später wieder Busverkehr sendet oder entdeckt wird, kann es erneut angelegt werden.",
            "Removes this device from the persistent device database and clears its retained Home Assistant discovery entries. If it later sends bus traffic or is discovered again, it can be learned again.");
  html += F("</p><form method='post' action='/device_forget' onsubmit=\"return confirm('");
  html += T("Gerät wirklich vergessen?", "Really forget this device?");
  html += F("');\"><input type='hidden' name='addr' value='"); html += hexAddress(device->address); html += F("'>");
  html += F("<button type='submit' style='background:#a00'>"); html += T("Gerät vergessen", "Forget device"); html += F("</button></form></div>");
  html += htmlFooter();
  webServer.send(200,"text/html; charset=utf-8",html);
}


void handleWebDeviceForget()
{
  if (!requireWebAuth())
    return;

  HM485Device *device = deviceFromWebArg();
  if (!device)
  {
    webServer.send(404, "text/plain; charset=utf-8", T("Gerät nicht gefunden", "Device not found"));
    return;
  }

  const uint32_t address = device->address;
  if (!forgetDevice(address))
  {
    webServer.send(409, "text/plain; charset=utf-8",
                   T("Gerät ist gerade in einer Bus-Transaktion aktiv. Bitte erneut versuchen.",
                     "Device is currently involved in a bus transaction. Please try again."));
    return;
  }

  webServer.sendHeader("Location", "/");
  webServer.send(303, "text/plain", "");
}

void handleWebChannel()
{
  if (!requireWebAuth()) return;
  HM485Device *device=nullptr; uint8_t ch=0;
  if (!webChannelFromArgs(device,ch))
  { webServer.send(404,"text/plain; charset=utf-8",T("Kanal nicht gefunden","Channel not found")); return; }
  const ChannelProfile *cp=findChannelProfile(device->profile,ch);
  ChannelBehaviour current=device->channel[ch].behaviourKnown ? device->channel[ch].behaviour : cp->defaultBehaviour;

  String html=htmlHeader(T("HM485 Kanal konfigurieren","Configure HM485 channel"));
  html += F("<h1>"); html += T("Kanal ","Channel "); html += String(busToHmWiredChannel(ch)); html += F("</h1>");
  html += F("<div class='card'><p><strong>"); html += htmlEscape(effectiveDeviceName(device)); html += F("</strong><br>");
  html += T("Hardware-Typ", "Hardware type"); html += F(": "); html += htmlEscape(String(channelTypeName(cp->type)));
  html += F("<br>"); html += T("Aktueller Modus", "Current mode"); html += F(": <strong>");
  html += device->channel[ch].behaviourKnown ? htmlEscape(String(behaviourName(device->channel[ch].behaviour))) : String(T("unbekannt","unknown"));
  html += F("</strong></p></div>");

  html += F("<form method='post' action='/channel_save' id='channelForm'><input type='hidden' name='addr' value='");
  html += hexAddress(device->address); html += F("'><input type='hidden' name='ch' value='"); html += String(ch); html += F("'>");

  html += F("<div class='card'><h2>"); html += T("Allgemein", "General"); html += F("</h2><label>Name</label><input name='name' maxlength='23' value='");
  html += htmlEscape(String(device->channelName[ch])); html += F("' placeholder='"); html += htmlEscape(effectiveChannelName(device,ch)); html += F("'>");

  bool configurable=(device->deviceType==0x001C && device->profile && device->profile->writeEnabled && cp->configurable);
  if (configurable)
  {
    html += F("<label>"); html += T("Hardware-Modus", "Hardware mode"); html += F("</label><select name='mode' id='mode'>");
    if (cp->type==ChannelType::DIGITAL_ANALOG_OUTPUT)
    {
      html += F("<option value='do'"); if(current==ChannelBehaviour::DIGITAL_OUTPUT) html += F(" selected"); html += F(">digital_output</option>");
      html += F("<option value='fo'"); if(current==ChannelBehaviour::ANALOG_OUTPUT) html += F(" selected"); html += F(">frequency_output (FHEM: analog_output)</option>");
    }
    else if (cp->type==ChannelType::DIGITAL_FREQUENCY_INPUT)
    {
      html += F("<option value='di'"); if(current==ChannelBehaviour::DIGITAL_INPUT) html += F(" selected"); html += F(">digital_input</option>");
      html += F("<option value='fi'"); if(current==ChannelBehaviour::FREQUENCY_INPUT) html += F(" selected"); html += F(">frequency_input</option>");
    }
    else if (cp->type==ChannelType::DIGITAL_ANALOG_INPUT)
    {
      html += F("<option value='di'"); if(current==ChannelBehaviour::DIGITAL_INPUT) html += F(" selected"); html += F(">digital_input</option>");
      html += F("<option value='ai'"); if(current==ChannelBehaviour::ANALOG_INPUT) html += F(" selected"); html += F(">analog_input</option>");
    }
    html += F("</select>");
  }
  else
  {
    html += F("<input type='hidden' name='mode' id='mode' value='"); html += behaviourModeToken(current); html += F("'>");
  }

  html += F("<label>"); html += T("Home-Assistant-Profil", "Home Assistant profile"); html += F("</label><select name='profile' id='profile'>");
  SemanticProfile effective=channelSemanticProfile(device,ch);
  // Render all known semantic profiles, tagged by allowed hardware mode. JS hides irrelevant options.
  for (uint8_t v=0; v<=static_cast<uint8_t>(SemanticProfile::ANALOG_PRESSURE_RAW); v++)
  {
    SemanticProfile sp=static_cast<SemanticProfile>(v);
    String modes;
    if (semanticAllowedForBehaviour(sp,ChannelBehaviour::DIGITAL_INPUT)) modes += " di";
    if (semanticAllowedForBehaviour(sp,ChannelBehaviour::FREQUENCY_INPUT)) modes += " fi";
    if (semanticAllowedForBehaviour(sp,ChannelBehaviour::ANALOG_INPUT)) modes += " ai";
    if (semanticAllowedForBehaviour(sp,ChannelBehaviour::DIGITAL_OUTPUT)) modes += " do";
    if (semanticAllowedForBehaviour(sp,ChannelBehaviour::ANALOG_OUTPUT)) modes += " fo";
    html += F("<option value='"); html += String(v); html += F("' data-modes='"); html += modes; html += F("'");
    if (sp==effective) html += F(" selected");
    html += F(">"); html += htmlEscape(String(semanticProfileLabel(sp))); html += F("</option>");
  }
  html += F("</select></div>");

  // Digital input settings only.
  html += F("<div class='card modebox' data-show='di'><h2>"); html += T("Digitaler Eingang", "Digital input"); html += F("</h2>");
  html += F("<label><input type='checkbox' name='invert' value='1'"); if(device->invertLogic[ch]) html += F(" checked"); html += F("> ");
  html += T("Logik invertieren (NO/NC)","Invert logic (NO/NC)"); html += F("</label></div>");

  // Digital output settings only; gateway-level pulse logic is shown only for Button profile by JS.
  html += F("<div class='card modebox' data-show='do'><h2>"); html += T("Digitaler Ausgang", "Digital output"); html += F("</h2>");
  html += F("<div id='buttonSettings'><label>"); html += T("Impulsdauer (s)","Pulse duration (s)"); html += F("</label><input type='number' min='0.1' max='3600' step='0.1' name='pulse' value='");
  html += String((device->buttonPulseMs[ch]?device->buttonPulseMs[ch]:2000)/1000.0f,1); html += F("'>");
  html += F("<label><input type='checkbox' name='rest_on' value='1'"); if(device->buttonRestOn[ch]) html += F(" checked"); html += F("> ");
  html += T("Ruhezustand EIN", "Rest state ON"); html += F("</label><p class='muted'>");
  html += T("Nur beim Profil Button/Impuls relevant.","Only relevant for the button/pulse profile."); html += F("</p></div></div>");

  // Frequency output settings only.
  if (cp->type==ChannelType::DIGITAL_ANALOG_OUTPUT)
  {
    uint16_t pr=0; float ps=0; getPulseTime(device,ch,pr,ps);
    float currentHz=(device->channel[ch].known && current==ChannelBehaviour::ANALOG_OUTPUT) ? device->channel[ch].value/1000.0f : 0.0f;
    html += F("<div class='card modebox' data-show='fo'><h2>"); html += T("Frequenzausgang", "Frequency output"); html += F("</h2>");
    html += F("<p class='muted'>"); html += T("FHEM nennt diesen Modus analog_output. Der Wert ist eine Frequenz in mHz; die Oberfläche verwendet Hz.","FHEM calls this mode analog_output. The wire value is a frequency in mHz; this UI uses Hz."); html += F("</p>");
    html += F("<label>"); html += T("Frequenz (Hz)","Frequency (Hz)"); html += F("</label><input type='number' min='0' max='65.535' step='0.001' name='frequency' value='"); html += String(currentHz,3); html += F("'>");
    html += F("<label>"); html += T("HMW Pulszeit (s)","HMW pulse time (s)"); html += F("</label><input type='number' min='0' max='655.35' step='0.01' name='hmw_pulse' value='"); html += String(ps,2); html += F("'>");
    html += F("</div>");
  }

  // Frequency input has no unrelated binary/output options.
  html += F("<div class='card modebox' data-show='fi'><h2>"); html += T("Frequenzeingang", "Frequency input"); html += F("</h2><p class='muted'>");
  html += T("Der Messwert wird als Frequenzsensor veröffentlicht. Keine Invertierung, Impuls- oder Kalibrierparameter erforderlich.","The measured value is published as a frequency sensor. No inversion, pulse or calibration parameters are needed."); html += F("</p></div>");

  if (cp->type==ChannelType::DIGITAL_ANALOG_INPUT)
  {
    int cal=0; uint16_t ca=0x000A+(ch-20); if(ca<EEPROM_CACHE_SIZE && device->eepromValid[ca]) cal=(int)device->eeprom[ca]-0x7F;
    html += F("<div class='card modebox' data-show='ai'><h2>"); html += T("Analogeingang", "Analog input"); html += F("</h2><label>");
    html += T("Kalibrierung", "Calibration"); html += F("</label><input type='number' min='-127' max='128' step='1' name='calibration' value='"); html += String(cal); html += F("'>");
    html += F("<p class='muted'>EEPROM-Kodierung: 0x7F + Kalibrierwert. ");
    html += T("Die HA-Profile Spannung/Strom/Leistung/Temperatur/Widerstand/Druck sind bewusst nur semantische Rohwert-Profile. Solange die physikalische Skalierung des HMW-Eingangs nicht verifiziert ist, werden keine falschen Einheiten oder Umrechnungen behauptet.",
              "The HA voltage/current/power/temperature/resistance/pressure profiles intentionally remain semantic raw-value profiles. Until the physical HMW input scaling is verified, the gateway does not claim units or apply conversions.");
    html += F("</p></div>");
  }

  html += F("<div class='card'><button type='submit' name='save_only' value='1'>"); html += T("Speichern", "Save"); html += F("</button>");
  if (configurable)
  {
    html += F(" <button type='submit' name='apply_hw' value='1'>"); html += T("Hardware-Konfiguration übernehmen", "Apply hardware configuration"); html += F("</button>");
    html += F("<p class='muted'>"); html += T("Speichern ändert nur Name/HA-Profil und Gateway-Optionen. Hardware-Konfiguration übernehmen schreibt nur die für diesen Port gültigen HMW-EEPROM-Werte, führt 0x43 aus und verifiziert per Read-back.",
      "Save changes only name/HA profile and gateway options. Apply hardware configuration writes only the valid HMW EEPROM values for this port, executes 0x43 and verifies via read-back."); html += F("</p>");
  }
  html += F("<a class='button' href='/device?addr="); html += hexAddress(device->address); html += F("'>"); html += T("Zurück","Back"); html += F("</a></div></form>");

  // Tiny client-side view logic: irrelevant sections disappear immediately when mode/profile changes.
  html += F("<script>(function(){const m=document.getElementById('mode'),p=document.getElementById('profile');function u(){const mode=m?m.value:'';document.querySelectorAll('.modebox').forEach(x=>x.style.display=x.dataset.show===mode?'block':'none');if(p){let first=null,selok=false;Array.from(p.options).forEach(o=>{const ok=(' '+o.dataset.modes+' ').indexOf(' '+mode+' ')>=0;o.hidden=!ok;o.disabled=!ok;if(ok&&!first)first=o;if(ok&&o.selected)selok=true;});if(!selok&&first)first.selected=true;}const b=document.getElementById('buttonSettings');if(b)b.style.display=(mode==='do'&&p&&p.options[p.selectedIndex]&&p.value==='12')?'block':'none';}if(m)m.addEventListener('change',u);if(p)p.addEventListener('change',u);u();})();</script>");
  html += htmlFooter();
  webServer.send(200,"text/html; charset=utf-8",html);
}

void handleWebDeviceSave()
{
  if (!requireWebAuth()) return;
  HM485Device *device=deviceFromWebArg();
  if (!device) { webServer.send(404,"text/plain; charset=utf-8",T("Gerät nicht gefunden","Device not found")); return; }
  if (webServer.hasArg("device_name")) copyStringToBuffer(webServer.arg("device_name"),device->friendlyName,sizeof(device->friendlyName));
  saveDeviceMetadata(device);
  mqttPublishDiscoveryAll(); mqttPublishAllStates();
  webServer.sendHeader("Location",String("/device?addr=")+hexAddress(device->address));
  webServer.send(303,"text/plain","");
}

void handleWebChannelSave()
{
  if (!requireWebAuth()) return;
  HM485Device *device=nullptr; uint8_t ch=0;
  if (!webChannelFromArgs(device,ch)) { webServer.send(404,"text/plain; charset=utf-8",T("Kanal nicht gefunden","Channel not found")); return; }
  const ChannelProfile *cp=findChannelProfile(device->profile,ch);

  if (webServer.hasArg("name")) copyStringToBuffer(webServer.arg("name"),device->channelName[ch],sizeof(device->channelName[ch]));
  String mode=webServer.hasArg("mode")?webServer.arg("mode"):String(behaviourModeToken(device->channel[ch].behaviour));
  ChannelBehaviour desired=ChannelBehaviour::UNKNOWN;
  if(mode=="do") desired=ChannelBehaviour::DIGITAL_OUTPUT;
  else if(mode=="fo") desired=ChannelBehaviour::ANALOG_OUTPUT;
  else if(mode=="di") desired=ChannelBehaviour::DIGITAL_INPUT;
  else if(mode=="fi") desired=ChannelBehaviour::FREQUENCY_INPUT;
  else if(mode=="ai") desired=ChannelBehaviour::ANALOG_INPUT;

  // A plain Save must never silently pretend that a not-yet-applied hardware
  // mode is already active. Use the real/current mode for NVS-only settings;
  // only Apply hardware configuration uses the newly selected mode.
  ChannelBehaviour current = device->channel[ch].behaviourKnown ? device->channel[ch].behaviour : cp->defaultBehaviour;
  ChannelBehaviour uiBehaviour = webServer.hasArg("apply_hw") ? desired : current;

  if (webServer.hasArg("profile"))
  {
    int v=webServer.arg("profile").toInt();
    if(v>=0 && v<=static_cast<int>(SemanticProfile::ANALOG_PRESSURE_RAW))
    {
      SemanticProfile requested=static_cast<SemanticProfile>(v);
      device->semanticProfile[ch]=semanticAllowedForBehaviour(requested,uiBehaviour) ? requested : SemanticProfile::AUTO;
    }
  }

  // Save only settings that make sense for the active (or explicitly applied) mode.
  if (uiBehaviour==ChannelBehaviour::DIGITAL_INPUT || uiBehaviour==ChannelBehaviour::CONTACT)
    device->invertLogic[ch]=webServer.hasArg("invert");
  else
    device->invertLogic[ch]=false;

  if (uiBehaviour==ChannelBehaviour::DIGITAL_OUTPUT)
  {
    if(webServer.hasArg("pulse"))
    {
      float sec=webServer.arg("pulse").toFloat(); if(sec<0.1f)sec=0.1f; if(sec>3600.0f)sec=3600.0f;
      device->buttonPulseMs[ch]=(uint32_t)(sec*1000.0f+0.5f);
    }
    device->buttonRestOn[ch]=webServer.hasArg("rest_on");
  }
  else
  {
    device->buttonRestOn[ch]=false;
  }

  saveDeviceMetadata(device);
  webLogAdd(String("[NVS] Channel metadata saved: ")+hexAddress(device->address)+" ch="+String(busToHmWiredChannel(ch)));
  mqttPublishDiscoveryAll(); mqttSubscribeOutputCommands(); mqttPublishAllStates();

  if (webServer.hasArg("apply_hw"))
  {
    if(device->deviceType!=0x001C || !device->profile || !device->profile->writeEnabled || !cp || !cp->configurable)
    { webServer.send(403,"text/plain; charset=utf-8",T("EEPROM-Konfiguration ist für diesen Kanal nicht freigegeben.","EEPROM configuration is not enabled for this channel.")); return; }

    bool writeParam=false; uint16_t paramRaw=0; bool setFreq=false; uint16_t freqMilliHz=0;
    if(cp->type==ChannelType::DIGITAL_ANALOG_OUTPUT && desired==ChannelBehaviour::ANALOG_OUTPUT)
    {
      // On the frequency-output page these are actual mode parameters, not opt-in checkboxes.
      if(webServer.hasArg("hmw_pulse"))
      { float sec=webServer.arg("hmw_pulse").toFloat(); if(sec<0)sec=0; if(sec>655.35f)sec=655.35f; paramRaw=(uint16_t)(sec*100.0f+0.5f); writeParam=true; }
      if(webServer.hasArg("frequency"))
      { float hz=webServer.arg("frequency").toFloat(); if(hz<0)hz=0; if(hz>65.535f)hz=65.535f; freqMilliHz=(uint16_t)(hz*1000.0f+0.5f); setFreq=true; }
    }
    else if(cp->type==ChannelType::DIGITAL_ANALOG_INPUT && desired==ChannelBehaviour::ANALOG_INPUT)
    {
      int cal=webServer.hasArg("calibration")?webServer.arg("calibration").toInt():0; if(cal<-127)cal=-127; if(cal>128)cal=128;
      paramRaw=(uint8_t)(0x7F+cal); writeParam=true;
    }

    if(!startIoConfiguration(device,ch,desired,writeParam,paramRaw,setFreq,freqMilliHz))
    {
      String reason;
      if (rawRxOnlyMode) reason = "RAW RX Only aktiv";
      else if (addressGuardBootActive) reason = "Adressprüfung nach Boot noch aktiv";
      else if (addressConflictLocked) reason = "Adresskonflikt: TX gesperrt";
      else if (ioConfig.active) reason = "andere EEPROM-Konfiguration läuft";
      else if (outputWrite.active) reason = "Ausgangstransaktion läuft";
      else if (buttonPulse.active) reason = "Button/Impuls-Transaktion läuft";
      else if (pending.active) reason = "HM485-Anfrage läuft";
      else if (scanActive) reason = "Gerätescan läuft";
      else if (statusPollActive) reason = "Statusabfrage läuft";
      else if (activeDiscovery.running) reason = "Discovery läuft";
      else if (passiveCaptureActive || passiveCaptureArmRequested) reason = "Passive Capture aktiv";
      else if (quietRootRequested) reason = "Discovery/Quiet-Root aktiv";
      else if (device->deviceType!=0x001C || !device->profile || !device->profile->writeEnabled)
        reason = "EEPROM-Schreiben für dieses Gerät nicht freigegeben";
      else if (!cp || !cp->configurable || !cp->behaviourEeprom.available)
        reason = "Kanal besitzt keine freigegebene EEPROM-Moduskonfiguration";
      else
      {
        uint8_t off = ch - cp->first;
        uint16_t absBit = cp->behaviourEeprom.firstBit + off * cp->behaviourEeprom.bitStep;
        uint16_t addr = cp->behaviourEeprom.byteAddress + absBit / 8;
        if (addr >= EEPROM_CACHE_SIZE || !device->eepromValid[addr])
          reason = "benötigtes EEPROM-Byte nicht im Cache";
        else
          reason = "Bus momentan nicht bereit";
      }

      String msg = String("[IO CONFIG BLOCKED] ") + hexAddress(device->address) +
                   " ch=" + String(busToHmWiredChannel(ch)) + ": " + reason;
      Serial.println(msg);
      webLogAdd(msg);

      webServer.send(409,"text/plain; charset=utf-8",
        String(T("Konfiguration konnte nicht gestartet werden: ",
                 "Configuration could not start: ")) + reason);
      return;
    }
  }

  webServer.sendHeader("Location",String("/channel?addr=")+hexAddress(device->address)+"&ch="+String(ch));
  webServer.send(303,"text/plain","");
}

void handleWebDeviceStatus()
{
  if (!requireWebAuth()) return;
  HM485Device *device = deviceFromWebArg();
  if (!device)
  {
    webServer.send(404, "text/plain; charset=utf-8", T("Gerät nicht gefunden", "Device not found"));
    return;
  }

  String html = htmlHeader(T("HM485 Kanalstatus", "HM485 channel status"));
  html += F("<meta http-equiv='refresh' content='3'>");
  html += F("<h1>"); html += T("Kanalstatus", "Channel status"); html += F("</h1><div class='card'><p><strong>");
  html += htmlEscape(effectiveDeviceName(device));
  html += F("</strong><br><code>"); html += hexAddress(device->address); html += F("</code>");
  if (device->serialKnown) { html += F(" &nbsp; "); html += htmlEscape(String(device->serialNumber)); }
  html += F("</p><table><tr><th>"); html += T("Kanal", "Channel");
  html += F("</th><th>Name</th><th>"); html += T("Profil", "Profile");
  html += F("</th><th>"); html += T("Invertiert", "Inverted");
  html += F("</th><th>"); html += T("Rohwert", "Raw value");
  html += F("</th><th>Status</th><th>"); html += T("Alter", "Age"); html += F("</th></tr>");

  uint8_t count = 0;
  if (device->profile)
    for (uint8_t g=0; g<device->profile->channelGroupCount; g++)
    { uint8_t e=device->profile->channels[g].first+device->profile->channels[g].count; if(e>count) count=e; }

  for (uint8_t ch=0; ch<count; ch++)
  {
    html += F("<tr><td>"); html += String(busToHmWiredChannel(ch)); html += F("</td><td>");
    html += htmlEscape(effectiveChannelName(device,ch)); html += F("</td><td>");
    html += htmlEscape(String(semanticProfileLabel(channelSemanticProfile(device,ch)))); html += F("</td><td>");
    html += device->invertLogic[ch] ? T("Ja","Yes") : T("Nein","No"); html += F("</td><td>");
    if (device->channel[ch].known)
    { html += String(device->channel[ch].value); html += F(" <span class='muted'>(0x"); html += String(device->channel[ch].value,HEX); html += F(")</span>"); }
    else html += F("-");
    html += F("</td><td>");
    if (!device->channel[ch].known) html += F("<span class='muted'>"); html += !device->channel[ch].known ? T("unbekannt","unknown") : ""; if (!device->channel[ch].known) html += F("</span>");
    else if (channelIsBinary(device,ch)) html += channelLogicalOn(device,ch) ? F("<span class='ok'>ON</span>") : F("<span class='bad'>OFF</span>");
    else html += String(device->channel[ch].value);
    html += F("</td><td>");
    if (device->channel[ch].known && device->channel[ch].lastUpdate) html += String((millis()-device->channel[ch].lastUpdate)/1000)+" s"; else html += F("-");
    html += F("</td></tr>");
  }
  html += F("</table><p><a class='button' href='/status_refresh?addr="); html += hexAddress(device->address);
  html += F("'>"); html += T("Status neu abfragen", "Refresh status"); html += F("</a> <a class='button' href='/device?addr="); html += hexAddress(device->address);
  html += F("'>"); html += T("Profile", "Profiles"); html += F("</a> <a class='button' href='/'>"); html += T("Zurück","Back"); html += F("</a></p>");
  html += F("<p class='muted'>"); html += T("Die Seite aktualisiert sich alle 3 Sekunden. Rohwert ist der unveränderte HM485-Wert; Status berücksichtigt die NO/NC-Invertierung.", "The page refreshes every 3 seconds. Raw value is the unchanged HM485 value; Status includes the configured NO/NC inversion."); html += F("</p></div>");
  html += htmlFooter();
  webServer.send(200,"text/html; charset=utf-8",html);
}

void handleWebStatusRefresh()
{
  if (!requireWebAuth())
    return;
  HM485Device *device = deviceFromWebArg();
  if (!device)
  {
    webServer.send(404, "text/plain; charset=utf-8", "Geraet nicht gefunden");
    return;
  }
  startStatusPoll();
  webServer.sendHeader("Location", String("/device_status?addr=") + hexAddress(device->address));
  webServer.send(303, "text/plain", "");
}

void handleWebRoot()
{
  String html = htmlHeader("HM485 Gateway");

  html += F("<h1>HM485 Gateway ");
  html += FW_VERSION;
  html += F("</h1>");
  html += F("<div class='muted'>HM-Wired RS485 → MQTT / Home Assistant</div>");

  html += F("<div class='card'><h2>"); html += T("Netzwerk", "Network"); html += F("</h2><table>");

  html += F("<tr><td>"); html += T("Aktiv", "Active"); html += F("</td><td><strong>");
  html += htmlEscape(activeNetworkName());
  html += F("</strong></td></tr>");

  html += F("<tr><td>Ethernet</td><td>");
  html += ethernetReady()
    ? String("<span class='ok'>") + T("verbunden","connected") + "</span>"
    : String("<span class='bad'>") + T("nicht verbunden","not connected") + "</span>";
  html += F("</td></tr>");

  html += F("<tr><td>Ethernet IP</td><td>");
  html += ethernetReady() ? ETH.localIP().toString() : String("-");
  html += F("</td></tr>");

  html += F("<tr><td>Ethernet MAC</td><td>");
  html += ethernetStarted ? ETH.macAddress() : String("-");
  html += F("</td></tr>");

  html += F("<tr><td>Wi-Fi Fallback</td><td>");
  if (WiFi.status() == WL_CONNECTED)
  {
    html += String("<span class='ok'>") + T("verbunden","connected") + "</span>";
  }
  else
  {
    html += String("<span class='bad'>") + T("nicht verbunden","not connected") + "</span>";
  }
  html += F("</td></tr>");

  html += F("<tr><td>SSID</td><td>");
  html += htmlEscape(WiFi.status() == WL_CONNECTED ? WiFi.SSID() : config.wifiSsid);
  html += F("</td></tr>");

  html += F("<tr><td>Aktive IP</td><td>");
  html += networkReady()
    ? activeLocalIP().toString()
    : String("-");
  html += F("</td></tr>");

  html += F("<tr><td>RSSI</td><td>");
  html += WiFi.status() == WL_CONNECTED
    ? String(WiFi.RSSI()) + " dBm"
    : String("-");
  html += F("</td></tr>");

  html += F("<tr><td>Setup-AP</td><td>");
  html += setupApActive
    ? htmlEscape(setupApSsid + " / " + WiFi.softAPIP().toString())
    : String(T("aus","off"));
  html += F("</td></tr>");

  html += F("</table></div>");

  html += F("<div class='card'><h2>MQTT</h2><table>");
  html += F("<tr><td>Status</td><td>");
  html += mqttClient.connected()
    ? String("<span class='ok'>") + T("verbunden","connected") + "</span>"
    : String("<span class='bad'>") + T("nicht verbunden","not connected") + "</span>";
  html += F("</td></tr>");
  html += F("<tr><td>Broker</td><td>");
  html += htmlEscape(config.mqttHost);
  html += ":";
  html += String(config.mqttPort);
  html += F("</td></tr>");
  html += F("<tr><td>Base Topic</td><td><code>");
  html += htmlEscape(config.mqttBaseTopic);
  html += F("</code></td></tr>");
  html += F("<tr><td>HA Discovery</td><td>");
  html += config.discoveryEnabled ? T("an","on") : T("aus","off");
  html += F("</td></tr>");
  html += F("</table></div>");

  html += F("<div class='card'><h2>HM485</h2><table>");
  html += F("<tr><td>"); html += T("Gateway-Adresse","Gateway address"); html += F("</td><td><code>");
  html += hexAddress(config.localAddress);
  html += F("</code></td></tr>");
  html += F("<tr><td>"); html += T("Adresse 00000001 passiv gesehen","Address 00000001 seen passively"); html += F("</td><td>");
  html += centralAddressOneSeen ? String("<span class='ok'>") + T("Ja","Yes") + "</span>" : String("<span class='muted'>") + T("Noch nicht","Not yet") + "</span>";
  html += F("</td></tr>");
  html += F("<tr><td>"); html += T("TX-Konfliktschutz","TX conflict guard"); html += F("</td><td>");
  if (addressConflictLocked)
  {
    html += F("<span class='bad'>"); html += T("GESPERRT - Adresskonflikt ","LOCKED - address conflict ");
    html += hexAddress(addressConflictSender);
    html += F("</span>");
  }
  else if (addressGuardBootActive)
    html += String("<span class='muted'>") + T("Boot-Prüfung / nur Empfang","Boot check / receive only") + "</span>";
  else
    html += String("<span class='ok'>") + T("aktiv, kein Konflikt erkannt","active, no conflict detected") + "</span>";
  html += F("</td></tr>");
  html += F("<tr><td>RX Frames</td><td>");
  html += String(statFrames);
  html += F("</td></tr>");
  html += F("<tr><td>"); html += T("CRC OK / Fehler","CRC OK / errors"); html += F("</td><td>");
  html += String(statCrcOk);
  html += " / ";
  html += String(statCrcError);
  html += F("</td></tr>");
  html += F("<tr><td>"); html += T("Experimentelle Aktor-Writes","Experimental actuator writes"); html += F("</td><td>");
  html += config.experimentalWritesEnabled ? String("<span class='bad'>") + T("AN","ON") + "</span>" : String("<span class='ok'>") + T("AUS","OFF") + "</span>";
  html += F("</td></tr>");

  html += F("<tr><td>TX Frames</td><td>");
  html += String(statTxFrames);
  html += F("</td></tr>");
  html += F("<tr><td>Timeouts / Retries</td><td>");
  html += String(statTimeouts);
  html += " / ";
  html += String(statRetries);
  html += F("</td></tr>");
  html += F("<tr><td>"); html += T("Passive Änderungen","Passive updates"); html += F("</td><td>");
  html += String(statPassiveUpdates);
  html += F("</td></tr>");
  html += F("</table>");
  html += F("</div>");

  html += F("<div class='card'><h2>"); html += T("Geräte","Devices"); html += F("</h2>");
  html += webDeviceTable();
  html += F("</div>");

  html += htmlFooter();

  webServer.send(
    200,
    "text/html; charset=utf-8",
    html
  );
}

void handleWebConfig()
{
  if (!requireWebAuth())
    return;
  String html = htmlHeader(T("HM485 Gateway Konfiguration", "HM485 Gateway Configuration"));

  html += F("<h1>"); html += T("Konfiguration", "Configuration"); html += F("</h1>");
  html += F("<form method='post' action='/save'>");

  html += F("<div class='card'><h2>"); html += T("Sprache", "Language"); html += F("</h2>");
  html += F("<label>"); html += T("Weboberfläche", "Web interface"); html += F("</label><select name='ui_lang'>");
  html += F("<option value='de'"); if (uiGerman()) html += F(" selected"); html += F(">Deutsch</option>");
  html += F("<option value='en'"); if (!uiGerman()) html += F(" selected"); html += F(">English</option></select>");
  html += F("<div class='muted'>"); html += T("Die Sprache beeinflusst nur die Weboberfläche; MQTT Topics und Home-Assistant unique_id bleiben unverändert.", "Language only affects the web interface; MQTT topics and Home Assistant unique_id remain unchanged."); html += F("</div></div>");

  html += F("<div class='card'><h2>Wi-Fi</h2>");

  html += F("<label>SSID</label>");
  html += F("<input name='wifi_ssid' value='");
  html += htmlEscape(config.wifiSsid);
  html += F("'>");

  html += F("<label>"); html += T("Passwort","Password"); html += F("</label>");
  html += F("<input type='password' name='wifi_pass' placeholder='"); html += T("leer = vorhandenes Passwort behalten","empty = keep current password"); html += F("'>");

  html += F("<label>Hostname</label>");
  html += F("<input name='hostname' value='");
  html += htmlEscape(config.hostname);
  html += F("'>");

  html += F("</div>");

  html += F("<div class='card'><h2>MQTT</h2>");

  html += F("<label>Broker / Host</label>");
  html += F("<input name='mqtt_host' value='");
  html += htmlEscape(config.mqttHost);
  html += F("'>");

  html += F("<label>Port</label>");
  html += F("<input type='number' min='1' max='65535' name='mqtt_port' value='");
  html += String(config.mqttPort);
  html += F("'>");

  html += F("<label>"); html += T("Benutzer","User"); html += F("</label>");
  html += F("<input name='mqtt_user' value='");
  html += htmlEscape(config.mqttUser);
  html += F("'>");

  html += F("<label>"); html += T("Passwort","Password"); html += F("</label>");
  html += F("<input type='password' name='mqtt_pass' placeholder='"); html += T("leer = vorhandenes Passwort behalten","empty = keep current password"); html += F("'>");

  html += F("<label>Client ID</label>");
  html += F("<input name='mqtt_id' value='");
  html += htmlEscape(config.mqttClientId);
  html += F("'>");

  html += F("<label>Base Topic</label>");
  html += F("<input name='mqtt_base' value='");
  html += htmlEscape(config.mqttBaseTopic);
  html += F("'>");

  html += F("</div>");

  html += F("<div class='card'><h2>Home Assistant</h2>");

  html += F("<label>Discovery Prefix</label>");
  html += F("<input name='ha_prefix' value='");
  html += htmlEscape(config.discoveryPrefix);
  html += F("'>");

  html += F("<label><input type='checkbox' name='ha_disc' value='1' ");
  if (config.discoveryEnabled)
    html += F("checked");
  html += F("> "); html += T("MQTT Discovery aktivieren","Enable MQTT Discovery"); html += F("</label>");

  html += F("</div>");

  html += F("<div class='card'><h2>HM485</h2>");
  html += F("<label>"); html += T("Eigene HM485-Adresse","Own HM485 address"); html += F("</label>");
  html += F("<input name='hm_addr' maxlength='8' pattern='[0-9A-Fa-f]{8}' value='");
  html += hexAddress(config.localAddress);
  html += F("'>");
  html += F("<div class='muted'>"); html += T("Regulär <code>00000001</code>. Andere Adressen nur für Parallelbetrieb, Sonderfälle oder Tests. Eine Adresse darf auf dem Bus nicht doppelt vergeben sein.","Normally <code>00000001</code>. Use other addresses only for parallel operation, special cases or tests. An address must never exist twice on the bus."); html += F("</div>");
  html += F("<div class='muted'>"); html += T("Sicherheitsfunktion: Nach jedem Boot lauscht das Gateway 8 Sekunden passiv. Wird die eigene Adresse als fremde Source erkannt, bleibt HM485-TX bis zur Adressänderung und zum Neustart gesperrt. ‘Nicht gesehen’ ist bei rein passiver Prüfung kein Beweis, dass eine Adresse frei ist.","Safety feature: after each boot the gateway listens passively for 8 seconds. If its own address is seen as a foreign source, HM485 TX remains locked until the address is changed and the gateway is restarted. ‘Not seen’ is not proof that an address is free."); html += F("</div>");
  html += F("<hr><label><input type='checkbox' name='exp_write' value='1' ");
  if (config.experimentalWritesEnabled) html += F("checked");
  html += F("> "); html += T("Experimentelle Schreibunterstützung für ungetestete Aktoren", "Experimental writes for untested actuators"); html += F("</label>");
  html += F("<div class='muted'>"); html += T("Standard AUS. Aktiviert ausschließlich dokumentierte Runtime-Schaltbefehle für HMW-LC-Sw2-DR, HMW-IO-12-Sw7-DR sowie HBW-LC-Sw8/HBW-LC-Sw-12. Keine freie EEPROM-Schreibfunktion und kein Cover/Dimmer-Write.", "Default OFF. Enables only documented runtime switch commands for HMW-LC-Sw2-DR, HMW-IO-12-Sw7-DR and HBW-LC-Sw8/HBW-LC-Sw-12. No unrestricted EEPROM writes and no cover/dimmer writes."); html += F("</div>");
  html += F("</div>");

  html += F("<div class='card'><h2>"); html += T("Web-Zugang","Web access"); html += F("</h2>");
  html += F("<div class='muted'>"); html += T("Schützt Konfiguration, Scan, Neustart und Firmware-Update per HTTP Basic Auth.","Protects configuration, scan, restart and firmware update using HTTP Basic Auth."); html += F("</div>");
  html += F("<label>"); html += T("Benutzername","Username"); html += F("</label>");
  html += F("<input name='web_user' value='");
  html += htmlEscape(config.webUser);
  html += F("'>");
  html += F("<label>"); html += T("Neues Passwort","New password"); html += F("</label>");
  html += F("<input type='password' name='web_pass' placeholder='"); html += T("leer = vorhandenes Passwort behalten","empty = keep current password"); html += F("'>");
  html += F("</div>");

  html += F("<button type='submit'>"); html += T("Speichern & Neustarten", "Save & restart"); html += F("</button>");
  html += F("</form>");
  html += F("<p><a href='/'>"); html += T("Zurück", "Back"); html += F("</a></p>");

  html += htmlFooter();

  webServer.send(
    200,
    "text/html; charset=utf-8",
    html
  );
}

void handleWebSave()
{
  if (!requireWebAuth())
    return;
  if (webServer.hasArg("ui_lang"))
  {
    String lang = webServer.arg("ui_lang"); lang.toLowerCase();
    if (lang == "de" || lang == "en") config.uiLanguage = lang;
  }

  if (webServer.hasArg("wifi_ssid"))
    config.wifiSsid = webServer.arg("wifi_ssid");

  if (
    webServer.hasArg("wifi_pass") &&
    webServer.arg("wifi_pass").length() > 0
  )
  {
    config.wifiPassword = webServer.arg("wifi_pass");
  }

  if (webServer.hasArg("hostname"))
    config.hostname = webServer.arg("hostname");

  if (webServer.hasArg("mqtt_host"))
    config.mqttHost = webServer.arg("mqtt_host");

  if (webServer.hasArg("mqtt_port"))
  {
    long port = webServer.arg("mqtt_port").toInt();

    if (port >= 1 && port <= 65535)
      config.mqttPort = (uint16_t)port;
  }

  if (webServer.hasArg("mqtt_user"))
    config.mqttUser = webServer.arg("mqtt_user");

  if (
    webServer.hasArg("mqtt_pass") &&
    webServer.arg("mqtt_pass").length() > 0
  )
  {
    config.mqttPassword = webServer.arg("mqtt_pass");
  }

  if (webServer.hasArg("mqtt_id"))
    config.mqttClientId = webServer.arg("mqtt_id");

  if (webServer.hasArg("mqtt_base"))
    config.mqttBaseTopic =
      normalizedTopic(webServer.arg("mqtt_base"));

  if (webServer.hasArg("ha_prefix"))
    config.discoveryPrefix =
      normalizedTopic(webServer.arg("ha_prefix"));

  config.discoveryEnabled =
    webServer.hasArg("ha_disc");

  config.experimentalWritesEnabled = webServer.hasArg("exp_write");

  if (webServer.hasArg("hm_addr"))
  {
    String a = webServer.arg("hm_addr");
    a.trim();
    char *end = nullptr;
    uint32_t parsed = strtoul(a.c_str(), &end, 16);
    if (a.length() == 8 && end && *end == 0 && parsed != 0 && parsed != 0xFFFFFFFFUL)
      config.localAddress = parsed;
  }

  if (webServer.hasArg("web_user"))
    config.webUser = webServer.arg("web_user");

  if (
    webServer.hasArg("web_pass") &&
    webServer.arg("web_pass").length() > 0
  )
  {
    config.webPassword = webServer.arg("web_pass");
  }

  if (config.localAddress == 0 || config.localAddress == 0xFFFFFFFFUL)
    config.localAddress = DEFAULT_LOCAL_ADDRESS;

  if (config.webUser.length() == 0)
    config.webUser = "admin";

  if (config.webPassword.length() == 0)
    config.webPassword = "hm485setup";

  if (config.mqttClientId.length() == 0)
    config.mqttClientId = "hm485-gateway";

  if (config.mqttBaseTopic.length() == 0)
    config.mqttBaseTopic = "hm485";

  if (config.discoveryPrefix.length() == 0)
    config.discoveryPrefix = "homeassistant";

  if (config.hostname.length() == 0)
    config.hostname = "hm485-gateway";

  config.uiLanguage.toLowerCase();
  if (config.uiLanguage != "de" && config.uiLanguage != "en") config.uiLanguage = "de";

  saveConfig();

  webServer.send(
    200,
    "text/html; charset=utf-8",
    String("<html><body><h1>") + T("Gespeichert", "Saved") + "</h1><p>" + T("Gateway startet neu.", "Gateway is restarting.") + "</p></body></html>"
  );

  delay(800);
  ESP.restart();
}

static String backupEscape(const String &in)
{
  const char hex[] = "0123456789ABCDEF";
  String out; out.reserve(in.length() * 2 + 8);
  for (size_t i=0; i<in.length(); i++)
  {
    uint8_t c=(uint8_t)in[i];
    if ((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-'||c=='_'||c=='.') out+=(char)c;
    else { out += '%'; out += hex[c>>4]; out += hex[c&15]; }
  }
  return out;
}

static int hexNibble(char c)
{
  if (c>='0'&&c<='9') return c-'0';
  if (c>='A'&&c<='F') return c-'A'+10;
  if (c>='a'&&c<='f') return c-'a'+10;
  return -1;
}

static String backupUnescape(const String &in)
{
  String out; out.reserve(in.length());
  for (size_t i=0; i<in.length(); i++)
  {
    if (in[i]=='%' && i+2<in.length())
    {
      int a=hexNibble(in[i+1]), b=hexNibble(in[i+2]);
      if (a>=0 && b>=0) { out += (char)((a<<4)|b); i+=2; continue; }
    }
    out += in[i];
  }
  return out;
}

static String bytesToHex(const uint8_t *data, size_t len)
{
  const char h[]="0123456789ABCDEF";
  String out; out.reserve(len*2);
  for(size_t i=0;i<len;i++){ out+=h[data[i]>>4]; out+=h[data[i]&15]; }
  return out;
}

static bool hexToBytes(const String &text, uint8_t *out, size_t len)
{
  if (text.length()!=len*2) return false;
  for(size_t i=0;i<len;i++)
  { int a=hexNibble(text[i*2]), b=hexNibble(text[i*2+1]); if(a<0||b<0) return false; out[i]=(uint8_t)((a<<4)|b); }
  return true;
}

static PersistedDeviceMetadata metadataSnapshot(const HM485Device &d)
{
  PersistedDeviceMetadata meta;
  meta.schema = DEVICE_METADATA_SCHEMA;
  meta.cachedType = d.typeKnown ? d.deviceType : 0;
  meta.cachedFirmware = d.firmwareKnown ? d.firmware : 0;
  if (d.serialKnown) strncpy(meta.cachedSerial,d.serialNumber,sizeof(meta.cachedSerial)-1);
  memcpy(meta.friendlyName,d.friendlyName,sizeof(meta.friendlyName));
  memcpy(meta.channelName,d.channelName,sizeof(meta.channelName));
  for(uint8_t ch=0; ch<MAX_CHANNELS; ch++)
  {
    meta.semanticProfile[ch]=(uint8_t)d.semanticProfile[ch];
    meta.invertLogic[ch]=d.invertLogic[ch]?1:0;
    meta.buttonPulseMs[ch]=d.buttonPulseMs[ch]?d.buttonPulseMs[ch]:2000;
    meta.buttonRestOn[ch]=d.buttonRestOn[ch]?1:0;
  }
  return meta;
}

static String buildNvsBackup()
{
  String out; out.reserve(12000);
  out += "HM485GW_BACKUP_V1\n";
  out += "# Firmware=" + String(FW_VERSION) + "\n";
  out += "# WARNING: contains Wi-Fi/MQTT/Web passwords in reversible form.\n";
  auto add=[&](const char *k,const String &v){ out += "cfg."; out += k; out += '='; out += backupEscape(v); out += '\n'; };
  add("wifi_ssid",config.wifiSsid); add("wifi_pass",config.wifiPassword); add("hostname",config.hostname);
  add("mqtt_host",config.mqttHost); add("mqtt_port",String(config.mqttPort)); add("mqtt_user",config.mqttUser); add("mqtt_pass",config.mqttPassword);
  add("mqtt_id",config.mqttClientId); add("mqtt_base",config.mqttBaseTopic); add("ha_prefix",config.discoveryPrefix); add("ha_disc",config.discoveryEnabled?"1":"0");
  add("hm_addr",hexAddress(config.localAddress)); add("ui_lang",config.uiLanguage); add("exp_write",config.experimentalWritesEnabled ? "1" : "0"); add("web_user",config.webUser); add("web_pass",config.webPassword);
  for(uint8_t i=0;i<MAX_DEVICES;i++) if(devices[i].used)
  {
    PersistedDeviceMetadata meta=metadataSnapshot(devices[i]);
    out += "dev." + hexAddress(devices[i].address) + '=' + bytesToHex((const uint8_t*)&meta,sizeof(meta)) + '\n';
  }
  return out;
}

static void applyImportedConfig(const String &key, const String &value)
{
  String v=backupUnescape(value);
  if(key=="wifi_ssid") config.wifiSsid=v; else if(key=="wifi_pass") config.wifiPassword=v; else if(key=="hostname") config.hostname=v;
  else if(key=="mqtt_host") config.mqttHost=v; else if(key=="mqtt_port"){ long p=v.toInt(); if(p>=1&&p<=65535) config.mqttPort=(uint16_t)p; }
  else if(key=="mqtt_user") config.mqttUser=v; else if(key=="mqtt_pass") config.mqttPassword=v; else if(key=="mqtt_id") config.mqttClientId=v;
  else if(key=="mqtt_base") config.mqttBaseTopic=normalizedTopic(v); else if(key=="ha_prefix") config.discoveryPrefix=normalizedTopic(v); else if(key=="ha_disc") config.discoveryEnabled=(v=="1"||v=="true");
  else if(key=="hm_addr"){ char *e=nullptr; uint32_t a=strtoul(v.c_str(),&e,16); if(v.length()==8&&e&&*e==0&&a&&a!=0xFFFFFFFFUL) config.localAddress=a; }
  else if(key=="ui_lang"){ v.toLowerCase(); if(v=="de"||v=="en") config.uiLanguage=v; }
  else if(key=="exp_write") config.experimentalWritesEnabled=(v=="1"||v=="true"||v=="on");
  else if(key=="web_user") config.webUser=v; else if(key=="web_pass") config.webPassword=v;
}

void handleWebBackup()
{
  if(!requireWebAuth()) return;
  String html=htmlHeader(T("HM485 Sicherung / Wiederherstellung","HM485 Backup / Restore"));
  html += F("<h1>"); html += T("Sicherung / Wiederherstellung","Backup / Restore"); html += F("</h1>");
  html += F("<div class='card'><h2>"); html += T("Export","Export"); html += F("</h2><p>");
  html += T("Exportiert Gateway-Konfiguration sowie Namen, Profile und Invertierung der aktuell bekannten Geräte.","Exports gateway configuration plus names, profiles and inversion settings of currently known devices.");
  html += F("</p><p><strong class='bad'>"); html += T("Achtung: Die Sicherung enthält Passwörter.","Warning: the backup contains passwords."); html += F("</strong></p>");
  html += F("<a class='button' href='/backup.txt'>"); html += T("Sicherung herunterladen","Download backup"); html += F("</a></div>");
  html += F("<div class='card'><h2>"); html += T("Import","Import"); html += F("</h2><form method='post' action='/backup_import'>");
  html += F("<label>"); html += T("Inhalt einer HM485GW_BACKUP_V1-Datei","Contents of an HM485GW_BACKUP_V1 file"); html += F("</label>");
  html += F("<textarea name='backup_data' rows='14' style='width:100%;box-sizing:border-box;font-family:monospace' required></textarea>");
  html += F("<p class='muted'>"); html += T("Der Import überschreibt die enthaltenen Einstellungen und startet das Gateway neu. HM485-Geräte werden weiterhin live per Discovery gefunden; der Import ersetzt keine Discovery.","Import overwrites included settings and restarts the gateway. HM485 devices are still found by live discovery; importing never replaces discovery."); html += F("</p>");
  html += F("<button type='submit'>"); html += T("Importieren & Neustarten","Import & restart"); html += F("</button></form></div>");
  html += htmlFooter(); webServer.send(200,"text/html; charset=utf-8",html);
}

void handleWebBackupText()
{
  if(!requireWebAuth()) return;
  webServer.sendHeader("Cache-Control","no-store");
  webServer.sendHeader("Content-Disposition","attachment; filename=hm485_gateway_backup.txt");
  webServer.send(200,"text/plain; charset=utf-8",buildNvsBackup());
}

void handleWebBackupImport()
{
  if(!requireWebAuth()) return;
  if(!webServer.hasArg("backup_data")){ webServer.send(400,"text/plain",T("Keine Sicherungsdaten","No backup data")); return; }
  String data=webServer.arg("backup_data"); data.replace("\r","");
  int nl=data.indexOf('\n'); String header=(nl>=0?data.substring(0,nl):data); header.trim();
  if(header!="HM485GW_BACKUP_V1"){ webServer.send(400,"text/plain",T("Unbekanntes Sicherungsformat","Unknown backup format")); return; }
  uint16_t importedDevices=0;
  Preferences mp; bool metaOpen=mp.begin(DEVICE_METADATA_NAMESPACE,false);
  int pos=(nl>=0?nl+1:data.length());
  while(pos<data.length())
  {
    int eol=data.indexOf('\n',pos); if(eol<0)eol=data.length(); String line=data.substring(pos,eol); pos=eol+1; line.trim();
    if(!line.length()||line[0]=='#') continue; int eq=line.indexOf('='); if(eq<=0) continue;
    String key=line.substring(0,eq), val=line.substring(eq+1);
    if(key.startsWith("cfg.")) applyImportedConfig(key.substring(4),val);
    else if(key.startsWith("dev.") && metaOpen)
    {
      String a=key.substring(4); char *ep=nullptr; uint32_t addr=strtoul(a.c_str(),&ep,16);
      PersistedDeviceMetadata meta;
      if(a.length()==8&&ep&&*ep==0&&hexToBytes(val,(uint8_t*)&meta,sizeof(meta))&&meta.schema==DEVICE_METADATA_SCHEMA)
      { mp.putBytes(deviceMetadataKey(addr).c_str(),&meta,sizeof(meta)); importedDevices++; }
    }
  }
  if(metaOpen) mp.end();
  if(config.mqttBaseTopic.length()==0) config.mqttBaseTopic="hm485"; if(config.discoveryPrefix.length()==0) config.discoveryPrefix="homeassistant";
  if(config.hostname.length()==0) config.hostname="hm485-gateway"; if(config.mqttClientId.length()==0) config.mqttClientId="hm485-gateway";
  if(config.webUser.length()==0) config.webUser="admin"; if(config.webPassword.length()==0) config.webPassword="hm485setup";
  if(config.uiLanguage!="de"&&config.uiLanguage!="en") config.uiLanguage="de";
  saveConfig();
  webLogAdd(String("[NVS] Backup import: device metadata=")+String(importedDevices));
  webServer.send(200,"text/html; charset=utf-8",String("<html><body><h1>")+T("Import erfolgreich","Import successful")+"</h1><p>"+T("Gateway startet neu.","Gateway is restarting.")+"</p></body></html>");
  delay(900); ESP.restart();
}

void handleWebScan()
{
  if (!requireWebAuth())
    return;
  startScan();

  webServer.sendHeader(
    "Location",
    "/"
  );

  webServer.send(
    303,
    "text/plain",
    ""
  );
}

void handleWebReboot()
{
  if (!requireWebAuth())
    return;
  webServer.send(
    200,
    "text/html; charset=utf-8",
    String("<html><body><h1>") + T("Neustart","Restart") + "</h1></body></html>"
  );

  delay(500);
  ESP.restart();
}

// ============================================================
// RAW RX-only diagnostic mode (v0.7.40)
// ============================================================
static String rawRxOnlyLine;
static uint32_t rawRxOnlyBurstStartUs = 0;
static uint32_t rawRxOnlyLastByteUs = 0;
static uint32_t rawRxOnlyByteCount = 0;

static void rawRxOnlyFlush()
{
  if (!rawRxOnlyLine.length()) return;
  Serial.println(rawRxOnlyLine);
  webLogAdd(rawRxOnlyLine);
  rawRxOnlyLine = "";
}

static void rawRxOnlyFeed(uint8_t b)
{
  const uint32_t now = micros();
  if (!rawRxOnlyLine.length())
  {
    rawRxOnlyBurstStartUs = now;
    rawRxOnlyLine.reserve(240);
    rawRxOnlyLine = "[BUS RX] t=";
    rawRxOnlyLine += String(now);
    rawRxOnlyLine += "us :";
  }
  char tmp[24];
  snprintf(tmp, sizeof(tmp), " +%lu:%02X",
           (unsigned long)(now - rawRxOnlyBurstStartUs), b);
  rawRxOnlyLine += tmp;
  rawRxOnlyLastByteUs = now;
  rawRxOnlyByteCount++;
  if (rawRxOnlyLine.length() > 210) rawRxOnlyFlush();
}

void handleRawRxOnlyOn()
{
  if (!requireWebAuth()) return;
  scanActive = false;
  statusPollActive = false;
  pending.active = false;
  outputWrite.active = false;
  buttonPulse.active = false;
  ioConfig.active = false;
  ioConfig.waitingAck = false;
  activeDiscovery.running = false;
  activeDiscovery.waiting = false;
  quietRootRequested = false;
  passiveCaptureActive = false;
  passiveCaptureArmRequested = false;
  rawRxOnlyMode = true;
  digitalWrite(HM485_DIR_PIN, LOW);
  while (HM485.available()) (void)HM485.read();
  rawRxOnlyLine = "";
  rawRxOnlyByteCount = 0;
  webLogAdd("[RAW RX ONLY] ENABLED - HARD TX INHIBIT, protocol parser bypassed");
  Serial.println(F("[RAW RX ONLY] ENABLED - HARD TX INHIBIT, protocol parser bypassed"));
  webServer.sendHeader("Location", "/log");
  webServer.send(303, "text/plain", "");
}

void handleRawRxOnlyOff()
{
  if (!requireWebAuth()) return;
  rawRxOnlyFlush();
  rawRxOnlyMode = false;
  digitalWrite(HM485_DIR_PIN, LOW);
  lastBusActivityUs = micros();
  webLogAdd(String("[RAW RX ONLY] DISABLED - captured bytes=") + rawRxOnlyByteCount);
  Serial.printf("[RAW RX ONLY] DISABLED - captured bytes=%lu\\n",
                (unsigned long)rawRxOnlyByteCount);
  webServer.sendHeader("Location", "/log");
  webServer.send(303, "text/plain", "");
}

void handleWebLogPage()
{
  if (!requireWebAuth())
    return;

  String html =
    htmlHeader(T("HM485 Gateway Log / Diagnose","HM485 Gateway Log / Diagnostics"));

  html += F("<h1>"); html += T("Log / Diagnose","Log / Diagnostics"); html += F("</h1>");

  html += F("<div class='card'>");
  html += F("<h2>RAW RX Only</h2><p><strong>Status:</strong> ");
  if (rawRxOnlyMode) html += F("<span class='ok'>AKTIV</span>");
  else html += String("<span class='muted'>") + T("AUS","OFF") + "</span>";
  html += F(" &nbsp; <strong>Bytes:</strong> ");
  html += String(rawRxOnlyByteCount);
  html += F("</p><p>");
  if (rawRxOnlyMode)
    html += F("<a class='button' href='/raw_rx_only/off'>RAW RX Only beenden</a>");
  else
    html += F("<a class='button' href='/raw_rx_only/on'>RAW RX Only starten</a>");
  html += F("</p><p class='muted'>Im RAW RX Only Mode ist DIR hart auf RX. Scan, Poll, ACK, Discovery und Parser sind deaktiviert. Der UART-Rohstrom wird unverändert mit Zeitabständen geloggt.</p></div>");

  html += F("<div class='card'>");
  html += F("<p><strong>HM485 Raw Logging:</strong> ");

  if (hm485RawLogEnabled)
    html += String("<span class='ok'>") + T("AN","ON") + "</span>";
  else
    html += String("<span class='muted'>") + T("AUS","OFF") + "</span>";

  html += F("</p>");

  html += F("<p>");
  html += F("<a class='button' href='/log/raw/on'>Raw Logging AN</a> ");
  html += F("<a class='button' href='/log/raw/off'>Raw Logging AUS</a> ");
  html += F("<a class='button' href='/log/clear'>"); html += T("Log löschen","Clear log"); html += F("</a> ");
  html += F("<a class='button' href='/log.txt'>"); html += T("Log als Text","Log as text"); html += F("</a>");
  html += F("</p>");

  html += F("<p class='muted'>");
  html += F("Der Ringpuffer liegt nur im RAM. ");
  html += F("Raw Logging zeichnet gültige HM485 RX/TX-Frames auf, ");
  html += F("erzeugt selbst aber keinen zusätzlichen Busverkehr.");
  html += F("</p>");

  html += F("</div>");

  html += F("<div class='card'>");
  html += F("<h2>Native HM485 Discovery</h2>");
  html += F("<p><strong>Status:</strong> ");

  if (activeDiscovery.running || quietRootRequested)
    html += F("<span class='ok'>LÄUFT / WARTET</span>");
  else
    html += F("<span class='muted'>gestoppt</span>");

  html += F(" &nbsp; <strong>Probes:</strong> ");
  html += String(activeDiscovery.probeCount);
  html += F(" &nbsp; <strong>ACKs:</strong> ");
  html += String(activeDiscovery.ackCount);
  html += F(" &nbsp; <strong>TX-Echos:</strong> ");
  html += String(activeDiscovery.ignoredEchoFrames);
  html += F(" &nbsp; <strong>Echo-Bytes:</strong> ");
  html += String(activeDiscovery.ignoredEchoBytes);
  html += F(" &nbsp; <strong>Gefunden:</strong> ");
  html += String(activeDiscovery.foundCount);
  html += F("</p>");

  if (activeDiscovery.foundCount)
  {
    html += F("<p><strong>Adressen:</strong><br>");
    for (uint8_t i = 0; i < activeDiscovery.foundCount; i++)
    {
      html += F("<code>");
      html += hexAddress(activeDiscovery.foundAddress[i]);
      html += F("</code><br>");
    }
    html += F("</p>");
  }

  html += F("<p>");
  html += F("<a class='button' href='/discovery_test/start'>Native Discovery starten</a> ");
  html += F("<a class='button' href='/discovery_test/stop'>Stop</a> ");
  html += F("<a class='button' href='/discovery_test.txt'>Ergebnis herunterladen</a>");
  html += F("</p>");

  html += F("<p><a class='button' href='/scan'>");
  html += T("Geräte neu einlesen", "Reload devices");
  html += F("</a></p>");
  html += F("<p class='muted'>");
  html += T("Fragt Typ, Seriennummer, Firmware, EEPROM und Kanalstatus der bereits entdeckten HM485-Geräte erneut ab. Es wird keine neue 32-Bit-Discovery gestartet.",
            "Re-reads type, serial number, firmware, EEPROM and channel status of already discovered HM485 devices. It does not start a new 32-bit discovery.");
  html += F("</p>");

  html += F("<p class='muted'><strong>v0.7.45 Native Discovery:</strong> ");
  html += F("hm485d/FHEM-Discovery bitte vorher stoppen. Der WT32 durchsucht den ");
  html += F("vollständigen 32-Bit-HM485-Adressbaum nach der hm485d-Logik. Positive ");
  html += F("Antwort ist das erste RX-Byte != 0; nach drei Null/Timeout-Probes wird der Ast verworfen. Gefundene Geräte werden anschließend automatisch gelesen.</p>");
  html += F("</div>");

  html += F("<div class='card'><h2>ESP32 / System</h2><table>");
  html += F("<tr><th>Firmware</th><td>"); html += FW_VERSION; html += F("</td></tr>");
  html += F("<tr><th>Chip</th><td>"); html += htmlEscape(String(ESP.getChipModel())); html += F("</td></tr>");
  html += F("<tr><th>Revision</th><td>"); html += String(ESP.getChipRevision()); html += F("</td></tr>");
  html += F("<tr><th>CPU</th><td>"); html += String(ESP.getCpuFreqMHz()); html += F(" MHz / "); html += String(ESP.getChipCores()); html += F(" Cores</td></tr>");
  html += F("<tr><th>SDK</th><td>"); html += htmlEscape(String(ESP.getSdkVersion())); html += F("</td></tr>");
  html += F("<tr><th>Heap</th><td>"); html += String(ESP.getFreeHeap()/1024); html += F(" KiB frei / "); html += String(ESP.getHeapSize()/1024); html += F(" KiB gesamt</td></tr>");
  html += F("<tr><th>Min. Heap</th><td>"); html += String(ESP.getMinFreeHeap()/1024); html += F(" KiB</td></tr>");
  html += F("<tr><th>Max. Block</th><td>"); html += String(ESP.getMaxAllocHeap()/1024); html += F(" KiB</td></tr>");
  html += F("<tr><th>Flash</th><td>"); html += String(ESP.getFlashChipSize()/1024/1024); html += F(" MiB @ "); html += String(ESP.getFlashChipSpeed()/1000000); html += F(" MHz</td></tr>");
  html += F("<tr><th>Sketch</th><td>"); html += String(ESP.getSketchSize()/1024); html += F(" KiB, "); html += String(ESP.getFreeSketchSpace()/1024); html += F(" KiB OTA-frei</td></tr>");
  if (ESP.getPsramSize() > 0)
  {
    html += F("<tr><th>PSRAM</th><td>"); html += String(ESP.getFreePsram()/1024); html += F(" KiB frei / "); html += String(ESP.getPsramSize()/1024); html += F(" KiB gesamt</td></tr>");
  }
  else
  {
    html += F("<tr><th>PSRAM</th><td>-</td></tr>");
  }
  html += F("<tr><th>Reset reason</th><td>"); html += String((int)esp_reset_reason()); html += F("</td></tr>");
  html += F("<tr><th>Uptime</th><td>"); html += String(millis()/1000); html += F(" s</td></tr>");
  html += F("</table><p><a class='button' href='/reboot'>"); html += T("Gateway neu starten", "Restart gateway"); html += F("</a>");
  html += F("</p><p class='muted'>"); html += T("Startet den ESP32 neu. Die im NVS gespeicherte Konfiguration bleibt erhalten.", "Restarts the ESP32. Configuration stored in NVS is retained."); html += F("</p></div>");

  html += F("<div class='card'>");
  html += F("<pre id='log' style='white-space:pre-wrap;word-break:break-word;");
  html += F("background:#111;color:#ddd;padding:12px;border-radius:8px;");
  html += F("height:60vh;overflow:auto'></pre>");
  html += F("</div>");

  html += F("<p><a href='/'>Zurück</a></p>");

  html += F(
    "<script>"
    "async function refreshLog(){"
      "try{"
        "const r=await fetch('/log.txt',{cache:'no-store'});"
        "if(r.ok){"
          "const e=document.getElementById('log');"
          "const b=(e.scrollHeight-e.scrollTop-e.clientHeight)<80;"
          "e.textContent=await r.text();"
          "if(b)e.scrollTop=e.scrollHeight;"
        "}"
      "}catch(x){}"
    "}"
    "refreshLog();"
    "setInterval(refreshLog,1000);"
    "</script>"
  );

  html += htmlFooter();

  webServer.send(
    200,
    "text/html; charset=utf-8",
    html
  );
}

void handleWebLogText()
{
  if (!requireWebAuth())
    return;

  webServer.sendHeader(
    "Cache-Control",
    "no-store"
  );

  webServer.send(
    200,
    "text/plain; charset=utf-8",
    webLogText()
  );
}

void handleWebLogRawOn()
{
  if (!requireWebAuth())
    return;

  hm485RawLogEnabled = true;
  webLogAdd("[LOG] HM485 raw logging enabled");

  webServer.sendHeader(
    "Location",
    "/log"
  );

  webServer.send(
    303,
    "text/plain",
    ""
  );
}

void handleWebLogRawOff()
{
  if (!requireWebAuth())
    return;

  webLogAdd("[LOG] HM485 raw logging disabled");
  hm485RawLogEnabled = false;

  webServer.sendHeader(
    "Location",
    "/log"
  );

  webServer.send(
    303,
    "text/plain",
    ""
  );
}

void handleWebLogClear()
{
  if (!requireWebAuth())
    return;

  webLogClear();
  webLogAdd("[LOG] Log cleared");

  webServer.sendHeader(
    "Location",
    "/log"
  );

  webServer.send(
    303,
    "text/plain",
    ""
  );
}

void handleWebUpdatePage()
{
  if (!requireWebAuth()) return;
  String html = htmlHeader(T("HM485 Gateway Firmware Update", "HM485 Gateway Firmware Update"));
  html += F("<h1>Firmware Update</h1><div class='card'><p>");
  html += T("Aktuelle Firmware", "Current firmware"); html += F(": <strong>"); html += FW_VERSION; html += F("</strong></p><p class='muted'>");
  html += T("Arduino/ESP32 Firmware als .bin hochladen. Nach erfolgreichem Update startet das Gateway neu.", "Upload Arduino/ESP32 firmware as a .bin file. After a successful update the gateway restarts.");
  html += F("</p><form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='firmware' accept='.bin,application/octet-stream' required><button type='submit'>");
  html += T("Firmware installieren", "Install firmware"); html += F("</button></form></div>");
  html += htmlFooter(); webServer.send(200,"text/html; charset=utf-8",html);
}

void handleWebUpdateFinish()
{
  if (!requireWebAuth()) return;
  bool ok = !Update.hasError();
  webServer.sendHeader("Connection","close");
  String msg = String("<html><body><h1>") + (ok ? T("Update erfolgreich","Update successful") : T("Update fehlgeschlagen","Update failed")) + "</h1><p>" + (ok ? T("Gateway startet neu.","Gateway is restarting.") : T("Bitte serielles Log prüfen.","Please check the serial log.")) + "</p></body></html>";
  webServer.send(ok?200:500,"text/html; charset=utf-8",msg);
  if(ok){ delay(800); ESP.restart(); }
}

void handleWebUpdateUpload()
{
  // Authentication is checked before accepting the upload body.
  if (
    !webServer.authenticate(
      config.webUser.c_str(),
      config.webPassword.c_str()
    )
  )
  {
    return;
  }

  HTTPUpload &upload =
    webServer.upload();

  if (upload.status == UPLOAD_FILE_START)
  {
    Serial.printf(
      "[OTA] Start: %s\n",
      upload.filename.c_str()
    );

    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
      Update.printError(Serial);
  }
  else if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (
      Update.write(
        upload.buf,
        upload.currentSize
      ) != upload.currentSize
    )
    {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_END)
  {
    if (Update.end(true))
    {
      Serial.printf(
        "[OTA] Success: %u bytes\n",
        upload.totalSize
      );
    }
    else
    {
      Update.printError(Serial);
    }
  }
  else if (upload.status == UPLOAD_FILE_ABORTED)
  {
    Update.abort();
    Serial.println(F("[OTA] Upload aborted"));
  }
}

void handleActiveDiscoveryStart()
{
  if (!requireWebAuth())
    return;


  activeDiscoveryStart();

  webServer.sendHeader("Location", "/log");
  webServer.send(303, "text/plain", "");
}

void handleActiveDiscoveryStop()
{
  if (!requireWebAuth())
    return;

  activeDiscoveryStop();

  webServer.sendHeader("Location", "/log");
  webServer.send(303, "text/plain", "");
}

void handleActiveDiscoveryText()
{
  if (!requireWebAuth())
    return;

  webServer.sendHeader("Cache-Control", "no-store");
  webServer.sendHeader(
    "Content-Disposition",
    "attachment; filename=hm485_native_discovery.txt"
  );

  webServer.send(
    200,
    "text/plain; charset=utf-8",
    activeDiscoveryResultText()
  );
}

void setupWebServer()
{
  webServer.on("/", HTTP_GET, handleWebRoot);
  webServer.on("/config", HTTP_GET, handleWebConfig);
  webServer.on("/save", HTTP_POST, handleWebSave);
  webServer.on("/device", HTTP_GET, handleWebDevice);
  webServer.on("/device_save", HTTP_POST, handleWebDeviceSave);
  webServer.on("/device_forget", HTTP_POST, handleWebDeviceForget);
  webServer.on("/channel", HTTP_GET, handleWebChannel);
  webServer.on("/channel_save", HTTP_POST, handleWebChannelSave);
  webServer.on("/device_status", HTTP_GET, handleWebDeviceStatus);
  webServer.on("/status_refresh", HTTP_GET, handleWebStatusRefresh);
  webServer.on("/backup", HTTP_GET, handleWebBackup);
  webServer.on("/backup.txt", HTTP_GET, handleWebBackupText);
  webServer.on("/backup_import", HTTP_POST, handleWebBackupImport);
  webServer.on("/scan", HTTP_GET, handleWebScan);
  webServer.on("/reboot", HTTP_GET, handleWebReboot);

  webServer.on("/log", HTTP_GET, handleWebLogPage);
  webServer.on("/log.txt", HTTP_GET, handleWebLogText);
  webServer.on("/log/raw/on", HTTP_GET, handleWebLogRawOn);
  webServer.on("/log/raw/off", HTTP_GET, handleWebLogRawOff);
  webServer.on("/raw_rx_only/on", HTTP_GET, handleRawRxOnlyOn);
  webServer.on("/raw_rx_only/off", HTTP_GET, handleRawRxOnlyOff);
  webServer.on("/log/clear", HTTP_GET, handleWebLogClear);
  webServer.on("/discovery_test/start", HTTP_GET, handleActiveDiscoveryStart);
  webServer.on("/discovery_test/stop", HTTP_GET, handleActiveDiscoveryStop);
  webServer.on("/discovery_test.txt", HTTP_GET, handleActiveDiscoveryText);

  webServer.on("/update", HTTP_GET, handleWebUpdatePage);
  webServer.on(
    "/update",
    HTTP_POST,
    handleWebUpdateFinish,
    handleWebUpdateUpload
  );

  webServer.onNotFound(
    []()
    {
      webServer.send(
        404,
        "text/plain; charset=utf-8",
        "Not found"
      );
    }
  );

  webServer.begin();

  Serial.println(F("[WEB] HTTP server started"));
}

// ============================================================
// HM485 CRC / address helpers
// ============================================================

uint16_t crc16Shift(
  uint8_t w,
  uint16_t reg
)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    bool status =
      (reg & 0x8000) != 0;

    reg =
      (reg << 1) & 0xFFFF;

    if (w & 0x80)
      reg |= 1;

    if (status)
      reg ^= 0x1002;

    w <<= 1;
  }

  return reg & 0xFFFF;
}

void addressToBytes(
  uint32_t address,
  uint8_t *out
)
{
  out[0] = (address >> 24) & 0xFF;
  out[1] = (address >> 16) & 0xFF;
  out[2] = (address >> 8) & 0xFF;
  out[3] = address & 0xFF;
}

uint32_t bytesToAddress(
  const uint8_t *a
)
{
  return
    ((uint32_t)a[0] << 24) |
    ((uint32_t)a[1] << 16) |
    ((uint32_t)a[2] << 8) |
    ((uint32_t)a[3]);
}

// ============================================================
// HM485 CTRL helpers
// ============================================================

bool ctrlHasSender(uint8_t ctrl)
{
  return (ctrl & 0x08) != 0;
}

bool ctrlIsDiscovery(uint8_t ctrl)
{
  return (ctrl & 0x07) == 0x03;
}

bool ctrlIsIframe(uint8_t ctrl)
{
  return (ctrl & 0x01) == 0;
}

bool ctrlIsAck(uint8_t ctrl)
{
  return (ctrl & 0x9F) == 0x19;
}

uint8_t ctrlTxNum(uint8_t ctrl)
{
  return (ctrl >> 1) & 0x03;
}

uint8_t ctrlAckNum(uint8_t ctrl)
{
  return (ctrl >> 5) & 0x03;
}

// ============================================================
// Profile helpers
// ============================================================

static const char *deviceSupportName(DeviceSupport support)
{
  switch (support)
  {
    case DeviceSupport::TESTED:         return "tested";
    case DeviceSupport::KNOWN_UNTESTED: return "known/untested";
    case DeviceSupport::EXPERIMENTAL:   return "experimental";
    default:                            return "unknown";
  }
}


const DeviceProfile *findDeviceProfile(
  uint16_t type
)
{
  for (size_t i = 0; i < DEVICE_PROFILE_COUNT; i++)
  {
    if (DEVICE_PROFILES[i].deviceType == type)
      return &DEVICE_PROFILES[i];
  }

  return nullptr;
}

const ChannelProfile *findChannelProfile(
  const DeviceProfile *device,
  uint8_t busChannel
)
{
  if (!device)
    return nullptr;

  for (uint8_t i = 0; i < device->channelGroupCount; i++)
  {
    const ChannelProfile &p =
      device->channels[i];

    if (
      busChannel >= p.first &&
      busChannel < p.first + p.count
    )
    {
      return &p;
    }
  }

  return nullptr;
}

const char *channelTypeName(
  ChannelType type
)
{
  switch (type)
  {
    case ChannelType::CONTACT:
      return "CONTACT";

    case ChannelType::DIGITAL_OUTPUT:
      return "DIGITAL_OUTPUT";

    case ChannelType::DIGITAL_ANALOG_OUTPUT:
      return "DIGITAL/ANALOG_OUTPUT";

    case ChannelType::DIGITAL_INPUT:
      return "DIGITAL_INPUT";

    case ChannelType::DIGITAL_FREQUENCY_INPUT:
      return "DIGITAL/FREQUENCY_INPUT";

    case ChannelType::DIGITAL_ANALOG_INPUT:
      return "DIGITAL/ANALOG_INPUT";
    case ChannelType::INPUT_OUTPUT:
      return "INPUT/OUTPUT";
    case ChannelType::COVER:
      return "COVER";
    case ChannelType::DIMMER:
      return "DIMMER";
    case ChannelType::TEMPERATURE:
      return "TEMPERATURE";
    case ChannelType::COUNTER:
      return "COUNTER";
    case ChannelType::KEY_INPUT:
      return "KEY_INPUT";
    case ChannelType::MOTION:
      return "MOTION";
    case ChannelType::RGB:
      return "RGB";

    default:
      return "UNKNOWN";
  }
}

const char *behaviourName(
  ChannelBehaviour behaviour
)
{
  switch (behaviour)
  {
    case ChannelBehaviour::CONTACT:
      return "CONTACT";

    case ChannelBehaviour::DIGITAL_OUTPUT:
      return "DIGITAL_OUTPUT";

    case ChannelBehaviour::ANALOG_OUTPUT:
      return "FREQUENCY_OUTPUT";

    case ChannelBehaviour::DIGITAL_INPUT:
      return "DIGITAL_INPUT";

    case ChannelBehaviour::FREQUENCY_INPUT:
      return "FREQUENCY_INPUT";

    case ChannelBehaviour::ANALOG_INPUT:
      return "ANALOG_INPUT";

    default:
      return "UNKNOWN";
  }
}

// ============================================================
// Device DB / EEPROM
// ============================================================

HM485Device *getDevice(
  uint32_t address,
  bool create
)
{
  if (
    address == 0 ||
    address == config.localAddress ||
    address == FHEM_ADDRESS ||
    address == 0xFFFFFFFF
  )
  {
    return nullptr;
  }

  for (uint8_t i = 0; i < MAX_DEVICES; i++)
  {
    if (
      devices[i].used &&
      devices[i].address == address
    )
    {
      devices[i].lastSeen = millis();
      return &devices[i];
    }
  }

  if (!create)
    return nullptr;

  for (uint8_t i = 0; i < MAX_DEVICES; i++)
  {
    if (!devices[i].used)
    {
      devices[i].used = true;
      devices[i].address = address;
      devices[i].lastSeen = millis();
      loadDeviceMetadata(&devices[i]);

      Serial.println();
      Serial.print(F("[DEVICE] New: "));
      printAddress(address);
      Serial.println();

      return &devices[i];
    }
  }

  return nullptr;
}

static void passiveDiscoveryObserveSender(uint32_t sender)
{
  // 0x00000001..0x000000FF are central-address territory according to the
  // classic protocol description. Never create device candidates from them.
  if (sender <= 0x000000FFUL || sender == 0xFFFFFFFFUL)
    return;

  if (getDevice(sender, false))
    return;

  HM485Device *device = getDevice(sender, true);
  if (!device) return;

  passiveDiscoveryLastCandidate = sender;
  passiveDiscoveryScanPending = true;
  String msg = String("[PASSIVE DISCOVERY] new address seen: ") + hexAddress(sender) +
               "; identification scheduled";
  Serial.println(msg);
  webLogAdd(msg);
}

static void processPassiveDiscovery()
{
  if (!passiveDiscoveryScanPending) return;
  if (addressGuardBootActive || addressConflictLocked || rawRxOnlyMode ||
      activeDiscovery.running || scanActive || statusPollActive || pending.active ||
      outputWrite.active || ioConfig.active || quietRootRequested ||
      passiveCaptureActive || passiveCaptureArmRequested || !busIsIdle())
    return;

  passiveDiscoveryScanPending = false;
  webLogAdd(String("[PASSIVE DISCOVERY] identifying candidate ") +
            hexAddress(passiveDiscoveryLastCandidate));
  startScan();
}

static void processPassiveIdentityStatusPoll()
{
  if (passiveIdentityStatusPollPendingAddress == 0)
    return;

  if (
    addressGuardBootActive ||
    addressConflictLocked ||
    rawRxOnlyMode ||
    activeDiscovery.running ||
    scanActive ||
    statusPollActive ||
    pending.active ||
    outputWrite.active ||
    ioConfig.active ||
    buttonPulse.active ||
    quietRootRequested ||
    passiveCaptureActive ||
    passiveCaptureArmRequested ||
    !busIsIdle()
  )
  {
    return;
  }

  const uint32_t address = passiveIdentityStatusPollPendingAddress;

  // Clear only after the targeted poll really starts. If the bus becomes busy,
  // the request remains queued and will be retried on a later loop iteration.
  if (startStatusPollForDevice(address))
  {
    passiveIdentityStatusPollPendingAddress = 0;
  }
}


static void processBootKnownDeviceVerification()
{
  if (!bootKnownDeviceVerifyPending || !initialNativeDiscoveryFinished)
    return;

  if (
    addressGuardBootActive ||
    addressConflictLocked ||
    rawRxOnlyMode ||
    activeDiscovery.running ||
    quietRootRequested ||
    scanActive ||
    statusPollActive ||
    pending.active ||
    outputWrite.active ||
    ioConfig.active ||
    buttonPulse.active ||
    passiveCaptureActive ||
    passiveCaptureArmRequested ||
    !busIsIdle()
  )
  {
    return;
  }

  while (bootKnownDeviceVerifyIndex < MAX_DEVICES)
  {
    HM485Device &d = devices[bootKnownDeviceVerifyIndex++];

    if (!d.used || !d.restoredFromNvs || !d.profile || !d.profile->activeReadSafe)
      continue;

    // A targeted status poll doubles as liveness verification and restores the
    // real channel states after a gateway restart. Passive-only profiles remain
    // known from NVS but are not actively touched.
    if (startStatusPollForDevice(d.address))
    {
      String msg = String("[NVS] Verifying restored device: ") + hexAddress(d.address);
      Serial.println(msg);
      webLogAdd(msg);
      return;
    }

    // Could not start right now; retry this same entry later.
    bootKnownDeviceVerifyIndex--;
    return;
  }

  bootKnownDeviceVerifyPending = false;
  webLogAdd("[NVS] Restored-device verification finished");
}

void decodeEepromConfiguration(
  HM485Device *device
);

void setDeviceType(
  HM485Device *device,
  uint16_t type
)
{
  if (!device)
    return;

  device->typeKnown = true;
  device->deviceType = type;
  device->profile = findDeviceProfile(type);

  if (!device->profile)
    return;

  for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++)
  {
    const ChannelProfile *cp =
      findChannelProfile(device->profile, ch);

    if (!cp)
      continue;

    device->channel[ch].behaviour =
      cp->defaultBehaviour;

    device->channel[ch].behaviourKnown =
      !cp->configurable;
  }

  decodeEepromConfiguration(device);
}

bool getEepromBit(
  HM485Device *device,
  uint16_t address,
  uint8_t bit,
  bool &value
)
{
  if (
    !device ||
    address >= EEPROM_CACHE_SIZE ||
    bit > 7 ||
    !device->eepromValid[address]
  )
  {
    return false;
  }

  value =
    (
      device->eeprom[address] &
      (1U << bit)
    ) != 0;

  return true;
}

bool getEepromUint16(
  HM485Device *device,
  uint16_t address,
  uint16_t &value
)
{
  if (
    !device ||
    address + 1 >= EEPROM_CACHE_SIZE ||
    !device->eepromValid[address] ||
    !device->eepromValid[address + 1]
  )
  {
    return false;
  }

  value =
    ((uint16_t)device->eeprom[address] << 8) |
    device->eeprom[address + 1];

  return true;
}

PulseValueState getPulseTime(
  HM485Device *device,
  uint8_t busChannel,
  uint16_t &raw,
  float &seconds
)
{
  if (
    !device ||
    device->deviceType != 0x001C ||
    busChannel < 6 ||
    busChannel > 13
  )
  {
    return PulseValueState::UNAVAILABLE;
  }

  uint16_t address =
    0x0010 +
    (busChannel - 6) * 2;

  if (
    !getEepromUint16(
      device,
      address,
      raw
    )
  )
  {
    return PulseValueState::UNAVAILABLE;
  }

  if (
    raw == 0x3333 ||
    raw == 0xFFFF
  )
  {
    return PulseValueState::SPECIAL;
  }

  seconds =
    raw / 100.0f;

  return PulseValueState::VALID;
}

void decodeEepromConfiguration(
  HM485Device *device
)
{
  if (!device || !device->profile)
    return;

  for (
    uint8_t busChannel = 0;
    busChannel < MAX_CHANNELS;
    busChannel++
  )
  {
    const ChannelProfile *cp =
      findChannelProfile(
        device->profile,
        busChannel
      );

    if (
      !cp ||
      !cp->configurable ||
      !cp->behaviourEeprom.available
    )
    {
      continue;
    }

    uint8_t offset =
      busChannel - cp->first;

    uint16_t absoluteBit =
      cp->behaviourEeprom.firstBit +
      offset *
      cp->behaviourEeprom.bitStep;

    uint16_t byteAddress =
      cp->behaviourEeprom.byteAddress +
      absoluteBit / 8;

    uint8_t bit =
      absoluteBit % 8;

    bool bitValue = false;

    if (
      !getEepromBit(
        device,
        byteAddress,
        bit,
        bitValue
      )
    )
    {
      continue;
    }

    switch (cp->type)
    {
      case ChannelType::DIGITAL_ANALOG_OUTPUT:
        device->channel[busChannel].behaviour =
          bitValue
          ? ChannelBehaviour::DIGITAL_OUTPUT
          : ChannelBehaviour::ANALOG_OUTPUT;
        break;

      case ChannelType::DIGITAL_ANALOG_INPUT:
        device->channel[busChannel].behaviour =
          bitValue
          ? ChannelBehaviour::DIGITAL_INPUT
          : ChannelBehaviour::ANALOG_INPUT;
        break;

      case ChannelType::DIGITAL_FREQUENCY_INPUT:
        device->channel[busChannel].behaviour =
          bitValue
          ? ChannelBehaviour::DIGITAL_INPUT
          : ChannelBehaviour::FREQUENCY_INPUT;
        break;

      default:
        continue;
    }

    device->channel[busChannel].behaviourKnown = true;
  }
}

void storeEepromBlock(
  HM485Device *device,
  uint16_t address,
  const uint8_t *data,
  uint8_t len
)
{
  if (!device)
    return;

  for (uint8_t i = 0; i < len; i++)
  {
    uint16_t a = address + i;

    if (a >= EEPROM_CACHE_SIZE)
      break;

    device->eeprom[a] = data[i];
    device->eepromValid[a] = true;
  }

  decodeEepromConfiguration(device);
}

uint8_t expectedValueSize(
  HM485Device *device,
  uint8_t busChannel
)
{
  if (!device || !device->profile)
    return 0;

  // HMW-IO-12-Sw14-DR frequency inputs use a 24-bit info-level value,
  // while the same physical channel uses a 16-bit value in digital mode.
  if (device->channel[busChannel].behaviourKnown &&
      device->channel[busChannel].behaviour == ChannelBehaviour::FREQUENCY_INPUT)
    return 3;

  const ChannelProfile *cp =
    findChannelProfile(
      device->profile,
      busChannel
    );

  if (!cp)
    return 0;

  switch (cp->infoLevelEncoding)
  {
    case ValueEncoding::BOOLEAN_1BYTE:
      return 1;

    case ValueEncoding::BOOLEAN_2BYTE:
    case ValueEncoding::UINT16:
      return 2;

    case ValueEncoding::UINT24:
      return 3;

    default:
      return 0;
  }
}

// ============================================================
// HM485 address conflict guard
// ============================================================

void latchAddressConflict(uint32_t sender)
{
  addressConflictLastSeenMs = millis();
  addressConflictSender = sender;

  if (addressConflictLocked)
    return;

  addressConflictLocked = true;
  addressGuardBootActive = false;

  // Stop every active protocol operation immediately. The gateway remains
  // connected to Ethernet/WiFi/Web/MQTT, but HM485 DE is held LOW.
  pending.active = false;
  scanActive = false;
  statusPollActive = false;
  activeDiscovery.running = false;
  quietRootRequested = false;

  digitalWrite(HM485_DIR_PIN, LOW);

  String msg = "[SAFETY] HM485 address conflict: source ";
  msg += hexAddress(sender);
  msg += " equals configured gateway address. TX LOCKED until address change/reboot.";
  webLogAdd(msg);
  Serial.println(msg);
}

void addressGuardObserveFrame(uint32_t sender)
{
  if (sender == 0)
    return;

  const uint32_t now = millis();

  if (sender == 0x00000001UL)
  {
    centralAddressOneSeen = true;
    centralAddressOneLastSeenMs = now;
  }

  if (sender != config.localAddress)
    return;

  // A transceiver can echo our own just-sent frame into RX. Ignore a short
  // window after our own TX; a real duplicate address will be seen again on
  // later independent traffic and will then latch the safety lock.
  if (lastLocalTxCompletedMs != 0 &&
      (uint32_t)(now - lastLocalTxCompletedMs) < ADDRESS_GUARD_OWN_TX_SUPPRESS_MS)
    return;

  latchAddressConflict(sender);
}

void processAddressGuard()
{
  if (!addressGuardBootActive || addressConflictLocked)
    return;

  if ((uint32_t)(millis() - addressGuardStartedMs) < ADDRESS_GUARD_BOOT_LISTEN_MS)
    return;

  addressGuardBootActive = false;

  Serial.print(F("[SAFETY] Passive address check complete. Address 00000001 seen: "));
  Serial.println(centralAddressOneSeen ? F("YES") : F("NO"));
  webLogAdd(String("[SAFETY] Passive address check complete; 00000001 seen=") +
            (centralAddressOneSeen ? "yes" : "no"));

  // Absence of a frame is not proof that the address is unused; this is a
  // passive best-effort guard. Runtime collision detection remains active.
  activeDiscoveryStart();
}

// ============================================================
// RS485 / frame TX
// ============================================================

void setReceiveMode()
{
  digitalWrite(HM485_DIR_PIN, LOW);
}

void setTransmitMode()
{
  if (rawRxOnlyMode || addressGuardBootActive || addressConflictLocked)
  {
    digitalWrite(HM485_DIR_PIN, LOW);
    return;
  }
  digitalWrite(HM485_DIR_PIN, HIGH);
}

bool busIsIdle()
{
  return
    (uint32_t)(
      micros() -
      lastBusActivityUs
    ) >= BUS_IDLE_US;
}

void appendEscaped(
  uint8_t *buffer,
  size_t &pos,
  uint8_t value
)
{
  if (
    value == FRAME_START_LONG ||
    value == FRAME_START_SHORT ||
    value == ESCAPE_CHAR
  )
  {
    buffer[pos++] = ESCAPE_CHAR;
    buffer[pos++] = value & 0x7F;
  }
  else
  {
    buffer[pos++] = value;
  }
}

static uint16_t activeDiscoverySweepDelayUs()
{
  static const uint16_t delays[ActiveDiscoveryState::SWEEP_STEPS] =
    { 20 };

  uint16_t probeIndex = activeDiscovery.probeCount;
  uint8_t step = probeIndex / ActiveDiscoveryState::PROBES_PER_STEP;
  if (step >= ActiveDiscoveryState::SWEEP_STEPS)
    step = ActiveDiscoveryState::SWEEP_STEPS - 1;

  return delays[step];
}

static uint8_t activeDiscoverySweepStep()
{
  uint16_t probeIndex =
    activeDiscovery.probeCount > 0
      ? activeDiscovery.probeCount - 1
      : 0;
  uint8_t step = probeIndex / ActiveDiscoveryState::PROBES_PER_STEP;
  if (step >= ActiveDiscoveryState::SWEEP_STEPS)
    step = ActiveDiscoveryState::SWEEP_STEPS - 1;
  return step;
}

bool sendLongFrame(
  uint32_t target,
  uint8_t ctrl,
  uint32_t sender,
  const uint8_t *payload,
  uint8_t payloadLen,
  bool waitForIdle
)
{
  if (addressGuardBootActive || addressConflictLocked || rawRxOnlyMode)
    return false;

  if (
    waitForIdle &&
    !busIsIdle()
  )
  {
    return false;
  }

  uint8_t txBuffer[128];
  size_t txPos = 0;
  uint16_t crc = 0xFFFF;

  txBuffer[txPos++] = FRAME_START_LONG;
  crc = crc16Shift(FRAME_START_LONG, crc);

  uint8_t targetBytes[4];
  addressToBytes(target, targetBytes);

  for (uint8_t i = 0; i < 4; i++)
  {
    appendEscaped(txBuffer, txPos, targetBytes[i]);
    crc = crc16Shift(targetBytes[i], crc);
  }

  appendEscaped(txBuffer, txPos, ctrl);
  crc = crc16Shift(ctrl, crc);

  if (
    !ctrlIsDiscovery(ctrl) &&
    ctrlHasSender(ctrl)
  )
  {
    uint8_t senderBytes[4];
    addressToBytes(sender, senderBytes);

    for (uint8_t i = 0; i < 4; i++)
    {
      appendEscaped(txBuffer, txPos, senderBytes[i]);
      crc = crc16Shift(senderBytes[i], crc);
    }
  }

  uint8_t dataLength =
    payloadLen + 2;

  appendEscaped(txBuffer, txPos, dataLength);
  crc = crc16Shift(dataLength, crc);

  for (uint8_t i = 0; i < payloadLen; i++)
  {
    appendEscaped(txBuffer, txPos, payload[i]);
    crc = crc16Shift(payload[i], crc);
  }

  crc = crc16Shift(0x00, crc);
  crc = crc16Shift(0x00, crc);

  appendEscaped(
    txBuffer,
    txPos,
    (crc >> 8) & 0xFF
  );

  appendEscaped(
    txBuffer,
    txPos,
    crc & 0xFF
  );

  if (hm485RawLogEnabled)
  {
    webLogAdd(
      hm485RawLine(
        "TX",
        target,
        sender,
        ctrl,
        payload,
        payloadLen
      )
    );
  }

  setTransmitMode();
  delayMicroseconds(50);

  // v0.7.8: remember exact wire bytes for raw RX echo suppression.
  activeDiscovery.txEchoLen = 0;
  activeDiscovery.txEchoPos = 0;
  activeDiscovery.txEchoMatching = false;

  if (activeDiscovery.running)
  {
    uint8_t copyLen = (txPos < ActiveDiscoveryState::TX_ECHO_MAX)
      ? txPos
      : ActiveDiscoveryState::TX_ECHO_MAX;

    memcpy(activeDiscovery.txEcho, txBuffer, copyLen);
    activeDiscovery.txEchoLen = copyLen;
    activeDiscovery.txEchoPos = 0;
    activeDiscovery.txEchoMatching = (copyLen > 0);
  }

  // v0.7.31: capture the exact encoded bytes handed to UART.
  if (activeDiscovery.running)
    wireVerifySetTx(txBuffer, txPos);

  HM485.write(
    txBuffer,
    txPos
  );

  HM485.flush();

  // v0.7.10:
  // The RS485 receiver may remain active while DE is HIGH. In that case
  // our own transmitted bytes are received locally and remain queued in
  // the ESP32 UART RX FIFO. This also happens with A/B disconnected,
  // because it is local transceiver loopback, not a bus response.
  //
  // TX is fully finished here (flush()), while DE is still HIGH, so no
  // real HM485 device can have started its response yet. It is therefore
  // safe to discard everything currently waiting in the UART RX FIFO.
  if (activeDiscovery.running)
  {
    uint16_t drained = 0;
    while (HM485.available())
    {
      (void)HM485.read();
      drained++;
    }

    if (drained > 0)
    {
      activeDiscovery.discardedEchoBytes += drained;
      activeDiscovery.analyzerPreDeDrained += drained;
      webLogAdd(
        String("[DISCOVERY RX-DRAIN] discarded ") +
        drained +
        " local RX byte(s) before DE->RX"
      );
    }
  }

  // v0.7.26: known-prefix turnaround delay sweep.
  // Only active for this diagnostic run. 20 probes each at
  // 0, 10, 15, 20, 25, 30, 35 and 40 us.
  if (activeDiscovery.running)
  {
    activeDiscovery.turnaroundDelayUs =
      activeDiscoverySweepDelayUs();
    if (activeDiscovery.turnaroundDelayUs > 0)
      delayMicroseconds(activeDiscovery.turnaroundDelayUs);
  }

  setReceiveMode();
  lastLocalTxCompletedMs = millis();

  // Reference point for discovery RX turnaround diagnostics.
  // Set only after the transceiver has been returned to receive mode.
  if (activeDiscovery.running)
    activeDiscovery.rxArmUs = micros();

  lastBusActivityUs = micros();
  statTxFrames++;

  return true;
}


// ============================================================
// Native active discovery
// v0.7.46 uses the proven HM485_Protocol.pm compatible 32-bit prefix/tree traversal.
// v0.7.44 proved the physical polarity and first-byte response semantics.
// ============================================================

void activeDiscoveryFinish();

void activeDiscoveryAddResult(uint32_t address)
{
  for (uint8_t i = 0; i < activeDiscovery.foundCount; i++)
  {
    if (activeDiscovery.foundAddress[i] == address)
      return;
  }

  if (activeDiscovery.foundCount >= ACTIVE_DISCOVERY_MAX_DEVICES)
  {
    webLogAdd("[DISCOVERY] result list full - aborting");
    activeDiscoveryFinish();
    return;
  }

  activeDiscovery.foundAddress[
    activeDiscovery.foundCount++
  ] = address;

  String line = "[DISCOVERY] FOUND ";
  line += hexAddress(address);
  webLogAdd(line);

  Serial.println(line);
}

bool sendDiscoveryZeroCommCommand(uint8_t command)
{
  // v0.7.21: reproduce the observed hm485d/CCU sequence exactly.
  // Capture showed both z and Z broadcasts as:
  // DST=FFFFFFFF SRC=00000001 CTRL=98 LEN=1
  //
  // Do not use config.localAddress (currently 00000002) here and do not
  // advance the normal TX sequence counter. Discovery probes themselves
  // remain source-less as required by the protocol.
  static constexpr uint32_t DISCOVERY_CENTRAL_ADDRESS = 0x00000001UL;
  static constexpr uint8_t DISCOVERY_ZERO_CTRL = 0x98;

  const uint8_t payload[] = { command };

  return sendLongFrame(
    0xFFFFFFFFUL,
    DISCOVERY_ZERO_CTRL,
    DISCOVERY_CENTRAL_ADDRESS,
    payload,
    sizeof(payload),
    true
  );
}

void sendDiscoveryZeroCommPair(uint8_t command)
{
  for (uint8_t i = 0; i < 2; i++)
  {
    // Give the multidrop bus enough time to settle between broadcasts.
    uint32_t started = millis();

    while (!sendDiscoveryZeroCommCommand(command))
    {
      delay(2);
      if (millis() - started > 250)
      {
        webLogAdd(
          String("[DISCOVERY TEST] zero-comm command 0x") +
          String(command, HEX) +
          " could not be sent"
        );
        break;
      }
    }

    delay(100);
  }
}

void activeDiscoveryFinish()
{
  activeDiscovery.running = false;
  initialNativeDiscoveryFinished = true;
  activeDiscovery.waiting = false;
  setReceiveMode();

  String line = "[DISCOVERY] finished devices=";
  line += String(activeDiscovery.foundCount);
  line += " probes=";
  line += String(activeDiscovery.probeCount);
  line += " positive=";
  line += String(activeDiscovery.ackCount);
  line += " raw_rx=";
  line += String(activeDiscovery.rawByteCount);

  webLogAdd(line);
  Serial.println(line);

  crcAbResultText += "\n# DISCOVERY FINISHED devices=";
  crcAbResultText += String(activeDiscovery.foundCount);
  crcAbResultText += " probes=";
  crcAbResultText += String(activeDiscovery.probeCount);
  crcAbResultText += " positive=";
  crcAbResultText += String(activeDiscovery.ackCount);
  crcAbResultText += " raw_rx_bytes=";
  crcAbResultText += String(activeDiscovery.rawByteCount);
  crcAbResultText += "\n# Found addresses:\n";

  for (uint8_t i = 0; i < activeDiscovery.foundCount; i++)
  {
    crcAbResultText += "#   ";
    crcAbResultText += hexAddress(activeDiscovery.foundAddress[i]);
    crcAbResultText += "\n";
  }

  // v0.7.45: immediately run the proven read-only metadata/status scan
  // against the dynamically discovered address list.
  if (activeDiscovery.foundCount > 0)
  {
    webLogAdd("[DISCOVERY] starting read-only scan of discovered devices");
    startScan();
  }
}

static void crcAbDrainRx()
{
  while (HM485.available())
    HM485.read();
}

static void crcAbAppendHexByte(String &out, uint8_t b)
{
  char x[4];
  snprintf(x, sizeof(x), "%02X", b);
  out += x;
}

static void crcAbSendRawFrame(
  const char *label,
  const uint8_t *frame,
  size_t len
)
{
  crcAbDrainRx();

  crcAbResultText += "\n[CRC A/B] ";
  crcAbResultText += label;
  crcAbResultText += "\nTX: ";

  Serial.print(F("\n[CRC A/B] "));
  Serial.println(label);
  Serial.print(F("TX: "));

  for (size_t i = 0; i < len; i++)
  {
    if (i)
    {
      Serial.print(' ');
      crcAbResultText += ' ';
    }
    if (frame[i] < 0x10) Serial.print('0');
    Serial.print(frame[i], HEX);
    crcAbAppendHexByte(crcAbResultText, frame[i]);
  }
  Serial.println();
  crcAbResultText += "\n";

  setTransmitMode();
  HM485.write(frame, len);
  HM485.flush();

  delayMicroseconds(20);
  setReceiveMode();

  const uint32_t armUs = micros();
  const uint32_t windowUs = 20000UL;
  uint16_t count = 0;
  uint16_t f8 = 0;

  while ((uint32_t)(micros() - armUs) < windowUs)
  {
    while (HM485.available())
    {
      uint8_t b = HM485.read();
      uint32_t delta = (uint32_t)(micros() - armUs);
      count++;
      if (b == 0xF8) f8++;

      Serial.print(F("RX +"));
      Serial.print(delta);
      Serial.print(F("us: "));
      if (b < 0x10) Serial.print('0');
      Serial.println(b, HEX);

      crcAbResultText += "RX +";
      crcAbResultText += String(delta);
      crcAbResultText += "us: ";
      crcAbAppendHexByte(crcAbResultText, b);
      crcAbResultText += "\n";
    }
    delayMicroseconds(10);
  }

  Serial.print(F("RESULT "));
  Serial.print(label);
  Serial.print(F(": bytes="));
  Serial.print(count);
  Serial.print(F(" F8="));
  Serial.println(f8);

  crcAbResultText += "RESULT ";
  crcAbResultText += label;
  crcAbResultText += ": bytes=";
  crcAbResultText += String(count);
  crcAbResultText += " F8=";
  crcAbResultText += String(f8);
  crcAbResultText += "\n";

  if (count == 0)
  {
    Serial.println(F("RX: <none>"));
    crcAbResultText += "RX: <none>\n";
  }
}


void activeDiscoveryStart()
{
  if (rawRxOnlyMode || ioConfig.active || outputWrite.active || quietRootRequested || activeDiscovery.running ||
      passiveCaptureActive || passiveCaptureArmRequested)
    return;

  scanActive = false;
  statusPollActive = false;
  pending.active = false;

  // Wait for a genuinely quiet bus before taking ownership of active discovery.
  quietRootRequested = true;
  lastBusActivityUs = micros();
  setReceiveMode();

  crcAbResultText = "# HM485 NATIVE DISCOVERY\n# Firmware ";
  crcAbResultText += FW_VERSION;
  crcAbResultText += "\n# Algorithm: HM485_Protocol.pm compatible 32-bit prefix tree\n";
  crcAbResultText += "# Start address=00000000 validBits=1 tries=3 timeout_ms=20\n";
  crcAbResultText += "# Positive: FIRST RX byte != 00; zero/timeout is negative\n";
  crcAbResultText += "# Initial continuous bus idle: 600000 us\n";
  crcAbResultText += "# IMPORTANT: stop hm485d/FHEM discovery before running this test.\n";

  webLogAdd("[DISCOVERY] requested - waiting for 600 ms continuous bus idle");
  Serial.println(F("[DISCOVERY] waiting for quiet bus..."));
}

void processPassiveCaptureArm()
{
  if (!quietRootRequested)
    return;

  uint32_t idleUs = (uint32_t)(micros() - lastBusActivityUs);
  if (idleUs < DISCOVERY_REPLAY_IDLE_US)
    return;

  quietRootRequested = false;

  // Safe C++ reset: ActiveDiscoveryState contains String objects.
  activeDiscovery = ActiveDiscoveryState();
  activeDiscovery.running = true;
  activeDiscovery.address = 0x00000000UL;
  activeDiscovery.validBits = 1;
  activeDiscovery.tries = 0;
  activeDiscovery.minAckDeltaUs = 0xFFFFFFFFUL;
  activeDiscovery.nextActionMs = millis();

  crcAbResultText += "# actual_idle_before_start_us=" + String(idleUs) + "\n# BEGIN NATIVE DISCOVERY\n";

  Serial.println(F("[DISCOVERY] native scan started"));
  webLogAdd("[DISCOVERY] native tree scan started");
}

void activeDiscoveryStop()
{
  if (quietRootRequested)
  {
    quietRootRequested = false;
    webLogAdd("[DISCOVERY] cancelled before start");
    Serial.println(F("[DISCOVERY] cancelled before start"));
    return;
  }

  if (activeDiscovery.running)
  {
    activeDiscovery.running = false;
    activeDiscovery.waiting = false;
    setReceiveMode();
    crcAbResultText += "\n# DISCOVERY CANCELLED\n";
    webLogAdd("[DISCOVERY] cancelled");
    Serial.println(F("[DISCOVERY] cancelled"));
    return;
  }

  if (passiveCaptureArmRequested && !passiveCaptureActive)
  {
    passiveCaptureArmRequested = false;
    passiveCaptureArmSinceMs = 0;
    Serial.println(F("[PASSIVE CAPTURE] arm cancelled"));
    webLogAdd("[PASSIVE CAPTURE] arm cancelled");
    return;
  }

  if (!passiveCaptureActive)
    return;

  passiveCaptureActive = false;
  Serial.println(F("[PASSIVE CAPTURE] stopped"));
}

void activeDiscoveryRawByte(uint8_t rawByte)
{
  // Native discovery uses the first raw byte after each source-less probe.
  // Do this before the normal frame parser: F8/F0/00 are not long frames.
  if (activeDiscovery.running && activeDiscovery.waiting)
  {
    activeDiscovery.rawByteCount++;
    activeDiscovery.rxHistogram[rawByte]++;

    if (!activeDiscovery.currentProbeHadRx)
    {
      activeDiscovery.currentProbeHadRx = true;
      activeDiscovery.ackByte = rawByte;
      activeDiscovery.lastAckDeltaUs =
        activeDiscovery.rxArmUs
          ? (uint32_t)(micros() - activeDiscovery.rxArmUs)
          : 0;

      if (activeDiscovery.lastAckDeltaUs < activeDiscovery.minAckDeltaUs)
        activeDiscovery.minAckDeltaUs = activeDiscovery.lastAckDeltaUs;
      if (activeDiscovery.lastAckDeltaUs > activeDiscovery.maxAckDeltaUs)
        activeDiscovery.maxAckDeltaUs = activeDiscovery.lastAckDeltaUs;

      if (rawByte != 0x00)
      {
        activeDiscovery.foundAck = true;
        activeDiscovery.ackCount++;
        if (rawByte == 0xF8)
          activeDiscovery.currentProbeHadF8 = true;
      }

      if (DISCOVERY_VERBOSE_FILE)
      {
        String rx = "[DISCOVERY RX] first=";
        char b[4];
        snprintf(b, sizeof(b), "%02X", rawByte);
        rx += b;
        rx += " +";
        rx += String(activeDiscovery.lastAckDeltaUs);
        rx += "us";
        crcAbResultText += rx + "\n";
      }
    }

    // Any trailing byte (notably the recurring 00 after a positive F8/F0)
    // is diagnostic only and must never overwrite the first-byte decision.
    return;
  }

  if (!passiveCaptureActive)
    return;

  uint16_t idx = passiveCaptureCount;
  if (idx >= PASSIVE_CAPTURE_MAX)
  {
    passiveCaptureFull = true;
    passiveCaptureActive = false;
    return;
  }

  passiveCaptureBuf[idx].tUs =
    (uint32_t)(micros() - passiveCaptureStartUs);
  passiveCaptureBuf[idx].b = rawByte;
  passiveCaptureCount = idx + 1;

  if (passiveCaptureCount >= PASSIVE_CAPTURE_MAX)
  {
    passiveCaptureFull = true;
    passiveCaptureActive = false;
  }
}

void activeDiscoveryForeignFrame(
  uint32_t target,
  uint32_t sender,
  uint8_t ctrl
)
{
  // v0.7.8: decoded frames do NOT decide discovery. The real hm485d
  // discovery response may be only a raw byte. Keep this function only
  // as a diagnostic counter for fully decoded traffic from ourselves.
  (void)target;
  (void)ctrl;

  if (
    activeDiscovery.running &&
    activeDiscovery.waiting &&
    sender == config.localAddress
  )
  {
    activeDiscovery.ignoredOwnFrames++;
  }
}

void activeDiscoveryTreeTrace(const String &line)
{
  if (activeDiscovery.treeTraceCount < ActiveDiscoveryState::TREE_TRACE_MAX)
    activeDiscovery.treeTrace[activeDiscovery.treeTraceCount++] = line;
  else
    activeDiscovery.treeTraceDropped++;
}

bool activeDiscoverySendProbe()
{
  if (!activeDiscovery.running || activeDiscovery.waiting)
    return false;

  if (!busIsIdle())
    return false;

  uint8_t ctrl =
    (uint8_t)(((activeDiscovery.validBits - 1U) << 3) | 0x03U);

  // Prepare first-byte capture before TX. The actual reply cannot arrive
  // until sendLongFrame() has flushed TX and switched DE back to receive.
  activeDiscovery.foundAck = false;
  activeDiscovery.ackByte = 0;
  activeDiscovery.currentProbeHadRx = false;
  activeDiscovery.currentProbeHadF8 = false;

  if (!sendLongFrame(
        activeDiscovery.address,
        ctrl,
        0,
        nullptr,
        0,
        true))
  {
    return false;
  }

  activeDiscovery.probeCount++;
  activeDiscovery.tries++;
  activeDiscovery.waiting = true;
  activeDiscovery.nextActionMs = millis() + ACTIVE_DISCOVERY_TIMEOUT_MS;

  if (DISCOVERY_VERBOSE_SERIAL || DISCOVERY_VERBOSE_FILE)
  {
    String line = "[DISCOVERY] probe=";
    line += String(activeDiscovery.probeCount);
    line += " try=";
    line += String(activeDiscovery.tries);
    line += "/3 DST=";
    line += hexAddress(activeDiscovery.address);
    line += " bits=";
    line += String(activeDiscovery.validBits);
    line += " CTRL=";
    char c[4];
    snprintf(c, sizeof(c), "%02X", ctrl);
    line += c;

    if (DISCOVERY_VERBOSE_SERIAL) Serial.println(line);
    if (DISCOVERY_VERBOSE_FILE)
    {
      crcAbResultText += line;
      crcAbResultText += " wire=";
      crcAbResultText += wireVerifyLastTx;
      crcAbResultText += "\n";
    }
  }

  return true;
}

void activeDiscoveryAdvanceTree()
{
  if (activeDiscovery.validBits == 33)
  {
    activeDiscoveryAddResult(
      activeDiscovery.address
    );

    // hm485d continues as if no answer had been received.
    activeDiscovery.validBits--;
    activeDiscovery.tries =
      ACTIVE_DISCOVERY_TRIES;
  }

  if (
    activeDiscovery.tries ==
    ACTIVE_DISCOVERY_TRIES
  )
  {
    if (activeDiscovery.validBits == 0)
    {
      activeDiscoveryFinish();
      return;
    }

    uint8_t bitIndex =
      32 -
      activeDiscovery.validBits;

    uint32_t bitMask =
      (uint32_t)1UL << bitIndex;

    if (
      (
        activeDiscovery.address &
        bitMask
      ) == 0
    )
    {
      // Least-significant valid bit is 0 -> set it.
      activeDiscovery.address |= bitMask;
    }
    else
    {
      // Set all invalid bits to 1 before incrementing.
      if (
        activeDiscovery.validBits != 32
      )
      {
        activeDiscovery.address |=
          0xFFFFFFFFUL >>
          activeDiscovery.validBits;
      }

      if (
        activeDiscovery.address ==
        0xFFFFFFFFUL
      )
      {
        activeDiscoveryFinish();
        return;
      }

      activeDiscovery.address++;

      activeDiscovery.validBits = 32;

      while (
        activeDiscovery.validBits > 0
      )
      {
        uint8_t testBitIndex =
          32 -
          activeDiscovery.validBits;

        uint32_t testMask =
          (uint32_t)1UL <<
          testBitIndex;

        if (
          (
            activeDiscovery.address &
            testMask
          ) != 0
        )
        {
          break;
        }

        activeDiscovery.validBits--;
      }
    }
  }

  activeDiscovery.foundAck = false;
  activeDiscovery.tries = 0;
  // HM485_Protocol.pm schedules every newly selected branch after the
  // discovery timeout; retries are handled separately at their timeout.
  activeDiscovery.nextActionMs = millis() + ACTIVE_DISCOVERY_TIMEOUT_MS;
}

void processActiveDiscovery()
{
  if (!activeDiscovery.running)
    return;

  const uint32_t now = millis();

  if (activeDiscovery.waiting)
  {
    if ((int32_t)(now - activeDiscovery.nextActionMs) < 0)
      return;

    activeDiscovery.waiting = false;

    // hm485d semantics: discoveryFound is Perl-truthy. Therefore the FIRST
    // received byte is decisive: 0x00 = no hit, any non-zero byte = hit.
    // v0.7.44 measured F8 normally and occasional F0, both on positive paths.
    if (activeDiscovery.foundAck)
    {
      if (DISCOVERY_VERBOSE_SERIAL || DISCOVERY_VERBOSE_FILE)
      {
        String line = "[DISCOVERY] POSITIVE DST=";
        line += hexAddress(activeDiscovery.address);
        line += " bits=";
        line += String(activeDiscovery.validBits);
        line += " byte=";
        char b[4];
        snprintf(b, sizeof(b), "%02X", activeDiscovery.ackByte);
        line += b;
        if (DISCOVERY_VERBOSE_SERIAL) Serial.println(line);
        if (DISCOVERY_VERBOSE_FILE) crcAbResultText += line + "\n";
      }

      // Exact HM485_Protocol.pm behavior: a positive probe descends one bit.
      activeDiscovery.validBits++;

      if (activeDiscovery.validBits == 33)
      {
        activeDiscoveryAddResult(activeDiscovery.address);

        // Continue exactly as if this leaf had been negative: back up one
        // level and force branch advancement below.
        activeDiscovery.validBits--;
        activeDiscovery.tries = ACTIVE_DISCOVERY_TRIES;
      }

      if (activeDiscovery.tries == ACTIVE_DISCOVERY_TRIES)
        activeDiscoveryAdvanceTree();
      else
      {
        activeDiscovery.foundAck = false;
        activeDiscovery.tries = 0;
        // hm485d schedules the next tree step after DISCOVERY_TIMEOUT.
        activeDiscovery.nextActionMs = now + ACTIVE_DISCOVERY_TIMEOUT_MS;
      }
      return;
    }

    // No positive first byte. Retry the exact same prefix up to 3 times.
    if (activeDiscovery.tries >= ACTIVE_DISCOVERY_TRIES)
    {
      if (DISCOVERY_VERBOSE_SERIAL || DISCOVERY_VERBOSE_FILE)
      {
        String line = "[DISCOVERY] NEGATIVE DST=";
        line += hexAddress(activeDiscovery.address);
        line += " bits=";
        line += String(activeDiscovery.validBits);
        if (DISCOVERY_VERBOSE_SERIAL) Serial.println(line);
        if (DISCOVERY_VERBOSE_FILE) crcAbResultText += line + "\n";
      }

      activeDiscoveryAdvanceTree();
      if (!activeDiscovery.running)
        return;
    }
  }

  if ((int32_t)(now - activeDiscovery.nextActionMs) < 0)
    return;

  // activeDiscoveryAdvanceTree() resets tries/foundAck for the new branch.
  // If a negative probe timed out but has tries remaining, address/bits stay
  // unchanged and this simply sends the retry.
  activeDiscoverySendProbe();
}

String activeDiscoveryResultText()
{
  if (crcAbResultText.length() == 0)
  {
    String out = "# HM485 NATIVE DISCOVERY\n# Firmware ";
    out += FW_VERSION;
    out += "\n# Start Native Discovery from the Diagnosis page.\n";
    return out;
  }

  String out = crcAbResultText;
  if (quietRootRequested)
    out += "\n# state: waiting for 600 ms continuous bus idle\n";
  else if (activeDiscovery.running)
  {
    out += "\n# state: running address=";
    out += hexAddress(activeDiscovery.address);
    out += " validBits=";
    out += String(activeDiscovery.validBits);
    out += " tries=";
    out += String(activeDiscovery.tries);
    out += " probes=";
    out += String(activeDiscovery.probeCount);
    out += " found=";
    out += String(activeDiscovery.foundCount);
    out += "\n";
  }
  return out;
}

void sendAck(
  uint32_t target,
  uint8_t receivedTxCounter
)
{
  if (
    activeDiscovery.running ||
    quietRootRequested ||
    passiveCaptureActive ||
    passiveCaptureArmRequested
  )
    return;

  uint8_t ctrl =
    0x19 |
    (
      (receivedTxCounter & 0x03)
      << 5
    );

  sendLongFrame(
    target,
    ctrl,
    config.localAddress,
    nullptr,
    0,
    false
  );

  statAckTx++;
}

bool sendRequest(
  uint32_t target,
  RequestType type,
  const uint8_t *payload,
  uint8_t payloadLen,
  uint8_t busChannel = 0,
  uint16_t eepromAddress = 0,
  uint8_t eepromLength = 0
)
{
  if (
    activeDiscovery.running ||
    quietRootRequested ||
    passiveCaptureActive ||
    passiveCaptureArmRequested
  )
    return false;

  if (
    pending.active ||
    !busIsIdle()
  )
  {
    return false;
  }

  uint8_t txNum =
    localTxCounter & 0x03;

  uint8_t ctrl =
    0x18 |
    (txNum << 1);

  if (
    !sendLongFrame(
      target,
      ctrl,
      config.localAddress,
      payload,
      payloadLen,
      true
    )
  )
  {
    return false;
  }

  pending.active = true;
  pending.type = type;
  pending.target = target;
  pending.txCounter = txNum;
  pending.channel = busChannel;
  pending.eepromAddress = eepromAddress;
  pending.eepromLength = eepromLength;
  pending.sentAt = millis();

  localTxCounter =
    (localTxCounter + 1) &
    0x03;

  return true;
}

// ============================================================
// Individual requests
// ============================================================

bool requestDeviceType(uint32_t target)
{
  const uint8_t payload[] = { 0x68 };

  return sendRequest(
    target,
    RequestType::REQ_DEVICE_TYPE,
    payload,
    sizeof(payload)
  );
}

bool requestSerialNumber(uint32_t target)
{
  const uint8_t payload[] = { 0x6E };

  return sendRequest(
    target,
    RequestType::REQ_SERIAL_NUMBER,
    payload,
    sizeof(payload)
  );
}

bool requestFirmware(uint32_t target)
{
  const uint8_t payload[] = { 0x76 };

  return sendRequest(
    target,
    RequestType::REQ_FIRMWARE,
    payload,
    sizeof(payload)
  );
}

bool requestStatus(
  uint32_t target,
  uint8_t busChannel
)
{
  const uint8_t payload[] =
  {
    0x53,
    busChannel
  };

  return sendRequest(
    target,
    RequestType::REQ_STATUS,
    payload,
    sizeof(payload),
    busChannel
  );
}

bool requestEeprom(
  uint32_t target,
  uint16_t address,
  uint8_t len
)
{
  uint8_t payload[] =
  {
    0x52,
    (uint8_t)((address >> 8) & 0xFF),
    (uint8_t)(address & 0xFF),
    len
  };

  return sendRequest(
    target,
    RequestType::REQ_EEPROM_READ,
    payload,
    sizeof(payload),
    0,
    address,
    len
  );
}

static bool transmitOutputAttempt();

// ============================================================
// v0.9.0 safe EEPROM / I/O configuration helpers
// ============================================================
static bool ioConfigBusSafe()
{
  return !rawRxOnlyMode && !addressGuardBootActive && !addressConflictLocked &&
         !activeDiscovery.running && !quietRootRequested &&
         !passiveCaptureActive && !passiveCaptureArmRequested &&
         !scanActive && !statusPollActive && !outputWrite.active &&
         !buttonPulse.active && !pending.active;
}

static bool sendIoConfigFrame(const uint8_t *payload, uint8_t len)
{
  if (!ioConfig.active || ioConfig.waitingAck || !ioConfigBusSafe() || !busIsIdle())
    return false;

  uint8_t txNum = localTxCounter & 0x03;
  uint8_t ctrl = 0x18 | (txNum << 1);
  if (!sendLongFrame(ioConfig.target, ctrl, config.localAddress, payload, len, true))
    return false;

  ioConfig.txCounter = txNum;
  ioConfig.waitingAck = true;
  ioConfig.sentAt = millis();
  ioConfig.attempts++;
  localTxCounter = (localTxCounter + 1) & 0x03;
  return true;
}

static void ioConfigFail(const String &reason)
{
  String msg = String("[IO CONFIG ERROR] ") + reason;
  Serial.println(msg); webLogAdd(msg);
  ioConfig.active = false;
  ioConfig.waitingAck = false;
  ioConfig.stage = IoConfigStage::ERROR;
}

static void ioConfigSuccess()
{
  HM485Device *d = getDevice(ioConfig.target, false);
  if (d) decodeEepromConfiguration(d);
  String msg = String("[IO CONFIG] verified ") + hexAddress(ioConfig.target) +
               " ch=" + String(busToHmWiredChannel(ioConfig.channel));
  Serial.println(msg); webLogAdd(msg);
  bool setFreq = ioConfig.setFrequencyAfter;
  uint16_t freq = ioConfig.frequencyMilliHz;
  uint8_t ch = ioConfig.channel;
  HM485Device *device = d;
  ioConfig.active = false;
  ioConfig.waitingAck = false;
  ioConfig.stage = IoConfigStage::DONE;
  mqttPublishDiscoveryAll();
  mqttPublishAllStates();
  if (setFreq && device && !outputWrite.active && !pending.active)
  {
    // Same 0x73 value command as observed with FHEM; here the 16-bit value is mHz.
    outputWrite = OutputWriteTransaction{};
    outputWrite.active = true;
    outputWrite.target = device->address;
    outputWrite.channel = ch;
    // Frequency outputs use the same 0x73 + channel + uint16 wire format
    // as the tested digital-output path.  v0.9.2a forgot to initialise
    // the protocol here, leaving outputWrite.active stuck forever because
    // transmitOutputAttempt() rejected RuntimeWriteProtocol::NONE.
    outputWrite.protocol = RuntimeWriteProtocol::SET_73_UINT16_03FF;
    outputWrite.value = freq;
    if (!transmitOutputAttempt())
    {
      String msg = String("[FREQUENCY OUTPUT ERROR] could not start write ") +
                   hexAddress(device->address) + " ch=" +
                   String(busToHmWiredChannel(ch));
      Serial.println(msg);
      webLogAdd(msg);
      outputWrite.active = false;
      outputWrite.waitingAckOrInfo = false;
    }
  }
}

static bool startIoConfiguration(HM485Device *device, uint8_t busChannel,
                                 ChannelBehaviour desired,
                                 bool writeParam, uint16_t paramRaw,
                                 bool setFrequencyAfter, uint16_t frequencyMilliHz)
{
  if (!device || device->deviceType != 0x001C || !device->profile ||
      !device->profile->writeEnabled || ioConfig.active || !ioConfigBusSafe())
    return false;

  const ChannelProfile *cp = findChannelProfile(device->profile, busChannel);
  if (!cp || !cp->configurable || !cp->behaviourEeprom.available)
    return false;

  uint8_t offset = busChannel - cp->first;
  uint16_t absoluteBit = cp->behaviourEeprom.firstBit + offset * cp->behaviourEeprom.bitStep;
  uint16_t modeAddress = cp->behaviourEeprom.byteAddress + absoluteBit / 8;
  uint8_t bit = absoluteBit % 8;
  if (modeAddress >= EEPROM_CACHE_SIZE || !device->eepromValid[modeAddress])
    return false; // explicit read-before-write safety

  bool digital = false;
  if (cp->type == ChannelType::DIGITAL_ANALOG_OUTPUT)
  {
    if (desired == ChannelBehaviour::DIGITAL_OUTPUT) digital = true;
    else if (desired == ChannelBehaviour::ANALOG_OUTPUT) digital = false;
    else return false;
  }
  else if (cp->type == ChannelType::DIGITAL_FREQUENCY_INPUT)
  {
    if (desired == ChannelBehaviour::DIGITAL_INPUT) digital = true;
    else if (desired == ChannelBehaviour::FREQUENCY_INPUT) digital = false;
    else return false;
  }
  else if (cp->type == ChannelType::DIGITAL_ANALOG_INPUT)
  {
    if (desired == ChannelBehaviour::DIGITAL_INPUT) digital = true;
    else if (desired == ChannelBehaviour::ANALOG_INPUT) digital = false;
    else return false;
  }
  else return false;

  ioConfig = IoConfigTransaction{};
  ioConfig.active = true;
  ioConfig.target = device->address;
  ioConfig.channel = busChannel;
  ioConfig.stage = IoConfigStage::WRITE_MODE;
  ioConfig.modeAddress = modeAddress;
  ioConfig.modeValue = device->eeprom[modeAddress];
  if (digital) ioConfig.modeValue |= (1U << bit);
  else ioConfig.modeValue &= ~(1U << bit);
  ioConfig.setFrequencyAfter = setFrequencyAfter;
  ioConfig.frequencyMilliHz = frequencyMilliHz;

  if (writeParam)
  {
    ioConfig.hasParam = true;
    if (cp->type == ChannelType::DIGITAL_ANALOG_OUTPUT)
    {
      ioConfig.paramAddress = 0x0010 + (busChannel - 6) * 2;
      ioConfig.paramLen = 2;
      ioConfig.paramData[0] = (uint8_t)(paramRaw >> 8);
      ioConfig.paramData[1] = (uint8_t)paramRaw;
    }
    else if (cp->type == ChannelType::DIGITAL_ANALOG_INPUT)
    {
      ioConfig.paramAddress = 0x000A + (busChannel - 20);
      ioConfig.paramLen = 1;
      ioConfig.paramData[0] = (uint8_t)paramRaw;
    }
    else
    {
      ioConfig.hasParam = false;
    }
  }

  String msg = String("[IO CONFIG] start ") + hexAddress(device->address) +
               " ch=" + String(busToHmWiredChannel(busChannel)) +
               " mode=" + behaviourName(desired);
  Serial.println(msg); webLogAdd(msg);
  return true;
}

static bool handleIoConfigAck(uint32_t source, uint8_t ackNum)
{
  if (!ioConfig.active || !ioConfig.waitingAck || source != ioConfig.target ||
      ackNum != ioConfig.txCounter)
    return false;

  ioConfig.waitingAck = false;
  ioConfig.attempts = 0;
  switch (ioConfig.stage)
  {
    case IoConfigStage::WRITE_MODE:
      ioConfig.stage = ioConfig.hasParam ? IoConfigStage::WRITE_PARAM : IoConfigStage::APPLY;
      break;
    case IoConfigStage::WRITE_PARAM:
      ioConfig.stage = IoConfigStage::APPLY;
      break;
    case IoConfigStage::APPLY:
      ioConfig.stage = IoConfigStage::VERIFY_MODE_START;
      break;
    default:
      break;
  }
  return true;
}

void processIoConfiguration()
{
  if (!ioConfig.active) return;

  if (ioConfig.waitingAck)
  {
    if (millis() - ioConfig.sentAt <= IO_CONFIG_ACK_TIMEOUT_MS) return;
    ioConfig.waitingAck = false;
    if (ioConfig.attempts >= IO_CONFIG_MAX_ATTEMPTS)
    {
      ioConfigFail("no ACK after 3 attempts");
      return;
    }
    statRetries++;
  }

  if (pending.active) return;

  if (ioConfig.stage == IoConfigStage::WRITE_MODE)
  {
    uint8_t p[] = {0x57, (uint8_t)(ioConfig.modeAddress >> 8), (uint8_t)ioConfig.modeAddress,
                   0x01, ioConfig.modeValue};
    sendIoConfigFrame(p, sizeof(p));
  }
  else if (ioConfig.stage == IoConfigStage::WRITE_PARAM)
  {
    uint8_t p[6] = {0x57, (uint8_t)(ioConfig.paramAddress >> 8), (uint8_t)ioConfig.paramAddress,
                    ioConfig.paramLen, ioConfig.paramData[0], ioConfig.paramData[1]};
    sendIoConfigFrame(p, 4 + ioConfig.paramLen);
  }
  else if (ioConfig.stage == IoConfigStage::APPLY)
  {
    uint8_t p[] = {0x43};
    sendIoConfigFrame(p, sizeof(p));
  }
  else if (ioConfig.stage == IoConfigStage::VERIFY_MODE_START)
  {
    HM485Device *d = getDevice(ioConfig.target, false);
    if (d && ioConfig.modeAddress < EEPROM_CACHE_SIZE) d->eepromValid[ioConfig.modeAddress] = false;
    if (requestEeprom(ioConfig.target, ioConfig.modeAddress, 1))
      ioConfig.stage = IoConfigStage::VERIFY_MODE_WAIT;
  }
  else if (ioConfig.stage == IoConfigStage::VERIFY_MODE_WAIT)
  {
    HM485Device *d = getDevice(ioConfig.target, false);
    if (!pending.active)
    {
      if (!d || !d->eepromValid[ioConfig.modeAddress] || d->eeprom[ioConfig.modeAddress] != ioConfig.modeValue)
      { ioConfigFail("mode read-back mismatch"); return; }
      ioConfig.stage = ioConfig.hasParam ? IoConfigStage::VERIFY_PARAM_START : IoConfigStage::DONE;
    }
  }
  else if (ioConfig.stage == IoConfigStage::VERIFY_PARAM_START)
  {
    HM485Device *d = getDevice(ioConfig.target, false);
    if (d) for (uint8_t i=0;i<ioConfig.paramLen;i++)
      if (ioConfig.paramAddress+i < EEPROM_CACHE_SIZE) d->eepromValid[ioConfig.paramAddress+i] = false;
    if (requestEeprom(ioConfig.target, ioConfig.paramAddress, ioConfig.paramLen))
      ioConfig.stage = IoConfigStage::VERIFY_PARAM_WAIT;
  }
  else if (ioConfig.stage == IoConfigStage::VERIFY_PARAM_WAIT)
  {
    HM485Device *d = getDevice(ioConfig.target, false);
    if (!pending.active)
    {
      bool ok = d != nullptr;
      for (uint8_t i=0; ok && i<ioConfig.paramLen; i++)
        ok = d->eepromValid[ioConfig.paramAddress+i] &&
             d->eeprom[ioConfig.paramAddress+i] == ioConfig.paramData[i];
      if (!ok) { ioConfigFail("parameter read-back mismatch"); return; }
      ioConfig.stage = IoConfigStage::DONE;
    }
  }

  if (ioConfig.stage == IoConfigStage::DONE)
    ioConfigSuccess();
}

// ============================================================
// v0.8.0 digital output control
// ============================================================
static void finishOutputWrite(bool confirmed)
{
  if (!outputWrite.active) return;
  uint32_t target = outputWrite.target;
  uint8_t channel = outputWrite.channel;
  uint16_t value = outputWrite.value;
  outputWrite.active = false;
  outputWrite.waitingAckOrInfo = false;

  String msg = String("[OUTPUT] ") + hexAddress(target) + " ch=" +
               String(busToHmWiredChannel(channel)) + " value=" + String(value) +
               (confirmed ? " confirmed" : " accepted; verify queued");
  Serial.println(msg);
  webLogAdd(msg);

  if (buttonPulse.active && buttonPulse.deviceAddress == target &&
      buttonPulse.channel == channel && buttonPulse.dueMs == 0)
  {
    buttonPulse.dueMs = millis() + buttonPulse.durationMs;
    String bmsg = String("[BUTTON] output confirmed, return due in ") +
                  String(buttonPulse.durationMs) + " ms addr=" +
                  hexAddress(target) + " ch=" +
                  String(busToHmWiredChannel(channel));
    Serial.println(bmsg);
    webLogAdd(bmsg);
  }

  if (!confirmed)
  {
    outputWrite.verifyPending = true;
    outputWrite.target = target;
    outputWrite.channel = channel;
    outputVerifyDueMs = millis() + OUTPUT_VERIFY_DELAY_MS;
  }
}

static bool transmitOutputAttempt()
{
  if (!outputWrite.active || ioConfig.active || pending.active || !busIsIdle() || rawRxOnlyMode ||
      addressGuardBootActive || addressConflictLocked || activeDiscovery.running ||
      quietRootRequested || passiveCaptureActive || passiveCaptureArmRequested)
    return false;

  uint8_t txNum = localTxCounter & 0x03;
  uint8_t ctrl = 0x18 | (txNum << 1);
  uint8_t payload[4] = {};
  uint8_t payloadLen = 0;

  if (outputWrite.protocol == RuntimeWriteProtocol::SET_73_UINT16_03FF)
  {
    payload[0] = 0x73;
    payload[1] = outputWrite.channel;
    payload[2] = (uint8_t)(outputWrite.value >> 8);
    payload[3] = (uint8_t)(outputWrite.value & 0xFF);
    payloadLen = 4;
  }
  else if (outputWrite.protocol == RuntimeWriteProtocol::SET_78_UINT8_C8)
  {
    payload[0] = 0x78;
    payload[1] = outputWrite.channel;
    payload[2] = (uint8_t)(outputWrite.value & 0xFF);
    payloadLen = 3;
  }
  else
    return false;

  if (!sendLongFrame(outputWrite.target, ctrl, config.localAddress,
                     payload, payloadLen, true))
    return false;

  outputWrite.txCounter = txNum;
  outputWrite.attempts++;
  outputWrite.sentAt = millis();
  outputWrite.waitingAckOrInfo = true;
  localTxCounter = (localTxCounter + 1) & 0x03;
  Serial.printf("[OUTPUT TX] %s ch=%u value=%u attempt=%u/%u\n",
                hexAddress(outputWrite.target).c_str(),
                busToHmWiredChannel(outputWrite.channel), outputWrite.value,
                outputWrite.attempts, OUTPUT_WRITE_MAX_ATTEMPTS);
  return true;
}

bool startDigitalOutputWrite(HM485Device *device, uint8_t busChannel, bool on)
{
  if (!device || !channelIsWritableOutput(device, busChannel)) return false;
  if (outputWrite.active || ioConfig.active || pending.active || rawRxOnlyMode || addressGuardBootActive ||
      addressConflictLocked || scanActive || statusPollActive || activeDiscovery.running)
    return false;
  outputWrite = OutputWriteTransaction{};
  outputWrite.active = true;
  outputWrite.target = device->address;
  outputWrite.channel = busChannel;
  outputWrite.protocol = runtimeWriteProtocol(device, busChannel);
  if (outputWrite.protocol == RuntimeWriteProtocol::SET_73_UINT16_03FF)
    outputWrite.value = on ? 0x03FF : 0x0000;
  else if (outputWrite.protocol == RuntimeWriteProtocol::SET_78_UINT8_C8)
    outputWrite.value = on ? 0x00C8 : 0x0000;
  else
  {
    outputWrite.active = false;
    return false;
  }

  // The command transaction is accepted once outputWrite has been prepared.
  // transmitOutputAttempt() may legitimately return false here when the HM485
  // bus has just been active (for example: input event -> MQTT -> HA automation
  // -> output command within a few milliseconds).  In that case keep the
  // transaction queued; processOutputWrite() will transmit it as soon as the
  // bus is idle.  Returning false here used to cancel buttonPulse while the
  // queued outputWrite later still switched the relay ON, leaving it stuck ON.
  transmitOutputAttempt();
  return true;
}

bool startFrequencyOutputWrite(HM485Device *device, uint8_t busChannel, uint16_t milliHz)
{
  if (!device || !channelIsFrequencyOutput(device, busChannel)) return false;
  if (outputWrite.active || ioConfig.active || pending.active || rawRxOnlyMode || addressGuardBootActive ||
      addressConflictLocked || scanActive || statusPollActive || activeDiscovery.running ||
      quietRootRequested || passiveCaptureActive || passiveCaptureArmRequested)
    return false;

  outputWrite = OutputWriteTransaction{};
  outputWrite.active = true;
  outputWrite.target = device->address;
  outputWrite.channel = busChannel;
  outputWrite.protocol = RuntimeWriteProtocol::SET_73_UINT16_03FF;
  outputWrite.value = milliHz;

  // Same queued-start semantics as digital outputs: a momentarily busy bus is
  // not a failed command. processOutputWrite() retries once it becomes idle.
  transmitOutputAttempt();
  return true;
}

static bool handleOutputInfoFrame(uint32_t source, const uint8_t *payload, uint8_t payloadLen)
{
  if (!outputWrite.active || source != outputWrite.target || payloadLen < 3 ||
      payload[0] != 0x69 || payload[1] != outputWrite.channel)
    return false;

  uint16_t value = 0;
  uint8_t valueSize = 0;
  if (outputWrite.protocol == RuntimeWriteProtocol::SET_73_UINT16_03FF)
  {
    if (payloadLen < 4) return false;
    value = ((uint16_t)payload[2] << 8) | payload[3];
    valueSize = 2;
  }
  else if (outputWrite.protocol == RuntimeWriteProtocol::SET_78_UINT8_C8)
  {
    // HBW-LC-Sw8 returns 69 <channel> <level> 00.
    // The logical switch level is the first value byte.
    value = payload[2];
    valueSize = 1;
  }
  else
    return false;

  HM485Device *d = getDevice(source, false);
  if (d && outputWrite.channel < MAX_CHANNELS)
  {
    auto &st = d->channel[outputWrite.channel];
    st.known = true; st.value = value; st.valueSize = valueSize; st.lastUpdate = millis();
    d->lastSeen = millis();
    mqttPublishChannel(d, outputWrite.channel);
  }
  if (value == outputWrite.value)
  {
    finishOutputWrite(true);
    return true;
  }
  return false;
}

static bool handleOutputAck(uint32_t source, uint8_t ackNum)
{
  if (!outputWrite.active || source != outputWrite.target || !outputWrite.waitingAckOrInfo)
    return false;
  if (ackNum != outputWrite.txCounter) return false;
  finishOutputWrite(false);
  return true;
}

void processOutputWrite()
{
  if (outputWrite.active && outputWrite.waitingAckOrInfo &&
      millis() - outputWrite.sentAt > OUTPUT_WRITE_TIMEOUT_MS)
  {
    outputWrite.waitingAckOrInfo = false;
    statTimeouts++;
    if (outputWrite.attempts < OUTPUT_WRITE_MAX_ATTEMPTS)
    {
      statRetries++;
      transmitOutputAttempt();
    }
    else
    {
      String msg = String("[OUTPUT ERROR] no ACK/INFO after 3 attempts: ") +
                   hexAddress(outputWrite.target) + " ch=" +
                   String(busToHmWiredChannel(outputWrite.channel));
      Serial.println(msg); webLogAdd(msg);
      if (buttonPulse.active && buttonPulse.deviceAddress == outputWrite.target &&
          buttonPulse.channel == outputWrite.channel && buttonPulse.dueMs == 0)
        buttonPulse.active = false;
      outputWrite.active = false;
    }
  }
  else if (outputWrite.active && !outputWrite.waitingAckOrInfo)
  {
    transmitOutputAttempt();
  }

  if (outputWrite.verifyPending && !outputWrite.active && !pending.active &&
      !scanActive && !statusPollActive && millis() >= outputVerifyDueMs)
  {
    uint32_t t = outputWrite.target; uint8_t ch = outputWrite.channel;
    outputWrite.verifyPending = false;
    requestStatus(t, ch);
  }
}

void processButtonPulse()
{
  if (!buttonPulse.active || buttonPulse.dueMs == 0 ||
      (int32_t)(millis() - buttonPulse.dueMs) < 0) return;

  if (outputWrite.active || pending.active || scanActive || statusPollActive)
  {
    static uint32_t lastButtonBlockedLogMs = 0;
    if (millis() - lastButtonBlockedLogMs >= 500)
    {
      lastButtonBlockedLogMs = millis();
      String bmsg = String("[BUTTON] return due but blocked: output=") +
                    String(outputWrite.active ? 1 : 0) +
                    " pending=" + String(pending.active ? 1 : 0) +
                    " scan=" + String(scanActive ? 1 : 0) +
                    " poll=" + String(statusPollActive ? 1 : 0);
      Serial.println(bmsg);
      webLogAdd(bmsg);
    }
    return;
  }

  HM485Device *d = getDevice(buttonPulse.deviceAddress, false);
  if (!d)
  {
    String bmsg = String("[BUTTON] return failed: device not found addr=") +
                  hexAddress(buttonPulse.deviceAddress);
    Serial.println(bmsg);
    webLogAdd(bmsg);
    buttonPulse.active = false;
    return;
  }

  String bmsg = String("[BUTTON] return starting addr=") +
                hexAddress(buttonPulse.deviceAddress) + " ch=" +
                String(busToHmWiredChannel(buttonPulse.channel)) +
                " value=" + String(buttonPulse.returnOn ? "ON" : "OFF");
  Serial.println(bmsg);
  webLogAdd(bmsg);

  if (startDigitalOutputWrite(d, buttonPulse.channel, buttonPulse.returnOn))
  {
    buttonPulse.active = false;
  }
  else
  {
    String emsg = String("[BUTTON] return start rejected");
    Serial.println(emsg);
    webLogAdd(emsg);
  }
}

static HM485Device *findDeviceByMqttKey(const String &key)
{
  for (uint8_t i=0;i<MAX_DEVICES;i++)
    if (devices[i].used && mqttDeviceKey(&devices[i]) == key) return &devices[i];
  return nullptr;
}

void mqttMessageCallback(char *topicChars, byte *payloadBytes, unsigned int length)
{
  String topic(topicChars), prefix = config.mqttBaseTopic + "/";
  if (!topic.startsWith(prefix) || !topic.endsWith("/set")) return;
  String rest = topic.substring(prefix.length());
  int p = rest.indexOf("/channel/"); if (p < 1) return;
  String key = rest.substring(0,p);
  String tail = rest.substring(p+9); int slash=tail.indexOf('/'); if (slash<1) return;
  int hmChannel = tail.substring(0,slash).toInt(); if (hmChannel<1 || hmChannel>MAX_CHANNELS) return;
  uint8_t ch=(uint8_t)(hmChannel-1);
  HM485Device *d=findDeviceByMqttKey(key);
  if (!d || (!channelIsWritableOutput(d,ch) && !channelIsFrequencyOutput(d,ch))) return;
  String cmd; cmd.reserve(length); for(unsigned int i=0;i<length;i++) cmd += (char)payloadBytes[i]; cmd.trim();

  if (channelIsFrequencyOutput(d,ch))
  {
    float hz = cmd.toFloat();
    if (hz < 0.0f) hz = 0.0f;
    if (hz > 65.535f) hz = 65.535f;
    uint16_t milliHz = (uint16_t)(hz * 1000.0f + 0.5f);
    startFrequencyOutputWrite(d,ch,milliHz);
    return;
  }

  cmd.toUpperCase();
  SemanticProfile sp=channelSemanticProfile(d,ch);
  if (sp == SemanticProfile::OUTPUT_SWITCH)
  {
    if (cmd=="ON") startDigitalOutputWrite(d,ch,true);
    else if (cmd=="OFF") startDigitalOutputWrite(d,ch,false);
  }
  else if (sp == SemanticProfile::OUTPUT_BUTTON && (cmd=="PRESS" || cmd=="ON"))
  {
    if (buttonPulse.active)
    {
      String bmsg = String("[BUTTON] ignored: pulse already active addr=") +
                    hexAddress(d->address) + " ch=" +
                    String(busToHmWiredChannel(ch));
      Serial.println(bmsg);
      webLogAdd(bmsg);
      return;
    }

    bool restOn=d->buttonRestOn[ch];
    buttonPulse.active=true;
    buttonPulse.deviceAddress=d->address;
    buttonPulse.channel=ch;
    buttonPulse.returnOn=restOn;
    buttonPulse.durationMs=(d->buttonPulseMs[ch]?d->buttonPulseMs[ch]:2000);
    buttonPulse.dueMs=0;

    String bmsg = String("[BUTTON] PRESS addr=") + hexAddress(d->address) +
                  " ch=" + String(busToHmWiredChannel(ch)) +
                  " duration=" + String(buttonPulse.durationMs) +
                  " ms rest=" + String(restOn ? "ON" : "OFF");
    Serial.println(bmsg);
    webLogAdd(bmsg);

    if (!startDigitalOutputWrite(d,ch,!restOn))
    {
      String emsg = String("[BUTTON] initial output start rejected");
      Serial.println(emsg);
      webLogAdd(emsg);
      buttonPulse.active=false;
    }
  }
}

// ============================================================
// Scan helpers
// ============================================================

void advanceScan()
{
  scanRetry = 0;

  switch (scanStep)
  {
    case ScanStep::STEP_TYPE:
      scanStep = ScanStep::STEP_SERIAL_NUMBER;
      break;

    case ScanStep::STEP_SERIAL_NUMBER:
      scanStep = ScanStep::STEP_FIRMWARE;
      break;

    case ScanStep::STEP_FIRMWARE:
      scanStep = ScanStep::STEP_EEPROM0;
      break;

    case ScanStep::STEP_EEPROM0:
      scanStep = ScanStep::STEP_EEPROM1;
      break;

    case ScanStep::STEP_EEPROM1:
      scanBusChannel = 0;
      scanStep = ScanStep::STEP_STATUS;
      break;

    case ScanStep::STEP_STATUS:
      scanBusChannel++;
      break;

    default:
      break;
  }

  lastScanActionMs = millis();
}

uint8_t deviceChannelCount(
  HM485Device *device
)
{
  if (!device || !device->profile)
    return 0;

  // Channel count describes the device/profile and must NOT depend on whether
  // active polling is considered safe.  HBW devices such as HBW-LC-Sw8 may
  // deliberately have native discovery / active reads disabled, but their
  // profile still defines real channels that must exist in Web UI, MQTT and HA.
  uint8_t count = device->profile->nominalChannelCount;

  for (
    uint8_t i = 0;
    i < device->profile->channelGroupCount;
    i++
  )
  {
    const ChannelProfile &cp =
      device->profile->channels[i];

    uint8_t end =
      cp.first +
      cp.count;

    if (end > count)
      count = end;
  }

  return count;
}

// v0.7.46: native discovery is the authoritative device source.
uint8_t activeDeviceCount()
{
  return activeDiscovery.foundCount;
}

uint32_t activeDeviceAddress(uint8_t index)
{
  if (index >= activeDiscovery.foundCount)
    return 0;
  return activeDiscovery.foundAddress[index];
}

// Runtime status polling must not be tied to native tree discovery.
// HBW devices can be learned only from passive/broadcast traffic and therefore
// may never appear in activeDiscovery.foundAddress[].  Poll only devices whose
// profile explicitly declares active reads safe.
uint8_t runtimePollDeviceCount()
{
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_DEVICES; i++)
  {
    HM485Device &d = devices[i];

    if (
      d.used &&
      d.profile &&
      d.profile->activeReadSafe &&
      deviceChannelCount(&d) > 0
    )
    {
      count++;
    }
  }

  return count;
}

uint32_t runtimePollDeviceAddress(uint8_t index)
{
  uint8_t current = 0;

  for (uint8_t i = 0; i < MAX_DEVICES; i++)
  {
    HM485Device &d = devices[i];

    if (
      !d.used ||
      !d.profile ||
      !d.profile->activeReadSafe ||
      deviceChannelCount(&d) == 0
    )
    {
      continue;
    }

    if (current == index)
      return d.address;

    current++;
  }

  return 0;
}

// ============================================================
// Status poll
// ============================================================

void startStatusPoll()
{
  if (
    statusPollActive ||
    scanActive ||
    pending.active ||
    ioConfig.active ||
    outputWrite.active ||
    activeDiscovery.running ||
    quietRootRequested ||
    passiveCaptureActive ||
    passiveCaptureArmRequested
  )
  {
    return;
  }

  statusPollActive = true;
  statusPollSingleDevice = false;
  statusPollSingleAddress = 0;
  statusPollDeviceIndex = 0;
  statusPollBusChannel = 0;
  lastStatusPollActionMs = 0;

  Serial.println(F("[POLL] Starting status refresh"));
}

bool startStatusPollForDevice(uint32_t address)
{
  if (
    statusPollActive ||
    scanActive ||
    pending.active ||
    ioConfig.active ||
    outputWrite.active ||
    activeDiscovery.running ||
    quietRootRequested ||
    passiveCaptureActive ||
    passiveCaptureArmRequested ||
    rawRxOnlyMode ||
    addressGuardBootActive ||
    addressConflictLocked
  )
  {
    return false;
  }

  const uint8_t count = runtimePollDeviceCount();
  for (uint8_t i = 0; i < count; i++)
  {
    if (runtimePollDeviceAddress(i) != address)
      continue;

    HM485Device *d = getDevice(address, false);
    if (!d || !d->profile || !d->profile->activeReadSafe || deviceChannelCount(d) == 0)
      return false;

    statusPollActive = true;
    statusPollSingleDevice = true;
    statusPollSingleAddress = address;
    statusPollDeviceIndex = i;
    statusPollBusChannel = 0;
    lastStatusPollActionMs = 0;

    String msg = String("[POLL] Starting targeted status refresh: ") + hexAddress(address);
    Serial.println(msg);
    webLogAdd(msg);
    return true;
  }

  return false;
}

void advanceStatusPoll()
{
  if (!statusPollActive)
    return;

  statusPollBusChannel++;

  if (statusPollDeviceIndex >= runtimePollDeviceCount())
    return;

  HM485Device *d =
    getDevice(
      runtimePollDeviceAddress(statusPollDeviceIndex),
      false
    );

  uint8_t count =
    (d && d->profile && d->profile->activeReadSafe)
      ? deviceChannelCount(d)
      : 0;

  if (
    !d ||
    count == 0 ||
    statusPollBusChannel >= count
  )
  {
    if (statusPollSingleDevice)
    {
      statusPollActive = false;
      lastStatusRefreshCompletedMs = millis();
      String msg = String("[POLL] Targeted status refresh finished: ") +
                   hexAddress(statusPollSingleAddress);
      Serial.println(msg);
      webLogAdd(msg);

      // Each REQ_STATUS response already publishes its channel immediately.
      // Republish all known retained states once more here as a robust final
      // synchronization step for HA/MQTT after an HBW restart.
      if (mqttClient.connected())
      {
        mqttSubscribeOutputCommands();
        mqttPublishAllStates();
        webLogAdd(String("[MQTT] HBW targeted poll states/subscriptions refreshed: ") +
                  hexAddress(statusPollSingleAddress));
      }

      statusPollSingleDevice = false;
      statusPollSingleAddress = 0;
      return;
    }

    statusPollDeviceIndex++;
    statusPollBusChannel = 0;
  }

  if (
    statusPollDeviceIndex >=
    runtimePollDeviceCount()
  )
  {
    statusPollActive = false;
    statusPollSingleDevice = false;
    statusPollSingleAddress = 0;
    lastStatusRefreshCompletedMs = millis();

    Serial.println(F("[POLL] Status refresh finished"));
  }
}

void processStatusPoll()
{
  if (
    !statusPollActive ||
    scanActive ||
    pending.active
  )
  {
    return;
  }

  if (
    millis() -
    lastStatusPollActionMs <
    REQUEST_GAP_MS
  )
  {
    return;
  }

  while (
    statusPollDeviceIndex <
    runtimePollDeviceCount()
  )
  {
    HM485Device *d =
      getDevice(
        runtimePollDeviceAddress(statusPollDeviceIndex),
        false
      );

    uint8_t count =
      (d && d->profile && d->profile->activeReadSafe)
        ? deviceChannelCount(d)
        : 0;

    if (
      !d ||
      count == 0 ||
      statusPollBusChannel >= count
    )
    {
      statusPollDeviceIndex++;
      statusPollBusChannel = 0;
      continue;
    }

    if (
      requestStatus(
        d->address,
        statusPollBusChannel
      )
    )
    {
      lastStatusPollActionMs = millis();
    }

    return;
  }

  statusPollActive = false;
  statusPollSingleDevice = false;
  statusPollSingleAddress = 0;
  lastStatusRefreshCompletedMs = millis();
}

// ============================================================
// Handle responses
// ============================================================

bool handleResponse(
  uint32_t source,
  const uint8_t *payload,
  uint8_t payloadLen
)
{
  if (
    !pending.active ||
    source != pending.target
  )
  {
    return false;
  }

  HM485Device *device =
    getDevice(source);

  if (!device)
    return false;

  bool valid = false;
  uint8_t publishedBusChannel = 0;
  bool publishChannel = false;

  const bool responseSerialVisible =
    pending.type == RequestType::REQ_DEVICE_TYPE ||
    pending.type == RequestType::REQ_SERIAL_NUMBER ||
    pending.type == RequestType::REQ_FIRMWARE ||
    (pending.type == RequestType::REQ_EEPROM_READ && SCAN_VERBOSE_EEPROM) ||
    (pending.type == RequestType::REQ_STATUS && SCAN_VERBOSE_STATUS);

  if (responseSerialVisible)
  {
    Serial.println();
    Serial.print(F("[RESPONSE] "));
    printAddress(source);
    Serial.print(F("  "));
  }

  switch (pending.type)
  {
    case RequestType::REQ_DEVICE_TYPE:
    {
      if (payloadLen >= 2)
      {
        uint16_t type =
          ((uint16_t)payload[1] << 8) |
          payload[0];

        setDeviceType(
          device,
          type
        );

        Serial.printf(
          "TYPE = 0x%04X",
          type
        );

        if (device->profile)
        {
          Serial.print(' ');
          Serial.print(device->profile->model);
        }

        valid = true;
      }
      break;
    }

    case RequestType::REQ_SERIAL_NUMBER:
    {
      if (payloadLen > 0)
      {
        uint8_t len =
          min(
            payloadLen,
            (uint8_t)(
              sizeof(device->serialNumber) - 1
            )
          );

        memcpy(
          device->serialNumber,
          payload,
          len
        );

        device->serialNumber[len] = '\0';
        device->serialKnown = true;

        Serial.print(F("SERIAL = "));
        Serial.print(device->serialNumber);

        valid = true;
      }
      break;
    }

    case RequestType::REQ_FIRMWARE:
    {
      if (payloadLen >= 2)
      {
        device->firmware =
          ((uint16_t)payload[0] << 8) |
          payload[1];

        device->firmwareKnown = true;

        Serial.printf(
          "FW = 0x%04X",
          device->firmware
        );

        valid = true;
      }
      break;
    }

    case RequestType::REQ_EEPROM_READ:
    {
      if (
        payloadLen ==
        pending.eepromLength
      )
      {
        if (SCAN_VERBOSE_EEPROM)
        {
          Serial.printf(
            "EEPROM 0x%04X = ",
            pending.eepromAddress
          );

          for (uint8_t i = 0; i < payloadLen; i++)
          {
            Serial.printf("%02X", payload[i]);
            if (i + 1 < payloadLen) Serial.print(' ');
          }
        }

        storeEepromBlock(
          device,
          pending.eepromAddress,
          payload,
          payloadLen
        );

        valid = true;
      }
      break;
    }

    case RequestType::REQ_STATUS:
    {
      uint8_t busChannel =
        pending.channel;

      uint8_t expectedSize =
        expectedValueSize(
          device,
          busChannel
        );

      uint8_t valueSize = 0;
      uint32_t value = 0;

      if (
        payloadLen >= 3 &&
        payload[0] == 0x69 &&
        payload[1] == busChannel
      )
      {
        if (
          expectedSize == 1 &&
          payloadLen >= 3
        )
        {
          value = payload[2];
          valueSize = 1;
          valid = true;
        }
        else if (
          expectedSize == 2 &&
          payloadLen >= 4
        )
        {
          value =
            ((uint16_t)payload[2] << 8) |
            payload[3];

          valueSize = 2;
          valid = true;
        }
        else if (
          expectedSize == 3 &&
          payloadLen >= 5
        )
        {
          value =
            ((uint32_t)payload[2] << 16) |
            ((uint32_t)payload[3] << 8) |
            payload[4];

          valueSize = 3;
          valid = true;
        }
        else if (expectedSize == 0)
        {
          if (payloadLen >= 4)
          {
            value =
              ((uint16_t)payload[2] << 8) |
              payload[3];

            valueSize = 2;
            valid = true;
          }
          else
          {
            value = payload[2];
            valueSize = 1;
            valid = true;
          }
        }
      }

      if (
        valid &&
        busChannel <
        MAX_CHANNELS
      )
      {
        device->channel[busChannel].known = true;
        device->channel[busChannel].value = value;
        device->channel[busChannel].valueSize = valueSize;
        device->channel[busChannel].lastUpdate = millis();

        if (SCAN_VERBOSE_STATUS)
        {
          Serial.printf(
            "STATUS HMWIRED=%u BUS=%u value=0x%lX",
            busToHmWiredChannel(busChannel),
            busChannel,
            value
          );
        }

        publishedBusChannel = busChannel;
        publishChannel = true;
      }

      break;
    }

    default:
      break;
  }

  if (responseSerialVisible)
    Serial.println();

  if (valid)
  {
    pending.active = false;

    if (publishChannel)
      mqttPublishChannel(
        device,
        publishedBusChannel
      );

    if (scanActive)
    {
      advanceScan();
    }
    else if (statusPollActive)
    {
      advanceStatusPoll();
    }
  }

  return valid;
}

// ============================================================
// Passive HM485 status/event snooping
//
// Frames not addressed to config.localAddress are NEVER ACKed.
// We only observe device -> other-central traffic and decode
// the same 0x69 channel-info payload used by status responses.
// This allows FHEM (00000001) and the ESP (00000002) to coexist.
// ============================================================

bool handlePassiveInfoFrame(
  uint32_t sender,
  uint32_t target,
  const uint8_t *payload,
  uint8_t payloadLen
)
{
  // Never interpret traffic originating from either central
  // as a device status update.
  if (
    sender == 0 ||
    sender == config.localAddress ||
    sender == FHEM_ADDRESS
  )
  {
    return false;
  }

  // HBW devices may have native tree discovery disabled. They still announce
  // their complete identity as a broadcast:
  //   41 <channel> <type_lo> <type_hi> <fw_hi> <fw_lo> <10-byte serial>
  //
  // IMPORTANT: this is intentionally handled here, in the already-existing
  // passive/broadcast path. Native discovery has already processed the frame
  // before this function is called, so HBW support cannot disturb its strict
  // timing/decision path.
  if (
    payload &&
    payloadLen >= 16 &&
    payload[0] == 0x41
  )
  {
    HM485Device *identityDevice = getDevice(sender, true);
    if (!identityDevice)
      return false;

    const uint16_t type =
      static_cast<uint16_t>(payload[2]) |
      (static_cast<uint16_t>(payload[3]) << 8);

    const uint16_t firmware =
      (static_cast<uint16_t>(payload[4]) << 8) |
      static_cast<uint16_t>(payload[5]);

    char serial[11] = {};
    memcpy(serial, payload + 6, 10);
    serial[10] = '\0';

    const bool changed =
      !identityDevice->typeKnown ||
      identityDevice->deviceType != type ||
      !identityDevice->firmwareKnown ||
      identityDevice->firmware != firmware ||
      !identityDevice->serialKnown ||
      strncmp(identityDevice->serialNumber, serial, 10) != 0;

    setDeviceType(identityDevice, type);
    identityDevice->firmware = firmware;
    identityDevice->firmwareKnown = true;
    memcpy(identityDevice->serialNumber, serial, 11);
    identityDevice->serialKnown = true;
    identityDevice->lastSeen = millis();

    if (changed)
    {
      String msg =
        String("[IDENTITY] ") + hexAddress(sender) +
        " type=0x" + String(type, HEX) +
        " model=" + String(identityDevice->profile ? identityDevice->profile->model : "unknown") +
        " fw=" + String((firmware >> 8) & 0xFF) + "." + String(firmware & 0xFF) +
        " serial=" + String(identityDevice->serialNumber);

      Serial.println(msg);
      webLogAdd(msg);
      saveDeviceMetadata(identityDevice);

    }

    // The gateway may connect to MQTT before a passive HBW device has announced
    // itself in this boot. Even when metadata was restored from NVS and therefore
    // 'changed' is false, refresh discovery and especially /set subscriptions now.
    if (mqttClient.connected())
    {
      mqttPublishGatewayDiscovery();
      mqttPublishDiscoveryAll();
      mqttSubscribeOutputCommands();
      mqttPublishAllStates();
    }

    // A complete 0x41 announcement already gives us type, firmware and serial.
    // It is also emitted again after an HBW device restart. Therefore this logic
    // MUST run even when type/firmware/serial are unchanged.
    if (passiveDiscoveryLastCandidate == sender)
      passiveDiscoveryScanPending = false;

    if (identityDevice->profile && identityDevice->profile->activeReadSafe)
    {
      passiveIdentityStatusPollPendingAddress = sender;
      String pmsg = String("[POLL] HBW 0x41 seen; targeted refresh queued: ") +
                    hexAddress(sender);
      Serial.println(pmsg);
      webLogAdd(pmsg);
    }

    return true;
  }

  // Known device metadata/profile is required to determine
  // whether the channel value is one, two or three bytes.
  HM485Device *device =
    getDevice(sender, false);

  if (
    !device ||
    !device->profile ||
    payloadLen < 3
  )
  {
    return false;
  }

  // HM485 INFO_LEVEL / channel status payload:
  //   0x69, BUS_CHANNEL, VALUE...
  if (payload[0] != 0x69)
    return false;

  uint8_t busChannel = payload[1];

  if (busChannel >= MAX_CHANNELS)
    return false;

  uint8_t expectedSize =
    expectedValueSize(device, busChannel);

  uint8_t valueSize = 0;
  uint32_t value = 0;

  if (
    expectedSize == 1 &&
    payloadLen >= 3
  )
  {
    value = payload[2];
    valueSize = 1;
  }
  else if (
    expectedSize == 2 &&
    payloadLen >= 4
  )
  {
    value =
      ((uint16_t)payload[2] << 8) |
      payload[3];

    valueSize = 2;
  }
  else if (
    expectedSize == 3 &&
    payloadLen >= 5
  )
  {
    value =
      ((uint32_t)payload[2] << 16) |
      ((uint32_t)payload[3] << 8) |
      payload[4];

    valueSize = 3;
  }
  else if (expectedSize == 0)
  {
    // Conservative fallback for a future/unknown profile.
    if (payloadLen >= 4)
    {
      value =
        ((uint16_t)payload[2] << 8) |
        payload[3];

      valueSize = 2;
    }
    else
    {
      value = payload[2];
      valueSize = 1;
    }
  }
  else
  {
    return false;
  }

  ChannelState &state =
    device->channel[busChannel];

  bool changed =
    !state.known ||
    state.value != value ||
    state.valueSize != valueSize;

  state.known = true;
  state.value = value;
  state.valueSize = valueSize;
  state.lastUpdate = millis();
  device->lastSeen = millis();

  if (changed)
  {
    statPassiveUpdates++;

    Serial.print(F("[PASSIVE] "));
    printAddress(sender);

    Serial.printf(
      " -> %08lX  HMWIRED=%u BUS=%u value=0x%lX\n",
      target,
      busToHmWiredChannel(busChannel),
      busChannel,
      value
    );

    // Retained MQTT state is updated immediately.
    mqttPublishChannel(device, busChannel);
  }

  return true;
}

// ============================================================
// HM485 parser
// ============================================================

class HM485Parser
{
public:
  void feed(uint8_t rawByte)
  {
    statRxBytes++;
    lastBusActivityUs = micros();
    activeDiscoveryRawByte(rawByte);

    if (
      rawByte == ESCAPE_CHAR &&
      !escaped
    )
    {
      escaped = true;
      return;
    }

    if (
      !escaped &&
      (
        rawByte == FRAME_START_LONG ||
        rawByte == FRAME_START_SHORT
      )
    )
    {
      startFrame(rawByte);
      return;
    }

    if (!active)
    {
      escaped = false;
      return;
    }

    uint8_t b = rawByte;

    if (escaped)
    {
      b |= 0x80;
      escaped = false;
    }

    crc = crc16Shift(b, crc);
    processByte(b);
  }

private:
  enum ParseState
  {
    TARGET,
    CONTROL,
    SENDER,
    LENGTH,
    DATA
  };

  bool active = false;
  bool escaped = false;

  ParseState state;

  uint8_t startByte = 0;

  uint8_t targetRaw[4] = {};
  uint8_t senderRaw[4] = {};

  uint8_t targetLength = 0;
  uint8_t targetPos = 0;
  uint8_t senderLength = 0;
  uint8_t senderPos = 0;

  uint8_t ctrl = 0;

  uint8_t dataLength = 0;
  uint8_t dataPos = 0;
  uint8_t data[255] = {};

  uint16_t crc = 0xFFFF;

  void startFrame(uint8_t start)
  {
    active = true;
    escaped = false;
    startByte = start;

    memset(targetRaw, 0, sizeof(targetRaw));
    memset(senderRaw, 0, sizeof(senderRaw));
    memset(data, 0, sizeof(data));

    targetPos = 0;
    senderPos = 0;
    dataPos = 0;
    ctrl = 0;
    dataLength = 0;

    crc = 0xFFFF;
    crc = crc16Shift(startByte, crc);

    targetLength =
      (
        startByte ==
        FRAME_START_LONG
      )
      ? 4
      : 1;

    state = TARGET;
  }

  void processByte(uint8_t b)
  {
    switch (state)
    {
      case TARGET:
        targetRaw[targetPos++] = b;

        if (targetPos >= targetLength)
          state = CONTROL;

        break;

      case CONTROL:
        ctrl = b;

        if (
          !ctrlIsDiscovery(ctrl) &&
          ctrlHasSender(ctrl) &&
          startByte == FRAME_START_LONG
        )
        {
          senderLength = 4;
          senderPos = 0;
          state = SENDER;
        }
        else
        {
          senderLength = 0;
          state = LENGTH;
        }

        break;

      case SENDER:
        senderRaw[senderPos++] = b;

        if (senderPos >= senderLength)
          state = LENGTH;

        break;

      case LENGTH:
        dataLength = b;
        dataPos = 0;

        if (dataLength < 2)
        {
          active = false;
          return;
        }

        state = DATA;
        break;

      case DATA:
        if (dataPos >= sizeof(data))
        {
          active = false;
          return;
        }

        data[dataPos++] = b;

        if (dataPos >= dataLength)
          frameComplete();

        break;
    }
  }

  void frameComplete()
  {
    statFrames++;

    if (crc == 0)
    {
      statCrcOk++;
      decodeFrame();
    }
    else
    {
      statCrcError++;

    }

    active = false;
    escaped = false;
  }

  void decodeFrame()
  {
    if (startByte != FRAME_START_LONG)
      return;

    uint32_t target =
      bytesToAddress(targetRaw);

    uint32_t sender = 0;

    if (senderLength == 4)
      sender = bytesToAddress(senderRaw);

    uint8_t payloadLen =
      dataLength - 2;

    if (hm485RawLogEnabled)
    {
      webLogAdd(
        hm485RawLine(
          "RX",
          target,
          sender,
          ctrl,
          data,
          payloadLen
        )
      );
    }

    // v0.7.49: every CRC-valid decoded frame contributes to passive central
    // address observation and duplicate-own-address protection.
    addressGuardObserveFrame(sender);
    passiveDiscoveryObserveSender(sender);

    // Active discovery v0.7.7:
    // make the ACK decision only from a decoded frame where source
    // and destination are known.
    if (activeDiscovery.running)
    {
      activeDiscoveryForeignFrame(
        target,
        sender,
        ctrl
      );
    }

    // Frames for another participant (usually FHEM 00000001)
    // are observed passively only. NEVER ACK them.
    if (target != config.localAddress)
    {
      handlePassiveInfoFrame(
        sender,
        target,
        data,
        payloadLen
      );

      return;
    }

    // ACK-only frame.
    if (ctrlIsAck(ctrl))
    {
      statAckRx++;

      Serial.print(F("[ACK RX] "));
      printAddress(sender);

      Serial.printf(
        " ack=%u ctrl=0x%02X\n",
        ctrlAckNum(ctrl),
        ctrl
      );
      if (!handleIoConfigAck(sender, ctrlAckNum(ctrl)))
        handleOutputAck(sender, ctrlAckNum(ctrl));

      return;
    }

    // I-frame -> ACK immediately.
    if (
      sender != 0 &&
      ctrlIsIframe(ctrl)
    )
    {
      sendAck(
        sender,
        ctrlTxNum(ctrl)
      );
    }

    // First let active write/request transactions consume their matching
    // replies. These paths update the channel state themselves.
    handleOutputInfoFrame(sender, data, payloadLen);
    handleResponse(
      sender,
      data,
      payloadLen
    );

    // Devices also send unsolicited 0x69 INFO_LEVEL frames directly to the
    // configured central address (for example contact changes). v0.9.2c
    // ACKed those frames but only published 0x69 data when it belonged to an
    // active request/write transaction. Re-use the observer decoder here so
    // spontaneous locally-addressed state changes reach retained MQTT / HA.
    //
    // If handleResponse()/handleOutputInfoFrame() already updated the same
    // state, handlePassiveInfoFrame() sees no change and therefore does not
    // publish a duplicate MQTT state.
    handlePassiveInfoFrame(
      sender,
      target,
      data,
      payloadLen
    );
  }
};

HM485Parser parser;

// ============================================================
// Timeout / retry
// ============================================================

void checkRequestTimeout()
{
  if (!pending.active)
    return;

  if (
    millis() -
    pending.sentAt <=
    RESPONSE_TIMEOUT_MS
  )
  {
    return;
  }

  statTimeouts++;

  Serial.print(F("[TIMEOUT] "));
  printAddress(pending.target);

  if (scanActive)
  {
    Serial.printf(
      " retry=%u/%u\n",
      scanRetry,
      MAX_REQUEST_RETRIES
    );
  }
  else
  {
    Serial.println();
  }

  pending.active = false;

  if (scanActive)
  {
    if (
      scanRetry <
      MAX_REQUEST_RETRIES
    )
    {
      scanRetry++;
      statRetries++;
      lastScanActionMs = millis();
      return;
    }

    Serial.println(
      F("[SCAN] Giving up this request.")
    );

    scanRetry = 0;
    advanceScan();
    return;
  }

  if (statusPollActive)
  {
    // Status refresh is best effort.
    advanceStatusPoll();
  }
}

// ============================================================
// Full scan
// ============================================================

void startScan()
{
  if (passiveCaptureActive || passiveCaptureArmRequested)
  {
    Serial.println(F("[SCAN] Passive capture armed/active."));
    return;
  }


  if (activeDiscovery.running)
  {
    Serial.println(F("[SCAN] Active discovery test running."));
    return;
  }

  if (pending.active || ioConfig.active || outputWrite.active)
  {
    Serial.println(
      F("[SCAN] Request/write currently pending.")
    );

    return;
  }

  statusPollActive = false;

  scanActive = true;
  scanDeviceIndex = 0;
  scanStep = ScanStep::STEP_TYPE;
  scanBusChannel = 0;
  scanRetry = 0;
  lastScanActionMs = 0;

  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F(" HM485 READ-ONLY DEVICE SCAN"));
  Serial.println(F("========================================"));
  webLogAdd("[HM485] Full device scan started");
}

void processScan()
{
  if (
    !scanActive ||
    pending.active
  )
  {
    return;
  }

  if (
    millis() -
    lastScanActionMs <
    REQUEST_GAP_MS
  )
  {
    return;
  }

  if (
    scanDeviceIndex >=
    activeDeviceCount()
  )
  {
    scanActive = false;
    scanStep = ScanStep::DONE;

    Serial.println();
    Serial.println(F("========================================"));
    Serial.println(F(" SCAN FINISHED"));
    Serial.println(F("========================================"));
    webLogAdd("[HM485] Full device scan finished");

    // Persist identity cache plus user names/profile selections. Writes are
    // skipped when the record is unchanged, limiting NVS wear.
    saveAllDeviceMetadata();

    // All metadata should now be known.
    mqttPublishGatewayDiscovery();
    mqttPublishGatewayDiagnostics();
    mqttPublishDiscoveryAll();
    mqttSubscribeOutputCommands();
    mqttPublishAllStates();

    lastStatusRefreshCompletedMs = millis();

    return;
  }

  uint32_t address =
    activeDeviceAddress(scanDeviceIndex);

  HM485Device *device =
    getDevice(address);

  if (!device)
    return;

  bool sent = false;

  switch (scanStep)
  {
    case ScanStep::STEP_TYPE:
      Serial.print(F("[SCAN] TYPE "));
      printAddress(address);
      Serial.println();
      sent = requestDeviceType(address);
      break;

    case ScanStep::STEP_SERIAL_NUMBER:
      sent = requestSerialNumber(address);
      break;

    case ScanStep::STEP_FIRMWARE:
      sent = requestFirmware(address);
      break;

    case ScanStep::STEP_EEPROM0:
      if (device->profile && !device->profile->activeReadSafe)
      {
        webLogAdd(String("[PROFILE] ") + device->profile->model +
                  " known but untested; active EEPROM/status scan skipped");
        scanStep = ScanStep::STEP_NEXT_DEVICE;
        lastScanActionMs = millis();
        return;
      }
      sent = requestEeprom(
        address,
        0x0000,
        0x10
      );
      break;

    case ScanStep::STEP_EEPROM1:
      sent = requestEeprom(
        address,
        0x0010,
        0x10
      );
      break;

    case ScanStep::STEP_STATUS:
    {
      uint8_t count =
        deviceChannelCount(device);

      if (
        count == 0 ||
        scanBusChannel >= count
      )
      {
        scanStep =
          ScanStep::STEP_NEXT_DEVICE;

        lastScanActionMs =
          millis();

        return;
      }

      sent = requestStatus(
        address,
        scanBusChannel
      );

      break;
    }

    case ScanStep::STEP_NEXT_DEVICE:
      scanDeviceIndex++;
      scanStep = ScanStep::STEP_TYPE;
      scanBusChannel = 0;
      scanRetry = 0;
      lastScanActionMs = millis();
      return;

    default:
      return;
  }

  if (sent)
    lastScanActionMs = millis();
}

// ============================================================
// Console output
// ============================================================

void printChannelValue(
  HM485Device &device,
  uint8_t busChannel
)
{
  ChannelState &state =
    device.channel[busChannel];

  if (state.valueSize == 1)
    Serial.printf("0x%02lX", state.value);
  else if (state.valueSize == 2)
    Serial.printf("0x%04lX", state.value);
  else if (state.valueSize == 3)
    Serial.printf("0x%06lX", state.value);
  else
    Serial.printf("0x%lX", state.value);
}

void printDeviceOverview()
{
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F(" HM485 DEVICE TABLE"));
  Serial.println(F("========================================"));

  for (uint8_t i = 0; i < MAX_DEVICES; i++)
  {
    HM485Device &d = devices[i];

    if (!d.used)
      continue;

    Serial.println();

    Serial.print(F("Address : "));
    printAddress(d.address);
    Serial.println();

    if (d.typeKnown)
      Serial.printf("Type    : %04X\n", d.deviceType);

    if (d.profile)
    {
      Serial.print(F("Model   : "));
      Serial.println(d.profile->model);
    }

    if (d.serialKnown)
    {
      Serial.print(F("Serial  : "));
      Serial.println(d.serialNumber);
    }

    if (d.firmwareKnown)
      Serial.printf("FW raw  : %04X\n", d.firmware);

    uint16_t cached = 0;

    for (
      uint16_t a = 0;
      a < EEPROM_CACHE_SIZE;
      a++
    )
    {
      if (d.eepromValid[a])
        cached++;
    }

    Serial.printf(
      "EEPROM  : %u bytes cached\n",
      cached
    );

    Serial.println(F("Channels:"));

    for (
      uint8_t busChannel = 0;
      busChannel < MAX_CHANNELS;
      busChannel++
    )
    {
      if (!d.channel[busChannel].known)
        continue;

      const ChannelProfile *cp =
        findChannelProfile(
          d.profile,
          busChannel
        );

      Serial.printf(
        "  HMWIRED %02u / BUS %02u  %-24s ",
        busToHmWiredChannel(busChannel),
        busChannel,
        cp
          ? channelTypeName(cp->type)
          : "UNKNOWN"
      );

      printChannelValue(
        d,
        busChannel
      );

      if (
        cp &&
        cp->configurable
      )
      {
        Serial.print(F("  ["));
        Serial.print(
          behaviourName(
            d.channel[busChannel].behaviour
          )
        );

        Serial.print(
          d.channel[busChannel].behaviourKnown
            ? F("/EEPROM]")
            : F("/pending]")
        );
      }

      uint16_t pulseRaw = 0;
      float pulseSeconds = 0;

      PulseValueState pulseState =
        getPulseTime(
          &d,
          busChannel,
          pulseRaw,
          pulseSeconds
        );

      if (
        pulseState ==
        PulseValueState::VALID
      )
      {
        if (
          d.channel[busChannel].behaviour ==
          ChannelBehaviour::ANALOG_OUTPUT
        )
        {
          Serial.printf(
            " [pulse %.2fs]",
            pulseSeconds
          );
        }
        else
        {
          Serial.printf(
            " [pulse stored %.2fs]",
            pulseSeconds
          );
        }
      }
      else if (
        pulseState ==
        PulseValueState::SPECIAL
      )
      {
        Serial.printf(
          " [pulse special 0x%04X]",
          pulseRaw
        );
      }

      Serial.println();
    }
  }

  Serial.println();

  Serial.printf("RX frames : %lu\n", statFrames);
  Serial.printf("CRC OK    : %lu\n", statCrcOk);
  Serial.printf("CRC errors: %lu\n", statCrcError);
  Serial.printf("TX frames : %lu\n", statTxFrames);
  Serial.printf("ACK TX    : %lu\n", statAckTx);
  Serial.printf("ACK RX    : %lu\n", statAckRx);
  Serial.printf("Timeouts  : %lu\n", statTimeouts);
  Serial.printf("Retries   : %lu\n", statRetries);
  Serial.printf("Passive   : %lu\n", statPassiveUpdates);

  Serial.println(F("========================================"));
}

void printStatistics()
{
  Serial.println();
  Serial.println(F("---------- STATUS ----------"));

  Serial.printf("RX bytes   : %lu\n", statRxBytes);
  Serial.printf("Frames     : %lu\n", statFrames);
  Serial.printf("CRC OK     : %lu\n", statCrcOk);
  Serial.printf("CRC errors : %lu\n", statCrcError);
  Serial.printf("TX frames  : %lu\n", statTxFrames);
  Serial.printf("ACK TX     : %lu\n", statAckTx);
  Serial.printf("ACK RX     : %lu\n", statAckRx);
  Serial.printf("Timeouts   : %lu\n", statTimeouts);
  Serial.printf("Retries    : %lu\n", statRetries);
  Serial.printf("Passive    : %lu\n", statPassiveUpdates);

  Serial.printf(
    "Pending    : %s\n",
    pending.active ? "YES" : "NO"
  );

  Serial.printf(
    "Bus idle   : %s\n",
    busIsIdle() ? "YES" : "NO"
  );

  Serial.printf(
    "Scan       : %s\n",
    scanActive ? "ACTIVE" : "IDLE"
  );

  Serial.printf(
    "Status poll: %s\n",
    statusPollActive ? "ACTIVE" : "IDLE"
  );

  Serial.printf(
    "WiFi       : %s\n",
    WiFi.status() == WL_CONNECTED
      ? "CONNECTED"
      : "DISCONNECTED"
  );

  Serial.printf(
    "MQTT       : %s\n",
    mqttClient.connected()
      ? "CONNECTED"
      : "DISCONNECTED"
  );

  Serial.println(F("----------------------------"));
}

void handleSerial()
{
  if (!Serial.available())
    return;

  char c = Serial.read();

  switch (c)
  {
    case 'a':
    case 'A':
      startScan();
      break;

    case 'p':
    case 'P':
      printDeviceOverview();
      break;

    case 's':
    case 'S':
      printStatistics();
      break;

    case 'm':
    case 'M':
      mqttPublishGatewayDiscovery();
      mqttPublishGatewayDiagnostics();
      mqttPublishDiscoveryAll();
      mqttSubscribeOutputCommands();
      mqttPublishAllStates();
      Serial.println(F("[MQTT] Gateway + discovery + states republished"));
      break;

    case 'r':
    case 'R':
      startStatusPoll();
      break;
  }
}

// ============================================================
// Setup / loop
// ============================================================

void setup()
{
  pinMode(
    HM485_DIR_PIN,
    OUTPUT
  );

  digitalWrite(
    HM485_DIR_PIN,
    LOW
  );

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println(F("========================================"));
  Serial.print(F(" HM485 Gateway v"));
  Serial.println(FW_VERSION);
  Serial.println(F(" WT32 Ethernet + WiFi fallback + Web OTA"));
  Serial.println(F(" HM485 READ/WRITE + SAFE I/O EEPROM CONFIG"));
  Serial.println(F("========================================"));

  loadConfig();
  restoreKnownDevicesFromNvs();

  webLogAdd(
    String("[BOOT] HM485 Gateway v") +
    FW_VERSION
  );

  // MQTT Discovery JSON can be >256 bytes.
  mqttClient.setBufferSize(1536);
  mqttClient.setKeepAlive(30);
  mqttClient.setCallback(mqttMessageCallback);

  HM485.begin(
    HM485_BAUD,
    SERIAL_8E1,
    HM485_RX_PIN,
    HM485_TX_PIN
  );

  setReceiveMode();
  lastBusActivityUs = micros();

  Serial.printf("RX  GPIO%d\n", HM485_RX_PIN);
  Serial.printf("TX  GPIO%d\n", HM485_TX_PIN);
  Serial.printf("DIR GPIO%d\n", HM485_DIR_PIN);

  Serial.println();

  Serial.print(F("Gateway HM485 address : "));
  printAddress(config.localAddress);
  Serial.println();
  if (config.localAddress != FHEM_ADDRESS)
  {
    Serial.print(F("Reserved legacy central: "));
    printAddress(FHEM_ADDRESS);
    Serial.println();
  }

  Serial.println();
  Serial.println(F("HM485 v0.9.0: digital outputs + verified HMW-IO-12-Sw14-DR I/O EEPROM configuration."));
  Serial.println(F("EEPROM writes are allow-listed, read/modify/write and read-back verified; shutters remain write-locked."));

  startEthernet();
  setupWebServer();

  Serial.println(F("[NET] Ethernet primary, WiFi fallback after 8 seconds."));

  Serial.println();
  Serial.println(F("Console commands:"));
  Serial.println(F("  a = full HM485 scan"));
  Serial.println(F("  p = print device table"));
  Serial.println(F("  s = statistics"));
  Serial.println(F("  r = status-only refresh"));
  Serial.println(F("  m = republish MQTT discovery + states"));
  Serial.println();

  // v0.7.49: before the first active HM485 transmission, stay receive-only
  // for a short passive collision check. Native discovery starts automatically
  // afterwards if no duplicate of our configured source address was observed.
  addressGuardBootActive = true;
  addressGuardStartedMs = millis();
  setReceiveMode();
  Serial.println(F("[SAFETY] Passive HM485 address check for 8 seconds; TX disabled."));
  webLogAdd("[SAFETY] Boot address guard active; HM485 TX disabled for 8 seconds");
}

void loop()
{
  if (rawRxOnlyMode)
  {
    // Hard diagnostic bypass: no parser and no protocol-layer activity.
    digitalWrite(HM485_DIR_PIN, LOW);
    while (HM485.available())
      rawRxOnlyFeed((uint8_t)HM485.read());
    if (rawRxOnlyLine.length() &&
        (uint32_t)(micros() - rawRxOnlyLastByteUs) >= 4000)
      rawRxOnlyFlush();
    processWifi();
    processNetwork();
    webServer.handleClient();
    digitalWrite(HM485_DIR_PIN, LOW);
    return;
  }

  while (HM485.available())
    parser.feed(HM485.read());

  processAddressGuard();
  checkRequestTimeout();
  processIoConfiguration();
  processOutputWrite();
  processButtonPulse();
  processActiveDiscovery();
  processPassiveDiscovery();
  processPassiveIdentityStatusPoll();
  processPassiveCaptureArm();

  if (passiveCaptureFull)
  {
    passiveCaptureFull = false;
    // Re-enter stop path only to format/dump the already complete buffer.
    passiveCaptureActive = true;
    Serial.println(F("[PASSIVE CAPTURE] FULL - capture stopped, dumping complete buffer"));
    activeDiscoveryStop();
  }

  processScan();
  processStatusPoll();
  processBootKnownDeviceVerification();

  processWifi();
  processNetwork();
  processMqtt();

  webServer.handleClient();

  handleSerial();

  // Periodic status-only refresh.
  if (
    !scanActive &&
    !statusPollActive &&
    !pending.active &&
    !outputWrite.active &&
    !ioConfig.active &&
    !buttonPulse.active &&
    !quietRootRequested &&
    !passiveCaptureActive &&
    !passiveCaptureArmRequested &&
    millis() -
      lastStatusRefreshCompletedMs >=
      STATUS_REFRESH_MS
  )
  {
    startStatusPoll();
  }
}
