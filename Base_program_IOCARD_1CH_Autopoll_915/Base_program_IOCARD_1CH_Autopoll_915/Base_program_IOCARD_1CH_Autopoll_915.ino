#include <RadioLib.h>
#include <SPI.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Forward declarations for Arduino auto-prototype issue
// -----------------------------------------------------------------------------
struct PollSlotConfig;
struct PersistedConfig;
struct LegacyPersistedConfig;
struct A6SequenceConfig;

enum ValidCommandKind {
  VALID_CMD_NONE, VALID_CMD_A1, VALID_CMD_A2, VALID_CMD_A3,
  VALID_CMD_A4, VALID_CMD_A5, VALID_CMD_A5_EMERGENCY, VALID_CMD_D, VALID_CMD_A6
};


const __FlashStringHelper* functionName(uint8_t functionCode);
bool dispatchModbusWriteNoAck(uint8_t slaveId, uint8_t functionCode,
                              uint16_t regAddr, uint16_t argument,
                              const uint16_t* values = NULL, uint8_t count = 0);
ValidCommandKind validateIncomingCommand(const uint8_t* data, size_t len);
void cancelA3ForValidatedCommand();
bool handleA5DisableAll();
bool executePollSlot(const PollSlotConfig& slot, uint8_t* responseData, uint8_t& responseLen);
bool executeOneShotWrite(uint8_t channel, uint8_t slaveId, uint8_t functionCode, uint16_t regAddr, uint16_t argument);
bool executeWriteMultipleRegisters(uint8_t channel, uint8_t slaveId, uint16_t startAddr, const uint16_t* values, uint8_t regCount);
void handleCommandA1(const uint8_t* data, size_t len);
void handleCommandA2(const uint8_t* data, size_t len);
void handleCommandA3(const uint8_t* data, size_t len);
void handleCommandA4(const uint8_t* data, size_t len);
void handleCommandA5(const uint8_t* data, size_t len);
void handleCommandA6(const uint8_t* data, size_t len);
bool validateA6Command(const uint8_t* data, size_t len);
void loadA6ConfigFromEEPROM();
void printA6ConfigAtBoot();
void powerOffA6AtStartup();
bool isA6SequenceBusy();
bool startA6Sequence(uint16_t rawValue); // Called by validated A2/A4 matching the A6 target.
bool a6MatchesOutput(uint8_t slave, uint8_t fc, uint16_t address);
bool a6IsProtectedOutputAddress(uint8_t slave, uint16_t address);
bool a6GetProtectedOutput(uint8_t& slave, uint16_t& address);
bool a6RawSetpointValid(uint16_t raw);
void a6QueueSetpoint(uint16_t raw);
void handleA6SequenceTask();
void a6ProcessDeferredCommand();
uint8_t expectedUnifiedSlotDataLen(const PollSlotConfig& slot);
bool validateA3SelectedSlots(uint8_t slotA, uint8_t slotB);
uint8_t countA3SelectedSlots(uint8_t slotA, uint8_t slotB);
uint16_t getA3MaxSamplesPerBatch();
uint16_t expectedA3RawSampleLen();
size_t getA3FixedBatchLen();
void handleA3LogTask();
bool isA3LogBusy();
bool sendLoRaWanUplink(const uint8_t* payload, size_t len, const __FlashStringHelper* label);
bool sendLoRaWanUplinkConfirmed(const uint8_t* payload, size_t len, const __FlashStringHelper* label);
void setCurrentCommandEchoSource(const uint8_t* data, size_t len);
bool sendCommandStatusEcho(uint8_t statusByte, const uint8_t* data, size_t len, const __FlashStringHelper* label);
bool sendCommandAppliedAck();
bool sendInvalidCommandMarker();
void buildA3CommandPayload(uint8_t* payload);
bool sendA3CycleStatusEcho(const __FlashStringHelper* label);
bool sendA3NoConfiguredSlotMarker();
bool sendA3ModbusBreakMarker();
bool buildSlotConfigPayloadPage(uint8_t firstSlot, uint8_t slotCount, uint8_t* payload, size_t payloadSize, size_t& payloadLen);
bool sendSlotConfigUplink(const __FlashStringHelper* label);
bool legacyConfigLooksValid(const struct LegacyPersistedConfig& cfg);
bool migrateLegacyConfigIfNeeded();
void migrateLegacyNonceStorageIfNeeded();
bool sendStartupSlotConfigUplink();
void printHexByte(uint8_t value);
void printDevEUIAtBoot();
void sendStartupHeartbeat();
void printDownlinkParsed(const uint8_t* data, size_t len);
void processDownlinkCommand(const uint8_t* data, size_t len);
void applyLoRaWANRuntimeSettings();
bool waitForUplinkAvailability();
bool isAckReceivedFromSendReceive(int16_t state, uint32_t elapsedMs, const __FlashStringHelper* label);
uint16_t crc16Update(uint16_t crc, uint8_t data);
uint16_t readU16LE(int address);
uint32_t readU32LE(int address);
void eepromUpdateU16LE(int address, uint16_t value);
bool readNonceSlot(int baseAddress, uint8_t* noncesOut, uint32_t* sequenceOut);
void writeNonceSlot(int baseAddress, uint32_t sequence, const uint8_t* nonces);
bool loadRadioLibNoncesFromEEPROM();
void saveRadioLibNoncesToEEPROM();
uint16_t readU16LEFromBuffer(const uint8_t* buffer, uint16_t offset);
void writeU16LEToBuffer(uint8_t* buffer, uint16_t offset, uint16_t value);
bool advanceDevNonceInRAM(uint16_t jumpAmount);
void beginOTAAAndLoadNonces();
void joinNetworkInstant();
bool sendSpecialHeartbeatConfirmed();
bool sendStartupStartedUplink();
bool resetRadioAndTryOneHeartbeat();
void setupClassCAndVerifyServer();
void fullJoinAndClassCStart();
void handleMissedHeartbeat();
void handleClassCDownlink();
void markLoRaWANTransmissionDone();
uint32_t getLoRaWANCooldownRemainingMs();

// -----------------------------------------------------------------------------
// IOCARD / Mega pin map
// -----------------------------------------------------------------------------
#define STATUS_LED       28
#define RS485_DE_RE_PIN  5
#define MODBUS_BAUD      9600

#define SOFT_UART_RX     62
#define SOFT_UART_TX     63
#define DEBUG_BAUD       9600

#define MODBUS_SERIAL    Serial

SoftwareSerial Debug(SOFT_UART_RX, SOFT_UART_TX);
#define DEBUG_SERIAL     Debug

RFM95 radio = new Module(10, 2, 9, 6);

// US915 region
const LoRaWANBand_t Region = US915;
const uint8_t subBand = 0;

// OTAA credentials
uint64_t joinEUI = 0x0000000000000000ULL;
uint64_t devEUI  = 0x0000000000000130ULL;

uint8_t appKey[] = {
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x01, 0x30
};

// -----------------------------------------------------------------------------
// LoRaWAN
// -----------------------------------------------------------------------------
LoRaWANNode node(&radio, &Region, subBand);

// -----------------------------------------------------------------------------
// LoRaWAN connection manager / recovery settings
// -----------------------------------------------------------------------------
#define LORAWAN_APP_PORT 1
#define LORAWAN_HEARTBEAT_PORT 2
#define LORA_RX_BUFFER_SIZE 64

// US915 DR3 gives the largest common 125 kHz uplink payload size for this region.
// Keep the region as US915. Do not use the EU868 DR value from the reference sketch.
#define US915_FIXED_DR 3
#define US915_RX2_DR 8

#define HEARTBEAT_INTERVAL_MS 3600000UL       // 60 minutes without successful uplink
#define HEARTBEAT_ACK_WAIT_LIMIT_MS 5000UL    // 5 seconds max ACK/uplink wait
#define ACK_FAIL_LIMIT 3

#define ENABLE_MCU_REBOOT_AFTER_REJOIN_FAILS 1
#define MAX_REJOIN_FAILURES_BEFORE_MCU_REBOOT 300

#define DEVNONCE_JUMP_EVERY_RETRIES 5
#define DEVNONCE_JUMP_AMOUNT 100

// Base config starts at EEPROM address 100.
// With 20 poll slots the config block is larger than the old 12-slot block.
// Nonce records are moved to 512+ so the 20-slot config cannot overwrite them.
#define EEPROM_NONCE_LEGACY_SLOT_A 256
#define EEPROM_NONCE_RECORD_HEADER_SIZE 8
#define EEPROM_NONCE_RECORD_CRC_SIZE 2
#define EEPROM_NONCE_RECORD_SIZE (EEPROM_NONCE_RECORD_HEADER_SIZE + RADIOLIB_LORAWAN_NONCES_BUF_SIZE + EEPROM_NONCE_RECORD_CRC_SIZE)
#define EEPROM_NONCE_LEGACY_SLOT_B (EEPROM_NONCE_LEGACY_SLOT_A + EEPROM_NONCE_RECORD_SIZE)
#define EEPROM_NONCE_SLOT_A 512
#define EEPROM_NONCE_SLOT_B (EEPROM_NONCE_SLOT_A + EEPROM_NONCE_RECORD_SIZE)

#define NONCE_MAGIC_0 0x4E   // 'N'
#define NONCE_MAGIC_1 0x43   // 'C'
#define NONCE_RECORD_VERSION 1

// -----------------------------------------------------------------------------
// Modbus
// -----------------------------------------------------------------------------
ModbusMaster mb;

// -----------------------------------------------------------------------------
// EEPROM config
// -----------------------------------------------------------------------------
const uint16_t EEPROM_MAGIC   = 0xA15C;
const uint8_t  EEPROM_VERSION = 0x06;  // v06 = 20 polling slots
const uint8_t  EEPROM_LEGACY_VERSION_12_SLOTS = 0x05;
const int EEPROM_CONFIG_ADDR  = 100;

const uint16_t DEFAULT_UPLOAD_INTERVAL_SECONDS = 600;
const uint8_t  LEGACY_MAX_POLL_SLOTS = 12;
const uint8_t  MAX_POLL_SLOTS = 20;  // 20 total polling slots
const uint16_t MAX_REGS_PER_SLOT = 20;
const uint8_t  MAX_FC16_REGS_PER_COMMAND = 20;
const uint8_t  FIXED_CHANNEL = 1;

// A5 is an optional setting on existing A2 read slots, not another poller.
// 20 x 5 bytes (slave, address, seconds) + 3 header + 2 CRC = 105 EEPROM bytes.
// Slave 0 means disabled. Channel 1 is fixed and needs no stored byte.
// Use spare EEPROM after the legacy A5 record to allow bytewise migration.
const int A5_EEPROM_BASE = 1700;
const uint16_t A5_EEPROM_MAGIC = 0xA5C1;
const uint8_t A5_EEPROM_VERSION = 2;
const uint8_t A5_EEPROM_HEADER_BYTES = 3;
const uint8_t A5_EEPROM_SLOT_BYTES = 5;
const uint16_t A5_MIN_STABILIZATION_SECONDS = 15;
const uint8_t A5_OFF_TRIES = 3;
const uint16_t A5_POLL_INTERVAL_MS = 1000;  // One sweep per second, best effort. Only A5-powered slots.
const uint16_t A5_POLL_STOP_EARLY_SECONDS = 5;  // Poll duration = command T - 5 seconds; never an extra 5 seconds.
const uint16_t A5_OFF_RETRY_MS = 200;
const int A5_EEPROM_CRC_ADDR = A5_EEPROM_BASE + A5_EEPROM_HEADER_BYTES + MAX_POLL_SLOTS * A5_EEPROM_SLOT_BYTES;
// One independent EEPROM byte after the A5 record. A failed local OFF dispatch
// latches an automatic-ON interlock; A5FFFF clears it after OFF frames transmit.
const int A5_SAFETY_LOCK_ADDR = A5_EEPROM_CRC_ADDR + 2;
const uint8_t A5_SAFETY_LOCK_MAGIC = 0x5A;
const int A5_LEGACY_EEPROM_BASE = 1536;  // Older A5 version 1: 145 bytes.

// A6 uses a separate EEPROM record. All bytes are stored in wire order (big-endian
// for uint16), independent of C++ structure packing and existing A1-A5 records.
// A5 safety byte ends at 1805; A6 begins at 1856. Mega EEPROM: 4096 bytes.
const int A6_EEPROM_BASE = 1856;
const uint16_t A6_EEPROM_MAGIC = 0xA6C1;
const uint8_t A6_EEPROM_VERSION = 1;
const uint8_t A6_COMMAND_LEN = 27;   // 000000A6 + 23 configuration bytes.
const uint8_t A6_DATA_BYTES = A6_COMMAND_LEN - 4;
const int A6_EEPROM_DATA_ADDR = A6_EEPROM_BASE + 3;
const int A6_EEPROM_CRC_ADDR = A6_EEPROM_DATA_ADDR + A6_DATA_BYTES;
const int A6_SAFETY_LOCK_ADDR = A6_EEPROM_CRC_ADDR + 2;
const uint8_t A6_SAFETY_LOCK_MAGIC = 0x6A;
// When an actuator OFF cannot be verified, retain the ORIGINAL physical key.
// A subsequent A6 reconfiguration must not clear the old relay's inhibit.
const int A6_RECOVERY_SLAVE_ADDR = A6_SAFETY_LOCK_ADDR + 1;
const int A6_RECOVERY_COIL_ADDR = A6_SAFETY_LOCK_ADDR + 2; // two little-endian bytes
// Persist ONLY when an A6 actuator ON is about to be transmitted. A firmware
// restart when no A6 actuator has run must not send commands to an unpowered
// device based merely on an EEPROM setup entry.
const int A6_ACTUATOR_MAY_BE_ON_ADDR = A6_RECOVERY_COIL_ADDR + 2;
const uint8_t A6_ACTUATOR_MAY_BE_ON_MAGIC = 0xA7;

// A6 v2 is entirely separate from the v1 EEPROM configuration and its safety
// bytes. The original 1856..1888 record is never moved or overwritten.
// The v2 payload appends power relay #2 [slave, coilBE] to legacy A6 fields.
const int A6_V2_EEPROM_BASE = 1904;
const uint8_t A6_V2_EEPROM_VERSION = 2;
const uint8_t A6_V2_COMMAND_LEN = 30;  // 000000A6 + 26 config bytes.
const uint8_t A6_V2_DATA_BYTES = A6_V2_COMMAND_LEN - 4;
const int A6_V2_DATA_ADDR = A6_V2_EEPROM_BASE + 3;
const int A6_V2_CRC_ADDR = A6_V2_DATA_ADDR + A6_V2_DATA_BYTES;

// Record the ORIGINAL four physical relay keys BEFORE sending any ON command.
// On reset, the original physical keys can all be turned OFF even if setup
// changed. Active marker is committed last, after the record CRC is valid.
const int A6_JOURNAL_BASE = 1952;
const uint16_t A6_JOURNAL_MAGIC = 0xA6E2;
const uint8_t A6_JOURNAL_VERSION = 1;
const int A6_JOURNAL_KEYS_ADDR = A6_JOURNAL_BASE + 3;
const int A6_JOURNAL_CRC_ADDR = A6_JOURNAL_KEYS_ADDR + 12;
const int A6_JOURNAL_ACTIVE_ADDR = A6_JOURNAL_CRC_ADDR + 2;
const int A6_JOURNAL_LOCK_ADDR = A6_JOURNAL_ACTIVE_ADDR + 1;
const uint8_t A6_JOURNAL_ACTIVE_MAGIC = 0xA8;
const uint8_t A6_JOURNAL_LOCK_MAGIC = 0xA9;
const uint32_t A6_RELAY_OFF_INTERVAL_MS = 3000UL;
// Repeat every relay ON three times without inspecting communication responses.
const uint32_t RELAY_ON_INTERVAL_MS = 3000UL;
const uint32_t A6_RELAY_ON_INTERVAL_MS = RELAY_ON_INTERVAL_MS;

// Slot format:
// enabled  = 0 or 1
// channel  = Modbus channel (1..4)
// slaveId  = Modbus slave
// function = Modbus function code
// regAddress = Modbus start address
// argument   = quantity for FC01/02/03/04, value for FC05/06
struct __attribute__((packed)) PollSlotConfig {
  uint8_t  enabled;
  uint8_t  channel;
  uint8_t  slaveId;
  uint8_t  function;
  uint16_t regAddress;
  uint16_t argument;
};

struct __attribute__((packed)) PersistedConfig {
  uint16_t magic;
  uint8_t  version;
  uint16_t uploadIntervalSeconds;
  PollSlotConfig slots[MAX_POLL_SLOTS];
};

struct __attribute__((packed)) LegacyPersistedConfig {
  uint16_t magic;
  uint8_t  version;
  uint16_t uploadIntervalSeconds;
  PollSlotConfig slots[LEGACY_MAX_POLL_SLOTS];
};

// A6: exactly the address and timing parameters in the setup command.
// No percentage/setpoint request or start/stop instruction is stored here.
struct __attribute__((packed)) A6SequenceConfig {
  uint8_t aoPowerSlave;
  uint16_t aoPowerCoil;
  uint8_t aiPowerSlave;
  uint16_t aiPowerCoil;
  uint8_t aoSetpointSlave;
  uint8_t aoSetpointFunction;  // FC06 or FC16 (one 16-bit register).
  uint16_t aoSetpointRegister;
  uint8_t actuatorSlave;
  uint16_t actuatorCoil;
  uint16_t rawMinimum;
  uint16_t rawMaximum;
  uint16_t t1Seconds;
  uint16_t t2Seconds;
  uint16_t t3Seconds;
  uint8_t extraPowerSlave;
  uint16_t extraPowerCoil;
};
static_assert(sizeof(A6SequenceConfig) == 26, "Unexpected A6 v2 configuration layout");

PersistedConfig gConfig;
// A5 safety state: no second Modbus registry or per-slot RAM data cache.
bool a5PowerSequenceActive = false;
bool a5PollCancelled = false;       // Only A5FFFF emergency cancels active A5; other downlinks deferred.
bool a5RegularPollInProgress = false; // Covers between coil groups and the confirmed retry delay.
bool a5InterruptionOffSent = false; // Prevent duplicate OFF after a valid command interrupts active polling.
bool a5ActiveRelayMayBeOn = false; // Conservative tracking while the active relay is being operated.
uint8_t a5ActiveRelaySlave = 0;
uint16_t a5ActiveRelayAddr = 0;
bool a5PowerLockout = false;        // Existing EEPROM marker retained for diagnostics; never gates relay ON.

// A6 remains CONFIGURATION-ONLY. A matching validated A2 FC06 (or A4 FC16
// single-register write) supplies its own raw value and triggers the sequence.
// The request is queued until any interrupted A5 relay cleanup has completed;
// never transmit an analog setpoint directly to an unpowered module.
enum A6SequenceState {
  A6_IDLE = 0, A6_POWER_ON, A6_WAIT_T1, A6_WAIT_T2,
  A6_ACTUATOR_ON, A6_WAIT_T3, A6_READING_SLOTS,
  A6_SENDING_SLOTS, A6_SHUTTING_DOWN
};
A6SequenceConfig a6Config;
A6SequenceConfig a6RunningConfig; // Preserve original actuator mapping while stopping.
bool a6Configured = false;
bool a6SafetyLockout = false;    // Persisted if actuator OFF cannot be verified.
uint8_t a6RecoverySlave = 0;     // Recovery mapping survives reconfiguration/reboot.
uint16_t a6RecoveryCoil = 0;
bool a6ActuatorMayBeOn = false;
A6SequenceState a6State = A6_IDLE;
uint16_t a6RequestedRaw = 0; // Exact incoming A2/A4 raw value; NEVER hardcoded.
uint16_t a6PendingRaw = 0;
bool a6PendingStart = false;
uint32_t a6StageStartedMs = 0;
uint8_t a6OnStep = 0;            // Current step in the 9-power / 3-actuator ON phases.
uint8_t a6NextPollSlot = 0;
uint8_t a6ShutdownStep = 0;      // 0..7: two reverse-order OFF passes.
uint8_t a6PageCount = 0;
size_t a6PageLen = 0;
// A6 never stops for a new downlink. Keep a small bounded FIFO and replay
// the original commands after all eight OFF frames have been issued.
const uint8_t A6_DEFERRED_COMMAND_CAPACITY = 4;
uint8_t a6DeferredCommands[A6_DEFERRED_COMMAND_CAPACITY][LORA_RX_BUFFER_SIZE];
uint8_t a6DeferredLengths[A6_DEFERRED_COMMAND_CAPACITY] = {0};
uint8_t a6DeferredHead = 0;
uint8_t a6DeferredCount = 0;

// -----------------------------------------------------------------------------
// Runtime config
// -----------------------------------------------------------------------------
uint16_t uploadIntervalSeconds = DEFAULT_UPLOAD_INTERVAL_SECONDS;
uint32_t lastUplink = 0;

const uint16_t timeBetweenPollsMs = 50;  // regular timed poll gap

// A3 fast capture timing. Set to 0 to sample selected sensors back-to-back.
// The remaining delay is only the real Modbus transaction/response time.
const uint16_t A3_SENSOR_TO_SENSOR_DELAY_MS = 0;
const uint16_t A3_SAMPLE_TO_SAMPLE_DELAY_MS = 0;
const uint16_t A3_MODBUS_RETRY_DELAY_MS = 0;

// Unified regular uplink format:
// [idHi][idLo] + repeated [slotIndex][function][dataLen][data...]
// + [pageHi][pageLo].
//
// Regular timed poll uses ID 0000 and page 0000.
// Regular timed poll is variable length now; it is NOT padded to 108 bytes.
// FF is used only inside a slot data section when that Modbus slot has no response.
const size_t MAX_UPLINK_PAYLOAD_SIZE = 108;
// Only A6 reuses this 108-byte buffer; A5/regular-poll buffers are untouched.
uint8_t a6PageBuffer[MAX_UPLINK_PAYLOAD_SIZE];

// -----------------------------------------------------------------------------
// A3 raw-data capture log
// -----------------------------------------------------------------------------
// A3 downlink format:
//   000000A3 + [recordIdHi][recordIdLo] + [slotA][slotB] + [captureRepeatCount]
//
// Example:
//   000000A3 ABCD 0001 03 -> Record ID ABCD, capture slot 0 and slot 1, repeat the full capture/send cycle 3 times.
//   000000A3 ABCD 00FF 01 -> Record ID ABCD, capture slot 0 only, run the capture/send cycle 1 time.
//
// A3 is meant for 1 or 2 selected variables/slaves only.
// Slot value FF means unused.
//
// A3 has NO sampling time.
// It captures samples first and stores ONLY raw Modbus response data in SRAM.
// It does NOT save record ID, slot number, function code, data length, or page
// number in the A3 SRAM pool. Those fields are added only when transmitting.
//
// A3 captures until the raw SRAM pool becomes full.
//
// Regular timed poll uses:
//   [idHi][idLo]
//   [slotIndex][function][dataLen][data...] repeated for each enabled slot
//   [pageHi][pageLo]
//
// A3 transmitted batch stays below 220 HEX characters.
// Each confirmed fixed-section batch is sent one-by-one in order and contains ONLY:
//   [slotA][slotB]
//   [2 bytes most-repeated Slot A response time ms]
//   [Slot A full-page data section; only the last short page is padded with FFFE]
//   [2 bytes most-repeated Slot B response time ms, only when Slot B is used]
//   [Slot B full-page data section; only the last short page is padded with FFFE, only when Slot B is used]
//   [totalTxPages][currentTxPage]
//
// Each A3 capture/page set is wrapped with 00 42 followed by the full original command: one echo before capture starts, and one echo after that set of pages finishes.
// The A3 data batches ignore the record ID and use Slot A + Slot B as the header.
// No A3 command header, no register number, no function code, no data length,
// no sample count and no final summary are added to A3 data batches.
// A3 full pages are sent without filler. Only the final short page is padded with FFFE to match the full-page length.
//
// Each slot response time is stored as uint16 milliseconds. It is measured from
// the start of that slot Modbus request to the end of that slot Modbus response.
//
// A3 SRAM storage uses only:
//   [slot A response time ms][raw data slot A]
//   [slot B response time ms][raw data slot B] repeated per sample
const uint8_t  A3_SELECTED_SLOT_UNUSED = 0xFF;
const uint8_t  A3_MAX_SELECTED_SLOTS = 2;

// Arduino Mega SRAM safety:
// Previous design reserved 1536 bytes for stored pages + 300 bytes for
// page offset/length tables. This optimized design reuses that same budget
// as one raw data pool, removing the fixed 100-page table limit.
const uint16_t A3_RAW_DATA_POOL_SIZE = 1836;

// A3 maximum batch format:
// 2 bytes selected slots: [slotA][slotB]
// + 2 bytes most-repeated Slot A response time
// + Slot A full-page data section; only the final short page is padded with FFFE when needed
// + 2 bytes most-repeated Slot B response time, only when Slot B is used
// + Slot B full-page data section; only the final short page is padded with FFFE when needed, only when Slot B is used
// + 2 bytes page footer: [totalTxPages][currentTxPage]
const uint8_t A3_BATCH_SECTION_SIZE = 50;
const uint8_t A3_RESPONSE_TIME_BYTES = 2;
const uint8_t A3_TX_PAGE_MAX_SIZE = 108;

// Delay between A3 stored batch uploads.
// First data batch sends immediately after capture finishes.
// Every next data batch waits at least 15 seconds after the previous upload.
// The last byte of the A3 command is NOT a final echo repeat. It is the number
// of full A3 capture/send cycles to run one after the other.
const uint32_t A3_PAGE_UPLOAD_DELAY_MS = 15000UL;

// If A3 breaks because Modbus did not answer, send an error echo with
// the original A3 command, then abort without sending stored pages.
// 00 49 = no complete sample was captured.
// 00 4A = capture had already stored at least one complete sample, then broke.
// If LoRaWAN is busy, retry the error echo.
const uint32_t A3_BREAK_MARKER_RETRY_DELAY_MS = 1000UL;

// A3 data batches are sent as confirmed LoRaWAN uplinks.
// If one batch is not confirmed, retry the same page only 2 times.
// There is always a 15 second gap between the original attempt and each retry.
const uint8_t A3_CONFIRMED_UPLOAD_MAX_RETRIES = 2;

// During A3 capture, each selected Modbus device/slot is tried 2 times.
// If slot A or slot B still does not answer correctly, A3 capture stops.
const uint8_t A3_MODBUS_READ_MAX_TRIES = 2;

const uint8_t  A3_FINAL_OK = 0x00;
const uint8_t  A3_FINAL_MAX_PAGES_REACHED = 0x01;
const uint8_t  A3_FINAL_NO_ACTIVE_SLOTS = 0x02;
const uint8_t  A3_FINAL_PAGE_TRUNCATED = 0x03;
const uint8_t  A3_FINAL_UPLOAD_FAILED = 0x04;
const uint8_t  A3_FINAL_MODBUS_NO_RESPONSE = 0x05;
const uint8_t  A3_FINAL_MODBUS_BREAK_MID_CAPTURE = 0x06;
// Startup payload: ASCII "A" = 41. Sent once only after Class C starts.
// Command accepted prefix: 00 42. Sent before the echoed command when a command is accepted/applied.
// Scheduled heartbeat payload: ASCII "C" = 43.
// Slot config request/response header: ASCII "D" = 44.
// Each D/44 status uplink is fixed to 100 bytes = 200 HEX characters when the
// 20-slot status is split into multiple D/44 messages.
// The final D/44 status message is completed with 00 padding so all D/44 pages
// have the same payload size.
// Invalid command prefix: 00 47. Sent before the echoed command when a received command is invalid/not applied.
// A3 no configured/valid slot payload: ASCII "H" = 48. Sent when A3 cannot start because no selected slot is configured/valid.
// A3 Modbus no-response prefix: 00 49. Sent with original A3 command echo when A3 stops before any complete sample is stored.
// A3 mid-capture break prefix: 00 4A. Sent with original A3 command echo when A3 already stored at least one complete sample and then a selected Modbus device stopped answering.
const uint8_t STARTUP_HEARTBEAT_PAYLOAD[] = { 'A' };
const uint8_t COMMAND_APPLIED_PAYLOAD[] = { 'B' };
const uint8_t HEARTBEAT_PAYLOAD[] = { 'C' };
const uint8_t SLOT_CONFIG_PAYLOAD_ID = 'D';
const size_t  SLOT_CONFIG_STATUS_MAX_BYTES = 100;  // 200 HEX characters
const uint8_t SLOT_CONFIG_HEADER_SIZE = 4;
const uint8_t SLOT_CONFIG_RECORD_SIZE = 8;
const uint8_t SLOT_CONFIG_SLOTS_PER_MESSAGE = (SLOT_CONFIG_STATUS_MAX_BYTES - SLOT_CONFIG_HEADER_SIZE) / SLOT_CONFIG_RECORD_SIZE;  // 12 slots per D/44 message
const uint8_t SLOT_CONFIG_MAX_MESSAGES = 3;
const uint8_t INVALID_COMMAND_PAYLOAD[] = { 'G' };
const uint8_t COMMAND_STATUS_PREFIX = 0x00;
const uint8_t A3_NO_CONFIGURED_SLOT_PAYLOAD[] = { 'H' };
const uint8_t COMMAND_STATUS_ACCEPTED = 'B';
const uint8_t COMMAND_STATUS_INVALID = 'G';
const uint8_t COMMAND_STATUS_A3_MODBUS_NO_RESPONSE = 'I';
const uint8_t COMMAND_STATUS_A3_CAPTURE_INTERRUPTED = 'J';
const uint8_t A3_COMMAND_PAYLOAD_LEN = 9;

uint8_t downlinkPayload[LORA_RX_BUFFER_SIZE];
size_t downlinkLen = 0;
LoRaWANEvent_t downlinkEvent;

const uint8_t* currentCommandEchoPayload = NULL;
uint8_t currentCommandEchoLen = 0;

bool joined = false;
bool connected = false;
bool classCReady = false;
bool rejoinRequested = false;

uint8_t ackFailCounter = 0;
uint16_t consecutiveRejoinFailures = 0;

uint32_t lastHeartbeatMs = 0;
uint32_t nonceSequence = 0;

bool startupHeartbeatSent = false;
bool startupConfigSent = false;
uint32_t lastLoRaWANTransmissionMs = 0;
bool loRaWANCooldownHasReference = false;

void (*softwareReset)(void) = 0;

enum A3LogState {
  A3_IDLE = 0,
  A3_SENDING_CYCLE_START_ECHO,
  A3_CAPTURING,
  A3_SENDING_BREAK_MARKER,
  A3_SENDING_PAGES,
  A3_SENDING_CYCLE_FINISH_ECHO,
  A3_SENDING_FINAL
};

A3LogState a3State = A3_IDLE;

uint16_t a3RecordId = 0;
uint8_t  a3OriginalCommandPayload[A3_COMMAND_PAYLOAD_LEN];
uint8_t  a3OriginalCommandLen = 0;
uint8_t  a3CaptureRepeatTotal = 1;
uint8_t  a3CaptureCycleCurrent = 0;
uint8_t  a3SelectedSlots[A3_MAX_SELECTED_SLOTS] = { A3_SELECTED_SLOT_UNUSED, A3_SELECTED_SLOT_UNUSED };
uint8_t  a3SelectedSlotCount = 0;
uint16_t a3TotalPages = 0;
uint16_t a3SendIndex = 0;
uint16_t a3SendPageNumber = 1;
uint8_t  a3CurrentBatchRetryCount = 0;
uint8_t  a3FinalStatus = A3_FINAL_OK;

uint8_t  a3RawDataPool[A3_RAW_DATA_POOL_SIZE];
uint16_t a3StorageUsed = 0;
uint16_t a3RawSampleLen = 0;
uint32_t a3LastPageUploadMillis = 0;
uint32_t a3BreakMarkerLastAttemptMillis = 0;

// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------
void printHexByte(uint8_t value) {
  if (value < 0x10) {
    DEBUG_SERIAL.print('0');
  }
  DEBUG_SERIAL.print(value, HEX);
}

void printDevEUIAtBoot() {
  DEBUG_SERIAL.print(F("Fw. Ver 1.6.2 A6Trig"));
  DEBUG_SERIAL.print(F("DevEUI: "));
  for (int8_t i = 7; i >= 0; i--) {
    uint8_t b = (uint8_t)((devEUI >> (8 * i)) & 0xFF);
    printHexByte(b);
  }
  DEBUG_SERIAL.println();
}

void blinkLed(uint8_t times, uint16_t onTime = 150, uint16_t offTime = 150) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(STATUS_LED, HIGH);
    delay(onTime);
    digitalWrite(STATUS_LED, LOW);
    delay(offTime);
  }
}

void blinkCommandReceived() {
  // During A3 capture, the activity LED remains solid ON (not a command priority).
  if (isA3LogBusy()) {
    digitalWrite(STATUS_LED, HIGH);
    return;
  }

  blinkLed(3, 60, 60);
}

void setA3LedActive() {
  digitalWrite(STATUS_LED, HIGH);
}

void setA3LedIdle() {
  digitalWrite(STATUS_LED, LOW);
}

void printStateDecode(Print& output, int16_t state) {
  switch (state) {
    case RADIOLIB_ERR_NONE:
      output.print(F("ERR_NONE"));
      break;
    case RADIOLIB_LORAWAN_NEW_SESSION:
      output.print(F("NEW_SESSION"));
      break;
    case RADIOLIB_LORAWAN_SESSION_RESTORED:
      output.print(F("SESSION_RESTORED"));
      break;
    case RADIOLIB_ERR_NETWORK_NOT_JOINED:
      output.print(F("NETWORK_NOT_JOINED"));
      break;
    case RADIOLIB_ERR_NO_JOIN_ACCEPT:
      output.print(F("NO_JOIN_ACCEPT"));
      break;
    case RADIOLIB_ERR_UPLINK_UNAVAILABLE:
      output.print(F("UPLINK_UNAVAILABLE"));
      break;
    case RADIOLIB_ERR_DOWNLINK_MALFORMED:
      output.print(F("DOWNLINK_MALFORMED"));
      break;
    default:
      output.print(state);
      break;
  }
}

void haltWithMessage(const char* msg, int16_t state) {
  DEBUG_SERIAL.print(msg);
  DEBUG_SERIAL.print(" : ");
  printStateDecode(DEBUG_SERIAL, state);
  DEBUG_SERIAL.println();

  while (true) {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(900);
  }
}

void dumpHex(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    if (data[i] < 0x10) {
      DEBUG_SERIAL.print('0');
    }
    DEBUG_SERIAL.print(data[i], HEX);
    DEBUG_SERIAL.print(' ');
  }
  DEBUG_SERIAL.println();
}


// -----------------------------------------------------------------------------
// LoRaWAN connection / nonce / heartbeat helpers
// -----------------------------------------------------------------------------
void applyLoRaWANRuntimeSettings() {
  node.setADR(false);
  node.setDatarate(US915_FIXED_DR);
  node.setRx2Dr(US915_RX2_DR);
  node.setDwellTime(false);
}

bool waitForUplinkAvailability() {
  uint32_t waitMs = node.timeUntilUplink();

  if (waitMs == 0) {
    return true;
  }

  DEBUG_SERIAL.print(F("Waiting uplink availability ms: "));
  DEBUG_SERIAL.println(waitMs);

  if (waitMs > HEARTBEAT_ACK_WAIT_LIMIT_MS) {
    DEBUG_SERIAL.println(F("Uplink wait is over 5 seconds. Skip now to avoid blocking."));
    return false;
  }

  uint32_t startMs = millis();
  while ((millis() - startMs) < (waitMs + 50UL)) {
    delay(10);
    yield();
  }

  return true;
}

bool isAckReceivedFromSendReceive(int16_t state, uint32_t elapsedMs, const __FlashStringHelper* label) {
  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(F(" ACK answer time ms: "));
  DEBUG_SERIAL.println(elapsedMs);

  if (elapsedMs > HEARTBEAT_ACK_WAIT_LIMIT_MS) {
    DEBUG_SERIAL.println(F("ACK wait exceeded 5 seconds. Treat as failed."));
    return false;
  }

  return (state > 0);
}

void markLoRaWANTransmissionDone() {
  lastLoRaWANTransmissionMs = millis();
  loRaWANCooldownHasReference = true;
  lastHeartbeatMs = lastLoRaWANTransmissionMs;
}

uint32_t getLoRaWANCooldownRemainingMs() {
  if (!loRaWANCooldownHasReference) {
    return 0;
  }

  uint32_t elapsedMs = millis() - lastLoRaWANTransmissionMs;
  uint32_t requiredGapMs = 1000UL;

  if (elapsedMs >= requiredGapMs) {
    return 0;
  }

  return requiredGapMs - elapsedMs;
}

// -----------------------------------------------------------------------------
// EEPROM CRC / nonce storage
// -----------------------------------------------------------------------------
uint16_t crc16Update(uint16_t crc, uint8_t data) {
  crc ^= data;

  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x0001) {
      crc >>= 1;
      crc ^= 0xA001;
    } else {
      crc >>= 1;
    }
  }

  return crc;
}

uint16_t readU16LE(int address) {
  uint16_t v = EEPROM.read(address);
  v |= ((uint16_t)EEPROM.read(address + 1) << 8);
  return v;
}

uint32_t readU32LE(int address) {
  uint32_t v = 0;
  v |= ((uint32_t)EEPROM.read(address));
  v |= ((uint32_t)EEPROM.read(address + 1) << 8);
  v |= ((uint32_t)EEPROM.read(address + 2) << 16);
  v |= ((uint32_t)EEPROM.read(address + 3) << 24);
  return v;
}

void eepromUpdateU16LE(int address, uint16_t value) {
  EEPROM.update(address, value & 0xFF);
  EEPROM.update(address + 1, (value >> 8) & 0xFF);
}

bool readNonceSlot(int baseAddress, uint8_t* noncesOut, uint32_t* sequenceOut) {
  if (EEPROM.read(baseAddress + 0) != NONCE_MAGIC_0) {
    return false;
  }

  if (EEPROM.read(baseAddress + 1) != NONCE_MAGIC_1) {
    return false;
  }

  if (EEPROM.read(baseAddress + 2) != NONCE_RECORD_VERSION) {
    return false;
  }

  if (EEPROM.read(baseAddress + 3) != RADIOLIB_LORAWAN_NONCES_BUF_SIZE) {
    return false;
  }

  uint16_t crc = 0xFFFF;

  for (uint16_t i = 0; i < EEPROM_NONCE_RECORD_HEADER_SIZE + RADIOLIB_LORAWAN_NONCES_BUF_SIZE; i++) {
    crc = crc16Update(crc, EEPROM.read(baseAddress + i));
  }

  uint16_t storedCrc = readU16LE(baseAddress + EEPROM_NONCE_RECORD_HEADER_SIZE + RADIOLIB_LORAWAN_NONCES_BUF_SIZE);

  if (crc != storedCrc) {
    return false;
  }

  *sequenceOut = readU32LE(baseAddress + 4);

  for (uint8_t i = 0; i < RADIOLIB_LORAWAN_NONCES_BUF_SIZE; i++) {
    noncesOut[i] = EEPROM.read(baseAddress + EEPROM_NONCE_RECORD_HEADER_SIZE + i);
  }

  return true;
}

void writeNonceSlot(int baseAddress, uint32_t sequence, const uint8_t* nonces) {
  uint8_t header[EEPROM_NONCE_RECORD_HEADER_SIZE];

  header[0] = NONCE_MAGIC_0;
  header[1] = NONCE_MAGIC_1;
  header[2] = NONCE_RECORD_VERSION;
  header[3] = RADIOLIB_LORAWAN_NONCES_BUF_SIZE;
  header[4] = sequence & 0xFF;
  header[5] = (sequence >> 8) & 0xFF;
  header[6] = (sequence >> 16) & 0xFF;
  header[7] = (sequence >> 24) & 0xFF;

  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < EEPROM_NONCE_RECORD_HEADER_SIZE; i++) {
    crc = crc16Update(crc, header[i]);
  }

  for (uint8_t i = 0; i < RADIOLIB_LORAWAN_NONCES_BUF_SIZE; i++) {
    crc = crc16Update(crc, nonces[i]);
  }

  for (uint8_t i = 0; i < EEPROM_NONCE_RECORD_HEADER_SIZE; i++) {
    EEPROM.update(baseAddress + i, header[i]);
  }

  for (uint8_t i = 0; i < RADIOLIB_LORAWAN_NONCES_BUF_SIZE; i++) {
    EEPROM.update(baseAddress + EEPROM_NONCE_RECORD_HEADER_SIZE + i, nonces[i]);
  }

  eepromUpdateU16LE(
    baseAddress + EEPROM_NONCE_RECORD_HEADER_SIZE + RADIOLIB_LORAWAN_NONCES_BUF_SIZE,
    crc);
}

void migrateLegacyNonceStorageIfNeeded() {
  uint8_t newNoncesA[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
  uint8_t newNoncesB[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
  uint32_t newSeqA = 0;
  uint32_t newSeqB = 0;

  bool newValidA = readNonceSlot(EEPROM_NONCE_SLOT_A, newNoncesA, &newSeqA);
  bool newValidB = readNonceSlot(EEPROM_NONCE_SLOT_B, newNoncesB, &newSeqB);

  if (newValidA || newValidB) {
    DEBUG_SERIAL.println(F("EEPROM nonces: new 20-slot storage already present."));
    return;
  }

  uint8_t oldNoncesA[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
  uint8_t oldNoncesB[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
  uint32_t oldSeqA = 0;
  uint32_t oldSeqB = 0;

  bool oldValidA = readNonceSlot(EEPROM_NONCE_LEGACY_SLOT_A, oldNoncesA, &oldSeqA);
  bool oldValidB = readNonceSlot(EEPROM_NONCE_LEGACY_SLOT_B, oldNoncesB, &oldSeqB);

  if (!oldValidA && !oldValidB) {
    DEBUG_SERIAL.println(F("EEPROM nonces: no legacy records to migrate."));
    return;
  }

  if (oldValidA) {
    writeNonceSlot(EEPROM_NONCE_SLOT_A, oldSeqA, oldNoncesA);
  }

  if (oldValidB) {
    writeNonceSlot(EEPROM_NONCE_SLOT_B, oldSeqB, oldNoncesB);
  }

  DEBUG_SERIAL.println(F("EEPROM nonces migrated to 20-slot-safe storage."));
}

bool loadRadioLibNoncesFromEEPROM() {
  uint8_t noncesA[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];
  uint8_t noncesB[RADIOLIB_LORAWAN_NONCES_BUF_SIZE];

  uint32_t seqA = 0;
  uint32_t seqB = 0;

  bool validA = readNonceSlot(EEPROM_NONCE_SLOT_A, noncesA, &seqA);
  bool validB = readNonceSlot(EEPROM_NONCE_SLOT_B, noncesB, &seqB);

  if (!validA && !validB) {
    DEBUG_SERIAL.println(F("EEPROM nonces: no valid record, using fresh RadioLib nonces"));
    nonceSequence = 0;
    return false;
  }

  uint8_t* selectedNonces;
  uint32_t selectedSeq;

  if (validA && (!validB || seqA >= seqB)) {
    selectedNonces = noncesA;
    selectedSeq = seqA;
    DEBUG_SERIAL.println(F("EEPROM nonces: using slot A"));
  } else {
    selectedNonces = noncesB;
    selectedSeq = seqB;
    DEBUG_SERIAL.println(F("EEPROM nonces: using slot B"));
  }

  nonceSequence = selectedSeq;

  int16_t state = node.setBufferNonces(selectedNonces);

  if (state == RADIOLIB_ERR_NONE) {
    DEBUG_SERIAL.print(F("EEPROM nonces loaded, sequence: "));
    DEBUG_SERIAL.println(nonceSequence);
    return true;
  }

  DEBUG_SERIAL.print(F("EEPROM nonces rejected by RadioLib: "));
  printStateDecode(DEBUG_SERIAL, state);
  DEBUG_SERIAL.println();
  return false;
}

void saveRadioLibNoncesToEEPROM() {
  uint8_t* nonces = node.getBufferNonces();

  nonceSequence++;

  int targetSlot = (nonceSequence & 0x01) ? EEPROM_NONCE_SLOT_A : EEPROM_NONCE_SLOT_B;

  writeNonceSlot(targetSlot, nonceSequence, nonces);

  DEBUG_SERIAL.print(F("EEPROM nonces saved, sequence: "));
  DEBUG_SERIAL.println(nonceSequence);
}

uint16_t readU16LEFromBuffer(const uint8_t* buffer, uint16_t offset) {
  uint16_t value = buffer[offset];
  value |= ((uint16_t)buffer[offset + 1] << 8);
  return value;
}

void writeU16LEToBuffer(uint8_t* buffer, uint16_t offset, uint16_t value) {
  buffer[offset] = value & 0xFF;
  buffer[offset + 1] = (value >> 8) & 0xFF;
}

bool advanceDevNonceInRAM(uint16_t jumpAmount) {
  uint8_t* nonceBuffer = node.getBufferNonces();

  uint16_t currentDevNonce = readU16LEFromBuffer(
    nonceBuffer,
    RADIOLIB_LORAWAN_NONCES_DEV_NONCE
  );

  if ((uint32_t)currentDevNonce + jumpAmount > 0xFFFF) {
    DEBUG_SERIAL.println(F("ERROR: DevNonce overflow risk. Cannot jump DevNonce."));
    return false;
  }

  uint16_t newDevNonce = currentDevNonce + jumpAmount;

  writeU16LEToBuffer(
    nonceBuffer,
    RADIOLIB_LORAWAN_NONCES_DEV_NONCE,
    newDevNonce
  );

  DEBUG_SERIAL.print(F("DevNonce jumped in RAM from "));
  DEBUG_SERIAL.print(currentDevNonce);
  DEBUG_SERIAL.print(F(" to "));
  DEBUG_SERIAL.println(newDevNonce);

  return true;
}

// -----------------------------------------------------------------------------
// LoRaWAN join / heartbeat / recovery
// -----------------------------------------------------------------------------
void beginOTAAAndLoadNonces() {
  DEBUG_SERIAL.println(F("LoRaWAN OTAA init..."));

  int16_t state = node.beginOTAA(
    joinEUI,
    devEUI,
    NULL,
    appKey);

  if (state != RADIOLIB_ERR_NONE) {
    haltWithMessage("beginOTAA failed", state);
  }

  applyLoRaWANRuntimeSettings();

  DEBUG_SERIAL.println(F("OTAA init OK"));

  loadRadioLibNoncesFromEEPROM();
}

void joinNetworkInstant() {
  joined = false;
  connected = false;
  classCReady = false;

  consecutiveRejoinFailures = 0;

  while (true) {
    DEBUG_SERIAL.println(F("Joining network..."));
    blinkLed(2, 120, 120);

    applyLoRaWANRuntimeSettings();

    LoRaWANJoinEvent_t joinEvent;
    int16_t state = node.activateOTAA(&joinEvent);

    if ((state == RADIOLIB_LORAWAN_NEW_SESSION) || (state == RADIOLIB_LORAWAN_SESSION_RESTORED)) {
      DEBUG_SERIAL.println(F("Join OK"));

      DEBUG_SERIAL.print(F("Accepted DevNonce: "));
      DEBUG_SERIAL.println(joinEvent.devNonce);

      saveRadioLibNoncesToEEPROM();

      joined = true;
      connected = false;
      classCReady = false;
      consecutiveRejoinFailures = 0;

      return;
    }

    consecutiveRejoinFailures++;

    DEBUG_SERIAL.print(F("Join failed: "));
    printStateDecode(DEBUG_SERIAL, state);
  DEBUG_SERIAL.println();

    DEBUG_SERIAL.print(F("Last transmitted DevNonce: "));
    DEBUG_SERIAL.println(joinEvent.devNonce);

    DEBUG_SERIAL.print(F("Consecutive instant join failures: "));
    DEBUG_SERIAL.println(consecutiveRejoinFailures);

    if ((consecutiveRejoinFailures % DEVNONCE_JUMP_EVERY_RETRIES) == 0) {
      DEBUG_SERIAL.print(F("Every "));
      DEBUG_SERIAL.print(DEVNONCE_JUMP_EVERY_RETRIES);
      DEBUG_SERIAL.print(F(" retries reached. Jumping DevNonce by +"));
      DEBUG_SERIAL.println(DEVNONCE_JUMP_AMOUNT);

      if (!advanceDevNonceInRAM(DEVNONCE_JUMP_AMOUNT)) {
        DEBUG_SERIAL.println(F("DevNonce jump failed. Continuing normal retries without reboot."));
      } else {
        uint8_t* debugNonces = node.getBufferNonces();
        uint16_t nextDevNonce = readU16LEFromBuffer(
          debugNonces,
          RADIOLIB_LORAWAN_NONCES_DEV_NONCE
        );

        DEBUG_SERIAL.print(F("NEXT DevNonce after jump: "));
        DEBUG_SERIAL.println(nextDevNonce);
      }
    }

#if ENABLE_MCU_REBOOT_AFTER_REJOIN_FAILS
    if (consecutiveRejoinFailures >= MAX_REJOIN_FAILURES_BEFORE_MCU_REBOOT) {
      DEBUG_SERIAL.println(F("Maximum instant join failures reached."));
      DEBUG_SERIAL.println(F("Saving final RadioLib nonces before MCU reboot."));

      saveRadioLibNoncesToEEPROM();

      DEBUG_SERIAL.println(F("Rebooting MCU."));
      delay(1000);
      softwareReset();
    }
#endif

    delay(1000);
  }
}

bool sendSpecialHeartbeatConfirmed() {
  if (!waitForUplinkAvailability()) {
    connected = false;
    return false;
  }

  applyLoRaWANRuntimeSettings();

  DEBUG_SERIAL.println(F("Sending special confirmed heartbeat: C"));

  uint32_t ackStartMs = millis();

  int16_t state = node.sendReceive(
    (uint8_t*)HEARTBEAT_PAYLOAD,
    sizeof(HEARTBEAT_PAYLOAD),
    LORAWAN_HEARTBEAT_PORT,
    true);

  uint32_t ackElapsedMs = millis() - ackStartMs;

  if (state >= RADIOLIB_ERR_NONE) {
    markLoRaWANTransmissionDone();
  }

  node.setClass(RADIOLIB_LORAWAN_CLASS_C);
  applyLoRaWANRuntimeSettings();

  if (isAckReceivedFromSendReceive(state, ackElapsedMs, F("Heartbeat"))) {
    DEBUG_SERIAL.println(F("Heartbeat ACK/downlink received. Server connection OK."));

    connected = true;
    classCReady = true;
    ackFailCounter = 0;

    return true;
  }

  if (state == RADIOLIB_ERR_NONE) {
    DEBUG_SERIAL.println(F("Heartbeat sent, but no ACK/downlink received."));
  } else {
    DEBUG_SERIAL.print(F("Heartbeat error: "));
    printStateDecode(DEBUG_SERIAL, state);
  DEBUG_SERIAL.println();
  }

  if (state == RADIOLIB_ERR_NETWORK_NOT_JOINED) {
    rejoinRequested = true;
  }

  connected = false;
  return false;
}

bool sendStartupStartedUplink() {
  if (startupHeartbeatSent) {
    DEBUG_SERIAL.println(F("Startup payload A already sent. Skipping."));
    return true;
  }

  if (!waitForUplinkAvailability()) {
    DEBUG_SERIAL.println(F("Startup payload A skipped: uplink not available within 5 seconds."));
    return false;
  }

  applyLoRaWANRuntimeSettings();

  DEBUG_SERIAL.println(F("Sending startup payload: A"));

  int16_t state = node.sendReceive(
    (uint8_t*)STARTUP_HEARTBEAT_PAYLOAD,
    sizeof(STARTUP_HEARTBEAT_PAYLOAD),
    LORAWAN_HEARTBEAT_PORT,
    false);

  if (state >= RADIOLIB_ERR_NONE) {
    markLoRaWANTransmissionDone();
  }

  node.setClass(RADIOLIB_LORAWAN_CLASS_C);
  applyLoRaWANRuntimeSettings();

  if (state < RADIOLIB_ERR_NONE) {
    DEBUG_SERIAL.print(F("Startup payload A error: "));
    printStateDecode(DEBUG_SERIAL, state);
  DEBUG_SERIAL.println();

    if (state == RADIOLIB_ERR_NETWORK_NOT_JOINED) {
      rejoinRequested = true;
    }

    return false;
  }

  DEBUG_SERIAL.println(F("Startup payload A sent."));
  startupHeartbeatSent = true;
  return true;
}

bool resetRadioAndTryOneHeartbeat() {
  DEBUG_SERIAL.println(F("Recovery: resetting/reinitializing radio module..."));

  int16_t state = radio.begin();

  if (state != RADIOLIB_ERR_NONE) {
    DEBUG_SERIAL.print(F("Radio reinit failed: "));
    printStateDecode(DEBUG_SERIAL, state);
  DEBUG_SERIAL.println();
    return false;
  }

  DEBUG_SERIAL.println(F("Radio reinit OK"));

  applyLoRaWANRuntimeSettings();

  node.setClass(RADIOLIB_LORAWAN_CLASS_C);
  applyLoRaWANRuntimeSettings();

  delay(1000);

  DEBUG_SERIAL.println(F("Recovery: trying one confirmed heartbeat after radio reset..."));

  return sendSpecialHeartbeatConfirmed();
}

void setupClassCAndVerifyServer() {
  DEBUG_SERIAL.println(F("Switching to Class C..."));

  node.setClass(RADIOLIB_LORAWAN_CLASS_C);
  applyLoRaWANRuntimeSettings();

  connected = false;
  classCReady = true;
  ackFailCounter = 0;

  if (!startupHeartbeatSent) {
    DEBUG_SERIAL.println(F("Sending startup payload A once only..."));

    if (sendStartupStartedUplink()) {
      DEBUG_SERIAL.println(F("Startup payload A completed."));
    } else {
      DEBUG_SERIAL.println(F("Startup payload A was not sent. Continue Class C listening."));
    }
  } else {
    DEBUG_SERIAL.println(F("Startup payload A already sent earlier. No startup uplink now."));
  }

  if (!startupConfigSent) {
    DEBUG_SERIAL.println(F("Sending startup slot configuration once only..."));

    if (sendStartupSlotConfigUplink()) {
      DEBUG_SERIAL.println(F("Startup slot configuration completed."));
    } else {
      DEBUG_SERIAL.println(F("Startup slot configuration was not sent. Continue Class C listening."));
    }
  } else {
    DEBUG_SERIAL.println(F("Startup slot configuration already sent earlier."));
  }

  node.setClass(RADIOLIB_LORAWAN_CLASS_C);
  applyLoRaWANRuntimeSettings();

  DEBUG_SERIAL.println(F("Class C ready"));
}

void fullJoinAndClassCStart() {
  // A6 owns the sequence until the eighth timed OFF. An OTAA join must
  // not preempt the actuator or its readings, even after radio failure.
  if (isA6SequenceBusy()) {
    rejoinRequested = true;
    DEBUG_SERIAL.println(F("LoRaWAN rejoin deferred until A6 reaches IDLE"));
    return;
  }
  rejoinRequested = false;

  connected = false;
  classCReady = false;
  joined = false;

  beginOTAAAndLoadNonces();
  joinNetworkInstant();
  setupClassCAndVerifyServer();

  applyLoRaWANRuntimeSettings();

  blinkLed(3, 120, 120);

  DEBUG_SERIAL.println(F("READY"));
  DEBUG_SERIAL.print(F("RX buffer size: "));
  DEBUG_SERIAL.println(LORA_RX_BUFFER_SIZE);
  DEBUG_SERIAL.print(F("Max uplink payload ready: "));
  DEBUG_SERIAL.println(node.getMaxPayloadLen());
  DEBUG_SERIAL.println(F("Autopoll + A1/A2/A3 command processing ready."));
  DEBUG_SERIAL.println(F("A6: matching A2 FC06 / A4 FC16 uses raw value and power-first relay sequence."));
  DEBUG_SERIAL.println();
}

void handleMissedHeartbeat() {
  DEBUG_SERIAL.print(F("Missed C heartbeat ACK counter: "));
  DEBUG_SERIAL.println(ackFailCounter);

  if (ackFailCounter < ACK_FAIL_LIMIT) {
    DEBUG_SERIAL.println(F("No immediate retry. Waiting for next scheduled C heartbeat."));
    return;
  }

  DEBUG_SERIAL.println(F("3 scheduled C heartbeats missed. Starting recovery."));

  connected = false;
  classCReady = false;

  if (resetRadioAndTryOneHeartbeat()) {
    DEBUG_SERIAL.println(F("Recovered after radio reset."));
    connected = true;
    classCReady = true;
    ackFailCounter = 0;
    return;
  }

  DEBUG_SERIAL.println(F("Still no ACK after radio reset. Starting instant OTAA rejoin."));

  fullJoinAndClassCStart();
}

void handleClassCDownlink() {
  downlinkLen = 0;

  int16_t state = node.getDownlinkClassC(
    downlinkPayload,
    &downlinkLen,
    &downlinkEvent);

  if (state > 0) {
    connected = true;
    classCReady = true;

    printDownlinkParsed(downlinkPayload, downlinkLen);

    DEBUG_SERIAL.print(F("Port: "));
    DEBUG_SERIAL.println(downlinkEvent.fPort);
    DEBUG_SERIAL.print(F("FCnt: "));
    DEBUG_SERIAL.println(downlinkEvent.fCnt);

    if (!isA6SequenceBusy() && !a5PowerSequenceActive && !a5RegularPollInProgress) blinkCommandReceived();

    processDownlinkCommand(downlinkPayload, downlinkLen);

    DEBUG_SERIAL.println();
  }

  if (state == RADIOLIB_ERR_DOWNLINK_MALFORMED) {
    return;
  }

  if (state < RADIOLIB_ERR_NONE) {
    DEBUG_SERIAL.print(F("Class C RX error: "));
    printStateDecode(DEBUG_SERIAL, state);
  DEBUG_SERIAL.println();

    if (state == RADIOLIB_ERR_NETWORK_NOT_JOINED) {
      connected = false;
      classCReady = false;
      rejoinRequested = true;
    }

    delay(10);
  }
}

void clearSlot(PollSlotConfig& slot) {
  slot.enabled = 0;
  slot.channel = FIXED_CHANNEL;
  slot.slaveId = 0;
  slot.function = 0x03;
  slot.regAddress = 0;
  slot.argument = 0;
}

bool isValidUploadInterval(uint16_t value) {
  return value >= 1;
}

bool isValidSlaveId(uint8_t slaveId) {
  return slaveId >= 1 && slaveId <= 247;
}

bool isValidRegCount(uint16_t regCount) {
  return regCount >= 1 && regCount <= MAX_REGS_PER_SLOT;
}

bool isValidChannel(uint8_t channel) {
  return channel >= 1 && channel <= 4;
}

bool isSupportedSlotFunction(uint8_t functionCode) {
  switch (functionCode) {
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x04:
    case 0x05:
    case 0x06:
      return true;
    default:
      return false;
  }
}

bool isValidSlotArgument(uint8_t functionCode, uint16_t argument) {
  switch (functionCode) {
    case 0x01:
    case 0x02:
    case 0x03:
    case 0x04:
      return isValidRegCount(argument);

    case 0x05:
      return argument == 0x0000 || argument == 0xFF00;

    case 0x06:
      return true;

    default:
      return false;
  }
}

bool isWriteFunction(uint8_t functionCode) {
  return functionCode == 0x05 || functionCode == 0x06;
}

// Modbus write frames are sent to RS485 without waiting for the destination
// to exist or acknowledge. Command validity is checked separately, before TX.
// ModbusMaster remains the ONLY reader and owns all FC01..FC04 transactions.
// Frame: [slave][function][arguments][CRC low][CRC high].
bool dispatchModbusWriteNoAck(uint8_t slaveId, uint8_t functionCode,
                              uint16_t regAddr, uint16_t argument,
                              const uint16_t* values, uint8_t count) {
  if (!isValidSlaveId(slaveId)) return false;
  if (functionCode != 0x05 && functionCode != 0x06 && functionCode != 0x10) return false;
  if (functionCode == 0x05 && argument != 0x0000 && argument != 0xFF00) return false;
  if (functionCode == 0x10 && (count == 0 || count > MAX_FC16_REGS_PER_COMMAND || values == NULL)) return false;

  uint8_t frame[9 + 2 * MAX_FC16_REGS_PER_COMMAND];
  uint8_t len = 0;
  frame[len++] = slaveId;
  frame[len++] = functionCode;
  frame[len++] = highByte(regAddr);
  frame[len++] = lowByte(regAddr);
  if (functionCode == 0x10) {
    frame[len++] = 0;
    frame[len++] = count;
    frame[len++] = (uint8_t)(count * 2U);
    for (uint8_t i = 0; i < count; i++) {
      frame[len++] = highByte(values[i]);
      frame[len++] = lowByte(values[i]);
    }
  } else {
    frame[len++] = highByte(argument);
    frame[len++] = lowByte(argument);
  }
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < len; i++) crc = crc16Update(crc, frame[i]);
  frame[len++] = lowByte(crc);
  frame[len++] = highByte(crc);

  // Only wait for local UART bytes to finish sending. Never wait for a slave reply.
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delay(2);
  size_t written = MODBUS_SERIAL.write(frame, len);
  MODBUS_SERIAL.flush();
  delay(2);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  if (written != len) {
    DEBUG_SERIAL.println(F("Modbus TX-only write failed locally; physical state unknown"));
    return false;
  }
  return true;
}

// A2 FC05 ON uses the existing main loop, so Class C reception is not blocked.
// Only transient relay state is added; no EEPROM/configuration or wire changes.
bool a2RelayOnActive = false;
uint8_t a2RelayOnSlave = 0;
uint16_t a2RelayOnCoil = 0;
uint8_t a2RelayOnAttempts = 0;
uint32_t a2RelayOnLastMs = 0;

bool isA2RelayOnBusy() { return a2RelayOnActive; }

void handleA2RelayOnTask() {
  if (!a2RelayOnActive || (millis() - a2RelayOnLastMs) < RELAY_ON_INTERVAL_MS) return;
  bool sent = dispatchModbusWriteNoAck(a2RelayOnSlave, 0x05, a2RelayOnCoil,
                                       0xFF00U, NULL, 0);
  a2RelayOnAttempts++;
  a2RelayOnLastMs = millis();
  DEBUG_SERIAL.print(F("A2 relay ON pass "));
  DEBUG_SERIAL.print(a2RelayOnAttempts);
  DEBUG_SERIAL.println(sent ? F(" TX attempted") : F(" local TX failed; continuing"));
  if (a2RelayOnAttempts >= 3U) a2RelayOnActive = false;
}

bool executeOneShotWrite(uint8_t channel, uint8_t slaveId, uint8_t functionCode, uint16_t regAddr, uint16_t argument) {
  DEBUG_SERIAL.print(F("CMD A2 one-shot write -> CH "));
  DEBUG_SERIAL.print(channel);
  DEBUG_SERIAL.print(F(", Slave "));
  DEBUG_SERIAL.print(slaveId);
  DEBUG_SERIAL.print(F(", "));
  DEBUG_SERIAL.print(functionName(functionCode));
  DEBUG_SERIAL.print(F(", Addr "));
  DEBUG_SERIAL.print(regAddr);
  DEBUG_SERIAL.print(F(", Arg "));
  DEBUG_SERIAL.println(argument);

  if (functionCode == 0x05 && argument == 0xFF00U) {
    // First ON immediately; the normal main loop schedules the two repeats.
    // Do not stop for missing ACK or a local TX error.
    bool sent = dispatchModbusWriteNoAck(slaveId, 0x05, regAddr, 0xFF00U, NULL, 0);
    a2RelayOnSlave = slaveId;
    a2RelayOnCoil = regAddr;
    a2RelayOnAttempts = 1;
    a2RelayOnLastMs = millis();
    a2RelayOnActive = true;
    DEBUG_SERIAL.println(sent ? F("A2 relay ON pass 1 TX attempted") :
                               F("A2 relay ON pass 1 local TX failure; continuing"));
    return sent;
  }

  // A2 FC05 OFF, FC06 and all other one-shot semantics are unchanged.
  bool ok = dispatchModbusWriteNoAck(slaveId, functionCode, regAddr, argument, NULL, 0);
  if (ok) {
    DEBUG_SERIAL.println(F("One-shot write frame sent; slave acknowledgment NOT required"));
  } else {
    DEBUG_SERIAL.println(F("One-shot write frame could not be sent locally"));
  }
  return ok;
}

bool executeWriteMultipleRegisters(uint8_t channel, uint8_t slaveId, uint16_t startAddr, const uint16_t* values, uint8_t regCount) {
  if (!isValidChannel(channel)) {
    DEBUG_SERIAL.println(F("FC16 invalid channel"));
    return false;
  }

  if (!isValidSlaveId(slaveId)) {
    DEBUG_SERIAL.println(F("FC16 invalid slave id"));
    return false;
  }

  if (regCount == 0 || regCount > MAX_FC16_REGS_PER_COMMAND) {
    DEBUG_SERIAL.println(F("FC16 invalid register count"));
    return false;
  }

  DEBUG_SERIAL.print(F("FC16 write multiple -> CH "));
  DEBUG_SERIAL.print(channel);
  DEBUG_SERIAL.print(F(", Slave "));
  DEBUG_SERIAL.print(slaveId);
  DEBUG_SERIAL.print(F(", Start Addr "));
  DEBUG_SERIAL.print(startAddr);
  DEBUG_SERIAL.print(F(", Count "));
  DEBUG_SERIAL.println(regCount);

  DEBUG_SERIAL.print(F("FC16 values: "));
  for (uint8_t i = 0; i < regCount; i++) {
    DEBUG_SERIAL.print(F("0x"));
    if (values[i] < 0x1000) DEBUG_SERIAL.print('0');
    if (values[i] < 0x0100) DEBUG_SERIAL.print('0');
    if (values[i] < 0x0010) DEBUG_SERIAL.print('0');
    DEBUG_SERIAL.print(values[i], HEX);
    DEBUG_SERIAL.print(' ');
  }
  DEBUG_SERIAL.println();

  bool sent = dispatchModbusWriteNoAck(slaveId, 0x10, startAddr, 0, values, regCount);
  DEBUG_SERIAL.println(sent ? F("FC16 frame sent; no slave ACK required") :
                              F("FC16 local TX error; physical state unknown"));
  return sent;
}

const __FlashStringHelper* functionName(uint8_t functionCode) {
  switch (functionCode) {
    case 0x01: return F("FC01");
    case 0x02: return F("FC02");
    case 0x03: return F("FC03");
    case 0x04: return F("FC04");
    case 0x05: return F("FC05");
    case 0x06: return F("FC06");
    case 0x10: return F("FC16");
    default:   return F("UNKNOWN");
  }
}

bool hasAnyEnabledSlot(const PersistedConfig& cfg) {
  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    if (cfg.slots[i].enabled) {
      return true;
    }
  }
  return false;
}

bool configsEqual(const PersistedConfig& a, const PersistedConfig& b) {
  return memcmp(&a, &b, sizeof(PersistedConfig)) == 0;
}

void applyRuntimeConfig() {
  uploadIntervalSeconds = gConfig.uploadIntervalSeconds;
}

void loadDefaultConfig() {
  gConfig.magic = EEPROM_MAGIC;
  gConfig.version = EEPROM_VERSION;
  gConfig.uploadIntervalSeconds = DEFAULT_UPLOAD_INTERVAL_SECONDS;

  // Safe default: no polling slot is enabled at boot/default.
  // Slots must be configured explicitly using A2 commands before regular
  // polling or A3 recording can read Modbus values.
  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    clearSlot(gConfig.slots[i]);
  }

  applyRuntimeConfig();
}

void saveConfigIfChanged(const PersistedConfig& newCfg) {
  PersistedConfig currentStored;
  EEPROM.get(EEPROM_CONFIG_ADDR, currentStored);

  if (configsEqual(currentStored, newCfg)) {
    DEBUG_SERIAL.println(F("EEPROM unchanged"));
    gConfig = newCfg;
    applyRuntimeConfig();
    return;
  }

  EEPROM.put(EEPROM_CONFIG_ADDR, newCfg);
  gConfig = newCfg;
  applyRuntimeConfig();
  DEBUG_SERIAL.println(F("EEPROM updated"));
}

bool configLooksValid(const PersistedConfig& cfg) {
  if (cfg.magic != EEPROM_MAGIC) return false;
  if (cfg.version != EEPROM_VERSION) return false;
  if (!isValidUploadInterval(cfg.uploadIntervalSeconds)) return false;

  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    if (cfg.slots[i].enabled > 1) return false;
    if (cfg.slots[i].enabled) {
      if (!isValidChannel(cfg.slots[i].channel)) return false;
      if (!isValidSlaveId(cfg.slots[i].slaveId)) return false;
      if (!isSupportedSlotFunction(cfg.slots[i].function)) return false;
      if (!isValidSlotArgument(cfg.slots[i].function, cfg.slots[i].argument)) return false;
    }
  }

  return true;
}

bool legacyConfigLooksValid(const LegacyPersistedConfig& cfg) {
  if (cfg.magic != EEPROM_MAGIC) return false;
  if (cfg.version != EEPROM_LEGACY_VERSION_12_SLOTS) return false;
  if (!isValidUploadInterval(cfg.uploadIntervalSeconds)) return false;

  for (uint8_t i = 0; i < LEGACY_MAX_POLL_SLOTS; i++) {
    if (cfg.slots[i].enabled > 1) return false;
    if (cfg.slots[i].enabled) {
      if (!isValidChannel(cfg.slots[i].channel)) return false;
      if (!isValidSlaveId(cfg.slots[i].slaveId)) return false;
      if (!isSupportedSlotFunction(cfg.slots[i].function)) return false;
      if (!isValidSlotArgument(cfg.slots[i].function, cfg.slots[i].argument)) return false;
    }
  }

  return true;
}

bool migrateLegacyConfigIfNeeded() {
  LegacyPersistedConfig legacyCfg;
  EEPROM.get(EEPROM_CONFIG_ADDR, legacyCfg);

  if (!legacyConfigLooksValid(legacyCfg)) {
    return false;
  }

  DEBUG_SERIAL.println(F("EEPROM legacy 12-slot config found. Migrating to 20 slots."));

  PersistedConfig newCfg;
  newCfg.magic = EEPROM_MAGIC;
  newCfg.version = EEPROM_VERSION;
  newCfg.uploadIntervalSeconds = legacyCfg.uploadIntervalSeconds;

  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    clearSlot(newCfg.slots[i]);
  }

  for (uint8_t i = 0; i < LEGACY_MAX_POLL_SLOTS; i++) {
    newCfg.slots[i] = legacyCfg.slots[i];
  }

  saveConfigIfChanged(newCfg);
  DEBUG_SERIAL.println(F("EEPROM config migration completed. Slots 12 to 19 are empty."));
  return true;
}

void loadConfigFromEEPROM() {
  PersistedConfig storedCfg;
  EEPROM.get(EEPROM_CONFIG_ADDR, storedCfg);

  if (!configLooksValid(storedCfg)) {
    if (migrateLegacyConfigIfNeeded()) {
      return;
    }

    DEBUG_SERIAL.println(F("EEPROM invalid, loading defaults"));
    loadDefaultConfig();
    saveConfigIfChanged(gConfig);
    return;
  }

  gConfig = storedCfg;
  applyRuntimeConfig();
  DEBUG_SERIAL.println(F("EEPROM config loaded"));
}

// -----------------------------------------------------------------------------
// A5 compact EEPROM record. Read one slot on demand; no A5 settings in SRAM.
// Storage remains independent of the proven A1/A2 and RadioLib nonce layouts.
// -----------------------------------------------------------------------------
int a5SlotAddress(uint8_t slot) {
  return A5_EEPROM_BASE + A5_EEPROM_HEADER_BYTES + (int)slot * A5_EEPROM_SLOT_BYTES;
}

uint16_t a5StoredCrc() {
  uint16_t crc = 0xFFFF;
  for (int addr = A5_EEPROM_BASE; addr < A5_EEPROM_CRC_ADDR; addr++) {
    crc = crc16Update(crc, EEPROM.read(addr));
  }
  return crc;
}

void a5SaveCrc() {
  eepromUpdateU16LE(A5_EEPROM_CRC_ADDR, a5StoredCrc());
}

bool readA5Slot(uint8_t slot, uint8_t& slave, uint16_t& coilAddr, uint16_t& seconds) {
  slave = 0;
  coilAddr = 0;
  seconds = 0;
  if (slot >= MAX_POLL_SLOTS) return false;
  int addr = a5SlotAddress(slot);
  slave = EEPROM.read(addr);
  if (slave == 0) return false;  // A5 disabled on this reading slot.
  coilAddr = readU16LE(addr + 1);
  seconds = readU16LE(addr + 3);
  return isValidSlaveId(slave) && seconds >= A5_MIN_STABILIZATION_SECONDS;
}

void a5WriteSlot(uint8_t slot, uint8_t slave, uint16_t coilAddr, uint16_t seconds) {
  int addr = a5SlotAddress(slot);
  EEPROM.update(addr, slave);
  eepromUpdateU16LE(addr + 1, coilAddr);
  eepromUpdateU16LE(addr + 3, seconds);
}

bool a5RecordValid() {
  if (readU16LE(A5_EEPROM_BASE) != A5_EEPROM_MAGIC ||
      EEPROM.read(A5_EEPROM_BASE + 2) != A5_EEPROM_VERSION ||
      readU16LE(A5_EEPROM_CRC_ADDR) != a5StoredCrc()) return false;
  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    int addr = a5SlotAddress(i);
    uint8_t slave = EEPROM.read(addr);
    if (slave != 0 && (!isValidSlaveId(slave) ||
        readU16LE(addr + 3) < A5_MIN_STABILIZATION_SECONDS)) return false;
  }
  return true;
}

// Check previous 145-byte A5 EEPROM format without allocating a 145-byte copy.
bool a5LegacyRecordValid() {
  if (readU16LE(A5_LEGACY_EEPROM_BASE) != 0xA5C0 ||
      EEPROM.read(A5_LEGACY_EEPROM_BASE + 2) != 1) return false;
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < 143; i++) {
    crc = crc16Update(crc, EEPROM.read(A5_LEGACY_EEPROM_BASE + i));
  }
  return crc == readU16LE(A5_LEGACY_EEPROM_BASE + 143);
}

void loadA5ConfigFromEEPROM() {
  a5PowerLockout = EEPROM.read(A5_SAFETY_LOCK_ADDR) == A5_SAFETY_LOCK_MAGIC;
  if (a5PowerLockout) {
    DEBUG_SERIAL.println(F("A5 previous OFF TX warning stored; no relay ON inhibit"));
  }
  if (a5RecordValid()) {
    DEBUG_SERIAL.println(F("A5 power control: compact EEPROM config loaded"));
    return;
  }

  bool legacy = a5LegacyRecordValid();
  eepromUpdateU16LE(A5_EEPROM_BASE, A5_EEPROM_MAGIC);
  EEPROM.update(A5_EEPROM_BASE + 2, A5_EEPROM_VERSION);
  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    uint8_t slave = 0;
    uint16_t coilAddr = 0;
    uint16_t seconds = 0;
    if (legacy) {
      int old = A5_LEGACY_EEPROM_BASE + 3 + (int)i * 7;
      if (EEPROM.read(old) == 1 && EEPROM.read(old + 1) == FIXED_CHANNEL) {
        uint8_t oldSlave = EEPROM.read(old + 2);
        uint16_t oldSeconds = readU16LE(old + 5);
        if (isValidSlaveId(oldSlave) && oldSeconds >= A5_MIN_STABILIZATION_SECONDS) {
          slave = oldSlave;
          coilAddr = readU16LE(old + 3);
          seconds = oldSeconds;
        }
      }
    }
    a5WriteSlot(i, slave, coilAddr, seconds);
  }
  a5SaveCrc();
  DEBUG_SERIAL.println(legacy ? F("A5 power control: older settings migrated without SRAM copy") :
                                F("A5 power control: defaults initialized (all disabled)"));
}

void printA5ConfigAtBoot() {
  DEBUG_SERIAL.println(F("A5 POWER CONTROL CONFIGURATION"));
  bool any = false;
  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    uint8_t slave;
    uint16_t coilAddr, seconds;
    if (!readA5Slot(i, slave, coilAddr, seconds)) continue;
    any = true;
    DEBUG_SERIAL.print(F("  SLOT "));
    DEBUG_SERIAL.print(i);
    DEBUG_SERIAL.print(F(" -> CH 1, Coil Slave "));
    DEBUG_SERIAL.print(slave);
    DEBUG_SERIAL.print(F(", Coil Addr "));
    DEBUG_SERIAL.print(coilAddr);
    DEBUG_SERIAL.print(F(", Stabilization "));
    DEBUG_SERIAL.print(seconds);
    DEBUG_SERIAL.println(F(" seconds"));
  }
  if (!any) DEBUG_SERIAL.println(F("  none"));
  DEBUG_SERIAL.println();
}

void printConfigAtBoot() {
  DEBUG_SERIAL.println(F("CONFIGURATION"));
  DEBUG_SERIAL.print(F("Upload interval: "));
  DEBUG_SERIAL.print(uploadIntervalSeconds);
  DEBUG_SERIAL.println(F(" seconds"));

  DEBUG_SERIAL.println(F("Configured slots:"));
  bool any = false;

  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    if (gConfig.slots[i].enabled) {
      any = true;
      DEBUG_SERIAL.print(F("  SLOT "));
      DEBUG_SERIAL.print(i);
      DEBUG_SERIAL.print(F(" -> CH "));
      DEBUG_SERIAL.print(gConfig.slots[i].channel);
      DEBUG_SERIAL.print(F(", Slave "));
      DEBUG_SERIAL.print(gConfig.slots[i].slaveId);
      DEBUG_SERIAL.print(F(", "));
      DEBUG_SERIAL.print(functionName(gConfig.slots[i].function));
      DEBUG_SERIAL.print(F(", Addr "));
      DEBUG_SERIAL.print(gConfig.slots[i].regAddress);
      DEBUG_SERIAL.print(F(", Arg "));
      DEBUG_SERIAL.println(gConfig.slots[i].argument);
    }
  }

  if (!any) {
    DEBUG_SERIAL.println(F("  none"));
  }

  DEBUG_SERIAL.println();
}

void preTransmission() {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delay(2);
}

void postTransmission() {
  delay(2);
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

bool executePollSlot(const PollSlotConfig& slot, uint8_t* responseData, uint8_t& responseLen) {
  responseLen = 0;

  if (!isValidChannel(slot.channel)) {
    return false;
  }

  if (!isValidSlaveId(slot.slaveId)) {
    return false;
  }

  if (!isSupportedSlotFunction(slot.function)) {
    return false;
  }

  if (!isValidSlotArgument(slot.function, slot.argument)) {
    return false;
  }

  mb.begin(slot.slaveId, MODBUS_SERIAL);
  mb.preTransmission(preTransmission);
  mb.postTransmission(postTransmission);

  uint8_t result = 0xFF;

  switch (slot.function) {
    case 0x01: {
      result = mb.readCoils(slot.regAddress, slot.argument);
      if (result == mb.ku8MBSuccess) {
        responseLen = (uint8_t)((slot.argument + 7U) / 8U);
        for (uint8_t i = 0; i < responseLen; i++) {
          uint16_t packedCoils = mb.getResponseBuffer(i / 2U);
          responseData[i] = (i & 1U) ? highByte(packedCoils) : lowByte(packedCoils);
        }
        return true;
      }
      break;
    }

    case 0x02: {
      result = mb.readDiscreteInputs(slot.regAddress, slot.argument);
      if (result == mb.ku8MBSuccess) {
        responseLen = (uint8_t)((slot.argument + 7U) / 8U);
        for (uint8_t i = 0; i < responseLen; i++) {
          responseData[i] = lowByte(mb.getResponseBuffer(i));
        }
        return true;
      }
      break;
    }

    case 0x03: {
      result = mb.readHoldingRegisters(slot.regAddress, slot.argument);
      if (result == mb.ku8MBSuccess) {
        responseLen = (uint8_t)(slot.argument * 2U);
        for (uint16_t i = 0; i < slot.argument; i++) {
          uint16_t value = mb.getResponseBuffer(i);
          responseData[(i * 2U)]     = highByte(value);
          responseData[(i * 2U) + 1] = lowByte(value);
        }
        return true;
      }
      break;
    }

    case 0x04: {
      result = mb.readInputRegisters(slot.regAddress, slot.argument);
      if (result == mb.ku8MBSuccess) {
        responseLen = (uint8_t)(slot.argument * 2U);
        for (uint16_t i = 0; i < slot.argument; i++) {
          uint16_t value = mb.getResponseBuffer(i);
          responseData[(i * 2U)]     = highByte(value);
          responseData[(i * 2U) + 1] = lowByte(value);
        }
        return true;
      }
      break;
    }

    case 0x05: {
      uint8_t coilState = (slot.argument == 0xFF00U) ? 1U : 0U;
      result = mb.writeSingleCoil(slot.regAddress, coilState);
      if (result == mb.ku8MBSuccess) {
        responseData[0] = highByte(slot.regAddress);
        responseData[1] = lowByte(slot.regAddress);
        responseData[2] = highByte(slot.argument);
        responseData[3] = lowByte(slot.argument);
        responseLen = 4;
        return true;
      }
      break;
    }

    case 0x06: {
      result = mb.writeSingleRegister(slot.regAddress, slot.argument);
      if (result == mb.ku8MBSuccess) {
        responseData[0] = highByte(slot.regAddress);
        responseData[1] = lowByte(slot.regAddress);
        responseData[2] = highByte(slot.argument);
        responseData[3] = lowByte(slot.argument);
        responseLen = 4;
        return true;
      }
      break;
    }

    default:
      break;
  }

  return false;
}


// A5 uses executePollSlot() and the SAME ModbusMaster as the existing A2.
// The caller supplies its existing Modbus result buffer; no new large buffers.
bool writeA5PowerCoil(uint8_t slave, uint16_t coilAddr, bool on, uint8_t* response) {
  uint16_t value = on ? 0xFF00U : 0x0000U;
  bool sent = dispatchModbusWriteNoAck(slave, 0x05, coilAddr, value, NULL, 0);
  if (response != NULL) {
    response[0] = highByte(coilAddr);
    response[1] = lowByte(coilAddr);
    response[2] = highByte(value);
    response[3] = lowByte(value);
  }
  DEBUG_SERIAL.print(F("A5 coil Slave "));
  DEBUG_SERIAL.print(slave);
  DEBUG_SERIAL.print(F(" Addr "));
  DEBUG_SERIAL.print(coilAddr);
  DEBUG_SERIAL.print(on ? F(" ON") : F(" OFF"));
  DEBUG_SERIAL.println(sent ? F(" -> TX sent (NOT verified)") :
                             F(" -> local TX ERROR"));
  return sent;
}


bool turnA5PowerOff(uint8_t slave, uint16_t coilAddr, uint8_t* response) {
  for (uint8_t attempt = 0; attempt < A5_OFF_TRIES; attempt++) {
    if (writeA5PowerCoil(slave, coilAddr, false, response)) return true;
    if (attempt + 1 < A5_OFF_TRIES) delay(A5_OFF_RETRY_MS);
  }
  DEBUG_SERIAL.println(F("A5 WARNING: coil OFF frame could not be dispatched after 3 tries"));
  return false;
}

// Preserve the original A5 EEPROM warning marker separately from mappings. If OFF fails,
// mappings MUST stay available for later emergency retries and startup cleanup.
void setA5SafetyLockout(bool locked) {
  a5PowerLockout = locked;
  EEPROM.update(A5_SAFETY_LOCK_ADDR, locked ? A5_SAFETY_LOCK_MAGIC : 0xFF);
  if (locked) DEBUG_SERIAL.println(F("A5 WARNING: previous relay OFF local TX failed; ON sequence remains enabled"));
}

// Check all 20 A5 mappings, including slots whose A2 readings are disabled.
// Physical keys are (slave ID, coil address); shared coils receive one FC05 OFF.
// Continue attempting every coil even if one fails. Never erase a failed key.
bool turnOffAllConfiguredA5Coils() {
  uint8_t response[4];
  bool allOff = true;
  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    uint8_t slave;
    uint16_t coilAddr, seconds;
    if (!readA5Slot(i, slave, coilAddr, seconds)) continue;
    bool handled = false;
    for (uint8_t j = 0; j < i; j++) {
      uint8_t oldSlave;
      uint16_t oldAddr, oldSeconds;
      if (readA5Slot(j, oldSlave, oldAddr, oldSeconds) &&
          oldSlave == slave && oldAddr == coilAddr) {
        handled = true;
        break;
      }
    }
    if (handled) continue;
    // The ON/OFF state of the physical relay cannot be confirmed without a reply.
    // Only a LOCAL UART dispatch failure prevents the cleanup from proceeding.
    bool sent = turnA5PowerOff(slave, coilAddr, response);
    if (!sent) sent = turnA5PowerOff(slave, coilAddr, response);
    if (!sent) {
      allOff = false;
      DEBUG_SERIAL.println(F("A5 WARNING: OFF not dispatched; preserving mapping for recovery"));
    }
  }
  if (!allOff) setA5SafetyLockout(true);
  return allOff;
}

// A5 emergency STOP: send OFF frames to all mapped relays without ACK checks.
// Disable mappings after successful LOCAL transmission, never claim physical OFF.
// If local TX fails, preserve recovery mappings but do not inhibit relay ON.
bool handleA5DisableAll() {
  DEBUG_SERIAL.println(F("CMD A5FFFF accepted: attempt OFF of all mapped coils and disable A5"));
  // Preserve the existing warning marker while handling emergency OFF requests.
  setA5SafetyLockout(true);
  bool allOffDispatched = turnOffAllConfiguredA5Coils();
  if (allOffDispatched) {
    for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) a5WriteSlot(i, 0, 0, 0);
    a5SaveCrc();
    setA5SafetyLockout(false);
    DEBUG_SERIAL.println(F("CMD A5FFFF: OFF requests SENT (physical state unverified); mappings disabled"));
  } else {
    DEBUG_SERIAL.println(F("CMD A5FFFF: local TX failure; retaining mappings for recovery (no automatic ON inhibit)"));
  }
  // 0042 represents a VALID and ACCEPTED emergency request, not relay feedback.
  sendCommandAppliedAck();
  return allOffDispatched;
}

// Keep Class C responsive. Only an explicit A5FFFF emergency can interrupt A5;
// normal downlinks are deferred and a network rejoin cannot stop relay timing.
bool waitA5Stabilization(uint16_t seconds) {
  uint32_t startedMs = millis();
  while (!a5PollCancelled &&
         (millis() - startedMs) < ((uint32_t)seconds * 1000UL)) {
    handleClassCDownlink();
    if (a5PollCancelled) return false;
    delay(10);
    yield();
  }
  return !a5PollCancelled;
}

// Recovery before LoRaWAN join: a reset may have happened with a coil ON.
// No change to the working boot path if all A5 slots are disabled.
void powerOffA5AtStartup() {
  uint8_t response[4];
  bool modbusReady = false;
  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    uint8_t slave;
    uint16_t coilAddr, seconds;
    if (!readA5Slot(i, slave, coilAddr, seconds)) continue;
    if (!modbusReady) {
      MODBUS_SERIAL.begin(MODBUS_BAUD, SERIAL_8N1);
      mb.begin(1, MODBUS_SERIAL);
      mb.preTransmission(preTransmission);
      mb.postTransmission(postTransmission);
      modbusReady = true;
    }
    // Avoid repeated writes if multiple slots share the same power coil.
    bool alreadyHandled = false;
    for (uint8_t j = 0; j < i; j++) {
      uint8_t otherSlave;
      uint16_t otherAddr, otherSeconds;
      if (readA5Slot(j, otherSlave, otherAddr, otherSeconds) &&
          otherSlave == slave && otherAddr == coilAddr) {
        alreadyHandled = true;
        break;
      }
    }
    if (alreadyHandled) continue;
    DEBUG_SERIAL.print(F("A5 boot safety OFF, slot "));
    DEBUG_SERIAL.println(i);
    if (!turnA5PowerOff(slave, coilAddr, response)) {
      if (!turnA5PowerOff(slave, coilAddr, response)) {
        setA5SafetyLockout(true);  // Persist safe state even after a reboot.
      }
    }
  }
}

// -----------------------------------------------------------------------------
// A6: configurable 4-20 mA actuator sequence. Setup NEVER starts an operation.
// Configuration payload (30 bytes):
// 000000A6 | AO power [slave, coilBE] | AI power [slave, coilBE] |
// AO setpoint [slave, FC(06/10), registerBE] | actuator [slave, coilBE] |
// rawMinBE | rawMaxBE | T1BE | T2BE | T3BE | extra power [slave, coilBE].
// T1/T2/T3 are uint16 seconds; zero is immediate. No address or time default.
// -----------------------------------------------------------------------------
uint16_t a6ReadBE(const uint8_t* p) {
  return ((uint16_t)p[0] << 8) | p[1];
}

void a6DecodeConfig(const uint8_t* data, A6SequenceConfig& cfg) {
  cfg.aoPowerSlave = data[0];
  cfg.aoPowerCoil = a6ReadBE(data + 1);
  cfg.aiPowerSlave = data[3];
  cfg.aiPowerCoil = a6ReadBE(data + 4);
  cfg.aoSetpointSlave = data[6];
  cfg.aoSetpointFunction = data[7];
  cfg.aoSetpointRegister = a6ReadBE(data + 8);
  cfg.actuatorSlave = data[10];
  cfg.actuatorCoil = a6ReadBE(data + 11);
  cfg.rawMinimum = a6ReadBE(data + 13);
  cfg.rawMaximum = a6ReadBE(data + 15);
  cfg.t1Seconds = a6ReadBE(data + 17);
  cfg.t2Seconds = a6ReadBE(data + 19);
  cfg.t3Seconds = a6ReadBE(data + 21);
  cfg.extraPowerSlave = data[23];
  cfg.extraPowerCoil = a6ReadBE(data + 24);
}

bool a6CoilEqual(uint8_t slaveA, uint16_t coilA, uint8_t slaveB, uint16_t coilB) {
  return slaveA == slaveB && coilA == coilB;
}

bool a6ConfigLooksValid(const A6SequenceConfig& cfg) {
  if (!isValidSlaveId(cfg.aoPowerSlave) || !isValidSlaveId(cfg.aiPowerSlave) ||
      !isValidSlaveId(cfg.extraPowerSlave) ||
      !isValidSlaveId(cfg.aoSetpointSlave) || !isValidSlaveId(cfg.actuatorSlave)) return false;
  if (cfg.aoSetpointFunction != 0x06 && cfg.aoSetpointFunction != 0x10) return false;
  if (cfg.rawMinimum >= cfg.rawMaximum) return false;
  // Each ON/OFF command must operate one distinct physical relay.
  const uint8_t slave[4] = {cfg.aoPowerSlave, cfg.aiPowerSlave,
                            cfg.extraPowerSlave, cfg.actuatorSlave};
  const uint16_t coil[4] = {cfg.aoPowerCoil, cfg.aiPowerCoil,
                             cfg.extraPowerCoil, cfg.actuatorCoil};
  for (uint8_t i = 0; i < 4; i++) {
    for (uint8_t j = 0; j < i; j++) {
      if (a6CoilEqual(slave[i], coil[i], slave[j], coil[j])) return false;
    }
  }
  return true;
}

bool validateA6Command(const uint8_t* data, size_t len) {
  if (data == NULL || len != A6_V2_COMMAND_LEN) return false;
  if (data[0] != 0 || data[1] != 0 || data[2] != 0 || data[3] != 0xA6) return false;
  A6SequenceConfig candidate;
  a6DecodeConfig(data + 4, candidate);
  return a6ConfigLooksValid(candidate);
}

uint16_t a6V2StoredCrc() {
  uint16_t crc = 0xFFFF;
  for (int addr = A6_V2_EEPROM_BASE; addr < A6_V2_CRC_ADDR; addr++) {
    crc = crc16Update(crc, EEPROM.read(addr));
  }
  return crc;
}

bool a6V2RecordValid() {
  return readU16LE(A6_V2_EEPROM_BASE) == A6_EEPROM_MAGIC &&
         EEPROM.read(A6_V2_EEPROM_BASE + 2) == A6_V2_EEPROM_VERSION &&
         readU16LE(A6_V2_CRC_ADDR) == a6V2StoredCrc();
}


uint16_t a6StoredCrc() {
  uint16_t crc = 0xFFFF;
  for (int addr = A6_EEPROM_BASE; addr < A6_EEPROM_CRC_ADDR; addr++) {
    crc = crc16Update(crc, EEPROM.read(addr));
  }
  return crc;
}

void a6LatchUnverifiedOff(const A6SequenceConfig& cfg) {
  // Do not overwrite an older unresolved key with a newly configured device.
  if (!a6SafetyLockout) {
    a6RecoverySlave = cfg.actuatorSlave;
    a6RecoveryCoil = cfg.actuatorCoil;
    EEPROM.update(A6_RECOVERY_SLAVE_ADDR, a6RecoverySlave);
    eepromUpdateU16LE(A6_RECOVERY_COIL_ADDR, a6RecoveryCoil);
    EEPROM.update(A6_SAFETY_LOCK_ADDR, A6_SAFETY_LOCK_MAGIC); // marker written last
  }
  a6SafetyLockout = true;
  DEBUG_SERIAL.println(F("A6 SAFETY: actuator OFF unverified; recovery coil preserved; ON inhibited"));
}

void a6ClearSafetyLockout() {
  if (!a6SafetyLockout) return;
  EEPROM.update(A6_SAFETY_LOCK_ADDR, 0xFF);
  a6SafetyLockout = false;
  a6RecoverySlave = 0;
  a6RecoveryCoil = 0;
  DEBUG_SERIAL.println(F("A6 legacy recovery marker cleared after scheduled OFF attempts (unverified)"));
}

void loadA6ConfigFromEEPROM() {
  a6Configured = false;
  // Keep the existing v1 actuator safety/recovery record fully compatible.
  a6SafetyLockout = (EEPROM.read(A6_SAFETY_LOCK_ADDR) == A6_SAFETY_LOCK_MAGIC);
  a6RecoverySlave = a6SafetyLockout ? EEPROM.read(A6_RECOVERY_SLAVE_ADDR) : 0;
  a6RecoveryCoil = a6SafetyLockout ? readU16LE(A6_RECOVERY_COIL_ADDR) : 0;
  if (!a6V2RecordValid()) {
    DEBUG_SERIAL.println(F("A6: v2 EEPROM setup absent/invalid; send 30-byte A6 configuration"));
    DEBUG_SERIAL.println(F("A6: original v1 EEPROM/safety bytes preserved"));
    return;
  }
  uint8_t bytes[A6_V2_DATA_BYTES];
  for (uint8_t i = 0; i < A6_V2_DATA_BYTES; i++) {
    bytes[i] = EEPROM.read(A6_V2_DATA_ADDR + i);
  }
  A6SequenceConfig candidate;
  a6DecodeConfig(bytes, candidate);
  if (!a6ConfigLooksValid(candidate)) {
    DEBUG_SERIAL.println(F("A6: v2 setup invalid; sequence DISABLED"));
    return;
  }
  a6Config = candidate;
  a6Configured = true;
  DEBUG_SERIAL.println(F("A6: version 2 EEPROM setup loaded with CRC; NO sequence started"));
  if (a6SafetyLockout) DEBUG_SERIAL.println(F("A6: old actuator safety lockout active"));
}


void printA6ConfigAtBoot() {
  DEBUG_SERIAL.print(F("A6 setup: "));
  if (!a6Configured) {
    DEBUG_SERIAL.println(F("UNCONFIGURED (no operation possible)"));
    return;
  }
  DEBUG_SERIAL.println(F("configured (setup only)"));
  DEBUG_SERIAL.print(F("  AO power: Slave "));
  DEBUG_SERIAL.print(a6Config.aoPowerSlave);
  DEBUG_SERIAL.print(F(" coil "));
  DEBUG_SERIAL.println(a6Config.aoPowerCoil);
  DEBUG_SERIAL.print(F("  AI power: Slave "));
  DEBUG_SERIAL.print(a6Config.aiPowerSlave);
  DEBUG_SERIAL.print(F(" coil "));
  DEBUG_SERIAL.println(a6Config.aiPowerCoil);
  DEBUG_SERIAL.print(F("  Third power relay: Slave "));
  DEBUG_SERIAL.print(a6Config.extraPowerSlave);
  DEBUG_SERIAL.print(F(" coil "));
  DEBUG_SERIAL.println(a6Config.extraPowerCoil);
  DEBUG_SERIAL.print(F("  AO setpoint: Slave "));
  DEBUG_SERIAL.print(a6Config.aoSetpointSlave);
  DEBUG_SERIAL.print(F(" function "));
  DEBUG_SERIAL.print(functionName(a6Config.aoSetpointFunction));
  DEBUG_SERIAL.print(F(" register "));
  DEBUG_SERIAL.println(a6Config.aoSetpointRegister);
  DEBUG_SERIAL.print(F("  Actuator: Slave "));
  DEBUG_SERIAL.print(a6Config.actuatorSlave);
  DEBUG_SERIAL.print(F(" coil "));
  DEBUG_SERIAL.println(a6Config.actuatorCoil);
  DEBUG_SERIAL.print(F("  Raw min/max: "));
  DEBUG_SERIAL.print(a6Config.rawMinimum);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.println(a6Config.rawMaximum);
  DEBUG_SERIAL.print(F("  T1/T2/T3 seconds: "));
  DEBUG_SERIAL.print(a6Config.t1Seconds);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(a6Config.t2Seconds);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.println(a6Config.t3Seconds);
}


// Legacy actuator recovery: three TX-only OFF requests with no FC01/ACK gate.
// Preserve existing EEPROM markers and original saved physical relay mapping.
bool a6SendActuatorOffNoVerify(const A6SequenceConfig& cfg) {
  for (uint8_t pass = 0; pass < 3; pass++) {
    if (pass != 0) delay(A6_RELAY_OFF_INTERVAL_MS);
    bool sent = dispatchModbusWriteNoAck(cfg.actuatorSlave, 0x05,
                   cfg.actuatorCoil, 0x0000, NULL, 0);
    DEBUG_SERIAL.print(F("A6 legacy actuator recovery OFF pass "));
    DEBUG_SERIAL.print(pass + 1);
    DEBUG_SERIAL.println(sent ? F(" sent (no physical feedback)") :
                               F(" local TX failure; continuing"));
  }
  EEPROM.update(A6_ACTUATOR_MAY_BE_ON_ADDR, 0xFF);
  if (a6SafetyLockout && a6RecoverySlave == cfg.actuatorSlave &&
      a6RecoveryCoil == cfg.actuatorCoil) {
    a6ClearSafetyLockout();
  }
  return true; // Completion is time/command based, never communication based.
}

bool a6RetryRecoveryOff() {
  if (!a6SafetyLockout) return true;
  if (!isValidSlaveId(a6RecoverySlave)) {
    DEBUG_SERIAL.println(F("A6 recovery mapping invalid in EEPROM; no valid relay address"));
    return false; // Data validity, not a communication failure.
  }
  A6SequenceConfig recovery = a6Config;
  recovery.actuatorSlave = a6RecoverySlave;
  recovery.actuatorCoil = a6RecoveryCoil;
  DEBUG_SERIAL.println(F("A6 legacy recovery: sending three OFF commands without checks"));
  return a6SendActuatorOffNoVerify(recovery);
}

// New A6 v2 four-relay journal. The mapping and its CRC are written before
// the ACTIVE byte, so a reset during setup never produces an untracked ON.
uint16_t a6JournalStoredCrc() {
  uint16_t crc = 0xFFFF;
  for (int addr = A6_JOURNAL_BASE; addr < A6_JOURNAL_CRC_ADDR; addr++) {
    crc = crc16Update(crc, EEPROM.read(addr));
  }
  return crc;
}

bool a6JournalValid() {
  if (readU16LE(A6_JOURNAL_BASE) != A6_JOURNAL_MAGIC ||
      EEPROM.read(A6_JOURNAL_BASE + 2) != A6_JOURNAL_VERSION ||
      readU16LE(A6_JOURNAL_CRC_ADDR) != a6JournalStoredCrc()) return false;
  for (uint8_t i = 0; i < 4; i++) {
    if (!isValidSlaveId(EEPROM.read(A6_JOURNAL_KEYS_ADDR + 3 * i))) return false;
  }
  return true;
}

bool a6JournalIsActive() {
  return EEPROM.read(A6_JOURNAL_ACTIVE_ADDR) == A6_JOURNAL_ACTIVE_MAGIC;
}

void a6RelayKey(const A6SequenceConfig& cfg, uint8_t keyIndex,
                uint8_t& slave, uint16_t& coil) {
  switch (keyIndex) {
    case 0: slave = cfg.aoPowerSlave; coil = cfg.aoPowerCoil; break;
    case 1: slave = cfg.aiPowerSlave; coil = cfg.aiPowerCoil; break;
    case 2: slave = cfg.extraPowerSlave; coil = cfg.extraPowerCoil; break;
    default: slave = cfg.actuatorSlave; coil = cfg.actuatorCoil; break;
  }
}

bool a6SaveRelayJournal(const A6SequenceConfig& cfg) {
  if (a6JournalIsActive()) return false;  // Do not destroy unresolved keys.
  eepromUpdateU16LE(A6_JOURNAL_BASE, A6_JOURNAL_MAGIC);
  EEPROM.update(A6_JOURNAL_BASE + 2, A6_JOURNAL_VERSION);
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t slave;
    uint16_t coil;
    a6RelayKey(cfg, i, slave, coil);
    EEPROM.update(A6_JOURNAL_KEYS_ADDR + 3 * i, slave);
    EEPROM.update(A6_JOURNAL_KEYS_ADDR + 3 * i + 1, highByte(coil));
    EEPROM.update(A6_JOURNAL_KEYS_ADDR + 3 * i + 2, lowByte(coil));
  }
  eepromUpdateU16LE(A6_JOURNAL_CRC_ADDR, a6JournalStoredCrc());
  if (!a6JournalValid()) return false;
  EEPROM.update(A6_JOURNAL_ACTIVE_ADDR, A6_JOURNAL_ACTIVE_MAGIC); // Commit LAST.
  return a6JournalIsActive();
}

void a6JournalLockOut() {
  EEPROM.update(A6_JOURNAL_LOCK_ADDR, A6_JOURNAL_LOCK_MAGIC);
  DEBUG_SERIAL.println(F("A6 SAFETY: invalid active journal; relay keys cannot be trusted"));
}

// Time-driven FC05 OFF. Never wait for FC05 response or FC01 to advance.
// Up to three LOCAL dispatch tries; the 3-second inter-coil schedule is intact.
bool a6TimedOffCoil(uint8_t slave, uint16_t coil) {
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (dispatchModbusWriteNoAck(slave, 0x05, coil, 0x0000, NULL, 0)) return true;
    if (attempt < 2) delay(8);
  }
  DEBUG_SERIAL.println(F("A6 WARNING: coil OFF could not be locally transmitted"));
  return false;
}

// Recovery needs no current configuration: use the snapshotted physical keys.
// Called before radio joins, or before any new A6 sequence is permitted.
bool a6RecoverJournalRelays() {
  if (!a6JournalIsActive()) return EEPROM.read(A6_JOURNAL_LOCK_ADDR) != A6_JOURNAL_LOCK_MAGIC;
  if (!a6JournalValid()) {
    DEBUG_SERIAL.println(F("A6 SAFETY: active relay journal CRC/key invalid; manual recovery required"));
    a6JournalLockOut();
    return false;
  }
  bool allSent = true;
  DEBUG_SERIAL.println(F("A6 recovery: four original relays OFF at 3-second intervals"));
  for (uint8_t step = 0; step < 4; step++) {
    if (step > 0) delay(A6_RELAY_OFF_INTERVAL_MS);
    uint8_t keyIndex = 3 - step;
    int keyAddr = A6_JOURNAL_KEYS_ADDR + 3 * keyIndex;
    uint8_t slave = EEPROM.read(keyAddr);
    uint16_t coil = ((uint16_t)EEPROM.read(keyAddr + 1) << 8) |
                    EEPROM.read(keyAddr + 2);
    DEBUG_SERIAL.print(F("A6 recovery OFF relay index "));
    DEBUG_SERIAL.println(keyIndex);
    if (!a6TimedOffCoil(slave, coil)) allSent = false;
  }
  if (!allSent) DEBUG_SERIAL.println(F("A6 recovery: some local OFF TX failed; no communication abort"));
  EEPROM.update(A6_JOURNAL_ACTIVE_ADDR, 0xFF);
  EEPROM.update(A6_JOURNAL_LOCK_ADDR, 0xFF);
  DEBUG_SERIAL.println(F("A6 recovery: all four OFF requests transmitted (not physically verified)"));
  return true;
}

// At boot, a configured actuator may still be energized after an MCU reset.
// Only the actuator receives an OFF request. AO/AI power relays are untouched.
// No A6 ON operation is issued on startup or when receiving A6 setup.
void powerOffA6AtStartup() {
  // A6 v2 uses a separate four-key journal; legacy v1 recovery below is
  // deliberately unchanged and remains compatible with prior deployments.
  if (a6JournalIsActive()) {
    MODBUS_SERIAL.begin(MODBUS_BAUD, SERIAL_8N1);
    mb.begin(1, MODBUS_SERIAL);
    mb.preTransmission(preTransmission);
    mb.postTransmission(postTransmission);
    a6RecoverJournalRelays();
  } else if (EEPROM.read(A6_JOURNAL_LOCK_ADDR) == A6_JOURNAL_LOCK_MAGIC) {
    DEBUG_SERIAL.println(F("A6 SAFETY: four-relay journal lockout requires manual review"));
  }
  bool activeRecorded = EEPROM.read(A6_ACTUATOR_MAY_BE_ON_ADDR) ==
                        A6_ACTUATOR_MAY_BE_ON_MAGIC;
  if (!activeRecorded) {
    if (a6SafetyLockout) {
      // An older setup-only firmware could create a safety lock during its
      // unconditional boot probe, even though no A6 sequence could start.
      // Preserve the lock for review, but DO NOT send to an unpowered target.
      DEBUG_SERIAL.println(F("A6 boot: legacy safety lockout with no active marker; manual recovery required"));
    } else {
      DEBUG_SERIAL.println(F("A6 boot: no recorded actuator ON; no Modbus writes at startup"));
    }
    return;
  }
  // The ORIGINAL actuator mapping is saved before every FC05 ON. Recover this
  // physical key even if configuration changed or its CRC was corrupted.
  uint8_t slave = EEPROM.read(A6_RECOVERY_SLAVE_ADDR);
  uint16_t coil = readU16LE(A6_RECOVERY_COIL_ADDR);
  if (!isValidSlaveId(slave)) {
    DEBUG_SERIAL.println(F("A6 boot safety: active marker, invalid relay recovery ID; manual shutdown required"));
    return;
  }
  MODBUS_SERIAL.begin(MODBUS_BAUD, SERIAL_8N1);
  mb.begin(slave, MODBUS_SERIAL);
  mb.preTransmission(preTransmission);
  mb.postTransmission(postTransmission);
  A6SequenceConfig original = {}; // Only the saved actuator key is used.
  original.actuatorSlave = slave;
  original.actuatorCoil = coil;
  DEBUG_SERIAL.println(F("A6 boot: recovering saved actuator with three TX-only OFF commands"));
  a6SendActuatorOffNoVerify(original);
}

void handleCommandA6(const uint8_t* data, size_t len) {
  // Setup only: never energize relays, setpoint, or actuator here.
  if (!validateA6Command(data, len)) {
    sendInvalidCommandMarker();
    return;
  }
  A6SequenceConfig candidate;
  a6DecodeConfig(data + 4, candidate);
  bool changed = !a6V2RecordValid();
  for (uint8_t i = 0; i < A6_V2_DATA_BYTES; i++) {
    if (EEPROM.read(A6_V2_DATA_ADDR + i) != data[4 + i]) changed = true;
  }
  if (changed) {
    // Same EEPROM/update-if-changed/CRC-last convention as the original A6.
    // This does not touch legacy A6/nonce/A5 configuration or safety bytes.
    eepromUpdateU16LE(A6_V2_EEPROM_BASE, A6_EEPROM_MAGIC);
    EEPROM.update(A6_V2_EEPROM_BASE + 2, A6_V2_EEPROM_VERSION);
    for (uint8_t i = 0; i < A6_V2_DATA_BYTES; i++) {
      EEPROM.update(A6_V2_DATA_ADDR + i, data[4 + i]);
    }
    eepromUpdateU16LE(A6_V2_CRC_ADDR, a6V2StoredCrc());
    DEBUG_SERIAL.println(F("A6: v2 setup saved to independent CRC-checked EEPROM record"));
  } else {
    DEBUG_SERIAL.println(F("A6: v2 setup unchanged; EEPROM write skipped"));
  }
  a6Configured = a6V2RecordValid();
  if (a6Configured) a6Config = candidate;
  printA6ConfigAtBoot();
  if (a6Configured) sendCommandAppliedAck();
  else sendInvalidCommandMarker();
}


bool isA6SequenceBusy() {
  return a6State != A6_IDLE;
}

// Protect the original v1 AO target until a valid v2 A6 setup arrives.
// Otherwise an old FC06 could bypass power-first sequencing after an upgrade.
bool a6GetProtectedOutput(uint8_t& slave, uint16_t& address) {
  if (a6Configured) {
    slave = a6Config.aoSetpointSlave;
    address = a6Config.aoSetpointRegister;
    return true;
  }
  if (readU16LE(A6_EEPROM_BASE) != A6_EEPROM_MAGIC ||
      EEPROM.read(A6_EEPROM_BASE + 2) != A6_EEPROM_VERSION ||
      readU16LE(A6_EEPROM_CRC_ADDR) != a6StoredCrc()) return false;
  slave = EEPROM.read(A6_EEPROM_DATA_ADDR + 6);
  address = ((uint16_t)EEPROM.read(A6_EEPROM_DATA_ADDR + 8) << 8) |
            EEPROM.read(A6_EEPROM_DATA_ADDR + 9);
  return isValidSlaveId(slave);
}

bool a6IsProtectedOutputAddress(uint8_t slave, uint16_t address) {
  uint8_t protectedSlave;
  uint16_t protectedAddress;
  return a6GetProtectedOutput(protectedSlave, protectedAddress) &&
         slave == protectedSlave && address == protectedAddress;
}

bool a6MatchesOutput(uint8_t slave, uint8_t fc, uint16_t address) {
  return a6IsProtectedOutputAddress(slave, address) &&
         fc == a6Config.aoSetpointFunction;
}

bool a6RawSetpointValid(uint16_t raw) {
  return a6Configured && raw >= a6Config.rawMinimum &&
         raw <= a6Config.rawMaximum;
}

// Validated LoRaWAN command has already been accepted. Queue the exact raw
// register value supplied at runtime; NEVER substitute a default setpoint.
// Never start the relay cycle inside an A5 polling callback: A5 first needs
// to finish its shutdown barrier for any previously active power coil.
void a6QueueSetpoint(uint16_t raw) {
  a6PendingRaw = raw;
  a6PendingStart = true;
  DEBUG_SERIAL.print(F("A6 queued matching output write; raw value: "));
  DEBUG_SERIAL.println(raw);
}

// A6 v2 post-movement reading. It reuses the A2 slot definitions, existing
// ModbusMaster reader and original regular [0000][slot/FC/len/data][0000]
// format, without ever calling the A5 relay scheduler. If the enabled slots
// exceed one 108-byte uplink, each packet is a normal-format partial page.
void a6ResetPage() {
  a6PageLen = 0;
  a6PageBuffer[a6PageLen++] = 0x00;
  a6PageBuffer[a6PageLen++] = 0x00;
}

void a6BeginShutdown(const __FlashStringHelper* reason);

void a6ReadNextSlot() {
  // No post-T3 timeout: attempt the complete configured read sweep.
  while (a6NextPollSlot < MAX_POLL_SLOTS &&
         !gConfig.slots[a6NextPollSlot].enabled) a6NextPollSlot++;
  if (a6NextPollSlot >= MAX_POLL_SLOTS) {
    if (a6PageLen > 2) {
      a6State = A6_SENDING_SLOTS;
    } else {
      a6BeginShutdown(F("A6 all enabled reading slots acquired and sent"));
    }
    return;
  }

  const PollSlotConfig& slot = gConfig.slots[a6NextPollSlot];
  const uint8_t expected = expectedUnifiedSlotDataLen(slot);
  if (slot.function < 0x01 || slot.function > 0x04 || expected == 0) {
    DEBUG_SERIAL.print(F("A6 refusing non-read polling slot "));
    DEBUG_SERIAL.println(a6NextPollSlot);
    a6NextPollSlot++;
    return;
  }
  size_t packetMax = node.getMaxPayloadLen();
  if (packetMax == 0 || packetMax > MAX_UPLINK_PAYLOAD_SIZE) packetMax = MAX_UPLINK_PAYLOAD_SIZE;
  if ((size_t)expected + 3U + 4U > packetMax) {
    // LoRaWAN availability/capacity is not an A6 control input. Build the
    // complete snapshot and attempt transmission even when the radio reports
    // a smaller or unavailable frame capacity (the uplink may then fail).
    DEBUG_SERIAL.println(F("A6 radio capacity below slot size; continuing full read sweep"));
    packetMax = MAX_UPLINK_PAYLOAD_SIZE;
  }
  if (a6PageLen + 3U + expected + 2U > packetMax) {
    a6State = A6_SENDING_SLOTS;
    return; // Send current page without dropping this next slot.
  }
  const uint8_t current = a6NextPollSlot++;
  a6PageBuffer[a6PageLen++] = current;
  a6PageBuffer[a6PageLen++] = slot.function;
  a6PageBuffer[a6PageLen++] = expected;
  uint8_t* dataStart = &a6PageBuffer[a6PageLen];
  memset(dataStart, 0xFF, expected);
  uint8_t data[2 * MAX_REGS_PER_SLOT];
  uint8_t readLen = 0;
  bool okay = executePollSlot(slot, data, readLen) && readLen == expected;
  if (okay) memcpy(dataStart, data, expected);
  a6PageLen += expected;
  DEBUG_SERIAL.print(F("A6 reading slot "));
  DEBUG_SERIAL.print(current);
  DEBUG_SERIAL.println(okay ? F(" OK") : F(" missing response (FF)"));
}

void a6SendCurrentPage() {
  if (a6PageLen <= 2) {
    a6BeginShutdown(F("A6 no reading data"));
    return;
  }
  a6PageBuffer[a6PageLen++] = 0x00;
  a6PageBuffer[a6PageLen++] = 0x00;
  a6PageCount++;
  DEBUG_SERIAL.print(F("A6 regular all-slot payload page "));
  DEBUG_SERIAL.println(a6PageCount);
  // One confirmed attempt per page, no 15-second A5/A3 resend schedule.
  // A network failure never prevents the subsequent relay OFF sequence.
  bool sent = sendLoRaWanUplinkConfirmed(a6PageBuffer, a6PageLen,
                                         F("A6 regular slot reading"));
  if (!sent) DEBUG_SERIAL.println(F("A6 WARNING: regular slot page not acknowledged"));
  lastUplink = millis();
  a6ResetPage();
  a6State = A6_READING_SLOTS;
}

// A6 power relays ON three passes: 0,1,2,0,1,2,0,1,2; 3 seconds apart.
// The last ON starts T1 immediately. Results are logged, never used to abort.
void a6SendPowerOnStep() {
  const uint8_t keyIndex = a6OnStep % 3U;
  const uint8_t pass = (a6OnStep / 3U) + 1U;
  uint8_t slave;
  uint16_t coil;
  a6RelayKey(a6RunningConfig, keyIndex, slave, coil);
  bool sent = dispatchModbusWriteNoAck(slave, 0x05, coil, 0xFF00, NULL, 0);
  DEBUG_SERIAL.print(F("A6 power ON pass "));
  DEBUG_SERIAL.print(pass);
  DEBUG_SERIAL.print(F(" relay "));
  DEBUG_SERIAL.print(keyIndex);
  DEBUG_SERIAL.println(sent ? F(" TX sent") : F(" local TX failed; continuing"));
  a6OnStep++;
  a6StageStartedMs = millis();
  if (a6OnStep == 9U) {
    a6State = A6_WAIT_T1;
    DEBUG_SERIAL.println(F("A6 power ON passes complete; T1 started"));
  }
}

// A6 actuator ON three times, 3 seconds apart. The third ON starts T3.
void a6SendActuatorOnStep() {
  a6ActuatorMayBeOn = true; // TX-only does not prove whether a relay is ON.
  bool sent = dispatchModbusWriteNoAck(a6RunningConfig.actuatorSlave, 0x05,
                                      a6RunningConfig.actuatorCoil, 0xFF00, NULL, 0);
  a6OnStep++;
  DEBUG_SERIAL.print(F("A6 actuator ON pass "));
  DEBUG_SERIAL.print(a6OnStep);
  DEBUG_SERIAL.println(sent ? F(" TX sent") : F(" local TX failed; continuing"));
  a6StageStartedMs = millis();
  if (a6OnStep == 3U) {
    a6State = A6_WAIT_T3;
    DEBUG_SERIAL.println(F("A6 actuator ON passes complete; T3 started"));
  }
}

// Normal shutdown: two independent timed passes 3,2,1,0,3,2,1,0.
// No reply/FC01/extra immediate retries; every step is attempted on schedule.
void a6SendShutdownStep() {
  const uint8_t keyIndex = 3U - (a6ShutdownStep % 4U);
  const uint8_t pass = (a6ShutdownStep / 4U) + 1U;
  uint8_t slave;
  uint16_t coil;
  a6RelayKey(a6RunningConfig, keyIndex, slave, coil);
  DEBUG_SERIAL.print(F("A6 timed OFF pass "));
  DEBUG_SERIAL.print(pass);
  DEBUG_SERIAL.print(F(" relay "));
  DEBUG_SERIAL.print(keyIndex);
  DEBUG_SERIAL.print(F(" Slave "));
  DEBUG_SERIAL.print(slave);
  DEBUG_SERIAL.print(F(" coil "));
  DEBUG_SERIAL.println(coil);
  bool sent = dispatchModbusWriteNoAck(slave, 0x05, coil, 0x0000, NULL, 0);
  if (!sent) DEBUG_SERIAL.println(F("A6 OFF local TX failed; next scheduled step continues"));
  a6ShutdownStep++;
  a6StageStartedMs = millis();
  if (a6ShutdownStep == 8U) {
    // Never use a local UART result to prevent a later A6 activation.
    EEPROM.update(A6_JOURNAL_ACTIVE_ADDR, 0xFF);
    EEPROM.update(A6_JOURNAL_LOCK_ADDR, 0xFF);
    DEBUG_SERIAL.println(F("A6: all eight timed OFF frames attempted, physical state unverified"));
    a6ActuatorMayBeOn = false;
    a6State = A6_IDLE;
  }
}

void a6BeginShutdown(const __FlashStringHelper* reason) {
  // Start only after the post-T3 slot reading/uplink attempts, or on an
  // explicit internal cleanup request. Never wait for Modbus responses.
  if (a6State == A6_SHUTTING_DOWN ||
      (a6State == A6_IDLE && !a6JournalIsActive())) return;
  DEBUG_SERIAL.print(F("A6 shutdown: "));
  DEBUG_SERIAL.println(reason);
  a6State = A6_SHUTTING_DOWN;
  a6ShutdownStep = 0;
  a6SendShutdownStep();  // Pass 1, Coil 3 OFF immediately at t=0.
}

// No external A6 stop path: only the programmed reading/uplink stage
// leads into the two timed OFF passes (eight independent commands).

bool startA6Sequence(uint16_t rawValue) {
  if (!a6RawSetpointValid(rawValue)) {
    DEBUG_SERIAL.println(F("A6 start unavailable: configuration missing or raw out of range"));
    return false;
  }
  if (isA6SequenceBusy()) {
    // No matching trigger may replace/abort an already-running A6 cycle.
    DEBUG_SERIAL.println(F("A6 already active; new trigger must wait until IDLE"));
    return false;
  }
  if (a6SafetyLockout) {
    if (EEPROM.read(A6_ACTUATOR_MAY_BE_ON_ADDR) == A6_ACTUATOR_MAY_BE_ON_MAGIC) {
      if (!a6RetryRecoveryOff()) return false; // Only invalid EEPROM mapping can block.
    } else {
      // Legacy lock from an earlier TX failure with no recorded active relay.
      a6ClearSafetyLockout();
    }
  }
  if (a6JournalIsActive() && !a6RecoverJournalRelays()) return false;
  if (EEPROM.read(A6_JOURNAL_LOCK_ADDR) == A6_JOURNAL_LOCK_MAGIC) {
    // Old local-UART-failure marker: do not make it a communication abort.
    DEBUG_SERIAL.println(F("A6 previous OFF TX warning cleared; not blocking new A6"));
    EEPROM.update(A6_JOURNAL_LOCK_ADDR, 0xFF);
  }
  a6RunningConfig = a6Config;
  a6RequestedRaw = rawValue;
  a6ActuatorMayBeOn = false;
  a6OnStep = 0;
  a6NextPollSlot = 0;
  a6PageCount = 0;
  a6ResetPage();
  if (!a6SaveRelayJournal(a6RunningConfig)) {
    DEBUG_SERIAL.println(F("A6 ERROR: failed to save relay recovery keys; NO relay ON"));
    return false;
  }
  DEBUG_SERIAL.print(F("A6 v2 start, received raw setpoint: "));
  DEBUG_SERIAL.println(rawValue);
  // Execute three scheduled ON passes. Neither ACKs nor local TX failures
  // influence the command count or the fixed three-second spacing.
  a6State = A6_POWER_ON;
  a6SendPowerOnStep();  // Coil 0 ON immediately at t=0.
  return true;
}

void handleA6SequenceTask() {
  if (a6PendingStart && a6State == A6_IDLE && !isA2RelayOnBusy() &&
      !a5PowerSequenceActive && !a5RegularPollInProgress) {
    const uint16_t raw = a6PendingRaw;
    a6PendingStart = false;
    if (!startA6Sequence(raw)) {
      DEBUG_SERIAL.println(F("A6 trigger could not start; no cycle was begun"));
    }
  }

  // Rejoin requests and additional commands NEVER interrupt active A6.
  if (a6State == A6_IDLE) return;
  uint32_t elapsed = millis() - a6StageStartedMs;
  if (a6State == A6_POWER_ON) {
    if (elapsed >= A6_RELAY_ON_INTERVAL_MS) a6SendPowerOnStep();
    return;
  }
  if (a6State == A6_ACTUATOR_ON) {
    if (elapsed >= A6_RELAY_ON_INTERVAL_MS) a6SendActuatorOnStep();
    return;
  }
  if (a6State == A6_SHUTTING_DOWN) {
    if (elapsed >= A6_RELAY_OFF_INTERVAL_MS) a6SendShutdownStep();
    return;
  }
  if (a6State == A6_WAIT_T1) {
    if (elapsed < (uint32_t)a6RunningConfig.t1Seconds * 1000UL) return;
    // T1 alone controls this transition: NO FC01 power-coil checks.
    const uint16_t raw = a6RequestedRaw;
    bool sent;
    if (a6RunningConfig.aoSetpointFunction == 0x06) {
      sent = dispatchModbusWriteNoAck(a6RunningConfig.aoSetpointSlave, 0x06,
                a6RunningConfig.aoSetpointRegister, raw, NULL, 0);
    } else {
      sent = dispatchModbusWriteNoAck(a6RunningConfig.aoSetpointSlave, 0x10,
                a6RunningConfig.aoSetpointRegister, 0, &raw, 1);
    }
    if (!sent) DEBUG_SERIAL.println(F("A6 setpoint local TX failed; continuing by T2"));
    DEBUG_SERIAL.print(F("A6 setpoint attempted, raw "));
    DEBUG_SERIAL.println(raw);
    a6StageStartedMs = millis();
    a6State = A6_WAIT_T2;
    return;
  }
  if (a6State == A6_WAIT_T2) {
    if (elapsed < (uint32_t)a6RunningConfig.t2Seconds * 1000UL) return;
    a6OnStep = 0;
    a6State = A6_ACTUATOR_ON;
    a6SendActuatorOnStep();  // First actuator ON; repeat twice at 3-second intervals.
    return;
  }
  if (a6State == A6_WAIT_T3) {
    if (elapsed < (uint32_t)a6RunningConfig.t3Seconds * 1000UL) return;
    DEBUG_SERIAL.println(F("A6 T3 complete: all four relays remain ON for one slot sweep/uplink"));
    a6NextPollSlot = 0;
    a6PageCount = 0;
    a6ResetPage();
    a6State = A6_READING_SLOTS;
    return;
  }
  if (a6State == A6_READING_SLOTS) {
    a6ReadNextSlot();
    return;
  }
  if (a6State == A6_SENDING_SLOTS) {
    a6SendCurrentPage();
    return;
  }
}


// -----------------------------------------------------------------------------
// A3 sampling log helpers
// -----------------------------------------------------------------------------
bool isA3LogBusy() {
  return a3State != A3_IDLE;
}

bool sendLoRaWanUplink(const uint8_t* payload, size_t len, const __FlashStringHelper* label) {
  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(F(" -> "));
  dumpHex(payload, len);

  if (!joined || rejoinRequested) {
    DEBUG_SERIAL.println(F("LoRaWAN uplink blocked: node is not joined or rejoin is pending."));
    rejoinRequested = true;
    return false;
  }

  uint32_t cooldownMs = getLoRaWANCooldownRemainingMs();
  if (cooldownMs > 0) {
    DEBUG_SERIAL.print(F("LoRaWAN cooldown remaining ms: "));
    DEBUG_SERIAL.println(cooldownMs);
    delay(cooldownMs);
  }

  if (!waitForUplinkAvailability()) {
    connected = false;
    return false;
  }

  applyLoRaWANRuntimeSettings();

  uint8_t maxPayload = node.getMaxPayloadLen();
  if (len > maxPayload) {
    DEBUG_SERIAL.print(F("LoRaWAN uplink blocked. Payload bytes: "));
    DEBUG_SERIAL.print(len);
    DEBUG_SERIAL.print(F(" / Max allowed: "));
    DEBUG_SERIAL.println(maxPayload);
    return false;
  }

  int16_t state = node.sendReceive((uint8_t*)payload, len, LORAWAN_APP_PORT, false);

  node.setClass(RADIOLIB_LORAWAN_CLASS_C);
  applyLoRaWANRuntimeSettings();

  if (state < RADIOLIB_ERR_NONE) {
    DEBUG_SERIAL.print(F("LoRaWAN uplink error: "));
    printStateDecode(DEBUG_SERIAL, state);
  DEBUG_SERIAL.println();

    if (state == RADIOLIB_ERR_NETWORK_NOT_JOINED) {
      connected = false;
      classCReady = false;
      rejoinRequested = true;
    }

    return false;
  }

  if (state > 0) {
    DEBUG_SERIAL.println(F("LoRaWAN uplink sent, downlink received during send"));
    connected = true;
    classCReady = true;
  } else {
    DEBUG_SERIAL.println(F("LoRaWAN uplink sent"));
  }

  markLoRaWANTransmissionDone();
  return true;
}

bool sendLoRaWanUplinkConfirmed(const uint8_t* payload, size_t len, const __FlashStringHelper* label) {
  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(F(" CONFIRMED -> "));
  dumpHex(payload, len);

  if (!joined || rejoinRequested) {
    DEBUG_SERIAL.println(F("LoRaWAN confirmed uplink blocked: node is not joined or rejoin is pending."));
    rejoinRequested = true;
    return false;
  }

  uint32_t cooldownMs = getLoRaWANCooldownRemainingMs();
  if (cooldownMs > 0) {
    DEBUG_SERIAL.print(F("LoRaWAN cooldown remaining ms: "));
    DEBUG_SERIAL.println(cooldownMs);
    delay(cooldownMs);
  }

  if (!waitForUplinkAvailability()) {
    connected = false;
    return false;
  }

  applyLoRaWANRuntimeSettings();

  uint8_t maxPayload = node.getMaxPayloadLen();
  if (len > maxPayload) {
    DEBUG_SERIAL.print(F("LoRaWAN confirmed uplink blocked. Payload bytes: "));
    DEBUG_SERIAL.print(len);
    DEBUG_SERIAL.print(F(" / Max allowed: "));
    DEBUG_SERIAL.println(maxPayload);
    return false;
  }

  uint32_t ackStartMs = millis();

  int16_t state = node.sendReceive((uint8_t*)payload, len, LORAWAN_APP_PORT, true);

  uint32_t ackElapsedMs = millis() - ackStartMs;

  node.setClass(RADIOLIB_LORAWAN_CLASS_C);
  applyLoRaWANRuntimeSettings();

  if (state >= RADIOLIB_ERR_NONE) {
    markLoRaWANTransmissionDone();
  }

  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(F(" confirmed answer time ms: "));
  DEBUG_SERIAL.println(ackElapsedMs);

  if (state < RADIOLIB_ERR_NONE) {
    DEBUG_SERIAL.print(F("LoRaWAN confirmed uplink error/no ACK: "));
    printStateDecode(DEBUG_SERIAL, state);
  DEBUG_SERIAL.println();

    if (state == RADIOLIB_ERR_NETWORK_NOT_JOINED) {
      connected = false;
      classCReady = false;
      rejoinRequested = true;
    }

    return false;
  }

  if (state > 0) {
    DEBUG_SERIAL.println(F("LoRaWAN confirmed uplink ACK/downlink received"));
    connected = true;
    classCReady = true;
  } else {
    DEBUG_SERIAL.println(F("LoRaWAN confirmed uplink ACK OK"));
  }

  return true;
}


void setCurrentCommandEchoSource(const uint8_t* data, size_t len) {
  currentCommandEchoLen = 0;
  currentCommandEchoPayload = NULL;

  if (len == 0 || data == NULL) {
    return;
  }

  if (len > LORA_RX_BUFFER_SIZE) {
    DEBUG_SERIAL.println(F("Command echo source skipped: command length is larger than RX buffer."));
    return;
  }

  // Do not duplicate the downlink command into a second SRAM buffer.
  // The echo is sent immediately while the original downlink buffer is still valid.
  currentCommandEchoPayload = data;
  currentCommandEchoLen = (uint8_t)len;
}

bool sendCommandStatusEcho(uint8_t statusByte, const uint8_t* data, size_t len, const __FlashStringHelper* label) {
  uint8_t payload[LORA_RX_BUFFER_SIZE + 2];
  size_t payloadLen = 0;

  payload[payloadLen++] = COMMAND_STATUS_PREFIX;
  payload[payloadLen++] = statusByte;

  if (data != NULL && len > 0) {
    if (len > LORA_RX_BUFFER_SIZE) {
      DEBUG_SERIAL.println(F("Command status echo skipped: command length is larger than RX buffer."));
      return false;
    }

    memcpy(&payload[payloadLen], data, len);
    payloadLen += len;
  }

  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(F(" status-prefixed echo -> "));
  dumpHex(payload, payloadLen);

  return sendLoRaWanUplink(payload, payloadLen, label);
}

bool sendCommandAppliedAck() {
  DEBUG_SERIAL.println(F("Sending command accepted/applied payload: 00 42 + echoed command"));

  if (currentCommandEchoPayload != NULL && currentCommandEchoLen > 0) {
    return sendCommandStatusEcho(COMMAND_STATUS_ACCEPTED, currentCommandEchoPayload, currentCommandEchoLen, F("Command accepted 0042 echo"));
  }

  return sendLoRaWanUplink(COMMAND_APPLIED_PAYLOAD, sizeof(COMMAND_APPLIED_PAYLOAD), F("Command applied B"));
}

bool sendInvalidCommandMarker() {
  DEBUG_SERIAL.println(F("Sending invalid/not-applied command payload: 00 47 + echoed command"));

  if (currentCommandEchoPayload != NULL && currentCommandEchoLen > 0) {
    return sendCommandStatusEcho(COMMAND_STATUS_INVALID, currentCommandEchoPayload, currentCommandEchoLen, F("Invalid command 0047 echo"));
  }

  return sendLoRaWanUplink(INVALID_COMMAND_PAYLOAD, sizeof(INVALID_COMMAND_PAYLOAD), F("Invalid command G"));
}

bool sendA3NoConfiguredSlotMarker() {
  DEBUG_SERIAL.println(F("Sending A3 no configured slot payload: H"));
  return sendLoRaWanUplink(A3_NO_CONFIGURED_SLOT_PAYLOAD, sizeof(A3_NO_CONFIGURED_SLOT_PAYLOAD), F("A3 no configured slot H"));
}

bool sendA3ModbusBreakMarker() {
  uint8_t payload[A3_COMMAND_PAYLOAD_LEN];
  const uint8_t* echoPayload = a3OriginalCommandPayload;
  size_t echoLen = a3OriginalCommandLen;

  if (echoLen != A3_COMMAND_PAYLOAD_LEN) {
    buildA3CommandPayload(payload);
    echoPayload = payload;
    echoLen = sizeof(payload);
  }

  uint8_t statusCode = COMMAND_STATUS_A3_MODBUS_NO_RESPONSE;
  const __FlashStringHelper* statusLabel = F("A3 Modbus no response 0049 echo");

  if (a3FinalStatus == A3_FINAL_MODBUS_BREAK_MID_CAPTURE) {
    statusCode = COMMAND_STATUS_A3_CAPTURE_INTERRUPTED;
    statusLabel = F("A3 mid-capture break 004A echo");
    DEBUG_SERIAL.println(F("Sending A3 mid-capture break payload: 00 4A + original A3 command echo"));
  } else {
    DEBUG_SERIAL.println(F("Sending A3 Modbus no-response payload: 00 49 + original A3 command echo"));
  }

  return sendCommandStatusEcho(statusCode, echoPayload, echoLen, statusLabel);
}

bool buildSlotConfigPayloadPage(uint8_t firstSlot, uint8_t slotCount, uint8_t* payload, size_t payloadSize, size_t& payloadLen) {
  // Slot config payload page format:
  // [0]      = 0x44, ASCII 'D'
  // [1..2]  = upload interval seconds, uint16 big-endian
  // [3]      = total available slots in this firmware, currently 20
  // Per slot, 8 bytes:
  //   [slotState][channel][slaveId][function][regHi][regLo][argHi][argLo]
  // slotState bit7 = enabled, bits0..4 = absolute slot index 0..19.
  // Each D/44 payload is fixed to 100 bytes = 200 HEX characters.
  // If this page has fewer slot records than the first page, the remaining bytes
  // are filled with 00 so all D/44 messages have the same payload size.
  if (slotCount == 0) {
    payloadLen = 0;
    DEBUG_SERIAL.println(F("Slot config page rejected: zero slots"));
    return false;
  }

  if (firstSlot >= MAX_POLL_SLOTS || ((uint16_t)firstSlot + slotCount) > MAX_POLL_SLOTS) {
    payloadLen = 0;
    DEBUG_SERIAL.println(F("Slot config page rejected: invalid slot range"));
    return false;
  }

  if (slotCount > SLOT_CONFIG_SLOTS_PER_MESSAGE) {
    payloadLen = 0;
    DEBUG_SERIAL.println(F("Slot config page rejected: too many slots for 200 HEX characters"));
    return false;
  }

  const size_t requiredLen = SLOT_CONFIG_HEADER_SIZE + ((size_t)slotCount * SLOT_CONFIG_RECORD_SIZE);

  if (requiredLen > SLOT_CONFIG_STATUS_MAX_BYTES) {
    payloadLen = 0;
    DEBUG_SERIAL.println(F("Slot config page rejected: would exceed 200 HEX characters"));
    return false;
  }

  if (payloadSize < SLOT_CONFIG_STATUS_MAX_BYTES) {
    payloadLen = 0;
    DEBUG_SERIAL.println(F("Slot config page buffer too small for fixed 100-byte page"));
    return false;
  }

  payloadLen = 0;
  payload[payloadLen++] = SLOT_CONFIG_PAYLOAD_ID;
  payload[payloadLen++] = highByte(gConfig.uploadIntervalSeconds);
  payload[payloadLen++] = lowByte(gConfig.uploadIntervalSeconds);
  payload[payloadLen++] = MAX_POLL_SLOTS;

  for (uint8_t offset = 0; offset < slotCount; offset++) {
    uint8_t i = firstSlot + offset;
    const PollSlotConfig& slot = gConfig.slots[i];
    uint8_t slotState = (uint8_t)(i & 0x1F);

    if (slot.enabled) {
      slotState |= 0x80;
    }

    payload[payloadLen++] = slotState;
    payload[payloadLen++] = slot.channel;
    payload[payloadLen++] = slot.slaveId;
    payload[payloadLen++] = slot.function;
    payload[payloadLen++] = highByte(slot.regAddress);
    payload[payloadLen++] = lowByte(slot.regAddress);
    payload[payloadLen++] = highByte(slot.argument);
    payload[payloadLen++] = lowByte(slot.argument);
  }

  while (payloadLen < SLOT_CONFIG_STATUS_MAX_BYTES) {
    payload[payloadLen++] = 0x00;
  }

  return true;
}

bool sendSlotConfigUplink(const __FlashStringHelper* label) {
  uint8_t messagesSent = 0;

  for (uint8_t firstSlot = 0; firstSlot < MAX_POLL_SLOTS; firstSlot += SLOT_CONFIG_SLOTS_PER_MESSAGE) {
    if (messagesSent >= SLOT_CONFIG_MAX_MESSAGES) {
      DEBUG_SERIAL.println(F("Slot config send stopped: message limit reached"));
      return false;
    }

    uint8_t remainingSlots = MAX_POLL_SLOTS - firstSlot;
    uint8_t slotsThisMessage = remainingSlots;

    if (slotsThisMessage > SLOT_CONFIG_SLOTS_PER_MESSAGE) {
      slotsThisMessage = SLOT_CONFIG_SLOTS_PER_MESSAGE;
    }

    uint8_t payload[SLOT_CONFIG_STATUS_MAX_BYTES];
    size_t payloadLen = 0;

    if (!buildSlotConfigPayloadPage(firstSlot, slotsThisMessage, payload, sizeof(payload), payloadLen)) {
      return false;
    }

    DEBUG_SERIAL.print(F("Slot config D/44 message "));
    DEBUG_SERIAL.print(messagesSent + 1);
    DEBUG_SERIAL.print(F(" first slot "));
    DEBUG_SERIAL.print(firstSlot);
    DEBUG_SERIAL.print(F(", slots "));
    DEBUG_SERIAL.print(slotsThisMessage);
    DEBUG_SERIAL.print(F(", bytes "));
    DEBUG_SERIAL.print(payloadLen);
    DEBUG_SERIAL.print(F(", HEX chars "));
    DEBUG_SERIAL.println(payloadLen * 2U);

    if (!sendLoRaWanUplink(payload, payloadLen, label)) {
      return false;
    }

    messagesSent++;
  }

  return true;
}

bool sendStartupSlotConfigUplink() {
  if (startupConfigSent) {
    DEBUG_SERIAL.println(F("Startup slot configuration already sent. Skipping."));
    return true;
  }

  if (sendSlotConfigUplink(F("Startup slot config D"))) {
    startupConfigSent = true;
    return true;
  }

  return false;
}


void sendStartupHeartbeat() {
  sendStartupStartedUplink();
}

void buildA3CommandPayload(uint8_t* payload) {
  // A3 command format:
  // 000000A3 + RECORD_ID + SLOT_A + SLOT_B + CAPTURE_REPEAT_COUNT
  payload[0] = 0x00;
  payload[1] = 0x00;
  payload[2] = 0x00;
  payload[3] = 0xA3;
  payload[4] = highByte(a3RecordId);
  payload[5] = lowByte(a3RecordId);
  payload[6] = a3SelectedSlots[0];
  payload[7] = a3SelectedSlots[1];
  payload[8] = a3CaptureRepeatTotal;
}

bool sendA3CycleStatusEcho(const __FlashStringHelper* label) {
  // A3 cycle boundary echo: 00 42 + full accepted A3 command.
  // Used at the start and finish of every repeated capture/page set.
  uint8_t payload[A3_COMMAND_PAYLOAD_LEN];
  buildA3CommandPayload(payload);
  return sendCommandStatusEcho(COMMAND_STATUS_ACCEPTED, payload, sizeof(payload), label);
}

void sendA3StartConfirmation() {
  // Kept for compatibility if called later: accepted A3 response is 00 42 + full command.
  sendA3CycleStatusEcho(F("A3 start confirmation 0042 echo"));
}

void resetA3CaptureCycleRuntime() {
  a3TotalPages = 0;
  a3SendIndex = 0;
  a3SendPageNumber = 1;
  a3CurrentBatchRetryCount = 0;
  a3LastPageUploadMillis = 0;
  a3BreakMarkerLastAttemptMillis = 0;
  a3FinalStatus = A3_FINAL_OK;
  a3StorageUsed = 0;
  a3RawSampleLen = expectedA3RawSampleLen();
}

void beginNextA3CaptureCycle() {
  a3CaptureCycleCurrent++;
  resetA3CaptureCycleRuntime();
  buildA3CommandPayload(a3OriginalCommandPayload);
  a3OriginalCommandLen = A3_COMMAND_PAYLOAD_LEN;

  DEBUG_SERIAL.print(F("A3 starting next capture/send cycle "));
  DEBUG_SERIAL.print(a3CaptureCycleCurrent);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.println(a3CaptureRepeatTotal);

  setA3LedActive();
  a3State = A3_SENDING_CYCLE_START_ECHO;
  lastUplink = millis();
}

void sendA3FinalSummary() {
  // This state is reached only after the cycle finish echo has been sent,
  // or when there are no pages to close with an echo.
  bool allA3DataPagesSent = (a3TotalPages > 0 && a3SendIndex >= a3TotalPages);
  bool currentCycleCompletedOk = allA3DataPagesSent &&
                                 (a3FinalStatus == A3_FINAL_OK ||
                                  a3FinalStatus == A3_FINAL_MAX_PAGES_REACHED);

  DEBUG_SERIAL.print(F("A3 capture/send cycle "));
  DEBUG_SERIAL.print(a3CaptureCycleCurrent);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.print(a3CaptureRepeatTotal);
  DEBUG_SERIAL.print(F(" finished locally. Total samples: "));
  DEBUG_SERIAL.print(a3TotalPages);
  DEBUG_SERIAL.print(F(", final status: "));
  DEBUG_SERIAL.println(a3FinalStatus);

  // The repeat byte means repeat the complete capture/send process.
  // Every cycle has: 0042 echo, data pages, 0042 echo.
  if (currentCycleCompletedOk && a3CaptureCycleCurrent < a3CaptureRepeatTotal) {
    beginNextA3CaptureCycle();
    return;
  }

  a3State = A3_IDLE;
  a3CaptureCycleCurrent = 0;
  setA3LedIdle();
  lastUplink = millis();

  DEBUG_SERIAL.println(F("A3 all requested capture/send cycles finished."));
}


bool isA3SelectedSlotValid(uint8_t slotIndex) {
  if (slotIndex >= MAX_POLL_SLOTS) {
    return false;
  }

  if (!gConfig.slots[slotIndex].enabled) {
    return false;
  }

  if (isWriteFunction(gConfig.slots[slotIndex].function)) {
    return false;
  }

  if (expectedUnifiedSlotDataLen(gConfig.slots[slotIndex]) == 0) {
    return false;
  }

  return true;
}

uint8_t countA3SelectedSlots(uint8_t slotA, uint8_t slotB) {
  uint8_t count = 0;

  if (slotA != A3_SELECTED_SLOT_UNUSED) {
    count++;
  }

  if (slotB != A3_SELECTED_SLOT_UNUSED) {
    count++;
  }

  return count;
}

bool validateA3SelectedSlots(uint8_t slotA, uint8_t slotB) {
  uint8_t selectedCount = countA3SelectedSlots(slotA, slotB);

  if (selectedCount == 0) {
    DEBUG_SERIAL.println(F("A3 invalid: no selected slots"));
    return false;
  }

  // Slot A is mandatory because the transmitted page always starts with the
  // fixed 34-byte Slave/Slot A section. For one variable, use A + FF.
  if (slotA == A3_SELECTED_SLOT_UNUSED) {
    DEBUG_SERIAL.println(F("A3 invalid: slot A cannot be FF"));
    return false;
  }

  if (!isA3SelectedSlotValid(slotA)) {
    DEBUG_SERIAL.print(F("A3 invalid slot A: "));
    DEBUG_SERIAL.println(slotA);
    return false;
  }

  if (expectedUnifiedSlotDataLen(gConfig.slots[slotA]) > A3_BATCH_SECTION_SIZE) {
    DEBUG_SERIAL.print(F("A3 invalid: slot A data length is larger than fixed section bytes: "));
    DEBUG_SERIAL.println(A3_BATCH_SECTION_SIZE);
    return false;
  }

  if (slotB != A3_SELECTED_SLOT_UNUSED) {
    if (!isA3SelectedSlotValid(slotB)) {
      DEBUG_SERIAL.print(F("A3 invalid slot B: "));
      DEBUG_SERIAL.println(slotB);
      return false;
    }

    if (expectedUnifiedSlotDataLen(gConfig.slots[slotB]) > A3_BATCH_SECTION_SIZE) {
      DEBUG_SERIAL.print(F("A3 invalid: slot B data length is larger than fixed section bytes: "));
      DEBUG_SERIAL.println(A3_BATCH_SECTION_SIZE);
      return false;
    }
  }

  if ((slotB != A3_SELECTED_SLOT_UNUSED) && (slotA == slotB)) {
    DEBUG_SERIAL.println(F("A3 invalid: slot A and slot B are the same"));
    return false;
  }

  return true;
}

uint8_t expectedUnifiedSlotDataLen(const PollSlotConfig& slot) {
  switch (slot.function) {
    case 0x01:
    case 0x02:
      return (uint8_t)((slot.argument + 7U) / 8U);

    case 0x03:
    case 0x04:
      return (uint8_t)(slot.argument * 2U);

    case 0x05:
    case 0x06:
      return 4;

    default:
      return 0;
  }
}

uint16_t expectedA3RawSampleLen() {
  // Stored per captured sample:
  // [slot A response time uint16 ms] + [slot A raw data]
  // + [slot B response time uint16 ms] + [slot B raw data]
  // If slot B is unused, only Slot A response time and Slot A data are stored.
  // Slot B transmit response time and data are filled with FF during upload.
  if (a3SelectedSlots[0] == A3_SELECTED_SLOT_UNUSED) {
    return 0;
  }

  uint16_t needed = A3_RESPONSE_TIME_BYTES;
  needed += (uint16_t)expectedUnifiedSlotDataLen(gConfig.slots[a3SelectedSlots[0]]);

  if (a3SelectedSlots[1] != A3_SELECTED_SLOT_UNUSED) {
    needed += A3_RESPONSE_TIME_BYTES;
    needed += (uint16_t)expectedUnifiedSlotDataLen(gConfig.slots[a3SelectedSlots[1]]);
  }

  return needed;
}

size_t getA3BatchHeaderLen() {
  // A3 batch fixed overhead only:
  // [slotA][slotB] + [totalTxPages][currentTxPage]
  return 4;
}

size_t getA3FixedBatchLen() {
  // Full A3 data page length for the selected slots:
  // [slotA][slotB]
  // [Slot A response time mode][Slot A full-page data section]
  // Optional [Slot B response time mode][Slot B full-page data section]
  // [totalTxPages][currentTxPage]
  //
  // Important:
  // Full pages are NOT padded with FFFE.
  // Only the final short page is padded with FFFE until it reaches this full-page length.
  uint16_t maxSamplesPerBatch = getA3MaxSamplesPerBatch();
  if (maxSamplesPerBatch == 0 || a3SelectedSlots[0] == A3_SELECTED_SLOT_UNUSED) {
    return 0;
  }

  uint8_t slotADataLen = expectedUnifiedSlotDataLen(gConfig.slots[a3SelectedSlots[0]]);
  if (slotADataLen == 0) {
    return 0;
  }

  size_t len = getA3BatchHeaderLen() + A3_RESPONSE_TIME_BYTES + ((size_t)maxSamplesPerBatch * (size_t)slotADataLen);

  if (a3SelectedSlots[1] != A3_SELECTED_SLOT_UNUSED) {
    uint8_t slotBDataLen = expectedUnifiedSlotDataLen(gConfig.slots[a3SelectedSlots[1]]);
    if (slotBDataLen == 0) {
      return 0;
    }
    len += A3_RESPONSE_TIME_BYTES + ((size_t)maxSamplesPerBatch * (size_t)slotBDataLen);
  }

  return len;
}

uint16_t getA3SlotSamplesPerBatch(uint8_t slotIndex) {
  if (slotIndex == A3_SELECTED_SLOT_UNUSED || slotIndex >= MAX_POLL_SLOTS) {
    return 0;
  }

  uint8_t slotDataLen = expectedUnifiedSlotDataLen(gConfig.slots[slotIndex]);
  if (slotDataLen == 0 || slotDataLen > A3_BATCH_SECTION_SIZE) {
    return 0;
  }

  return (uint16_t)(A3_BATCH_SECTION_SIZE / slotDataLen);
}

uint16_t getA3MaxSamplesPerBatch() {
  // Keep each A3 payload under the 108-byte / 216-HEX maximum.
  // Packing rule per 50-byte slot section:
  // - INT/UINT slot with 1 Modbus register = 2 bytes -> 25 values per page.
  // - FLOAT slot with 2 Modbus registers = 4 bytes -> 12 values per page.
  // - Mixed INT/UINT + FLOAT uses the smaller capacity -> 12 values per page
  //   for both slots, so Slot A and Slot B stay sample-aligned.
  if (a3RawSampleLen == 0) {
    return 0;
  }

  if (a3SelectedSlots[0] == A3_SELECTED_SLOT_UNUSED) {
    return 0;
  }

  uint16_t maxSamples = getA3SlotSamplesPerBatch(a3SelectedSlots[0]);
  if (maxSamples == 0) {
    return 0;
  }

  if (a3SelectedSlots[1] != A3_SELECTED_SLOT_UNUSED) {
    uint16_t maxBSamples = getA3SlotSamplesPerBatch(a3SelectedSlots[1]);
    if (maxBSamples == 0) {
      return 0;
    }

    if (maxSamples > maxBSamples) {
      maxSamples = maxBSamples;
    }
  }

  if (maxSamples > 255) {
    maxSamples = 255;
  }

  size_t fullPageLen = getA3BatchHeaderLen();
  fullPageLen += A3_RESPONSE_TIME_BYTES + ((size_t)maxSamples * (size_t)expectedUnifiedSlotDataLen(gConfig.slots[a3SelectedSlots[0]]));

  if (a3SelectedSlots[1] != A3_SELECTED_SLOT_UNUSED) {
    fullPageLen += A3_RESPONSE_TIME_BYTES + ((size_t)maxSamples * (size_t)expectedUnifiedSlotDataLen(gConfig.slots[a3SelectedSlots[1]]));
  }

  if (fullPageLen > A3_TX_PAGE_MAX_SIZE) {
    DEBUG_SERIAL.print(F("A3 full-page transmit batch too large. Required: "));
    DEBUG_SERIAL.print(fullPageLen);
    DEBUG_SERIAL.print(F(" / Max: "));
    DEBUG_SERIAL.println(A3_TX_PAGE_MAX_SIZE);
    return 0;
  }

  return maxSamples;
}

uint8_t getA3TotalTransmitPages() {
  uint16_t maxSamplesPerBatch = getA3MaxSamplesPerBatch();

  if (maxSamplesPerBatch == 0 || a3TotalPages == 0) {
    return 0;
  }

  uint16_t totalTransmitPages = a3TotalPages / maxSamplesPerBatch;
  if ((a3TotalPages % maxSamplesPerBatch) != 0) {
    totalTransmitPages++;
  }

  if (totalTransmitPages > 255) {
    DEBUG_SERIAL.println(F("A3 total TX pages exceeds one-byte footer limit"));
    return 0;
  }

  return (uint8_t)totalTransmitPages;
}

uint16_t getA3MaximumRawSamples() {
  if (a3RawSampleLen == 0) {
    return 0;
  }

  return (uint16_t)(A3_RAW_DATA_POOL_SIZE / a3RawSampleLen);
}

uint16_t readU16BEFromA3RawPool(uint32_t offset) {
  uint16_t value = ((uint16_t)a3RawDataPool[offset] << 8);
  value |= (uint16_t)a3RawDataPool[offset + 1];
  return value;
}

uint16_t findMostRepeatedA3ResponseTime(uint32_t rawOffset32, uint16_t samplesToSend, uint16_t responseTimeOffsetInSample) {
  if (samplesToSend == 0) {
    return 0xFFFF;
  }

  uint16_t bestValue = readU16BEFromA3RawPool(rawOffset32 + responseTimeOffsetInSample);
  uint16_t bestCount = 0;

  for (uint16_t i = 0; i < samplesToSend; i++) {
    uint32_t currentOffset32 = rawOffset32 + ((uint32_t)i * (uint32_t)a3RawSampleLen) + responseTimeOffsetInSample;
    uint16_t currentValue = readU16BEFromA3RawPool(currentOffset32);
    uint16_t currentCount = 0;

    for (uint16_t j = 0; j < samplesToSend; j++) {
      uint32_t compareOffset32 = rawOffset32 + ((uint32_t)j * (uint32_t)a3RawSampleLen) + responseTimeOffsetInSample;
      uint16_t compareValue = readU16BEFromA3RawPool(compareOffset32);

      if (compareValue == currentValue) {
        currentCount++;
      }
    }

    // If there is a tie, keep the first value that reached the best count.
    if (currentCount > bestCount) {
      bestCount = currentCount;
      bestValue = currentValue;
    }
  }

  return bestValue;
}

bool executeA3PollSlotWithRetries(const __FlashStringHelper* label, uint8_t slotIndex, uint8_t* responseData, uint8_t& responseLen, uint8_t expectedLen, uint16_t& responseTimeMs) {
  responseLen = 0;
  responseTimeMs = 0;

  for (uint8_t attempt = 1; attempt <= A3_MODBUS_READ_MAX_TRIES; attempt++) {
    DEBUG_SERIAL.print(label);
    DEBUG_SERIAL.print(F(" slot "));
    DEBUG_SERIAL.print(slotIndex);
    DEBUG_SERIAL.print(F(" Modbus try "));
    DEBUG_SERIAL.print(attempt);
    DEBUG_SERIAL.print(F(" / "));
    DEBUG_SERIAL.println(A3_MODBUS_READ_MAX_TRIES);

    uint8_t attemptResponseLen = 0;
    uint32_t responseStartMs = millis();
    bool ok = executePollSlot(gConfig.slots[slotIndex], responseData, attemptResponseLen);
    uint32_t responseElapsedMs32 = millis() - responseStartMs;

    if (responseElapsedMs32 > 0xFFFFUL) {
      responseElapsedMs32 = 0xFFFFUL;
    }

    responseTimeMs = (uint16_t)responseElapsedMs32;
    responseLen = attemptResponseLen;

    if (ok && attemptResponseLen == expectedLen) {
      DEBUG_SERIAL.print(label);
      DEBUG_SERIAL.print(F(" slot "));
      DEBUG_SERIAL.print(slotIndex);
      DEBUG_SERIAL.print(F(" answered correctly. Response time ms: "));
      DEBUG_SERIAL.println(responseTimeMs);
      return true;
    }

    DEBUG_SERIAL.print(label);
    DEBUG_SERIAL.print(F(" slot "));
    DEBUG_SERIAL.print(slotIndex);
    DEBUG_SERIAL.print(F(" failed try "));
    DEBUG_SERIAL.print(attempt);

    if (ok) {
      DEBUG_SERIAL.print(F(" due to length mismatch. Expected "));
      DEBUG_SERIAL.print(expectedLen);
      DEBUG_SERIAL.print(F(", received "));
      DEBUG_SERIAL.println(attemptResponseLen);
    } else {
      DEBUG_SERIAL.println(F(" due to no/invalid Modbus response"));
    }

    if (attempt < A3_MODBUS_READ_MAX_TRIES && A3_MODBUS_RETRY_DELAY_MS > 0) {
      delay(A3_MODBUS_RETRY_DELAY_MS);
    }
  }

  DEBUG_SERIAL.print(label);
  DEBUG_SERIAL.print(F(" slot "));
  DEBUG_SERIAL.print(slotIndex);
  DEBUG_SERIAL.println(F(" did not answer correctly after 2 tries. Breaking A3 capture."));

  if (a3TotalPages > 0) {
    a3FinalStatus = A3_FINAL_MODBUS_BREAK_MID_CAPTURE;
  } else {
    a3FinalStatus = A3_FINAL_MODBUS_NO_RESPONSE;
  }
  return false;
}

bool buildAndStoreA3RawSample() {
  if (a3SelectedSlotCount == 0) {
    DEBUG_SERIAL.println(F("A3 no selected slots available"));
    a3FinalStatus = A3_FINAL_NO_ACTIVE_SLOTS;
    return false;
  }

  if (a3RawSampleLen == 0) {
    DEBUG_SERIAL.println(F("A3 raw sample length is zero"));
    a3FinalStatus = A3_FINAL_NO_ACTIVE_SLOTS;
    return false;
  }

  if (((uint32_t)a3StorageUsed + (uint32_t)a3RawSampleLen) > A3_RAW_DATA_POOL_SIZE) {
    DEBUG_SERIAL.println(F("A3 raw data pool full, stopping capture"));
    if (a3FinalStatus == A3_FINAL_OK) {
      a3FinalStatus = A3_FINAL_MAX_PAGES_REACHED;
    }
    return false;
  }

  uint16_t pageNumber = a3TotalPages + 1;
  uint16_t sampleStart = a3StorageUsed;
  uint16_t pos = sampleStart;

  DEBUG_SERIAL.print(F("A3 capturing raw sample "));
  DEBUG_SERIAL.println(pageNumber);

  // Slot A is mandatory. It must answer correctly within 2 tries.
  uint8_t slotAIndex = a3SelectedSlots[0];
  uint8_t slotAExpectedLen = expectedUnifiedSlotDataLen(gConfig.slots[slotAIndex]);
  uint8_t slotAData[2 * MAX_REGS_PER_SLOT];
  uint8_t slotAResponseLen = 0;
  uint16_t slotAResponseTimeMs = 0;

  bool slotAOk = executeA3PollSlotWithRetries(F("A3 Slot A"), slotAIndex, slotAData, slotAResponseLen, slotAExpectedLen, slotAResponseTimeMs);

  DEBUG_SERIAL.print(F("  Slot A "));
  DEBUG_SERIAL.print(slotAIndex);
  DEBUG_SERIAL.print(F(" | "));
  DEBUG_SERIAL.print(functionName(gConfig.slots[slotAIndex].function));
  DEBUG_SERIAL.print(F(" | "));
  DEBUG_SERIAL.print(slotAOk ? F("OK") : F("ERROR"));
  DEBUG_SERIAL.print(F(" | Raw saved: "));

  if (!slotAOk) {
    DEBUG_SERIAL.println(F("NOT SAVED - A3 CAPTURE BREAK"));
    return false;
  }

  // Store Slot A response time before Slot A raw data.
  a3RawDataPool[pos++] = highByte(slotAResponseTimeMs);
  a3RawDataPool[pos++] = lowByte(slotAResponseTimeMs);

  memcpy(&a3RawDataPool[pos], slotAData, slotAExpectedLen);
  dumpHex(slotAData, slotAResponseLen);
  pos += slotAExpectedLen;

  DEBUG_SERIAL.print(F("  Slot A response time ms: "));
  DEBUG_SERIAL.println(slotAResponseTimeMs);

  if (a3SelectedSlots[1] != A3_SELECTED_SLOT_UNUSED) {
    if (A3_SENSOR_TO_SENSOR_DELAY_MS > 0) {
      delay(A3_SENSOR_TO_SENSOR_DELAY_MS);
    }

    // Slot B is optional, but if selected it must answer correctly within 2 tries.
    uint8_t slotBIndex = a3SelectedSlots[1];
    uint8_t slotBExpectedLen = expectedUnifiedSlotDataLen(gConfig.slots[slotBIndex]);
    uint8_t slotBData[2 * MAX_REGS_PER_SLOT];
    uint8_t slotBResponseLen = 0;
    uint16_t slotBResponseTimeMs = 0;

    bool slotBOk = executeA3PollSlotWithRetries(F("A3 Slot B"), slotBIndex, slotBData, slotBResponseLen, slotBExpectedLen, slotBResponseTimeMs);

    DEBUG_SERIAL.print(F("  Slot B "));
    DEBUG_SERIAL.print(slotBIndex);
    DEBUG_SERIAL.print(F(" | "));
    DEBUG_SERIAL.print(functionName(gConfig.slots[slotBIndex].function));
    DEBUG_SERIAL.print(F(" | "));
    DEBUG_SERIAL.print(slotBOk ? F("OK") : F("ERROR"));
    DEBUG_SERIAL.print(F(" | Raw saved: "));

    if (!slotBOk) {
      DEBUG_SERIAL.println(F("NOT SAVED - A3 CAPTURE BREAK"));
      return false;
    }

    // Store Slot B response time before Slot B raw data.
    a3RawDataPool[pos++] = highByte(slotBResponseTimeMs);
    a3RawDataPool[pos++] = lowByte(slotBResponseTimeMs);

    memcpy(&a3RawDataPool[pos], slotBData, slotBExpectedLen);
    dumpHex(slotBData, slotBResponseLen);
    pos += slotBExpectedLen;

    DEBUG_SERIAL.print(F("  Slot B response time ms: "));
    DEBUG_SERIAL.println(slotBResponseTimeMs);

    if (A3_SAMPLE_TO_SAMPLE_DELAY_MS > 0) {
      delay(A3_SAMPLE_TO_SAMPLE_DELAY_MS);
    }
  } else {
    if (A3_SAMPLE_TO_SAMPLE_DELAY_MS > 0) {
      delay(A3_SAMPLE_TO_SAMPLE_DELAY_MS);
    }
  }

  a3StorageUsed += a3RawSampleLen;

  DEBUG_SERIAL.print(F("A3 stored raw sample data: "));
  dumpHex(&a3RawDataPool[sampleStart], a3RawSampleLen);
  DEBUG_SERIAL.print(F("A3 raw storage used: "));
  DEBUG_SERIAL.print(a3StorageUsed);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.println(A3_RAW_DATA_POOL_SIZE);

  a3TotalPages++;
  return true;
}

bool buildA3TransmitBatchFromRawSamples(uint16_t startSampleIndex, uint16_t batchPageNumber, uint8_t* payload, size_t& payloadLen, uint8_t& batchSampleCount) {
  payloadLen = 0;
  batchSampleCount = 0;

  if (startSampleIndex >= a3TotalPages) {
    return false;
  }

  if (a3RawSampleLen == 0) {
    return false;
  }

  uint16_t maxSamplesPerBatch = getA3MaxSamplesPerBatch();
  if (maxSamplesPerBatch == 0) {
    DEBUG_SERIAL.println(F("A3 full-page batch cannot be built"));
    if (a3FinalStatus == A3_FINAL_OK) {
      a3FinalStatus = A3_FINAL_PAGE_TRUNCATED;
    }
    return false;
  }

  uint16_t remainingSamples = a3TotalPages - startSampleIndex;
  uint16_t samplesToSend = remainingSamples;
  if (samplesToSend > maxSamplesPerBatch) {
    samplesToSend = maxSamplesPerBatch;
  }

  uint32_t rawOffset32 = (uint32_t)startSampleIndex * (uint32_t)a3RawSampleLen;
  uint32_t realRawBytes32 = (uint32_t)samplesToSend * (uint32_t)a3RawSampleLen;

  if ((rawOffset32 + realRawBytes32) > a3StorageUsed) {
    return false;
  }

  size_t maxBatchLen = getA3FixedBatchLen();
  if (maxBatchLen > A3_TX_PAGE_MAX_SIZE) {
    DEBUG_SERIAL.print(F("A3 maximum transmit batch too large. Required: "));
    DEBUG_SERIAL.print(maxBatchLen);
    DEBUG_SERIAL.print(F(" / Max: "));
    DEBUG_SERIAL.println(A3_TX_PAGE_MAX_SIZE);
    if (a3FinalStatus == A3_FINAL_OK) {
      a3FinalStatus = A3_FINAL_PAGE_TRUNCATED;
    }
    return false;
  }

  uint8_t totalTransmitPages = getA3TotalTransmitPages();
  if (totalTransmitPages == 0 || batchPageNumber == 0 || batchPageNumber > totalTransmitPages) {
    DEBUG_SERIAL.println(F("A3 invalid TX page footer values"));
    if (a3FinalStatus == A3_FINAL_OK) {
      a3FinalStatus = A3_FINAL_PAGE_TRUNCATED;
    }
    return false;
  }

  // Payload format, full-page slot sections:
  // [slotA][slotB]
  // [2 bytes most-repeated Slot A response time]
  // [Slot A full-page data section; FFFE padding only if this is a short final page]
  // Optional [2 bytes most-repeated Slot B response time]
  // Optional [Slot B full-page data section; FFFE padding only if this is a short final page]
  // [totalTxPages][currentTxPage]

  payload[payloadLen++] = a3SelectedSlots[0];
  payload[payloadLen++] = a3SelectedSlots[1];

  uint8_t slotAIndex = a3SelectedSlots[0];
  uint8_t slotADataLen = expectedUnifiedSlotDataLen(gConfig.slots[slotAIndex]);
  bool hasSlotB = (a3SelectedSlots[1] != A3_SELECTED_SLOT_UNUSED);
  uint8_t slotBDataLen = 0;

  uint16_t slotAResponseTimeOffsetInSample = 0;
  uint16_t slotADataOffsetInSample = A3_RESPONSE_TIME_BYTES;
  uint16_t slotBResponseTimeOffsetInSample = 0;
  uint16_t slotBDataOffsetInSample = 0;

  if (hasSlotB) {
    uint8_t slotBIndex = a3SelectedSlots[1];
    slotBDataLen = expectedUnifiedSlotDataLen(gConfig.slots[slotBIndex]);
    slotBResponseTimeOffsetInSample = A3_RESPONSE_TIME_BYTES + slotADataLen;
    slotBDataOffsetInSample = slotBResponseTimeOffsetInSample + A3_RESPONSE_TIME_BYTES;
  }

  // Slot A response-time mode, exactly 2 bytes.
  uint16_t slotAResponseTimeMode = findMostRepeatedA3ResponseTime(rawOffset32, samplesToSend, slotAResponseTimeOffsetInSample);
  payload[payloadLen++] = highByte(slotAResponseTimeMode);
  payload[payloadLen++] = lowByte(slotAResponseTimeMode);

  // Slot A full-page section. Full pages have no filler; only short final pages get FFFE padding.
  uint16_t slotATargetSectionBytes = (uint16_t)maxSamplesPerBatch * (uint16_t)slotADataLen;
  uint16_t slotASectionBytes = 0;
  for (uint16_t sample = 0; sample < samplesToSend; sample++) {
    uint32_t sourceOffset32 = rawOffset32 + ((uint32_t)sample * (uint32_t)a3RawSampleLen) + slotADataOffsetInSample;
    memcpy(&payload[payloadLen], &a3RawDataPool[sourceOffset32], slotADataLen);
    payloadLen += slotADataLen;
    slotASectionBytes += slotADataLen;
  }

  uint8_t slotAPaddingByteIndex = 0;
  while (slotASectionBytes < slotATargetSectionBytes) {
    payload[payloadLen++] = (slotAPaddingByteIndex % 2 == 0) ? 0xFF : 0xFE;
    slotAPaddingByteIndex++;
    slotASectionBytes++;
  }

  if (hasSlotB) {
    // Slot B response-time mode, exactly 2 bytes.
    uint16_t slotBResponseTimeMode = findMostRepeatedA3ResponseTime(rawOffset32, samplesToSend, slotBResponseTimeOffsetInSample);
    payload[payloadLen++] = highByte(slotBResponseTimeMode);
    payload[payloadLen++] = lowByte(slotBResponseTimeMode);

    // Slot B full-page section. Full pages have no filler; only short final pages get FFFE padding.
    uint16_t slotBTargetSectionBytes = (uint16_t)maxSamplesPerBatch * (uint16_t)slotBDataLen;
    uint16_t slotBSectionBytes = 0;
    for (uint16_t sample = 0; sample < samplesToSend; sample++) {
      uint32_t sourceOffset32 = rawOffset32 + ((uint32_t)sample * (uint32_t)a3RawSampleLen) + slotBDataOffsetInSample;
      memcpy(&payload[payloadLen], &a3RawDataPool[sourceOffset32], slotBDataLen);
      payloadLen += slotBDataLen;
      slotBSectionBytes += slotBDataLen;
    }

    uint8_t slotBPaddingByteIndex = 0;
    while (slotBSectionBytes < slotBTargetSectionBytes) {
      payload[payloadLen++] = (slotBPaddingByteIndex % 2 == 0) ? 0xFF : 0xFE;
      slotBPaddingByteIndex++;
      slotBSectionBytes++;
    }
  }

  payload[payloadLen++] = totalTransmitPages;
  payload[payloadLen++] = (uint8_t)batchPageNumber;

  if (payloadLen > A3_TX_PAGE_MAX_SIZE) {
    DEBUG_SERIAL.print(F("A3 generated full-page TX batch too large. Bytes: "));
    DEBUG_SERIAL.print(payloadLen);
    DEBUG_SERIAL.print(F(" / Max: "));
    DEBUG_SERIAL.println(A3_TX_PAGE_MAX_SIZE);
    if (a3FinalStatus == A3_FINAL_OK) {
      a3FinalStatus = A3_FINAL_PAGE_TRUNCATED;
    }
    payloadLen = 0;
    return false;
  }

  batchSampleCount = (uint8_t)samplesToSend;
  return true;
}

void startA3Log(uint16_t recordId, uint8_t slotA, uint8_t slotB, uint8_t captureRepeatCount) {
  a3RecordId = recordId;
  a3CaptureRepeatTotal = captureRepeatCount;
  a3CaptureCycleCurrent = 1;
  a3SelectedSlots[0] = slotA;
  a3SelectedSlots[1] = slotB;
  a3SelectedSlotCount = countA3SelectedSlots(slotA, slotB);

  resetA3CaptureCycleRuntime();
  buildA3CommandPayload(a3OriginalCommandPayload);
  a3OriginalCommandLen = A3_COMMAND_PAYLOAD_LEN;

  setA3LedActive();

  DEBUG_SERIAL.println(F("A3 raw-data capture started"));
  DEBUG_SERIAL.print(F("A3 Record ID: 0x"));
  DEBUG_SERIAL.println(a3RecordId, HEX);

  DEBUG_SERIAL.print(F("A3 selected slot A: "));
  if (a3SelectedSlots[0] == A3_SELECTED_SLOT_UNUSED) {
    DEBUG_SERIAL.println(F("FF"));
  } else {
    DEBUG_SERIAL.println(a3SelectedSlots[0]);
  }

  DEBUG_SERIAL.print(F("A3 selected slot B: "));
  if (a3SelectedSlots[1] == A3_SELECTED_SLOT_UNUSED) {
    DEBUG_SERIAL.println(F("FF"));
  } else {
    DEBUG_SERIAL.println(a3SelectedSlots[1]);
  }

  DEBUG_SERIAL.print(F("A3 selected slot count: "));
  DEBUG_SERIAL.println(a3SelectedSlotCount);
  DEBUG_SERIAL.print(F("A3 capture repeat count: "));
  DEBUG_SERIAL.println(a3CaptureRepeatTotal);
  DEBUG_SERIAL.print(F("A3 current capture/send cycle: "));
  DEBUG_SERIAL.print(a3CaptureCycleCurrent);
  DEBUG_SERIAL.print(F(" / "));
  DEBUG_SERIAL.println(a3CaptureRepeatTotal);
  DEBUG_SERIAL.print(F("A3 slot A values per page capacity: "));
  DEBUG_SERIAL.println(getA3SlotSamplesPerBatch(a3SelectedSlots[0]));
  DEBUG_SERIAL.print(F("A3 slot B values per page capacity: "));
  if (a3SelectedSlots[1] == A3_SELECTED_SLOT_UNUSED) {
    DEBUG_SERIAL.println(F("FF"));
  } else {
    DEBUG_SERIAL.println(getA3SlotSamplesPerBatch(a3SelectedSlots[1]));
  }
  DEBUG_SERIAL.print(F("A3 aligned values per LoRaWAN page: "));
  DEBUG_SERIAL.println(getA3MaxSamplesPerBatch());
  DEBUG_SERIAL.print(F("A3 raw bytes per sample: "));
  DEBUG_SERIAL.println(a3RawSampleLen);
  DEBUG_SERIAL.print(F("A3 max samples by raw pool: "));
  DEBUG_SERIAL.println(getA3MaximumRawSamples());
  DEBUG_SERIAL.print(F("A3 raw data pool bytes: "));
  DEBUG_SERIAL.println(A3_RAW_DATA_POOL_SIZE);
  DEBUG_SERIAL.print(F("A3 max samples per LoRaWAN batch: "));
  DEBUG_SERIAL.println(getA3MaxSamplesPerBatch());
  DEBUG_SERIAL.print(F("A3 maximum TX batch bytes: "));
  DEBUG_SERIAL.println(getA3FixedBatchLen());
  DEBUG_SERIAL.print(F("A3 maximum TX batch HEX chars: "));
  DEBUG_SERIAL.println(getA3FixedBatchLen() * 2U);
  DEBUG_SERIAL.println(F("A3 will capture until the raw SRAM pool is full or a selected device fails after 2 tries"));

  if (a3SelectedSlotCount == 0 || a3RawSampleLen == 0) {
    DEBUG_SERIAL.println(F("A3 cannot start because no valid slots were selected"));
    a3FinalStatus = A3_FINAL_NO_ACTIVE_SLOTS;
    sendA3NoConfiguredSlotMarker();
    a3State = A3_IDLE;
    setA3LedIdle();
    return;
  }

  if (getA3MaxSamplesPerBatch() == 0) {
    DEBUG_SERIAL.println(F("A3 cannot start because batch TX header would be too large"));
    a3FinalStatus = A3_FINAL_PAGE_TRUNCATED;
    a3State = A3_SENDING_FINAL;
    return;
  }

  // The accepted A3 command status echo is sent by the A3 state machine
  // at the start of every capture/page set.

  a3State = A3_SENDING_CYCLE_START_ECHO;
  lastUplink = millis();
}

void handleA3LogTask() {
  if (a3State == A3_IDLE) {
    return;
  }

  if (a3State == A3_SENDING_CYCLE_START_ECHO) {
    DEBUG_SERIAL.println(F("A3 sending cycle start 0042 echo."));

    if (!sendA3CycleStatusEcho(F("A3 cycle start 0042 echo"))) {
      DEBUG_SERIAL.println(F("A3 cycle start echo not sent yet. Will retry."));
      return;
    }

    DEBUG_SERIAL.println(F("A3 cycle start echo sent. Starting capture."));
    a3State = A3_CAPTURING;
    lastUplink = millis();
    return;
  }

  if (a3State == A3_CAPTURING) {
    if (!buildAndStoreA3RawSample()) {
      DEBUG_SERIAL.println(F("A3 capture finished, preparing stored raw samples"));

      a3SendIndex = 0;
      a3SendPageNumber = 1;
      a3CurrentBatchRetryCount = 0;
      a3LastPageUploadMillis = 0;
      a3BreakMarkerLastAttemptMillis = 0;

      if (a3FinalStatus == A3_FINAL_MODBUS_NO_RESPONSE ||
          a3FinalStatus == A3_FINAL_MODBUS_BREAK_MID_CAPTURE) {
        // Do not fire-and-forget I here. If LoRaWAN is busy right after
        // the A3 command echo, stay in this state and retry until I is sent.
        a3State = A3_SENDING_BREAK_MARKER;
      } else if (a3TotalPages > 0) {
        a3State = A3_SENDING_PAGES;
      } else {
        a3State = A3_SENDING_FINAL;
      }
      return;
    }

    return;
  }

  if (a3State == A3_SENDING_BREAK_MARKER) {
    if (a3BreakMarkerLastAttemptMillis != 0 &&
        (millis() - a3BreakMarkerLastAttemptMillis) < A3_BREAK_MARKER_RETRY_DELAY_MS) {
      return;
    }

    if (a3FinalStatus == A3_FINAL_MODBUS_BREAK_MID_CAPTURE) {
      DEBUG_SERIAL.println(F("A3 mid-capture break detected. Sending 004A echo and aborting without pages."));
    } else {
      DEBUG_SERIAL.println(F("A3 Modbus no-response detected before stored samples. Sending 0049 echo and aborting without pages."));
    }

    if (!sendA3ModbusBreakMarker()) {
      a3BreakMarkerLastAttemptMillis = millis();
      if (a3FinalStatus == A3_FINAL_MODBUS_BREAK_MID_CAPTURE) {
        DEBUG_SERIAL.println(F("A3 004A echo not sent yet. Will retry."));
      } else {
        DEBUG_SERIAL.println(F("A3 0049 echo not sent yet. Will retry."));
      }
      return;
    }

    if (a3FinalStatus == A3_FINAL_MODBUS_BREAK_MID_CAPTURE) {
      DEBUG_SERIAL.println(F("A3 004A echo sent. Capture aborted; stored pages will not be transmitted."));
    } else {
      DEBUG_SERIAL.println(F("A3 0049 echo sent. Capture aborted; stored pages will not be transmitted."));
    }
    a3BreakMarkerLastAttemptMillis = 0;
    a3LastPageUploadMillis = 0;
    a3SendIndex = 0;
    a3SendPageNumber = 1;
    a3CurrentBatchRetryCount = 0;
    a3State = A3_SENDING_FINAL;
    lastUplink = millis();
    return;
  }

  if (a3State == A3_SENDING_PAGES) {
    if (a3SendIndex < a3TotalPages) {
      // First data batch sends immediately after capture.
      // Every following page/retry waits at least 15 seconds after the previous attempt.
      if (a3LastPageUploadMillis != 0 && (millis() - a3LastPageUploadMillis) < A3_PAGE_UPLOAD_DELAY_MS) {
        return;
      }

      DEBUG_SERIAL.print(F("A3 sending confirmed stored raw batch page "));
      DEBUG_SERIAL.print(a3SendPageNumber);
      DEBUG_SERIAL.print(F(" from sample "));
      DEBUG_SERIAL.print(a3SendIndex + 1);
      DEBUG_SERIAL.print(F(" / "));
      DEBUG_SERIAL.print(a3TotalPages);
      DEBUG_SERIAL.print(F(" | retry "));
      DEBUG_SERIAL.print(a3CurrentBatchRetryCount);
      DEBUG_SERIAL.print(F(" / "));
      DEBUG_SERIAL.println(A3_CONFIRMED_UPLOAD_MAX_RETRIES);

      uint8_t payload[A3_TX_PAGE_MAX_SIZE];
      size_t payloadLen = 0;
      uint8_t batchSampleCount = 0;

      if (!buildA3TransmitBatchFromRawSamples(a3SendIndex, a3SendPageNumber, payload, payloadLen, batchSampleCount)) {
        DEBUG_SERIAL.println(F("A3 stored raw samples could not be converted to full-page TX batch."));
        a3State = A3_SENDING_FINAL;
        return;
      }

      DEBUG_SERIAL.print(F("A3 generated full-page TX batch bytes: "));
      DEBUG_SERIAL.print(payloadLen);
      DEBUG_SERIAL.print(F(" / HEX chars: "));
      DEBUG_SERIAL.println(payloadLen * 2);
      DEBUG_SERIAL.print(F("A3 generated full-page TX batch: "));
      dumpHex(payload, payloadLen);

      bool a3PageAcked = sendLoRaWanUplinkConfirmed(payload, payloadLen, F("A3 confirmed raw batch"));

      if (!a3PageAcked) {
        DEBUG_SERIAL.println(F("A3 confirmed raw batch not ACKed. Advancing anyway to avoid repeated page number."));
      }

      // Always advance after one send attempt.
      // This prevents repeated A3 page numbers like 1,2,3,4,5,5,6...
      a3SendIndex += batchSampleCount;
      a3SendPageNumber++;
      a3CurrentBatchRetryCount = 0;
      a3LastPageUploadMillis = millis();
      lastUplink = a3LastPageUploadMillis;
      return;
    }

    if (a3TotalPages > 0 && a3FinalStatus != A3_FINAL_UPLOAD_FAILED) {
      a3State = A3_SENDING_CYCLE_FINISH_ECHO;
    } else {
      a3State = A3_SENDING_FINAL;
    }
    return;
  }

  if (a3State == A3_SENDING_CYCLE_FINISH_ECHO) {
    DEBUG_SERIAL.println(F("A3 sending cycle finish 0042 echo."));

    if (!sendA3CycleStatusEcho(F("A3 cycle finish 0042 echo"))) {
      DEBUG_SERIAL.println(F("A3 cycle finish echo not sent yet. Will retry."));
      return;
    }

    DEBUG_SERIAL.println(F("A3 cycle finish echo sent."));
    a3State = A3_SENDING_FINAL;
    lastUplink = millis();
    return;
  }

  if (a3State == A3_SENDING_FINAL) {
    sendA3FinalSummary();
    return;
  }
}

// -----------------------------------------------------------------------------
// Command handling
// -----------------------------------------------------------------------------
// A1 format:
// 000000A1 + 2 bytes uint16 big-endian
//
// A2 clear all:
// 000000A2FFFF
//
// A2 write slot format (legacy):
// 000000A2 + [slotIndex][enable][slaveId][regAddrHi][regAddrLo][regCountHi][regCountLo]
// Stored as channel 1 + FC03
//
// A2 write slot format (new):
// 000000A2 + [slotIndex][enable][channel][slaveId][function][regAddrHi][regAddrLo][argHi][argLo]
//
// D slot config request format:
// 44
//
// D slot config response format, 1 to 3 messages, each fixed to 100 bytes / 200 HEX chars:
// 44 + [uploadIntervalHi][uploadIntervalLo] + [totalSlotCount]
// + up to 12 repeated slot records per message:
// [slotState][channel][slaveId][function][regAddrHi][regAddrLo][argHi][argLo]
// slotState bit7 = enabled, bits0..4 = absolute slot index.
// If the final D/44 message carries fewer slot records, the remaining bytes are 00 padding.
//
// A3 raw-data capture format:
// 000000A3 + [recordIdHi][recordIdLo][slotA][slotB][captureRepeatCount]
//
// Example:
// 000000A3ABCD000103 = Record ID ABCD, capture slot 0 and slot 1, run 3 full capture/send cycles.
// 000000A3ABCD00FF01 = Record ID ABCD, capture slot 0 only, run 1 full capture/send cycle.
//
// For every capture/page cycle, A3 sends 00 42 + the full A3 command before capture starts.
// Then it captures only raw Modbus response data from the selected slot(s) into RAM.
// It does NOT transmit samples during capture.
// Capture stops when A3_RAW_DATA_POOL_SIZE is full, or when one selected
// Modbus device fails to answer correctly after 2 tries.
// During transmission, the program sends confirmed full-page A3 batches:
// [slotA][slotB]
// + 2 bytes most-repeated Slot A response time + Slot A full-page data section
// + 2 bytes most-repeated Slot B response time + Slot B full-page data section, only when Slot B is used
// + [totalTxPages][currentTxPage]
// Only the final short page is filled with FFFE repeated so it matches the full-page length. Full pages are not padded.
// Each page is advanced only after confirmed uplink success.
// If a page is not confirmed, the same page is retried 2 times only, with 15 seconds between attempts.
// After each A3 data-page set finishes sending, A3 sends 00 42 + the full A3 command
// as the finish marker. Then the next capture cycle starts automatically until
// captureRepeatCount cycles are completed.
// If slot A or slot B fails after 2 Modbus tries at any time during capture,
// A3 sends 00 49 + the original A3 command echo when no complete sample was stored,
// or 00 4A + the original A3 command echo when capture breaks mid-session.
// No stored A3 pages and no final normal 00 42 finish echo are sent after that failure.

void handleCommandA1(const uint8_t* data, size_t len) {
  if (len != 6) {
    DEBUG_SERIAL.print(F("CMD A1 invalid size: "));
    DEBUG_SERIAL.println(len);
    sendInvalidCommandMarker();
    return;
  }

  uint16_t newInterval = ((uint16_t)data[4] << 8) | data[5];

  DEBUG_SERIAL.print(F("CMD A1 interval request: "));
  DEBUG_SERIAL.print(newInterval);
  DEBUG_SERIAL.println(F(" seconds"));

  if (!isValidUploadInterval(newInterval)) {
    DEBUG_SERIAL.println(F("CMD A1 invalid value"));
    sendInvalidCommandMarker();
    return;
  }

  if (newInterval == gConfig.uploadIntervalSeconds) {
    DEBUG_SERIAL.println(F("CMD A1 same value, no change"));
    uploadIntervalSeconds = newInterval;
    sendCommandAppliedAck();
    return;
  }

  PersistedConfig newCfg = gConfig;
  newCfg.uploadIntervalSeconds = newInterval;
  saveConfigIfChanged(newCfg);

  lastUplink = millis();

  DEBUG_SERIAL.print(F("New upload interval: "));
  DEBUG_SERIAL.print(uploadIntervalSeconds);
  DEBUG_SERIAL.println(F(" seconds"));

  sendCommandAppliedAck();
}

void handleCommandA2(const uint8_t* data, size_t len) {
  if (len == 6 && data[4] == 0xFF && data[5] == 0xFF) {
    DEBUG_SERIAL.println(F("CMD A2 clear all slots"));

    PersistedConfig newCfg = gConfig;
    for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
      clearSlot(newCfg.slots[i]);
    }

    saveConfigIfChanged(newCfg);
    printConfigAtBoot();
    sendCommandAppliedAck();
    return;
  }

  uint8_t  slotIndex = 0;
  uint8_t  enable    = 0;
  uint8_t  channel   = FIXED_CHANNEL;
  uint8_t  slaveId   = 0;
  uint8_t  function  = 0x03;
  uint16_t regAddr   = 0;
  uint16_t argument  = 0;

  if (len == 11) {
    slotIndex = data[4];
    enable    = data[5];
    channel   = FIXED_CHANNEL;
    slaveId   = data[6];
    function  = 0x03;
    regAddr   = ((uint16_t)data[7] << 8) | data[8];
    argument  = ((uint16_t)data[9] << 8) | data[10];
    DEBUG_SERIAL.println(F("CMD A2 legacy format detected -> CH1 + FC03"));
  } else if (len == 13) {
    slotIndex = data[4];
    enable    = data[5];
    channel   = data[6];
    slaveId   = data[7];
    function  = data[8];
    regAddr   = ((uint16_t)data[9] << 8) | data[10];
    argument  = ((uint16_t)data[11] << 8) | data[12];
  } else {
    DEBUG_SERIAL.print(F("CMD A2 invalid size: "));
    DEBUG_SERIAL.println(len);
    sendInvalidCommandMarker();
    return;
  }

  DEBUG_SERIAL.print(F("CMD A2 slot: "));
  DEBUG_SERIAL.println(slotIndex);

  if (enable > 1) {
    DEBUG_SERIAL.println(F("CMD A2 invalid enable value"));
    sendInvalidCommandMarker();
    return;
  }

  if (enable == 0) {
    if (slotIndex >= MAX_POLL_SLOTS) {
      DEBUG_SERIAL.println(F("CMD A2 invalid slot index"));
      sendInvalidCommandMarker();
      return;
    }

    PersistedConfig newCfg = gConfig;
    clearSlot(newCfg.slots[slotIndex]);

    DEBUG_SERIAL.print(F("CMD A2 cleared slot "));
    DEBUG_SERIAL.println(slotIndex);

    saveConfigIfChanged(newCfg);
    printConfigAtBoot();
    sendCommandAppliedAck();
    return;
  }

  if (!isValidChannel(channel)) {
    DEBUG_SERIAL.println(F("CMD A2 invalid channel"));
    sendInvalidCommandMarker();
    return;
  }

  if (!isValidSlaveId(slaveId)) {
    DEBUG_SERIAL.println(F("CMD A2 invalid slave id"));
    sendInvalidCommandMarker();
    return;
  }

  if (!isSupportedSlotFunction(function)) {
    DEBUG_SERIAL.println(F("CMD A2 invalid function"));
    sendInvalidCommandMarker();
    return;
  }

  if (!isValidSlotArgument(function, argument)) {
    DEBUG_SERIAL.println(F("CMD A2 invalid argument for function"));
    sendInvalidCommandMarker();
    return;
  }

  // FC05 / FC06 = one-shot write only. A matching FC06 output request
  // supplies its OWN raw setpoint and triggers the stored A6 power sequence.
  // Never send that FC06 directly while the output module is unpowered.
  if (isWriteFunction(function)) {
    if (function == 0x06 && a6MatchesOutput(slaveId, function, regAddr)) {
      a6QueueSetpoint(argument);
    } else {
      // Non-A6 writes retain original A2 behavior and addressing.
      executeOneShotWrite(channel, slaveId, function, regAddr, argument);
    }
    sendCommandAppliedAck();
    return;
  }

  // FC01 / FC02 / FC03 / FC04 = save to slot
  if (slotIndex >= MAX_POLL_SLOTS) {
    DEBUG_SERIAL.println(F("CMD A2 invalid slot index"));
    sendInvalidCommandMarker();
    return;
  }

  PersistedConfig newCfg = gConfig;
  newCfg.slots[slotIndex].enabled    = 1;
  newCfg.slots[slotIndex].channel    = channel;
  newCfg.slots[slotIndex].slaveId    = slaveId;
  newCfg.slots[slotIndex].function   = function;
  newCfg.slots[slotIndex].regAddress = regAddr;
  newCfg.slots[slotIndex].argument   = argument;

  DEBUG_SERIAL.print(F("CMD A2 set slot "));
  DEBUG_SERIAL.print(slotIndex);
  DEBUG_SERIAL.print(F(" -> CH "));
  DEBUG_SERIAL.print(channel);
  DEBUG_SERIAL.print(F(", Slave "));
  DEBUG_SERIAL.print(slaveId);
  DEBUG_SERIAL.print(F(", "));
  DEBUG_SERIAL.print(functionName(function));
  DEBUG_SERIAL.print(F(", Addr "));
  DEBUG_SERIAL.print(regAddr);
  DEBUG_SERIAL.print(F(", Arg "));
  DEBUG_SERIAL.println(argument);

  saveConfigIfChanged(newCfg);
  printConfigAtBoot();
  sendCommandAppliedAck();
}


void handleCommandA3(const uint8_t* data, size_t len) {
  if (len != A3_COMMAND_PAYLOAD_LEN) {
    DEBUG_SERIAL.print(F("CMD A3 invalid size: "));
    DEBUG_SERIAL.println(len);
    sendInvalidCommandMarker();
    return;
  }

  uint16_t recordId = ((uint16_t)data[4] << 8) | data[5];
  uint8_t slotA = data[6];
  uint8_t slotB = data[7];
  uint8_t captureRepeatCount = data[8];

  DEBUG_SERIAL.print(F("CMD A3 record id: 0x"));
  DEBUG_SERIAL.println(recordId, HEX);

  DEBUG_SERIAL.print(F("CMD A3 slot A: "));
  if (slotA == A3_SELECTED_SLOT_UNUSED) {
    DEBUG_SERIAL.println(F("FF"));
  } else {
    DEBUG_SERIAL.println(slotA);
  }

  DEBUG_SERIAL.print(F("CMD A3 slot B: "));
  if (slotB == A3_SELECTED_SLOT_UNUSED) {
    DEBUG_SERIAL.println(F("FF"));
  } else {
    DEBUG_SERIAL.println(slotB);
  }

  DEBUG_SERIAL.print(F("CMD A3 capture repeat count: "));
  DEBUG_SERIAL.println(captureRepeatCount);

  DEBUG_SERIAL.println(F("CMD A3 captures only selected slot(s) and stops when the raw SRAM pool is full"));

  DEBUG_SERIAL.print(F("CMD A3 raw data pool bytes: "));
  DEBUG_SERIAL.println(A3_RAW_DATA_POOL_SIZE);

  if (captureRepeatCount == 0) {
    DEBUG_SERIAL.println(F("CMD A3 invalid capture repeat count: 0"));
    sendInvalidCommandMarker();
    return;
  }

  if (!validateA3SelectedSlots(slotA, slotB)) {
    DEBUG_SERIAL.println(F("CMD A3 rejected because no configured/valid selected slot is available"));
    sendInvalidCommandMarker();
    sendA3NoConfiguredSlotMarker();
    return;
  }

  if (isA3LogBusy()) {
    DEBUG_SERIAL.println(F("CMD A3 previous session interrupted by new A3 command"));
  }

  startA3Log(recordId, slotA, slotB, captureRepeatCount);
}

void handleCommandA4(const uint8_t* data, size_t len) {
  // A4 format for FC16 Write Multiple Registers:
  // 000000A4 + [channel][slaveId][startAddrHi][startAddrLo][regCount]
  // + repeated [valueHi][valueLo]
  //
  // Example structure:
  // 000000A4 01 01 012C 02 03ED 0001
  // Means: CH1, Slave 1, Start 300, write 2 registers: 1005, 1

  if (len < 11) {
    DEBUG_SERIAL.print(F("CMD A4 invalid size: "));
    DEBUG_SERIAL.println(len);
    sendInvalidCommandMarker();
    return;
  }

  uint8_t channel = data[4];
  uint8_t slaveId = data[5];
  uint16_t startAddr = ((uint16_t)data[6] << 8) | data[7];
  uint8_t regCount = data[8];

  size_t expectedLen = 9U + ((size_t)regCount * 2U);

  if (len != expectedLen) {
    DEBUG_SERIAL.print(F("CMD A4 invalid size. Expected "));
    DEBUG_SERIAL.print(expectedLen);
    DEBUG_SERIAL.print(F(", received "));
    DEBUG_SERIAL.println(len);
    sendInvalidCommandMarker();
    return;
  }

  if (regCount == 0 || regCount > MAX_FC16_REGS_PER_COMMAND) {
    DEBUG_SERIAL.println(F("CMD A4 invalid register count"));
    sendInvalidCommandMarker();
    return;
  }

  uint16_t values[MAX_FC16_REGS_PER_COMMAND];

  uint8_t pos = 9;
  for (uint8_t i = 0; i < regCount; i++) {
    values[i] = ((uint16_t)data[pos] << 8) | data[pos + 1];
    pos += 2;
  }

  DEBUG_SERIAL.print(F("CMD A4 FC16 request -> CH "));
  DEBUG_SERIAL.print(channel);
  DEBUG_SERIAL.print(F(", Slave "));
  DEBUG_SERIAL.print(slaveId);
  DEBUG_SERIAL.print(F(", Start Addr "));
  DEBUG_SERIAL.print(startAddr);
  DEBUG_SERIAL.print(F(", Count "));
  DEBUG_SERIAL.println(regCount);

  if (!isValidChannel(channel)) {
    DEBUG_SERIAL.println(F("CMD A4 invalid channel"));
    sendInvalidCommandMarker();
    return;
  }

  if (!isValidSlaveId(slaveId)) {
    DEBUG_SERIAL.println(F("CMD A4 invalid slave id"));
    sendInvalidCommandMarker();
    return;
  }

  if (regCount == 1 && a6MatchesOutput(slaveId, 0x10, startAddr)) {
    // A4 single-register FC16 can trigger an FC16-configured A6 sequence.
    a6QueueSetpoint(values[0]);
  } else {
    executeWriteMultipleRegisters(channel, slaveId, startAddr, values, regCount);
  }
  // Accepted command status is NOT a physical Modbus execution confirmation.
  sendCommandAppliedAck();
}

// A5: 000000A5 [slot][enable][channel][slave][coilHi][coilLo][secHi][secLo]
// Emergency disable-all: 000000A5FFFF -> all distinct configured coils OFF,
// then disable all A5 mappings (A2 polling slots remain untouched).
// A5 regular configuration keeps the original 12-byte command/EEPROM format.
void handleCommandA5(const uint8_t* data, size_t len) {
  if (len != 12) {
    DEBUG_SERIAL.println(F("CMD A5 invalid length"));
    sendInvalidCommandMarker();
    return;
  }

  uint8_t slot = data[4];
  uint8_t enable = data[5];
  uint8_t channel = data[6];
  uint8_t slave = data[7];
  uint16_t coilAddr = ((uint16_t)data[8] << 8) | data[9];
  uint16_t seconds = ((uint16_t)data[10] << 8) | data[11];
  if (slot >= MAX_POLL_SLOTS || enable > 1 ||
      (enable && (channel != FIXED_CHANNEL || !isValidSlaveId(slave) ||
                  seconds < A5_MIN_STABILIZATION_SECONDS))) {
    DEBUG_SERIAL.println(F("CMD A5 invalid slot/channel/slave/delay (<15 sec)"));
    sendInvalidCommandMarker();
    return;
  }

  int addr = a5SlotAddress(slot);
  uint8_t targetSlave = enable ? slave : 0;
  uint16_t targetAddr = enable ? coilAddr : 0;
  uint16_t targetSeconds = enable ? seconds : 0;
  bool changed = EEPROM.read(addr) != targetSlave ||
                 readU16LE(addr + 1) != targetAddr ||
                 readU16LE(addr + 3) != targetSeconds;
  if (changed) {
    a5WriteSlot(slot, targetSlave, targetAddr, targetSeconds);
    a5SaveCrc();
    DEBUG_SERIAL.println(F("A5 power control: EEPROM updated"));
  } else {
    DEBUG_SERIAL.println(F("A5 power control: EEPROM unchanged"));
  }

  DEBUG_SERIAL.print(F("CMD A5 slot "));
  DEBUG_SERIAL.print(slot);
  if (enable) {
    DEBUG_SERIAL.print(F(" enabled -> CH 1, Coil Slave "));
    DEBUG_SERIAL.print(slave);
    DEBUG_SERIAL.print(F(", Addr "));
    DEBUG_SERIAL.print(coilAddr);
    DEBUG_SERIAL.print(F(", Stabilization "));
    DEBUG_SERIAL.print(seconds);
    DEBUG_SERIAL.println(F(" seconds"));
  } else {
    DEBUG_SERIAL.println(F(" disabled"));
  }
  sendCommandAppliedAck();
}

// All recognized commands have EXACTLY the same dispatch priority. Validate
// the COMPLETE command before touching ongoing acquisition, EEPROM or relays.
ValidCommandKind validateIncomingCommand(const uint8_t* data, size_t len) {
  if (data == NULL) return VALID_CMD_NONE;
  if (len == 1 && data[0] == SLOT_CONFIG_PAYLOAD_ID) return VALID_CMD_D;
  if (len < 4 || data[0] != 0 || data[1] != 0 || data[2] != 0) return VALID_CMD_NONE;
  switch (data[3]) {
    case 0xA1: {
      if (len != 6) return VALID_CMD_NONE;
      uint16_t seconds = ((uint16_t)data[4] << 8) | data[5];
      return isValidUploadInterval(seconds) ? VALID_CMD_A1 : VALID_CMD_NONE;
    }
    case 0xA2: {
      if (len == 6 && data[4] == 0xFF && data[5] == 0xFF) return VALID_CMD_A2;
      if (len != 11 && len != 13) return VALID_CMD_NONE;
      uint8_t slot = data[4], enable = data[5];
      if (enable > 1 || (enable == 0 && slot >= MAX_POLL_SLOTS)) return VALID_CMD_NONE;
      if (enable == 0) return VALID_CMD_A2;
      uint8_t channel = (len == 13) ? data[6] : FIXED_CHANNEL;
      uint8_t slave = (len == 13) ? data[7] : data[6];
      uint8_t fc = (len == 13) ? data[8] : 0x03;
      uint16_t argument = (len == 13) ?
          (((uint16_t)data[11] << 8) | data[12]) :
          (((uint16_t)data[9] << 8) | data[10]);
      if (!isValidChannel(channel) || !isValidSlaveId(slave) ||
          !isSupportedSlotFunction(fc) || !isValidSlotArgument(fc, argument)) return VALID_CMD_NONE;
      // A6 protects its configured AO output even if a caller tries the wrong
      // write function. Reject BEFORE interrupting; never bypass relay power.
      if (fc == 0x06 && len == 13) {
        uint16_t addr = ((uint16_t)data[9] << 8) | data[10];
        if (a6IsProtectedOutputAddress(slave, addr) &&
            (!a6MatchesOutput(slave, fc, addr) || !a6RawSetpointValid(argument))) {
          return VALID_CMD_NONE;
        }
      }
      // One-shot FC05/06 do not consume an A2 slot (existing behavior).
      return (isWriteFunction(fc) || slot < MAX_POLL_SLOTS) ? VALID_CMD_A2 : VALID_CMD_NONE;
    }
    case 0xA3: {
      if (len != A3_COMMAND_PAYLOAD_LEN || data[8] == 0) return VALID_CMD_NONE;
      return validateA3SelectedSlots(data[6], data[7]) ? VALID_CMD_A3 : VALID_CMD_NONE;
    }
    case 0xA4: {
      if (len < 11) return VALID_CMD_NONE;
      uint8_t count = data[8];
      if (count == 0 || count > MAX_FC16_REGS_PER_COMMAND ||
          len != 9U + (size_t)count * 2U ||
          !isValidChannel(data[4]) || !isValidSlaveId(data[5])) return VALID_CMD_NONE;
      uint8_t a6ProtectedSlave;
      uint16_t target;
      if (a6GetProtectedOutput(a6ProtectedSlave, target) && data[5] == a6ProtectedSlave) {
        uint16_t first = ((uint16_t)data[6] << 8) | data[7];
        uint32_t last = (uint32_t)first + (uint32_t)count - 1UL;
        // Never allow a multi-register FC16 to bypass power-on sequencing
        // by including the configured analog-output register in its span.
        if (first <= target && target <= last) {
          if (first != target || count != 1 ||
              a6Config.aoSetpointFunction != 0x10) return VALID_CMD_NONE;
          uint16_t raw = ((uint16_t)data[9] << 8) | data[10];
          if (!a6RawSetpointValid(raw)) return VALID_CMD_NONE;
        }
      }
      return VALID_CMD_A4;
    }
    case 0xA6: return validateA6Command(data, len) ? VALID_CMD_A6 : VALID_CMD_NONE;
    case 0xA5: {
      if (len == 6 && data[4] == 0xFF && data[5] == 0xFF) return VALID_CMD_A5_EMERGENCY;
      if (len != 12 || data[4] >= MAX_POLL_SLOTS || data[5] > 1) return VALID_CMD_NONE;
      if (data[5] == 1) {
        uint16_t seconds = ((uint16_t)data[10] << 8) | data[11];
        if (data[6] != FIXED_CHANNEL || !isValidSlaveId(data[7]) ||
            seconds < A5_MIN_STABILIZATION_SECONDS) return VALID_CMD_NONE;
      }
      return VALID_CMD_A5;
    }
    default: return VALID_CMD_NONE;
  }
}

// A3 sampling and transmission is cancelled ONLY AFTER another command has
// passed full validation. No incomplete A3 pages or stale capture state remain.
void cancelA3ForValidatedCommand() {
  if (!isA3LogBusy()) return;
  DEBUG_SERIAL.println(F("A3 stopped by another VALID command (equal command priority)"));
  a3State = A3_IDLE;
  a3CaptureCycleCurrent = 0;
  a3OriginalCommandLen = 0;
  a3TotalPages = 0;
  a3StorageUsed = 0;
  a3SendIndex = 0;
  setA3LedIdle();
  lastUplink = millis();
}

void a6QueueDeferredCommand(const uint8_t* data, size_t len) {
  if (data == NULL || len == 0 || len > LORA_RX_BUFFER_SIZE) {
    DEBUG_SERIAL.println(F("A6: empty/oversized downlink ignored during active sequence"));
    return;
  }
  if (a6DeferredCount == A6_DEFERRED_COMMAND_CAPACITY) {
    DEBUG_SERIAL.println(F("A6: deferred command FIFO full; downlink not accepted"));
    return;
  }
  uint8_t index = (a6DeferredHead + a6DeferredCount) % A6_DEFERRED_COMMAND_CAPACITY;
  memcpy(a6DeferredCommands[index], data, len);
  a6DeferredLengths[index] = (uint8_t)len;
  a6DeferredCount++;
  DEBUG_SERIAL.println(F("Relay sequence busy: downlink deferred until operation completes"));
}

void a6ProcessDeferredCommand() {
  if (isA6SequenceBusy() || isA2RelayOnBusy() || a6PendingStart ||
      a5RegularPollInProgress || a5PowerSequenceActive ||
      rejoinRequested || a6DeferredCount == 0) return;
  uint8_t payload[LORA_RX_BUFFER_SIZE];
  uint8_t length = a6DeferredLengths[a6DeferredHead];
  memcpy(payload, a6DeferredCommands[a6DeferredHead], length);
  a6DeferredHead = (a6DeferredHead + 1U) % A6_DEFERRED_COMMAND_CAPACITY;
  a6DeferredCount--;
  // The standard dispatcher processes this command using its ORIGINAL rules.
  processDownlinkCommand(payload, length);
  setCurrentCommandEchoSource(NULL, 0); // Do not retain a pointer to stack RAM.
}

void processDownlinkCommand(const uint8_t* data, size_t len) {
  if (isA6SequenceBusy()) {
    a6QueueDeferredCommand(data, len);
    return;
  }
  // Ordinary commands may not abort an A5 relay sequence or its acquisition.
  // Preserve the original A5FFFF emergency OFF operation as an explicit stop.
  const bool emergencyA5 = data != NULL && len == 6 &&
       data[0] == 0 && data[1] == 0 && data[2] == 0 &&
       data[3] == 0xA5 && data[4] == 0xFF && data[5] == 0xFF;
  if ((a5PowerSequenceActive || a5RegularPollInProgress || a6PendingStart ||
       isA2RelayOnBusy()) && !emergencyA5) {
    a6QueueDeferredCommand(data, len);
    return;
  }
  if (emergencyA5 && a2RelayOnActive) {
    // Explicit A5FFFF emergency takes priority over a direct ON repeat.
    a2RelayOnActive = false;
    DEBUG_SERIAL.println(F("A2 pending ON repeats stopped by explicit A5FFFF emergency"));
  }
  setCurrentCommandEchoSource(data, len);
  ValidCommandKind kind = validateIncomingCommand(data, len);
  if (kind == VALID_CMD_NONE) {
    DEBUG_SERIAL.println(F("Command structurally invalid: no operation interrupted"));
    sendInvalidCommandMarker();
    // Preserve the original A3 missing/invalid-slot H notification. No state
    // changes are made for this invalid command.
    if (len == A3_COMMAND_PAYLOAD_LEN && data != NULL &&
        data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 0xA3 &&
        data[8] != 0) sendA3NoConfiguredSlotMarker();
    return;
  }

  // Normal commands retain original processing when no relay operation is active.
  a6PendingStart = false;

  // During A5, only the explicit A5FFFF emergency reaches this branch.
  if (a5PowerSequenceActive || a5RegularPollInProgress) {
    a5PollCancelled = true;
    DEBUG_SERIAL.println(F("A5 STOP POLLING: validated command; no further sensor reads"));
    // Normal valid commands stop ONLY the currently energized relay. A5FFFF
    // performs its own all-mapped-coils emergency OFF instead.
    if (kind != VALID_CMD_A5_EMERGENCY &&
        a5PowerSequenceActive && a5ActiveRelayMayBeOn) {
      uint8_t response[4];
      bool offSent = turnA5PowerOff(a5ActiveRelaySlave, a5ActiveRelayAddr, response);
      if (offSent) {
        a5InterruptionOffSent = true;
        a5ActiveRelayMayBeOn = false;
      } else {
        setA5SafetyLockout(true);
        DEBUG_SERIAL.println(F("A5 active relay OFF dispatch failed; valid command still accepted"));
      }
    }
  }

  // D is a read-only configuration request: it can run while A3 retains its
  // existing capture state. State-changing valid commands replace A3 work.
  if (kind != VALID_CMD_D) cancelA3ForValidatedCommand();

  switch (kind) {
    case VALID_CMD_A1: handleCommandA1(data, len); return;
    case VALID_CMD_A2: handleCommandA2(data, len); return;
    case VALID_CMD_A3: handleCommandA3(data, len); return;
    case VALID_CMD_A4: handleCommandA4(data, len); return;
    case VALID_CMD_A5: handleCommandA5(data, len); return;
    case VALID_CMD_A6: handleCommandA6(data, len); return;
    case VALID_CMD_A5_EMERGENCY: {
      bool allOffSent = handleA5DisableAll();
      if (a5PowerSequenceActive && allOffSent) {
        a5InterruptionOffSent = true;
        a5ActiveRelayMayBeOn = false;
      }
      return;
    }
    case VALID_CMD_D:
      DEBUG_SERIAL.println(F("CMD D slot configuration request"));
      sendSlotConfigUplink(F("D slot config response"));
      return;
    default: return; // Impossible: validation above rejects unknown commands.
  }
}

void printDownlinkParsed(const uint8_t* data, size_t len) {
  DEBUG_SERIAL.println(F("Received Class C downlink"));

  if (len == 0) {
    DEBUG_SERIAL.println(F("Downlink empty"));
    return;
  }

  DEBUG_SERIAL.print(F("Downlink bytes: "));
  dumpHex(data, len);

  DEBUG_SERIAL.print(F("Downlink HEX: "));
  for (size_t i = 0; i < len; i++) {
    printHexByte(data[i]);
  }
  DEBUG_SERIAL.println();

  if (len >= 4) {
    DEBUG_SERIAL.print(F("CMD HEADER: "));
    for (uint8_t i = 0; i < 4; i++) {
      printHexByte(data[i]);
    }
    DEBUG_SERIAL.println();

    DEBUG_SERIAL.print(F("CMD VALUE : "));
    if (len > 4) {
      for (size_t i = 4; i < len; i++) {
        printHexByte(data[i]);
      }
      DEBUG_SERIAL.println();
    } else {
      DEBUG_SERIAL.println(F("(empty)"));
    }
  }
}

// A5 shared-coil helpers. Config remains EEPROM-backed, never duplicated in RAM.
// Only enabled A2 reading slots participate in a power group. A group is keyed
// by the physical FC05 slave + coil address on this IOCARD's fixed channel 1.
void findA5SharedCoilSpan(uint8_t coilSlave, uint16_t coilAddr,
                          uint8_t& firstSlot, uint8_t& lastSlot,
                          uint16_t& longestStabilization) {
  firstSlot = MAX_POLL_SLOTS;
  lastSlot = 0;
  longestStabilization = A5_MIN_STABILIZATION_SECONDS;
  for (uint8_t j = 0; j < MAX_POLL_SLOTS; j++) {
    if (!gConfig.slots[j].enabled) continue;
    uint8_t otherSlave;
    uint16_t otherAddr, otherSeconds;
    if (!readA5Slot(j, otherSlave, otherAddr, otherSeconds)) continue;
    if (otherSlave != coilSlave || otherAddr != coilAddr) continue;
    if (firstSlot == MAX_POLL_SLOTS) firstSlot = j;
    lastSlot = j;
    if (otherSeconds > longestStabilization) longestStabilization = otherSeconds;
  }
}

// The existing regular uplink payload also stores the last successful reading
// for every slot. No second data cache or per-slot SRAM array is allocated.
// The payload layout is the same original [00 00][slot][FC][len][data]...[00 00].
uint8_t* findRegularSlotData(uint8_t* payload, size_t payloadLen,
                             uint8_t slot, uint8_t expectedLen) {
  if (payloadLen < 4) return NULL;
  const size_t entriesEnd = payloadLen - 2;
  size_t pos = 2;
  while ((pos + 3U) <= entriesEnd) {
    uint8_t length = payload[pos + 2];
    if (pos + 3U + length > entriesEnd) return NULL;
    if (payload[pos] == slot) {
      return length == expectedLen ? &payload[pos + 3] : NULL;
    }
    pos += 3U + length;
  }
  return NULL;
}

// A5 ONLY: poll solely during the T-5 second read window, never while
// stabilizing and never in an extra post-window interval.
// One sweep roughly every second (best effort); RS485 transactions never overlap.
// The existing regular uplink buffer retains the LAST VALID response per slot.
// Failed reads cannot erase a prior valid value.
void pollA5SharedCoilWindow(uint8_t coilSlave, uint16_t coilAddr,
                            uint32_t windowDurationMs,
                            uint8_t* payload, size_t payloadLen) {
  const uint32_t windowStartMs = millis();
  while (!a5PollCancelled &&
         (millis() - windowStartMs) < windowDurationMs) {
    const uint32_t sweepStartMs = millis();
    // Poll all dependent A2 slots, including nonadjacent slots and separate
    // reading slave IDs, while using only the established ModbusMaster.
    for (uint8_t j = 0; j < MAX_POLL_SLOTS; j++) {
      if (a5PollCancelled ||
          (millis() - windowStartMs) >= windowDurationMs) break;
      if (!gConfig.slots[j].enabled) continue;

      uint8_t otherSlave;
      uint16_t otherAddr, otherSeconds;
      if (!readA5Slot(j, otherSlave, otherAddr, otherSeconds) ||
          otherSlave != coilSlave || otherAddr != coilAddr) continue;

      const uint8_t expectedLen = expectedUnifiedSlotDataLen(gConfig.slots[j]);
      if (expectedLen == 0) continue;
      uint8_t* savedData = findRegularSlotData(payload, payloadLen, j, expectedLen);
      uint8_t responseData[2 * MAX_REGS_PER_SLOT];
      uint8_t responseLen = 0;
      bool readOK = executePollSlot(gConfig.slots[j], responseData, responseLen);
      if (readOK && responseLen == expectedLen) {
        if (savedData != NULL) memcpy(savedData, responseData, expectedLen);
        DEBUG_SERIAL.print(F("A5 slot "));
        DEBUG_SERIAL.print(j);
        DEBUG_SERIAL.print(F(" latest valid response: "));
        dumpHex(responseData, responseLen);
      } else {
        DEBUG_SERIAL.print(F("A5 slot "));
        DEBUG_SERIAL.print(j);
        DEBUG_SERIAL.println(F(" no valid reply; keeping previous valid value"));
      }
      // Class C reception is served between transactions, not during a read.
      handleClassCDownlink();
    }

    while (!a5PollCancelled &&
           (millis() - windowStartMs) < windowDurationMs &&
           (millis() - sweepStartMs) < A5_POLL_INTERVAL_MS) {
      handleClassCDownlink();
      delay(10);
      yield();
    }
  }
}

// A5 sequence: 3 FC05 ON commands, 3 seconds apart -> wait T -> poll T-5 ->
// STOP POLLING (finish any in-flight Modbus read) -> original FC05 OFF.
// T remains the configured stabilization duration, beginning after the third ON.
// Shared-coil dependencies and normal (A5-disabled) polling are unchanged.
void sampleA5SharedCoil(uint8_t coilSlave, uint16_t coilAddr,
                        uint16_t stabilizationSeconds,
                        uint8_t* payload, size_t payloadLen,
                        bool& powerOffFailed) {
  uint8_t powerResponse[4];
  powerOffFailed = false;
  a5PowerSequenceActive = true;
  a5InterruptionOffSent = false;
  a5ActiveRelaySlave = coilSlave;
  a5ActiveRelayAddr = coilAddr;
  a5ActiveRelayMayBeOn = false;
  // All accepted A5 timings are at least 15 seconds; subtracting 5 is safe.
  const uint32_t pollingMs =
      ((uint32_t)stabilizationSeconds - A5_POLL_STOP_EARLY_SECONDS) * 1000UL;

  DEBUG_SERIAL.print(F("A5 coil Slave "));
  DEBUG_SERIAL.print(coilSlave);
  DEBUG_SERIAL.print(F(" Addr "));
  DEBUG_SERIAL.print(coilAddr);
  DEBUG_SERIAL.print(F(" | startup wait "));
  DEBUG_SERIAL.print(stabilizationSeconds);
  DEBUG_SERIAL.print(F("s; polling "));
  DEBUG_SERIAL.print(stabilizationSeconds - A5_POLL_STOP_EARLY_SECONDS);
  DEBUG_SERIAL.println(F("s; STOP POLLING then FC05 OFF"));

  if (!a5PollCancelled) {
    // Three ON passes; neither FC01 nor local UART result decides progression.
    a5ActiveRelayMayBeOn = true;
    for (uint8_t pass = 0; pass < 3; pass++) {
      if (pass > 0 && !waitA5Stabilization(3)) break; // A5FFFF emergency only.
      if (a5PollCancelled) break;
      bool sent = writeA5PowerCoil(coilSlave, coilAddr, true, powerResponse);
      DEBUG_SERIAL.print(F("A5 relay ON pass "));
      DEBUG_SERIAL.print(pass + 1);
      DEBUG_SERIAL.println(sent ? F(" TX attempted") : F(" local TX failure; continuing"));
    }
    if (!a5PollCancelled && waitA5Stabilization(stabilizationSeconds)) {
      // No FC01 relay read: begin the original T-5 sensor acquisition window.
      DEBUG_SERIAL.println(F("A5 START POLLING: associated A2 slots only"));
      pollA5SharedCoilWindow(coilSlave, coilAddr, pollingMs, payload, payloadLen);
    }
  }

  // Explicit STOP barrier. pollA5SharedCoilWindow has returned, so any
  // in-flight Modbus read is finished and NO new instrument read may start.
  // This is before the current relay's normal FC05 OFF transaction.
  DEBUG_SERIAL.println(F("A5 STOP POLLING: all sensor reads stopped; turning current relay OFF"));

  // Always attempt the original FC05 OFF, regardless of all ON transmissions.
  // If an accepted command interrupted this cycle and already sent OFF,
  // DO NOT write OFF twice or touch another relay group.
  if (a5PollCancelled && a5InterruptionOffSent) {
    DEBUG_SERIAL.println(F("A5 cancelled: active relay OFF already dispatched"));
  } else {
    bool offSent = turnA5PowerOff(coilSlave, coilAddr, powerResponse);
    if (!offSent) {
      DEBUG_SERIAL.println(F("A5 emergency cleanup: retrying local coil OFF transmit"));
      offSent = turnA5PowerOff(coilSlave, coilAddr, powerResponse);
    }
    powerOffFailed = !offSent;
    if (powerOffFailed) setA5SafetyLockout(true);
  }

  a5ActiveRelayMayBeOn = false;
  a5InterruptionOffSent = false;
  a5PowerSequenceActive = false;
}

bool buildRegularTimedPayload(uint8_t* uplinkPayload, size_t payloadSize, size_t& uplinkLen) {
  uplinkLen = 0;

  if (payloadSize < 4) {
    DEBUG_SERIAL.println(F("Regular payload buffer too small"));
    return false;
  }

  DEBUG_SERIAL.println(F("Polling configured Modbus slots..."));

  // Reserve the EXISTING combined regular payload first. Its data fields act
  // as the A5 last-valid-response cache: no additional 20-slot SRAM buffer.
  uplinkPayload[uplinkLen++] = 0x00;
  uplinkPayload[uplinkLen++] = 0x00;
  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    if (!gConfig.slots[i].enabled) continue;
    const uint8_t length = expectedUnifiedSlotDataLen(gConfig.slots[i]);
    if (uplinkLen + 3U + length + 2U > payloadSize) {
      DEBUG_SERIAL.println(F("Uplink buffer full, remaining slots skipped"));
      break;
    }
    uplinkPayload[uplinkLen++] = i;
    uplinkPayload[uplinkLen++] = gConfig.slots[i].function;
    uplinkPayload[uplinkLen++] = length;
    for (uint8_t j = 0; j < length; j++) uplinkPayload[uplinkLen++] = 0xFF;
  }

  uplinkPayload[uplinkLen++] = 0x00;
  uplinkPayload[uplinkLen++] = 0x00;

  // One bit for each coil group's FIRST A2 slot; nothing persisted in SRAM.
  uint32_t completedA5Groups = 0;
  a5PowerSequenceActive = false;

  for (uint8_t i = 0; i < MAX_POLL_SLOTS; i++) {
    if (a5PollCancelled) {
      DEBUG_SERIAL.println(F("A5 validated-command cancellation: do not resume this scheduled poll"));
      return false;
    }
    if (!gConfig.slots[i].enabled) continue;
    const uint8_t expectedLen = expectedUnifiedSlotDataLen(gConfig.slots[i]);
    uint8_t* savedData = findRegularSlotData(uplinkPayload, uplinkLen, i, expectedLen);
    if (savedData == NULL) break; // Original 108-byte regular payload limit.

    uint8_t powerSlave;
    uint16_t powerAddr, stabilizeSeconds;
    if (readA5Slot(i, powerSlave, powerAddr, stabilizeSeconds)) {
      uint8_t groupFirst, groupLast;
      uint16_t groupStabilization;
      findA5SharedCoilSpan(powerSlave, powerAddr, groupFirst,
                            groupLast, groupStabilization);
      const uint32_t bit = 1UL << groupFirst;
      if ((completedA5Groups & bit) == 0) {
        completedA5Groups |= bit;
        // Service any valid commands that arrived during the preceding coil gap.
        handleClassCDownlink();
        if (a5PollCancelled) return false;
        bool powerOffFailed = false;
        sampleA5SharedCoil(powerSlave, powerAddr, groupStabilization,
                           uplinkPayload, uplinkLen, powerOffFailed);
        if (a5PollCancelled) {
          DEBUG_SERIAL.println(F("A5 validated command applied: stop this poll and confirmed retries"));
          return false;
        }
        if (powerOffFailed) DEBUG_SERIAL.println(F("A5 WARNING: OFF local TX failure; next coil group continues"));
      }
      // Dependent slots were already sampled into their reserved data fields.
    } else {
      // Unpowered A2 slots: the original one-read Modbus behavior.
      uint8_t responseData[2 * MAX_REGS_PER_SLOT];
      uint8_t responseLen = 0;
      bool ok = executePollSlot(gConfig.slots[i], responseData, responseLen);
      bool validResponse = ok && responseLen == expectedLen && expectedLen > 0;

      DEBUG_SERIAL.print(F("SLOT "));
      DEBUG_SERIAL.print(i);
      DEBUG_SERIAL.print(F(" | CH "));
      DEBUG_SERIAL.print(gConfig.slots[i].channel);
      DEBUG_SERIAL.print(F(" | Slave "));
      DEBUG_SERIAL.print(gConfig.slots[i].slaveId);
      DEBUG_SERIAL.print(F(" | "));
      DEBUG_SERIAL.print(functionName(gConfig.slots[i].function));
      DEBUG_SERIAL.print(F(" | Addr "));
      DEBUG_SERIAL.print(gConfig.slots[i].regAddress);
      DEBUG_SERIAL.print(F(" | Arg "));
      DEBUG_SERIAL.print(gConfig.slots[i].argument);
      DEBUG_SERIAL.print(F(" | "));
      if (validResponse) {
        DEBUG_SERIAL.print(F("Response: "));
        dumpHex(responseData, responseLen);
        memcpy(savedData, responseData, responseLen);
      } else if (ok && responseLen != expectedLen) {
        DEBUG_SERIAL.print(F("ERROR length mismatch. Expected "));
        DEBUG_SERIAL.print(expectedLen);
        DEBUG_SERIAL.print(F(", received "));
        DEBUG_SERIAL.println(responseLen);
      } else {
        DEBUG_SERIAL.println(F("ERROR no/invalid Modbus response"));
      }
    }
    // Service Class C between coils and after each unpowered A2 read.
    // Ordinary new downlinks are deferred until the entire active poll ends.
    handleClassCDownlink();
    if (a5PollCancelled) return false;
    delay(timeBetweenPollsMs);
  }

  if (a5PollCancelled) return false;

  DEBUG_SERIAL.print(F("Uplink payload bytes: "));
  DEBUG_SERIAL.print(uplinkLen);
  DEBUG_SERIAL.print(F(" / HEX chars: "));
  DEBUG_SERIAL.println(uplinkLen * 2);
  DEBUG_SERIAL.print(F("Uplink payload: "));
  dumpHex(uplinkPayload, uplinkLen);

  return uplinkLen > 0;
}

bool sendRegularTimedPollConfirmedWithRetries() {
  // Reset only for a NEW scheduled cycle. Never reset on a confirmed retry.
  a5PollCancelled = false;
  a5RegularPollInProgress = true;
  uint8_t uplinkPayload[MAX_UPLINK_PAYLOAD_SIZE];
  size_t uplinkLen = 0;
  // Run the physical relay / sensor acquisition ONCE per scheduled cycle.
  // Confirmed retries only transmit this frozen snapshot.
  if (!buildRegularTimedPayload(uplinkPayload, sizeof(uplinkPayload), uplinkLen)) {
    DEBUG_SERIAL.println(F("Regular confirmed payload build failed or was interrupted."));
    a5RegularPollInProgress = false;
    return false;
  }
  if (a5PollCancelled || rejoinRequested) {
    a5RegularPollInProgress = false;
    return false;
  }
  for (uint8_t attempt = 0; attempt <= A3_CONFIRMED_UPLOAD_MAX_RETRIES; attempt++) {
    if (attempt > 0) {
      DEBUG_SERIAL.print(F("Regular confirmed retry waiting seconds: "));
      DEBUG_SERIAL.println(A3_PAGE_UPLOAD_DELAY_MS / 1000UL);

      uint32_t startMs = millis();
      while ((millis() - startMs) < A3_PAGE_UPLOAD_DELAY_MS &&
             !a5PollCancelled && !rejoinRequested) {
        handleClassCDownlink();
        delay(50);
        yield();
      }
    }
    if (a5PollCancelled || rejoinRequested) {
      a5RegularPollInProgress = false;
      return false;
    }

    DEBUG_SERIAL.print(F("Regular confirmed attempt "));
    DEBUG_SERIAL.print(attempt);
    DEBUG_SERIAL.print(F(" / "));
    DEBUG_SERIAL.println(A3_CONFIRMED_UPLOAD_MAX_RETRIES);
    DEBUG_SERIAL.println(F("Transmitting existing acquisition snapshot (no relay re-cycle)."));

    if (sendLoRaWanUplinkConfirmed(uplinkPayload, uplinkLen, F("Regular timed poll"))) {
      DEBUG_SERIAL.println(F("Regular confirmed timed poll sent and ACKed."));
      a5RegularPollInProgress = false;
      return true;
    }

    if (rejoinRequested || a5PollCancelled) {
      DEBUG_SERIAL.println(F("Regular confirmed timed poll interrupted; cancelling retries."));
      a5RegularPollInProgress = false;
      return false;
    }

    if (attempt < A3_CONFIRMED_UPLOAD_MAX_RETRIES) {
      DEBUG_SERIAL.println(F("Regular confirmed timed poll not ACKed. Will retransmit the SAME data without polling."));
    }
  }

  DEBUG_SERIAL.println(F("Regular confirmed timed poll failed after 2 retries."));
  a5RegularPollInProgress = false;
  return false;
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);

  DEBUG_SERIAL.begin(DEBUG_BAUD);
  delay(200);

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("BOOT"));
  DEBUG_SERIAL.println(F("Debug SoftwareSerial started"));
  DEBUG_SERIAL.println(F("Starting program..."));
  printDevEUIAtBoot();

  migrateLegacyNonceStorageIfNeeded();
  loadConfigFromEEPROM();
  loadA5ConfigFromEEPROM();
  loadA6ConfigFromEEPROM();
  printConfigAtBoot();
  printA5ConfigAtBoot();
  printA6ConfigAtBoot();
  powerOffA5AtStartup();  // Only initializes RS485 early if A5 is configured.
  powerOffA6AtStartup();  // Safety OFF for stored actuator mapping; no A6 ON at boot.

  blinkLed(1, 250, 250);

  DEBUG_SERIAL.println(F("Radio init..."));
  int16_t state = radio.begin();
  if (state != RADIOLIB_ERR_NONE) {
    haltWithMessage("radio.begin failed", state);
  }
  DEBUG_SERIAL.println(F("Radio OK"));

  fullJoinAndClassCStart();

  MODBUS_SERIAL.begin(MODBUS_BAUD, SERIAL_8N1);

  mb.begin(1, MODBUS_SERIAL);
  mb.preTransmission(preTransmission);
  mb.postTransmission(postTransmission);

  lastUplink = millis();
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop() {
  // Process queued commands in arrival order before newer downlinks. Defer
  // them further if a network rejoin is pending or an A6 trigger is queued.
  if (!isA6SequenceBusy() && !isA2RelayOnBusy() && !a6PendingStart &&
      !rejoinRequested && a6DeferredCount > 0) {
    a6ProcessDeferredCommand();
    handleA6SequenceTask(); // Deferred A2/A4 may start another A6 cycle.
    return;
  }
  handleClassCDownlink();
  handleA2RelayOnTask(); // A2 FC05 ON repeats at 3 seconds without blocking Class C.
  handleA6SequenceTask(); // Only programmed relay ON/OFF and T1/T2/T3 timers advance A6.

  // A6 owns the running cycle, including all slots and the timed OFF frames.
  // Do not enter OTAA rejoin, A3, heartbeat, or regular polling while active.
  if (isA6SequenceBusy() || isA2RelayOnBusy()) return;

  if (rejoinRequested) {
    DEBUG_SERIAL.println(F("NETWORK_NOT_JOINED or rejoin requested. Starting instant OTAA rejoin."));
    fullJoinAndClassCStart();
    lastUplink = millis();
    return;
  }

  handleA3LogTask();

  if (rejoinRequested) {
    DEBUG_SERIAL.println(F("NETWORK_NOT_JOINED after A3 operation. Starting instant OTAA rejoin."));
    fullJoinAndClassCStart();
    lastUplink = millis();
    return;
  }

  if (isA3LogBusy() || isA6SequenceBusy()) {
    // Avoid starting regular Modbus polling while A6 owns the RS485 sequence.
    return;
  }

  if ((millis() - lastHeartbeatMs) >= HEARTBEAT_INTERVAL_MS) {
    DEBUG_SERIAL.println(F("No successful LoRaWAN uplink for 60 minutes. Sending C heartbeat."));

    if (sendSpecialHeartbeatConfirmed()) {
      ackFailCounter = 0;
    } else {
      ackFailCounter++;
      handleMissedHeartbeat();
    }

    node.setClass(RADIOLIB_LORAWAN_CLASS_C);
    applyLoRaWANRuntimeSettings();

    if (rejoinRequested) {
      DEBUG_SERIAL.println(F("Rejoin requested after heartbeat. Starting instant OTAA rejoin."));
      fullJoinAndClassCStart();
      lastUplink = millis();
      return;
    }
  }

  if (millis() - lastUplink < (uint32_t)uploadIntervalSeconds * 1000UL) {
    return;
  }

  if (!hasAnyEnabledSlot(gConfig)) {
    DEBUG_SERIAL.println(F("No active poll slots configured"));
    lastUplink = millis();
    return;
  }

  sendRegularTimedPollConfirmedWithRetries();

  if (rejoinRequested) {
    DEBUG_SERIAL.println(F("NETWORK_NOT_JOINED after regular uplink. Starting instant OTAA rejoin."));
    fullJoinAndClassCStart();
    lastUplink = millis();
    return;
  }

  lastUplink = millis();

  DEBUG_SERIAL.print(F("Next uplink in "));
  DEBUG_SERIAL.print(uploadIntervalSeconds);
  DEBUG_SERIAL.println(F(" seconds"));
  DEBUG_SERIAL.println();
}
