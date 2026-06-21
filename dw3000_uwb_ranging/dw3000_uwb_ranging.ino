/*
 * ============================================================================
 *  DW3000 UWB - Basic TWR (Two-Way Ranging) code
 *  ESP32-DevKitC + DWM3000 Module
 * ============================================================================
 *
 *  Modes : ANCHOR (fixed anchor) or TAG (mobile)
 *  Protocol : DS-TWR (Double-Sided Two-Way Ranging)
 *  Accuracy : ~2-5 cm typical
 *
 *  ESP32 connections :
 *    GPIO18 → CLK    (SPI Clock)
 *    GPIO23 → MOSI   (SPI Data Out)
 *    GPIO19 → MISO   (SPI Data In)
 *    GPIO5  → CS_n   (Chip Select)
 *    GPIO27 → IRQn   (Interrupt)
 *    GPIO26 → RSTn   (Reset)
 *    3.3V   → VCC
 *    GND    → GND
 *
 *  Library : https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000
 *
 *  Author : Jonathan
 *  Date   : March 2026
 * ============================================================================
 */

#include <SPI.h>
#include "dw3000.h"

// Internal variables of dw3000_port.cpp : we drive them directly
// because spiSelect() of this lib writes to DW1000 registers that
// do not exist on DW3000 and corrupt the chip state.
extern uint8_t _ss;
extern uint8_t _rst;
extern uint8_t _irq;

// ============================================================================
//  CONFIGURATION - Modify according to your setup
// ============================================================================

// Operating mode : uncomment ONLY one mode
#define MODE_ANCHOR          // Fixed anchor (responder)
//#define MODE_TAG           // Mobile tag (initiator)

// Unique identifier of this node (modify for each anchor/tag)
#define NODE_ID          0x04

// Network identifiers
#define PAN_ID           0xDECA    // PAN ID common to all nodes
#define ANCHOR_ADDR      0x0002    // Anchor address (for the tag)

// ============================================================================
//  PINS ESP32
// ============================================================================
#define PIN_SCK          18
#define PIN_MOSI         23
#define PIN_MISO         19
#define PIN_CS           5
#define PIN_IRQ          27
#define PIN_RST          26

// ============================================================================
//  UWB CONSTANTS
// ============================================================================

// Speed of light (m/s)
#define SPEED_OF_LIGHT   299702547.0

// Antenna delay (to calibrate for your module !)
// Default value for DWM3000, adjust with a known distance
#define TX_ANT_DLY       16385
#define RX_ANT_DLY       16385

// Delays and timeouts (UUS = UWB microseconds, ~1 µs)
// IMPORTANT : these values must give the CPU + SPI enough time to prepare
// the TX frame before the delayed starttx. This lib has a slow SPI → wide margins.
// For the RX TIMEOUTs, the internal unit is different (DTU/256), ~1.026 µs.
#define POLL_RX_TO_RESP_TX_DLY   1500    // Anchor: POLL RX → RESPONSE TX (µs)
#define RESP_RX_TO_FINAL_TX_DLY  3500    // Tag: RESPONSE RX → FINAL TX (µs)
#define RESP_RX_TIMEOUT          5000    // Tag waits for RESPONSE (~5 ms)
#define FINAL_RX_TIMEOUT         8000    // Anchor waits for FINAL (~8 ms)
#define RESULT_RX_TIMEOUT        5000    // Tag waits for RESULT (~5 ms)

// Interval between measurements (ms)
#define RANGING_INTERVAL 100

// ============================================================================
//  UWB FRAMES
// ============================================================================

// POLL frame (sent by the TAG)
static uint8_t tx_poll_msg[] = {
    0x41, 0x88,           // Frame Control (data frame, PAN ID compression)
    0,                    // Sequence number (auto-incremented)
    0xCA, 0xDE,           // PAN ID
    'W', 'A',             // Destination (Anchor)
    'V', 'E',             // Source (Tag)
    0x21,                 // Function code : POLL
    0, 0                  // CRC (auto-computed by DW3000)
};

// RESPONSE frame (sent by the ANCHOR)
// In DS-TWR, no timestamps needed : the Anchor keeps them locally
static uint8_t tx_resp_msg[] = {
    0x41, 0x88,           // Frame Control
    0,                    // Sequence number
    0xCA, 0xDE,           // PAN ID
    'V', 'E',             // Destination (Tag)
    'W', 'A',             // Source (Anchor)
    0x10,                 // Function code : RESPONSE
    0x02,                 // Activity code
    0, 0                  // CRC
};

// FINAL frame (sent by the TAG → Anchor)
// Contains the 3 Tag timestamps : poll_tx, resp_rx, final_tx
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

static uint8_t tx_final_msg[] = {
    0x41, 0x88,           // Frame Control
    0,                    // Sequence number
    0xCA, 0xDE,           // PAN ID
    'W', 'A',             // Destination (Anchor)
    'V', 'E',             // Source (Tag)
    0x23,                 // Function code : FINAL
    0, 0, 0, 0,           // [10-13] poll_tx_ts (32-bit)
    0, 0, 0, 0,           // [14-17] resp_rx_ts (32-bit)
    0, 0, 0, 0,           // [18-21] final_tx_ts (32-bit)
    0, 0                  // CRC
};

// RESULT frame (sent by the ANCHOR → Tag)
// Contains the computed distance (float)
#define RESULT_MSG_DIST_IDX 10

static uint8_t tx_result_msg[] = {
    0x41, 0x88,           // Frame Control
    0,                    // Sequence number
    0xCA, 0xDE,           // PAN ID
    'V', 'E',             // Destination (Tag)
    'W', 'A',             // Source (Anchor)
    0x11,                 // Function code : RESULT
    0, 0, 0, 0,           // [10-13] distance (float, meters)
    0, 0                  // CRC
};

// Reception buffer (large enough for the 24-byte FINAL frame)
#define RX_BUF_LEN 28
static uint8_t rx_buffer[RX_BUF_LEN];

// Index of the check bytes within the frames
#define ALL_MSG_SN_IDX          2     // Sequence number index

// ============================================================================
//  GLOBAL VARIABLES
// ============================================================================
static uint8_t frame_seq_nb = 0;
static double last_distance = 0.0;
static uint32_t ranging_count = 0;
static uint32_t error_count = 0;

// Status
typedef enum {
    STATUS_IDLE,
    STATUS_POLLING,
    STATUS_WAITING_RESP,
    STATUS_SENDING_FINAL,
    STATUS_WAITING_FINAL,
    STATUS_WAITING_RESULT,
    STATUS_LISTENING,
    STATUS_RESPONDING,
    STATUS_RANGING_OK,
    STATUS_ERROR
} ranging_status_t;

static ranging_status_t status = STATUS_IDLE;

// ============================================================================
//  DW3000 CONFIGURATION
// ============================================================================

/* Default configuration :
 *  - Channel 5 (6489.6 MHz) — the most common
 *  - PRF 64 MHz
 *  - Preamble 128 symbols
 *  - Datarate 6.8 Mbps
 *  - STS disabled (to keep it simple)
 */
static dwt_config_t config = {
    5,                /* Channel number (5 or 9) */
    DWT_PLEN_128,     /* Preamble length */
    DWT_PAC8,         /* Preamble Acquisition Chunk size */
    9,                /* TX preamble code */
    9,                /* RX preamble code */
    1,                /* SFD type (0 = standard, 1 = non-standard) */
    DWT_BR_6M8,       /* Data rate 6.8 Mbps */
    DWT_PHRMODE_STD,  /* PHR mode */
    DWT_PHRRATE_STD,  /* PHR rate */
    (129 + 8 - 8),    /* SFD timeout */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length */
    DWT_PDOA_M0       /* PDOA mode off */
};

// ============================================================================
//  UTILITY FUNCTIONS
// ============================================================================

/* Note : get_tx_timestamp_u64() and get_rx_timestamp_u64() are provided by
 * the Makerfabs library (dw3000_shared_functions.h) — we use those
 * directly, no local redeclaration.
 */

/**
 * DS-TWR distance computation (Double-Sided Two-Way Ranging)
 *
 * Uses the 6 timestamps to cancel the clock drift error :
 *
 *   TAG                          ANCHOR
 *   t1 (poll_tx) ──── POLL ────> t2 (poll_rx)
 *   t4 (resp_rx) <─── RESP ──── t3 (resp_tx)
 *   t5 (final_tx) ─── FINAL ──> t6 (final_rx)
 *
 * R1 = t4 - t1  (round-trip 1, Tag side)
 * D1 = t3 - t2  (response delay, Anchor side)
 * R2 = t6 - t3  (round-trip 2, Anchor side)
 * D2 = t5 - t4  (final delay, Tag side)
 *
 * ToF = (R1 × R2 - D1 × D2) / (R1 + R2 + D1 + D2)
 */
static double compute_distance_dstwr(uint64_t poll_tx_ts, uint64_t poll_rx_ts,
                                      uint64_t resp_tx_ts, uint64_t resp_rx_ts,
                                      uint64_t final_tx_ts, uint64_t final_rx_ts) {
    // Round-trip and delays
    double Ra = (double)((uint32_t)(resp_rx_ts - poll_tx_ts));   // R1 (Tag)
    double Db = (double)((uint32_t)(resp_tx_ts - poll_rx_ts));   // D1 (Anchor)
    double Rb = (double)((uint32_t)(final_rx_ts - resp_tx_ts));  // R2 (Anchor)
    double Da = (double)((uint32_t)(final_tx_ts - resp_rx_ts));  // D2 (Tag)

    // DS-TWR formula
    double tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);

    // Conversion to meters
    double tof_seconds = tof_dtu * DWT_TIME_UNITS;
    double distance = tof_seconds * SPEED_OF_LIGHT;

    return distance;
}

/**
 * Print status on the serial port
 */
void print_status(const char* msg) {
    Serial.printf("[%s #%02X] %s\n",
#ifdef MODE_ANCHOR
        "ANCHOR",
#else
        "TAG",
#endif
        NODE_ID, msg);
}

void print_distance(double distance) {
    ranging_count++;
    Serial.printf("[%s #%02X] Distance: %.2f m | Measurement #%lu | Errors: %lu\n",
#ifdef MODE_ANCHOR
                  "ANCHOR",
#else
                  "TAG",
#endif
                  NODE_ID, distance, ranging_count, error_count);
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  DW3000 UWB - DS-TWR Ranging");
#ifdef MODE_ANCHOR
    Serial.println("  Mode : ANCHOR (Responder)");
#else
    Serial.println("  Mode : TAG (Initiator)");
#endif
    Serial.printf("  Node ID : 0x%02X\n", NODE_ID);
    Serial.println("========================================\n");

    // --- Init SPI + pins (bypass spiSelect which contains DW1000 code) ---
    print_status("Reset DW3000 + SPI init...");

    // Control pins
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_IRQ, INPUT);

    // SPI bus
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

    // Store the pins in the lib's internal globals
    // (otherwise readBytes/writetospi toggle GPIO0 instead of PIN_CS)
    _ss  = PIN_CS;
    _rst = PIN_RST;
    _irq = PIN_IRQ;

    // DW3000 hardware reset — RST is open-drain, do NOT drive it HIGH,
    // we set it back to INPUT (DW3000 internal pull-up brings it to VDDIO)
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(2);
    pinMode(PIN_RST, INPUT);
    delay(10);  // INIT_RC transition

    // --- DW3000 initialization ---
    print_status("DW3000 initialization...");

    // Wait for IDLE_RC
    int idle_retries = 0;
    while (!dwt_checkidlerc()) {
        Serial.print(".");
        delay(50);
        if (++idle_retries > 40) {
            Serial.println(" TIMEOUT idle_rc");
            print_status("ERROR: DW3000 does not enter IDLE_RC");
            print_status("Check 3.3V supply, MISO, 100nF capacitor.");
            while (1) { delay(1000); }
        }
    }
    Serial.println(" OK");

    // Init (Decawave DW3000 driver — THIS is what does the real init)
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        print_status("ERROR: DW3000 initialization failed !");
        print_status("Device ID read incorrect — check 3.3V supply and MISO.");
        while (1) { delay(1000); }
    }
    print_status("DW3000 initialized successfully");

    // Enable the DW3000 LEDs (useful for debugging)
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    // --- UWB configuration ---
    if (dwt_configure(&config)) {
        print_status("ERROR: UWB configuration failed !");
        while (1) { delay(1000); }
    }
    print_status("UWB configuration: Channel 5 | 6.8 Mbps | PRF 64 MHz");

    // --- Antenna delays ---
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    // --- Mode-specific configuration ---
#ifdef MODE_TAG
    // The tag waits for the response with a timeout
    dwt_setrxtimeout(RESP_RX_TIMEOUT);
    dwt_setpreambledetecttimeout(0);  // No timeout on the preamble
    print_status("Tag ready - Starting DS-TWR ranging");
#else
    // The anchor listens continuously (no timeout for the POLL)
    dwt_setrxtimeout(0);
    print_status("Anchor ready - Listening DS-TWR...");
#endif

    Serial.println();
}

// ============================================================================
//  LOOP - TAG MODE (DS-TWR initiator)
//  Phase 1 : Send POLL
//  Phase 2 : Wait for RESPONSE
//  Phase 3 : Send FINAL (with timestamps)
//  Phase 4 : Wait for RESULT (distance computed by the Anchor)
// ============================================================================
#ifdef MODE_TAG

void loop() {
    // === Phase 1 : Send POLL ===
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    // Send POLL and enable RX for the RESPONSE
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    status = STATUS_POLLING;

    // === Phase 2 : Wait for the RESPONSE ===
    uint32_t status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // Active wait
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        // Timeout or error on RESPONSE
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        error_count++;
        if (status_reg & SYS_STATUS_ALL_RX_TO) {
            print_status("Timeout - no RESPONSE from anchor");
        } else {
            print_status("RESPONSE reception error");
        }
        status = STATUS_ERROR;
        frame_seq_nb++;
        delay(RANGING_INTERVAL);
        return;
    }

    // Good RESPONSE reception
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) {
        frame_seq_nb++;
        delay(RANGING_INTERVAL);
        return;
    }
    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;

    if (rx_buffer[9] != 0x10) {  // Not a RESPONSE function code
        frame_seq_nb++;
        delay(RANGING_INTERVAL);
        return;
    }

    status = STATUS_WAITING_RESP;

    // === Phase 3 : Send FINAL ===
    // Read the POLL TX and RESPONSE RX timestamps
    uint64_t poll_tx_ts = get_tx_timestamp_u64();
    uint64_t resp_rx_ts = get_rx_timestamp_u64();

    // Compute the FINAL send time (delayed TX)
    uint32_t final_tx_time = (resp_rx_ts +
        (RESP_RX_TO_FINAL_TX_DLY * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);

    // Planned FINAL TX timestamp (to insert into the frame)
    uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    // Insert the 3 Tag timestamps into the FINAL frame (truncated 32-bit)
    uint32_t poll_tx_ts_32  = (uint32_t)poll_tx_ts;
    uint32_t resp_rx_ts_32  = (uint32_t)resp_rx_ts;
    uint32_t final_tx_ts_32 = (uint32_t)final_tx_ts;

    memcpy(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX],  &poll_tx_ts_32,  4);
    memcpy(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX],  &resp_rx_ts_32,  4);
    memcpy(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts_32, 4);

    tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);

    // Configure the RX timeout for the RESULT
    dwt_setrxtimeout(RESULT_RX_TIMEOUT);

    // Send FINAL delayed and enable RX for the RESULT
    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS) {
        print_status("Error: FINAL send failed (too late)");
        // Restore the timeout for the next RESPONSE
        dwt_setrxtimeout(RESP_RX_TIMEOUT);
        error_count++;
        status = STATUS_ERROR;
        frame_seq_nb++;
        delay(RANGING_INTERVAL);
        return;
    }

    status = STATUS_SENDING_FINAL;

    // === Phase 4 : Wait for the RESULT (optional) ===
    status = STATUS_WAITING_RESULT;

    status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // Active wait
    }

    // Restore the timeout for the next RESPONSE
    dwt_setrxtimeout(RESP_RX_TIMEOUT);

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        // Good RESULT reception
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (rx_buffer[9] == 0x11) {  // RESULT function code
                float distance;
                memcpy(&distance, &rx_buffer[RESULT_MSG_DIST_IDX], sizeof(float));

                if (distance > 0 && distance < 300.0) {
                    last_distance = (double)distance;
                    print_distance(last_distance);
                    status = STATUS_RANGING_OK;
                } else {
                    print_status("Distance out of bounds - ignored");
                    error_count++;
                    status = STATUS_ERROR;
                }
            }
        }
    } else {
        // Timeout or error - the RESULT is optional, not critical
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }

    // Increment the sequence number
    frame_seq_nb++;

    // Wait before the next measurement
    delay(RANGING_INTERVAL);
}

#endif // MODE_TAG

// ============================================================================
//  LOOP - ANCHOR MODE (DS-TWR responder)
//  Phase 1 : Listen → receive POLL
//  Phase 2 : Send RESPONSE delayed (+ enable RX for FINAL)
//  Phase 3 : Wait for FINAL
//  Phase 4 : Compute the DS-TWR distance (6 timestamps)
//  Phase 5 : Send RESULT to the Tag
// ============================================================================
#ifdef MODE_ANCHOR

void loop() {
    // === Phase 1 : Enable the receiver and wait for a POLL ===
    dwt_setrxtimeout(0);  // No timeout for POLL listening
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    status = STATUS_LISTENING;

    uint32_t status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // Active wait
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        // Reception error
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return;
    }

    // Good reception
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) return;
    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;

    if (rx_buffer[9] != 0x21) return;  // Not a POLL → ignore

    // === Phase 2 : Send RESPONSE delayed ===
    // Read the POLL reception timestamp
    uint64_t poll_rx_ts = get_rx_timestamp_u64();

    // Compute the RESPONSE send timestamp (delayed)
    uint32_t resp_tx_time = (poll_rx_ts +
        (POLL_RX_TO_RESP_TX_DLY * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    // Configure the RX timeout for the FINAL (before the starttx)
    dwt_setrxtimeout(FINAL_RX_TIMEOUT);

    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

    // Send RESPONSE delayed and enable RX for the FINAL
    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS) {
        print_status("Error: RESPONSE send failed (too late)");
        error_count++;
        status = STATUS_ERROR;
        return;
    }

    status = STATUS_RESPONDING;

    // Wait for the end of the RESPONSE transmission to read the real TX timestamp
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

    // Read the real RESPONSE TX timestamp (more accurate than the computation)
    uint64_t resp_tx_ts = get_tx_timestamp_u64();

    // === Phase 3 : Wait for the FINAL ===
    status = STATUS_WAITING_FINAL;

    status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // Active wait
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        // Timeout or error on FINAL
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        if (status_reg & SYS_STATUS_ALL_RX_TO) {
            print_status("Timeout - no FINAL from tag");
        } else {
            print_status("FINAL reception error");
        }
        error_count++;
        status = STATUS_ERROR;
        frame_seq_nb++;
        return;
    }

    // Good FINAL reception
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) {
        frame_seq_nb++;
        return;
    }
    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;

    if (rx_buffer[9] != 0x23) {  // Not a FINAL → ignore
        frame_seq_nb++;
        return;
    }

    // === Phase 4 : Compute the DS-TWR distance ===
    // Read the FINAL reception timestamp
    uint64_t final_rx_ts = get_rx_timestamp_u64();

    // Extract the 3 Tag timestamps from the FINAL frame
    uint32_t poll_tx_ts_32, resp_rx_ts_32, final_tx_ts_32;
    memcpy(&poll_tx_ts_32,  &rx_buffer[FINAL_MSG_POLL_TX_TS_IDX],  4);
    memcpy(&resp_rx_ts_32,  &rx_buffer[FINAL_MSG_RESP_RX_TS_IDX],  4);
    memcpy(&final_tx_ts_32, &rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], 4);

    // Rebuild the 64-bit timestamps (32-bit are enough for the differences)
    uint64_t poll_tx_ts  = (uint64_t)poll_tx_ts_32;
    uint64_t resp_rx_ts  = (uint64_t)resp_rx_ts_32;
    uint64_t final_tx_ts = (uint64_t)final_tx_ts_32;

    // Compute the distance with the 6 timestamps
    double distance = compute_distance_dstwr(poll_tx_ts, poll_rx_ts,
                                              resp_tx_ts, resp_rx_ts,
                                              final_tx_ts, final_rx_ts);

    // Filter out aberrant values
    if (distance > 0 && distance < 300.0) {
        last_distance = distance;
        print_distance(distance);
        status = STATUS_RANGING_OK;
    } else {
        print_status("Distance out of bounds - ignored");
        error_count++;
        status = STATUS_ERROR;
    }

    // === Phase 5 : Send RESULT to the Tag ===
    float dist_float = (float)distance;
    memcpy(&tx_result_msg[RESULT_MSG_DIST_IDX], &dist_float, sizeof(float));
    tx_result_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    dwt_writetxdata(sizeof(tx_result_msg), tx_result_msg, 0);
    dwt_writetxfctrl(sizeof(tx_result_msg), 0, 1);

    dwt_starttx(DWT_START_TX_IMMEDIATE);

    // Wait for the end of the RESULT transmission
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

    frame_seq_nb++;
}

#endif // MODE_ANCHOR
