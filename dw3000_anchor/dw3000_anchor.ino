/*
 * ============================================================================
 *  DW3000 UWB - ANCHOR (DS-TWR responder, multi-anchor system)
 *  ESP32-DevKitC + DWM3000 module
 * ============================================================================
 *
 *  STANDALONE sketch for a FIXED ANCHOR. The system has 3 anchors
 *  (ID 1, 2, 3) that all respond to the same tag. Each anchor only processes
 *  the POLL frames carrying ITS identifier: flash the 3 anchors with this
 *  same code, changing only ANCHOR_ID.
 *
 *  >>> BEFORE FLASHING: set ANCHOR_ID below (1, 2 or 3) <<<
 *
 *  Orthonormal frame (see schema_positionnement_ancres.html):
 *    A1 = origin (0,0,0) | A2 = X axis (Lx,0,0) | A3 = Y axis (0,Ly,0)
 *  Anchor positions are configured on the browser side (visualization).
 *
 *  ESP32 wiring (see schema_cablage_esp32.html):
 *    GPIO18->CLK  GPIO23->MOSI  GPIO19->MISO  GPIO5->CS  GPIO27->IRQ  GPIO26->RST
 *    3.3V->VCC  GND->GND  -  100 nF capacitor as close as possible to the DWM3000
 *
 *  Protocol: DS-TWR (Double-Sided Two-Way Ranging), 4 frames per measurement.
 *  Library: https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000
 * ============================================================================
 */

#include <SPI.h>
#include "dw3000.h"

// Internal variables of dw3000_port.cpp driven directly (cf. firmware
// dw3000_uwb_ranging: the lib's spiSelect() corrupts the DW3000 state).
extern uint8_t _ss;
extern uint8_t _rst;
extern uint8_t _irq;

// ============================================================================
//  >>> CONFIGURATION - TO SET FOR EACH ANCHOR <<<
// ============================================================================
#define ANCHOR_ID   2       // Identifier of THIS anchor: 1, 2 or 3

// ============================================================================
//  PINS ESP32
// ============================================================================
#define PIN_SCK     18
#define PIN_MOSI    23
#define PIN_MISO    19
#define PIN_CS      5
#define PIN_IRQ     27
#define PIN_RST     26

// ============================================================================
//  CONSTANTES UWB
// ============================================================================
#define SPEED_OF_LIGHT          299702547.0   // m/s

// Antenna delay - TO CALIBRATE (CALIB command of firmware dw3000_uwb_ranging)
#define TX_ANT_DLY              16385
#define RX_ANT_DLY              16385

#define POLL_RX_TO_RESP_TX_DLY  1500          // POLL RX -> RESPONSE TX (us)
#define FINAL_RX_TIMEOUT        8000          // FINAL wait (us)

// ============================================================================
//  UWB FRAMES  -  byte [10] carries the target anchor identifier
// ============================================================================
#define MSG_SN_IDX          2     // sequence number
#define MSG_FN_IDX          9     // function code
#define MSG_ANCHOR_IDX      10    // anchor identifier

#define FN_POLL             0x21
#define FN_RESP             0x10
#define FN_FINAL            0x23
#define FN_RESULT           0x11

// Indices of the 3 Tag timestamps in the FINAL frame
#define FINAL_POLL_TX_IDX   11
#define FINAL_RESP_RX_IDX   15
#define FINAL_FINAL_TX_IDX  19
// Index of the distance (float) in the RESULT frame
#define RESULT_DIST_IDX     11

// RESPONSE: Anchor -> Tag
static uint8_t tx_resp_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A',
    FN_RESP, ANCHOR_ID, 0x02, 0, 0
};

// RESULT: Anchor -> Tag (contains the computed distance)
static uint8_t tx_result_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A',
    FN_RESULT, ANCHOR_ID, 0, 0, 0, 0, 0, 0
};

#define RX_BUF_LEN 32
static uint8_t rx_buffer[RX_BUF_LEN];

static uint8_t  frame_seq_nb  = 0;
static uint32_t ranging_count = 0;
static uint32_t error_count   = 0;

// ============================================================================
//  DW3000 CONFIGURATION  (channel 5, 6.8 Mbps, PRF 64 MHz, STS off)
// ============================================================================
static dwt_config_t config = {
    5,                /* Channel 5 (6489.6 MHz) */
    DWT_PLEN_128,     /* Preamble length */
    DWT_PAC8,         /* Preamble Acquisition Chunk */
    9,                /* TX preamble code */
    9,                /* RX preamble code */
    1,                /* SFD type */
    DWT_BR_6M8,       /* Data rate 6.8 Mbps */
    DWT_PHRMODE_STD,
    DWT_PHRRATE_STD,
    (129 + 8 - 8),    /* SFD timeout */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,
    DWT_PDOA_M0
};

// ============================================================================
//  UTILITY FUNCTIONS
// ============================================================================

void log_msg(const char* msg) {
    Serial.printf("[ANCHOR #%d] %s\n", ANCHOR_ID, msg);
}

/* DS-TWR distance computation from the 6 timestamps (cancels clock
 * drift). Identical to firmware dw3000_uwb_ranging. */
static double compute_distance_dstwr(uint64_t poll_tx_ts, uint64_t poll_rx_ts,
                                      uint64_t resp_tx_ts, uint64_t resp_rx_ts,
                                      uint64_t final_tx_ts, uint64_t final_rx_ts) {
    double Ra = (double)((uint32_t)(resp_rx_ts  - poll_tx_ts));
    double Db = (double)((uint32_t)(resp_tx_ts  - poll_rx_ts));
    double Rb = (double)((uint32_t)(final_rx_ts - resp_tx_ts));
    double Da = (double)((uint32_t)(final_tx_ts - resp_rx_ts));
    double tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);
    return tof_dtu * DWT_TIME_UNITS * SPEED_OF_LIGHT;
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  DW3000 UWB - ANCHOR DS-TWR multi-anchor");
    Serial.printf("  Anchor identifier : A%d\n", ANCHOR_ID);
    Serial.println("========================================\n");

    log_msg("Reset DW3000 + init SPI...");

    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_IRQ, INPUT);

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

    // Store the pins in the lib's internal globals
    _ss  = PIN_CS;
    _rst = PIN_RST;
    _irq = PIN_IRQ;

    // DW3000 hardware reset - RST open-drain: do not drive HIGH
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(2);
    pinMode(PIN_RST, INPUT);
    delay(10);

    log_msg("Initializing DW3000...");

    int idle_retries = 0;
    while (!dwt_checkidlerc()) {
        Serial.print(".");
        delay(50);
        if (++idle_retries > 40) {
            log_msg("ERROR: DW3000 does not enter IDLE_RC");
            log_msg("Check 3.3V supply, MISO, 100nF capacitor.");
            while (1) { delay(1000); }
        }
    }
    Serial.println(" OK");

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        log_msg("ERROR: DW3000 initialization failed !");
        while (1) { delay(1000); }
    }
    log_msg("DW3000 initialized successfully");

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    if (dwt_configure(&config)) {
        log_msg("ERROR: UWB configuration failed !");
        while (1) { delay(1000); }
    }
    log_msg("UWB configuration: Channel 5 | 6.8 Mbps | PRF 64 MHz");

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    // The anchor listens continuously (no timeout for the POLL)
    dwt_setrxtimeout(0);

    log_msg("Anchor ready - listening DS-TWR...");
    Serial.println();
}

// ============================================================================
//  LOOP - DS-TWR responder
//   1. Listen -> receive a POLL addressed to THIS anchor
//   2. Send RESPONSE (delayed)
//   3. Wait for the FINAL
//   4. Compute the distance (6 timestamps)
//   5. Send RESULT to the Tag
// ============================================================================
void loop() {
    // === 1. Enable the receiver and wait for a POLL ===
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    uint32_t status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // active wait
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return;
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) return;
    dwt_readrxdata(rx_buffer, frame_len, 0);

    // Filtering: only process POLLs addressed to THIS anchor
    if (rx_buffer[MSG_FN_IDX] != FN_POLL) return;
    if (rx_buffer[MSG_ANCHOR_IDX] != ANCHOR_ID) return;   // POLL for another anchor

    // === 2. Send RESPONSE (delayed) ===
    uint64_t poll_rx_ts = get_rx_timestamp_u64();

    uint32_t resp_tx_time = (poll_rx_ts +
        (POLL_RX_TO_RESP_TX_DLY * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    // RX timeout for the FINAL (to configure before the starttx)
    dwt_setrxtimeout(FINAL_RX_TIMEOUT);

    tx_resp_msg[MSG_SN_IDX]     = frame_seq_nb;
    tx_resp_msg[MSG_ANCHOR_IDX] = ANCHOR_ID;
    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (ret != DWT_SUCCESS) {
        log_msg("Error: RESPONSE send failed (too late)");
        error_count++;
        return;
    }

    // Wait for the end of TX to read the real TX timestamp of the RESPONSE
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    uint64_t resp_tx_ts = get_tx_timestamp_u64();

    // === 3. Wait for the FINAL ===
    status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // active wait
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        error_count++;
        frame_seq_nb++;
        return;
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) { frame_seq_nb++; return; }
    dwt_readrxdata(rx_buffer, frame_len, 0);

    if (rx_buffer[MSG_FN_IDX] != FN_FINAL) { frame_seq_nb++; return; }

    // === 4. Compute the DS-TWR distance ===
    uint64_t final_rx_ts = get_rx_timestamp_u64();

    uint32_t poll_tx_ts_32, resp_rx_ts_32, final_tx_ts_32;
    memcpy(&poll_tx_ts_32,  &rx_buffer[FINAL_POLL_TX_IDX],   4);
    memcpy(&resp_rx_ts_32,  &rx_buffer[FINAL_RESP_RX_IDX],   4);
    memcpy(&final_tx_ts_32, &rx_buffer[FINAL_FINAL_TX_IDX],  4);

    double distance = compute_distance_dstwr(
        (uint64_t)poll_tx_ts_32, poll_rx_ts,
        resp_tx_ts,              (uint64_t)resp_rx_ts_32,
        (uint64_t)final_tx_ts_32, final_rx_ts);

    if (distance > 0 && distance < 300.0) {
        ranging_count++;
        Serial.printf("[ANCHOR #%d] Tag at %.2f m | Measurement #%lu | Errors: %lu\n",
                      ANCHOR_ID, distance, ranging_count, error_count);
    } else {
        log_msg("Distance out of range - ignored");
        error_count++;
        distance = -1.0;
    }

    // === 5. Send RESULT to the Tag ===
    float dist_float = (float)distance;
    memcpy(&tx_result_msg[RESULT_DIST_IDX], &dist_float, sizeof(float));
    tx_result_msg[MSG_SN_IDX]     = frame_seq_nb;
    tx_result_msg[MSG_ANCHOR_IDX] = ANCHOR_ID;

    dwt_writetxdata(sizeof(tx_result_msg), tx_result_msg, 0);
    dwt_writetxfctrl(sizeof(tx_result_msg), 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE);

    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

    frame_seq_nb++;
}
