/*
 * ============================================================================
 *  DW3000 UWB - ANCRE (repondeur DS-TWR, systeme multi-ancres)
 *  ESP32-DevKitC + module DWM3000
 * ============================================================================
 *
 *  Sketch AUTONOME pour une ANCRE FIXE. Le systeme comporte 3 ancres
 *  (ID 1, 2, 3) qui repondent toutes au meme tag. Chaque ancre ne traite que
 *  les trames POLL qui portent SON identifiant : flasher les 3 ancres avec ce
 *  meme code, en changeant uniquement ANCHOR_ID.
 *
 *  >>> AVANT DE FLASHER : regler ANCHOR_ID ci-dessous (1, 2 ou 3) <<<
 *
 *  Repere orthonorme (voir schema_positionnement_ancres.html) :
 *    A1 = origine (0,0,0) | A2 = axe X (Lx,0,0) | A3 = axe Y (0,Ly,0)
 *  Les positions des ancres sont configurees cote navigateur (visualisation).
 *
 *  Cablage ESP32 (voir schema_cablage_esp32.html) :
 *    GPIO18->CLK  GPIO23->MOSI  GPIO19->MISO  GPIO5->CS  GPIO27->IRQ  GPIO26->RST
 *    3.3V->VCC  GND->GND  -  condensateur 100 nF au plus pres du DWM3000
 *
 *  Protocole : DS-TWR (Double-Sided Two-Way Ranging), 4 trames par mesure.
 *  Bibliotheque : https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000
 * ============================================================================
 */

#include <SPI.h>
#include "dw3000.h"

// Variables internes de dw3000_port.cpp pilotees directement (cf. firmware
// dw3000_uwb_ranging : spiSelect() de la lib corrompt l'etat du DW3000).
extern uint8_t _ss;
extern uint8_t _rst;
extern uint8_t _irq;

// ============================================================================
//  >>> CONFIGURATION - A REGLER POUR CHAQUE ANCRE <<<
// ============================================================================
#define ANCHOR_ID   2        // Identifiant de CETTE ancre : 1, 2 ou 3

// ============================================================================
//  PINS ESP32
// ============================================================================
#define PIN_SCK     12
#define PIN_MOSI    11
#define PIN_MISO    13
#define PIN_CS      10
#define PIN_IRQ     14
#define PIN_RST     15

// ============================================================================
//  CONSTANTES UWB
// ============================================================================
#define SPEED_OF_LIGHT          299702547.0   // m/s

// Delai d'antenne - A CALIBRER (commande CALIB du firmware dw3000_uwb_ranging)
#define TX_ANT_DLY              16385
#define RX_ANT_DLY              16385

#define POLL_RX_TO_RESP_TX_DLY  1500          // POLL RX -> RESPONSE TX (us)
#define FINAL_RX_TIMEOUT        8000          // attente du FINAL (us)

// ============================================================================
//  TRAMES UWB  -  l'octet [10] porte l'identifiant de l'ancre cible
// ============================================================================
#define MSG_SN_IDX          2     // numero de sequence
#define MSG_FN_IDX          9     // code fonction
#define MSG_ANCHOR_IDX      10    // identifiant d'ancre

#define FN_POLL             0x21
#define FN_RESP             0x10
#define FN_FINAL            0x23
#define FN_RESULT           0x11

// Indices des 3 timestamps du Tag dans la trame FINAL
#define FINAL_POLL_TX_IDX   11
#define FINAL_RESP_RX_IDX   15
#define FINAL_FINAL_TX_IDX  19
// Index de la distance (float) dans la trame RESULT
#define RESULT_DIST_IDX     11

// RESPONSE : Ancre -> Tag
static uint8_t tx_resp_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A',
    FN_RESP, ANCHOR_ID, 0x02, 0, 0
};

// RESULT : Ancre -> Tag (contient la distance calculee)
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
//  CONFIGURATION DW3000  (canal 5, 6.8 Mbps, PRF 64 MHz, STS off)
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
//  FONCTIONS UTILITAIRES
// ============================================================================

void log_msg(const char* msg) {
    Serial.printf("[ANCRE #%d] %s\n", ANCHOR_ID, msg);
}

/* Calcul de distance DS-TWR a partir des 6 timestamps (annule la derive
 * d'horloge). Identique au firmware dw3000_uwb_ranging. */
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
    Serial.println("  DW3000 UWB - ANCRE DS-TWR multi-ancres");
    Serial.printf("  Identifiant de l'ancre : A%d\n", ANCHOR_ID);
    Serial.println("========================================\n");

    log_msg("Reset DW3000 + init SPI...");

    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_IRQ, INPUT);

    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

    // Memoriser les pins dans les globales internes de la lib
    _ss  = PIN_CS;
    _rst = PIN_RST;
    _irq = PIN_IRQ;

    // Reset materiel DW3000 - RST open-drain : ne pas driver HIGH
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(2);
    pinMode(PIN_RST, INPUT);
    delay(10);

    log_msg("Initialisation DW3000...");

    int idle_retries = 0;
    while (!dwt_checkidlerc()) {
        Serial.print(".");
        delay(50);
        if (++idle_retries > 40) {
            log_msg("ERREUR: DW3000 ne passe pas en IDLE_RC");
            log_msg("Verifier alim 3.3V, MISO, condensateur 100nF.");
            while (1) { delay(1000); }
        }
    }
    Serial.println(" OK");

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        log_msg("ERREUR: Initialisation DW3000 echouee !");
        while (1) { delay(1000); }
    }
    log_msg("DW3000 initialise avec succes");

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    if (dwt_configure(&config)) {
        log_msg("ERREUR: Configuration UWB echouee !");
        while (1) { delay(1000); }
    }
    log_msg("Configuration UWB: Canal 5 | 6.8 Mbps | PRF 64 MHz");

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    // L'ancre ecoute en continu (pas de timeout pour le POLL)
    dwt_setrxtimeout(0);

    log_msg("Ancre prete - en ecoute DS-TWR...");
    Serial.println();
}

// ============================================================================
//  LOOP - Repondeur DS-TWR
//   1. Ecouter -> recevoir un POLL adresse a CETTE ancre
//   2. Envoyer RESPONSE (delayed)
//   3. Attendre le FINAL
//   4. Calculer la distance (6 timestamps)
//   5. Envoyer RESULT au Tag
// ============================================================================
void loop() {
    // === 1. Activer le recepteur et attendre un POLL ===
    dwt_setrxtimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    uint32_t status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // attente active
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return;
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) return;
    dwt_readrxdata(rx_buffer, frame_len, 0);

    // Filtrage : ne traiter que les POLL adresses a CETTE ancre
    if (rx_buffer[MSG_FN_IDX] != FN_POLL) return;
    if (rx_buffer[MSG_ANCHOR_IDX] != ANCHOR_ID) return;   // POLL pour une autre ancre

    // === 2. Envoyer RESPONSE (delayed) ===
    uint64_t poll_rx_ts = get_rx_timestamp_u64();

    uint32_t resp_tx_time = (poll_rx_ts +
        (POLL_RX_TO_RESP_TX_DLY * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    // Timeout RX pour le FINAL (a configurer avant le starttx)
    dwt_setrxtimeout(FINAL_RX_TIMEOUT);

    tx_resp_msg[MSG_SN_IDX]     = frame_seq_nb;
    tx_resp_msg[MSG_ANCHOR_IDX] = ANCHOR_ID;
    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (ret != DWT_SUCCESS) {
        log_msg("Erreur: envoi RESPONSE echoue (trop tard)");
        error_count++;
        return;
    }

    // Attendre la fin du TX pour lire le timestamp TX reel de la RESPONSE
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
    uint64_t resp_tx_ts = get_tx_timestamp_u64();

    // === 3. Attendre le FINAL ===
    status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // attente active
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

    // === 4. Calculer la distance DS-TWR ===
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
        Serial.printf("[ANCRE #%d] Tag a %.2f m | Mesure #%lu | Erreurs: %lu\n",
                      ANCHOR_ID, distance, ranging_count, error_count);
    } else {
        log_msg("Distance hors limites - ignoree");
        error_count++;
        distance = -1.0;
    }

    // === 5. Envoyer RESULT au Tag ===
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
