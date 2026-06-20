/*
 * ============================================================================
 *  DW3000 UWB - Code de base TWR (Two-Way Ranging)
 *  ESP32-DevKitC + DWM3000 Module
 * ============================================================================
 *
 *  Modes : ANCHOR (ancre fixe) ou TAG (mobile)
 *  Protocole : DS-TWR (Double-Sided Two-Way Ranging)
 *  Précision : ~2-5 cm typique
 *
 *  Connexions ESP32 :
 *    GPIO18 → CLK    (SPI Clock)
 *    GPIO23 → MOSI   (SPI Data Out)
 *    GPIO19 → MISO   (SPI Data In)
 *    GPIO5  → CS_n   (Chip Select)
 *    GPIO27 → IRQn   (Interrupt)
 *    GPIO26 → RSTn   (Reset)
 *    3.3V   → VCC
 *    GND    → GND
 *
 *  Bibliothèque : https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000
 *
 *  Auteur : Jonathan
 *  Date   : Mars 2026
 * ============================================================================
 */

#include <SPI.h>
#include "dw3000.h"

// Variables internes de dw3000_port.cpp : on les pilote directement
// car spiSelect() de cette lib écrit dans des registres DW1000 qui
// n'existent pas sur DW3000 et corrompent l'état du chip.
extern uint8_t _ss;
extern uint8_t _rst;
extern uint8_t _irq;

// ============================================================================
//  CONFIGURATION - Modifier selon ton setup
// ============================================================================

// Mode de fonctionnement : décommenter UN seul mode
#define MODE_ANCHOR          // Ancre fixe (répondeur)
//#define MODE_TAG           // Tag mobile (initiateur)

// Identifiant unique de ce noeud (modifier pour chaque ancre/tag)
#define NODE_ID          0x04

// Identifiants pour le réseau
#define PAN_ID           0xDECA    // PAN ID commun à tous les noeuds
#define ANCHOR_ADDR      0x0002    // Adresse de l'ancre (pour le tag)

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
//  CONSTANTES UWB
// ============================================================================

// Vitesse de la lumière (m/s)
#define SPEED_OF_LIGHT   299702547.0

// Délai d'antenne (à calibrer pour ton module !)
// Valeur par défaut pour DWM3000, ajuster avec une distance connue
#define TX_ANT_DLY       16385
#define RX_ANT_DLY       16385

// Délais et timeouts (UUS = UWB microseconds, ~1 µs)
// IMPORTANT : ces valeurs doivent laisser le temps au CPU + SPI de préparer
// la trame TX avant le starttx delayed. Cette lib a un SPI lent → marges larges.
// Pour les TIMEOUTs RX, l'unité interne est différente (DTU/256), ~1.026 µs.
#define POLL_RX_TO_RESP_TX_DLY   1500    // Anchor: POLL RX → RESPONSE TX (µs)
#define RESP_RX_TO_FINAL_TX_DLY  3500    // Tag: RESPONSE RX → FINAL TX (µs)
#define RESP_RX_TIMEOUT          5000    // Tag attend RESPONSE (~5 ms)
#define FINAL_RX_TIMEOUT         8000    // Anchor attend FINAL (~8 ms)
#define RESULT_RX_TIMEOUT        5000    // Tag attend RESULT (~5 ms)

// Intervalle entre les mesures (ms)
#define RANGING_INTERVAL 100

// ============================================================================
//  TRAMES UWB
// ============================================================================

// Trame POLL (envoyée par le TAG)
static uint8_t tx_poll_msg[] = {
    0x41, 0x88,           // Frame Control (data frame, PAN ID compression)
    0,                    // Sequence number (auto-incrémenté)
    0xCA, 0xDE,           // PAN ID
    'W', 'A',             // Destination (Anchor)
    'V', 'E',             // Source (Tag)
    0x21,                 // Function code : POLL
    0, 0                  // CRC (auto-calculé par DW3000)
};

// Trame RESPONSE (envoyée par l'ANCHOR)
// En DS-TWR, pas besoin de timestamps : l'Anchor les garde localement
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

// Trame FINAL (envoyée par le TAG → Anchor)
// Contient les 3 timestamps du Tag : poll_tx, resp_rx, final_tx
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

// Trame RESULT (envoyée par l'ANCHOR → Tag)
// Contient la distance calculée (float)
#define RESULT_MSG_DIST_IDX 10

static uint8_t tx_result_msg[] = {
    0x41, 0x88,           // Frame Control
    0,                    // Sequence number
    0xCA, 0xDE,           // PAN ID
    'V', 'E',             // Destination (Tag)
    'W', 'A',             // Source (Anchor)
    0x11,                 // Function code : RESULT
    0, 0, 0, 0,           // [10-13] distance (float, mètres)
    0, 0                  // CRC
};

// Buffer de réception (assez grand pour la trame FINAL de 24 octets)
#define RX_BUF_LEN 28
static uint8_t rx_buffer[RX_BUF_LEN];

// Index des octets de vérification dans les trames
#define ALL_MSG_SN_IDX          2     // Index du sequence number

// ============================================================================
//  VARIABLES GLOBALES
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
//  CONFIGURATION DW3000
// ============================================================================

/* Configuration par défaut :
 *  - Canal 5 (6489.6 MHz) — le plus courant
 *  - PRF 64 MHz
 *  - Preamble 128 symbols
 *  - Datarate 6.8 Mbps
 *  - STS désactivé (pour simplifier)
 */
static dwt_config_t config = {
    5,                /* Channel number (5 ou 9) */
    DWT_PLEN_128,     /* Preamble length */
    DWT_PAC8,         /* Preamble Acquisition Chunk size */
    9,                /* TX preamble code */
    9,                /* RX preamble code */
    1,                /* SFD type (0 = standard, 1 = non-standard) */
    DWT_BR_6M8,       /* Data rate 6.8 Mbps */
    DWT_PHRMODE_STD,  /* PHR mode */
    DWT_PHRRATE_STD,  /* PHR rate */
    (129 + 8 - 8),    /* SFD timeout */
    DWT_STS_MODE_OFF, /* STS désactivé */
    DWT_STS_LEN_64,   /* STS length */
    DWT_PDOA_M0       /* PDOA mode off */
};

// ============================================================================
//  FONCTIONS UTILITAIRES
// ============================================================================

/* Note : get_tx_timestamp_u64() et get_rx_timestamp_u64() sont fournies par
 * la bibliothèque Makerfabs (dw3000_shared_functions.h) — on utilise celles-là
 * directement, pas de redéclaration locale.
 */

/**
 * Calcul de distance DS-TWR (Double-Sided Two-Way Ranging)
 *
 * Utilise les 6 timestamps pour annuler l'erreur de dérive d'horloge :
 *
 *   TAG                          ANCHOR
 *   t1 (poll_tx) ──── POLL ────> t2 (poll_rx)
 *   t4 (resp_rx) <─── RESP ──── t3 (resp_tx)
 *   t5 (final_tx) ─── FINAL ──> t6 (final_rx)
 *
 * R1 = t4 - t1  (round-trip 1, côté Tag)
 * D1 = t3 - t2  (délai réponse, côté Anchor)
 * R2 = t6 - t3  (round-trip 2, côté Anchor)
 * D2 = t5 - t4  (délai final, côté Tag)
 *
 * ToF = (R1 × R2 - D1 × D2) / (R1 + R2 + D1 + D2)
 */
static double compute_distance_dstwr(uint64_t poll_tx_ts, uint64_t poll_rx_ts,
                                      uint64_t resp_tx_ts, uint64_t resp_rx_ts,
                                      uint64_t final_tx_ts, uint64_t final_rx_ts) {
    // Round-trip et délais
    double Ra = (double)((uint32_t)(resp_rx_ts - poll_tx_ts));   // R1 (Tag)
    double Db = (double)((uint32_t)(resp_tx_ts - poll_rx_ts));   // D1 (Anchor)
    double Rb = (double)((uint32_t)(final_rx_ts - resp_tx_ts));  // R2 (Anchor)
    double Da = (double)((uint32_t)(final_tx_ts - resp_rx_ts));  // D2 (Tag)

    // Formule DS-TWR
    double tof_dtu = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db);

    // Conversion en mètres
    double tof_seconds = tof_dtu * DWT_TIME_UNITS;
    double distance = tof_seconds * SPEED_OF_LIGHT;

    return distance;
}

/**
 * Affichage du statut sur le port série
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
    Serial.printf("[%s #%02X] Distance: %.2f m | Mesure #%lu | Erreurs: %lu\n",
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
    Serial.println("  Mode : ANCHOR (Répondeur)");
#else
    Serial.println("  Mode : TAG (Initiateur)");
#endif
    Serial.printf("  Node ID : 0x%02X\n", NODE_ID);
    Serial.println("========================================\n");

    // --- Init SPI + pins (bypass de spiSelect qui contient du code DW1000) ---
    print_status("Reset DW3000 + init SPI...");

    // Pins de contrôle
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_IRQ, INPUT);

    // SPI bus
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

    // Mémoriser les pins dans les globales internes de la lib
    // (sinon readBytes/writetospi togglent GPIO0 au lieu de PIN_CS)
    _ss  = PIN_CS;
    _rst = PIN_RST;
    _irq = PIN_IRQ;

    // Reset matériel DW3000 — RST est open-drain, NE PAS le driver HIGH,
    // on le repasse en INPUT (pull-up interne du DW3000 le ramène à VDDIO)
    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(2);
    pinMode(PIN_RST, INPUT);
    delay(10);  // Transition INIT_RC

    // --- Initialisation du DW3000 ---
    print_status("Initialisation DW3000...");

    // Attendre IDLE_RC
    int idle_retries = 0;
    while (!dwt_checkidlerc()) {
        Serial.print(".");
        delay(50);
        if (++idle_retries > 40) {
            Serial.println(" TIMEOUT idle_rc");
            print_status("ERREUR: DW3000 ne passe pas en IDLE_RC");
            print_status("Vérifier alim 3.3V, MISO, condensateur 100nF.");
            while (1) { delay(1000); }
        }
    }
    Serial.println(" OK");

    // Init (driver Decawave DW3000 — c'est ÇA qui fait la vraie init)
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        print_status("ERREUR: Initialisation DW3000 échouée !");
        print_status("Device ID lu incorrect — vérifier alim 3.3V et MISO.");
        while (1) { delay(1000); }
    }
    print_status("DW3000 initialisé avec succès");

    // Activer les LEDs du DW3000 (utile pour le debug)
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    // --- Configuration UWB ---
    if (dwt_configure(&config)) {
        print_status("ERREUR: Configuration UWB échouée !");
        while (1) { delay(1000); }
    }
    print_status("Configuration UWB: Canal 5 | 6.8 Mbps | PRF 64 MHz");

    // --- Délais d'antenne ---
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    // --- Configuration spécifique au mode ---
#ifdef MODE_TAG
    // Le tag attend la réponse avec un timeout
    dwt_setrxtimeout(RESP_RX_TIMEOUT);
    dwt_setpreambledetecttimeout(0);  // Pas de timeout sur le préambule
    print_status("Tag prêt - Début du ranging DS-TWR");
#else
    // L'ancre écoute en continu (pas de timeout pour le POLL)
    dwt_setrxtimeout(0);
    print_status("Ancre prête - En écoute DS-TWR...");
#endif

    Serial.println();
}

// ============================================================================
//  LOOP - MODE TAG (Initiateur DS-TWR)
//  Phase 1 : Envoyer POLL
//  Phase 2 : Attendre RESPONSE
//  Phase 3 : Envoyer FINAL (avec timestamps)
//  Phase 4 : Attendre RESULT (distance calculée par l'Anchor)
// ============================================================================
#ifdef MODE_TAG

void loop() {
    // === Phase 1 : Envoyer POLL ===
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

    // Envoyer POLL et activer le RX pour la RESPONSE
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    status = STATUS_POLLING;

    // === Phase 2 : Attendre la RESPONSE ===
    uint32_t status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // Attente active
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        // Timeout ou erreur sur RESPONSE
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        error_count++;
        if (status_reg & SYS_STATUS_ALL_RX_TO) {
            print_status("Timeout - pas de RESPONSE de l'ancre");
        } else {
            print_status("Erreur réception RESPONSE");
        }
        status = STATUS_ERROR;
        frame_seq_nb++;
        delay(RANGING_INTERVAL);
        return;
    }

    // Bonne réception RESPONSE
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) {
        frame_seq_nb++;
        delay(RANGING_INTERVAL);
        return;
    }
    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;

    if (rx_buffer[9] != 0x10) {  // Pas un function code RESPONSE
        frame_seq_nb++;
        delay(RANGING_INTERVAL);
        return;
    }

    status = STATUS_WAITING_RESP;

    // === Phase 3 : Envoyer FINAL ===
    // Lire les timestamps du POLL TX et RESPONSE RX
    uint64_t poll_tx_ts = get_tx_timestamp_u64();
    uint64_t resp_rx_ts = get_rx_timestamp_u64();

    // Calculer le moment d'envoi du FINAL (delayed TX)
    uint32_t final_tx_time = (resp_rx_ts +
        (RESP_RX_TO_FINAL_TX_DLY * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);

    // Timestamp prévu du FINAL TX (pour l'insérer dans la trame)
    uint64_t final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    // Insérer les 3 timestamps du Tag dans la trame FINAL (32-bit tronqués)
    uint32_t poll_tx_ts_32  = (uint32_t)poll_tx_ts;
    uint32_t resp_rx_ts_32  = (uint32_t)resp_rx_ts;
    uint32_t final_tx_ts_32 = (uint32_t)final_tx_ts;

    memcpy(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX],  &poll_tx_ts_32,  4);
    memcpy(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX],  &resp_rx_ts_32,  4);
    memcpy(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts_32, 4);

    tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);

    // Configurer le timeout RX pour le RESULT
    dwt_setrxtimeout(RESULT_RX_TIMEOUT);

    // Envoyer FINAL delayed et activer le RX pour le RESULT
    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS) {
        print_status("Erreur: envoi FINAL échoué (trop tard)");
        // Restaurer le timeout pour la prochaine RESPONSE
        dwt_setrxtimeout(RESP_RX_TIMEOUT);
        error_count++;
        status = STATUS_ERROR;
        frame_seq_nb++;
        delay(RANGING_INTERVAL);
        return;
    }

    status = STATUS_SENDING_FINAL;

    // === Phase 4 : Attendre le RESULT (optionnel) ===
    status = STATUS_WAITING_RESULT;

    status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // Attente active
    }

    // Restaurer le timeout pour la prochaine RESPONSE
    dwt_setrxtimeout(RESP_RX_TIMEOUT);

    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        // Bonne réception RESULT
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
        if (frame_len <= sizeof(rx_buffer)) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (rx_buffer[9] == 0x11) {  // Function code RESULT
                float distance;
                memcpy(&distance, &rx_buffer[RESULT_MSG_DIST_IDX], sizeof(float));

                if (distance > 0 && distance < 300.0) {
                    last_distance = (double)distance;
                    print_distance(last_distance);
                    status = STATUS_RANGING_OK;
                } else {
                    print_status("Distance hors limites - ignorée");
                    error_count++;
                    status = STATUS_ERROR;
                }
            }
        }
    } else {
        // Timeout ou erreur - le RESULT est optionnel, pas critique
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
    }

    // Incrémenter le sequence number
    frame_seq_nb++;

    // Attendre avant la prochaine mesure
    delay(RANGING_INTERVAL);
}

#endif // MODE_TAG

// ============================================================================
//  LOOP - MODE ANCHOR (Répondeur DS-TWR)
//  Phase 1 : Écouter → recevoir POLL
//  Phase 2 : Envoyer RESPONSE delayed (+ activer RX pour FINAL)
//  Phase 3 : Attendre FINAL
//  Phase 4 : Calculer la distance DS-TWR (6 timestamps)
//  Phase 5 : Envoyer RESULT au Tag
// ============================================================================
#ifdef MODE_ANCHOR

void loop() {
    // === Phase 1 : Activer le récepteur et attendre un POLL ===
    dwt_setrxtimeout(0);  // Pas de timeout pour l'écoute POLL
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    status = STATUS_LISTENING;

    uint32_t status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // Attente active
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        // Erreur de réception
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return;
    }

    // Bonne réception
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) return;
    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;

    if (rx_buffer[9] != 0x21) return;  // Pas un POLL → ignorer

    // === Phase 2 : Envoyer RESPONSE delayed ===
    // Lire le timestamp de réception du POLL
    uint64_t poll_rx_ts = get_rx_timestamp_u64();

    // Calculer le timestamp d'envoi de la RESPONSE (delayed)
    uint32_t resp_tx_time = (poll_rx_ts +
        (POLL_RX_TO_RESP_TX_DLY * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);

    // Configurer le timeout RX pour le FINAL (avant le starttx)
    dwt_setrxtimeout(FINAL_RX_TIMEOUT);

    tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
    dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

    // Envoyer RESPONSE delayed et activer le RX pour le FINAL
    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if (ret != DWT_SUCCESS) {
        print_status("Erreur: envoi RESPONSE échoué (trop tard)");
        error_count++;
        status = STATUS_ERROR;
        return;
    }

    status = STATUS_RESPONDING;

    // Attendre la fin de l'envoi de la RESPONSE pour lire le timestamp TX réel
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

    // Lire le timestamp TX réel de la RESPONSE (plus précis que le calcul)
    uint64_t resp_tx_ts = get_tx_timestamp_u64();

    // === Phase 3 : Attendre le FINAL ===
    status = STATUS_WAITING_FINAL;

    status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
        // Attente active
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        // Timeout ou erreur sur FINAL
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        if (status_reg & SYS_STATUS_ALL_RX_TO) {
            print_status("Timeout - pas de FINAL du tag");
        } else {
            print_status("Erreur réception FINAL");
        }
        error_count++;
        status = STATUS_ERROR;
        frame_seq_nb++;
        return;
    }

    // Bonne réception FINAL
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) {
        frame_seq_nb++;
        return;
    }
    dwt_readrxdata(rx_buffer, frame_len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;

    if (rx_buffer[9] != 0x23) {  // Pas un FINAL → ignorer
        frame_seq_nb++;
        return;
    }

    // === Phase 4 : Calculer la distance DS-TWR ===
    // Lire le timestamp de réception du FINAL
    uint64_t final_rx_ts = get_rx_timestamp_u64();

    // Extraire les 3 timestamps du Tag depuis la trame FINAL
    uint32_t poll_tx_ts_32, resp_rx_ts_32, final_tx_ts_32;
    memcpy(&poll_tx_ts_32,  &rx_buffer[FINAL_MSG_POLL_TX_TS_IDX],  4);
    memcpy(&resp_rx_ts_32,  &rx_buffer[FINAL_MSG_RESP_RX_TS_IDX],  4);
    memcpy(&final_tx_ts_32, &rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], 4);

    // Reconstruire les timestamps 64-bit (32-bit suffisent pour les différences)
    uint64_t poll_tx_ts  = (uint64_t)poll_tx_ts_32;
    uint64_t resp_rx_ts  = (uint64_t)resp_rx_ts_32;
    uint64_t final_tx_ts = (uint64_t)final_tx_ts_32;

    // Calculer la distance avec les 6 timestamps
    double distance = compute_distance_dstwr(poll_tx_ts, poll_rx_ts,
                                              resp_tx_ts, resp_rx_ts,
                                              final_tx_ts, final_rx_ts);

    // Filtrer les valeurs aberrantes
    if (distance > 0 && distance < 300.0) {
        last_distance = distance;
        print_distance(distance);
        status = STATUS_RANGING_OK;
    } else {
        print_status("Distance hors limites - ignorée");
        error_count++;
        status = STATUS_ERROR;
    }

    // === Phase 5 : Envoyer RESULT au Tag ===
    float dist_float = (float)distance;
    memcpy(&tx_result_msg[RESULT_MSG_DIST_IDX], &dist_float, sizeof(float));
    tx_result_msg[ALL_MSG_SN_IDX] = frame_seq_nb;

    dwt_writetxdata(sizeof(tx_result_msg), tx_result_msg, 0);
    dwt_writetxfctrl(sizeof(tx_result_msg), 0, 1);

    dwt_starttx(DWT_START_TX_IMMEDIATE);

    // Attendre la fin de l'envoi du RESULT
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK)) {}
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);

    frame_seq_nb++;
}

#endif // MODE_ANCHOR
