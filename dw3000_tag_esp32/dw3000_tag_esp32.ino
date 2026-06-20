/*
 * ============================================================================
 *  DW3000 UWB - TAG mobile (initiateur DS-TWR multi-ancres)
 *  ESP32-DevKitC + module DWM3000
 * ============================================================================
 *
 *  Sketch AUTONOME pour le TAG MOBILE. A chaque cycle, le tag interroge
 *  successivement les 3 ancres (ID 1, 2, 3), recupere les 3 distances, les
 *  lisse, puis les envoie en UDP (JSON) au relais PC.
 *
 *      Tag  --DS-TWR-->  Ancres        Tag  --WiFi/UDP-->  relais_uwb/relais.js
 *
 *  Le relais rediffuse en SSE ; la trilateration 3D est faite par le
 *  navigateur (visualisation_3d_positionnement.html). Voir
 *  documentation_donnees_reelles.html pour la chaine complete.
 *
 *  >>> AVANT DE FLASHER : renseigner WIFI_SSID / WIFI_PASSWORD / RELAY_IP <<<
 *
 *  Cablage ESP32 (voir schema_cablage_esp32.html) :
 *    GPIO18->CLK  GPIO23->MOSI  GPIO19->MISO  GPIO5->CS  GPIO27->IRQ  GPIO26->RST
 *
 *  Bibliotheque : https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000
 * ============================================================================
 */

#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "dw3000.h"

extern uint8_t _ss;
extern uint8_t _rst;
extern uint8_t _irq;

// ============================================================================
//  >>> CONFIGURATION - A RENSEIGNER <<<
// ============================================================================
#define TAG_ID          1                 // Identifiant de ce tag

#define WIFI_SSID       "VOTRE_SSID"         // Reseau WiFi
#define WIFI_PASSWORD   "VOTRE_MOT_DE_PASSE"
#define RELAY_IP        "192.168.1.100"    // IP du PC qui execute relais.js
#define RELAY_PORT      8080               // Port UDP du relais

// Ancres interrogees, dans l'ordre (doit correspondre a A1, A2, A3 cote visu)
static const uint8_t ANCHOR_IDS[3] = { 1, 2, 3 };

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
#define TX_ANT_DLY              16385      // Delai d'antenne - A CALIBRER
#define RX_ANT_DLY              16385

#define RESP_RX_TO_FINAL_TX_DLY 3500       // RESPONSE RX -> FINAL TX (us)
#define RESP_RX_TIMEOUT         5000       // attente de la RESPONSE (us)
#define RESULT_RX_TIMEOUT       5000       // attente du RESULT (us)

#define INTER_ANCRE_MS          8          // pause entre 2 ancres (ms)
#define CYCLE_MS                25         // pause entre 2 cycles complets (ms)

// ============================================================================
//  TRAMES UWB  -  l'octet [10] porte l'identifiant de l'ancre cible
// ============================================================================
#define MSG_SN_IDX          2
#define MSG_FN_IDX          9
#define MSG_ANCHOR_IDX      10

#define FN_POLL             0x21
#define FN_RESP             0x10
#define FN_FINAL            0x23
#define FN_RESULT           0x11

#define FINAL_POLL_TX_IDX   11
#define FINAL_RESP_RX_IDX   15
#define FINAL_FINAL_TX_IDX  19
#define RESULT_DIST_IDX     11

// POLL : Tag -> Ancre
static uint8_t tx_poll_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
    FN_POLL, 0, 0, 0
};

// FINAL : Tag -> Ancre (contient les 3 timestamps du Tag)
static uint8_t tx_final_msg[] = {
    0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
    FN_FINAL, 0,
    0, 0, 0, 0,           // [11-14] poll_tx_ts
    0, 0, 0, 0,           // [15-18] resp_rx_ts
    0, 0, 0, 0,           // [19-22] final_tx_ts
    0, 0
};

#define RX_BUF_LEN 32
static uint8_t rx_buffer[RX_BUF_LEN];

static uint8_t  frame_seq_nb  = 0;
static uint32_t cycle_count   = 0;

// ============================================================================
//  CONFIGURATION DW3000  (canal 5, 6.8 Mbps, PRF 64 MHz, STS off)
// ============================================================================
static dwt_config_t config = {
    5, DWT_PLEN_128, DWT_PAC8, 9, 9, 1, DWT_BR_6M8,
    DWT_PHRMODE_STD, DWT_PHRRATE_STD, (129 + 8 - 8),
    DWT_STS_MODE_OFF, DWT_STS_LEN_64, DWT_PDOA_M0
};

// ============================================================================
//  FILTRE MOYENNEUR GLISSANT  (lisse les distances, 5 echantillons par ancre)
// ============================================================================
#define FILTER_SIZE 5

typedef struct {
    float samples[FILTER_SIZE];
    int   index;
    int   count;
    float sum;
} moving_avg_t;

static moving_avg_t filtres[3];

void filter_init(moving_avg_t* f) {
    memset(f->samples, 0, sizeof(f->samples));
    f->index = 0;
    f->count = 0;
    f->sum   = 0;
}

float filter_update(moving_avg_t* f, float v) {
    if (f->count >= FILTER_SIZE) f->sum -= f->samples[f->index];
    f->samples[f->index] = v;
    f->sum += v;
    f->index = (f->index + 1) % FILTER_SIZE;
    if (f->count < FILTER_SIZE) f->count++;
    return f->sum / f->count;
}

// ============================================================================
//  WiFi / UDP
// ============================================================================
static WiFiUDP udp;
static bool wifi_ok = false;

void wifi_connect() {
    Serial.printf("[TAG #%d] Connexion WiFi a %s", TAG_ID, WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    int t = 0;
    while (WiFi.status() != WL_CONNECTED && t < 30) {
        delay(500);
        Serial.print(".");
        t++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        wifi_ok = true;
        Serial.printf(" OK - IP %s\n", WiFi.localIP().toString().c_str());
    } else {
        wifi_ok = false;
        Serial.println(" ECHEC (le tag continue le ranging sans envoi reseau)");
    }
}

void envoyer_distances(double d0, double d1, double d2) {
    if (WiFi.status() != WL_CONNECTED) { wifi_ok = false; return; }
    char json[128];
    snprintf(json, sizeof(json),
        "{\"tag\":%d,\"d\":[%.3f,%.3f,%.3f],\"t\":%lu}",
        TAG_ID, d0, d1, d2, millis());
    udp.beginPacket(RELAY_IP, RELAY_PORT);
    udp.write((const uint8_t*)json, strlen(json));
    udp.endPacket();
}

// ============================================================================
//  MESURE DS-TWR vers UNE ancre
//  Retourne true si la distance a ete obtenue ; la place dans *dist_out.
// ============================================================================
bool mesurer_ancre(uint8_t anchor_id, double* dist_out) {
    // --- POLL ---
    dwt_setrxtimeout(RESP_RX_TIMEOUT);

    tx_poll_msg[MSG_SN_IDX]     = frame_seq_nb;
    tx_poll_msg[MSG_ANCHOR_IDX] = anchor_id;
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    // --- Attendre la RESPONSE ---
    uint32_t status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
    }

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        frame_seq_nb++;
        return false;   // ancre absente ou hors de portee
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    uint32_t frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) { frame_seq_nb++; return false; }
    dwt_readrxdata(rx_buffer, frame_len, 0);

    if (rx_buffer[MSG_FN_IDX] != FN_RESP ||
        rx_buffer[MSG_ANCHOR_IDX] != anchor_id) {
        frame_seq_nb++;
        return false;
    }

    // --- FINAL : inserer les 3 timestamps du Tag ---
    uint64_t poll_tx_ts = get_tx_timestamp_u64();
    uint64_t resp_rx_ts = get_rx_timestamp_u64();

    uint32_t final_tx_time = (resp_rx_ts +
        (RESP_RX_TO_FINAL_TX_DLY * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);

    uint64_t final_tx_ts =
        (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    uint32_t poll_tx_32  = (uint32_t)poll_tx_ts;
    uint32_t resp_rx_32  = (uint32_t)resp_rx_ts;
    uint32_t final_tx_32 = (uint32_t)final_tx_ts;
    memcpy(&tx_final_msg[FINAL_POLL_TX_IDX],   &poll_tx_32,  4);
    memcpy(&tx_final_msg[FINAL_RESP_RX_IDX],   &resp_rx_32,  4);
    memcpy(&tx_final_msg[FINAL_FINAL_TX_IDX],  &final_tx_32, 4);

    tx_final_msg[MSG_SN_IDX]     = frame_seq_nb;
    tx_final_msg[MSG_ANCHOR_IDX] = anchor_id;
    dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
    dwt_writetxfctrl(sizeof(tx_final_msg), 0, 1);

    dwt_setrxtimeout(RESULT_RX_TIMEOUT);
    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (ret != DWT_SUCCESS) {
        dwt_setrxtimeout(RESP_RX_TIMEOUT);
        frame_seq_nb++;
        return false;   // FINAL envoye trop tard
    }

    // --- Attendre le RESULT (distance calculee par l'ancre) ---
    status_reg = 0;
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
    }
    dwt_setrxtimeout(RESP_RX_TIMEOUT);
    frame_seq_nb++;

    if (!(status_reg & SYS_STATUS_RXFCG_BIT_MASK)) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        return false;
    }
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);

    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len > sizeof(rx_buffer)) return false;
    dwt_readrxdata(rx_buffer, frame_len, 0);

    if (rx_buffer[MSG_FN_IDX] != FN_RESULT ||
        rx_buffer[MSG_ANCHOR_IDX] != anchor_id) {
        return false;
    }

    float distance;
    memcpy(&distance, &rx_buffer[RESULT_DIST_IDX], sizeof(float));
    if (distance <= 0 || distance >= 300.0f) return false;

    *dist_out = (double)distance;
    return true;
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n========================================");
    Serial.println("  DW3000 UWB - TAG DS-TWR multi-ancres");
    Serial.printf("  Tag ID : %d  |  Ancres : %d, %d, %d\n",
                  TAG_ID, ANCHOR_IDS[0], ANCHOR_IDS[1], ANCHOR_IDS[2]);
    Serial.println("========================================\n");

    for (int i = 0; i < 3; i++) filter_init(&filtres[i]);

    // --- WiFi ---
    wifi_connect();

    // --- Init SPI + reset DW3000 ---
    Serial.printf("[TAG #%d] Reset DW3000 + init SPI...\n", TAG_ID);
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, HIGH);
    pinMode(PIN_IRQ, INPUT);
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

    _ss  = PIN_CS;
    _rst = PIN_RST;
    _irq = PIN_IRQ;

    pinMode(PIN_RST, OUTPUT);
    digitalWrite(PIN_RST, LOW);
    delay(2);
    pinMode(PIN_RST, INPUT);
    delay(10);

    // --- Init DW3000 ---
    int idle_retries = 0;
    while (!dwt_checkidlerc()) {
        Serial.print(".");
        delay(50);
        if (++idle_retries > 40) {
            Serial.printf("\n[TAG #%d] ERREUR: DW3000 ne passe pas en IDLE_RC\n", TAG_ID);
            Serial.println("Verifier alim 3.3V, MISO, condensateur 100nF.");
            while (1) { delay(1000); }
        }
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        Serial.printf("[TAG #%d] ERREUR: Initialisation DW3000 echouee !\n", TAG_ID);
        while (1) { delay(1000); }
    }

    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    if (dwt_configure(&config)) {
        Serial.printf("[TAG #%d] ERREUR: Configuration UWB echouee !\n", TAG_ID);
        while (1) { delay(1000); }
    }

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setrxtimeout(RESP_RX_TIMEOUT);
    dwt_setpreambledetecttimeout(0);

    Serial.printf("[TAG #%d] Tag pret - debut du ranging DS-TWR\n\n", TAG_ID);
}

// ============================================================================
//  LOOP - un cycle = ranging des 3 ancres + envoi UDP
// ============================================================================
void loop() {
    double d[3];
    bool   ok[3];

    for (int i = 0; i < 3; i++) {
        double brut = 0;
        if (mesurer_ancre(ANCHOR_IDS[i], &brut)) {
            d[i]  = filter_update(&filtres[i], (float)brut);   // distance lissee
            ok[i] = true;
        } else {
            d[i]  = -1.0;                                      // ancre injoignable
            ok[i] = false;
        }
        delay(INTER_ANCRE_MS);
    }

    cycle_count++;

    // Envoi au relais (les distances invalides valent -1 ; la visu les ignore)
    envoyer_distances(d[0], d[1], d[2]);

    // Trace serie (1 cycle sur 10)
    if (cycle_count % 10 == 1) {
        Serial.printf("[TAG #%d] cycle %lu | d1=%s d2=%s d3=%s\n",
            TAG_ID, cycle_count,
            ok[0] ? String(d[0], 2).c_str() : "--",
            ok[1] ? String(d[1], 2).c_str() : "--",
            ok[2] ? String(d[2], 2).c_str() : "--");
    }

    // Reconnexion WiFi si la liaison est tombee
    if (WiFi.status() != WL_CONNECTED) {
        wifi_ok = false;
        static uint32_t dernier_essai = 0;
        if (millis() - dernier_essai > 5000) {
            dernier_essai = millis();
            wifi_connect();
        }
    }

    delay(CYCLE_MS);
}
