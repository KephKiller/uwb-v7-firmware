/*
 * ============================================================================
 *  DW3000 UWB - Trilateration & Calibration Module
 *  Companion file for 2D/3D positioning
 * ============================================================================
 *
 *  This file contains:
 *  - Moving average filter to smooth distances
 *  - Antenna delay calibration
 *  - 2D trilateration from 3+ anchors
 *  - WiFi communication to send the position
 *
 *  To be included in the main sketch (TAG mode only)
 * ============================================================================
 */

#ifndef DW3000_TRILAT_H
#define DW3000_TRILAT_H

#include <WiFi.h>
#include <math.h>

// ============================================================================
//  NETWORK CONFIGURATION
// ============================================================================

// WiFi (to send positions to a server)
#define WIFI_SSID        "TON_SSID"
#define WIFI_PASSWORD    "TON_PASSWORD"
#define SERVER_IP        "192.168.1.100"
#define SERVER_PORT      8080

// ============================================================================
//  ANCHOR CONFIGURATION
// ============================================================================

#define MAX_ANCHORS 8
#define MIN_ANCHORS_FOR_2D 3
#define MIN_ANCHORS_FOR_3D 4

// Anchor positions (measure and fill in precisely!)
// Coordinates in meters relative to an origin point
typedef struct {
    uint8_t   id;           // Anchor identifier
    uint16_t  address;      // UWB address
    float     x, y, z;     // Position (m)
    float     last_dist;    // Last measured distance
    bool      active;       // Anchor available?
    uint32_t  last_seen;    // Last contact (ms)
} anchor_t;

// Example: 4 anchors in a 5m x 5m room
static anchor_t anchors[MAX_ANCHORS] = {
    { 0x01, 0x0001, 0.0f, 0.0f, 0.0f, 0.0f, false, 0 },  // Origin corner
    { 0x02, 0x0002, 5.0f, 0.0f, 0.0f, 0.0f, false, 0 },  // X corner
    { 0x03, 0x0003, 5.0f, 5.0f, 0.0f, 0.0f, false, 0 },  // XY corner
    { 0x04, 0x0004, 0.0f, 5.0f, 0.0f, 0.0f, false, 0 },  // Y corner
};
static int num_anchors = 4;

// ============================================================================
//  MOVING AVERAGE FILTER
// ============================================================================

#define FILTER_SIZE 5  // Number of samples for the average

typedef struct {
    float   samples[FILTER_SIZE];
    int     index;
    int     count;
    float   sum;
} moving_avg_t;

void filter_init(moving_avg_t* f) {
    memset(f->samples, 0, sizeof(f->samples));
    f->index = 0;
    f->count = 0;
    f->sum = 0;
}

float filter_update(moving_avg_t* f, float new_value) {
    // Remove the oldest sample from the sum
    if (f->count >= FILTER_SIZE) {
        f->sum -= f->samples[f->index];
    }

    // Add the new sample
    f->samples[f->index] = new_value;
    f->sum += new_value;
    
    f->index = (f->index + 1) % FILTER_SIZE;
    if (f->count < FILTER_SIZE) f->count++;
    
    return f->sum / f->count;
}

// One filter per anchor
static moving_avg_t dist_filters[MAX_ANCHORS];

// ============================================================================
//  ANTENNA DELAY CALIBRATION
// ============================================================================

/*
 * Calibration procedure:
 * 1. Place the tag at a KNOWN distance from an anchor (e.g. 2.00 m)
 * 2. Call calibrate_antenna_delay(2.0)
 * 3. The function adjusts the antenna delay so that the measurement
 *    matches the real distance
 * 4. Note the value and put it in TX_ANT_DLY / RX_ANT_DLY
 */

#define CALIB_SAMPLES 100  // Number of measurements for the calibration

typedef struct {
    bool     running;
    float    known_distance;    // Real distance (m)
    float    measured_sum;      // Sum of measurements
    int      sample_count;
    uint16_t best_delay;        // Calibration result
} calibration_t;

static calibration_t calib = { false, 0, 0, 0, 0 };

void calibrate_start(float known_distance_m) {
    calib.running = true;
    calib.known_distance = known_distance_m;
    calib.measured_sum = 0;
    calib.sample_count = 0;
    
    Serial.printf("\n[CALIB] Début calibration - Distance connue: %.2f m\n", known_distance_m);
    Serial.printf("[CALIB] Collecte de %d échantillons...\n", CALIB_SAMPLES);
}

bool calibrate_add_sample(float measured_distance) {
    if (!calib.running) return false;
    
    calib.measured_sum += measured_distance;
    calib.sample_count++;
    
    if (calib.sample_count % 10 == 0) {
        Serial.printf("[CALIB] %d/%d échantillons\n", calib.sample_count, CALIB_SAMPLES);
    }
    
    if (calib.sample_count >= CALIB_SAMPLES) {
        float avg_measured = calib.measured_sum / calib.sample_count;
        float error = avg_measured - calib.known_distance;

        // Convert the error into antenna delay ticks
        // error_ticks = error_m / (c * DWT_TIME_UNITS)
        // Divided by 2 because the delay applies to both TX and RX
        float error_ticks = (error / (SPEED_OF_LIGHT * DWT_TIME_UNITS)) / 2.0;
        
        uint16_t current_delay = TX_ANT_DLY;
        int16_t adjustment = (int16_t)error_ticks;
        calib.best_delay = current_delay + adjustment;
        
        Serial.println("\n[CALIB] ====== RÉSULTAT ======");
        Serial.printf("[CALIB] Distance connue : %.3f m\n", calib.known_distance);
        Serial.printf("[CALIB] Distance mesurée: %.3f m\n", avg_measured);
        Serial.printf("[CALIB] Erreur          : %.3f m\n", error);
        Serial.printf("[CALIB] Délai actuel    : %u\n", current_delay);
        Serial.printf("[CALIB] Ajustement      : %d ticks\n", adjustment);
        Serial.printf("[CALIB] >>> NOUVEAU DÉLAI : %u <<<\n", calib.best_delay);
        Serial.println("[CALIB] Mettre à jour TX_ANT_DLY et RX_ANT_DLY dans le code");
        Serial.println("[CALIB] ========================\n");
        
        calib.running = false;
        return true;  // Calibration complete
    }

    return false;  // Not finished yet
}

// ============================================================================
//  2D TRILATERATION
// ============================================================================

typedef struct {
    float x, y;
    float accuracy;   // Accuracy estimate (m)
    bool  valid;
} position_2d_t;

/*
 * Least Squares trilateration
 * Minimum 3 anchors with valid distances
 *
 * Principle:
 * (x - x1)² + (y - y1)² = d1²
 * (x - x2)² + (y - y2)² = d2²
 * ...
 * Linearization by subtracting the first equation
 */
position_2d_t trilaterate_2d(void) {
    position_2d_t pos = { 0, 0, 999, false };

    // Count the active anchors
    int active_count = 0;
    int active_idx[MAX_ANCHORS];

    for (int i = 0; i < num_anchors; i++) {
        if (anchors[i].active &&
            anchors[i].last_dist > 0 &&
            (millis() - anchors[i].last_seen) < 2000) {  // Max 2s age
            active_idx[active_count++] = i;
        }
    }
    
    if (active_count < MIN_ANCHORS_FOR_2D) {
        Serial.printf("[TRILAT] Pas assez d'ancres actives: %d/%d\n", 
                      active_count, MIN_ANCHORS_FOR_2D);
        return pos;
    }
    
    // Reference = first active anchor
    int ref = active_idx[0];
    float x1 = anchors[ref].x;
    float y1 = anchors[ref].y;
    float d1 = anchors[ref].last_dist;

    // Build the linear system Ax = b
    // (n-1) equations, 2 unknowns
    int n = active_count - 1;

    // Matrices (simplified for small n)
    float A[MAX_ANCHORS][2];
    float b[MAX_ANCHORS];
    
    for (int i = 0; i < n; i++) {
        int idx = active_idx[i + 1];
        float xi = anchors[idx].x;
        float yi = anchors[idx].y;
        float di = anchors[idx].last_dist;
        
        A[i][0] = 2.0f * (xi - x1);
        A[i][1] = 2.0f * (yi - y1);
        b[i] = (d1 * d1 - di * di) - (x1 * x1 - xi * xi) - (y1 * y1 - yi * yi);
    }
    
    // Solve by pseudo-inverse (At*A)^-1 * At * b
    // For 2 unknowns, direct computation
    float AtA[2][2] = {{0, 0}, {0, 0}};
    float Atb[2] = {0, 0};
    
    for (int i = 0; i < n; i++) {
        AtA[0][0] += A[i][0] * A[i][0];
        AtA[0][1] += A[i][0] * A[i][1];
        AtA[1][0] += A[i][1] * A[i][0];
        AtA[1][1] += A[i][1] * A[i][1];
        Atb[0] += A[i][0] * b[i];
        Atb[1] += A[i][1] * b[i];
    }
    
    // Determinant
    float det = AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0];
    if (fabsf(det) < 1e-6f) {
        Serial.println("[TRILAT] Matrice singulière - ancres colinéaires ?");
        return pos;
    }

    // 2x2 inverse
    float inv_det = 1.0f / det;
    pos.x = inv_det * (AtA[1][1] * Atb[0] - AtA[0][1] * Atb[1]);
    pos.y = inv_det * (AtA[0][0] * Atb[1] - AtA[1][0] * Atb[0]);

    // Accuracy estimate (mean residual)
    float residual_sum = 0;
    for (int i = 0; i < active_count; i++) {
        int idx = active_idx[i];
        float dx = pos.x - anchors[idx].x;
        float dy = pos.y - anchors[idx].y;
        float estimated_dist = sqrtf(dx * dx + dy * dy);
        float residual = fabsf(estimated_dist - anchors[idx].last_dist);
        residual_sum += residual;
    }
    pos.accuracy = residual_sum / active_count;
    pos.valid = true;
    
    return pos;
}

// ============================================================================
//  SEND POSITION VIA WiFi (UDP)
// ============================================================================

#include <WiFiUdp.h>
static WiFiUDP udp;
static bool wifi_connected = false;

void wifi_init(void) {
    Serial.printf("[WIFI] Connexion à %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int timeout = 0;
    while (WiFi.status() != WL_CONNECTED && timeout < 20) {
        delay(500);
        Serial.print(".");
        timeout++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf(" OK! IP: %s\n", WiFi.localIP().toString().c_str());
        wifi_connected = true;
    } else {
        Serial.println(" ÉCHEC (mode offline)");
        wifi_connected = false;
    }
}

void send_position_udp(position_2d_t* pos) {
    if (!wifi_connected) return;

    // Simple JSON format
    char json[200];
    snprintf(json, sizeof(json),
        "{\"node\":%d,\"x\":%.3f,\"y\":%.3f,\"acc\":%.3f,\"t\":%lu}",
        NODE_ID, pos->x, pos->y, pos->accuracy, millis());
    
    udp.beginPacket(SERVER_IP, SERVER_PORT);
    udp.write((uint8_t*)json, strlen(json));
    udp.endPacket();
}

// ============================================================================
//  SERIAL COMMANDS (for calibration and debug)
// ============================================================================

/*
 * Commands available via the serial monitor:
 *   CALIB 2.00   → Start calibration at 2.00 m
 *   STATUS       → Display the anchor states
 *   WIFI         → Reconnect WiFi
 */
void handle_serial_commands(void) {
    if (!Serial.available()) return;

    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd.startsWith("CALIB ")) {
        float dist = cmd.substring(6).toFloat();
        if (dist > 0.1 && dist < 50.0) {
            calibrate_start(dist);
        } else {
            Serial.println("[CMD] Distance invalide (0.1 - 50.0 m)");
        }
    }
    else if (cmd == "STATUS") {
        Serial.println("\n=== ÉTAT DES ANCRES ===");
        for (int i = 0; i < num_anchors; i++) {
            Serial.printf("  Ancre #%02X [%04X] @ (%.1f, %.1f, %.1f) | dist=%.2f m | %s | age=%lu ms\n",
                anchors[i].id, anchors[i].address,
                anchors[i].x, anchors[i].y, anchors[i].z,
                anchors[i].last_dist,
                anchors[i].active ? "ACTIVE" : "inactive",
                millis() - anchors[i].last_seen);
        }
        Serial.println("=======================\n");
    }
    else if (cmd == "WIFI") {
        wifi_init();
    }
    else {
        Serial.println("[CMD] Commandes: CALIB <dist_m> | STATUS | WIFI");
    }
}

#endif // DW3000_TRILAT_H
