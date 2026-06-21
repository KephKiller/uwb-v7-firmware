'use strict';
/*
 * ============================================================================
 *  UWB tag simulator
 * ============================================================================
 *  Sends fake UDP datagrams to the relay, in the same format as the real
 *  ESP32 tag. Allows testing the whole relay -> browser chain WITHOUT UWB
 *  hardware.
 *
 *  Usage:  node simulateur_tag.js [host] [port]
 *          (default: 127.0.0.1 8080)
 * ============================================================================
 */
const dgram = require('dgram');

const HOTE = process.argv[2] || '127.0.0.1';
const PORT = Number(process.argv[3]) || 8080;

// Anchors: same positions as the orthonormal frame of the 3D visualization
const ANCRES = [
  { x: 0, y: 0, z: 0 },   // A1 — origin
  { x: 6, y: 0, z: 0 },   // A2 — X axis
  { x: 0, y: 5, z: 0 },   // A3 — Y axis
];
const BRUIT_M    = 0.012;   // standard deviation of the distance noise (m)
const PERIODE_MS = 100;     // emission period (~10 Hz, like the real tag)

const sock = dgram.createSocket('udp4');

// Moving average filter per anchor — the real ESP32 tag also smooths the distances
const TAILLE_FILTRE = 5;
const filtres = ANCRES.map(() => []);
function filtrer(idx, valeur) {
  const f = filtres[idx];
  f.push(valeur);
  if (f.length > TAILLE_FILTRE) f.shift();
  return f.reduce((s, v) => s + v, 0) / f.length;
}

// Gaussian noise (Box-Muller)
function gauss() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

let t = 0;
console.log('[SIM] fake tag -> ' + HOTE + ':' + PORT + '   (Ctrl+C to stop)');

setInterval(() => {
  t += PERIODE_MS / 1000;

  // Smooth trajectory, inside the anchor triangle
  const p = {
    x: 2.00 + 1.05 * Math.sin(0.42 * t),
    y: 1.55 + 0.78 * Math.sin(0.29 * t + 1.1),
    z: 1.35 + 0.80 * Math.sin(0.21 * t + 0.6),
  };

  // True distances + measurement noise
  const d = ANCRES.map((a, i) => {
    const vraie = Math.hypot(p.x - a.x, p.y - a.y, p.z - a.z);
    const brut  = vraie + gauss() * BRUIT_M;
    return Math.round(filtrer(i, brut) * 1000) / 1000;   // moving average
  });

  const datagramme = JSON.stringify({ tag: 1, d: d, t: Date.now() });
  sock.send(datagramme, PORT, HOTE, (err) => {
    if (err) console.error('[SIM] send error : ' + err.message);
  });
}, PERIODE_MS);
