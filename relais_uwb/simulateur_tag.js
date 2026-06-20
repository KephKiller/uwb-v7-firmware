'use strict';
/*
 * ============================================================================
 *  Simulateur de tag UWB
 * ============================================================================
 *  Envoie au relais des datagrammes UDP factices, au même format que le vrai
 *  tag ESP32. Permet de tester toute la chaîne relais -> navigateur SANS
 *  matériel UWB.
 *
 *  Usage :  node simulateur_tag.js [hôte] [port]
 *           (défaut : 127.0.0.1 8080)
 * ============================================================================
 */
const dgram = require('dgram');

const HOTE = process.argv[2] || '127.0.0.1';
const PORT = Number(process.argv[3]) || 8080;

// Ancres : mêmes positions que le repère orthonormé de la visualisation 3D
const ANCRES = [
  { x: 0, y: 0, z: 0 },   // A1 — origine
  { x: 6, y: 0, z: 0 },   // A2 — axe X
  { x: 0, y: 5, z: 0 },   // A3 — axe Y
];
const BRUIT_M    = 0.012;   // écart-type du bruit de distance (m)
const PERIODE_MS = 100;     // période d'émission (~10 Hz, comme le vrai tag)

const sock = dgram.createSocket('udp4');

// Filtre moyenneur glissant par ancre — le vrai tag ESP32 lisse aussi les distances
const TAILLE_FILTRE = 5;
const filtres = ANCRES.map(() => []);
function filtrer(idx, valeur) {
  const f = filtres[idx];
  f.push(valeur);
  if (f.length > TAILLE_FILTRE) f.shift();
  return f.reduce((s, v) => s + v, 0) / f.length;
}

// Bruit gaussien (Box-Muller)
function gauss() {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

let t = 0;
console.log('[SIM] tag factice -> ' + HOTE + ':' + PORT + '   (Ctrl+C pour arrêter)');

setInterval(() => {
  t += PERIODE_MS / 1000;

  // Trajectoire douce, à l'intérieur du triangle des ancres
  const p = {
    x: 2.00 + 1.05 * Math.sin(0.42 * t),
    y: 1.55 + 0.78 * Math.sin(0.29 * t + 1.1),
    z: 1.35 + 0.80 * Math.sin(0.21 * t + 0.6),
  };

  // Distances vraies + bruit de mesure
  const d = ANCRES.map((a, i) => {
    const vraie = Math.hypot(p.x - a.x, p.y - a.y, p.z - a.z);
    const brut  = vraie + gauss() * BRUIT_M;
    return Math.round(filtrer(i, brut) * 1000) / 1000;   // moyenne glissante
  });

  const datagramme = JSON.stringify({ tag: 1, d: d, t: Date.now() });
  sock.send(datagramme, PORT, HOTE, (err) => {
    if (err) console.error('[SIM] erreur d\'envoi : ' + err.message);
  });
}, PERIODE_MS);
