'use strict';
/*
 * ============================================================================
 *  Relais UWB — UDP -> SSE
 * ============================================================================
 *  Reçoit les datagrammes JSON du tag ESP32 sur un port UDP, les rediffuse
 *  aux navigateurs via un flux SSE (Server-Sent Events), et sert la page de
 *  visualisation 3D.
 *
 *  Node.js >= 16 — AUCUNE dépendance externe (modules natifs http + dgram).
 *
 *  Lancement :  node relais.js
 *  Puis ouvrir :  http://localhost:8090/
 * ============================================================================
 */
const http  = require('http');
const dgram = require('dgram');
const fs    = require('fs');
const path  = require('path');

const PORT_UDP  = Number(process.env.PORT_UDP)  || 8080;   // datagrammes du tag
const PORT_HTTP = Number(process.env.PORT_HTTP) || 8090;   // navigateur (visu + flux)
const FICHIER_VISU = path.join(__dirname, '..', 'visualisation_3d_positionnement.html');

// Navigateurs abonnés au flux SSE (objets « response » HTTP gardés ouverts)
const clients = new Set();
let nbMessages = 0;

function diffuser(donnees) {
  const ligne = 'data: ' + JSON.stringify(donnees) + '\n\n';
  for (const c of clients) {
    try { c.write(ligne); } catch (_) { /* client déconnecté */ }
  }
}

// ============================================================================
//  Réception UDP — datagrammes envoyés par le ou les tag(s)
// ============================================================================
const sock = dgram.createSocket('udp4');

sock.on('message', (buf, info) => {
  const texte = buf.toString('utf8').trim();
  let msg;
  try {
    msg = JSON.parse(texte);
  } catch (_) {
    console.warn('[UDP] paquet non-JSON ignoré de ' + info.address + ' : ' + texte);
    return;
  }
  nbMessages++;
  msg.recu = Date.now();           // horodatage de réception côté relais
  diffuser(msg);
  if (nbMessages % 25 === 1) {     // trace allégée : 1 ligne sur 25
    console.log('[UDP] ' + info.address + '  ' + texte
      + '   (' + clients.size + ' navigateur(s) connecté(s))');
  }
});

sock.on('error', (e) => console.error('[UDP] erreur : ' + e.message));
sock.bind(PORT_UDP, () => console.log('[UDP]  écoute des datagrammes du tag sur le port ' + PORT_UDP));

// ============================================================================
//  Serveur HTTP — visualisation 3D + flux SSE
// ============================================================================
const serveur = http.createServer((req, res) => {
  const chemin = req.url.split('?')[0];

  // --- Flux SSE temps réel ---
  if (chemin === '/flux') {
    res.writeHead(200, {
      'Content-Type'                : 'text/event-stream',
      'Cache-Control'               : 'no-cache, no-transform',
      'Connection'                  : 'keep-alive',
      'Access-Control-Allow-Origin' : '*',
    });
    res.write('retry: 3000\n\n');
    res.write(': flux UWB connecté\n\n');
    clients.add(res);
    console.log('[SSE] navigateur connecté (' + clients.size + ' au total)');
    req.on('close', () => {
      clients.delete(res);
      console.log('[SSE] navigateur déconnecté (' + clients.size + ' restant(s))');
    });
    return;
  }

  // --- État du relais (diagnostic) ---
  if (chemin === '/sante') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({
      ok: true, navigateurs: clients.size, messagesRecus: nbMessages, portUDP: PORT_UDP
    }));
    return;
  }

  // --- Visualisation 3D ---
  if (chemin === '/' || chemin === '/index.html'
      || chemin === '/visualisation_3d_positionnement.html') {
    fs.readFile(FICHIER_VISU, (err, data) => {
      if (err) {
        res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
        res.end('visualisation_3d_positionnement.html introuvable.\n'
          + 'Le dossier relais_uwb/ doit rester à côté du fichier.');
        return;
      }
      res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
      res.end(data);
    });
    return;
  }

  res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
  res.end('Ressource inconnue : ' + chemin);
});

serveur.listen(PORT_HTTP, () => {
  console.log('[HTTP] visualisation : http://localhost:' + PORT_HTTP + '/');
  console.log('[HTTP] flux SSE      : http://localhost:' + PORT_HTTP + '/flux');
  console.log('');
  console.log('En attente des datagrammes du tag…  (test sans matériel : node simulateur_tag.js)');
});

// Ping périodique pour maintenir les connexions SSE ouvertes
setInterval(() => {
  for (const c of clients) {
    try { c.write(': ping\n\n'); } catch (_) { /* ignoré */ }
  }
}, 20000);
