'use strict';
/*
 * ============================================================================
 *  UWB Relay — UDP -> SSE
 * ============================================================================
 *  Receives JSON datagrams from the ESP32 tag on a UDP port, rebroadcasts
 *  them to browsers via an SSE (Server-Sent Events) stream, and serves the
 *  3D visualization page.
 *
 *  Node.js >= 16 — NO external dependencies (native http + dgram modules).
 *
 *  Launch:  node relais.js
 *  Then open:  http://localhost:8090/
 * ============================================================================
 */
const http  = require('http');
const dgram = require('dgram');
const fs    = require('fs');
const path  = require('path');

const PORT_UDP  = Number(process.env.PORT_UDP)  || 8080;   // tag datagrams
const PORT_HTTP = Number(process.env.PORT_HTTP) || 8090;   // browser (visu + stream)
const FICHIER_VISU = path.join(__dirname, '..', 'visualisation_3d_positionnement.html');

// Browsers subscribed to the SSE stream (HTTP "response" objects kept open)
const clients = new Set();
let nbMessages = 0;

function diffuser(donnees) {
  const ligne = 'data: ' + JSON.stringify(donnees) + '\n\n';
  for (const c of clients) {
    try { c.write(ligne); } catch (_) { /* disconnected client */ }
  }
}

// ============================================================================
//  UDP reception — datagrams sent by the tag(s)
// ============================================================================
const sock = dgram.createSocket('udp4');

sock.on('message', (buf, info) => {
  const texte = buf.toString('utf8').trim();
  let msg;
  try {
    msg = JSON.parse(texte);
  } catch (_) {
    console.warn('[UDP] non-JSON packet ignored from ' + info.address + ' : ' + texte);
    return;
  }
  nbMessages++;
  msg.recu = Date.now();           // reception timestamp on the relay side
  diffuser(msg);
  if (nbMessages % 25 === 1) {     // reduced trace: 1 line out of 25
    console.log('[UDP] ' + info.address + '  ' + texte
      + '   (' + clients.size + ' browser(s) connected)');
  }
});

sock.on('error', (e) => console.error('[UDP] error : ' + e.message));
sock.bind(PORT_UDP, () => console.log('[UDP]  listening for tag datagrams on port ' + PORT_UDP));

// ============================================================================
//  HTTP server — 3D visualization + SSE stream
// ============================================================================
const serveur = http.createServer((req, res) => {
  const chemin = req.url.split('?')[0];

  // --- Real-time SSE stream ---
  if (chemin === '/flux') {
    res.writeHead(200, {
      'Content-Type'                : 'text/event-stream',
      'Cache-Control'               : 'no-cache, no-transform',
      'Connection'                  : 'keep-alive',
      'Access-Control-Allow-Origin' : '*',
    });
    res.write('retry: 3000\n\n');
    res.write(': UWB stream connected\n\n');
    clients.add(res);
    console.log('[SSE] browser connected (' + clients.size + ' total)');
    req.on('close', () => {
      clients.delete(res);
      console.log('[SSE] browser disconnected (' + clients.size + ' remaining)');
    });
    return;
  }

  // --- Relay state (diagnostics) ---
  if (chemin === '/sante') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    res.end(JSON.stringify({
      ok: true, navigateurs: clients.size, messagesRecus: nbMessages, portUDP: PORT_UDP
    }));
    return;
  }

  // --- 3D visualization ---
  if (chemin === '/' || chemin === '/index.html'
      || chemin === '/visualisation_3d_positionnement.html') {
    fs.readFile(FICHIER_VISU, (err, data) => {
      if (err) {
        res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
        res.end('visualisation_3d_positionnement.html not found.\n'
          + 'The relais_uwb/ folder must stay next to the file.');
        return;
      }
      res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' });
      res.end(data);
    });
    return;
  }

  res.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
  res.end('Unknown resource : ' + chemin);
});

serveur.listen(PORT_HTTP, () => {
  console.log('[HTTP] visualization : http://localhost:' + PORT_HTTP + '/');
  console.log('[HTTP] SSE stream     : http://localhost:' + PORT_HTTP + '/flux');
  console.log('');
  console.log('Waiting for tag datagrams…  (test without hardware: node simulateur_tag.js)');
});

// Periodic ping to keep the SSE connections open
setInterval(() => {
  for (const c of clients) {
    try { c.write(': ping\n\n'); } catch (_) { /* ignored */ }
  }
}, 20000);
