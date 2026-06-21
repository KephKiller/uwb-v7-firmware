# UWB Relay — UDP → SSE

Relay between the UWB tag (ESP32) and the 3D visualization in the browser.
The tag sends its measurements over **UDP**; the relay rebroadcasts them as an **SSE**
(Server-Sent Events) stream and serves the visualization page.

```
Tag ESP32  ──UDP──►  relais.js  ──SSE──►  navigateur (visualisation 3D)
```

## Requirements

Node.js ≥ 16. **No dependencies to install** (native modules only).

## Launch

```
node relais.js
```

Then open **http://localhost:8090/** — the visualization is served by the relay
and connects automatically to the real-time stream.

## Testing without hardware

In a second terminal, run the tag simulator:

```
node simulateur_tag.js
```

It emits fake measurements; the visualization switches to "LIVE" mode.

## Ports

| Port | Protocol | Role |
|------|-----------|------|
| 8080 | UDP | tag datagrams: `{"tag":1,"d":[d1,d2,d3],"t":...}` |
| 8090 | HTTP | visualization (`/`), SSE stream (`/flux`), state (`/sante`) |

Overridable: `PORT_UDP=9000 PORT_HTTP=9090 node relais.js`

## Expected message format

UDP datagram, JSON text:

```json
{ "tag": 1, "d": [3.21, 4.05, 3.88], "t": 1234567 }
```

- `tag` — tag identifier
- `d`   — distances measured to anchors A1, A2, A3 (meters)
- `t`   — tag timestamp (`millis()`)

3D trilateration is performed by the browser. See
`documentation_donnees_reelles.html` for the complete chain.
