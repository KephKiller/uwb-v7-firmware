# Relais UWB — UDP → SSE

Relais entre le tag UWB (ESP32) et la visualisation 3D dans le navigateur.
Le tag envoie ses mesures en **UDP** ; le relais les rediffuse en flux **SSE**
(Server-Sent Events) et sert la page de visualisation.

```
Tag ESP32  ──UDP──►  relais.js  ──SSE──►  navigateur (visualisation 3D)
```

## Prérequis

Node.js ≥ 16. **Aucune dépendance à installer** (modules natifs uniquement).

## Lancement

```
node relais.js
```

Puis ouvrir **http://localhost:8090/** — la visualisation est servie par le relais
et se connecte automatiquement au flux temps réel.

## Test sans matériel

Dans un second terminal, lancer le simulateur de tag :

```
node simulateur_tag.js
```

Il émet des mesures factices ; la visualisation passe en mode « EN DIRECT ».

## Ports

| Port | Protocole | Rôle |
|------|-----------|------|
| 8080 | UDP | datagrammes du tag : `{"tag":1,"d":[d1,d2,d3],"t":...}` |
| 8090 | HTTP | visualisation (`/`), flux SSE (`/flux`), état (`/sante`) |

Surcharge possible : `PORT_UDP=9000 PORT_HTTP=9090 node relais.js`

## Format du message attendu

Datagramme UDP, texte JSON :

```json
{ "tag": 1, "d": [3.21, 4.05, 3.88], "t": 1234567 }
```

- `tag` — identifiant du tag
- `d`   — distances mesurées vers les ancres A1, A2, A3 (mètres)
- `t`   — horodatage du tag (`millis()`)

La trilatération 3D est effectuée par le navigateur. Voir
`documentation_donnees_reelles.html` pour la chaîne complète.
