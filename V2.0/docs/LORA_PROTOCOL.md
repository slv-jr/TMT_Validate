# LoRa BATTLEBOATS — Mapping PDF officiel ↔ code StormWings

Ce document fait le pont entre **`SwarmZ_fichier_Orga/BATTLEBOATS_LORA_PROTOCOL_v5.md.pdf`** et notre implémentation. Il décrit :

1. La topologie du réseau (qui émet quoi sur quel canal)
2. Le format exact des trames `P|...` et `W|...`
3. Le routage interne dans le code (LoRa → wind/roles/tactique → Cube)
4. Le filtrage selon le mode (`STORMWINGS_MODE=ESSAI|REGATE`)
5. Comment vérifier en course que tout fonctionne

---

## 1. Topologie du réseau

Tous les ESP32 LoRa V3 sont flashés Meshtastic et configurés sur le canal partagé **`BATTLEBOATS`** (868 MHz EU, preset MEDIUM_FAST). Tous les nœuds reçoivent tous les messages — pas de routage applicatif, juste du broadcast.

| Nœud   | Acteur                | Émet                        | Période |
|--------|-----------------------|-----------------------------|---------|
| `WIND` | Station orga (terre)  | `W\|<dir>\|<spd×10>\|<ts>`    | 60 s    |
| `ORG`  | Base orga (plot/score)| (rien — écoute seulement)   | —       |
| `U1B1` | UTT, drone Scout      | `P\|U1B1\|<lat>\|<lon>\|<hdg>\|<spd>` | 60 s |
| `U1B2` | UTT, drone Optimizer  | `P\|U1B2\|<lat>\|<lon>\|<hdg>\|<spd>` | 60 s (T+30) |
| `D2Bn` | DaVinci Hive (×3)     | `P\|D2Bn\|...`              | 60 s    |
| `I3Bn` | IPSA (×3)             | `P\|I3Bn\|...`              | 60 s    |
| `E4Bn` | Enseeiht (×3)         | `P\|E4Bn\|...`              | 60 s    |

**Obligation réglementaire** (cf. PDF §3.3) : un bateau qui n'émet pas `P|...` est invisible et **pénalisé sportivement**. L'émission doit continuer en MANUAL, en AUTO et en PENALITE, et même sans fix GPS (`P|<id>|0|0|0|0`).

---

## 2. Format des trames

### MSG_POS — chaque bateau, 60 s

```
P|<boat_id>|<lat×1e5>|<lon×1e5>|<hdg_deg_3d>|<spd_knots×10_2d>
```

| Champ | Type   | Exemple    | Description                              |
|-------|--------|------------|------------------------------------------|
| `id`  | str    | `U1B1`     | Identifiant officiel (cf. tableau §1)    |
| `lat` | int32  | `4348256`  | Latitude × 100 000 (~1 m de résolution)  |
| `lon` | int32  | `649872`   | Longitude × 100 000                      |
| `hdg` | uint16 | `185`      | Cap magnétique 0-359° (3 chiffres)       |
| `spd` | uint16 | `32`       | Vitesse fond × 10 nœuds (2 chiffres)     |

Exemples conformes (cf. PDF §3.3 et tests `TestProtocol.test_pdf_v5_position_examples`) :

```
P|U1B1|4348256|649872|185|32   → UTT Bateau 1 (3.2 nœuds)
P|D2B2|4347100|650134|092|18   → DaVinci Hive Bateau 2 (1.8 nœuds)
P|I3B3|4348500|649500|270|25   → IPSA Bateau 3 (2.5 nœuds)
P|E4B1|4347800|650200|045|10   → Enseeiht Bateau 1 (1.0 nœud)
P|U1B2|0|0|0|0                 → Pas de fix GPS
```

### MSG_WIND — station orga, 60 s

```
W|<dir_deg_3d>|<spd_ms×10_2d>|<unix_ts>
```

Exemples :
```
W|245|63|1746787652   → vent du SW, 6.3 m/s
W|090|15|1746787657   → vent d'Est, 1.5 m/s
W|000|00|1746787662   → capteur orga hors ligne
```

Convention météo : `0° = Nord`, `90° = Est`, `180° = Sud`, `270° = Ouest`.

---

## 3. Routage dans le code

### Émission `P|...` (TX)

```
Cube Orange (GPS RTK + heading)
     │ MAVLink GLOBAL_POSITION_INT
     ▼
main._tick() → tlm = mav.get_telemetry()
     │
     ▼
main._comm_step()  ─ TDMA gate (60 s, U1B1@T+0, U1B2@T+30) ─
     │
     ▼
LoRaInterface.broadcast_position()
     │
     ▼
protocol.build_position_from_telemetry()  →  PositionMessage.encode()
     │ "P|U1B1|4348256|649872|185|32"
     ▼
meshtastic.sendText() → ESP32 → 868 MHz → tous les nœuds
```

Code clés :
- `main.py::_comm_step` (boucle 10 Hz, broadcast effectif via TDMA)
- `comms/lora_iface.py::broadcast_position`
- `comms/protocol.py::PositionMessage.encode`
- `swarm/tdma.py::TDMAScheduler` (cadence 60 s, offset T+0/T+30)

### Réception `W|...` (orga → nav stack)

```
ESP32 → meshtastic on_receive
     │
     ▼
LoRaInterface._handle_text(text)
     ├── parse_message(text) → WindMessage
     ▼
LoRaInterface.on_wind callback
     │
     ▼
main._on_wind_received(msg)
     │  log.info("[LORA-NET] WIND orga → nav stack : 245°/6.3m/s")
     ▼
WindEstimator.push_orga_wind()
     │  (en ESSAI : ignoré ; en REGATE : ajouté à l'historique lissé)
     ▼
WindEstimator.estimate() → utilisé par
     - navigation/vmg.py        (lay-lines)
     - navigation/polar.py      (vitesse cible)
     - navigation/waypoints.py  (choix bouée au vent en parcours 1)
```

### Réception `P|...` (autres bateaux)

```
ESP32 → on_receive → LoRaInterface._handle_text
     │
     ▼
PositionMessage parsed
     │
     ├── boat_id ∈ TEAM_BOATS (UTT) ?
     │      ├── OUI → on_position callback → main._on_position_received
     │      │        → roles.update_teammate (coordination essaim, scoring)
     │      │
     │      └── NON, ennemi ?
     │             ├── REGATE → main._on_position_received
     │             │            → _last_adversary_pos[bid] (anti-collision +
     │             │              tactical_compute → enemies_ahead/close)
     │             └── ESSAI  → IGNORÉ à la couche LoRa
     ▼
neighbors{} mis à jour pour l'API get_active/get_team/get_enemy_neighbors
```

### Exploitation tactique (REGATE uniquement)

Toutes les 5 s, `main._tactical_step` calcule :

| Indicateur          | Description                                        | Conséquence stratégique                              |
|---------------------|----------------------------------------------------|------------------------------------------------------|
| `enemies_ahead`     | Ennemis dans cône avant entre nous et la cible     | Anticipation des layline conflicts                    |
| `enemies_close`     | Ennemis à < 30 m                                   | Priorité au potential field repulsor                  |
| `blocked_target`    | ≥ 2 ennemis à < 30 m de la prochaine bouée         | Bascule en mode défensif (clearance ×1.5, tack ×0.8) |
| `closest_enemy_id`  | ID du plus proche ennemi                           | Logging                                              |

Code :
- `swarm/tactical.py::compute()`
- `swarm/roles.py::role_modifies_strategy(tactical=...)` (mode "defensive")

---

## 4. Filtrage selon `STORMWINGS_MODE`

| Source         | REGATE        | ESSAI                     |
|----------------|---------------|---------------------------|
| `P|U1Bn|...` (allié) | ✅ accepté | ✅ accepté                |
| `P|<autre>|...` (ennemi) | ✅ accepté | ❌ filtré (LoRa) |
| `W|...` (orga) | ✅ source principale | ❌ filtré (vent = `WIND_ESSAI_*`) |
| Émission `P|<moi>|...` | ✅ obligatoire 60 s | ✅ activée (test radio) |

Implémenté dans `comms/lora_iface.py::_handle_text`.

---

## 5. Vérification de conformité — convention de direction

Le PDF v5 §3.2 définit explicitement :

> **Convention direction : 0° = Nord · 90° = Est · 180° = Sud · 270° = Ouest**

Cette convention s'applique :
- au champ `dir` du message **WIND** (direction d'**OÙ vient le vent**)
- au champ `hdg` du message **POSITION** (cap magnétique du bateau)

### Mapping côté code StormWings

| Champ PDF v5      | Source MAVLink                          | Conversion                     | Convention en interne |
|-------------------|-----------------------------------------|--------------------------------|-----------------------|
| `dir` (W\|...)    | _N/A_ (reçu de l'orga)                  | `int(parts[1])`                | météo, "d'où il vient" |
| `spd_wind` (W\|...) | _N/A_                                 | `int(parts[2]) / 10.0` (m/s)   | toujours en m/s          |
| `lat` (P\|...)    | `GLOBAL_POSITION_INT.lat` (×1e7 deg)    | `lat * 1e5` (=÷100 si en ×1e7) | degrés décimaux           |
| `lon` (P\|...)    | `GLOBAL_POSITION_INT.lon` (×1e7 deg)    | idem                           | degrés décimaux           |
| `hdg` (P\|...)    | `GLOBAL_POSITION_INT.hdg` (centidegrés) | `hdg / 100.0`                  | cap magnétique 0-359°    |
| `spd_boat` (P\|...) | `hypot(VFR_HUD.vx, vy)` (cm/s) ou     | `m/s × 1.94384`                | nœuds décimaux           |
|                   | `VFR_HUD.groundspeed` (m/s)             |                                |                          |

### Cas-tests cardinaux

Les 11 tests `TestProtocol.test_wind_convention_*` et `test_position_*` valident bit-à-bit :

| Trame entrée                  | Interprétation              | Vérifié par |
|-------------------------------|-----------------------------|-------------|
| `W\|000\|50\|...`             | Vent du Nord, 5.0 m/s       | `test_wind_convention_north` |
| `W\|090\|50\|...`             | Vent de l'Est               | `test_wind_convention_east`  |
| `W\|180\|50\|...`             | Vent du Sud                 | `test_wind_convention_south` |
| `W\|270\|50\|...`             | Vent de l'Ouest             | `test_wind_convention_west`  |
| `W\|000\|00\|...`             | Capteur HORS LIGNE          | `test_wind_offline_marker`   |
| `W\|000\|01\|...`             | Vent Nord 0.1 m/s (PAS offline) | idem                  |
| `P\|U1B1\|...\|000\|10`       | Cap Nord, 1.0 kn            | `test_position_heading_convention_cardinal` |
| `P\|U1B1\|-4348256\|-649872\|...` | Hémisphère Sud, lon Ouest | `test_position_negative_coords` |
| MAVLink → P\|U1B1\|4348256\|... | Pipeline complet validé   | `test_full_chain_mavlink_to_lora` |

### Outil terrain `tools/lora_diag.py`

Pour vérifier la conformité **en live** sur le Pi le matin de la course :

```bash
# Décoder une trame reçue
python3 -m tools.lora_diag decode "W|245|63|1746787652"
# → [ORGA] WIND : 245° (WSW) souffle vers ENE, vitesse 6.3 m/s (12.2 kn)

# Encoder ce qu'on va broadcaster
python3 -m tools.lora_diag encode-pos --id U1B1 \
    --lat 43.48256 --lon 6.49872 --hdg 185 --spd-ms 1.6464
# → P|U1B1|4348256|649872|185|32

# Écouter le canal en live + valider chaque trame reçue
python3 -m tools.lora_diag listen --port /dev/ttyUSB0
# Affiche timestamp, validation, stats WIND/POS/erreurs toutes les 60s
```

L'outil flagge automatiquement :
- Champ direction hors `[0..359]`
- Padding zero incorrect (ex: `90` au lieu de `090`)
- ID bateau non officiel
- Timestamp invalide
- Cas spécial `P|<id>|0|0|0|0` (no-fix) toléré comme dans le PDF

### Note importante : sens du vent

Si la station orga annonce `W|245|63|...` :
- Direction **245°** = vent **vient du Sud-Ouest** (WSW)
- Donc le vent **souffle vers le Nord-Est** (ENE)
- Pour remonter au vent, le bateau doit caper vers le **245°** (vers le SW)
- Pour descendre vent arrière, il cap vers **65°** (NE)

Cette convention est **identique** à celle du cap navigation : un cap `185°` veut dire "le bateau pointe vers le Sud" (185° depuis le Nord en sens horaire). Donc pas de risque de confusion entre le `dir` du WIND et le `hdg` de la POSITION — ils utilisent la même rose des vents.

---

## 6. Vérification en course

### Au boot, vérifier dans les logs :

```
StormWings U1B1 — MODE=REGATE — COURSE 2 — démarrage
Réseau LoRa BATTLEBOATS : émission P|U1B1|… toutes les 60s (slot TDMA T+0s)
Réseau LoRa : écoute W|… (orga) + P|… (alliés UTT + 9 ennemis)
[LoRa] Connecté — node=U1B1
```

### Toutes les 60 s, vérifier la TX :

```
[LoRa-TX] POS #5 → P|U1B1|4348256|649872|185|32 (mode=REGATE, fix=GPS)
```

### Quand l'orga envoie un W|... :

```
[LoRa-RX] WIND #3 dir=245° spd=6.3m/s offline=False → relayé au wind_estimator (Pi/Cube)
[LORA-NET] WIND orga → nav stack : 245°/6.3m/s (ts=1746787652) — utilisé pour VMG et lay-lines
```

### Quand un autre bateau émet :

```
[LoRa-RX] POS U1B2 [ALLIÉ]  → 43.09650,5.95330 hdg=92 spd=2.1kn (rx#12)
[LoRa-RX] POS D2B1 [ENNEMI] → 43.09680,5.95350 hdg=185 spd=1.8kn (rx#13)
```

### Toutes les 5 s, snapshot tactique :

```
[TACT] team=1 ennemis=4 (proches=1, devant=2) closest=D2B1@28m cible_bloquée=False
```

### Logs CSV (colonnes ajoutées) :

```
… enemies_total, enemies_ahead, enemies_close, blocked_target,
   lora_tx_pos, lora_rx_pos, lora_rx_wind, …
```

### Si quelque chose cloche

| Symptôme dans les logs | Cause probable | Action |
|------------------------|----------------|--------|
| Aucun `[LoRa-TX] POS` après 60 s | LoRa non connecté ou TDMA pas démarré | `meshtastic --port /dev/ttyUSB0 --info` |
| `[LORA-NET] Broadcast P|… échoué` | Lien USB cassé ou ESP32 reset | Vérifier `dmesg`, débrancher/rebrancher |
| Aucun `WIND #...` reçu | Station orga muette ou hors portée | `WindEstimator` bascule sur fallback statique |
| `WIND offline=True` | Capteur orga en panne | Idem, fallback statique |
| Aucun `POS [ENNEMI]` | Hors portée ou en mode ESSAI | Vérifier `MODE` et `--listen` |

Pour tester la chaîne radio sans l'orga, lancer :

```bash
DRONE_ID=U1B1 STORMWINGS_MODE=ESSAI COURSE_NUMBER=2 python3 main.py
```

Puis vérifier que les `P|U1B1|...` apparaissent toutes les 60 s sur un terminal qui écoute :

```bash
meshtastic --port /dev/ttyUSB0 --listen
```
