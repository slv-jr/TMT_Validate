# StormWings — Challenge SWARMz BattleBoats 2026

> Système de navigation autonome multi-drones voiliers pour le **Challenge SWARMz BattleBoats** organisé à Toulon les **9-10 mai 2026** par l'équipe UTT (Université de Technologie de Troyes).

3 voiliers RC Joysway Focus V2 (~1 m, IDs `U1B1`, `U1B2`, `U1B3`) en pilotage autonome coopératif sur le **parcours côtier N°3** (bouée C visitée 3×), face aux équipes DaVinci, IPSA, ENSEEIHT. La station-sol diffuse le vent réel par LoRa. Les drones se coordonnent en TDMA. Un opérateur reprend la main via CH3 à tout moment.

---

## 📚 Documentation

| Document | Contenu | Quand le lire |
|----------|---------|---------------|
| [`docs/DEPLOYMENT.md`](./docs/DEPLOYMENT.md) | **Guide complet J-7 → J0**, ordre exact des opérations, commandes à copier-coller | À l'ouverture du kit, étape par étape |
| [`docs/CABLAGE.md`](./docs/CABLAGE.md) | Schéma détaillé de toutes les connexions (BEC, RC, TELEM2, servos, LoRa) | Pendant le câblage de chaque drone |
| [`docs/ARDUPILOT_PARAMS.md`](./docs/ARDUPILOT_PARAMS.md) | Explication ligne par ligne du fichier `ardupilot_params.txt` | Pendant la config Mission Planner |
| [`docs/ardupilot_params.txt`](./docs/ardupilot_params.txt) | Fichier de paramètres à charger dans Mission Planner | Cube Orange+ neuf à configurer |
| [`docs/CHECKLIST_J0.md`](./docs/CHECKLIST_J0.md) | **Checklist imprimable** course du 9 mai | À garder sur soi le jour J |

---

## Architecture matérielle

```
┌─────────────────────────────────────────────────────────────────┐
│  STATION-SOL                   │  À BORD DU DRONE              │
│  Calypso anémomètre            │                               │
│  ESP32 LoRa (Tx vent)          │  ESP32 LoRa V3 (Meshtastic)   │
│           │ W|dir|spd|ts        │  canal BATTLEBOATS            │
│           └──── LoRa 868 MHz ──►│  /dev/ttyUSB0                 │
│                                 │       ▼                       │
│  Télécommande RC                │  Raspberry Pi 4B (10 Hz)      │
│  CH1: gouvernail                │       │ MAVLink /dev/serial0  │
│  CH2: voile      PPM            │       ▼                       │
│  CH3: AUTO/MAN ────────────────►│  Cube Orange+ ArduRover 4.6.3 │
│  CH4: boost                     │  mode MANUAL permanent        │
│                                 │       │                       │
│                                 │  HERE4 RTK ──► Cube Orange+   │
│                                 │  (corrections RTCM3 internes) │
│                                 │       │ PWM                   │
│                                 │  Servo gouvernail  (OUT1)     │
│                                 │  Servo voile       (OUT2)     │
│                                 │  Boost moteur      (OUT4)*    │
│                                 │  * uniquement U1B1            │
└─────────────────────────────────────────────────────────────────┘
```

| Composant   | Modèle                          | Précision                                   |
|-------------|---------------------------------|---------------------------------------------|
| Calculateur | Raspberry Pi 4B                 | —                                           |
| Pilote auto | Cube Orange+ ArduRover 4.6.3    | —                                           |
| GNSS        | HERE4 RTK                       | ±2 cm (RTK fix) / ±3 m (GPS seul)           |
| LoRa        | ESP32 LoRa V3 + Meshtastic      | <500 m LOS                                  |
| Vent sol    | Calypso Ultrasonic              | ±0.1 m/s                                    |

> **Pas de caméra** — choix délibéré. Détection adversaires par LoRa + analyse comportementale (`stall_detector.py`).

---

## Arborescence

```
V2.0/
├── main.py                       # boucle principale 10 Hz
├── config.py                     # ⚠️ différencie les 3 drones via DRONE_ID
├── requirements.txt
├── README.md                     # ce fichier
│
├── navigation/                   # cerveau du voilier
│   ├── geo_utils.py              #   GPS (haversine, bearing, offset_meters)
│   ├── polar.py                  #   polaire — table par défaut + chargement table calibrée
│   ├── vmg.py                    #   VMG, vent apparent, cap optimal
│   ├── layline.py                #   détection layline + décision tack
│   ├── waypoints.py              #   CourseManager (parcours N°3, 9 étapes,
│   │                             #     rayon adaptatif RTK 4m / GPS 7m)
│   ├── potential_field.py        #   anti-collision Khatib
│   ├── heading_pid.py            #   PID de cap → angle de safran
│   └── state_machine.py          #   ATTENTE → EN_COURSE → STALL/PENALITE → FIN
│
├── safety/                       # sécurités & pénalité & blocage
│   ├── mode_switch.py            #   bascule MANUEL ↔ AUTO via CH3
│   ├── degraded_modes.py         #   GPS_LOST, RTK_DEGRADED, LORA_LOST,
│   │                             #     WIND_STALE, LOW_BATTERY, MAVLINK_LOST,
│   │                             #     STALL_DETECTED, ADVERSARY_SILENT
│   ├── stall_detector.py         #   détection blocage (2/3 conditions, palier)
│   ├── penalty_manager.py        #   séquence pénalité A→Z1→Z2 bâbord→Z1 bâbord
│   │                             #     dual-mode (5 s pour bascule manuelle)
│   └── logger.py                 #   logger CSV (~35 colonnes)
│
├── boost/
│   └── boost_controller.py       #   BUDGET TOTAL BOOST_MAX_S (pas 3×30s)
│
├── comms/
│   ├── mavlink_iface.py          #   wrapper pymavlink (override + télémétrie)
│   ├── protocol.py               #   trames LoRa officielles (W|… et P|…)
│   └── lora_iface.py             #   wrapper Meshtastic
│
├── wind/
│   └── wind_estimator.py         #   fusion Calypso + lissage + fallback
│
├── swarm/
│   ├── roles.py                  #   Scout / Optimizer / Safety dynamique
│   └── tdma.py                   #   slots TDMA décalés par drone
│
├── calibration/                  # calibration sur l'eau (J-1)
│   └── polar_calibration.py      #   passes 30 s à différents angles
│
├── tests/
│   ├── test_logic_unit.py        #   tests unitaires (logique pure)
│   ├── test_connexion.py         #   sanity hardware (MAVLink + LoRa)
│   ├── test_servos.py            #   sweep gauche/centre/droite + voile
│   ├── test_bascule.py           #   vérification CH3 manuel ↔ auto
│   └── test_rtk.py               #   attend RTK_FIXED avant validation départ
│
├── tools/
│   ├── buoy_entry.py             #   saisie interactive bouées matin J0
│   ├── gps_buoy_survey.py        #   relevé RTK terrain (plan B)
│   ├── replay_log.py             #   visualisation post-course
│   └── simulator.py              #   simulateur 2D parcours N°3
│
├── docs/
│   ├── DEPLOYMENT.md             #   guide complet J-7 → J0
│   ├── CABLAGE.md                #   schéma câblage détaillé
│   ├── ARDUPILOT_PARAMS.md       #   explications paramètres Cube
│   ├── ardupilot_params.txt      #   fichier params à charger
│   └── CHECKLIST_J0.md           #   checklist imprimable course
│
└── scripts/
    ├── install.sh                #   installation Pi (UART, deps, systemd)
    └── stormwings.service        #   service systemd
```

---

## Configuration des 3 drones

| ID    | Rôle           | Boost | Slot TDMA | Stratégie       | Décalage départ |
|-------|----------------|-------|-----------|-----------------|------------------|
| U1B1  | **Scout**      | ✅    | t+0 ms    | aggressive      | t+0 s            |
| U1B2  | **Optimizer**  | ❌    | t+500 ms  | vmg_optimal     | t+20 s           |
| U1B3  | **Safety**     | ❌    | t+1000 ms | conservative    | t+40 s           |

Le code est **identique** sur les 3 drones. Une variable d'environnement `DRONE_ID` aiguille `config.py` qui charge le bon profil. Sur Pi 4B :

```bash
echo 'DRONE_ID=U1B1' | sudo tee /etc/stormwings/drone_id.env
```

---

## Démarrage rapide

### Sur PC (vérification logique, sans matériel)

```bash
git clone <repo> stormwings && cd stormwings/V2.0
pip install -r requirements.txt

# Tests unitaires
python3 -m tests.test_logic_unit

# Simulateur (parcours complet en ~7 minutes simulées)
python3 -m tools.simulator --wind-dir 90 --wind-speed 5
```

### Sur Raspberry Pi 4B (déploiement réel)

```bash
sudo bash scripts/install.sh && sudo reboot

echo 'DRONE_ID=U1B1' | sudo tee /etc/stormwings/drone_id.env

# Tests hardware (bateau hors de l'eau pour test_servos)
DRONE_ID=U1B1 python3 -m tests.test_connexion
DRONE_ID=U1B1 python3 -m tests.test_rtk            # attendre RTK FIXED
DRONE_ID=U1B1 python3 -m tests.test_servos
DRONE_ID=U1B1 python3 -m tests.test_bascule

# Démarrage
DRONE_ID=U1B1 python3 main.py
# OU via systemd :
sudo systemctl start stormwings
sudo journalctl -u stormwings -f
```

---

## Parcours N°3

```
DÉPART    : Franchir porte A–B (cap Ouest)
WP 1 (C)  : Contourner C à TRIBORD  ← 1er passage
WP 2 (D)  : Contourner D à BÂBORD
WP 3 (E)  : Contourner E à BÂBORD
WP 4 (C)  : Contourner C à TRIBORD  ← 2ème passage
WP 5 (F)  : Contourner F à TRIBORD  ← bouée au large ~200 m
WP 6 (G)  : Contourner G à TRIBORD
WP 7 (C)  : Contourner C à TRIBORD  ← 3ème passage
ARRIVÉE   : Franchir porte A–B (cap Est)
Pénalité  : A→Z1, enrouler Z2 bâbord, enrouler Z1 bâbord
```

### Rayon de capture adaptatif

```python
# config.py
CAPTURE_RADIUS_RTK = 4.0   # m — fix_type ≥ 5 (RTK Float ou Fixed)
CAPTURE_RADIUS_GPS = 7.0   # m — fix_type 3-4 (GPS seul / DGPS)
```

`waypoints.py` choisit dynamiquement le rayon à chaque tick selon la qualité du fix retourné par `GPS_RAW_INT`. La validation vérifie aussi le **côté de passage** par produit vectoriel.

---

## Boost — règlement §2c

> La **somme totale** des activations = `BOOST_MAX_S` secondes par course (valeur annoncée la veille au briefing). Ce n'est **PAS** 3 × 30 s.

```python
# config.py
BOOST_MAX_S       = 30.0   # budget temps TOTAL par course
BOOST_ACTIONS_MAX = 3      # nombre max d'activations
```

Activation automatique (priorité décroissante) :
1. Vent < 1.5 m/s sur segment critique
2. Vitesse < 0.3 m/s pendant > 15 s (et budget restant)
3. Sortie de zone calme (entre F et G)

Le contrôleur coupe le boost dès que la vitesse remonte (économie du budget).

---

## Pénalité — bascule manuel/auto

```
Pénalité notifiée
        │
        ├─ CH3 basculé MANUEL dans les 5 s ?
        │      OUI → opérateur pilote (max 30 s RC, puis auto reprend)
        │      NON → séquence autonome :
        │              1. Cap vers Z1
        │              2. Passage A→Z1
        │              3. Enrouler Z2 bâbord
        │              4. Enrouler Z1 bâbord
        │              5. Retour waypoint courant
```

Toute la logique est dans `safety/penalty_manager.py`. Le déclenchement programmatique se fait via `app.request_penalty(reason=...)`.

---

## Détection blocage (sans caméra)

**Drone bloqué** = 2 conditions sur 3 vraies pendant > 3 s :

| Indicateur                | Seuil                          |
|---------------------------|--------------------------------|
| Vitesse sol quasi-nulle   | `speed < 0.15 m/s`             |
| Commandes safran actives  | `\|rudder\| > 15°`             |
| Distance waypoint figée   | `Δd < 0.5 m sur 3 s`           |

**Réaction par paliers** :

| Durée du blocage | Niveau   | Action                               |
|------------------|----------|--------------------------------------|
| 0 - 8 s          | LIGHT    | choquer voile + virer de bord        |
| 8 - 15 s         | MEDIUM   | boost (si budget) + manœuvre         |
| > 15 s           | HARD     | log warning + proposer reprise RC    |

`safety/stall_detector.py` expose le statut, `main.py` orchestre la réaction.

---

## Modes dégradés

| Mode               | Déclencheur                       | Réaction                          |
|--------------------|-----------------------------------|-----------------------------------|
| `GPS_LOST`         | pas de fix > 5 s                  | dead-reckoning (cap+vitesse)      |
| `RTK_DEGRADED`     | RTK perdu, GPS standard           | rayon capture → 7 m               |
| `LORA_LOST`        | pas de trame > 10 s               | vent défaut (E 4.5 m/s) + solo    |
| `WIND_STALE`       | vent > 30 s                       | idem `LORA_LOST`                  |
| `LOW_BATTERY`      | < 11.1 V (< 20 %)                 | désactive boost                   |
| `MAVLINK_LOST`     | pas de heartbeat > 3 s            | pause + watchdog reset            |
| `STALL_DETECTED`   | blocage > 3 s                     | manœuvre dégagement auto          |
| `ADVERSARY_SILENT` | pas de P\|... > 10 s              | obstacle figé dernier relevé      |

---

## Communications inter-drones

Tous les messages passent par **LoRa 868 MHz via Meshtastic**, en broadcast sur le canal `BATTLEBOATS`.

**TDMA** pour étaler les broadcasts officiels (toutes les 60 s) et éviter les collisions d'air :

```
Cycle de 60 s :
  U1B1 émet à T+0s, T+60s, T+120s, …
  U1B2 émet à T+20s, T+80s, …
  U1B3 émet à T+40s, T+100s, …
```

Trame de 1500 ms / 3 slots de 500 ms pour la couche TDMA fine interne (extension future).

| Sans communication | Avec communication |
|-------------------|-------------------|
| 3 polaires de vent identiques et figées | Polaire affinée par les observations du Scout |
| Virements décidés sur vent local | Virement anticipé si Scout a trouvé mieux |
| Risque collision inter-UTT en approche bouée | Répulsion active, spacing maintenu |
| Si un drone tombe, les 2 autres ne savent pas | Redistribution automatique des rôles |

---

## Checklist J-2 → J0

**J-2 (7 mai)**
- [ ] Mettre à jour `BOOST_MAX_S` après annonce organisateurs
- [ ] Vérifier canal Meshtastic = `BATTLEBOATS` sur les 3 ESP32
- [ ] Simulateur parcours N°3 sur les 3 configs

**J-1 (8 mai — entraînement)**
- [ ] **Calibration polaire** sur l'eau : `python3 -m calibration.polar_calibration --wind-source lora`
- [ ] **Réglage PID gouvernail** sur l'eau
- [ ] Fix RTK vérifié sur les 3 drones (`tests/test_rtk.py`)
- [ ] Test essaim 3 drones simultanés (TDMA + LoRa)
- [ ] Test séquence pénalité autonome sur l'eau
- [ ] Charger `docs/ardupilot_params.txt` sur les 3 Cubes

**J0 matin (9 mai)**
- [ ] Briefing 9h30 → coordonnées bouées A B C D E F G Z1 Z2
- [ ] `python3 -m tools.buoy_entry` → écrit `/etc/stormwings/buoys_today.json`
- [ ] Fix RTK établi (`fix_type=6`) sur les 3 drones
- [ ] Confirmer `BOOST_MAX_S` si valeur modifiée au briefing
- [ ] Batteries de rechange chargées (3 courses requises)
- [ ] Attestation RC disponible

---

## Validation

- **Tests unitaires** sur la logique pure (geo, polaire, VMG, layline, PID, anti-collision, protocole, boost, stall, penalty, RTK radius)
- **Simulateur 2D** : course finie en ~7 min 30 s sur 990 m avec vent E 5 m/s
- **Tests hardware** dédiés : connexion, servos, bascule manuel/auto, RTK fix

---

## Crédits

Équipe UTT · ArduRover 4.6.3 · HERE4 RTK · Meshtastic · Joysway Focus V2 · Modifications matérielles déclarées à l'AO.

---

*StormWings v2.0 — Mai 2026 — README aligné sur Readme2.md (architecture révisée).*
