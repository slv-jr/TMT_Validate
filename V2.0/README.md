# StormWings — Challenge SWARMz BattleBoats 2026

> Système de navigation autonome multi-drones voiliers pour le **Challenge SWARMz BattleBoats** organisé à Toulon les **9-10 mai 2026** par l'équipe UTT (Université de Technologie de Troyes).

**2 voiliers RC Joysway Focus V2** (~1 m, IDs `U1B1` Scout et `U1B2` Optimizer ; `U1B3` optionnel) en pilotage autonome coopératif. Le code supporte les **2 parcours officiels** retenus (1 banane et 2 côtier court) sélectionnables au lancement, et **2 modes opératoires** (ESSAI pour les tests, RÉGATE pour la course officielle). La station-sol diffuse le vent réel par LoRa toutes les 60 s. Les 2 drones se coordonnent en TDMA et s'échangent leurs positions GPS. Un opérateur reprend la main via le levier 3 positions de la J4C05 à tout moment.

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
│  STATION-SOL (orga)             │  À BORD DU DRONE              │
│  Station vent (Calypso)         │                               │
│  Nœud WIND (Meshtastic)         │  ESP32 LoRa V3 (Meshtastic)   │
│           │ W|dir|spd|ts        │  canal BATTLEBOATS            │
│           └──── LoRa 868 MHz ──►│  /dev/ttyUSB0                 │
│                (60 s)           │       ▼                       │
│  Télécommande J4C05             │  Raspberry Pi 5 (10 Hz)       │
│  Stick safran  (CH1 récept)     │       │ MAVLink /dev/serial0  │
│  Stick voile   (CH2 récept) PPM │       ▼                       │
│  Levier mode   (CH5 récept)────►│  Cube Orange+ ArduRover 4.6.3 │
│  → encodé Nano (chan4/5/6)      │  mode MANUAL permanent        │
│                                 │       │                       │
│                                 │  HERE4 RTK ──► Cube Orange+   │
│                                 │  (corrections RTCM3 internes) │
│                                 │       │ PWM                   │
│                                 │  Servo gouvernail  (OUT1)     │
│                                 │  Servo voile       (OUT2)     │
└─────────────────────────────────────────────────────────────────┘
```

| Composant   | Modèle                          | Précision                                   |
|-------------|---------------------------------|---------------------------------------------|
| Calculateur | Raspberry Pi 5                  | —                                           |
| Pilote auto | Cube Orange+ ArduRover 4.6.3    | —                                           |
| GNSS        | HERE4 RTK                       | ±2 cm (RTK fix) / ±3 m (GPS seul)           |
| LoRa        | ESP32 LoRa V3 + Meshtastic      | <500 m LOS                                  |
| Vent (orga) | Nœud WIND (Calypso côté orga)   | ±0.1 m/s, ±5° (cf. PDF protocole §3.2)      |

> **Pas de caméra** — choix délibéré. Détection adversaires par LoRa + analyse comportementale (`stall_detector.py`).

---

## Arborescence

```
V2.0/
├── main.py                       # boucle principale 10 Hz
├── config.py                     # ⚠️ DRONE_ID, MODE, COURSE_NUMBER pilotent tout
├── requirements.txt
├── README.md                     # ce fichier
│
├── navigation/                   # cerveau du voilier
│   ├── geo_utils.py              #   GPS (haversine, bearing, portes 12/34/AB)
│   ├── polar.py                  #   polaire — table par défaut + chargement table calibrée
│   ├── vmg.py                    #   VMG, vent apparent, cap optimal
│   ├── layline.py                #   détection layline + décision tack
│   ├── waypoints.py              #   CourseManager (2 parcours, stratégie côté vent
│   │                             #     parcours 1, rayon adaptatif RTK 4m / GPS 7m)
│   ├── potential_field.py        #   anti-collision Khatib
│   ├── heading_pid.py            #   PID de cap → angle de safran
│   └── state_machine.py          #   ATTENTE → EN_COURSE → STALL/PENALITE → FIN
│
├── safety/                       # sécurités & pénalité & blocage
│   ├── mode_switch.py            #   bascule MANUEL ↔ AUTO via levier (chan6)
│   ├── degraded_modes.py         #   GPS_LOST, RTK_DEGRADED, LORA_LOST,
│   │                             #     WIND_STALE, LOW_BATTERY, MAVLINK_LOST,
│   │                             #     STALL_DETECTED, ADVERSARY_SILENT
│   ├── stall_detector.py         #   détection blocage (2/3 conditions, palier)
│   ├── penalty_manager.py        #   séquence pénalité dynamique
│   │                             #     P1/P2 (parcours 1) ou Z1/Z2 (parcours 2)
│   └── logger.py                 #   logger CSV (~32 colonnes incl. mode + course_n)
│
├── comms/
│   ├── mavlink_iface.py          #   wrapper pymavlink (override + télémétrie)
│   ├── protocol.py               #   trames LoRa officielles (W|… et P|…)
│   └── lora_iface.py             #   wrapper Meshtastic + filtre ESSAI/RÉGATE
│
├── wind/
│   └── wind_estimator.py         #   ESSAI : vent fixe / RÉGATE : orga + fallback
│
├── swarm/
│   ├── roles.py                  #   Scout / Optimizer (régate à 2 drones)
│   └── tdma.py                   #   2 slots TDMA décalés de 30 s
│
├── calibration/                  # calibration sur l'eau (J-1)
│   └── polar_calibration.py      #   passes 30 s à différents angles
│
├── tests/
│   ├── test_logic_unit.py        #   tests unitaires (logique pure, 55 tests)
│   ├── test_connexion.py         #   sanity hardware (MAVLink + LoRa)
│   ├── test_servos.py            #   sweep gauche/centre/droite + voile
│   ├── test_bascule.py           #   vérification levier mode (chan6) manuel ↔ auto
│   └── test_rtk.py               #   attend RTK_FIXED avant validation départ
│
├── tools/
│   ├── buoy_entry.py             #   saisie interactive bouées (adaptée parcours)
│   ├── gps_buoy_survey.py        #   relevé RTK terrain (plan B)
│   ├── replay_log.py             #   visualisation post-course
│   └── simulator.py              #   simulateur 2D parcours
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

## Variables d'environnement principales

| Variable             | Valeurs           | Effet                                                            |
|----------------------|-------------------|------------------------------------------------------------------|
| `DRONE_ID`           | `U1B1` / `U1B2`   | Identité du drone (pilote profil & TDMA & rôle)                  |
| `STORMWINGS_MODE`    | `ESSAI` / `REGATE`| ESSAI = vent simulé, pas d'écoute orga ; RÉGATE = course officielle |
| `COURSE_NUMBER`      | `1`, `2`, `3`     | Sélection du parcours actif                                       |
| `WIND_DIR_DEG`       | 0-359             | (ESSAI) direction vent simulé d'OÙ il vient (météo)              |
| `WIND_SPEED_MS`      | float             | (ESSAI) vitesse vent simulé en m/s                                |
| `WIND_FALLBACK_DIR`  | 0-359             | (RÉGATE) direction fallback si orga muette > 30 s                |
| `WIND_FALLBACK_SPD`  | float             | (RÉGATE) vitesse fallback                                         |
| `BUOYS_OVERRIDE_PATH`| chemin            | Fichier JSON des bouées du jour                                   |

Lancement type :

```bash
# Course officielle parcours 2 sur U1B1
DRONE_ID=U1B1 STORMWINGS_MODE=REGATE COURSE_NUMBER=2 python3 main.py

# Test à sec parcours 1 avec vent simulé NW 5 m/s
DRONE_ID=U1B1 STORMWINGS_MODE=ESSAI COURSE_NUMBER=1 \
    WIND_DIR_DEG=315 WIND_SPEED_MS=5.0 python3 main.py

# Diagnostic config
DRONE_ID=U1B2 STORMWINGS_MODE=ESSAI COURSE_NUMBER=3 python3 config.py
```

---

## Configuration des 2 drones

| ID    | Rôle           | Slot TDMA | Stratégie           | Décalage départ | Comportement pré-départ          |
|-------|----------------|-----------|---------------------|------------------|----------------------------------|
| U1B1  | **Scout**      | t+0 ms    | scout_aggressive    | T+0 s            | safran centré + voile mi-position|
| U1B2  | **Optimizer**  | t+750 ms  | optimizer_smart     | T+30 s           | cercle 50 m derrière la porte    |

**Pourquoi cette stratégie ?**
- U1B1 (Scout) part en premier, cap direct vers la porte de départ. Il mesure le vent réel ressenti et broadcast sa position toutes les 60 s.
- U1B2 (Optimizer) attend 30 s en faisant un cercle 50 m en retrait. Pendant ces 30 s, il reçoit ≥ 1 broadcast du Scout, calibre sa polaire avec les data réelles, puis franchit la porte avec une stratégie VMG affinée.

Le code est **identique** sur les 2 drones. La variable `DRONE_ID` aiguille `config.py` qui charge le bon profil :

```bash
echo 'DRONE_ID=U1B1' | sudo tee /etc/stormwings/drone_id.env
```

---

## Les 2 parcours

### Parcours 1 — Banane (parcours construit)

```
Bouées  : 1, 2 (porte départ/arrivée), 3, 4 (porte au vent)
            P1, P2 (zone pénalité)

DÉPART   : Franchir porte 1-2
TOUR 1   : Porte 3-4 (enrouler la bouée côté vent)
            Porte 1-2 (enrouler la bouée côté vent)
TOUR 2   : Porte 3-4 (idem)
ARRIVÉE  : Franchir porte 1-2
Pénalité : Porte 2-P1, P2 bâbord, P1 bâbord
```

**Stratégie côté vent** : à chaque porte, le code détermine quelle bouée est la plus exposée à la brise (projection vectorielle vent · bouée) et l'enroule en passant le plus près possible.

### Parcours 2 — Côtier court

```
Bouées  : A, B (porte), C, D, E, Z1, Z2 (pénalité)

DÉPART   : Porte A-B
WP 1 (C) : Contourner C TRIBORD
WP 2 (D) : Contourner D BÂBORD
WP 3 (E) : Contourner E BÂBORD
WP 4 (C) : Contourner C TRIBORD
ARRIVÉE  : Porte A-B
Pénalité : A-Z1, Z2 bâbord, Z1 bâbord
```

> **Note** : le parcours N°3 (côtier long avec bouées F/G au large) a été
> retiré du programme. UTT court uniquement les parcours 1 ou 2 en 2026.

### Rayon de capture adaptatif (commun aux 2 parcours)

```python
# config.py
CAPTURE_RADIUS_RTK = 4.0   # m — fix_type ≥ 5 (RTK Float ou Fixed)
CAPTURE_RADIUS_GPS = 7.0   # m — fix_type 3-4 (GPS seul / DGPS)
```

`waypoints.py` choisit dynamiquement le rayon à chaque tick selon la qualité du fix retourné par `GPS_RAW_INT`. La validation vérifie aussi le **côté de passage** par produit vectoriel.

---

## Modes ESSAI vs RÉGATE

| Composant            | ESSAI                                | RÉGATE                                  |
|----------------------|--------------------------------------|-----------------------------------------|
| Vent                 | `WIND_DIR_DEG`/`WIND_SPEED_MS` env   | `W|...` orga reçu LoRa toutes les 60 s  |
| Bouées               | Coordonnées de test (overrides JSON) | Vraies coordonnées briefing (`tools/buoy_entry.py`) |
| LoRa filtres         | Alliés UTT seulement                 | Alliés + ennemis + WIND orga             |
| Heartbeat orga       | Non requis                           | Required pour valider la cohérence       |
| Logging              | tag `mode=ESSAI` dans CSV            | tag `mode=REGATE` dans CSV               |
| Fallback vent        | (sans objet)                         | `WIND_FALLBACK_DIR/SPD` après 30 s       |

Le mode ESSAI permet de valider toute la stratégie de nav (VMG, anti-collision, gestion des portes, contournements) sans dépendre du réseau orga, en échangeant uniquement entre les 2 drones UTT.

---

## Démarrage rapide

### Sur PC (vérification logique, sans matériel)

```bash
git clone <repo> stormwings && cd stormwings/V2.0
pip install -r requirements.txt

# Tests unitaires (55 tests, ~14ms)
python3 -m tests.test_logic_unit

# Diagnostic config
DRONE_ID=U1B1 STORMWINGS_MODE=ESSAI COURSE_NUMBER=1 python3 config.py

# Simulateur (parcours complet en ~7 minutes simulées)
python3 -m tools.simulator --wind-dir 90 --wind-speed 5
```

### Sur Raspberry Pi 5 (déploiement réel)

```bash
sudo bash scripts/install.sh && sudo reboot

echo 'DRONE_ID=U1B1' | sudo tee /etc/stormwings/drone_id.env

# Tests hardware (bateau hors de l'eau pour test_servos)
DRONE_ID=U1B1 python3 -m tests.test_connexion
DRONE_ID=U1B1 python3 -m tests.test_rtk            # attendre RTK FIXED
DRONE_ID=U1B1 python3 -m tests.test_servos
DRONE_ID=U1B1 python3 -m tests.test_bascule

# Démarrage course (parcours 2)
DRONE_ID=U1B1 STORMWINGS_MODE=REGATE COURSE_NUMBER=2 python3 main.py
# OU via systemd :
sudo systemctl start stormwings
sudo journalctl -u stormwings -f
```

---

## Pénalité — bascule manuel/auto

**Détection** : aucun signal externe. Le pilote constate visuellement que le drone dévie de la trajectoire correcte (n'enroule pas une bouée du bon côté, dérive franche, etc.) → il bascule le levier de mode en HAUT (MANUEL).

```
Pilote constate la dérive → levier HAUT (MANUEL)
        │
        ├─ Le drone passe instantanément en RC manuel
        ├─ Le pilote remet le drone sur la trajectoire
        ├─ Puis pilote le tour de pénalité à la main :
        │      Parcours 1 : porte 2-P1 → P2 bâbord → P1 bâbord
        │      Parcours 2 : porte A-Z1 → Z2 bâbord → Z1 bâbord
        ├─ Pilote rebascule le levier en BAS (AUTO)
        │
        └─ Le drone reprend le parcours à l'étape COURANTE
           (`race_started` reste True, donc pas de retour en pré-départ)
```

Limite réglementaire : **30 s max RC** par pénalité. Au-delà, l'auto reprend automatiquement.

Le déclenchement programmatique de la séquence Z1/Z2 ou P1/P2 reste possible via `app.request_penalty(reason=...)` (utile pour des tests).

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
| 8 - 15 s         | MEDIUM   | manœuvre de dégagement plus agressive|
| > 15 s           | HARD     | log warning + proposer reprise RC    |

`safety/stall_detector.py` expose le statut, `main.py` orchestre la réaction.

---

## Modes dégradés

| Mode               | Déclencheur                       | Réaction                          |
|--------------------|-----------------------------------|-----------------------------------|
| `GPS_LOST`         | pas de fix > 5 s                  | dead-reckoning (cap+vitesse)      |
| `RTK_DEGRADED`     | RTK perdu, GPS standard           | rayon capture → 7 m               |
| `LORA_LOST`        | pas de trame > 10 s               | vent fallback statique + solo     |
| `WIND_STALE`       | vent > 30 s                       | idem `LORA_LOST`                  |
| `LOW_BATTERY`      | < 11.1 V (< 20 %)                 | log warning + reprise RC suggérée |
| `MAVLINK_LOST`     | pas de heartbeat > 3 s            | pause + watchdog reset            |
| `STALL_DETECTED`   | blocage > 3 s                     | manœuvre dégagement auto          |
| `ADVERSARY_SILENT` | pas de P\|... > 10 s              | obstacle figé dernier relevé      |

---

## Communications inter-drones

Tous les messages passent par **LoRa 868 MHz via Meshtastic**, en broadcast sur le canal `BATTLEBOATS`.

**TDMA** pour étaler les broadcasts officiels (toutes les 60 s) et éviter les collisions d'air :

```
Cycle de 60 s (régate à 2 drones) :
  U1B1 émet à T+0s,  T+60s, T+120s, …
  U1B2 émet à T+30s, T+90s, T+150s, …
```

Trame de 1500 ms / 2 slots de 750 ms pour la couche TDMA fine interne.

| Sans communication | Avec communication |
|-------------------|-------------------|
| 2 polaires de vent identiques et figées | Polaire de l'Optimizer affinée par les observations du Scout (T+30s) |
| Virements décidés sur vent local | Virement anticipé si Scout a trouvé mieux |
| Risque collision UTT en approche bouée | Répulsion active, spacing maintenu |
| Si un drone tombe, l'autre ne sait pas | Détection silence > 10s → mode solo |

---

## Checklist J-2 → J0

**J-2 (7 mai)**
- [ ] Vérifier canal Meshtastic = `BATTLEBOATS` sur les 2 ESP32
- [ ] Simulateur sur les 2 parcours (1 banane, 2 côtier court) avec les 2 configs

**J-1 (8 mai — entraînement)**
- [ ] **Calibration polaire** sur l'eau : `python3 -m calibration.polar_calibration --wind-source lora`
- [ ] **Réglage PID gouvernail** sur l'eau
- [ ] Fix RTK vérifié sur les 2 drones (`tests/test_rtk.py`)
- [ ] Test essaim 2 drones simultanés (TDMA + LoRa)
- [ ] Test séquence pénalité autonome sur l'eau (P1/P2 et Z1/Z2)
- [ ] Test mode ESSAI avec coordonnées custom
- [ ] Charger `docs/ardupilot_params.txt` sur les 2 Cubes

**J0 matin (9 mai)**
- [ ] Briefing 9h30 → coordonnées officielles bouées + parcours du jour
- [ ] `COURSE_NUMBER=X python3 -m tools.buoy_entry` → écrit `/etc/stormwings/buoys_today.json`
- [ ] Fix RTK établi (`fix_type=6`) sur les 2 drones
- [ ] Régler `WIND_FALLBACK_DIR` / `WIND_FALLBACK_SPD` selon météo prévue
- [ ] Batteries de rechange chargées (3 courses requises)
- [ ] Attestation RC disponible

---

## Validation

- **Tests unitaires** sur la logique pure (84 tests : geo, polaire, VMG, layline, PID, anti-collision, protocole, stall, penalty, RTK radius, modes ESSAI/RÉGATE, parcours 1 et 2, portes 12/34/AB, spirale alternée parcours 1, conformité LoRa v5, indicateurs tactiques)
- **Simulateur 2D** : course finie en ~7 min 30 s sur 990 m avec vent E 5 m/s (parcours 2)
- **Tests hardware** dédiés : connexion, servos, bascule manuel/auto, RTK fix

---

## Crédits

Équipe UTT · ArduRover 4.6.3 · HERE4 RTK · Meshtastic · Joysway Focus V2 · Modifications matérielles déclarées à l'AO.

---

*StormWings v2.0 — Mai 2026 — README aligné sur Readme2.md (régate à 2 drones, 2 parcours, 2 modes).*
