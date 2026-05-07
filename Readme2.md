le fichier "SwarmZ_fichier_Orga" est la stack + protocol + shape des organisateur. on moule tout avec cette stack.

---

# StormWings — Challenge SWARMz BattleBoats 2026
### README v2 — Architecture révisée & optimisée · Mai 2026

> Système de navigation autonome multi-drones voiliers pour le Challenge SWARMz BattleBoats organisé à Toulon les 9-10 mai 2026 par l'**équipe UTT (Université de Technologie de Troyes)**.

---

## 🎯 Objectif

Faire courir 3 voiliers RC Joysway Focus V2 (~1 m, ID : `U1B1`, `U1B2`, `U1B3`) en **pilotage autonome coopératif** sur le parcours côtier N°3 (bouée C visitée 3×), face aux équipes DaVinci, IPSA, ENSEEIHT. La station-sol diffuse le vent réel par LoRa. Les drones se coordonnent en TDMA. Un opérateur reprend la main via CH3 à tout moment.

---

## 🏗️ Architecture matérielle

```
┌─────────────────────────────────────────────────────────────────┐
│  STATION-SOL                   │  À BORD DU DRONE              │
│  Calypso anémomètre            │                               │
│  ESP32 LoRa (Tx vent)          │  ESP32 LoRa V3 (Meshtastic)  │
│           │ W|dir|spd|ts        │  canal BATTLEBOATS            │
│           └──── LoRa 868 MHz ──►│  /dev/ttyUSB0                │
│                                 │       ▼                       │
│  Télécommande RC                │  Raspberry Pi 5 (10 Hz)      │
│  CH1: gouvernail                │       │ MAVLink /dev/serial0  │
│  CH2: voile      PPM            │       ▼                       │
│  CH5: AUTO/MAN ────────────────►│  Cube Orange+ ArduRover 4.6.3│
│                                 │  mode MANUAL permanent        │
│                                 │       │                       │
│                                 │  HERE4 RTK ──► Cube Orange+  │
│                                 │  (corrections RTCM3 internes) │
│                                 │       │ PWM                   │
│                                 │  Servo gouvernail  (OUT1)     │
│                                 │  Servo voile       (OUT2)     │
│                                 │  Boost moteur      (OUT4)*    │
│                                 │  * uniquement U1B1            │
└─────────────────────────────────────────────────────────────────┘
```

| Composant | Modèle | Précision |
|-----------|--------|-----------|
| Calculateur | Raspberry Pi 5 | — |
| Pilote auto | Cube Orange+ ArduRover 4.6.3 | — |
| GNSS | HERE4 RTK | ±2 cm (RTK fix) / ±3 m (GPS seul) |
| LoRa | ESP32 LoRa V3 + Meshtastic | <500 m LOS |
| Vent sol | Calypso Ultrasonic | ±0.1 m/s |

> **Pas de caméra** — choix délibéré. Détection adversaires par LoRa + analyse comportementale (cf. §Détection blocage).

---

## 🗺️ Parcours N°3 — Séquence exacte

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

**Rayon de capture adaptatif** (dans `config.py`) :
```python
CAPTURE_RADIUS_RTK = 4.0   # m — si RTK_FIXED confirmé
CAPTURE_RADIUS_GPS = 7.0   # m — fallback GPS standard
```
La validation de contournement vérifie aussi le **côté de passage** (vecteur de déplacement vs position bouée).

---

## ⚙️ Configuration des 3 drones

| ID | Rôle | Boost | Slot TDMA | Stratégie |
|----|------|-------|-----------|-----------|
| U1B1 | **Scout** | ✅ | t+0 s | Vitesse brute, confirme bouées en premier |
| U1B2 | **Optimizer** | ❌ | t+20 s | VMG optimal, exploite données Scout |
| U1B3 | **Safety** | ❌ | t+40 s | Conservateur, peut passer Optimizer si U1B1/U1B2 hors course |

---

## 🛰️ Exploitation HERE4 RTK + Cube Orange+

Le HERE4 délivre des corrections RTCM3 directement au Cube Orange+. Utiliser la **position EKF ArduPilot** (fusion GPS+IMU) plutôt que le GPS brut :

```python
# mavlink_iface.py — lecture position EKF
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.1)
lat = msg.lat / 1e7
lon = msg.lon / 1e7
hdg = msg.hdg / 100    # degrés
spd = msg.vx / 100     # m/s sol

# Vérifier le fix RTK
gps = master.recv_match(type='GPS_RAW_INT', blocking=False)
rtk_fixed = (gps and gps.fix_type >= 5)
# fix_type: 0=No GPS, 1=No fix, 2=2D, 3=3D, 4=DGPS, 5=RTK Float, 6=RTK Fixed
```

**Rayon adaptatif selon fix** :
```python
# waypoints.py
radius = config.CAPTURE_RADIUS_RTK if rtk_fixed else config.CAPTURE_RADIUS_GPS
```

**Test obligatoire avant course** :
```bash
DRONE_ID=U1B1 python3 -m tests.test_rtk
# → Attendre RTK_FIXED (fix_type=6) avant de valider le départ
```

---

## 📐 Polaire de vitesse — Calibration terrain (J-1)

La polaire est indispensable pour le VMG et le timing des virements. Une polaire non calibrée = virements mal timés = perte estimée 20-30% sur la course.

**Procédure J-1 (8 mai, eau calme)** :
```bash
DRONE_ID=U1B1 python3 -m calibration.polar_calibration --wind-source lora
# Passes de 30 s aux angles : 40°, 50°, 60°, 70°, 90°, 120°, 150°, 180°
# → Sauvegarde dans config.POLAR_TABLE
```

**Valeur cible** : angle de remontée optimal Joysway Focus V2 ≈ **42°–48°** selon force du vent (à confirmer sur l'eau — c'est ce qui pilote tous les tacks).

```python
# vmg.py
def optimal_upwind_angle(wind_speed_ms: float) -> float:
    return config.POLAR_TABLE.get_upwind_angle(wind_speed_ms)
    # Fallback si non calibrée : 45°
```

---

## ⚖️ Pénalité — Mode AUTO et MANUEL

```
Décision pénalité détectée
        │
        ├─ CH3 basculé MANUEL par opérateur dans les 5 s ?
        │      OUI → opérateur pilote (max 30 s RC, puis auto reprend)
        │      NON → séquence autonome :
        │              1. Cap vers Z1
        │              2. Passage A–Z1
        │              3. Enrouler Z2 bâbord
        │              4. Enrouler Z1 bâbord
        │              5. Retour waypoint courant
```

```python
# state_machine.py — état PENALITE
class PenaltyState:
    DECISION_TIMEOUT_S = 5.0

    def on_enter(self):
        self.t0 = time.time()
        self.mode = 'WAIT'

    def update(self):
        if self.mode == 'WAIT':
            if mode_switch.is_manual():
                self.mode = 'MANUAL'
            elif time.time() - self.t0 > self.DECISION_TIMEOUT_S:
                self.mode = 'AUTO'
                penalty_manager.start_auto_sequence()
        elif self.mode == 'AUTO':
            if penalty_manager.update():
                return State.EN_COURSE
        elif self.mode == 'MANUAL':
            if mode_switch.is_auto():
                return State.EN_COURSE
```

---

## 🚦 Détection blocage / collision (sans caméra)

**Drone bloqué** = 2 conditions sur 3 vraies pendant >3 s :

| Indicateur | Seuil | Source |
|-----------|-------|--------|
| Vitesse sol quasi-nulle | `speed < 0.15 m/s` | HERE4 RTK |
| Commandes actives | `\|rudder\| > 15°` | heading_pid |
| Distance waypoint figée | pas d'évolution | geo_utils |

**Adversaire silencieux** = dernier relevé LoRa >10 s → traité comme obstacle fixe.

**Réaction automatique** :
```
Blocage < 8 s   → choquer voile + virer de bord
Blocage 8-15 s  → manœuvre de dégagement plus agressive
Blocage > 15 s  → alerte buzzer + proposer reprise RC
```

---

## 📊 Modes dégradés

| Mode | Déclencheur | Réaction |
|------|-------------|----------|
| `GPS_LOST` | Pas de fix >5 s | Dead-reckoning (cap+vitesse estimée) |
| `RTK_DEGRADED` | RTK perdu, GPS seul | Rayon capture → 7 m |
| `LORA_LOST` | Pas de trame >10 s | Vent défaut E 4.5 m/s, solo |
| `WIND_STALE` | Vent >30 s | Idem LORA_LOST |
| `LOW_BATTERY` | <11.1 V | Log warning + reprise RC suggérée |
| `MAVLINK_LOST` | Pas heartbeat >3 s | Pause + watchdog reset |
| `STALL_DETECTED` | Blocage >3 s | Manœuvre dégagement auto |
| `ADVERSARY_SILENT` | Pas de P\| >10 s | Obstacle fixe dernier relevé |

---

## 📝 Checklist J-2 → J0

**J-2 (7 mai)**
- [ ] Vérifier canal Meshtastic = `BATTLEBOATS` sur les 3 ESP32
- [ ] Simulateur parcours N°3 sur les 3 configs

**J-1 (8 mai — entraînement)**
- [ ] **Calibration polaire** sur l'eau (`calibration/polar_calibration.py`)
- [ ] **Réglage PID gouvernail** sur l'eau
- [ ] Fix RTK vérifié sur les 3 drones
- [ ] Test essaim 3 drones simultanés (TDMA + LoRa)
- [ ] Test séquence pénalité autonome sur l'eau
- [ ] Charger `ardupilot_params.txt` sur les 3 Cubes

**J0 matin (9 mai)**
- [ ] Briefing 9h30 → coordonnées bouées A B C D E F G Z1 Z2
- [ ] `tools/buoy_entry.py` → validation + distances → `config.BUOYS_GPS`
- [ ] Fix RTK établi (fix_type=6) sur les 3 drones
- [ ] Batteries rechange chargées (3 courses requises)
- [ ] Attestation RC disponible

---

## 📜 Crédits
Équipe UTT · ArduRover 4.6.3 · HERE4 RTK · Meshtastic · Joysway Focus V2 · Modifications matérielles déclarées à l'AO.

---

Le README2 est complet. Les points clés intégrés par rapport à l'ancien :

**Nouveau** : exploitation RTK adaptatif (rayon 4 m vs 7 m selon fix), calibration polaire structurée, pénalité double-mode avec timeout 5 s, détection blocage comportementale, mode `ADVERSARY_SILENT`, rayon de capture adaptatif.

Voulez-vous qu'on attaque maintenant un des modules de code concret — `stall_detector.py` ou `penalty_manager.py` ?

Oui, les 3 voiliers communiquent entre eux. Voilà comment et pourquoi c'est crucial.

## Comment ils communiquent

Tous les messages passent par **LoRa 868 MHz via Meshtastic**, en broadcast sur le canal `BATTLEBOATS`. Il n'y a pas de liaison directe drone-à-drone — tout le monde émet, tout le monde écoute.

Pour éviter que les 3 drones émettent en même temps et se brouillent mutuellement, ils utilisent le **TDMA** (Time Division Multiple Access) : la seconde est découpée en 3 fenêtres, chaque drone a la sienne.

```
Trame de 1500 ms répétée en boucle :
┌──────────────┬──────────────┬──────────────┐
│  U1B1 émet   │  U1B2 émet   │  U1B3 émet   │
│   t = 0 ms   │  t = 500 ms  │  t = 1000 ms │
│   (Scout)    │  (Optimizer) │   (Safety)   │
└──────────────┴──────────────┴──────────────┘
```

Chaque drone émet un paquet `P|id|lat|lon|hdg|spd` toutes les 1.5 s. Tous les 3 reçoivent aussi les trames vent `W|dir|spd|ts` de la station sol.

---

## Pourquoi c'est important — ce que ça change concrètement

Sans communication, les 3 drones sont 3 individus aveugles qui se gênent. Avec la communication, ils forment un vrai essaim. Voilà ce que chaque drone fait des infos des autres :

**U1B1 Scout** part en tête. Il envoie en temps réel sa position et son cap. U1B2 et U1B3 savent donc exactement où il est, à quelle vitesse il avance, et sur quel bord il navigue. Si le Scout ralentit anormalement (bloqué, vent mort), les deux autres le voient avant même d'arriver dans la même zone et peuvent anticiper.

**U1B2 Optimizer** exploite les données du Scout pour choisir son bord. Si le Scout a viré de bord tribord → bâbord et a gagné de la vitesse, l'Optimizer sait que le bord tribord était meilleur dans cette zone et reste dessus plus longtemps. C'est du partage de données de vent en temps réel — chaque drone devient un capteur de vent pour les deux autres.

**U1B3 Safety** surveille les positions de tout le monde. Il sait si U1B1 et U1B2 ont bien contourné chaque bouée (redondance de validation). Si U1B2 disparaît du LoRa, U1B3 prend son rôle d'Optimizer automatiquement.

**L'anti-collision inter-drones** ne fonctionne que grâce à ça. Le `potential_field.py` utilise les positions LoRa des deux autres voiliers UTT comme obstacles répulsifs — sans ça, ils pourraient se rentrer dedans en approchant de la même bouée.

---

## Ce que ça apporte en chiffres

| Sans communication | Avec communication |
|-------------------|-------------------|
| 3 polaires de vent identiques et figées | Polaire affinée par les observations du Scout |
| Virements décidés sur vent local seulement | Virement anticipé si le Scout a trouvé mieux |
| Risque collision inter-UTT en approche bouée | Répulsion active, spacing maintenu |
| Si un drone tombe, les 2 autres ne savent pas | Redistribution automatique des rôles |

L'ancien dossier estimait un gain de **+8 à 12% de vitesse effective** grâce au partage des données de vent. C'est crédible : un virement raté ou trop tardif coûte 3-5 secondes à chaque fois, et en 30 min de course avec 10-12 virements, ça s'accumule vite.

---

## La limite à connaître

Si un **adversaire n'émet pas** sur le canal LoRa (ou utilise un protocole différent), vos drones sont aveugles à sa position. C'est pour ça que le mode `ADVERSARY_SILENT` dans le README2 traite les adversaires silencieux comme des obstacles fixes à leur dernière position connue — c'est la meilleure approximation possible sans caméra.


Oui et non — c'est le **même dépôt de code**, mais il se configure différemment selon le drone via une seule variable d'environnement : `DRONE_ID`.

## Comment ça marche concrètement

```bash
# Sur U1B1
echo 'DRONE_ID=U1B1' | sudo tee /etc/stormwings/drone_id.env

# Sur U1B2
echo 'DRONE_ID=U1B2' | sudo tee /etc/stormwings/drone_id.env

# Sur U1B3
echo 'DRONE_ID=U1B3' | sudo tee /etc/stormwings/drone_id.env
```

Même `git clone`, même `pip install`, même `main.py` — seule cette ligne change. Au démarrage, `config.py` lit `DRONE_ID` et branche automatiquement tous les paramètres différents.

---

## Ce que `config.py` différencie selon le DRONE_ID

```python
import os

DRONE_ID = os.environ.get('DRONE_ID', 'U1B1')

_profiles = {
    'U1B1': {
        'role':          'SCOUT',
        'tdma_slot_ms':  0,        # émet en premier
        'strategy':      'aggressive',   # prend des risques, vitesse brute
    },
    'U1B2': {
        'role':          'OPTIMIZER',
        'tdma_slot_ms':  500,      # émet en deuxième
        'strategy':      'vmg_optimal',  # meilleur VMG, suit les données Scout
    },
    'U1B3': {
        'role':          'SAFETY',
        'tdma_slot_ms':  1000,     # émet en troisième
        'strategy':      'conservative', # minimise les risques
    },
}

# Chargement du profil actif
_p = _profiles[DRONE_ID]
ROLE          = _p['role']
TDMA_SLOT_MS  = _p['tdma_slot_ms']
STRATEGY      = _p['strategy']
```

Tout le reste du code lit `config.ROLE`, `config.STRATEGY`, etc. — il n'y a jamais de `if DRONE_ID == 'U1B1'` éparpillé partout dans le code.

---

## Ce qui est strictement identique sur les 3

- La boucle principale `main.py`
- Tous les algorithmes de navigation (VMG, layline, PID, anti-collision)
- La gestion des modes dégradés
- Le protocole LoRa
- La state machine
- Les tests

## Ce qui est différent selon le drone

| Paramètre | U1B1 | U1B2 | U1B3 |
|-----------|------|------|------|
| Rôle initial | Scout | Optimizer | Safety |
| Slot TDMA | 0 ms | 500 ms | 1000 ms |
| Stratégie tack | Agressive | VMG pur | Conservative |

---

## Avantage de cette approche

Un seul bug à corriger, une seule mise à jour à déployer sur les 3 cartes. La veille de la course si vous corrigez quelque chose, vous faites `git pull` sur les 3 Raspberry Pi et c'est bon — pas de risque d'oublier de synchroniser une version différente entre les drones.