"""
StormWings — configuration centrale.

Pour différencier les 3 drones de l'équipe UTT, un seul paramètre change :
la variable d'environnement DRONE_ID qu'on règle au lancement du service systemd
(ou en CLI : `DRONE_ID=U1B2 python3 main.py`).

ID officiels (cf. BATTLEBOATS_LORA_PROTOCOL_v5.pdf, équipe UTT) :
    U1B1 → drone 1 (Scout)
    U1B2 → drone 2 (Optimizer)
    U1B3 → drone 3 (Safety)

IMPORTANT — paramètres ArduPilot validés (setup NouvelEncodeur officiel) :
    Encodeur PPM = Arduino Nano flashé ArduPPM v2.3.16. Mapping côté Cube :
        chan4_raw  ← CH1 récepteur Joysway J5C01R  → safran (RCMAP_ROLL=4)
        chan5_raw  ← CH2 récepteur                  → voile  (RCMAP_THROTTLE=5)
        chan6_raw  ← CH5 récepteur (levier mode)    → MODE_CH=6
    Voir docs/ARDUPILOT_PARAMS.md et SwarmZ_fichier_Orga/NouvelEncodeur/.

Stratégie générale (cf. README2) :
    - Mode MANUAL permanent côté Cube ; le Pi écrit RC_CHANNELS_OVERRIDE
      sur chan4/chan5 (safran/voile) à 10 Hz.
    - Bascule MANUEL/AUTO via le levier de mode lu sur chan6_raw côté Pi.
    - Position EKF du Cube + corrections RTCM3 du HERE4 → fix RTK.
    - Rayon de capture des bouées ADAPTATIF selon le fix : 4 m en RTK,
      7 m en fallback GPS standard.
"""

import json
import logging
import os
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

log = logging.getLogger(__name__)


# ════════════════════════════════════════════════════════════════════════
# IDENTITÉ DU DRONE (différencie les 3 instances)
# ════════════════════════════════════════════════════════════════════════
DRONE_ID: str = os.environ.get("DRONE_ID", "U1B1").upper()
TEAM_ID: str = "UTT"

# Mapping ID → numéro logique dans l'essaim (1 = Scout par défaut)
_DRONE_NUM_MAP = {"U1B1": 1, "U1B2": 2, "U1B3": 3}
DRONE_NUM: int = _DRONE_NUM_MAP.get(DRONE_ID, 1)

# Liste des coéquipiers (broadcast → utile pour filtrer les voisins)
TEAM_BOATS = ("U1B1", "U1B2", "U1B3")
ENEMY_BOATS = (
    "D2B1", "D2B2", "D2B3",   # DaVinci Hive
    "I3B1", "I3B2", "I3B3",   # IPSA
    "E4B1", "E4B2", "E4B3",   # ENSEEIHT
)


# ════════════════════════════════════════════════════════════════════════
# RÔLES
# ════════════════════════════════════════════════════════════════════════
ROLE_SCOUT: str = "SCOUT"
ROLE_OPTIMIZER: str = "OPTIMIZER"
ROLE_SAFETY: str = "SAFETY"

# Profil par drone (cf. README2)
_DRONE_PROFILES: Dict[str, Dict] = {
    "U1B1": {
        "role": ROLE_SCOUT,
        "tdma_slot_ms": 0,
        "race_start_offset_s": 0.0,    # part en premier
        "strategy": "aggressive",
    },
    "U1B2": {
        "role": ROLE_OPTIMIZER,
        "tdma_slot_ms": 500,
        "race_start_offset_s": 20.0,   # 20 s après le Scout
        "strategy": "vmg_optimal",
    },
    "U1B3": {
        "role": ROLE_SAFETY,
        "tdma_slot_ms": 1000,
        "race_start_offset_s": 40.0,
        "strategy": "conservative",
    },
}
_PROFILE = _DRONE_PROFILES.get(DRONE_ID, _DRONE_PROFILES["U1B2"])

DEFAULT_ROLE: str = _PROFILE["role"]
STRATEGY: str = _PROFILE["strategy"]
RACE_START_OFFSET_S: float = _PROFILE["race_start_offset_s"]

ROLE_REEVAL_PERIOD_S: float = 5.0


# ════════════════════════════════════════════════════════════════════════
# MATÉRIEL EMBARQUÉ
# ════════════════════════════════════════════════════════════════════════
# UART vers Cube Orange+ (port TELEM2 du Cube → /dev/serial0 du Pi)
# Sur Pi 5 (et Pi 4B) avec dtoverlay=disable-bt, /dev/serial0 → /dev/ttyAMA0
MAVLINK_PORT: str = "/dev/serial0"
MAVLINK_BAUD: int = 57600

# UART vers ESP32 LoRa V3 (Meshtastic) — branché en USB
LORA_PORT: str = "/dev/ttyUSB0"
LORA_BAUD: int = 115200      # auto-géré par Meshtastic, info seulement


# ════════════════════════════════════════════════════════════════════════
# FRÉQUENCES DES BOUCLES
# ════════════════════════════════════════════════════════════════════════
NAV_LOOP_HZ: float = 10.0          # boucle navigation (capteurs + décisions)
COMM_LOOP_HZ: float = 2.0          # boucle LoRa
NAV_DT: float = 1.0 / NAV_LOOP_HZ
COMM_DT: float = 1.0 / COMM_LOOP_HZ

# Période d'envoi continu RC_CHANNELS_OVERRIDE (Cube timeout ~0.5s)
OVERRIDE_REFRESH_HZ: float = 10.0

# Le protocole BattleBoats officiel impose 60 s entre 2 broadcasts P|...
LORA_BROADCAST_PERIOD_S: float = 60.0


# ════════════════════════════════════════════════════════════════════════
# PROTOCOLE LORA TDMA (interne UTT, pour anti-collision radio)
# ════════════════════════════════════════════════════════════════════════
# Trame de 1500 ms divisée en 3 slots de 500 ms (cf. README2)
TDMA_FRAME_MS: int = 1500
TDMA_SLOT_MS: int = 500
TDMA_SLOTS = {"U1B1": 0, "U1B2": 500, "U1B3": 1000}
MY_TDMA_OFFSET_MS: int = TDMA_SLOTS.get(DRONE_ID, 0)


# ════════════════════════════════════════════════════════════════════════
# PARCOURS N°3 — POSITIONS DES BOUÉES (coordonnées GPS lat/lon)
# ════════════════════════════════════════════════════════════════════════
# Les coordonnées officielles sont fournies par les organisateurs le matin
# de la course. À J0 : `python3 -m tools.buoy_entry` permet de les saisir.
# Les valeurs ci-dessous sont des PLACEHOLDERS représentant la géométrie
# estimée du parcours N°3 sur la baie du Mourillon, Toulon.
@dataclass(frozen=True)
class BuoyPos:
    """Position locale (east, north) en mètres — pour outils de visualisation."""
    east: float
    north: float


# Origine GPS du repère local — Plage du Mourillon, Toulon
# (utilisée UNIQUEMENT par les outils de visualisation — pas par le runtime)
ORIGIN_LAT: float = 43.0967     # latitude décimale °
ORIGIN_LON: float = 5.9533      # longitude décimale °


# ⚠️ ÉDITER LE MATIN DE LA COURSE avec les coordonnées GPS officielles ⚠️
# Format : "NOM_BOUÉE": (latitude_décimale, longitude_décimale)
BUOYS_GPS: Dict[str, Tuple[float, float]] = {
    # Porte départ/arrivée (~30 m d'écartement)
    "A":  (43.0967000, 5.9542853),   # tribord porte
    "B":  (43.0964302, 5.9542853),   # bâbord porte
    # Bouée centrale (visitée 3 fois)
    "C":  (43.0965201, 5.9533000),
    # Triangle nord-ouest
    "D":  (43.0968799, 5.9529305),
    "E":  (43.0967450, 5.9523147),
    # Boucle sud-ouest (F au large, ~200 m)
    "F":  (43.0961604, 5.9508368),
    "G":  (43.0963403, 5.9514526),
    # Bouées zone pénalité (au nord-est du parcours)
    "Z1": (43.0969698, 5.9545316),
    "Z2": (43.0971497, 5.9545316),
}


def load_buoys_from_file(path: str) -> bool:
    """Charge un override JSON des coordonnées (généré par tools/buoy_entry.py).

    Le fichier doit avoir la forme {"A": [lat, lon], ...}. En cas d'échec,
    on garde les valeurs en dur ci-dessus.
    """
    global BUOYS_GPS, BUOYS_LOCAL
    if not os.path.exists(path):
        return False
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        new_buoys: Dict[str, Tuple[float, float]] = {}
        for name, coords in data.items():
            if not isinstance(coords, (list, tuple)) or len(coords) != 2:
                continue
            new_buoys[name.upper()] = (float(coords[0]), float(coords[1]))
        if new_buoys:
            BUOYS_GPS = {**BUOYS_GPS, **new_buoys}
            BUOYS_LOCAL = _derive_buoys_local()
            log.info("[CONFIG] Bouées rechargées depuis %s (%d entrées)",
                     path, len(new_buoys))
            return True
    except Exception as e:
        log.warning("[CONFIG] load_buoys_from_file(%s) : %s", path, e)
    return False


def _derive_buoys_local() -> Dict[str, "BuoyPos"]:
    """Calcule les positions locales (east, north) à partir des positions GPS.
    Pour l'usage exclusif des outils de visualisation (replay, simulateur)."""
    import math
    earth = 6_371_000.0
    lat0 = math.radians(ORIGIN_LAT)
    out: Dict[str, BuoyPos] = {}
    for name, (lat, lon) in BUOYS_GPS.items():
        dlat = math.radians(lat - ORIGIN_LAT)
        dlon = math.radians(lon - ORIGIN_LON)
        east = dlon * math.cos(lat0) * earth
        north = dlat * earth
        out[name] = BuoyPos(east=east, north=north)
    return out


# Dérivé automatiquement de BUOYS_GPS — ne pas éditer manuellement.
BUOYS_LOCAL: Dict[str, BuoyPos] = _derive_buoys_local()

# Chemin par défaut où buoy_entry.py écrit les coordonnées du jour
BUOYS_OVERRIDE_PATH: str = os.environ.get(
    "BUOYS_OVERRIDE_PATH",
    "/etc/stormwings/buoys_today.json",
)
load_buoys_from_file(BUOYS_OVERRIDE_PATH)


# ════════════════════════════════════════════════════════════════════════
# SÉQUENCE DU PARCOURS N°3
# ════════════════════════════════════════════════════════════════════════
# (waypoint, side) où side = "starboard" → bouée à droite (tribord) du drone
#                              "port"      → bouée à gauche  (bâbord)
#                              "gate"      → franchissement de porte
#
# Description ordonnée du parcours :
#   Départ porte A-B (cap O) → C(stbd) → D(port) → E(port) → C(stbd)
#                            → F(stbd) → G(stbd) → C(stbd) → Arrivée porte A-B
COURSE_3_LEGS = [
    {"name": "DEPART",       "buoy": "AB", "side": "gate"},
    {"name": "ETAPE_1_C",    "buoy": "C",  "side": "starboard"},
    {"name": "ETAPE_2_D",    "buoy": "D",  "side": "port"},
    {"name": "ETAPE_3_E",    "buoy": "E",  "side": "port"},
    {"name": "ETAPE_4_C",    "buoy": "C",  "side": "starboard"},
    {"name": "ETAPE_5_F",    "buoy": "F",  "side": "starboard"},
    {"name": "ETAPE_6_G",    "buoy": "G",  "side": "starboard"},
    {"name": "ETAPE_7_C",    "buoy": "C",  "side": "starboard"},
    {"name": "ARRIVEE",      "buoy": "AB", "side": "gate"},
]

# Marge de contournement des bouées
BUOY_CLEARANCE_M: float = 2.0         # rayon de sécurité bouée (offset waypoint)
GATE_HALF_WIDTH_M: float = 18.0       # demi-largeur ligne A-B (≈ 30 m / 2 + marge)

# Rayons de capture ADAPTATIFS selon la qualité du fix GPS (cf. README2)
CAPTURE_RADIUS_RTK: float = 4.0       # m — fix_type ≥ 5 (RTK Float ou Fixed)
CAPTURE_RADIUS_GPS: float = 7.0       # m — fix_type 3-4 (3D / DGPS)
# Ancien nom conservé pour compat — fallback GPS standard
WAYPOINT_VALIDATION_M: float = CAPTURE_RADIUS_GPS


# ════════════════════════════════════════════════════════════════════════
# SÉQUENCE PÉNALITÉ
# ════════════════════════════════════════════════════════════════════════
# Quand une pénalité est annoncée :
#   1) Aller vers Z1 (passage A → Z1)
#   2) Enrouler Z2 BÂBORD
#   3) Enrouler Z1 BÂBORD
#   4) Reprendre l'étape interrompue
PENALTY_LEGS = [
    {"name": "PEN_1_Z1",  "buoy": "Z1", "side": "port"},
    {"name": "PEN_2_Z2",  "buoy": "Z2", "side": "port"},
    {"name": "PEN_3_Z1",  "buoy": "Z1", "side": "port"},
]
# Le pilote a 5 s pour basculer en MANUEL avant que l'auto prenne la main
PENALTY_DECISION_TIMEOUT_S: float = 5.0
# Limite réglementaire : reprise RC max 30 s puis l'auto reprend
PENALTY_MANUAL_MAX_S: float = 30.0


# ════════════════════════════════════════════════════════════════════════
# NAVIGATION — POLAIRE DE VITESSE & VMG
# ════════════════════════════════════════════════════════════════════════
# Modèle empirique tabulé. La table de référence est dans navigation/polar.py
# et peut être OVERRIDÉE le J-1 par calibration/polar_calibration.py qui écrit
# un fichier JSON listé ici.
POLAR_TABLE_PATH: str = os.environ.get(
    "POLAR_TABLE_PATH",
    "/etc/stormwings/polar_table.json",
)

POLAR_THETA_MIN_DEG: float = 35.0   # angle minimum de remontée au vent
# Angle optimal upwind par défaut (raffiné via polaire calibrée si dispo)
VMG_UPWIND_ANGLE_DEG: float = 45.0  # entre 42° et 48° selon force du vent

# Hystérésis pour éviter les tacks intempestifs
TACK_DECISION_HYSTERESIS_DEG: float = 8.0
MIN_TIME_BETWEEN_TACKS_S: float = 6.0   # 1 tack coûte ~3-5 s de vitesse perdue


# ════════════════════════════════════════════════════════════════════════
# CHAMP DE POTENTIEL — ANTI-COLLISION INTER-DRONES
# ════════════════════════════════════════════════════════════════════════
DRONE_REPULSION_RADIUS_M: float = 4.0  # rayon d'anti-collision drone-drone
DRONE_REPULSION_GAIN: float = 30.0     # force répulsion (sans dimension)
BUOY_REPULSION_RADIUS_M: float = BUOY_CLEARANCE_M
BUOY_REPULSION_GAIN: float = 20.0


# ════════════════════════════════════════════════════════════════════════
# BASCULE MANUEL ↔ AUTO via le levier de mode (CH5 récepteur Joysway)
# ════════════════════════════════════════════════════════════════════════
# Avec le setup NouvelEncodeur, le levier 3 positions de la J4C05 sort sur
# CH5 du récepteur J5C01R, qui est encodé en PPM canal 6 par le Nano. Côté
# Cube, ArduPilot voit donc ce levier dans `RC_CHANNELS.chan6_raw` :
#   chan6 < MODE_THRESHOLD_LOW  → Pi en contrôle (mode AUTO)
#   chan6 > MODE_THRESHOLD_HIGH → RC en contrôle (mode MANUAL téléopéré)
#   Entre les deux               → état précédent conservé (hystérésis)
MODE_THRESHOLD_LOW: int = 1300       # µs
MODE_THRESHOLD_HIGH: int = 1500      # µs

# Mapping des canaux côté MAVLink (RC_CHANNELS / RC_CHANNELS_OVERRIDE).
# Ces numéros correspondent au PPM en sortie du Nano (cf. RCMAP_* du
# fichier docs/ardupilot_params.txt et CubeNouvelEncodeur.param).
CH_RUDDER: int = 4                   # PPM ch4 ← D3 Nano ← CH1 récepteur (gouvernail)
CH_SAIL: int = 5                     # PPM ch5 ← D4 Nano ← CH2 récepteur (voile)
CH_MODE: int = 6                     # PPM ch6 ← D7 Nano ← CH5 récepteur (levier mode)

# Aliases rétrocompat — anciens noms encore utilisés en lecture par certains
# tests/scripts. Ne pas s'en servir dans le code neuf : utiliser MODE_*.
CH3_THRESHOLD_LOW: int = MODE_THRESHOLD_LOW
CH3_THRESHOLD_HIGH: int = MODE_THRESHOLD_HIGH


# ════════════════════════════════════════════════════════════════════════
# MAPPING PWM ↔ ANGLES PHYSIQUES
# ════════════════════════════════════════════════════════════════════════
# Safran : ±45° max ↔ 1100 / 1900 µs (centre 1500)
RUDDER_PWM_MIN: int = 1100
RUDDER_PWM_MAX: int = 1900
RUDDER_PWM_TRIM: int = 1500
RUDDER_ANGLE_MAX_DEG: float = 45.0

# Voile (winch) : 0 % (border) → 1100 µs, 100 % (choquer) → 1900 µs
SAIL_PWM_MIN: int = 1100   # voile bordée à fond (vent de face)
SAIL_PWM_MAX: int = 1900   # voile choquée à fond (vent arrière)


# ════════════════════════════════════════════════════════════════════════
# PID DU CAP (consigne envoyée au gouvernail)
# ════════════════════════════════════════════════════════════════════════
HEADING_PID_KP: float = 2.0
HEADING_PID_KI: float = 0.1
HEADING_PID_KD: float = 0.5
HEADING_PID_OUTPUT_MAX: float = 1.0    # normalisé [-1, 1] → mappé sur PWM


# ════════════════════════════════════════════════════════════════════════
# DÉTECTION BLOCAGE (sans caméra) — cf. README2 §"Détection blocage"
# ════════════════════════════════════════════════════════════════════════
# Drone bloqué = 2 conditions sur 3 vraies pendant >STALL_DURATION_S secondes
STALL_SPEED_THRESHOLD_MS: float = 0.15      # m/s — vitesse sol quasi-nulle
STALL_RUDDER_THRESHOLD_DEG: float = 15.0    # rudder > 15° → commandes actives
STALL_WP_DISTANCE_HYSTERESIS_M: float = 0.5 # < 0.5 m d'évolution sur la fenêtre
STALL_WINDOW_S: float = 3.0                  # fenêtre d'analyse (s)
STALL_DURATION_S: float = 3.0                # durée minimale pour déclencher

# Réactions par paliers :
STALL_LIGHT_REACTION_MAX_S: float = 8.0     # 0-8 s : choquer voile + virer
STALL_MED_REACTION_MAX_S: float = 15.0      # 8-15 s : manœuvre de dégagement forcée
# > 15 s : alerte / proposer reprise RC


# ════════════════════════════════════════════════════════════════════════
# WATCHDOG / TIMEOUTS
# ════════════════════════════════════════════════════════════════════════
HEARTBEAT_TIMEOUT_S: float = 2.0          # Cube Orange+ heartbeat
GPS_FIX_TIMEOUT_S: float = 5.0
WIND_FALLBACK_TIMEOUT_S: float = 30.0     # si pas de WIND msg → mode dégradé
LORA_NEIGHBOR_TIMEOUT_S: float = 180.0    # voisin équipe perdu (broadcast 60 s)
ADVERSARY_SILENT_TIMEOUT_S: float = 10.0  # adversaire silencieux → obstacle figé
LOW_BATTERY_PCT: float = 20.0
LOW_BATTERY_VOLTAGE_V: float = 11.1


# ════════════════════════════════════════════════════════════════════════
# LOGGING
# ════════════════════════════════════════════════════════════════════════
LOG_DIR: str = os.environ.get(
    "STORMWINGS_LOG_DIR",
    "/home/admin/stormwings/logs",
)
LOG_LEVEL: str = os.environ.get("STORMWINGS_LOG_LEVEL", "INFO")


# ════════════════════════════════════════════════════════════════════════
# Helpers identité
# ════════════════════════════════════════════════════════════════════════
def is_my_id(boat_id: str) -> bool:
    return boat_id.upper() == DRONE_ID


def is_teammate(boat_id: str) -> bool:
    bid = boat_id.upper()
    return bid in TEAM_BOATS and bid != DRONE_ID


def is_enemy(boat_id: str) -> bool:
    return boat_id.upper() in ENEMY_BOATS


def capture_radius_for_fix(rtk_fixed: bool) -> float:
    """Rayon de capture WAYPOINT à utiliser selon la qualité du fix.

    Args:
        rtk_fixed: True si fix_type GPS_RAW_INT ≥ 5 (RTK Float ou Fixed).
    """
    return CAPTURE_RADIUS_RTK if rtk_fixed else CAPTURE_RADIUS_GPS


if __name__ == "__main__":
    # Auto-diagnostic : afficher la config active
    print("=== StormWings config ===")
    print(f"DRONE_ID            = {DRONE_ID}  (num={DRONE_NUM})")
    print(f"DEFAULT_ROLE        = {DEFAULT_ROLE}  (strategy={STRATEGY})")
    print(f"RACE_START_OFFSET_S = {RACE_START_OFFSET_S}")
    print(f"MAVLINK_PORT        = {MAVLINK_PORT} @ {MAVLINK_BAUD} baud")
    print(f"LORA_PORT           = {LORA_PORT}")
    print(f"TDMA slot           = {MY_TDMA_OFFSET_MS} ms / {TDMA_FRAME_MS} ms")
    print(f"Capture radius      = RTK {CAPTURE_RADIUS_RTK} m / GPS {CAPTURE_RADIUS_GPS} m")
    print(f"Buoys               = {list(BUOYS_LOCAL.keys())}")
    print(f"Course legs         = {len(COURSE_3_LEGS)}")
    print(f"Penalty legs        = {len(PENALTY_LEGS)}")
