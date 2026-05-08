"""
StormWings — configuration centrale (régate 2 drones, parcours sélectionnable).

USAGE
    DRONE_ID=U1B1 STORMWINGS_MODE=REGATE COURSE_NUMBER=2 python3 main.py
    DRONE_ID=U1B2 STORMWINGS_MODE=ESSAI  COURSE_NUMBER=1 WIND_DIR_DEG=270 \
        WIND_SPEED_MS=4.5 python3 main.py

VARIABLES D'ENVIRONNEMENT
    DRONE_ID            U1B1 ou U1B2 (défaut U1B1)
    STORMWINGS_MODE     ESSAI ou REGATE (défaut REGATE)
    COURSE_NUMBER       1 (banane) ou 2 ("5 bouées" briefing 8/5) — défaut 2
    WIND_DIR_DEG        Direction vent en mode ESSAI (défaut 90 = E)
    WIND_SPEED_MS       Vitesse vent en mode ESSAI (défaut 4.5)
    WIND_FALLBACK_DIR   Direction fallback en RÉGATE si orga muette (défaut 90)
    WIND_FALLBACK_SPD   Vitesse fallback en RÉGATE                  (défaut 4.5)
    BUOYS_OVERRIDE_PATH Chemin JSON bouées (défaut /etc/stormwings/buoys_today.json)

ID OFFICIELS (cf. BATTLEBOATS_LORA_PROTOCOL_v5.pdf, équipe UTT)
    U1B1 → drone 1 (Scout, départ T+0, agressif)
    U1B2 → drone 2 (Optimizer, départ T+30s, utilise data Scout)

PARCOURS — coordonnées briefing du 9 mai 2026 (Plage du Mourillon, Toulon)
    1 → "banane"      4 bouées (1, 2, 3, 4) + pénalité (P1, P2). 2 tours.
                       Portes 1-2 (départ/arrivée) et 3-4 (au vent).
                       Stratégie StormWings : SPIRALE ALTERNÉE figée au
                       franchissement de la ligne de départ —
                       côté T = 3 → 1 → 4 → arrivée  (si bouée 3 + au vent)
                       côté B = 4 → 2 → 3 → arrivée  (si bouée 4 + au vent)
    2 → "5 bouées"    5 bouées (1, 2, 3, 4, 5) + pénalité (P1, P2). 2 tours.
                       Porte 1-2 (départ/arrivée). Séquence par tour :
                       porte 1-2 → bouée 5 → bouée 4 → bouée 3 → bouée 1.
                       Tous les contournements en BÂBORD par défaut
                       (à confirmer au briefing matin).

HARDWARE — paramètres ArduPilot validés (setup NouvelEncodeur officiel)
    Encodeur PPM = Arduino Nano flashé ArduPPM v2.3.16. Mapping côté Cube :
        chan4_raw  ← CH1 récepteur Joysway J5C01R  → safran (RCMAP_ROLL=4)
        chan5_raw  ← CH2 récepteur                  → voile  (RCMAP_THROTTLE=5)
        chan6_raw  ← CH5 récepteur (levier mode)    → MODE_CH=6
    Voir docs/ARDUPILOT_PARAMS.md et SwarmZ_fichier_Orga/NouvelEncodeur/.

STRATÉGIE GÉNÉRALE
    - Mode MANUAL permanent côté Cube ; le Pi écrit RC_CHANNELS_OVERRIDE
      sur chan4/chan5 (safran/voile) à 10 Hz.
    - Bascule MANUEL/AUTO via le levier 3 positions lu sur chan6_raw.
    - Position EKF du Cube + corrections RTCM3 du HERE4 → fix RTK.
    - Rayon de capture des bouées ADAPTATIF selon le fix : 4 m en RTK,
      7 m en fallback GPS standard.
    - Pénalité : NON détectée automatiquement. Le pilote constate la dérive,
      reprend la main (levier HAUT), fait le tour P1/P2 à la radio,
      remet AUTO (levier BAS). Le drone reprend le parcours à course.current_idx.
"""

import json
import logging
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

log = logging.getLogger(__name__)


# ════════════════════════════════════════════════════════════════════════
# IDENTITÉ DU DRONE
# ════════════════════════════════════════════════════════════════════════
DRONE_ID: str = os.environ.get("DRONE_ID", "U1B1").upper()
TEAM_ID: str = "UTT"

# Mapping ID → numéro logique (1 = Scout, 2 = Optimizer)
_DRONE_NUM_MAP = {"U1B1": 1, "U1B2": 2}
DRONE_NUM: int = _DRONE_NUM_MAP.get(DRONE_ID, 1)

# Liste des coéquipiers UTT (régate à 2 drones)
TEAM_BOATS = ("U1B1", "U1B2")
ENEMY_BOATS = (
    "D2B1", "D2B2", "D2B3",   # DaVinci Hive
    "I3B1", "I3B2", "I3B3",   # IPSA
    "E4B1", "E4B2", "E4B3",   # ENSEEIHT
)


# ════════════════════════════════════════════════════════════════════════
# MODE OPÉRATOIRE — ESSAI vs RÉGATE
# ════════════════════════════════════════════════════════════════════════
# ESSAI  : phase de test — vent simulé via env WIND_DIR_DEG/WIND_SPEED_MS,
#          pas d'écoute orga, écoute alliés uniquement (les 2 drones se parlent).
# REGATE : course officielle — vent reçu de l'orga (W|... toutes les 60s),
#          écoute alliés + ennemis + WIND, fallback statique si orga muette.
MODE: str = os.environ.get("STORMWINGS_MODE", "REGATE").upper()
if MODE not in ("ESSAI", "REGATE"):
    log.warning("[CONFIG] STORMWINGS_MODE=%s invalide, fallback REGATE", MODE)
    MODE = "REGATE"


def is_essai() -> bool:
    return MODE == "ESSAI"


def is_regate() -> bool:
    return MODE == "REGATE"


# ════════════════════════════════════════════════════════════════════════
# SÉLECTION DU PARCOURS
# ════════════════════════════════════════════════════════════════════════
COURSE_NUMBER: int = int(os.environ.get("COURSE_NUMBER", "2"))
if COURSE_NUMBER not in (1, 2):
    log.warning("[CONFIG] COURSE_NUMBER=%d invalide (1 ou 2 attendu), fallback 2",
                COURSE_NUMBER)
    COURSE_NUMBER = 2


# ════════════════════════════════════════════════════════════════════════
# RÔLES — régate à 2 drones (Scout + Optimizer)
# ════════════════════════════════════════════════════════════════════════
ROLE_SCOUT: str = "SCOUT"
ROLE_OPTIMIZER: str = "OPTIMIZER"

# Profils par drone — stratégie "custom" validée :
#   U1B1 (Scout)     : départ immédiat, agressif, mesure le vent réel au largue
#                      et broadcast pour aider U1B2.
#   U1B2 (Optimizer) : départ T+30s, fait un cercle/figure-8 derrière la porte
#                      pendant les 30 premières secondes, puis prend la main avec
#                      polaire calibrée par les data du Scout.
_DRONE_PROFILES: Dict[str, Dict] = {
    "U1B1": {
        "role": ROLE_SCOUT,
        "tdma_slot_ms": 0,
        "race_start_offset_s": 0.0,
        "strategy": "scout_aggressive",
    },
    "U1B2": {
        "role": ROLE_OPTIMIZER,
        "tdma_slot_ms": 750,
        "race_start_offset_s": 30.0,    # part 30 s après le Scout
        "strategy": "optimizer_smart",
    },
}
if DRONE_ID not in _DRONE_PROFILES:
    log.warning("[CONFIG] DRONE_ID=%s inconnu pour la régate 2 drones, fallback U1B1", DRONE_ID)
    _PROFILE = _DRONE_PROFILES["U1B1"]
else:
    _PROFILE = _DRONE_PROFILES[DRONE_ID]

DEFAULT_ROLE: str = _PROFILE["role"]
STRATEGY: str = _PROFILE["strategy"]
RACE_START_OFFSET_S: float = _PROFILE["race_start_offset_s"]

ROLE_REEVAL_PERIOD_S: float = 5.0


# ════════════════════════════════════════════════════════════════════════
# MATÉRIEL EMBARQUÉ
# ════════════════════════════════════════════════════════════════════════
MAVLINK_PORT: str = "/dev/serial0"
MAVLINK_BAUD: int = 57600

LORA_PORT: str = "/dev/ttyUSB0"
LORA_BAUD: int = 115200      # auto-géré par Meshtastic, info seulement


# ════════════════════════════════════════════════════════════════════════
# RÉSEAU LORA BATTLEBOATS  ──  cf. BATTLEBOATS_LORA_PROTOCOL_v5.pdf
# ════════════════════════════════════════════════════════════════════════
# Canal partagé "BATTLEBOATS" sur 868 MHz EU, géré par l'organisation.
# Tous les nœuds du mesh sont VISIBLES par tout le monde et ÉMETTENT en
# broadcast (pas de routage applicatif).
#
# RÔLES NŒUD (cf. PDF §1.4) :
#   WIND  → station météo de l'orga (à terre)   — émet W|... toutes les 60 s
#   ORG   → base de plot/scoring de l'orga      — écoute uniquement
#   U1Bn  → bateaux UTT (nous)                  — émet P|... toutes les 60 s
#   D2Bn  → DaVinci Hive (concurrent)           — émet P|... toutes les 60 s
#   I3Bn  → IPSA (concurrent)                   — émet P|... toutes les 60 s
#   E4Bn  → Enseeiht (concurrent)               — émet P|... toutes les 60 s
#
# QUI ÉMET QUOI (cf. PDF §3) :
#   Type     | Émetteur | Période | Format                              |
#   W|...    | WIND     | 60 s    | W|<dir>|<spd_ms×10>|<unix_ts>       |
#   P|...    | bateaux  | 60 s    | P|<id>|<lat×1e5>|<lon×1e5>|<hdg>|<spd_kn×10>
#
# OBLIGATIONS RÉGLEMENTAIRES (cf. PDF §3.3) :
#   - Tout bateau qui n'émet pas P|... est invisible → pénalité sportive
#   - L'émission DOIT continuer même sans fix GPS (P|<id>|0|0|0|0)
#   - L'émission DOIT continuer même en MANUAL et en PENALITE
#
# DANS NOTRE CODE :
#   TX P|...  → comms.lora_iface.LoRaInterface.broadcast_position()
#               cadencé par swarm.tdma.TDMAScheduler (60 s, U1B1 à T+0,
#               U1B2 à T+30 — étalement intra-équipe pour éviter collision air)
#   RX P|...  → main._on_position_received()
#               teammate (UTT)  → roles.update_teammate (coordination essaim)
#               enemy           → _last_adversary_pos    (anti-collision tactique)
#   RX W|...  → main._on_wind_received → wind.push_orga_wind (source météo)

LORA_CHANNEL_NAME: str = "BATTLEBOATS"
LORA_REGION: str = "EU_868"
LORA_MODEM_PRESET: str = "MEDIUM_FAST"     # SF9/250 kHz, ~3-5 km en mer
LORA_HOP_LIMIT: int = 3

# Identifiants nœud orga (NE PAS confondre avec les concurrents)
LORA_NODE_WIND: str = "WIND"
LORA_NODE_ORG: str = "ORG"

# Périodes d'émission imposées par le règlement (60 s)
LORA_BROADCAST_POSITION_PERIOD_S: float = 60.0   # P|... obligatoire
LORA_BROADCAST_WIND_PERIOD_S: float = 60.0       # W|... attendu de l'orga


# ════════════════════════════════════════════════════════════════════════
# FRÉQUENCES DES BOUCLES
# ════════════════════════════════════════════════════════════════════════
NAV_LOOP_HZ: float = 10.0
COMM_LOOP_HZ: float = 2.0
NAV_DT: float = 1.0 / NAV_LOOP_HZ
COMM_DT: float = 1.0 / COMM_LOOP_HZ

OVERRIDE_REFRESH_HZ: float = 10.0

# Alias rétrocompatible — pointe sur la constante "officielle" plus haut
LORA_BROADCAST_PERIOD_S: float = LORA_BROADCAST_POSITION_PERIOD_S


# ════════════════════════════════════════════════════════════════════════
# PROTOCOLE LORA TDMA — 2 drones × 750 ms = trame 1500 ms
# ════════════════════════════════════════════════════════════════════════
TDMA_FRAME_MS: int = 1500
TDMA_SLOT_MS: int = 750
TDMA_SLOTS = {"U1B1": 0, "U1B2": 750}
MY_TDMA_OFFSET_MS: int = TDMA_SLOTS.get(DRONE_ID, 0)


# ════════════════════════════════════════════════════════════════════════
# REPÈRE GPS LOCAL — origine pour outils de visualisation
# ════════════════════════════════════════════════════════════════════════
# Plage du Mourillon Ouest, Toulon — barycentre des bouées 1-5 du briefing
# du 9 mai 2026. Sert uniquement à l'affichage local (matplotlib), tous les
# calculs de navigation se font en GPS.
ORIGIN_LAT: float = 43.10723
ORIGIN_LON: float = 5.94099


@dataclass(frozen=True)
class BuoyPos:
    """Position locale (east, north) en mètres — pour outils de visualisation."""
    east: float
    north: float


# ════════════════════════════════════════════════════════════════════════
# BOUÉES — coordonnées officielles communiquées par l'orga le 8/5/2026 (J-1)
# ════════════════════════════════════════════════════════════════════════
# ⚠️ Le briefing matin (J0, 9h30) peut affiner ces valeurs. Si c'est le cas,
#    saisir les corrections via `python3 -m tools.buoy_entry`, qui écrit
#    /etc/stormwings/buoys_today.json et override automatiquement les
#    valeurs ci-dessous (sans toucher au code).
#
# Format : "NOM_BOUÉE": (latitude_décimale, longitude_décimale)
#
# RÈGLES DE PARTAGE :
#   - Bouées 1, 2, 3, 4 : COMMUNES aux parcours 1 (banane) et 2 (5 bouées).
#   - Bouée 5            : utilisée UNIQUEMENT par le parcours 2.
#   - Bouées P1, P2      : pénalité P1/P2 sur les DEUX parcours (à confirmer
#                          au briefing — placeholders ci-dessous).

_BUOYS_REGATE: Dict[str, Tuple[float, float]] = {
    # --- Bouées de régate (briefing officiel J-1) ---
    "1":  (43.10729166666667, 5.9412796666666665),   # porte départ — tribord
    "2":  (43.1074215,        5.941155833333333),    # porte départ — bâbord
    "3":  (43.10696383333333, 5.940718666666667),    # bouée sud
    "4":  (43.10710616666667, 5.9405145),            # bouée ouest
    "5":  (43.107527,         5.940586),             # bouée nord (parcours 2)
    # --- Pénalité P1/P2 ---
    # Coordonnées NON encore communiquées par l'orga ; placeholder décalé
    # ~50 m au sud-est de la porte 1-2 (à éditer au briefing matin si l'orga
    # publie les coords). Comme la pénalité est gérée manuellement à la radio
    # par le pilote, ces valeurs ne sont JAMAIS waypoint actif du runtime —
    # elles servent juste de référence visuelle et de cohérence interne.
    "P1": (43.10720,          5.94155),
    "P2": (43.10720,          5.94170),
}

BUOYS_GPS: Dict[str, Tuple[float, float]] = dict(_BUOYS_REGATE)


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


BUOYS_LOCAL: Dict[str, BuoyPos] = _derive_buoys_local()

BUOYS_OVERRIDE_PATH: str = os.environ.get(
    "BUOYS_OVERRIDE_PATH",
    "/etc/stormwings/buoys_today.json",
)
load_buoys_from_file(BUOYS_OVERRIDE_PATH)


# ════════════════════════════════════════════════════════════════════════
# DÉFINITION DES 2 PARCOURS — séquences ordonnées de "legs"
# ════════════════════════════════════════════════════════════════════════
# Format leg : {"name": str, "buoy": str, "side": "starboard"|"port"|"gate"}
#
#   "starboard" → bouée à TRIBORD du drone au moment du contournement
#                 (le drone passe à gauche de la bouée vue d'en haut)
#   "port"      → bouée à BÂBORD du drone (drone passe à droite)
#   "gate"      → franchissement de porte entre 2 bouées (champ buoy = "12",
#                 "34", etc. — le code lit chaque caractère)

# --- PARCOURS N°1 — Banane (parcours construit) ---
# 2 tours, chaque porte (1-2, 3-4) impose d'enrouler la bouée 1 OU 2 (resp.
# 3 OU 4). La stratégie StormWings est une SPIRALE ALTERNÉE :
#
#   1. Au franchissement de la ligne de départ (1-2), on regarde le vent.
#   2. On choisit la bouée la plus AU VENT pour le 1er upwind :
#        - Si bouée 3 plus exposée → côté T : séquence 3 → 1 → 4 → arrivée
#        - Si bouée 4 plus exposée → côté B : séquence 4 → 2 → 3 → arrivée
#   3. Le choix est FIGÉ : il ne change plus même si le vent évolue.
#   4. Tour 1 (haut + bas) garde le MÊME côté ; tour 2 bascule au côté OPPOSÉ.
#
# Cette logique est implémentée par CourseManager._apply_course1_choice()
# qui réécrit dynamiquement les 3 legs intermédiaires au runtime. Les valeurs
# définies ici sont des PLACEHOLDERS (porte 34 / porte 12) qui n'apparaissent
# que jusqu'au franchissement du DEPART.
COURSE_1_LEGS: List[Dict] = [
    {"name": "DEPART",          "buoy": "12", "side": "gate"},
    # Les 3 legs ci-dessous sont REMPLACÉS au runtime par la spirale (T ou B).
    {"name": "BOUEE_HAUTE_T1",  "buoy": "34", "side": "gate"},
    {"name": "BOUEE_BASSE_T1",  "buoy": "12", "side": "gate"},
    {"name": "BOUEE_HAUTE_T2",  "buoy": "34", "side": "gate"},
    {"name": "ARRIVEE",         "buoy": "12", "side": "gate"},
]

# --- PARCOURS N°2 — 5 bouées (cf. Parcours2.png briefing du 8/5/2026) ---
# Géométrie : porte 1-2 départ/arrivée, plus 3 bouées formant une boucle
# (5 au nord, 4 à l'ouest, 3 au sud) puis retour par 1 vers la porte. 2 tours.
#
# Sens de contournement : tous BÂBORD par défaut (à confirmer au briefing
# matin — modifier `_COURSE2_DEFAULT_SIDE` ci-dessous au besoin).
_COURSE2_DEFAULT_SIDE: str = "port"

COURSE_2_LEGS: List[Dict] = [
    {"name": "DEPART",  "buoy": "12", "side": "gate"},
    # Tour 1
    {"name": "T1_5",    "buoy": "5",  "side": _COURSE2_DEFAULT_SIDE},
    {"name": "T1_4",    "buoy": "4",  "side": _COURSE2_DEFAULT_SIDE},
    {"name": "T1_3",    "buoy": "3",  "side": _COURSE2_DEFAULT_SIDE},
    {"name": "T1_1",    "buoy": "1",  "side": _COURSE2_DEFAULT_SIDE},
    # Tour 2
    {"name": "T2_5",    "buoy": "5",  "side": _COURSE2_DEFAULT_SIDE},
    {"name": "T2_4",    "buoy": "4",  "side": _COURSE2_DEFAULT_SIDE},
    {"name": "T2_3",    "buoy": "3",  "side": _COURSE2_DEFAULT_SIDE},
    {"name": "T2_1",    "buoy": "1",  "side": _COURSE2_DEFAULT_SIDE},
    {"name": "ARRIVEE", "buoy": "12", "side": "gate"},
]

# Dictionnaire d'index pour sélection runtime
_COURSES_LEGS: Dict[int, List[Dict]] = {
    1: COURSE_1_LEGS,
    2: COURSE_2_LEGS,
}

# Séquence active (pour le runtime — alias rétrocompat avec l'ancien code)
COURSE_LEGS: List[Dict] = _COURSES_LEGS[COURSE_NUMBER]


# ════════════════════════════════════════════════════════════════════════
# SÉQUENCE DE PÉNALITÉ — commune aux 2 parcours retenus (briefing du 8/5/2026)
# ════════════════════════════════════════════════════════════════════════
# Cf. Reglement Battleboats 2026 V2.3 §6 "Système de pénalité"
#
# Les parcours 1 (banane) et 2 (5 bouées) partagent la même paire de bouées
# de pénalité P1/P2. La séquence : passer entre 2 et P1 → P2 bâbord → P1 bâbord.
#
# Cette séquence n'est PAS exécutée automatiquement par le code : la pénalité
# est constatée par le pilote, qui reprend la main au levier MANUEL et fait
# le tour à la radio. La structure ci-dessous sert uniquement de référence
# pour le module penalty_manager (logging, snapshot, reprise).
PENALTY_LEGS_BANANE: List[Dict] = [
    {"name": "PEN_1_P1",  "buoy": "P1", "side": "port"},
    {"name": "PEN_2_P2",  "buoy": "P2", "side": "port"},
    {"name": "PEN_3_P1",  "buoy": "P1", "side": "port"},
]

# Pénalité active : P1/P2 quel que soit le parcours retenu (1 ou 2).
PENALTY_LEGS: List[Dict] = PENALTY_LEGS_BANANE

# Le pilote a 5 s pour basculer en MANUEL avant que l'auto prenne la main
PENALTY_DECISION_TIMEOUT_S: float = 5.0
# Limite réglementaire (cf. règlement §6) : reprise RC max 30 s
PENALTY_MANUAL_MAX_S: float = 30.0


# ════════════════════════════════════════════════════════════════════════
# GÉOMÉTRIE — marges et rayons de capture
# ════════════════════════════════════════════════════════════════════════
BUOY_CLEARANCE_M: float = 2.0         # rayon de sécurité bouée (offset waypoint)
GATE_HALF_WIDTH_M: float = 18.0       # demi-largeur porte (~30 m / 2 + marge)

# Rayons de capture ADAPTATIFS selon la qualité du fix GPS
CAPTURE_RADIUS_RTK: float = 4.0       # m — fix_type ≥ 5 (RTK Float ou Fixed)
CAPTURE_RADIUS_GPS: float = 7.0       # m — fix_type 3-4 (3D / DGPS)
WAYPOINT_VALIDATION_M: float = CAPTURE_RADIUS_GPS   # alias rétrocompat


# ════════════════════════════════════════════════════════════════════════
# VENT — sources selon le mode opératoire
# ════════════════════════════════════════════════════════════════════════
# En ESSAI :
#     - Le vent est fixé via WIND_DIR_DEG / WIND_SPEED_MS
#     - Le wind_estimator n'écoute pas l'orga (filtre LoRa)
# En RÉGATE :
#     - Le wind_estimator écoute W|... toutes les 60 s du nœud WIND
#     - Si pas reçu depuis WIND_FALLBACK_TIMEOUT_S → fallback statique
#       (WIND_FALLBACK_DIR / WIND_FALLBACK_SPD, à régler au briefing matin)

# Vent ESSAI — valeurs simulées par défaut (E 4.5 m/s)
WIND_ESSAI_DIR_DEG: float = float(os.environ.get("WIND_DIR_DEG", "90"))
WIND_ESSAI_SPEED_MS: float = float(os.environ.get("WIND_SPEED_MS", "4.5"))

# Vent fallback en RÉGATE (si orga muette > 30 s)
WIND_FALLBACK_DIR_DEG: float = float(os.environ.get("WIND_FALLBACK_DIR", "90"))
WIND_FALLBACK_SPEED_MS: float = float(os.environ.get("WIND_FALLBACK_SPD", "4.5"))


# ════════════════════════════════════════════════════════════════════════
# NAVIGATION — POLAIRE DE VITESSE & VMG
# ════════════════════════════════════════════════════════════════════════
POLAR_TABLE_PATH: str = os.environ.get(
    "POLAR_TABLE_PATH",
    "/etc/stormwings/polar_table.json",
)

POLAR_THETA_MIN_DEG: float = 35.0
VMG_UPWIND_ANGLE_DEG: float = 45.0

TACK_DECISION_HYSTERESIS_DEG: float = 8.0
MIN_TIME_BETWEEN_TACKS_S: float = 6.0


# ════════════════════════════════════════════════════════════════════════
# DÉPART — paramètres spécifiques (loiter U1B2 derrière la porte)
# ════════════════════════════════════════════════════════════════════════
# Quand DRONE_ID=U1B2 et race_started=False et race_start_offset_s>0 :
#   - On fait un FIGURE-8 ou cercle 50 m derrière la porte de départ pour
#     calibrer la polaire pendant que U1B1 part en avant.
#   - À RACE_START_OFFSET_S secondes après le top départ orga
#     (qui est inféré au moment du levier BAS), on lance la course.
LOITER_RADIUS_M: float = 25.0          # rayon du cercle de loiter
LOITER_BEHIND_GATE_M: float = 50.0     # distance derrière la porte de départ
LOITER_TURN_PERIOD_S: float = 30.0     # période complète d'un cercle/figure-8


# ════════════════════════════════════════════════════════════════════════
# CHAMP DE POTENTIEL — ANTI-COLLISION INTER-DRONES
# ════════════════════════════════════════════════════════════════════════
DRONE_REPULSION_RADIUS_M: float = 4.0
DRONE_REPULSION_GAIN: float = 30.0
BUOY_REPULSION_RADIUS_M: float = BUOY_CLEARANCE_M
BUOY_REPULSION_GAIN: float = 20.0


# ════════════════════════════════════════════════════════════════════════
# BASCULE MANUEL ↔ AUTO via le levier de mode (CH5 récepteur Joysway)
# ════════════════════════════════════════════════════════════════════════
MODE_THRESHOLD_LOW: int = 1300       # µs → AUTO
MODE_THRESHOLD_HIGH: int = 1500      # µs → MANUAL

# Mapping des canaux côté MAVLink (RC_CHANNELS / RC_CHANNELS_OVERRIDE).
CH_RUDDER: int = 4                   # PPM ch4 ← D3 Nano ← CH1 récepteur
CH_SAIL: int = 5                     # PPM ch5 ← D4 Nano ← CH2 récepteur
CH_MODE: int = 6                     # PPM ch6 ← D7 Nano ← CH5 récepteur

# Aliases rétrocompat (anciens scripts/tests)
CH3_THRESHOLD_LOW: int = MODE_THRESHOLD_LOW
CH3_THRESHOLD_HIGH: int = MODE_THRESHOLD_HIGH


# ════════════════════════════════════════════════════════════════════════
# MAPPING PWM ↔ ANGLES PHYSIQUES
# ════════════════════════════════════════════════════════════════════════
RUDDER_PWM_MIN: int = 1100
RUDDER_PWM_MAX: int = 1900
RUDDER_PWM_TRIM: int = 1500
RUDDER_ANGLE_MAX_DEG: float = 45.0

SAIL_PWM_MIN: int = 1100   # voile bordée à fond
SAIL_PWM_MAX: int = 1900   # voile choquée à fond


# ════════════════════════════════════════════════════════════════════════
# PID DU CAP
# ════════════════════════════════════════════════════════════════════════
HEADING_PID_KP: float = 2.0
HEADING_PID_KI: float = 0.1
HEADING_PID_KD: float = 0.5
HEADING_PID_OUTPUT_MAX: float = 1.0


# ════════════════════════════════════════════════════════════════════════
# DÉTECTION BLOCAGE
# ════════════════════════════════════════════════════════════════════════
STALL_SPEED_THRESHOLD_MS: float = 0.15
STALL_RUDDER_THRESHOLD_DEG: float = 15.0
STALL_WP_DISTANCE_HYSTERESIS_M: float = 0.5
STALL_WINDOW_S: float = 3.0
STALL_DURATION_S: float = 3.0

STALL_LIGHT_REACTION_MAX_S: float = 8.0
STALL_MED_REACTION_MAX_S: float = 15.0


# ════════════════════════════════════════════════════════════════════════
# WATCHDOG / TIMEOUTS
# ════════════════════════════════════════════════════════════════════════
HEARTBEAT_TIMEOUT_S: float = 2.0
GPS_FIX_TIMEOUT_S: float = 5.0
WIND_FALLBACK_TIMEOUT_S: float = 30.0
LORA_NEIGHBOR_TIMEOUT_S: float = 180.0   # broadcast P| toutes les 60 s, on tolère 3×
ADVERSARY_SILENT_TIMEOUT_S: float = 10.0
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


def course_legs_for(course_num: int) -> List[Dict]:
    """Retourne la séquence de legs pour le parcours `course_num` (1 ou 2).

    Utile pour le simulateur ou les tests qui veulent comparer les 2 parcours
    sans dépendre de la variable d'env COURSE_NUMBER.
    """
    return _COURSES_LEGS.get(course_num, COURSE_2_LEGS)


def buoys_used_in_course(course_num: int) -> List[str]:
    """Retourne la liste des bouées utilisées dans le parcours (utile pour
    `tools/buoy_entry.py` qui ne demande que les bouées pertinentes).

    Les noms de gates (ex. "12") sont automatiquement décomposés en
    bouées individuelles ("1", "2"). Les bouées de pénalité P1/P2 sont
    ajoutées en queue de liste (communes aux 2 parcours).
    """
    legs = course_legs_for(course_num)
    seen: List[str] = []
    for leg in legs:
        buoy = leg["buoy"]
        # Une "gate" est un nom à plusieurs caractères chiffrés (ex. "12")
        if leg.get("side") == "gate" and len(buoy) > 1:
            for ch in buoy:
                if ch not in seen:
                    seen.append(ch)
        else:
            if buoy not in seen:
                seen.append(buoy)
    # Bouées de pénalité (toujours P1/P2)
    seen.extend(["P1", "P2"])
    return seen


if __name__ == "__main__":
    # Auto-diagnostic : afficher la config active
    print("=" * 60)
    print(f"StormWings config — DRONE_ID={DRONE_ID}  MODE={MODE}  COURSE={COURSE_NUMBER}")
    print("=" * 60)
    print(f"  Rôle              : {DEFAULT_ROLE}  (strategy={STRATEGY})")
    print(f"  Départ offset     : T+{RACE_START_OFFSET_S:.0f}s  (TDMA slot {MY_TDMA_OFFSET_MS}ms)")
    print(f"  MAVLink           : {MAVLINK_PORT} @ {MAVLINK_BAUD}")
    print(f"  LoRa              : {LORA_PORT}")
    print(f"  Capture radius    : RTK {CAPTURE_RADIUS_RTK}m / GPS {CAPTURE_RADIUS_GPS}m")
    print(f"  Bouées du parcours: {buoys_used_in_course(COURSE_NUMBER)}")
    print(f"  Étapes parcours   : {len(COURSE_LEGS)}")
    print(f"  Pénalité          : {len(PENALTY_LEGS)} legs (P1/P2 — manuelle)")
    if is_essai():
        print(f"  Vent ESSAI        : {WIND_ESSAI_DIR_DEG:.0f}° / {WIND_ESSAI_SPEED_MS:.1f} m/s")
    else:
        print(f"  Vent fallback     : {WIND_FALLBACK_DIR_DEG:.0f}° / {WIND_FALLBACK_SPEED_MS:.1f} m/s (orga muette > {WIND_FALLBACK_TIMEOUT_S:.0f}s)")
    print("=" * 60)
