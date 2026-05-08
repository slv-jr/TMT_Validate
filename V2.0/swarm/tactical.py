"""
Indicateurs tactiques dérivés des positions LoRa reçues.

Le canal BATTLEBOATS expose la position de TOUS les bateaux (UTT + 9
adversaires) toutes les 60 s (cf. PDF protocole §3.3). On exploite cette
information pour produire des métriques stratégiques utilisées par la
boucle principale et la sélection de cap :

    - enemies_ahead    : ennemis devant nous sur la trajectoire de course
    - enemies_close    : ennemis à moins de TACTICAL_RADIUS_M (anti-collision
                          fine, vient en complément du potential_field)
    - team_progress    : progression estimée des coéquipiers UTT (utile à
                          l'Optimizer pour caler son cap sur le Scout)
    - blocked_target   : True si la prochaine bouée est "encombrée" (≥2
                          ennemis dans un rayon de TACTICAL_RADIUS_M)

Pas de décision active ici : ce module retourne des indicateurs qu'on
log et qu'on remonte dans le CSV. La modulation de stratégie se fait dans
roles.role_modifies_strategy() en tenant compte de ces métriques.
"""

import logging
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import config
from navigation import geo_utils

log = logging.getLogger(__name__)


TACTICAL_RADIUS_M: float = 30.0   # rayon "zone proche" autour de nous / d'une bouée
AHEAD_HEADING_TOL_DEG: float = 60.0   # cône avant pour "devant"


@dataclass
class TacticalSnapshot:
    enemies_total: int = 0
    enemies_ahead: int = 0
    enemies_close: int = 0
    team_active: int = 0
    blocked_target: bool = False
    closest_enemy_id: str = ""
    closest_enemy_d_m: float = 1e9
    enemy_positions: List[Tuple[str, float, float]] = field(default_factory=list)


def _bearing_deg(from_pos: Tuple[float, float], to_pos: Tuple[float, float]) -> float:
    de, dn = geo_utils.offset_meters(from_pos, to_pos)
    return (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0


def _angle_diff_abs(a: float, b: float) -> float:
    d = abs((a - b + 180.0) % 360.0 - 180.0)
    return d


def compute(boat_pos: Tuple[float, float],
            heading_deg: float,
            target_pos: Tuple[float, float],
            team_neighbors: Dict,
            enemy_neighbors: Dict) -> TacticalSnapshot:
    """Calcule la photo tactique courante.

    Args:
        boat_pos        : (lat, lon) du drone local
        heading_deg     : cap actuel (degrés vrais)
        target_pos      : prochaine bouée/porte ciblée
        team_neighbors  : dict {boat_id: NeighborState} alliés
        enemy_neighbors : dict {boat_id: NeighborState} ennemis
    """
    snap = TacticalSnapshot()
    snap.enemies_total = len(enemy_neighbors)
    snap.team_active = len(team_neighbors)

    if not enemy_neighbors:
        return snap

    target_brg = _bearing_deg(boat_pos, target_pos) if target_pos else heading_deg

    enemies_in_target_zone = 0
    for bid, ns in enemy_neighbors.items():
        if not ns.has_fix:
            continue
        epos = (ns.lat, ns.lon)
        snap.enemy_positions.append((bid, ns.lat, ns.lon))

        d_self = geo_utils.distance_m(boat_pos, epos)
        if d_self < snap.closest_enemy_d_m:
            snap.closest_enemy_d_m = d_self
            snap.closest_enemy_id = bid

        if d_self < TACTICAL_RADIUS_M:
            snap.enemies_close += 1

        # "Devant" = dans le cône orienté vers le target (et plus proche du
        # target que nous). On compare le bearing self→ennemi avec le bearing
        # self→target.
        brg_to_enemy = _bearing_deg(boat_pos, epos)
        if _angle_diff_abs(brg_to_enemy, target_brg) < AHEAD_HEADING_TOL_DEG:
            d_enemy_to_target = geo_utils.distance_m(epos, target_pos) if target_pos else 0.0
            d_self_to_target = geo_utils.distance_m(boat_pos, target_pos) if target_pos else 1.0
            if d_enemy_to_target < d_self_to_target:
                snap.enemies_ahead += 1

        # Encombrement de la cible : combien d'ennemis dans le rayon tactique
        # autour de la bouée visée.
        if target_pos and geo_utils.distance_m(epos, target_pos) < TACTICAL_RADIUS_M:
            enemies_in_target_zone += 1

    snap.blocked_target = enemies_in_target_zone >= 2
    return snap


def log_snapshot(snap: TacticalSnapshot):
    """Log INFO d'un snapshot tactique (cadencer côté caller, ~1×/5s)."""
    if snap.enemies_total == 0 and snap.team_active == 0:
        return
    log.info(
        "[TACT] team=%d ennemis=%d (proches=%d, devant=%d) "
        "closest=%s@%.0fm cible_bloquée=%s",
        snap.team_active, snap.enemies_total,
        snap.enemies_close, snap.enemies_ahead,
        snap.closest_enemy_id or "-",
        snap.closest_enemy_d_m if snap.closest_enemy_d_m < 1e8 else 0.0,
        snap.blocked_target,
    )
