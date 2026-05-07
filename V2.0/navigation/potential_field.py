"""
Champ de potentiel pour anti-collision et évitement de bouées.

Combine :
    - Force ATTRACTIVE vers le waypoint courant (gain configurable, F = -k·∇U)
    - Force RÉPULSIVE des bouées (rayon BUOY_REPULSION_RADIUS_M)
    - Force RÉPULSIVE des autres drones connus (rayon DRONE_REPULSION_RADIUS_M)

Toutes les positions (`boat_pos`, `waypoint`, `Repulsor.pos`) sont en GPS
(lat, lon). Les forces sont calculées dans un repère (east, north) en
mètres relatif à la position du bateau (via `offset_meters`), ce qui rend
toute la physique standard 2D et indépendante de la latitude.

La force résultante (vecteur east/north) est convertie en cap recommandé.
On applique ensuite une PONDÉRATION avec le cap VMG : on ne dévie de la
trajectoire optimale que si une force répulsive significative l'exige.
"""

import math
from dataclasses import dataclass
from typing import Iterable, Tuple, List

from . import geo_utils
import config

GPSPos = Tuple[float, float]


@dataclass
class Repulsor:
    pos: GPSPos             # position GPS (lat, lon)
    radius_m: float
    gain: float
    label: str = ""


def attractive_force(
    boat_pos: GPSPos,
    waypoint: GPSPos,
    gain: float = 1.0,
) -> Tuple[float, float]:
    """Force attractive vers le waypoint (champ conique → constant en module).

    Retour : vecteur (east, north) sans dimension géométrique (déjà normé
    et multiplié par `gain`).
    """
    de, dn = geo_utils.offset_meters(boat_pos, waypoint)
    n = math.hypot(de, dn)
    if n < 1e-3:
        return (0.0, 0.0)
    return (gain * de / n, gain * dn / n)


def repulsive_force(
    boat_pos: GPSPos,
    repulsor: Repulsor,
) -> Tuple[float, float]:
    """Force répulsive (champ en 1/r², coupé au-delà du rayon).

    Forme classique de Khatib (1986) :
        F = gain · (1/d - 1/R) / d² · (boat - repulsor) / d
    """
    # Vecteur répulseur → bateau, en mètres E/N (la force pousse dans ce sens)
    de, dn = geo_utils.offset_meters(repulsor.pos, boat_pos)
    d = math.hypot(de, dn)
    if d > repulsor.radius_m or d < 1e-3:
        return (0.0, 0.0)
    coeff = repulsor.gain * (1.0 / d - 1.0 / repulsor.radius_m) / (d * d)
    return (coeff * de / d, coeff * dn / d)


def total_force(
    boat_pos: GPSPos,
    waypoint: GPSPos,
    repulsors: Iterable[Repulsor],
    attractive_gain: float = 1.0,
) -> Tuple[float, float]:
    """Force totale = attractive + somme des répulsions (vecteur east/north)."""
    fx, fy = attractive_force(boat_pos, waypoint, attractive_gain)
    for r in repulsors:
        rx, ry = repulsive_force(boat_pos, r)
        fx += rx
        fy += ry
    return (fx, fy)


def force_to_heading(force: Tuple[float, float]) -> float:
    """Convertit un vecteur (east, north) en cap navigation (degrés, 0=N)."""
    fx, fy = force
    if abs(fx) < 1e-6 and abs(fy) < 1e-6:
        return 0.0
    angle = math.degrees(math.atan2(fx, fy))
    return (angle + 360.0) % 360.0


def is_heading_in_dead_zone(heading_deg: float, wind_dir_deg: float,
                            dead_zone_half_deg: float = 35.0) -> bool:
    """Vrai si ce cap mettrait le bateau dans la zone morte du vent.

    Utilisé pour rejeter les caps PF qui paralyseraient le voilier.
    """
    diff = abs(((heading_deg - wind_dir_deg + 540.0) % 360.0) - 180.0)
    twa = 180.0 - diff
    return twa < dead_zone_half_deg


def safe_heading_against_wind(heading_deg: float, wind_dir_deg: float,
                              dead_zone_half_deg: float = 35.0) -> float:
    """Si heading_deg est en zone morte, renvoie le cap valide le plus proche.

    Utilisé en complément de force_to_heading pour s'assurer que le bateau
    peut effectivement avancer avec ce cap.
    """
    if not is_heading_in_dead_zone(heading_deg, wind_dir_deg, dead_zone_half_deg):
        return heading_deg
    # Trouver les deux limites de la zone morte
    limit_starboard = (wind_dir_deg + dead_zone_half_deg) % 360.0
    limit_port = (wind_dir_deg - dead_zone_half_deg) % 360.0
    # Choisir la limite la plus proche du cap demandé
    diff_s = abs(((heading_deg - limit_starboard + 540.0) % 360.0) - 180.0)
    diff_p = abs(((heading_deg - limit_port + 540.0) % 360.0) - 180.0)
    return limit_starboard if diff_s < diff_p else limit_port


def build_repulsors(
    other_drones_positions: List[Tuple[str, GPSPos]],
    nearby_buoys: List[str],
) -> List[Repulsor]:
    """Construit la liste des répulseurs actifs au tick courant.

    Args:
        other_drones_positions : liste de (id, (lat, lon)) — alliés ET
                                 ennemis, le champ ne fait pas la différence
                                 pour la sécurité de navigation.
        nearby_buoys : liste de noms de bouées présentes à proximité.
    """
    repulsors = []
    for boat_id, gps_pos in other_drones_positions:
        repulsors.append(Repulsor(
            pos=gps_pos,
            radius_m=config.DRONE_REPULSION_RADIUS_M,
            gain=config.DRONE_REPULSION_GAIN,
            label=f"drone:{boat_id}",
        ))
    for buoy_name in nearby_buoys:
        try:
            pos = geo_utils.buoy_gps(buoy_name)
        except KeyError:
            continue
        repulsors.append(Repulsor(
            pos=pos,
            radius_m=config.BUOY_REPULSION_RADIUS_M,
            gain=config.BUOY_REPULSION_GAIN,
            label=f"buoy:{buoy_name}",
        ))
    return repulsors


def emergency_avoidance_needed(
    boat_pos: GPSPos,
    repulsors: Iterable[Repulsor],
    threshold_m: float = 3.0,
) -> bool:
    """Retourne True si un répulseur est à moins de threshold_m
    → la boucle principale doit basculer en évitement d'urgence."""
    for r in repulsors:
        if geo_utils.distance_m(boat_pos, r.pos) < threshold_m:
            return True
    return False
