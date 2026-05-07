"""
Layline et décision de virement (tack / gybe).

LAYLINE : ligne droite depuis la bouée cible, orientée à ±θ_VMG par rapport
au vent. Dès que le drone CROISE la layline du bord opposé, il doit virer
immédiatement (sinon il dépasse la bouée).

Toutes les positions sont en GPS (lat, lon). Les calculs vectoriels (produit
scalaire pour la distance signée) se font dans un repère (east, north) en
mètres relatif à la bouée cible, via `offset_meters`.

Le module retourne 3 booléens fondamentaux :
    - layline_reached : on a atteint la layline du bord actuel
    - should_tack    : on a intérêt à virer maintenant
    - in_dead_zone   : on est dans le secteur impossible (vent < ±θ_min)
"""

import math
from dataclasses import dataclass
from typing import Tuple

from . import polar
from . import geo_utils
import config

GPSPos = Tuple[float, float]


@dataclass
class LaylineState:
    layline_reached: bool
    should_tack: bool
    in_dead_zone: bool
    distance_to_layline_m: float    # distance signée — négatif si layline dépassée
    on_starboard_tack: bool         # bord actuel d'après le cap
    laylines: Tuple[GPSPos, GPSPos]
    # Coordonnées GPS des points "loin" sur les laylines tribord et bâbord


def _tack_from_heading(heading_deg: float, true_wind_dir_deg: float) -> bool:
    """Détermine le bord (amure) à partir du cap.

    Tribord amures : le vent vient du côté tribord (droite) du bateau.
    """
    # Différence vent → cap dans [-180, +180]
    diff = geo_utils.angle_diff_deg(true_wind_dir_deg, heading_deg)
    return diff > 0   # vent à droite = tribord amures


def signed_distance_to_layline(
    boat_pos: GPSPos,
    target_buoy_pos: GPSPos,
    true_wind_dir_deg: float,
    vmg_angle_deg: float,
    starboard: bool,
) -> float:
    """Distance signée à la layline du bord donné, en mètres.

    Convention : positif = on n'a PAS encore atteint la layline,
                 négatif = on l'a dépassée (il fallait avoir viré déjà).
    """
    # Direction de la layline depuis la bouée :
    #   tribord amures → bouée à gauche du bateau → on l'aborde par le secteur
    #   où le vent vient de tribord. Donc cap d'arrivée à la bouée =
    #   wind_dir + vmg_angle. La layline est la droite issue de la bouée
    #   dirigée à l'opposé (vers bateau) avec ce même cap.
    if starboard:
        cap_arrivee = (true_wind_dir_deg + vmg_angle_deg) % 360.0
    else:
        cap_arrivee = (true_wind_dir_deg - vmg_angle_deg) % 360.0

    # Vecteur unitaire DEPUIS la bouée VERS le bateau (sens inverse cap arrivée)
    cap_depuis_bouee = (cap_arrivee + 180.0) % 360.0
    layline_dir = geo_utils.cap_to_unit_vector(cap_depuis_bouee)
    # Vecteur normal à la layline (à gauche de layline_dir = +90°)
    normal = (-layline_dir[1], layline_dir[0])

    # Vecteur bouée → bateau, en mètres E/N
    rel = geo_utils.offset_meters(target_buoy_pos, boat_pos)
    return geo_utils.dot_2d(rel, normal)


def evaluate(
    boat_pos: GPSPos,
    boat_heading_deg: float,
    target_buoy_pos: GPSPos,
    true_wind_dir_deg: float,
    true_wind_speed_ms: float,
) -> LaylineState:
    """Evalue la situation par rapport aux laylines de la bouée cible."""

    # Angle waypoint vs vent
    bearing_wp = geo_utils.bearing_deg(boat_pos, target_buoy_pos)
    diff_wp = abs(geo_utils.angle_diff_deg(bearing_wp, true_wind_dir_deg))

    in_dead = diff_wp < config.POLAR_THETA_MIN_DEG

    if diff_wp >= 50.0:
        # Pas en remontée → pas besoin de layline
        return LaylineState(
            layline_reached=False,
            should_tack=False,
            in_dead_zone=False,
            distance_to_layline_m=0.0,
            on_starboard_tack=_tack_from_heading(boat_heading_deg, true_wind_dir_deg),
            laylines=(target_buoy_pos, target_buoy_pos),
        )

    vmg_angle = polar.optimal_upwind_angle(true_wind_speed_ms)

    # Bord actuel
    on_starboard = _tack_from_heading(boat_heading_deg, true_wind_dir_deg)

    # Distance signée aux deux laylines
    d_starboard = signed_distance_to_layline(
        boat_pos, target_buoy_pos, true_wind_dir_deg, vmg_angle, starboard=True)
    d_port = signed_distance_to_layline(
        boat_pos, target_buoy_pos, true_wind_dir_deg, vmg_angle, starboard=False)

    # Layline atteinte du bord opposé → on doit virer
    if on_starboard:
        d_current = d_starboard
        d_opposite = d_port
    else:
        d_current = d_port
        d_opposite = d_starboard

    # On a "atteint" la layline opposée quand sa distance signée passe sous 0
    layline_reached = d_opposite <= 1.0   # marge de 1m (anticiper un peu)
    should_tack = layline_reached

    # Définir des points "lointains" sur les laylines pour visu/debug (en GPS)
    L = 200.0  # mètres
    lay_starboard_dir = (true_wind_dir_deg + vmg_angle + 180.0) % 360.0
    lay_port_dir = (true_wind_dir_deg - vmg_angle + 180.0) % 360.0
    p_s = geo_utils.destination_point(target_buoy_pos, lay_starboard_dir, L)
    p_p = geo_utils.destination_point(target_buoy_pos, lay_port_dir, L)

    return LaylineState(
        layline_reached=layline_reached,
        should_tack=should_tack,
        in_dead_zone=in_dead,
        distance_to_layline_m=d_current,
        on_starboard_tack=on_starboard,
        laylines=(p_s, p_p),
    )
