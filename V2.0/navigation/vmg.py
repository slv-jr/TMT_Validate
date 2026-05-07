"""
VMG — Velocity Made Good.

Le VMG est la composante de vitesse du bateau dans la direction du
waypoint cible. C'est la métrique d'optimisation de la stratégie.

    VMG = V_bateau · cos(angle_entre_cap_et_waypoint)

Le module distingue 3 régimes :
    - REMONTÉE   : waypoint dans le secteur de vent (TWA_to_wp < ±50°)
    - REACHING   : waypoint sur les côtés (TWA_to_wp ~90°)
    - PORTANT    : waypoint sous le vent (TWA_to_wp > ±130°)
"""

import math
from dataclasses import dataclass
from typing import Tuple

from . import polar
from . import geo_utils
import config


@dataclass
class VMGAdvice:
    """Recommandation de cap basée sur le VMG."""
    target_heading_deg: float       # cap à tenir (degrés navigation, 0=N)
    expected_speed_ms: float        # vitesse prévue à ce cap
    expected_vmg_ms: float          # VMG prévu vers le waypoint
    regime: str                     # "UPWIND" | "REACHING" | "DOWNWIND"
    on_starboard_tack: bool         # True = bâbord amures (vent à gauche)
    needs_tacking: bool             # True si on est en zone morte
    tack_options: Tuple[float, float]  # (cap_tribord, cap_babord) en upwind


def compute_optimal_heading(
    boat_pos: Tuple[float, float],
    waypoint: Tuple[float, float],
    true_wind_dir_deg: float,
    true_wind_speed_ms: float,
    current_heading_deg: float,
) -> VMGAdvice:
    """Calcule le cap qui maximise le VMG vers le waypoint.

    Args:
        boat_pos      : (lat, lon) en degrés décimaux
        waypoint      : (lat, lon) en degrés décimaux — bouée cible
        true_wind_dir_deg : direction d'OÙ vient le vent (convention météo,
                            0=N, 90=E). Si le vent vient d'EST, dir=90°.
        true_wind_speed_ms : vitesse du vent réel en m/s
        current_heading_deg : cap actuel du bateau (pour décider du bord)

    Returns:
        VMGAdvice avec le cap recommandé et la stratégie.
    """
    # Cap géodésique vers le waypoint
    bearing_wp = geo_utils.bearing_deg(boat_pos, waypoint)

    # Angle relatif waypoint vs vent (TWA_wp = vent → waypoint)
    # Si le vent vient de la même direction que le waypoint → on doit louvoyer
    wind_to_wp_diff = geo_utils.angle_diff_deg(bearing_wp, true_wind_dir_deg)
    abs_diff = abs(wind_to_wp_diff)

    # ───── REMONTÉE AU VENT ─────
    if abs_diff < 50.0:   # waypoint à moins de 50° du vent → louvoyage
        opt_angle = polar.optimal_upwind_angle(true_wind_speed_ms)
        # Deux caps possibles : tribord amures ou bâbord amures
        cap_tribord = (true_wind_dir_deg + opt_angle) % 360.0
        cap_babord  = (true_wind_dir_deg - opt_angle) % 360.0

        # Choix : celui qui a la composante VMG la plus favorable vers WP
        # (ou qui est le plus proche du cap actuel pour éviter un tack inutile)
        diff_trib = abs(geo_utils.angle_diff_deg(cap_tribord, current_heading_deg))
        diff_bab  = abs(geo_utils.angle_diff_deg(cap_babord,  current_heading_deg))
        if diff_trib < diff_bab:
            target = cap_tribord
            on_starboard = True   # vent vient de tribord (côté droit)
        else:
            target = cap_babord
            on_starboard = False

        v = polar.boat_speed_predicted(opt_angle, true_wind_speed_ms)
        vmg = polar.vmg_upwind(opt_angle, true_wind_speed_ms)
        return VMGAdvice(
            target_heading_deg=target,
            expected_speed_ms=v,
            expected_vmg_ms=vmg,
            regime="UPWIND",
            on_starboard_tack=on_starboard,
            needs_tacking=True,
            tack_options=(cap_tribord, cap_babord),
        )

    # ───── PORTANT (waypoint sous le vent) ─────
    if abs_diff > 130.0:
        # Vent arrière : on peut souvent aller direct au waypoint
        # mais sur grand largue, le VMG est meilleur en zigzaguant un peu
        opt_angle = polar.optimal_downwind_angle(true_wind_speed_ms)
        # Pour le portant on calcule par rapport au vent inverse (180°-θ)
        cap_tribord = (true_wind_dir_deg + 180.0 - (180.0 - opt_angle)) % 360.0
        cap_babord  = (true_wind_dir_deg + 180.0 + (180.0 - opt_angle)) % 360.0

        # Si l'écart au waypoint est grand, on choisit la gybe la plus proche
        diff_trib = abs(geo_utils.angle_diff_deg(cap_tribord, bearing_wp))
        diff_bab  = abs(geo_utils.angle_diff_deg(cap_babord,  bearing_wp))
        # Si même cap direct est rentable (vent arrière franc), on le prend
        direct_speed = polar.boat_speed_predicted(
            abs(geo_utils.angle_diff_deg(bearing_wp, true_wind_dir_deg)),
            true_wind_speed_ms,
        )
        # Préférer le cap direct si le bateau peut le tenir efficacement
        if abs_diff > 160.0 and direct_speed > 0.5 * polar.boat_speed_predicted(opt_angle, true_wind_speed_ms):
            target = bearing_wp
            on_starboard = (geo_utils.angle_diff_deg(true_wind_dir_deg, bearing_wp) > 0)
        else:
            if diff_trib < diff_bab:
                target = cap_tribord
                on_starboard = True
            else:
                target = cap_babord
                on_starboard = False
        v = polar.boat_speed_predicted(opt_angle, true_wind_speed_ms)
        return VMGAdvice(
            target_heading_deg=target,
            expected_speed_ms=v,
            expected_vmg_ms=v,   # approximation
            regime="DOWNWIND",
            on_starboard_tack=on_starboard,
            needs_tacking=False,
            tack_options=(cap_tribord, cap_babord),
        )

    # ───── REACHING (waypoint sur les côtés, allure rapide) ─────
    target = bearing_wp
    twa_at_target = abs(geo_utils.angle_diff_deg(target, true_wind_dir_deg))
    v = polar.boat_speed_predicted(twa_at_target, true_wind_speed_ms)
    on_starboard = (geo_utils.angle_diff_deg(true_wind_dir_deg, bearing_wp) > 0)
    return VMGAdvice(
        target_heading_deg=target,
        expected_speed_ms=v,
        expected_vmg_ms=v,   # cap direct → VMG = vitesse
        regime="REACHING",
        on_starboard_tack=on_starboard,
        needs_tacking=False,
        tack_options=(target, target),
    )


def true_to_apparent_wind(
    true_wind_dir_deg: float,
    true_wind_speed_ms: float,
    boat_heading_deg: float,
    boat_speed_ms: float,
) -> Tuple[float, float]:
    """Conversion vent réel → vent apparent (référentiel bateau).

    Le vent apparent est la composition vectorielle du vent réel et du
    vent dû au mouvement du bateau (qui souffle en sens inverse de sa marche).

    Returns:
        (awa_deg, awa_speed_ms) : awa_deg dans [-180, 180] (signe = côté).
                                   awa_deg=0 → vent face, +90 → tribord.
    """
    # Vent réel exprimé en vecteur (vitesse à laquelle l'air se déplace,
    # pas la direction d'où il vient → on retourne)
    wind_to_dir_deg = (true_wind_dir_deg + 180.0) % 360.0
    we = true_wind_speed_ms * math.sin(math.radians(wind_to_dir_deg))
    wn = true_wind_speed_ms * math.cos(math.radians(wind_to_dir_deg))

    # Vent dû au mouvement bateau (sens inverse de sa course)
    boat_e = boat_speed_ms * math.sin(math.radians(boat_heading_deg))
    boat_n = boat_speed_ms * math.cos(math.radians(boat_heading_deg))

    # Vent apparent = vent réel - vent bateau (en référentiel sol)
    # mais ressenti à bord : vent_app = vent_air_sol - vent_bateau_sol
    # avec vent_air_sol = vecteur du déplacement de l'air
    app_e = we - boat_e
    app_n = wn - boat_n
    awa_speed = math.hypot(app_e, app_n)
    # Direction d'où vient le vent apparent (convention météo) :
    awa_from_dir = (math.degrees(math.atan2(-app_e, -app_n)) + 360.0) % 360.0
    # AWA dans le référentiel bateau (signé)
    awa_deg = geo_utils.angle_diff_deg(awa_from_dir, boat_heading_deg)
    return awa_deg, awa_speed


def apparent_to_true_wind(
    awa_deg: float,
    awa_speed_ms: float,
    boat_heading_deg: float,
    boat_speed_ms: float,
) -> Tuple[float, float]:
    """Inverse : vent apparent → vent réel.

    Utile en mode dégradé (perte du vent Calypso) si on dispose d'une
    girouette embarquée. Sans girouette, fallback "triangle des vents"
    via wind/wind_estimator.py.
    """
    # Direction d'OÙ vient le vent apparent dans le référentiel sol
    awa_from_sol = (boat_heading_deg + awa_deg) % 360.0
    # Composantes du vent apparent au sol
    app_e = awa_speed_ms * math.sin(math.radians((awa_from_sol + 180) % 360))
    app_n = awa_speed_ms * math.cos(math.radians((awa_from_sol + 180) % 360))
    # Composantes vent bateau
    boat_e = boat_speed_ms * math.sin(math.radians(boat_heading_deg))
    boat_n = boat_speed_ms * math.cos(math.radians(boat_heading_deg))
    # Vent réel
    we = app_e + boat_e
    wn = app_n + boat_n
    speed = math.hypot(we, wn)
    from_dir = (math.degrees(math.atan2(-we, -wn)) + 360.0) % 360.0
    return from_dir, speed
