"""
Polaire de vitesse du JOYSWAY FOCUS V2.

Deux niveaux de polaire :
    1) Table par DÉFAUT codée en dur (cf. ci-dessous, _DEFAULT_POLAR_TABLE).
       Estimation initiale basée sur des polaires IOM/M-Class.
    2) Table CALIBRÉE sur l'eau via `calibration/polar_calibration.py`,
       enregistrée dans `config.POLAR_TABLE_PATH` (JSON). Si elle existe,
       elle est chargée au démarrage du module et écrase la table par
       défaut — d'où l'importance de faire la calibration le J-1.

La polaire représente le rapport V_bateau / V_vent en fonction de l'angle
au vent réel (TWA — True Wind Angle). Maximum en VITESSE BRUTE vers 110-130°,
mais maximum en VMG UPWIND vers 42-48°.

Sources :
    - README2 §"Polaire de vitesse — Calibration terrain (J-1)"
    - DOSSIER_TECHNIQUE.docx §V.1 (allures théoriques)
"""

import json
import logging
import math
import os

import config

log = logging.getLogger(__name__)


# ════════════════════════════════════════════════════════════════════════
# POLAIRE EMPIRIQUE — V_bateau / V_vent en fonction du TWA (degrés absolus)
# ════════════════════════════════════════════════════════════════════════
# Format : liste de (angle_deg, ratio V_boat/V_wind) interpolée linéairement.
# Valeurs cohérentes avec un Joysway Focus V2 par vent moyen 4-6 m/s.
_DEFAULT_POLAR_TABLE = [
    (0,   0.00),
    (30,  0.00),    # zone morte (sous θ_min)
    (35,  0.05),    # tout juste sortie de la zone morte
    (40,  0.30),
    (45,  0.42),    # près serré — bon VMG upwind
    (50,  0.52),
    (60,  0.62),
    (75,  0.72),
    (90,  0.80),    # travers — souvent vitesse max ratio
    (110, 0.82),    # largue — vitesse max
    (135, 0.78),    # grand largue
    (150, 0.65),
    (165, 0.50),    # vent arrière strict — sous-optimal
    (180, 0.45),
]


def _load_calibrated_table():
    """Charge la table calibrée si elle existe, sinon renvoie la table par défaut."""
    path = config.POLAR_TABLE_PATH
    if not os.path.exists(path):
        return list(_DEFAULT_POLAR_TABLE)
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        # Format attendu : {"table": [[angle, ratio], ...]}
        # OU directement [[angle, ratio], ...]
        raw = data["table"] if isinstance(data, dict) else data
        out = sorted([(float(a), float(r)) for a, r in raw], key=lambda x: x[0])
        if len(out) >= 4:
            log.info("[POLAR] Table calibrée chargée depuis %s (%d points)",
                     path, len(out))
            return out
    except Exception as e:
        log.warning("[POLAR] Impossible de charger %s : %s", path, e)
    return list(_DEFAULT_POLAR_TABLE)


_POLAR_TABLE = _load_calibrated_table()


def reload_table():
    """Force le rechargement de la table calibrée. Renvoie la nouvelle table."""
    global _POLAR_TABLE
    _POLAR_TABLE = _load_calibrated_table()
    return _POLAR_TABLE


def _ratio_at(theta_deg: float) -> float:
    """Interpolation linéaire dans la polaire."""
    a = abs(theta_deg) % 360.0
    if a > 180.0:
        a = 360.0 - a
    if a <= _POLAR_TABLE[0][0]:
        return _POLAR_TABLE[0][1]
    if a >= _POLAR_TABLE[-1][0]:
        return _POLAR_TABLE[-1][1]
    for i in range(len(_POLAR_TABLE) - 1):
        a0, r0 = _POLAR_TABLE[i]
        a1, r1 = _POLAR_TABLE[i + 1]
        if a0 <= a <= a1:
            t = (a - a0) / (a1 - a0)
            return r0 + t * (r1 - r0)
    return 0.0


def _g_wind(v_wind_ms: float) -> float:
    """Coefficient lié à la force du vent.

    Sous 1.5 m/s : trop peu pour avancer (vent mort).
    Au-delà de ~7 m/s : saturation hydrodynamique du Joysway.
    """
    if v_wind_ms < 1.5:
        return 0.0
    if v_wind_ms < 7.0:
        return 0.6 + 0.4 * (v_wind_ms - 1.5) / 5.5     # rampe 0.6→1.0
    return 1.0 + 0.05 * math.tanh((v_wind_ms - 7.0) / 3.0)


def boat_speed_predicted(theta_deg: float, v_wind_ms: float) -> float:
    """Vitesse prédite du bateau (m/s) à un angle de vent réel donné.

    Args:
        theta_deg: TWA en degrés (signe ignoré).
        v_wind_ms: vitesse du vent réel en m/s.
    Returns:
        Vitesse prédite en m/s. 0 si dans la zone morte.
    """
    return v_wind_ms * _ratio_at(theta_deg) * _g_wind(v_wind_ms)


def vmg_upwind(theta_deg: float, v_wind_ms: float) -> float:
    """VMG en remontée au vent (vitesse projetée sur l'axe du vent).

    VMG_upwind = V_bateau · cos(θ)
    Maximum atteint pour θ ≈ 42-48° sur le Joysway Focus V2.
    """
    v = boat_speed_predicted(theta_deg, v_wind_ms)
    return v * math.cos(math.radians(theta_deg))


def vmg_downwind(theta_deg: float, v_wind_ms: float) -> float:
    """VMG en descente (180° - θ par rapport au vent)."""
    v = boat_speed_predicted(theta_deg, v_wind_ms)
    # Au portant on veut MAXIMISER -cos(θ), donc on prend l'opposé
    return -v * math.cos(math.radians(theta_deg))


def optimal_upwind_angle(v_wind_ms: float) -> float:
    """Trouve numériquement l'angle de remontée qui maximise le VMG.

    Recherche brute entre θ_min et 80°.
    """
    best_theta = config.VMG_UPWIND_ANGLE_DEG
    best_vmg = -1.0
    theta = config.POLAR_THETA_MIN_DEG + 1.0
    while theta < 80.0:
        v = vmg_upwind(theta, v_wind_ms)
        if v > best_vmg:
            best_vmg = v
            best_theta = theta
        theta += 0.5
    return best_theta


def optimal_downwind_angle(v_wind_ms: float) -> float:
    """Angle optimal au portant (entre 130° et 180°).

    Au vent arrière strict (180°) la vitesse sature, donc on cherche
    typiquement un grand largue ~150-160° qui fait mieux en VMG.
    """
    best_theta = 160.0
    best_vmg = -1.0
    theta = 100.0
    while theta < 180.0:
        v = vmg_downwind(theta, v_wind_ms)
        if v > best_vmg:
            best_vmg = v
            best_theta = theta
        theta += 0.5
    return best_theta


def is_dead_zone(theta_deg: float) -> bool:
    """Vrai si l'angle de vent réel est dans la zone morte."""
    return abs(theta_deg) < config.POLAR_THETA_MIN_DEG


# ════════════════════════════════════════════════════════════════════════
# RÉGLAGE VOILE — table de lookup angle vent apparent → ouverture voile (%)
# ════════════════════════════════════════════════════════════════════════
def sail_trim_percent(awa_deg: float) -> float:
    """Position d'ouverture de la voile en % (0 = bordée, 100 = choquée).

    Règle "milieu de bord" : la voile fait approximativement la moitié de
    l'angle de vent apparent par rapport à l'axe du bateau.
    Avec quelques ajustements empiriques pour les transitions d'allure.

    Args:
        awa_deg: Apparent Wind Angle en degrés (0=face, 180=arrière).
    Returns:
        Ouverture en pourcentage [0, 100].
    """
    awa_abs = abs(awa_deg)
    if awa_abs < 25.0:
        return 0.0    # vent de face → voile complètement bordée
    if awa_abs < 45.0:
        return 5.0 + (awa_abs - 25.0) * 0.75   # près serré
    if awa_abs < 90.0:
        return 20.0 + (awa_abs - 45.0) * 0.66  # près à travers
    if awa_abs < 150.0:
        return 50.0 + (awa_abs - 90.0) * 0.66  # travers à largue
    return 90.0 + (awa_abs - 150.0) * 0.33     # grand largue / vent arrière
