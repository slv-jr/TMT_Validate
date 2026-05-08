"""
Utilitaires géographiques.

Le RUNTIME (navigation, anti-collision, layline, VMG, simulateur) travaille
exclusivement en coordonnées GPS (lat, lon). Cela évite la dépendance à un
repère local fragile (origine GPS qui pourrait changer entre tests, dérive
si les bouées ne sont pas centrées sur l'origine, etc.).

Les fonctions historiques `gps_to_local` / `local_to_gps` / `buoy_local` /
`gate_endpoints` sont conservées pour les outils de visualisation (plot
matplotlib en mètres, replay_log, etc.) — elles ne sont plus appelées dans
la boucle de navigation.

CONVENTION GPS : `pos = (lat_deg, lon_deg)` en degrés décimaux.

Précision : pour des distances < 5 km à la latitude de Toulon, l'approximation
équirectangulaire (cos(lat₀) constant) introduit une erreur < 0.1 m. La
formule haversine est utilisée pour `distance_m` afin d'être exacte aux
basses distances.
"""

import math
from typing import Tuple

import config


# Rayon terrestre moyen
EARTH_RADIUS_M = 6_371_000.0

# Position GPS = (lat_deg, lon_deg) — alias type pour la lisibilité
GPSPos = Tuple[float, float]
# Position locale = (east_m, north_m) — utilisée uniquement par les outils
LocalPos = Tuple[float, float]


def deg2rad(d: float) -> float:
    return d * math.pi / 180.0


def rad2deg(r: float) -> float:
    return r * 180.0 / math.pi


# ════════════════════════════════════════════════════════════════════════
# API GPS NATIVE — utilisée par tout le runtime
# ════════════════════════════════════════════════════════════════════════

def distance_m(p1: GPSPos, p2: GPSPos) -> float:
    """Distance entre deux points GPS en mètres (formule haversine).

    Args:
        p1, p2: tuples (lat_deg, lon_deg).
    """
    lat1 = deg2rad(p1[0])
    lat2 = deg2rad(p2[0])
    dlat = lat2 - lat1
    dlon = deg2rad(p2[1] - p1[1])
    a = (math.sin(dlat / 2.0) ** 2
         + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2)
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(p_from: GPSPos, p_to: GPSPos) -> float:
    """Cap (bearing) initial en degrés depuis p_from vers p_to.

    Convention NAVIGATION : 0° = nord, 90° = est, 180° = sud, 270° = ouest.
    """
    lat1 = deg2rad(p_from[0])
    lat2 = deg2rad(p_to[0])
    dlon = deg2rad(p_to[1] - p_from[1])
    y = math.sin(dlon) * math.cos(lat2)
    x = (math.cos(lat1) * math.sin(lat2)
         - math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    angle = math.degrees(math.atan2(y, x))
    return (angle + 360.0) % 360.0


def offset_meters(p_from: GPSPos, p_to: GPSPos) -> Tuple[float, float]:
    """Décalage (Δeast_m, Δnord_m) entre deux points GPS proches.

    Approximation équirectangulaire : utilise cos(lat_from) comme facteur
    constant. Précision suffisante pour des distances < 5 km. Les vecteurs
    obtenus s'utilisent comme un repère 2D métrique standard (east = x,
    north = y), ce qui simplifie les produits scalaires/vectoriels du
    champ de potentiel et de la layline.
    """
    lat0 = deg2rad(p_from[0])
    dlat = deg2rad(p_to[0] - p_from[0])
    dlon = deg2rad(p_to[1] - p_from[1])
    east = dlon * math.cos(lat0) * EARTH_RADIUS_M
    north = dlat * EARTH_RADIUS_M
    return east, north


def move_meters(p_gps: GPSPos, east_m: float, north_m: float) -> GPSPos:
    """Avance d'un point GPS de (east_m, north_m) mètres et renvoie le nouveau point.

    Inverse de offset_meters dans la même approximation.
    """
    lat0 = deg2rad(p_gps[0])
    dlat = north_m / EARTH_RADIUS_M
    dlon = east_m / (math.cos(lat0) * EARTH_RADIUS_M)
    return (p_gps[0] + rad2deg(dlat), p_gps[1] + rad2deg(dlon))


def destination_point(p_gps: GPSPos, bearing_deg_: float, distance_m_: float) -> GPSPos:
    """Position d'un point situé à `distance_m_` mètres de `p_gps` selon le cap.

    Implémentation sphérique exacte (formule de Veness). Utile pour générer
    des laylines ou des waypoints d'approche à partir d'un cap et d'une
    distance.
    """
    lat1 = deg2rad(p_gps[0])
    lon1 = deg2rad(p_gps[1])
    brg = deg2rad(bearing_deg_)
    ang = distance_m_ / EARTH_RADIUS_M
    lat2 = math.asin(
        math.sin(lat1) * math.cos(ang)
        + math.cos(lat1) * math.sin(ang) * math.cos(brg)
    )
    lon2 = lon1 + math.atan2(
        math.sin(brg) * math.sin(ang) * math.cos(lat1),
        math.cos(ang) - math.sin(lat1) * math.sin(lat2),
    )
    return (rad2deg(lat2), (rad2deg(lon2) + 540.0) % 360.0 - 180.0)


def angle_diff_deg(target: float, current: float) -> float:
    """Différence d'angle signée dans [-180, +180].

    Positif si target est à droite (sens horaire) de current.
    """
    d = (target - current + 540.0) % 360.0 - 180.0
    return d


def cap_to_unit_vector(cap_deg: float) -> Tuple[float, float]:
    """Convertit un cap NAVIGATION en vecteur unitaire (east, north)."""
    rad = deg2rad(cap_deg)
    return math.sin(rad), math.cos(rad)


def cross_2d(v1: Tuple[float, float], v2: Tuple[float, float]) -> float:
    """Produit vectoriel 2D (composante z), repère (east, north)."""
    return v1[0] * v2[1] - v1[1] * v2[0]


def dot_2d(v1: Tuple[float, float], v2: Tuple[float, float]) -> float:
    return v1[0] * v2[0] + v1[1] * v2[1]


def buoy_gps(buoy_name: str) -> GPSPos:
    """Position GPS d'une bouée par son nom (ex: 'C', 'D', 'AB', '12', '34').

    Pour un nom à 2 caractères composé de bouées valides (ex: 'AB', '12', '34'),
    retourne le milieu géométrique de la porte. Sinon retourne la bouée
    individuelle.
    """
    # Cas porte : 2 caractères, chacun est une bouée valide
    if len(buoy_name) == 2:
        a_name, b_name = buoy_name[0], buoy_name[1]
        if a_name in config.BUOYS_GPS and b_name in config.BUOYS_GPS:
            a = config.BUOYS_GPS[a_name]
            b = config.BUOYS_GPS[b_name]
            return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    return config.BUOYS_GPS[buoy_name]


def gate_endpoints_gps(gate_name: str = "AB") -> Tuple[GPSPos, GPSPos]:
    """Coordonnées GPS des deux bouées d'une porte (ex: 'AB', '12', '34').

    Args:
        gate_name: nom de la porte (2 caractères, chacun une bouée valide).
                   Défaut "AB" (porte départ/arrivée du parcours côtier).
    """
    a_name, b_name = gate_name[0], gate_name[1]
    return config.BUOYS_GPS[a_name], config.BUOYS_GPS[b_name]


def line_crossed(p_prev: GPSPos, p_curr: GPSPos,
                 line_a: GPSPos, line_b: GPSPos) -> bool:
    """Vrai si le segment [p_prev → p_curr] croise le segment [line_a → line_b].

    Test géométrique pur basé sur les SIGNES des produits vectoriels — il est
    invariant sous transformation affine, donc fonctionne aussi bien en GPS
    qu'en local tant que les 4 points sont dans le même système.
    Pour la précision numérique aux petits angles, on calcule en mètres
    relatifs à `line_a`.
    """
    prev = offset_meters(line_a, p_prev)
    curr = offset_meters(line_a, p_curr)
    b = offset_meters(line_a, line_b)
    a = (0.0, 0.0)
    d1 = cross_2d(
        (b[0] - a[0], b[1] - a[1]),
        (prev[0] - a[0], prev[1] - a[1]),
    )
    d2 = cross_2d(
        (b[0] - a[0], b[1] - a[1]),
        (curr[0] - a[0], curr[1] - a[1]),
    )
    d3 = cross_2d(
        (curr[0] - prev[0], curr[1] - prev[1]),
        (a[0] - prev[0], a[1] - prev[1]),
    )
    d4 = cross_2d(
        (curr[0] - prev[0], curr[1] - prev[1]),
        (b[0] - prev[0], b[1] - prev[1]),
    )
    return (d1 * d2 < 0) and (d3 * d4 < 0)


# ════════════════════════════════════════════════════════════════════════
# API LOCALE — UNIQUEMENT POUR LES OUTILS DE VISUALISATION
# (plot matplotlib, replay, simulateur, debug). Pas utilisée par le runtime.
# ════════════════════════════════════════════════════════════════════════

def gps_to_local(lat: float, lon: float) -> LocalPos:
    """Conversion GPS → (east, north) mètres depuis config.ORIGIN_LAT/LON.

    ⚠️ Outil de visualisation seulement. La navigation runtime utilise
    directement les coordonnées GPS (cf. distance_m / bearing_deg / etc.).
    """
    return offset_meters((config.ORIGIN_LAT, config.ORIGIN_LON), (lat, lon))


def local_to_gps(east: float, north: float) -> GPSPos:
    """Inverse de gps_to_local. Retourne (lat, lon) en degrés.

    ⚠️ Outil de visualisation seulement.
    """
    return move_meters((config.ORIGIN_LAT, config.ORIGIN_LON), east, north)


def buoy_local(buoy_name: str) -> LocalPos:
    """Position locale (east, north) d'une bouée par son nom.

    Pour les portes à 2 caractères (ex: 'AB', '12', '34'), retourne le
    milieu de la porte.

    ⚠️ Outil de visualisation seulement (replay_log, simulator plot).
    """
    if len(buoy_name) == 2:
        a_name, b_name = buoy_name[0], buoy_name[1]
        if a_name in config.BUOYS_GPS and b_name in config.BUOYS_GPS:
            a = buoy_local(a_name)
            b = buoy_local(b_name)
            return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
    return gps_to_local(*config.BUOYS_GPS[buoy_name])


def gate_endpoints(gate_name: str = "AB") -> Tuple[LocalPos, LocalPos]:
    """Coordonnées locales des deux bouées d'une porte (ex: 'AB', '12', '34').

    ⚠️ Outil de visualisation seulement.
    """
    return buoy_local(gate_name[0]), buoy_local(gate_name[1])
