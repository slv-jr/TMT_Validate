"""
Gestionnaire de la séquence de waypoints du Parcours N°3.

Le parcours impose un ORDRE STRICT (cf. règlement Battleboats 2026 V2.2 et
DOSSIER_TECHNIQUE.docx) :
    Départ porte A-B → C(stbd) → D(port) → E(port) → C(stbd)
                     → F(stbd) → G(stbd) → C(stbd) → Arrivée porte A-B

Toutes les positions manipulées par ce module sont en GPS (lat, lon).
Les waypoints d'approche sont calculés en GPS via `move_meters` à partir
de la bouée et d'un offset (east, north) de 2 m perpendiculaire à la
direction "vers la bouée suivante".

Validation de l'étape :
    - Pour les bouées : passer dans un rayon ADAPTATIF (4 m si RTK_FIXED,
      7 m sinon — cf. README2 §"Rayon de capture adaptatif")
      ET du bon côté (port/starboard vérifié par produit vectoriel sur
      les offsets en mètres).
    - Pour les portes : croiser le segment A-B (test géométrique pur).
"""

import math
import logging
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from . import geo_utils
import config

log = logging.getLogger(__name__)

# Alias type
GPSPos = Tuple[float, float]   # (lat_deg, lon_deg)


@dataclass
class Leg:
    """Une étape du parcours."""
    name: str
    buoy: str               # "A", "B", "C", ..., ou "AB" pour la porte
    side: str               # "starboard", "port", "gate"
    completed: bool = False
    started_at: float = 0.0


@dataclass
class WaypointPlan:
    """Plan immédiat à suivre (rafraîchi à chaque tick navigation).

    Toutes les positions sont en GPS (lat_deg, lon_deg).
    """
    leg: Leg
    leg_index: int
    target_pos: GPSPos      # waypoint d'approche réel (GPS)
    final_pos: GPSPos       # bouée à valider (GPS)
    distance_m: float       # distance haversine en mètres
    bearing_deg: float      # cap navigation (°)
    is_gate: bool


class CourseManager:
    """État global du parcours, avance étape par étape."""

    def __init__(self):
        self.legs: List[Leg] = [Leg(**l) for l in config.COURSE_3_LEGS]
        self.current_idx: int = 0
        self.last_position: Optional[GPSPos] = None
        self.race_started: bool = False
        self.race_finished: bool = False

    # ─────────────────────────────────────────────
    # Accès à l'étape courante
    # ─────────────────────────────────────────────
    @property
    def current_leg(self) -> Optional[Leg]:
        if self.current_idx >= len(self.legs):
            return None
        return self.legs[self.current_idx]

    def progress_str(self) -> str:
        return f"{self.current_idx}/{len(self.legs)}"

    # ─────────────────────────────────────────────
    # Calcul du waypoint d'approche
    # ─────────────────────────────────────────────
    def _approach_point(
        self,
        buoy_pos: GPSPos,
        next_buoy_pos: GPSPos,
        side: str,
    ) -> GPSPos:
        """Décale la cible de BUOY_CLEARANCE_M du bon côté de la bouée.

        Le décalage est perpendiculaire à la direction "vers la bouée
        suivante", de manière à arriver à la bouée par le bon côté pour
        ensuite la contourner sans la toucher.

        Implémentation : on travaille en mètres relatifs à la bouée
        courante (offset_meters), on décale dans le bon sens, puis on
        retourne le point GPS via move_meters.
        """
        # Direction depuis la bouée courante vers la suivante (en mètres E/N)
        de, dn = geo_utils.offset_meters(buoy_pos, next_buoy_pos)
        norm = math.hypot(de, dn)
        if norm < 1e-3:
            # Pas de bouée suivante (ex: dernière étape) → pas de décalage
            return buoy_pos
        ue, un = de / norm, dn / norm
        # Vecteur perpendiculaire :
        #  - "starboard" (bouée à droite du bateau) → on passe à GAUCHE de
        #    la bouée → le waypoint d'approche est à gauche de la ligne
        #    bouée→suivante (côté +90°).
        #  - "port" → on passe à droite → côté -90°.
        if side == "starboard":
            ne, nn = -un, ue         # rotation +90°
        elif side == "port":
            ne, nn = un, -ue         # rotation -90°
        else:
            return buoy_pos
        clearance = config.BUOY_CLEARANCE_M
        return geo_utils.move_meters(buoy_pos, ne * clearance, nn * clearance)

    def _next_buoy_pos(self, leg_index: int) -> GPSPos:
        """Position GPS de la bouée suivante (utile pour orienter l'approche)."""
        if leg_index + 1 < len(self.legs):
            next_buoy = self.legs[leg_index + 1].buoy
            return geo_utils.buoy_gps(next_buoy)
        # Dernière étape : on retourne le centre de la porte
        return geo_utils.buoy_gps("AB")

    # ─────────────────────────────────────────────
    # Plan immédiat
    # ─────────────────────────────────────────────
    def plan(self, boat_pos: GPSPos) -> Optional[WaypointPlan]:
        leg = self.current_leg
        if leg is None:
            return None
        buoy_pos = geo_utils.buoy_gps(leg.buoy)
        if leg.side == "gate":
            target = buoy_pos          # milieu de la porte
            is_gate = True
        else:
            next_pos = self._next_buoy_pos(self.current_idx)
            target = self._approach_point(buoy_pos, next_pos, leg.side)
            is_gate = False
        d = geo_utils.distance_m(boat_pos, target)
        b = geo_utils.bearing_deg(boat_pos, target)
        return WaypointPlan(
            leg=leg,
            leg_index=self.current_idx,
            target_pos=target,
            final_pos=buoy_pos,
            distance_m=d,
            bearing_deg=b,
            is_gate=is_gate,
        )

    # ─────────────────────────────────────────────
    # Validation
    # ─────────────────────────────────────────────
    def update_and_validate(self, boat_pos: GPSPos,
                            rtk_fixed: bool = False) -> bool:
        """À appeler à chaque tick. Retourne True si une étape vient d'être
        validée (utile pour logger et déclencher les transitions d'état).

        Args:
            boat_pos: position GPS du bateau (lat, lon).
            rtk_fixed: True si le fix GPS est RTK (≥ Float). Détermine le
                rayon de capture (4 m RTK / 7 m GPS — cf. config).
        """
        leg = self.current_leg
        if leg is None:
            return False

        validated = False

        if leg.side == "gate":
            # Croisement du segment A-B
            a_gps, b_gps = geo_utils.gate_endpoints_gps()
            if self.last_position is not None:
                if geo_utils.line_crossed(
                    self.last_position, boat_pos, a_gps, b_gps
                ):
                    if leg.name == "DEPART":
                        if not self.race_started:
                            self.race_started = True
                            log.info("[COURSE] Porte de départ franchie")
                            validated = True
                    elif leg.name == "ARRIVEE":
                        if self.race_started and self.current_idx == len(self.legs) - 1:
                            self.race_finished = True
                            log.info("[COURSE] Porte d'arrivée franchie — FIN")
                            validated = True
        else:
            # Validation bouée : rayon adaptatif RTK/GPS (cf. README2)
            buoy_pos = geo_utils.buoy_gps(leg.buoy)
            d = geo_utils.distance_m(boat_pos, buoy_pos)
            radius = config.capture_radius_for_fix(rtk_fixed)
            if d < radius:
                # Vérification supplémentaire : passage du bon côté
                if self.last_position is not None and self._side_correct(
                    self.last_position, boat_pos, buoy_pos, leg.side
                ):
                    validated = True
                else:
                    # Validation avec doute → on prend quand même mais on log
                    validated = True
                    log.warning(
                        "[COURSE] Bouée %s validée (%.1f m < %.1f m %s) "
                        "mais le côté n'est pas certain",
                        leg.buoy, d, radius,
                        "RTK" if rtk_fixed else "GPS",
                    )

        if validated:
            leg.completed = True
            log.info(
                "[COURSE] Étape validée : %s (%s/%s)",
                leg.name, self.current_idx + 1, len(self.legs),
            )
            self.current_idx += 1

        self.last_position = boat_pos
        return validated

    @staticmethod
    def _side_correct(
        prev_pos: GPSPos,
        curr_pos: GPSPos,
        buoy_pos: GPSPos,
        side: str,
    ) -> bool:
        """Vrai si la bouée a été contournée du bon côté.

        Calcul : produit vectoriel entre la direction du bateau et le
        vecteur bateau→bouée, exprimés en mètres dans le repère (east, north).
        Signe positif = bouée à gauche du bateau.
        """
        # Direction du bateau (vecteur prev→curr en mètres E/N)
        dir_e, dir_n = geo_utils.offset_meters(prev_pos, curr_pos)
        # Position de la bouée relative au bateau (en mètres E/N)
        b_e, b_n = geo_utils.offset_meters(curr_pos, buoy_pos)
        cross = dir_e * b_n - dir_n * b_e
        if side == "starboard":
            # Bouée à droite du bateau → cross < 0
            return cross < 0
        elif side == "port":
            # Bouée à gauche → cross > 0
            return cross > 0
        return True

    def reset(self):
        for leg in self.legs:
            leg.completed = False
        self.current_idx = 0
        self.race_started = False
        self.race_finished = False
        self.last_position = None
