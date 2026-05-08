"""
Gestionnaire de la séquence de waypoints — supporte les 2 parcours officiels.

Le parcours actif est déterminé par `config.COURSE_LEGS` (alias dynamique
selon `COURSE_NUMBER`) :

    Parcours 1 (banane) — STRATÉGIE SPIRALE ALTERNÉE :
        Porte 1-2 (départ) → Bouée HAUTE_T1 → Bouée BASSE_T1 → Bouée HAUTE_T2 → Porte 1-2 (arrivée)

        Au franchissement de la porte de DÉPART, on évalue le vent ET on
        FIGE pour tout le parcours le côté à enrouler en premier (T ou B) :

          - Si bouée 3 plus au vent → côté T : 3 → 1 → 4 → arrivée
          - Si bouée 4 plus au vent → côté B : 4 → 2 → 3 → arrivée

        La logique « spirale » impose :
          • Tour 1 montée : bouée du côté choisi (3 ou 4)
          • Tour 1 descente : bouée du MÊME côté en bas (1 ou 2)
          • Tour 2 montée : bouée du côté OPPOSÉ en haut (4 ou 3)
          • Arrivée : porte 1-2 (gate)

        Le choix est figé au franchissement du départ et NE CHANGE PLUS
        même si le vent évolue (cohérence de trajectoire).

    Parcours 2 ("5 bouées" — briefing officiel du 8/5/2026) :
        Départ porte 1-2 →
        Tour 1 : 5(port) → 4(port) → 3(port) → 1(port) →
        Tour 2 : 5(port) → 4(port) → 3(port) → 1(port) →
        Arrivée porte 1-2

        Tous les contournements en BÂBORD (port) par défaut. Modifier
        `_COURSE2_DEFAULT_SIDE` dans config.py si l'orga annonce TRIBORD.

Toutes les positions manipulées sont en GPS (lat, lon). Les waypoints
d'approche sont calculés via `move_meters` à partir d'un offset
perpendiculaire à la direction "vers la bouée suivante".

Validation de l'étape :
    - Pour les bouées : passer dans un rayon ADAPTATIF (4 m si RTK_FIXED,
      7 m sinon) ET du bon côté (port/starboard vérifié par produit
      vectoriel sur les offsets en mètres).
    - Pour les portes : croiser le segment des 2 bouées.
"""

import math
import logging
from dataclasses import dataclass
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
    buoy: str               # "A", "B", "C", ..., ou "AB"/"12"/"34" pour une porte
    side: str               # "starboard", "port", "gate"
    completed: bool = False
    started_at: float = 0.0


@dataclass
class WaypointPlan:
    """Plan immédiat à suivre (rafraîchi à chaque tick navigation)."""
    leg: Leg
    leg_index: int
    target_pos: GPSPos      # waypoint d'approche réel (GPS)
    final_pos: GPSPos       # bouée à valider (GPS)
    distance_m: float       # distance haversine en mètres
    bearing_deg: float      # cap navigation (°)
    is_gate: bool


class CourseManager:
    """État global du parcours, avance étape par étape."""

    # Mapping spirale parcours 1 :
    #   "T" : on commence par la bouée 3 (côté tribord du parcours)
    #   "B" : on commence par la bouée 4 (côté bâbord du parcours)
    # Conventions side="starboard"/"port" pour le contournement (cf. doc en tête).
    _COURSE1_SPIRAL_T = [
        ("BOUEE_HAUTE_T1", "3", "starboard"),
        ("BOUEE_BASSE_T1", "1", "port"),
        ("BOUEE_HAUTE_T2", "4", "port"),
    ]
    _COURSE1_SPIRAL_B = [
        ("BOUEE_HAUTE_T1", "4", "port"),
        ("BOUEE_BASSE_T1", "2", "starboard"),
        ("BOUEE_HAUTE_T2", "3", "starboard"),
    ]

    def __init__(self):
        # Utilise COURSE_LEGS (alias dynamique selon config.COURSE_NUMBER)
        self.legs: List[Leg] = [Leg(**l) for l in config.COURSE_LEGS]
        self.current_idx: int = 0
        self.last_position: Optional[GPSPos] = None
        self.race_started: bool = False
        self.race_finished: bool = False
        # Parcours 1 : côté choisi UNE FOIS au franchissement du départ
        self.course1_chosen_side: Optional[str] = None   # "T" ou "B"

    # ─────────────────────────────────────────────
    # Parcours 1 — choix initial et résolution
    # ─────────────────────────────────────────────
    def _choose_course1_side(self, wind_dir_deg: Optional[float]) -> str:
        """Choisit T ou B selon la bouée la plus exposée au vent.

        Si le vent est inconnu → fallback "T" (choix arbitraire mais stable).
        Convention : direction d'OÙ vient le vent (météo).
        """
        if wind_dir_deg is None:
            log.warning("[COURSE1] Vent inconnu au franchissement DEPART → "
                        "fallback côté T (3 → 1 → 4)")
            return "T"
        try:
            b3 = config.BUOYS_GPS["3"]
            b4 = config.BUOYS_GPS["4"]
        except KeyError:
            log.error("[COURSE1] Bouées 3/4 absentes de BUOYS_GPS")
            return "T"
        # Centre de la porte basse comme origine
        b1 = config.BUOYS_GPS.get("1")
        b2 = config.BUOYS_GPS.get("2")
        if b1 is None or b2 is None:
            origin = (
                (b3[0] + b4[0]) / 2.0,
                (b3[1] + b4[1]) / 2.0,
            )
        else:
            origin = ((b1[0] + b2[0]) / 2.0, (b1[1] + b2[1]) / 2.0)
        # Vecteur unitaire "vers le vent" (direction d'OÙ il vient)
        wind_e, wind_n = geo_utils.cap_to_unit_vector(wind_dir_deg)
        # Position des 2 bouées hautes en m relatifs à origin
        e3, n3 = geo_utils.offset_meters(origin, b3)
        e4, n4 = geo_utils.offset_meters(origin, b4)
        proj_3 = e3 * wind_e + n3 * wind_n
        proj_4 = e4 * wind_e + n4 * wind_n
        if proj_3 >= proj_4:
            return "T"
        return "B"

    def _apply_course1_choice(self, side: str):
        """Réécrit les 3 legs intermédiaires selon le côté retenu (T ou B).

        Idempotent : appelable plusieurs fois sans effet de bord supplémentaire.
        """
        if config.COURSE_NUMBER != 1 or len(self.legs) < 4:
            return
        spiral = self._COURSE1_SPIRAL_T if side == "T" else self._COURSE1_SPIRAL_B
        for offset, (name, buoy, side_buoy) in enumerate(spiral, start=1):
            self.legs[offset] = Leg(name=name, buoy=buoy, side=side_buoy)
        self.course1_chosen_side = side
        seq = " → ".join(s[1] for s in spiral)
        log.info(
            "[COURSE1] Côté retenu = %s → séquence figée : %s → arrivée(1-2)",
            side, seq,
        )

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
        """Décale la cible de BUOY_CLEARANCE_M du bon côté de la bouée."""
        de, dn = geo_utils.offset_meters(buoy_pos, next_buoy_pos)
        norm = math.hypot(de, dn)
        if norm < 1e-3:
            return buoy_pos
        ue, un = de / norm, dn / norm
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
        # Dernière étape : on retourne le centre de la porte d'arrivée
        # (qui est la même que celle de départ)
        return geo_utils.buoy_gps(self.legs[0].buoy)

    def _gate_target_pos(self, gate_name: str) -> GPSPos:
        """Point d'approche pour une porte = milieu géométrique.

        En parcours 1, le choix de la bouée à enrouler est fait UNE FOIS
        au franchissement de la ligne de départ (cf. _apply_course1_choice).
        Les 3 legs intermédiaires deviennent des legs de bouée individuelle
        avec side="starboard"/"port" — ils ne passent donc plus par cette
        méthode. Seules les portes DEPART et ARRIVEE l'utilisent encore.
        """
        return geo_utils.buoy_gps(gate_name)

    # ─────────────────────────────────────────────
    # Plan immédiat
    # ─────────────────────────────────────────────
    def plan(
        self,
        boat_pos: GPSPos,
        wind_dir_deg: Optional[float] = None,
    ) -> Optional[WaypointPlan]:
        """Calcule le plan de navigation immédiat.

        Args:
            boat_pos: position GPS du bateau (lat, lon).
            wind_dir_deg: direction d'OÙ vient le vent (utilisé seulement
                          au franchissement de la porte départ du parcours 1
                          pour figer le côté T/B de la spirale).

        Returns:
            WaypointPlan ou None si la course est finie.
        """
        leg = self.current_leg
        if leg is None:
            return None
        if leg.side == "gate":
            target = self._gate_target_pos(leg.buoy)
            buoy_pos = geo_utils.buoy_gps(leg.buoy)   # centre de porte (validation)
            is_gate = True
        else:
            buoy_pos = geo_utils.buoy_gps(leg.buoy)
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
                            rtk_fixed: bool = False,
                            wind_dir_deg: Optional[float] = None) -> bool:
        """À appeler à chaque tick. Retourne True si une étape vient d'être
        validée (utile pour logger et déclencher les transitions d'état).

        Args:
            boat_pos: position GPS du bateau (lat, lon).
            rtk_fixed: True si le fix GPS est RTK (≥ Float). Détermine le
                rayon de capture (4 m RTK / 7 m GPS).
            wind_dir_deg: direction d'OÙ vient le vent — utilisé seulement
                au franchissement de la porte départ du parcours 1 pour
                figer le côté T/B de la spirale.
        """
        leg = self.current_leg
        if leg is None:
            return False

        validated = False

        if leg.side == "gate":
            # Croisement du segment de la porte (n'importe laquelle : AB/12/34)
            try:
                a_gps, b_gps = geo_utils.gate_endpoints_gps(leg.buoy)
            except (KeyError, IndexError):
                log.error("[COURSE] Porte %s introuvable dans BUOYS_GPS", leg.buoy)
                return False
            if self.last_position is not None:
                if geo_utils.line_crossed(
                    self.last_position, boat_pos, a_gps, b_gps
                ):
                    if leg.name == "DEPART":
                        if not self.race_started:
                            self.race_started = True
                            log.info("[COURSE] Porte de départ %s franchie", leg.buoy)
                            # Parcours 1 : c'est ICI qu'on FIGE le côté de
                            # la spirale (3→1→4 ou 4→2→3) selon le vent
                            if (config.COURSE_NUMBER == 1
                                    and self.course1_chosen_side is None):
                                side = self._choose_course1_side(wind_dir_deg)
                                self._apply_course1_choice(side)
                            validated = True
                    elif leg.name == "ARRIVEE":
                        if self.race_started and self.current_idx == len(self.legs) - 1:
                            self.race_finished = True
                            log.info("[COURSE] Porte d'arrivée %s franchie — FIN", leg.buoy)
                            validated = True
                    else:
                        # Porte intermédiaire — n'est plus utilisée par les
                        # 2 parcours du briefing 8/5 (1 banane et 2 "5 bouées"),
                        # qui n'ont que DEPART et ARRIVEE comme gates. Le code
                        # reste en place pour rétrocompatibilité future.
                        if self.race_started:
                            log.info("[COURSE] Porte %s (%s) franchie", leg.buoy, leg.name)
                            validated = True
        else:
            # Validation bouée : rayon adaptatif RTK/GPS
            buoy_pos = geo_utils.buoy_gps(leg.buoy)
            d = geo_utils.distance_m(boat_pos, buoy_pos)
            radius = config.capture_radius_for_fix(rtk_fixed)
            if d < radius:
                if self.last_position is not None and self._side_correct(
                    self.last_position, boat_pos, buoy_pos, leg.side
                ):
                    validated = True
                else:
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
        """Vrai si la bouée a été contournée du bon côté."""
        dir_e, dir_n = geo_utils.offset_meters(prev_pos, curr_pos)
        b_e, b_n = geo_utils.offset_meters(curr_pos, buoy_pos)
        cross = dir_e * b_n - dir_n * b_e
        if side == "starboard":
            return cross < 0
        elif side == "port":
            return cross > 0
        return True

    def reset(self):
        # Recharge la définition initiale (au cas où on a déjà remplacé
        # les legs par la spirale parcours 1 lors d'une session précédente).
        self.legs = [Leg(**l) for l in config.COURSE_LEGS]
        self.current_idx = 0
        self.race_started = False
        self.race_finished = False
        self.last_position = None
        self.course1_chosen_side = None
