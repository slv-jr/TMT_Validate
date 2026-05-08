"""
Gestion d'une pénalité (cf. règlement Battleboats 2026 V2.3 §6).

La séquence de pénalité dépend du parcours actif (config.COURSE_NUMBER) :

    Parcours 1 (banane) :
        1. Passer entre 2 et P1 (vers P1)
        2. Enrouler P2 par BÂBORD
        3. Enrouler P1 par BÂBORD
    Parcours 2 (côtier court) :
        1. Passer entre A et Z1 (vers Z1)
        2. Enrouler Z2 par BÂBORD
        3. Enrouler Z1 par BÂBORD
    4. Reprendre le parcours là où on l'avait laissé.

La séquence de bouées correspondante (P1/P2 ou Z1/Z2) est déjà
sélectionnée dynamiquement via `config.PENALTY_LEGS`. Ce module n'a
donc pas besoin de connaître le parcours — il suit la séquence active.

Politique de bascule (cf. README2) :
    - Au déclenchement d'une pénalité : on entre en sous-état `WAIT`.
    - Pendant `PENALTY_DECISION_TIMEOUT_S` (5 s) le pilote peut basculer
      le levier de mode (config.CH_MODE) en HAUT (MANUEL) — alors le
      sous-état devient `MANUAL` et l'auto cède. L'opérateur est limité
      à `PENALTY_MANUAL_MAX_S` (30 s) puis l'auto reprend (règlement §6).
    - Si le pilote ne bascule pas → sous-état `AUTO` : on enchaîne la
      séquence automatique.

À la fin de la séquence (auto ou manuelle), le PenaltyManager retourne
`finished=True` et `main.py` rebascule sur l'étape de course interrompue.

Toute la logique est en GPS (lat, lon). Les distances utilisent le rayon
adaptatif RTK/GPS de `config.capture_radius_for_fix(rtk_fixed)`.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import List, Optional, Tuple

import config
from navigation import geo_utils

log = logging.getLogger(__name__)

GPSPos = Tuple[float, float]


class PenaltyMode(Enum):
    INACTIVE = "INACTIVE"
    WAIT = "WAIT"        # 0-5 s : laisser au pilote l'opportunité de prendre la main
    MANUAL = "MANUAL"    # le pilote tient le bateau
    AUTO = "AUTO"        # séquence autonome
    DONE = "DONE"        # pénalité purgée → reprise course


@dataclass
class PenaltyLeg:
    name: str
    buoy: str
    side: str
    completed: bool = False


@dataclass
class PenaltyStatus:
    mode: PenaltyMode
    current_leg_idx: int
    total_legs: int
    target_pos: Optional[GPSPos]
    distance_m: float
    bearing_deg: float
    elapsed_s: float
    finished: bool


class PenaltyManager:
    """État et progression d'une pénalité (cf. README2)."""

    def __init__(self):
        self._mode: PenaltyMode = PenaltyMode.INACTIVE
        self._legs: List[PenaltyLeg] = [
            PenaltyLeg(**leg) for leg in config.PENALTY_LEGS
        ]
        self._idx: int = 0
        self._t0: float = 0.0
        self._mode_start_t: float = 0.0
        self._last_position: Optional[GPSPos] = None
        # Étape du parcours interrompue (réinjectée après la pénalité)
        self._interrupted_leg_idx: int = -1

    # ─────────────────────────────────────────────
    # Cycle de vie
    # ─────────────────────────────────────────────
    @property
    def mode(self) -> PenaltyMode:
        return self._mode

    @property
    def is_active(self) -> bool:
        return self._mode not in (PenaltyMode.INACTIVE, PenaltyMode.DONE)

    @property
    def interrupted_leg_idx(self) -> int:
        return self._interrupted_leg_idx

    def start(self, interrupted_leg_idx: int = -1):
        """Déclenche une pénalité."""
        if self.is_active:
            log.warning("[PENALTY] Déjà active — start() ignoré")
            return
        self._mode = PenaltyMode.WAIT
        self._idx = 0
        self._t0 = time.monotonic()
        self._mode_start_t = self._t0
        self._interrupted_leg_idx = interrupted_leg_idx
        for leg in self._legs:
            leg.completed = False
        log.warning(
            "[PENALTY] Démarrée — étape interrompue=%d. Pilote a %.0fs pour reprendre RC.",
            interrupted_leg_idx, config.PENALTY_DECISION_TIMEOUT_S,
        )

    def reset(self):
        """Termine et nettoie l'état (pour une nouvelle course)."""
        self._mode = PenaltyMode.INACTIVE
        self._idx = 0
        self._t0 = 0.0
        self._mode_start_t = 0.0
        self._last_position = None
        self._interrupted_leg_idx = -1
        for leg in self._legs:
            leg.completed = False

    # ─────────────────────────────────────────────
    # Boucle principale
    # ─────────────────────────────────────────────
    def update(self,
               boat_pos: GPSPos,
               control_mode_is_manual: bool,
               rtk_fixed: bool) -> PenaltyStatus:
        """À appeler à chaque tick navigation tant que la pénalité est active."""
        now = time.monotonic()

        # ── Sous-état WAIT : 5 s pour basculer en MANUEL ──
        if self._mode == PenaltyMode.WAIT:
            if control_mode_is_manual:
                self._mode = PenaltyMode.MANUAL
                self._mode_start_t = now
                log.warning("[PENALTY] Pilote a pris la main → mode MANUAL")
            elif (now - self._t0) >= config.PENALTY_DECISION_TIMEOUT_S:
                self._mode = PenaltyMode.AUTO
                self._mode_start_t = now
                seq = " → ".join(f"{leg.buoy} {leg.side}" for leg in self._legs)
                log.warning(
                    "[PENALTY] Timeout %.0fs → séquence AUTO (%s)",
                    config.PENALTY_DECISION_TIMEOUT_S, seq,
                )

        # ── Sous-état MANUAL : limiter à 30 s puis reprendre auto ──
        if self._mode == PenaltyMode.MANUAL:
            if not control_mode_is_manual:
                # Le pilote a relâché — on bascule en AUTO pour finir la séquence
                self._mode = PenaltyMode.AUTO
                self._mode_start_t = now
                log.warning(
                    "[PENALTY] RC relâchée → reprise séquence AUTO à l'étape %d",
                    self._idx,
                )
            elif (now - self._mode_start_t) >= config.PENALTY_MANUAL_MAX_S:
                self._mode = PenaltyMode.AUTO
                self._mode_start_t = now
                log.warning(
                    "[PENALTY] Limite %.0fs en MANUEL → reprise AUTO",
                    config.PENALTY_MANUAL_MAX_S,
                )

        # ── Sous-état AUTO : avancer dans la séquence ──
        if self._mode == PenaltyMode.AUTO:
            self._advance_if_reached(boat_pos, rtk_fixed)

        # ── Construction du status ──
        target_pos: Optional[GPSPos] = None
        distance = 0.0
        bearing = 0.0
        if self._idx < len(self._legs):
            leg = self._legs[self._idx]
            target_pos = geo_utils.buoy_gps(leg.buoy)
            distance = geo_utils.distance_m(boat_pos, target_pos)
            bearing = geo_utils.bearing_deg(boat_pos, target_pos)
        else:
            self._mode = PenaltyMode.DONE

        finished = (self._mode == PenaltyMode.DONE)

        self._last_position = boat_pos
        return PenaltyStatus(
            mode=self._mode,
            current_leg_idx=self._idx,
            total_legs=len(self._legs),
            target_pos=target_pos,
            distance_m=distance,
            bearing_deg=bearing,
            elapsed_s=now - self._t0 if self._t0 else 0.0,
            finished=finished,
        )

    # ─────────────────────────────────────────────
    # Logique interne
    # ─────────────────────────────────────────────
    def _advance_if_reached(self, boat_pos: GPSPos, rtk_fixed: bool):
        if self._idx >= len(self._legs):
            self._mode = PenaltyMode.DONE
            return
        leg = self._legs[self._idx]
        buoy_pos = geo_utils.buoy_gps(leg.buoy)
        dist = geo_utils.distance_m(boat_pos, buoy_pos)
        radius = config.capture_radius_for_fix(rtk_fixed)
        if dist < radius:
            # Vérification du côté de passage si on a un point précédent
            side_ok = True
            if self._last_position is not None:
                side_ok = self._side_correct(
                    self._last_position, boat_pos, buoy_pos, leg.side,
                )
            if not side_ok:
                log.warning(
                    "[PENALTY] Bouée %s atteinte (%.1f m) mais côté %s incertain",
                    leg.buoy, dist, leg.side,
                )
            leg.completed = True
            log.info(
                "[PENALTY] Étape %s validée (%d/%d, dist=%.1f m, %s)",
                leg.name, self._idx + 1, len(self._legs), dist,
                "RTK" if rtk_fixed else "GPS",
            )
            self._idx += 1
            if self._idx >= len(self._legs):
                self._mode = PenaltyMode.DONE
                log.warning(
                    "[PENALTY] Séquence terminée — reprise du parcours à l'étape %d",
                    self._interrupted_leg_idx,
                )

    @staticmethod
    def _side_correct(prev: GPSPos, curr: GPSPos, buoy: GPSPos, side: str) -> bool:
        de1, dn1 = geo_utils.offset_meters(prev, curr)
        de2, dn2 = geo_utils.offset_meters(curr, buoy)
        cross = de1 * dn2 - dn1 * de2
        if side == "starboard":
            return cross < 0
        if side == "port":
            return cross > 0
        return True

    # ─────────────────────────────────────────────
    # Pour state_machine et logging
    # ─────────────────────────────────────────────
    def current_target(self) -> Optional[GPSPos]:
        if not (0 <= self._idx < len(self._legs)):
            return None
        return geo_utils.buoy_gps(self._legs[self._idx].buoy)

    def progress_str(self) -> str:
        return f"{self._idx}/{len(self._legs)} ({self._mode.value})"
