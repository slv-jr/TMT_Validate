"""
Détection de blocage / collision sans caméra (cf. README2 §"Détection blocage").

Le drone est considéré BLOQUÉ si **2 conditions sur 3** sont vraies pendant
plus de `STALL_DURATION_S` secondes :

    1. Vitesse sol quasi-nulle (`speed < STALL_SPEED_THRESHOLD_MS`)
    2. Commandes safran actives (`|rudder_cmd| > STALL_RUDDER_THRESHOLD_DEG`)
    3. Distance au waypoint figée (variation < `STALL_WP_DISTANCE_HYSTERESIS_M`
       sur la fenêtre `STALL_WINDOW_S`)

L'escalade des réactions est temporelle :

    - 0 - 8 s   → niveau LIGHT  : choquer la voile + virer de bord
    - 8 - 15 s  → niveau MEDIUM : boost (si budget) + manœuvre dégagement
    - > 15 s    → niveau HARD   : alerte buzzer / proposer reprise RC

Ce module ne fait pas l'action : il EXPOSE l'état (`StallStatus`) et c'est
`main.py` qui décide de la réaction, en s'appuyant sur le boost_controller,
le mode_switch et la state_machine.
"""

from __future__ import annotations

import logging
import time
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Deque, Tuple

import config

log = logging.getLogger(__name__)


class StallLevel(Enum):
    NONE = "NONE"
    LIGHT = "LIGHT"     # 0-8 s : voile + virement
    MEDIUM = "MEDIUM"   # 8-15 s : boost + dégagement
    HARD = "HARD"       # > 15 s : alerte / RC


@dataclass
class StallStatus:
    is_stalled: bool
    level: StallLevel
    stalled_since_s: float          # secondes depuis détection
    conditions_met: int             # 0-3
    speed_low: bool
    rudder_active: bool
    distance_frozen: bool


class StallDetector:
    """Détecteur comportemental — historique glissant pour la condition #3."""

    def __init__(self):
        # Historique (timestamp, distance_au_wp_m) sur la fenêtre STALL_WINDOW_S
        self._history: Deque[Tuple[float, float]] = deque()
        self._stall_start_t: float = 0.0
        self._level: StallLevel = StallLevel.NONE

    def update(self,
               boat_speed_ms: float,
               rudder_cmd_deg: float,
               distance_to_wp_m: float) -> StallStatus:
        """À appeler à chaque tick."""
        now = time.monotonic()

        # Mettre à jour l'historique de distance au WP
        self._history.append((now, distance_to_wp_m))
        # Purge des points hors fenêtre
        while self._history and now - self._history[0][0] > config.STALL_WINDOW_S:
            self._history.popleft()

        # ── Évaluer chaque condition ──
        speed_low = boat_speed_ms < config.STALL_SPEED_THRESHOLD_MS
        rudder_active = abs(rudder_cmd_deg) > config.STALL_RUDDER_THRESHOLD_DEG

        distance_frozen = False
        if len(self._history) >= 2:
            ds = [d for _, d in self._history]
            d_min, d_max = min(ds), max(ds)
            # Fenêtre couverte ?
            window_covered = (
                self._history[-1][0] - self._history[0][0]
                >= 0.8 * config.STALL_WINDOW_S
            )
            if window_covered:
                distance_frozen = (d_max - d_min) < config.STALL_WP_DISTANCE_HYSTERESIS_M

        conditions_met = sum([speed_low, rudder_active, distance_frozen])
        currently_stalling = conditions_met >= 2

        # ── Machine d'état ──
        if currently_stalling:
            if self._stall_start_t == 0.0:
                self._stall_start_t = now
                # On NE déclenche pas tout de suite — il faut attendre STALL_DURATION_S
            stalled_for = now - self._stall_start_t
            if stalled_for < config.STALL_DURATION_S:
                # Latence — on considère pas encore "officiellement" bloqué
                return StallStatus(
                    is_stalled=False,
                    level=StallLevel.NONE,
                    stalled_since_s=0.0,
                    conditions_met=conditions_met,
                    speed_low=speed_low,
                    rudder_active=rudder_active,
                    distance_frozen=distance_frozen,
                )
            # Niveau d'escalade selon la durée
            new_level = self._level_from_duration(stalled_for)
            if new_level != self._level:
                log.warning(
                    "[STALL] Niveau %s → %s (depuis %.1fs ; conditions=%d/3 "
                    "speed_low=%s rudder=%s frozen=%s)",
                    self._level.value, new_level.value, stalled_for,
                    conditions_met, speed_low, rudder_active, distance_frozen,
                )
                self._level = new_level
            return StallStatus(
                is_stalled=True,
                level=self._level,
                stalled_since_s=stalled_for,
                conditions_met=conditions_met,
                speed_low=speed_low,
                rudder_active=rudder_active,
                distance_frozen=distance_frozen,
            )

        # Pas (plus) bloqué → reset
        if self._stall_start_t > 0.0:
            log.info(
                "[STALL] Résolution — était bloqué depuis %.1fs",
                now - self._stall_start_t,
            )
        self._stall_start_t = 0.0
        self._level = StallLevel.NONE
        return StallStatus(
            is_stalled=False,
            level=StallLevel.NONE,
            stalled_since_s=0.0,
            conditions_met=conditions_met,
            speed_low=speed_low,
            rudder_active=rudder_active,
            distance_frozen=distance_frozen,
        )

    @staticmethod
    def _level_from_duration(stalled_for: float) -> StallLevel:
        if stalled_for < config.STALL_LIGHT_REACTION_MAX_S:
            return StallLevel.LIGHT
        if stalled_for < config.STALL_MED_REACTION_MAX_S:
            return StallLevel.MEDIUM
        return StallLevel.HARD

    def reset(self):
        """À appeler à la fin d'un événement (ex: après une réaction réussie)."""
        self._history.clear()
        self._stall_start_t = 0.0
        self._level = StallLevel.NONE
