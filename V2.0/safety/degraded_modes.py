"""
Modes dégradés (cf. README2 §"Modes dégradés").

Surveille les pannes possibles et bascule la stratégie en conséquence :

| Mode               | Déclencheur                       | Réaction                          |
|--------------------|-----------------------------------|-----------------------------------|
| GPS_LOST           | pas de fix > GPS_FIX_TIMEOUT_S    | dead-reckoning (cap+vitesse)      |
| RTK_DEGRADED       | RTK perdu, GPS standard           | rayon capture → 7 m               |
| LORA_LOST          | pas de trame > 10 s               | vent défaut + solo                |
| WIND_STALE         | vent > WIND_FALLBACK_TIMEOUT_S    | idem LORA_LOST                    |
| LOW_BATTERY        | < LOW_BATTERY_PCT (ou < 11.1 V)   | désactive boost                   |
| MAVLINK_LOST       | pas de heartbeat > 3 s            | pause + watchdog reset            |
| STALL_DETECTED     | blocage > 3 s                     | manœuvre dégagement auto          |
| ADVERSARY_SILENT   | pas de P\\| > 10 s pour adversaire | obstacle figé dernier relevé      |

L'objectif : ne JAMAIS s'arrêter complètement, dégrader proprement.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from enum import Enum
from typing import List

import config

log = logging.getLogger(__name__)


class DegradedMode(Enum):
    NOMINAL = "NOMINAL"
    GPS_LOST = "GPS_LOST"                   # plus aucun fix → estime
    RTK_DEGRADED = "RTK_DEGRADED"           # GPS standard, plus de RTK
    LORA_LOST = "LORA_LOST"                 # plus de comm essaim
    WIND_STALE = "WIND_STALE"               # plus de Calypso
    LOW_BATTERY = "LOW_BATTERY"             # < seuil
    MAVLINK_LOST = "MAVLINK_LOST"           # heartbeat Cube perdu (CRITIQUE)
    STALL_DETECTED = "STALL_DETECTED"       # blocage comportemental
    ADVERSARY_SILENT = "ADVERSARY_SILENT"   # adversaire muet → obstacle figé


@dataclass
class DegradedState:
    active_modes: List[DegradedMode] = field(default_factory=list)
    severity: int = 0       # 0 = nominal, 1 = warning, 2 = critique
    rtk_fixed: bool = False

    def has(self, mode: DegradedMode) -> bool:
        return mode in self.active_modes


class DegradedManager:
    """Surveille les capteurs et publie un état de dégradation global."""

    LORA_LOST_TIMEOUT_S = 10.0   # cf. README2 — différent du timeout équipe (180 s)

    def __init__(self):
        self._state = DegradedState()

    def update(self,
               has_gps_fix: bool,
               gps_fix_type: int,
               last_gps_age_s: float,
               last_lora_msg_age_s: float,
               wind_age_s: float,
               wind_confident: bool,
               battery_pct: float,
               mavlink_alive: bool,
               stall_detected: bool = False,
               adversary_silent: bool = False) -> DegradedState:
        """À appeler à chaque tick navigation."""
        modes: List[DegradedMode] = []
        severity = 0

        if not mavlink_alive:
            modes.append(DegradedMode.MAVLINK_LOST)
            severity = max(severity, 2)

        # GPS — distinguer perte totale et perte RTK
        rtk_fixed = (gps_fix_type >= 5) and has_gps_fix
        if not has_gps_fix or last_gps_age_s > config.GPS_FIX_TIMEOUT_S:
            modes.append(DegradedMode.GPS_LOST)
            severity = max(severity, 2)
        elif gps_fix_type < 5:
            modes.append(DegradedMode.RTK_DEGRADED)
            severity = max(severity, 1)

        if last_lora_msg_age_s > self.LORA_LOST_TIMEOUT_S:
            modes.append(DegradedMode.LORA_LOST)
            severity = max(severity, 1)

        if not wind_confident or wind_age_s > config.WIND_FALLBACK_TIMEOUT_S:
            modes.append(DegradedMode.WIND_STALE)
            severity = max(severity, 1)

        if battery_pct < config.LOW_BATTERY_PCT:
            modes.append(DegradedMode.LOW_BATTERY)
            severity = max(severity, 1)

        if stall_detected:
            modes.append(DegradedMode.STALL_DETECTED)
            severity = max(severity, 1)

        if adversary_silent:
            modes.append(DegradedMode.ADVERSARY_SILENT)
            severity = max(severity, 1)

        # Diff par rapport à l'état précédent → log uniquement si changement
        prev = set(m.value for m in self._state.active_modes)
        curr = set(m.value for m in modes)
        if prev != curr:
            for m in (curr - prev):
                log.warning("[DEGRADED] Activation : %s", m)
            for m in (prev - curr):
                log.info("[DEGRADED] Résolution : %s", m)

        self._state = DegradedState(
            active_modes=modes,
            severity=severity,
            rtk_fixed=rtk_fixed,
        )
        return self._state

    @property
    def state(self) -> DegradedState:
        return self._state
