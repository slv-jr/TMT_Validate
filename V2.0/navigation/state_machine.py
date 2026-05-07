"""
Machine à états de navigation principale (cf. README2).

États :
    INIT                 → connexion capteurs, attente fix RTK
    ATTENTE              → attend signal départ
    EN_COURSE            → navigation active
       ├─ APPROCHE_BOUEE
       ├─ REMONTEE_VENT      (allure de près, louvoyage)
       ├─ DESCENTE           (allure portant)
       ├─ REACHING           (travers / largue)
       └─ EVITEMENT_URGENCE  (collision imminente)
    STALL_RECOVERY       → manœuvre dégagement auto (cf. stall_detector)
    PENALITE             → tour pénalité Z1/Z2/Z1 (cf. penalty_manager)
    DEGRADE              → mode dégradé (perte GPS/LoRa)
    REPRISE_RC           → reprise télécommande (CH3 haut)
    FIN_COURSE           → post-arrivée, logging
"""

import logging
import time
from enum import Enum
from dataclasses import dataclass

log = logging.getLogger(__name__)


class NavState(Enum):
    INIT = "INIT"
    ATTENTE = "ATTENTE"
    EN_COURSE = "EN_COURSE"
    APPROCHE_BOUEE = "APPROCHE_BOUEE"
    REMONTEE_VENT = "REMONTEE_VENT"
    DESCENTE = "DESCENTE"
    REACHING = "REACHING"
    EVITEMENT_URGENCE = "EVITEMENT_URGENCE"
    STALL_RECOVERY = "STALL_RECOVERY"
    PENALITE = "PENALITE"
    DEGRADE = "DEGRADE"
    REPRISE_RC = "REPRISE_RC"
    FIN_COURSE = "FIN_COURSE"


class NavStateMachine:
    """Machine à états simple avec garde-fous anti-flapping."""

    MIN_TIME_IN_STATE_S = 0.5

    def __init__(self):
        self._state: NavState = NavState.INIT
        self._entered_t: float = time.monotonic()
        self._previous: NavState = NavState.INIT

    @property
    def state(self) -> NavState:
        return self._state

    @property
    def time_in_state(self) -> float:
        return time.monotonic() - self._entered_t

    def transition(self, new_state: NavState, reason: str = "") -> bool:
        if new_state == self._state:
            return False
        if self.time_in_state < self.MIN_TIME_IN_STATE_S and new_state == self._previous:
            # Anti-oscillation rapide
            return False
        log.info("[NAV] %s → %s%s",
                 self._state.value, new_state.value,
                 f" ({reason})" if reason else "")
        self._previous = self._state
        self._state = new_state
        self._entered_t = time.monotonic()
        return True

    def force(self, new_state: NavState, reason: str = ""):
        log.warning("[NAV] FORCE %s → %s%s",
                    self._state.value, new_state.value,
                    f" ({reason})" if reason else "")
        self._previous = self._state
        self._state = new_state
        self._entered_t = time.monotonic()
