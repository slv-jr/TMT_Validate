"""
Bascule entre mode MANUEL (téléopéré RC) et mode AUTO (Pi en contrôle).

Stratégie validée par le PoC SwarmZ + setup officiel "NouvelEncodeur" :
    - Le Cube Orange+ reste en mode permanent MANUAL.
    - Le levier 3 positions de la J4C05 sert d'interrupteur. Côté récepteur
      Joysway J5C01R, c'est le canal CH5. Côté Pi (après le PPM du Nano),
      on le lit dans `RC_CHANNELS.chan6_raw` (config.CH_MODE = 6) :
        chan6 < 1300 µs → AUTO (Pi prend la main via RC_CHANNELS_OVERRIDE)
        chan6 > 1500 µs → MANUAL (Pi libère, RC reprend physiquement)
    - Hystérésis pour éviter les bascules erratiques en mi-position.

En mode MANUAL, le Pi continue à logger et à broadcaster sa position LoRa,
mais il N'ENVOIE PAS d'override → la radio J4C05 contrôle directement les
servos via le passthrough ArduRover (chan4 → safran, chan5 → voile).

En mode AUTO, le Pi pilote tout via RC_CHANNELS_OVERRIDE à 10 Hz.

⚠️ Sécurité : si le Pi détecte une perte de heartbeat MAVLink ou un
problème grave, il libère IMMÉDIATEMENT les overrides → la RC reprend
le contrôle physique sans autre action de l'opérateur.
"""

import logging
import time
from enum import Enum
from dataclasses import dataclass

import config
from comms.mavlink_iface import MavlinkInterface

log = logging.getLogger(__name__)


class ControlMode(Enum):
    AUTO = "AUTO"        # Pi en contrôle
    MANUAL = "MANUAL"    # RC en contrôle (téléopéré)
    UNKNOWN = "UNKNOWN"


@dataclass
class ModeStatus:
    mode: ControlMode
    mode_pwm: int                  # PWM brut du levier (config.CH_MODE)
    transition_count: int
    last_change_t: float

    # Compat rétro (anciens scripts/tests qui lisent .ch3_pwm)
    @property
    def ch3_pwm(self) -> int:
        return self.mode_pwm


class ModeSwitch:
    """Lit le canal du levier mode et décide du mode de contrôle."""

    def __init__(self, mav: MavlinkInterface):
        self.mav = mav
        self._mode = ControlMode.UNKNOWN
        self._transitions = 0
        self._last_change_t = time.monotonic()
        self._last_mode_pwm = 0

    def update(self) -> ModeStatus:
        """À appeler à chaque tick navigation."""
        tlm = self.mav.get_telemetry()
        ch_mode = tlm.rc_channels.get(config.CH_MODE, 0)
        self._last_mode_pwm = ch_mode

        new_mode = self._mode
        if ch_mode == 0:
            # Pas encore reçu — état UNKNOWN (sécurité : on ne pilote pas)
            new_mode = ControlMode.UNKNOWN
        elif ch_mode < config.MODE_THRESHOLD_LOW:
            new_mode = ControlMode.AUTO
        elif ch_mode > config.MODE_THRESHOLD_HIGH:
            new_mode = ControlMode.MANUAL
        # Sinon, on conserve l'état précédent (hystérésis)

        if new_mode != self._mode:
            log.info(
                "[MODE] Bascule %s → %s (chan%d=%dµs)",
                self._mode.value, new_mode.value, config.CH_MODE, ch_mode,
            )
            self._transitions += 1
            self._last_change_t = time.monotonic()
            self._mode = new_mode

            # Si on bascule vers MANUAL : libérer immédiatement les overrides
            if new_mode == ControlMode.MANUAL:
                self.mav.clear_all_overrides()

        return ModeStatus(
            mode=self._mode,
            mode_pwm=ch_mode,
            transition_count=self._transitions,
            last_change_t=self._last_change_t,
        )

    @property
    def is_auto(self) -> bool:
        return self._mode == ControlMode.AUTO

    @property
    def is_manual(self) -> bool:
        return self._mode == ControlMode.MANUAL

    def force_manual(self, reason: str = ""):
        """Force le mode MANUAL (urgence). Libère les overrides du Pi."""
        if self._mode != ControlMode.MANUAL:
            log.warning("[MODE] Force MANUAL — raison: %s", reason)
        self._mode = ControlMode.MANUAL
        self.mav.clear_all_overrides()
