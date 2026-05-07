"""
Gestion du moteur boost électrique (cf. README2 §"Boost").

Règlement BattleBoats 2026 §2c — version annoncée la veille du briefing :
    - Boost moteur autorisé : Ø hélice ≤ 30 mm
    - **BUDGET TOTAL** = `BOOST_MAX_S` secondes par course (sommables sur
      plusieurs activations — par défaut 30 s, à confirmer après briefing)
    - **NOMBRE MAX D'ACTIVATIONS** = `BOOST_ACTIONS_MAX` (par défaut 3)

⚠️ Ce n'est PAS 3 × 30 s = 90 s — c'est 30 s TOTAL répartis sur ≤ 3 actions.

Sur l'équipe UTT, seul le drone U1B1 (Scout) embarque le moteur. Le pilotage
se fait via le canal RC `BOOST_RC_CHANNEL` (CH4 par défaut) en envoyant un
override PWM lorsque les conditions d'activation sont réunies.

Conditions d'activation automatique (priorité décroissante, README2) :
    1. Vent < BOOST_TRIGGER_WIND_MS sur segment critique (zone morte)
    2. Vitesse < BOOST_TRIGGER_SPEED_MS pendant > BOOST_TRIGGER_LOW_SPEED_S
    3. Sortie de zone calme (entre F et G)

Sécurités :
    - Refuse si budget temps épuisé OU activations restantes = 0
    - Refuse si batterie < BOOST_LOW_BATTERY_PCT
    - Coupe automatiquement à BOOST_MAX_BURST_S ou si vitesse remonte
    - Coupe immédiatement à fin de course
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import config

log = logging.getLogger(__name__)


class BoostState(Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    EXHAUSTED = "EXHAUSTED"   # plus de budget temps OU plus d'activations
    DISABLED = "DISABLED"     # drone sans boost matériel


@dataclass
class BoostStatus:
    state: BoostState
    activations_used: int
    activations_remaining: int
    seconds_used: float
    seconds_remaining: float
    current_runtime_s: float


class BoostController:
    """Pilote le moteur boost — budget en temps total (cf. README2)."""

    def __init__(self, mav=None):
        self.mav = mav
        self._has_boost: bool = config.HAS_BOOST_MOTOR
        self._state: BoostState = (
            BoostState.IDLE if self._has_boost else BoostState.DISABLED
        )
        self._activations_used: int = 0
        self._seconds_used: float = 0.0
        self._activation_start_t: float = 0.0
        self._low_speed_since_t: float = 0.0
        self._last_high_speed_t: float = time.monotonic()

    # ─────────────────────────────────────────────
    # Propriétés
    # ─────────────────────────────────────────────
    @property
    def has_boost(self) -> bool:
        return self._has_boost

    @property
    def state(self) -> BoostState:
        return self._state

    @property
    def seconds_remaining(self) -> float:
        return max(0.0, config.BOOST_MAX_S - self._seconds_used)

    @property
    def activations_remaining(self) -> int:
        return max(0, config.BOOST_ACTIONS_MAX - self._activations_used)

    def can_boost(self) -> bool:
        """Renvoie True s'il reste du budget (temps + actions)."""
        return (self._has_boost
                and self._state != BoostState.EXHAUSTED
                and self._state != BoostState.DISABLED
                and self.seconds_remaining > 0.5
                and self.activations_remaining > 0)

    # ─────────────────────────────────────────────
    # API publique — appel à chaque tick navigation
    # ─────────────────────────────────────────────
    def update(self, boat_speed_ms: float, wind_speed_ms: float,
               battery_pct: float) -> BoostStatus:
        now = time.monotonic()

        if not self._has_boost or self._state == BoostState.DISABLED:
            return self._status(now)

        # Suivi de la vitesse pour détecter les conditions d'éligibilité
        if boat_speed_ms < config.BOOST_TRIGGER_SPEED_MS:
            if self._low_speed_since_t == 0.0:
                self._low_speed_since_t = now
        else:
            self._low_speed_since_t = 0.0
            if boat_speed_ms > config.BOOST_HIGH_SPEED_CUTOFF_MS:
                self._last_high_speed_t = now

        # Si le boost tourne → vérifier les conditions de coupure
        if self._state == BoostState.RUNNING:
            runtime = now - self._activation_start_t
            # Décompte du budget en continu (pas seulement à l'arrêt)
            # → on ne risque pas de dépasser même en cas de plantage du stop()
            if self._seconds_used + runtime >= config.BOOST_MAX_S:
                self._stop("budget temps épuisé")
            elif runtime >= config.BOOST_MAX_BURST_S:
                self._stop(f"rafale max {config.BOOST_MAX_BURST_S:.0f}s atteinte")
            elif boat_speed_ms > config.BOOST_HIGH_SPEED_CUTOFF_MS:
                # Le boost a redonné de la vitesse → on coupe pour économiser
                self._stop(f"vitesse > {config.BOOST_HIGH_SPEED_CUTOFF_MS:.1f} m/s")
            elif battery_pct < config.BOOST_LOW_BATTERY_PCT:
                self._stop(f"batterie < {config.BOOST_LOW_BATTERY_PCT:.0f}%")

        return self._status(now)

    def request_boost(self, boat_speed_ms: float, wind_speed_ms: float,
                      battery_pct: float, reason: str = "auto") -> bool:
        """Demande explicite. Retourne True si activation effective."""
        now = time.monotonic()

        if not self._has_boost:
            return False
        if self._state == BoostState.RUNNING:
            log.debug("[BOOST] Déjà en cours")
            return False
        if not self.can_boost():
            self._state = BoostState.EXHAUSTED
            log.warning(
                "[BOOST] Refus — budget épuisé (%.1fs/%.1fs · actions %d/%d)",
                self._seconds_used, config.BOOST_MAX_S,
                self._activations_used, config.BOOST_ACTIONS_MAX,
            )
            return False
        if battery_pct < config.BOOST_LOW_BATTERY_PCT:
            log.warning(
                "[BOOST] Refus : batterie %.1f%% < %.0f%%",
                battery_pct, config.BOOST_LOW_BATTERY_PCT,
            )
            return False

        # Activation effective
        self._activations_used += 1
        self._activation_start_t = now
        self._state = BoostState.RUNNING
        log.warning(
            "[BOOST] ON — activation %d/%d (raison=%s, "
            "vent=%.2fm/s, vitesse=%.2fm/s, "
            "budget restant=%.1fs)",
            self._activations_used, config.BOOST_ACTIONS_MAX,
            reason, wind_speed_ms, boat_speed_ms, self.seconds_remaining,
        )

        # Override PWM ON sur le canal CH4
        if self.mav is not None:
            self.mav.set_aux_pwm(config.BOOST_RC_CHANNEL, config.BOOST_PWM_ON)

        return True

    def auto_check(self, boat_speed_ms: float, wind_speed_ms: float,
                   battery_pct: float, in_dead_zone: bool,
                   role_authorizes: bool) -> bool:
        """Décide automatiquement s'il faut activer le boost.

        Conditions cumulatives :
            - Pas déjà actif
            - Vent < seuil OU dans la zone morte
            - Vitesse < seuil depuis BOOST_TRIGGER_LOW_SPEED_S
            - Budget temps + activations restant
            - Batterie OK
            - Rôle autorisé (Scout uniquement par défaut)
        """
        if not self._has_boost or not role_authorizes:
            return False
        if self._state != BoostState.IDLE:
            return False
        if not self.can_boost():
            return False
        if battery_pct < config.BOOST_LOW_BATTERY_PCT:
            return False
        # Vent ou zone morte
        if (wind_speed_ms >= config.BOOST_TRIGGER_WIND_MS
                and not in_dead_zone):
            return False
        # Bateau bloqué depuis assez longtemps
        if self._low_speed_since_t == 0.0:
            return False
        low_duration = time.monotonic() - self._low_speed_since_t
        if low_duration < config.BOOST_TRIGGER_LOW_SPEED_S:
            return False

        return self.request_boost(
            boat_speed_ms, wind_speed_ms, battery_pct, reason="auto-vent-mort",
        )

    def stop(self, reason: str = "manuel"):
        """Arrêt manuel du boost (sortie de zone morte, etc.)."""
        self._stop(reason)

    def reset_for_new_race(self):
        """À appeler entre 2 courses — remet les compteurs à zéro."""
        self._activations_used = 0
        self._seconds_used = 0.0
        self._state = (
            BoostState.IDLE if self._has_boost else BoostState.DISABLED
        )
        if self.mav is not None:
            self.mav.clear_aux_pwm(config.BOOST_RC_CHANNEL)
        log.info("[BOOST] Reset complet pour nouvelle course")

    # Alias rétrocompat
    reset = reset_for_new_race

    # ─────────────────────────────────────────────
    # Privé
    # ─────────────────────────────────────────────
    def _stop(self, reason: str):
        if self._state != BoostState.RUNNING:
            return
        runtime = time.monotonic() - self._activation_start_t
        self._seconds_used += runtime
        log.warning(
            "[BOOST] OFF — durée=%.1fs (raison=%s, total utilisé=%.1f/%.1fs · "
            "actions=%d/%d)",
            runtime, reason,
            self._seconds_used, config.BOOST_MAX_S,
            self._activations_used, config.BOOST_ACTIONS_MAX,
        )
        if (self._seconds_used >= config.BOOST_MAX_S
                or self._activations_used >= config.BOOST_ACTIONS_MAX):
            self._state = BoostState.EXHAUSTED
        else:
            self._state = BoostState.IDLE
        if self.mav is not None:
            self.mav.set_aux_pwm(config.BOOST_RC_CHANNEL, config.BOOST_PWM_OFF)

    def _status(self, now: float) -> BoostStatus:
        runtime = (
            (now - self._activation_start_t)
            if self._state == BoostState.RUNNING else 0.0
        )
        return BoostStatus(
            state=self._state,
            activations_used=self._activations_used,
            activations_remaining=self.activations_remaining,
            seconds_used=self._seconds_used + runtime,
            seconds_remaining=max(
                0.0, config.BOOST_MAX_S - (self._seconds_used + runtime)
            ),
            current_runtime_s=runtime,
        )
