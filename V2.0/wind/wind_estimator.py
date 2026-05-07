"""
Estimation du vent réel pour la stratégie de navigation.

Sources, par priorité :
    1) Station Calypso (à terre) reçue par LoRa (W|... toutes les 60s).
       C'est la SOURCE PRINCIPALE — vent réel non perturbé par le mouvement.
    2) Fallback "triangle des vents" : si on connaît le vent apparent (via
       les performances bateau et le cap), on peut estimer le vent réel.
       En l'absence de girouette embarquée (cf. brief utilisateur), ce
       fallback est dégradé et utilise une moyenne des dernières valeurs
       Calypso reçues.

Sortie unifiée : WindEstimate(direction_deg, speed_ms, age_s, source).
"""

import logging
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import config

log = logging.getLogger(__name__)


@dataclass
class WindEstimate:
    direction_deg: float       # 0 = vent du Nord (convention météo)
    speed_ms: float
    age_s: float               # ancienneté de la mesure
    source: str                # "CALYPSO" | "STALE_CALYPSO" | "TRIANGLE" | "DEFAULT"
    confident: bool            # True si la mesure est récente et non offline


class WindEstimator:
    """Maintient une estimation du vent à partir des messages WIND LoRa."""

    DEFAULT_DIR_DEG = 90.0     # E (brise thermique de mai)
    DEFAULT_SPEED_MS = 4.5     # 9 nœuds (cf. dossier technique)
    HISTORY_LEN = 5            # nombre de mesures Calypso pour lisser

    def __init__(self):
        self._history: Deque[tuple] = deque(maxlen=self.HISTORY_LEN)
        self._last_received_t: float = 0.0
        self._last_offline: bool = False

    def push_calypso(self, direction_deg: int, speed_ms: float,
                     timestamp: int, sensor_offline: bool):
        """À appeler depuis la callback LoRa quand un W|... arrive."""
        now = time.monotonic()
        self._last_received_t = now
        self._last_offline = sensor_offline
        if not sensor_offline and speed_ms > 0:
            self._history.append((float(direction_deg), float(speed_ms), now))
            log.debug("[WIND] Calypso dir=%.0f spd=%.2f ajouté",
                      direction_deg, speed_ms)

    def _lissage(self) -> Optional[tuple]:
        """Moyenne pondérée des dernières mesures (vecteur unitaire)."""
        if not self._history:
            return None
        import math
        sx = sy = 0.0
        total_speed = 0.0
        for d, s, _ in self._history:
            rad = math.radians(d)
            sx += s * math.sin(rad)
            sy += s * math.cos(rad)
            total_speed += s
        n = len(self._history)
        avg_speed = total_speed / n
        if abs(sx) < 1e-6 and abs(sy) < 1e-6:
            return self._history[-1][0], avg_speed
        avg_dir = (math.degrees(math.atan2(sx, sy)) + 360.0) % 360.0
        return avg_dir, avg_speed

    def estimate(self) -> WindEstimate:
        now = time.monotonic()
        age = now - self._last_received_t if self._last_received_t > 0 else 1e9

        if self._last_received_t == 0.0:
            return WindEstimate(
                direction_deg=self.DEFAULT_DIR_DEG,
                speed_ms=self.DEFAULT_SPEED_MS,
                age_s=age,
                source="DEFAULT",
                confident=False,
            )

        if self._last_offline:
            # Capteur signalé offline → utiliser l'historique récent
            data = self._lissage()
            if data is not None:
                d, s = data
                return WindEstimate(
                    direction_deg=d, speed_ms=s, age_s=age,
                    source="STALE_CALYPSO", confident=False,
                )
            return WindEstimate(
                direction_deg=self.DEFAULT_DIR_DEG,
                speed_ms=self.DEFAULT_SPEED_MS,
                age_s=age, source="DEFAULT", confident=False,
            )

        if age > config.WIND_FALLBACK_TIMEOUT_S:
            # Plus de Calypso depuis trop longtemps → mode dégradé
            data = self._lissage()
            if data is not None:
                d, s = data
                return WindEstimate(
                    direction_deg=d, speed_ms=s, age_s=age,
                    source="STALE_CALYPSO", confident=False,
                )
            return WindEstimate(
                direction_deg=self.DEFAULT_DIR_DEG,
                speed_ms=self.DEFAULT_SPEED_MS,
                age_s=age, source="DEFAULT", confident=False,
            )

        # Cas nominal — Calypso récent et online
        data = self._lissage()
        if data is None:
            return WindEstimate(
                direction_deg=self.DEFAULT_DIR_DEG,
                speed_ms=self.DEFAULT_SPEED_MS,
                age_s=age, source="DEFAULT", confident=False,
            )
        d, s = data
        return WindEstimate(
            direction_deg=d, speed_ms=s, age_s=age,
            source="CALYPSO", confident=True,
        )

    def estimate_from_triangle(self,
                               apparent_dir_deg: float,
                               apparent_speed_ms: float,
                               heading_deg: float,
                               boat_speed_ms: float) -> WindEstimate:
        """Estimation du vent réel depuis le vent apparent et la vitesse bateau.

        Réservé au cas où une girouette embarquée serait ajoutée plus tard.
        Sans girouette (cas brief), ce module n'est pas appelé.
        """
        from navigation import vmg
        true_dir, true_speed = vmg.apparent_to_true_wind(
            apparent_dir_deg, apparent_speed_ms, heading_deg, boat_speed_ms,
        )
        return WindEstimate(
            direction_deg=true_dir,
            speed_ms=true_speed,
            age_s=0.0,
            source="TRIANGLE",
            confident=True,
        )
