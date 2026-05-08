"""
Estimation du vent réel pour la stratégie de navigation.

DEUX MODES (cf. config.MODE) :

    MODE = REGATE (course officielle) :
        - Source PRINCIPALE : nœud WIND de l'organisateur via LoRa
          (W|<dir>|<spd×10>|<unix_ts> toutes les 60 s, cf. PDF protocole v5)
        - Fallback STATIQUE si > WIND_FALLBACK_TIMEOUT_S sans message orga :
          on utilise WIND_FALLBACK_DIR_DEG / WIND_FALLBACK_SPEED_MS définis
          dans config.py (à régler au briefing du matin selon la météo).

    MODE = ESSAI (test à sec ou en mer hors course) :
        - Le vent est FIXÉ par les variables d'env WIND_DIR_DEG / WIND_SPEED_MS
          (cf. config.WIND_ESSAI_DIR_DEG / config.WIND_ESSAI_SPEED_MS).
        - Aucune écoute orga (le filtre LoRa rejette les W|... orga).
        - Permet de tester la stratégie de nav avec un vent contrôlé.

Sortie unifiée : WindEstimate(direction_deg, speed_ms, age_s, source, confident).

Convention direction : 0° = vent du Nord (météo), 90° = E, 180° = S, 270° = O.
"""

import logging
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, Tuple

import config

log = logging.getLogger(__name__)


@dataclass
class WindEstimate:
    direction_deg: float       # 0 = vent du Nord (convention météo)
    speed_ms: float
    age_s: float               # ancienneté de la mesure
    source: str                # "ORGA" | "STALE_ORGA" | "ESSAI" | "FALLBACK" | "DEFAULT"
    confident: bool            # True si la mesure est récente et non offline


class WindEstimator:
    """Maintient une estimation du vent à partir des messages WIND LoRa.

    En mode ESSAI, retourne directement le vent simulé (constant).
    En mode REGATE, écoute le nœud WIND orga (push_orga_wind) et
    fallback sur WIND_FALLBACK_* si silence > WIND_FALLBACK_TIMEOUT_S.
    """

    HISTORY_LEN = 5            # nombre de mesures pour le lissage

    def __init__(self):
        self._history: Deque[Tuple[float, float, float]] = deque(maxlen=self.HISTORY_LEN)
        self._last_received_t: float = 0.0
        self._last_offline: bool = False

    # ════════════════════════════════════════════════════════════════════
    # Push depuis la couche LoRa
    # ════════════════════════════════════════════════════════════════════
    def push_orga_wind(self, direction_deg: int, speed_ms: float,
                       timestamp: int, sensor_offline: bool):
        """À appeler depuis la callback LoRa quand un W|... orga arrive.

        En mode ESSAI : ignoré silencieusement (la source est WIND_ESSAI_*).
        En mode REGATE : ajouté à l'historique pour lissage.
        """
        if config.is_essai():
            log.debug("[WIND] message orga ignoré (mode ESSAI)")
            return
        now = time.monotonic()
        self._last_received_t = now
        self._last_offline = sensor_offline
        if not sensor_offline and speed_ms > 0:
            self._history.append((float(direction_deg), float(speed_ms), now))
            log.debug("[WIND] orga dir=%.0f spd=%.2f ajouté",
                      direction_deg, speed_ms)

    # Alias rétrocompat pour le main et autres modules existants
    def push_calypso(self, direction_deg: int, speed_ms: float,
                     timestamp: int, sensor_offline: bool):
        """Alias rétrocompat — l'ancien nom 'Calypso' reste accepté."""
        self.push_orga_wind(direction_deg, speed_ms, timestamp, sensor_offline)

    # ════════════════════════════════════════════════════════════════════
    # Lissage interne
    # ════════════════════════════════════════════════════════════════════
    def _lissage(self) -> Optional[Tuple[float, float]]:
        """Moyenne pondérée des dernières mesures (vecteur unitaire)."""
        if not self._history:
            return None
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

    # ════════════════════════════════════════════════════════════════════
    # API publique : estimate()
    # ════════════════════════════════════════════════════════════════════
    def estimate(self) -> WindEstimate:
        """Retourne l'estimation courante du vent réel.

        En mode ESSAI : valeur fixée par config.WIND_ESSAI_*.
        En mode REGATE : dernière donnée orga lissée, ou fallback statique
        si pas de message orga reçu depuis WIND_FALLBACK_TIMEOUT_S.
        """
        # ─── Mode ESSAI : vent simulé fixe ─────────────────────────────
        if config.is_essai():
            return WindEstimate(
                direction_deg=config.WIND_ESSAI_DIR_DEG,
                speed_ms=config.WIND_ESSAI_SPEED_MS,
                age_s=0.0,
                source="ESSAI",
                confident=True,
            )

        # ─── Mode REGATE : écoute orga + fallback ──────────────────────
        now = time.monotonic()
        age = now - self._last_received_t if self._last_received_t > 0 else 1e9

        # Cas 1 : aucun message reçu depuis le démarrage → fallback statique
        if self._last_received_t == 0.0:
            return WindEstimate(
                direction_deg=config.WIND_FALLBACK_DIR_DEG,
                speed_ms=config.WIND_FALLBACK_SPEED_MS,
                age_s=age,
                source="FALLBACK",
                confident=False,
            )

        # Cas 2 : capteur orga déclaré offline (W|000|00|ts)
        if self._last_offline:
            data = self._lissage()
            if data is not None:
                d, s = data
                return WindEstimate(
                    direction_deg=d, speed_ms=s, age_s=age,
                    source="STALE_ORGA", confident=False,
                )
            return WindEstimate(
                direction_deg=config.WIND_FALLBACK_DIR_DEG,
                speed_ms=config.WIND_FALLBACK_SPEED_MS,
                age_s=age, source="FALLBACK", confident=False,
            )

        # Cas 3 : silence orga prolongé → fallback (mais on garde l'historique
        # comme estimation alternative si dispo)
        if age > config.WIND_FALLBACK_TIMEOUT_S:
            data = self._lissage()
            if data is not None:
                d, s = data
                return WindEstimate(
                    direction_deg=d, speed_ms=s, age_s=age,
                    source="STALE_ORGA", confident=False,
                )
            return WindEstimate(
                direction_deg=config.WIND_FALLBACK_DIR_DEG,
                speed_ms=config.WIND_FALLBACK_SPEED_MS,
                age_s=age, source="FALLBACK", confident=False,
            )

        # Cas 4 : nominal — orga récent et online
        data = self._lissage()
        if data is None:
            return WindEstimate(
                direction_deg=config.WIND_FALLBACK_DIR_DEG,
                speed_ms=config.WIND_FALLBACK_SPEED_MS,
                age_s=age, source="FALLBACK", confident=False,
            )
        d, s = data
        return WindEstimate(
            direction_deg=d, speed_ms=s, age_s=age,
            source="ORGA", confident=True,
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
