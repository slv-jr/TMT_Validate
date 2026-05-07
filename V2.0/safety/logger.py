"""
Logger structuré : trace l'état complet du système chaque tick (10 Hz).

Sortie : fichier CSV horodaté dans config.LOG_DIR. Un fichier par session
(création au lancement, fermeture à l'arrêt). Format CSV pour analyse
post-course dans Excel/Pandas.

⚠️ La microSD recommandée est de 32 Go. À 10 Hz et ~30 colonnes
numériques, on génère ~5 Mo / heure → largement dans les marges.
"""

import csv
import logging
import os
import time
from datetime import datetime
from typing import Any, Dict, Optional, TextIO

import config

log = logging.getLogger(__name__)


class FlightLogger:
    """Écrit un CSV avec un en-tête fixe + une ligne par tick."""

    HEADER = [
        "timestamp_unix",
        "elapsed_s",
        "drone_id",
        "nav_state",
        "control_mode",
        "role",
        "leg_index",
        "leg_name",
        "lat",
        "lon",
        "east_m",
        "north_m",
        "fix_type",
        "rtk_fixed",
        "heading_deg",
        "speed_ms",
        "roll_deg",
        "wp_distance_m",
        "wp_bearing_deg",
        "target_heading_deg",
        "rudder_cmd_deg",
        "rudder_pwm",
        "sail_pwm",
        "sail_pct",
        "wind_dir_deg",
        "wind_speed_ms",
        "wind_age_s",
        "wind_source",
        "battery_pct",
        "penalty_mode",
        "penalty_progress",
        "neighbors_count",
        "degraded_modes",
    ]

    def __init__(self, log_dir: Optional[str] = None):
        self.log_dir = log_dir or config.LOG_DIR
        self._file: Optional[TextIO] = None
        self._writer = None
        self._start_t: float = 0.0
        self._tick_count: int = 0
        self._path: Optional[str] = None

    def open(self) -> str:
        os.makedirs(self.log_dir, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = f"flight_{config.DRONE_ID}_{ts}.csv"
        path = os.path.join(self.log_dir, fname)
        self._file = open(path, "w", newline="", buffering=1)
        self._writer = csv.DictWriter(self._file, fieldnames=self.HEADER,
                                      extrasaction="ignore")
        self._writer.writeheader()
        self._start_t = time.monotonic()
        self._path = path
        log.info("[LOGGER] Ouverture %s", path)
        return path

    def close(self):
        if self._file is not None:
            log.info("[LOGGER] Fermeture %s (%d lignes)",
                     self._path, self._tick_count)
            try:
                self._file.flush()
                self._file.close()
            except Exception as e:
                log.warning("[LOGGER] Erreur fermeture : %s", e)
            self._file = None
            self._writer = None

    def log(self, row: Dict[str, Any]):
        if self._writer is None:
            return
        try:
            row_filled = dict(row)
            row_filled.setdefault("timestamp_unix", time.time())
            row_filled.setdefault("elapsed_s",
                                  time.monotonic() - self._start_t)
            row_filled.setdefault("drone_id", config.DRONE_ID)
            self._writer.writerow(row_filled)
            self._tick_count += 1
            # Flush périodique
            if self._tick_count % 100 == 0 and self._file is not None:
                self._file.flush()
        except Exception as e:
            log.warning("[LOGGER] Erreur écriture : %s", e)

    @property
    def path(self) -> Optional[str]:
        return self._path
