"""
Protocole de messages BattleBoats 2026 (cf. BATTLEBOATS_LORA_PROTOCOL_v5.pdf).

Format ASCII pipe-délimité — c'est la version officielle qui sera arbitrée.
NB : la mention TYPE_WIND=0x03 du brief initial évoquait un format binaire ;
     ce format n'est PAS celui adopté par l'organisation. On suit le PDF v5.

Messages :
    W|<dir_deg>|<spd_ms×10>|<unix_ts>            — vent (toutes les 60s)
    P|<id>|<lat×1e5>|<lon×1e5>|<hdg>|<spd_kn×10>  — position (60s par bateau)

Identifiants UTT : U1B1, U1B2 (régate à 2 drones — cf. config.TEAM_BOATS).
"""

import logging
import time
from dataclasses import dataclass
from typing import Optional, Union

import config

log = logging.getLogger(__name__)


# ════════════════════════════════════════════════════════════════════════
# WIND
# ════════════════════════════════════════════════════════════════════════
@dataclass
class WindMessage:
    direction_deg: int        # 0-359° — direction d'OÙ vient le vent (météo)
    speed_ms: float           # m/s
    timestamp: int            # Unix epoch
    sensor_offline: bool      # vrai si dir=0 ET speed=0

    @classmethod
    def parse(cls, text: str) -> Optional["WindMessage"]:
        parts = text.strip().split("|")
        if len(parts) != 4 or parts[0] != "W":
            return None
        try:
            d = int(parts[1])
            s_x10 = int(parts[2])
            ts = int(parts[3])
        except ValueError:
            return None
        return cls(
            direction_deg=d % 360,
            speed_ms=s_x10 / 10.0,
            timestamp=ts,
            sensor_offline=(d == 0 and s_x10 == 0),
        )

    def encode(self) -> str:
        s_x10 = int(round(self.speed_ms * 10))
        return f"W|{self.direction_deg:03d}|{s_x10:02d}|{self.timestamp}"


# ════════════════════════════════════════════════════════════════════════
# POSITION
# ════════════════════════════════════════════════════════════════════════
@dataclass
class PositionMessage:
    boat_id: str
    lat: float                # degrés décimaux
    lon: float                # degrés décimaux
    heading_deg: int          # 0-359°
    speed_knots: float        # nœuds
    no_fix: bool              # vrai si lat=0 ET lon=0

    @classmethod
    def parse(cls, text: str) -> Optional["PositionMessage"]:
        parts = text.strip().split("|")
        if len(parts) != 6 or parts[0] != "P":
            return None
        try:
            bid = parts[1]
            lat_x = int(parts[2])
            lon_x = int(parts[3])
            hdg = int(parts[4])
            spd_x10 = int(parts[5])
        except ValueError:
            return None
        return cls(
            boat_id=bid,
            lat=lat_x / 1e5,
            lon=lon_x / 1e5,
            heading_deg=hdg % 360,
            speed_knots=spd_x10 / 10.0,
            no_fix=(lat_x == 0 and lon_x == 0),
        )

    def encode(self) -> str:
        if self.no_fix:
            return f"P|{self.boat_id}|0|0|0|0"
        lat_x = int(round(self.lat * 1e5))
        lon_x = int(round(self.lon * 1e5))
        # Le PDF spécifie lat int32 ×1e5 — peut être négatif
        spd_x10 = int(round(self.speed_knots * 10))
        return (
            f"P|{self.boat_id}|{lat_x}|{lon_x}|"
            f"{self.heading_deg % 360:03d}|{spd_x10:02d}"
        )


def parse_message(text: str) -> Optional[Union[WindMessage, PositionMessage]]:
    """Routeur — détermine le type de message à partir du préfixe."""
    if not text:
        return None
    t = text.strip()
    if t.startswith("W|"):
        return WindMessage.parse(t)
    if t.startswith("P|"):
        return PositionMessage.parse(t)
    return None


def build_position_from_telemetry(boat_id: str, lat: float, lon: float,
                                  heading_deg: float,
                                  ground_speed_ms: float,
                                  has_fix: bool) -> PositionMessage:
    """Helper : construit une PositionMessage à partir des données MAVLink."""
    if not has_fix:
        return PositionMessage(boat_id, 0.0, 0.0, 0, 0.0, no_fix=True)
    # Conversion m/s → nœuds (1 m/s = 1.94384 kn)
    spd_kn = ground_speed_ms * 1.94384
    return PositionMessage(
        boat_id=boat_id,
        lat=lat,
        lon=lon,
        heading_deg=int(round(heading_deg)) % 360,
        speed_knots=spd_kn,
        no_fix=False,
    )


def now_unix() -> int:
    return int(time.time())
