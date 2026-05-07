"""
Interface MAVLink avec le Cube Orange+ (ArduRover 4.6.3).

Stratégie validée par le PoC SwarmZ :
    - Mode permanent MANUAL (le Cube reste en passthrough RC)
    - Le Pi prend le contrôle en envoyant RC_CHANNELS_OVERRIDE sur CH1
      (safran) et CH2 (voile), envoi continu à 10 Hz pour éviter le timeout.
    - SERVO1_FUNCTION = 1 (RCPassThru) — OBLIGATOIRE
    - SERVO2_FUNCTION = 89 (MainSail)
    - La bascule manuel/auto se fait par la position physique du levier
      CH3 sur la radio J4C05, lue via RC_CHANNELS.

Les paramètres SERVO_FUNCTION=0 du brief initial étaient une erreur :
0 désactiverait le servo. Voir docs/ARDUPILOT_PARAMS.md.
"""

import logging
import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

import config

try:
    from pymavlink import mavutil
    HAS_MAVLINK = True
except ImportError:
    HAS_MAVLINK = False
    mavutil = None  # type: ignore

log = logging.getLogger(__name__)

IGNORE_PWM = 65535     # valeur spéciale RC_CHANNELS_OVERRIDE = "ne pas overrider"
TARGET_SYSID = 1
TARGET_COMP = 1
SOURCE_SYSID = 255     # GCS standard


@dataclass
class TelemetrySnapshot:
    """État instantané du bateau, mis à jour en continu par le thread RX."""
    lat: float = 0.0                  # degrés décimaux
    lon: float = 0.0                  # degrés décimaux
    has_gps_fix: bool = False
    fix_type: int = 0                 # 0/1=no fix, 3=3D, 6=RTK fix
    sats_visible: int = 0
    heading_deg: float = 0.0          # cap magnétique (degrés)
    ground_speed_ms: float = 0.0      # m/s
    cog_deg: float = 0.0              # course over ground
    roll_deg: float = 0.0             # gîte
    pitch_deg: float = 0.0
    voltage_v: float = 0.0
    battery_remaining_pct: float = 100.0
    rc_channels: dict = field(default_factory=dict)   # {1: 1500, 2: 1500, 3: 1900, ...}
    last_heartbeat_t: float = 0.0
    last_gps_t: float = 0.0
    armed: bool = False
    mode: str = "UNKNOWN"


class MavlinkInterface:
    """Wrapper synchrone autour de pymavlink pour la boucle principale."""

    def __init__(self,
                 port: str = config.MAVLINK_PORT,
                 baud: int = config.MAVLINK_BAUD):
        if not HAS_MAVLINK:
            raise RuntimeError(
                "pymavlink non installé. Run: pip install pymavlink --break-system-packages"
            )
        self.port = port
        self.baud = baud
        self.master = None
        self.telemetry = TelemetrySnapshot()

        # Lock pour protéger l'accès à self.telemetry depuis le thread RX
        self._lock = threading.Lock()
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_running = False

        # Override actuel à émettre
        self._override_rudder_pwm = IGNORE_PWM
        self._override_sail_pwm = IGNORE_PWM
        self._override_aux_pwms: dict = {}    # ex: {4: 1900} pour boost

    # ──────────────────────────────────────────────
    # Connexion
    # ──────────────────────────────────────────────
    def connect(self, timeout_s: float = 30.0) -> bool:
        log.info("[MAV] Connexion %s @ %d…", self.port, self.baud)
        self.master = mavutil.mavlink_connection(
            self.port, baud=self.baud, source_system=SOURCE_SYSID,
        )
        log.info("[MAV] Attente heartbeat (timeout=%.0fs)…", timeout_s)
        hb = self.master.wait_heartbeat(timeout=timeout_s)
        if hb is None:
            log.error("[MAV] Pas de heartbeat reçu")
            return False
        log.info(
            "[MAV] Connecté — sysid=%d compid=%d type=%d",
            self.master.target_system,
            self.master.target_component,
            hb.type,
        )

        # Demander streams utiles
        self._request_streams()

        # Démarrer thread RX
        self._rx_running = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="MavlinkRX", daemon=True,
        )
        self._rx_thread.start()
        return True

    def _request_streams(self):
        """Demande les streams utiles au Cube."""
        m = self.master
        # GPS, position globale, attitude
        m.mav.request_data_stream_send(
            TARGET_SYSID, TARGET_COMP,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 5, 1,
        )
        m.mav.request_data_stream_send(
            TARGET_SYSID, TARGET_COMP,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1,    # ATTITUDE
        )
        m.mav.request_data_stream_send(
            TARGET_SYSID, TARGET_COMP,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 5, 1,     # VFR_HUD
        )
        m.mav.request_data_stream_send(
            TARGET_SYSID, TARGET_COMP,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 10, 1,
        )
        m.mav.request_data_stream_send(
            TARGET_SYSID, TARGET_COMP,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1,
        )

    def close(self):
        self._rx_running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=1.0)
        # Libérer tous les overrides avant de partir
        try:
            self.clear_all_overrides()
        except Exception:
            pass
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

    # ──────────────────────────────────────────────
    # Thread de réception
    # ──────────────────────────────────────────────
    def _rx_loop(self):
        while self._rx_running and self.master is not None:
            try:
                msg = self.master.recv_match(blocking=True, timeout=0.5)
            except Exception as e:
                log.warning("[MAV-RX] Erreur recv: %s", e)
                time.sleep(0.5)
                continue
            if msg is None:
                continue
            mtype = msg.get_type()
            now = time.monotonic()

            if mtype == "BAD_DATA":
                continue

            with self._lock:
                if mtype == "HEARTBEAT":
                    self.telemetry.last_heartbeat_t = now
                    self.telemetry.armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    # Décodage du mode (custom_mode = numéro de mode ArduPilot)
                    self.telemetry.mode = mavutil.mode_string_v10(msg)

                elif mtype == "GLOBAL_POSITION_INT":
                    self.telemetry.lat = msg.lat / 1e7
                    self.telemetry.lon = msg.lon / 1e7
                    self.telemetry.heading_deg = msg.hdg / 100.0 if msg.hdg != 65535 else 0.0
                    vx = msg.vx / 100.0    # cm/s → m/s
                    vy = msg.vy / 100.0
                    self.telemetry.ground_speed_ms = math.hypot(vx, vy)
                    self.telemetry.last_gps_t = now

                elif mtype == "GPS_RAW_INT":
                    self.telemetry.fix_type = msg.fix_type
                    self.telemetry.has_gps_fix = msg.fix_type >= 3
                    self.telemetry.sats_visible = msg.satellites_visible
                    if msg.cog != 65535:
                        self.telemetry.cog_deg = msg.cog / 100.0

                elif mtype == "ATTITUDE":
                    self.telemetry.roll_deg = math.degrees(msg.roll)
                    self.telemetry.pitch_deg = math.degrees(msg.pitch)

                elif mtype == "VFR_HUD":
                    if msg.groundspeed > 0:
                        self.telemetry.ground_speed_ms = msg.groundspeed

                elif mtype == "RC_CHANNELS":
                    for i in range(1, 9):
                        self.telemetry.rc_channels[i] = getattr(
                            msg, f"chan{i}_raw", 0
                        )

                elif mtype == "SYS_STATUS":
                    self.telemetry.voltage_v = msg.voltage_battery / 1000.0
                    if msg.battery_remaining >= 0:
                        self.telemetry.battery_remaining_pct = float(msg.battery_remaining)

    # ──────────────────────────────────────────────
    # Lecture (côté boucle navigation)
    # ──────────────────────────────────────────────
    def get_telemetry(self) -> TelemetrySnapshot:
        with self._lock:
            # On retourne une copie pour éviter les races
            t = TelemetrySnapshot(**self.telemetry.__dict__)
            t.rc_channels = dict(self.telemetry.rc_channels)
            return t

    def is_alive(self) -> bool:
        with self._lock:
            return (
                self.master is not None
                and (time.monotonic() - self.telemetry.last_heartbeat_t)
                < config.HEARTBEAT_TIMEOUT_S
            )

    # ──────────────────────────────────────────────
    # Heartbeat GCS (anti-failsafe)
    # ──────────────────────────────────────────────
    def send_heartbeat(self):
        if self.master is None:
            return
        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception as e:
            log.debug("[MAV] heartbeat send err: %s", e)

    # ──────────────────────────────────────────────
    # Commandes (override servos)
    # ──────────────────────────────────────────────
    def set_rudder_pwm(self, pwm_us: int):
        """Définit la consigne safran (CH1)."""
        self._override_rudder_pwm = max(
            config.RUDDER_PWM_MIN, min(config.RUDDER_PWM_MAX, pwm_us)
        )

    def set_sail_pwm(self, pwm_us: int):
        """Définit la consigne voile (CH2, winch)."""
        self._override_sail_pwm = max(
            config.SAIL_PWM_MIN, min(config.SAIL_PWM_MAX, pwm_us)
        )

    def set_rudder_angle_deg(self, angle_deg: float):
        """Convertit un angle de safran en PWM (limité par RUDDER_ANGLE_MAX_DEG)."""
        a = max(-config.RUDDER_ANGLE_MAX_DEG,
                min(config.RUDDER_ANGLE_MAX_DEG, angle_deg))
        # +angle = barre à droite → PWM > trim (à valider par calibration)
        ratio = a / config.RUDDER_ANGLE_MAX_DEG
        pwm = config.RUDDER_PWM_TRIM + int(
            ratio * (config.RUDDER_PWM_MAX - config.RUDDER_PWM_TRIM)
        )
        self.set_rudder_pwm(pwm)

    def set_sail_percent(self, percent: float):
        """0% = bordée (PWM_MIN), 100% = choquée (PWM_MAX)."""
        p = max(0.0, min(100.0, percent))
        pwm = int(config.SAIL_PWM_MIN
                  + p / 100.0 * (config.SAIL_PWM_MAX - config.SAIL_PWM_MIN))
        self.set_sail_pwm(pwm)

    def set_aux_pwm(self, channel: int, pwm_us: int):
        """Définit un canal auxiliaire (ex: CH4 pour le boost)."""
        self._override_aux_pwms[channel] = pwm_us

    def clear_aux_pwm(self, channel: int):
        if channel in self._override_aux_pwms:
            del self._override_aux_pwms[channel]

    def push_overrides(self):
        """Envoie effectivement les overrides au Cube. Doit être appelé à
        OVERRIDE_REFRESH_HZ pour éviter le timeout d'override (~0.5s).

        Format : RC_CHANNELS_OVERRIDE prend 8 canaux PWM. On passe IGNORE_PWM
        pour ne pas toucher aux canaux non gérés (ex: CH3 mode reste lu du RC).
        """
        if self.master is None:
            return
        chans = [IGNORE_PWM] * 8
        chans[0] = self._override_rudder_pwm   # CH1
        chans[1] = self._override_sail_pwm     # CH2
        # CH3 : NE JAMAIS overrider — c'est le levier de mode lu depuis la RC
        for ch, pwm in self._override_aux_pwms.items():
            if 1 <= ch <= 8 and ch != 3:
                chans[ch - 1] = pwm
        try:
            self.master.mav.rc_channels_override_send(
                TARGET_SYSID, TARGET_COMP, *chans,
            )
        except Exception as e:
            log.warning("[MAV] override send err: %s", e)

    def clear_all_overrides(self):
        """Libère tous les overrides → la RC reprend physiquement la main."""
        self._override_rudder_pwm = IGNORE_PWM
        self._override_sail_pwm = IGNORE_PWM
        self._override_aux_pwms.clear()
        if self.master is None:
            return
        try:
            self.master.mav.rc_channels_override_send(
                TARGET_SYSID, TARGET_COMP, *([IGNORE_PWM] * 8),
            )
        except Exception as e:
            log.warning("[MAV] clear override err: %s", e)
