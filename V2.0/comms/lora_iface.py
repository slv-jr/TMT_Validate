"""
Interface avec l'ESP32 LoRa V3 (Meshtastic) via port série USB.

L'ESP32 doit avoir été flashé Meshtastic et configuré par l'organisation
sur le canal BATTLEBOATS (cf. BATTLEBOATS_LORA_PROTOCOL_v5.pdf §1).

On utilise la lib `meshtastic` Python qui parle au firmware via le port
série. Elle expose :
    - sendText(text)                    pour broadcaster un message
    - on_receive(packet) callback       pour recevoir
    - on_connection callback            pour suivre l'état de la liaison

Ce module wrappe ces APIs pour exposer une interface simple à la boucle
principale, avec callbacks pour PositionMessage et WindMessage.
"""

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, Dict, Optional, Tuple

from . import protocol
import config

log = logging.getLogger(__name__)

try:
    import meshtastic
    import meshtastic.serial_interface
    from pubsub import pub
    HAS_MESHTASTIC = True
except ImportError:
    HAS_MESHTASTIC = False


@dataclass
class NeighborState:
    """Dernière position connue d'un autre bateau du réseau."""
    boat_id: str
    lat: float = 0.0
    lon: float = 0.0
    heading_deg: int = 0
    speed_knots: float = 0.0
    last_seen_t: float = 0.0
    has_fix: bool = False

    def is_stale(self, timeout_s: float = config.LORA_NEIGHBOR_TIMEOUT_S) -> bool:
        return (time.monotonic() - self.last_seen_t) > timeout_s


@dataclass
class WindReading:
    direction_deg: int = 0
    speed_ms: float = 0.0
    timestamp: int = 0
    last_received_t: float = 0.0
    sensor_offline: bool = True

    def is_stale(self, timeout_s: float = config.WIND_FALLBACK_TIMEOUT_S) -> bool:
        return (time.monotonic() - self.last_received_t) > timeout_s


class LoRaInterface:
    """Wrapper Meshtastic pour le réseau BattleBoats."""

    def __init__(self,
                 port: str = config.LORA_PORT,
                 boat_id: str = config.DRONE_ID):
        if not HAS_MESHTASTIC:
            raise RuntimeError(
                "meshtastic non installé. Run: pip install meshtastic pypubsub"
            )
        self.port = port
        self.boat_id = boat_id
        self.iface = None
        self._connected = False

        # États
        self.neighbors: Dict[str, NeighborState] = {}
        self.wind: WindReading = WindReading()

        # Callbacks externes (optionnels)
        self.on_position: Optional[Callable[[protocol.PositionMessage], None]] = None
        self.on_wind: Optional[Callable[[protocol.WindMessage], None]] = None

        # File d'envoi (transmissions limitées par le slot TDMA)
        self._tx_queue: Deque[str] = deque()
        self._lock = threading.Lock()
        self._last_tx_t: float = 0.0

    # ─────────────────────────────────────────────
    # Connexion
    # ─────────────────────────────────────────────
    def connect(self) -> bool:
        log.info("[LoRa] Ouverture %s…", self.port)
        try:
            self.iface = meshtastic.serial_interface.SerialInterface(
                devPath=self.port,
            )
        except Exception as e:
            log.error("[LoRa] Échec ouverture %s : %s", self.port, e)
            return False
        # Hooks pubsub
        pub.subscribe(self._on_receive, "meshtastic.receive")
        pub.subscribe(self._on_connection, "meshtastic.connection.established")
        pub.subscribe(self._on_disconnect, "meshtastic.connection.lost")
        # Vérifier le canal
        time.sleep(1.0)
        node_info = self.iface.getMyNodeInfo()
        log.info("[LoRa] Connecté — node=%s", node_info.get("user", {}).get("shortName", "?"))
        self._connected = True
        return True

    def close(self):
        if self.iface is not None:
            try:
                self.iface.close()
            except Exception:
                pass
        self.iface = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    # ─────────────────────────────────────────────
    # Callbacks Meshtastic
    # ─────────────────────────────────────────────
    def _on_connection(self, interface, topic=None):
        log.info("[LoRa] Connexion établie")
        self._connected = True

    def _on_disconnect(self, interface, topic=None):
        log.warning("[LoRa] Connexion perdue")
        self._connected = False

    def _on_receive(self, packet, interface):
        """Callback Meshtastic. packet est un dict.
        Pour les messages texte : packet['decoded']['text'].
        """
        try:
            decoded = packet.get("decoded", {})
            if decoded.get("portnum") != "TEXT_MESSAGE_APP":
                return
            text = decoded.get("text", "")
            if not text:
                return
            self._handle_text(text)
        except Exception as e:
            log.debug("[LoRa-RX] Erreur parsing : %s — packet=%s", e, packet)

    def _handle_text(self, text: str):
        msg = protocol.parse_message(text)
        if msg is None:
            log.debug("[LoRa-RX] Message inconnu: %s", text[:80])
            return

        now = time.monotonic()
        if isinstance(msg, protocol.PositionMessage):
            # On filtre nos propres broadcasts (au cas où Meshtastic les renvoie)
            if msg.boat_id == self.boat_id:
                return
            with self._lock:
                ns = self.neighbors.get(msg.boat_id)
                if ns is None:
                    ns = NeighborState(boat_id=msg.boat_id)
                    self.neighbors[msg.boat_id] = ns
                ns.lat = msg.lat
                ns.lon = msg.lon
                ns.heading_deg = msg.heading_deg
                ns.speed_knots = msg.speed_knots
                ns.has_fix = not msg.no_fix
                ns.last_seen_t = now
            log.debug("[LoRa-RX] POS %s → %.5f,%.5f hdg=%d spd=%.1fkn",
                      msg.boat_id, msg.lat, msg.lon,
                      msg.heading_deg, msg.speed_knots)
            if self.on_position:
                try:
                    self.on_position(msg)
                except Exception as e:
                    log.warning("[LoRa] on_position cb err : %s", e)

        elif isinstance(msg, protocol.WindMessage):
            with self._lock:
                self.wind.direction_deg = msg.direction_deg
                self.wind.speed_ms = msg.speed_ms
                self.wind.timestamp = msg.timestamp
                self.wind.last_received_t = now
                self.wind.sensor_offline = msg.sensor_offline
            log.info("[LoRa-RX] WIND dir=%d° spd=%.1fm/s offline=%s",
                     msg.direction_deg, msg.speed_ms, msg.sensor_offline)
            if self.on_wind:
                try:
                    self.on_wind(msg)
                except Exception as e:
                    log.warning("[LoRa] on_wind cb err : %s", e)

    # ─────────────────────────────────────────────
    # Envoi
    # ─────────────────────────────────────────────
    def send_text(self, text: str) -> bool:
        if not self._connected or self.iface is None:
            log.warning("[LoRa-TX] Non connecté — message rejeté: %s", text[:60])
            return False
        try:
            self.iface.sendText(text)
            self._last_tx_t = time.monotonic()
            log.debug("[LoRa-TX] %s", text[:80])
            return True
        except Exception as e:
            log.warning("[LoRa-TX] Erreur d'envoi : %s", e)
            return False

    def broadcast_position(self, lat: float, lon: float, heading_deg: float,
                           speed_ms: float, has_fix: bool) -> bool:
        msg = protocol.build_position_from_telemetry(
            self.boat_id, lat, lon, heading_deg, speed_ms, has_fix,
        )
        return self.send_text(msg.encode())

    # ─────────────────────────────────────────────
    # Lectures pour la boucle principale
    # ─────────────────────────────────────────────
    def get_wind_snapshot(self) -> WindReading:
        with self._lock:
            return WindReading(
                direction_deg=self.wind.direction_deg,
                speed_ms=self.wind.speed_ms,
                timestamp=self.wind.timestamp,
                last_received_t=self.wind.last_received_t,
                sensor_offline=self.wind.sensor_offline,
            )

    def get_neighbors(self) -> Dict[str, NeighborState]:
        with self._lock:
            return {bid: NeighborState(**ns.__dict__) for bid, ns in self.neighbors.items()}

    def get_active_neighbors(self) -> Dict[str, NeighborState]:
        """Voisins ayant émis dans les LORA_NEIGHBOR_TIMEOUT_S secondes."""
        with self._lock:
            return {
                bid: NeighborState(**ns.__dict__)
                for bid, ns in self.neighbors.items()
                if not ns.is_stale()
            }

    def get_team_neighbors(self) -> Dict[str, NeighborState]:
        """Coéquipiers UTT actifs (utile pour la coordination d'essaim)."""
        all_ = self.get_active_neighbors()
        return {bid: ns for bid, ns in all_.items() if config.is_teammate(bid)}
