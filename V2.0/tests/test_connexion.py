"""
Test 1 : connexion aux périphériques.

À exécuter en premier sur le drone, AVANT toute autre opération.
Vérifie :
    - Heartbeat MAVLink en provenance du Cube Orange+
    - Réception GPS_RAW_INT et GLOBAL_POSITION_INT
    - Réception RC_CHANNELS (notamment chan6_raw, le levier de mode, qui
      doit varier entre ~950 et ~2050 µs en bougeant le levier 3 positions)
    - Connexion à l'ESP32 LoRa V3 (Meshtastic)
    - Présence du canal BATTLEBOATS

Usage :
    DRONE_ID=U1B1 python3 -m tests.test_connexion
"""

import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from comms.mavlink_iface import MavlinkInterface
from comms.lora_iface import LoRaInterface

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger("test_connexion")


def test_mavlink() -> bool:
    log.info("=" * 60)
    log.info("TEST 1 : MAVLink (Cube Orange+ via /dev/serial0)")
    log.info("=" * 60)
    mav = MavlinkInterface()
    if not mav.connect(timeout_s=10):
        log.error("❌ Pas de heartbeat MAVLink")
        return False
    log.info("✅ Heartbeat reçu")

    log.info("Lecture télémétrie pendant 10 secondes…")
    end = time.monotonic() + 10
    last_print = 0
    while time.monotonic() < end:
        # Heartbeat GCS
        mav.send_heartbeat()
        tlm = mav.get_telemetry()
        now = time.monotonic()
        if now - last_print > 1.0:
            log.info(
                "GPS=%s fix=%d sats=%d | lat=%.6f lon=%.6f hdg=%.0f° spd=%.2fm/s "
                "| RC: rudder(ch%d)=%d sail(ch%d)=%d mode(ch%d)=%d | bat=%.1fV",
                "OK" if tlm.has_gps_fix else "NO",
                tlm.fix_type, tlm.sats_visible,
                tlm.lat, tlm.lon, tlm.heading_deg, tlm.ground_speed_ms,
                config.CH_RUDDER, tlm.rc_channels.get(config.CH_RUDDER, 0),
                config.CH_SAIL,   tlm.rc_channels.get(config.CH_SAIL, 0),
                config.CH_MODE,   tlm.rc_channels.get(config.CH_MODE, 0),
                tlm.voltage_v,
            )
            last_print = now
        time.sleep(0.1)

    final = mav.get_telemetry()
    ok = True
    if not final.has_gps_fix:
        log.warning("⚠ GPS sans fix — vérifier antenne et ciel ouvert")
        ok = False
    if final.rc_channels.get(config.CH_MODE, 0) == 0:
        log.warning(
            "⚠ Levier mode jamais reçu (chan%d=0) — vérifier émetteur RC "
            "allumé, encodeur Nano alimenté, et bouger le levier 3 positions",
            config.CH_MODE,
        )
        ok = False
    if final.voltage_v < 10.0:
        log.warning("⚠ Tension batterie faible : %.2fV", final.voltage_v)

    mav.close()
    return ok


def test_lora() -> bool:
    log.info("=" * 60)
    log.info("TEST 2 : LoRa Meshtastic (ESP32 V3)")
    log.info("=" * 60)
    try:
        lora = LoRaInterface()
    except Exception as e:
        log.error("❌ Lib meshtastic absente : %s", e)
        log.info("    Install : pip install meshtastic pypubsub")
        return False

    if not lora.connect():
        log.error("❌ Impossible d'ouvrir %s", config.LORA_PORT)
        log.info("    Vérifier : 1) cable USB-C, 2) /dev/ttyUSB0 visible, "
                 "3) ESP32 sous tension")
        return False

    log.info("Écoute du canal BATTLEBOATS pendant 90 secondes "
             "(au moins 1 message vent attendu)…")
    log.info("Le test s'arrête dès la première trame W|... reçue")
    end = time.monotonic() + 90
    last_print = 0
    while time.monotonic() < end:
        wind = lora.get_wind_snapshot()
        neighbors = lora.get_active_neighbors()
        now = time.monotonic()
        if now - last_print > 2.0:
            log.info(
                "Vent=%.0f° %.1fm/s (age=%.0fs offline=%s) | voisins=%d %s",
                wind.direction_deg, wind.speed_ms,
                now - wind.last_received_t if wind.last_received_t else 999,
                wind.sensor_offline,
                len(neighbors),
                ",".join(neighbors.keys()),
            )
            last_print = now
        if wind.last_received_t > 0:
            log.info("✅ Au moins une trame WIND reçue")
            break
        time.sleep(0.5)

    # Test d'envoi
    log.info("Test d'envoi : broadcast P|... avec position factice")
    sent = lora.broadcast_position(
        lat=43.0967, lon=5.9533,
        heading_deg=90.0, speed_ms=0.0, has_fix=True,
    )
    if sent:
        log.info("✅ Envoi LoRa OK")
    else:
        log.error("❌ Envoi LoRa échoué")

    lora.close()
    return True


if __name__ == "__main__":
    log.info("StormWings — test de connexion (drone %s)", config.DRONE_ID)
    ok_mav = test_mavlink()
    ok_lora = test_lora()
    log.info("=" * 60)
    log.info("RÉSUMÉ : MAVLink=%s LoRa=%s",
             "OK" if ok_mav else "ÉCHEC",
             "OK" if ok_lora else "ÉCHEC")
    sys.exit(0 if (ok_mav and ok_lora) else 1)
