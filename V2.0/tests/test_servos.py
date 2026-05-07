"""
Test 2 : sweep des servos.

⚠️ Ce test ENVOIE des commandes au safran et à la voile.
Le bateau doit être HORS DE L'EAU et sur ses BERS, ou maintenu manuellement.
Le levier 3 positions de la J4C05 doit être en position BAS (mode AUTO)
pour que le Pi prenne la main. Côté MAVLink, ce levier est lu sur
config.CH_MODE (par défaut chan6_raw avec le setup NouvelEncodeur).

Séquence :
    1. Vérifie qu'on est en mode AUTO (chan_mode < MODE_THRESHOLD_LOW)
    2. Centre le safran (1500 µs) pendant 2 s
    3. Safran à GAUCHE (1200 µs) pendant 2 s
    4. Safran au CENTRE pendant 2 s
    5. Safran à DROITE (1800 µs) pendant 2 s
    6. Centre le safran
    7. Voile BORDÉE (1100 µs) pendant 3 s
    8. Voile MI-OUVERTE (1500 µs) pendant 3 s
    9. Voile CHOQUÉE (1900 µs) pendant 3 s
    10. Centre + voile mi-ouverte
    11. Libère les overrides → RC reprend la main

Usage :
    DRONE_ID=U1B1 python3 -m tests.test_servos
"""

import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from comms.mavlink_iface import MavlinkInterface

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger("test_servos")


def hold_pwm(mav: MavlinkInterface, rudder: int, sail: int, duration: float):
    """Maintient les overrides à 10 Hz pendant `duration` secondes."""
    end = time.monotonic() + duration
    next_push = time.monotonic()
    next_hb = time.monotonic()
    while time.monotonic() < end:
        mav.set_rudder_pwm(rudder)
        mav.set_sail_pwm(sail)
        now = time.monotonic()
        if now >= next_push:
            mav.push_overrides()
            next_push = now + 0.1
        if now >= next_hb:
            mav.send_heartbeat()
            next_hb = now + 1.0
        # Affichage SERVO_OUTPUT_RAW
        tlm = mav.get_telemetry()
        time.sleep(0.05)


def main():
    log.info("=" * 60)
    log.info("TEST SERVOS — drone %s", config.DRONE_ID)
    log.info("⚠️  Bateau HORS DE L'EAU obligatoire !")
    log.info("=" * 60)
    mav = MavlinkInterface()
    if not mav.connect(timeout_s=10):
        log.error("Pas de connexion MAVLink")
        return 1

    # Vérification mode (chan_mode = config.CH_MODE)
    log.info("Vérification du mode RC/Auto (chan%d_raw)…", config.CH_MODE)
    time.sleep(2.0)   # laisser le temps au stream RC d'arriver
    tlm = mav.get_telemetry()
    ch_mode = tlm.rc_channels.get(config.CH_MODE, 0)
    log.info("chan%d (levier mode) = %d µs", config.CH_MODE, ch_mode)
    if ch_mode == 0:
        log.error(
            "Levier mode jamais reçu (chan%d=0) — vérifier émetteur RC "
            "allumé, récepteur J5C01R alimenté, encodeur Nano branché.",
            config.CH_MODE,
        )
        mav.close()
        return 1
    if ch_mode > config.MODE_THRESHOLD_LOW:
        log.error(
            "chan%d = %d µs > %d µs (mode RC actif). "
            "Bascule le levier 3 positions en BAS pour passer en AUTO.",
            config.CH_MODE, ch_mode, config.MODE_THRESHOLD_LOW,
        )
        mav.close()
        return 1
    log.info("✅ Mode AUTO actif")

    # Séquence
    log.info("→ Safran CENTRE / Voile MI-OUVERTE (échauffement 2s)")
    hold_pwm(mav, 1500, 1500, 2.0)

    log.info("→ Safran GAUCHE")
    hold_pwm(mav, 1200, 1500, 2.0)
    log.info("→ Safran CENTRE")
    hold_pwm(mav, 1500, 1500, 2.0)
    log.info("→ Safran DROITE")
    hold_pwm(mav, 1800, 1500, 2.0)
    log.info("→ Safran CENTRE")
    hold_pwm(mav, 1500, 1500, 2.0)

    log.info("→ Voile BORDÉE (1100 µs)")
    hold_pwm(mav, 1500, 1100, 3.0)
    log.info("→ Voile MI-OUVERTE (1500 µs)")
    hold_pwm(mav, 1500, 1500, 3.0)
    log.info("→ Voile CHOQUÉE (1900 µs)")
    hold_pwm(mav, 1500, 1900, 3.0)
    log.info("→ Voile MI-OUVERTE (1500 µs)")
    hold_pwm(mav, 1500, 1500, 2.0)

    log.info("Libération overrides — RC reprend la main")
    mav.clear_all_overrides()
    time.sleep(0.5)
    mav.close()
    log.info("✅ Test terminé")
    return 0


if __name__ == "__main__":
    sys.exit(main())
