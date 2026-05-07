"""
Test 2 : sweep des servos.

⚠️ Ce test ENVOIE des commandes au safran et à la voile.
Le bateau doit être HORS DE L'EAU et sur ses BERS, ou maintenu manuellement.
Le levier CH3 doit être en position BAS (mode AUTO) pour que le Pi prenne la main.

Séquence :
    1. Vérifie qu'on est en mode AUTO (CH3 < 1300 µs)
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

    # Vérification mode CH3
    log.info("Vérification du mode RC/Auto (CH3)…")
    time.sleep(2.0)   # laisser le temps au stream RC d'arriver
    tlm = mav.get_telemetry()
    ch3 = tlm.rc_channels.get(3, 0)
    log.info("CH3 = %d µs", ch3)
    if ch3 == 0:
        log.error("CH3 jamais reçu — vérifier l'émetteur RC")
        mav.close()
        return 1
    if ch3 > config.CH3_THRESHOLD_LOW:
        log.error(
            "CH3 = %d µs > %d µs (mode RC actif). "
            "Bascule le levier CH3 en BAS pour passer en AUTO.",
            ch3, config.CH3_THRESHOLD_LOW,
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
