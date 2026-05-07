"""
Test 3 : bascule manuel / automatique via CH3.

Vérifie que :
    - CH3 bas → Pi prend la main, le safran balaye gauche/centre/droite
    - CH3 haut → RC reprend physiquement la main, le safran suit le stick
    - La transition est franche (hystérésis OK)

Usage :
    DRONE_ID=U1B1 python3 -m tests.test_bascule
"""

import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from comms.mavlink_iface import MavlinkInterface
from safety.mode_switch import ControlMode, ModeSwitch

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger("test_bascule")


def main():
    log.info("=" * 60)
    log.info("TEST BASCULE MANUEL/AUTO — drone %s", config.DRONE_ID)
    log.info("Bouge le levier CH3 et observe la sortie")
    log.info("Ctrl+C pour arrêter")
    log.info("=" * 60)
    mav = MavlinkInterface()
    if not mav.connect(timeout_s=10):
        log.error("Pas de connexion MAVLink")
        return 1
    sw = ModeSwitch(mav)

    sweep = [(1200, "GAUCHE"), (1500, "CENTRE"),
             (1800, "DROITE"), (1500, "CENTRE")]
    sweep_idx = 0
    last_sweep = 0.0
    last_hb = 0.0
    last_push = 0.0

    try:
        while True:
            now = time.monotonic()
            if now - last_hb >= 1.0:
                mav.send_heartbeat()
                last_hb = now

            status = sw.update()
            tlm = mav.get_telemetry()

            if status.mode == ControlMode.AUTO:
                # Sweep automatique du safran
                if now - last_sweep >= 1.5:
                    pwm, label = sweep[sweep_idx % len(sweep)]
                    mav.set_rudder_pwm(pwm)
                    log.info("[AUTO] CH3=%d µs → safran %s (%dµs) "
                             "OUT1=%d", status.ch3_pwm, label, pwm,
                             tlm.rc_channels.get(1, 0))
                    sweep_idx += 1
                    last_sweep = now
                if now - last_push >= 0.1:
                    mav.push_overrides()
                    last_push = now
            elif status.mode == ControlMode.MANUAL:
                # En manuel, on n'override rien — on log juste l'état
                if int(now * 2) % 4 == 0:    # ~ chaque 2s
                    log.info("[MANUAL] CH3=%d µs ; CH1(stick)=%d → OUT1=%d",
                             status.ch3_pwm,
                             tlm.rc_channels.get(1, 0),
                             0)   # SERVO_OUTPUT_RAW si exposé
                    time.sleep(0.4)
            else:
                log.info("[UNKNOWN] CH3=%d µs", status.ch3_pwm)
                time.sleep(0.2)

            time.sleep(0.02)
    except KeyboardInterrupt:
        log.info("Arrêt.")
    finally:
        mav.clear_all_overrides()
        mav.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
