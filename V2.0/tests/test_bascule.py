"""
Test 3 : bascule manuel / automatique via le levier 3 positions.

Côté MAVLink, ce levier est sur config.CH_MODE (par défaut chan6_raw avec
le setup NouvelEncodeur). Vérifie que :
    - Levier BAS  → Pi prend la main, le safran balaye gauche/centre/droite
    - Levier HAUT → RC reprend physiquement la main, le safran suit le stick
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
    log.info("Bouge le levier 3 positions (chan%d) et observe la sortie",
             config.CH_MODE)
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
                    log.info("[AUTO] mode(ch%d)=%d µs → safran %s (%dµs) "
                             "rudder_in(ch%d)=%d",
                             config.CH_MODE, status.mode_pwm, label, pwm,
                             config.CH_RUDDER,
                             tlm.rc_channels.get(config.CH_RUDDER, 0))
                    sweep_idx += 1
                    last_sweep = now
                if now - last_push >= 0.1:
                    mav.push_overrides()
                    last_push = now
            elif status.mode == ControlMode.MANUAL:
                # En manuel, on n'override rien — on log juste l'état
                if int(now * 2) % 4 == 0:    # ~ chaque 2s
                    log.info("[MANUAL] mode(ch%d)=%d µs ; "
                             "rudder_stick(ch%d)=%d",
                             config.CH_MODE, status.mode_pwm,
                             config.CH_RUDDER,
                             tlm.rc_channels.get(config.CH_RUDDER, 0))
                    time.sleep(0.4)
            else:
                log.info("[UNKNOWN] mode(ch%d)=%d µs",
                         config.CH_MODE, status.mode_pwm)
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
