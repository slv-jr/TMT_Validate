"""
Test RTK : attente d'un fix HERE4 RTK_FIXED avant d'autoriser le départ
(cf. README2 §"Test obligatoire avant course").

fix_type (GPS_RAW_INT) :
    0 = No GPS         1 = No fix       2 = 2D       3 = 3D
    4 = DGPS           5 = RTK Float    6 = RTK Fixed

Usage :
    DRONE_ID=U1B1 python3 -m tests.test_rtk
    # Sortie 0 = RTK Fixed obtenu, sortie 1 = échec ou timeout
"""

from __future__ import annotations

import argparse
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
log = logging.getLogger("test_rtk")


FIX_LABELS = {
    0: "NO GPS", 1: "NO FIX", 2: "2D",
    3: "3D", 4: "DGPS", 5: "RTK FLOAT", 6: "RTK FIXED",
}


def main():
    parser = argparse.ArgumentParser(
        description="Attend un fix RTK FIXED (cf. README2)",
    )
    parser.add_argument(
        "--timeout", type=float, default=180.0,
        help="Délai max en secondes (défaut 180s)",
    )
    parser.add_argument(
        "--require-fixed", action="store_true",
        help="Exige fix_type=6 (RTK Fixed) au lieu de ≥5 (Float ou Fixed)",
    )
    parser.add_argument(
        "--keep-watching", type=float, default=10.0,
        help="Une fois le fix obtenu, surveiller `N` secondes pour vérifier la stabilité",
    )
    args = parser.parse_args()

    log.info("=" * 60)
    log.info("TEST RTK — drone %s", config.DRONE_ID)
    log.info("Cible : fix_type %s (timeout=%.0fs)",
             "= 6 (RTK FIXED)" if args.require_fixed else "≥ 5 (RTK FLOAT/FIXED)",
             args.timeout)
    log.info("=" * 60)

    mav = MavlinkInterface()
    if not mav.connect(timeout_s=15):
        log.error("❌ Pas de heartbeat MAVLink")
        return 1

    target = 6 if args.require_fixed else 5
    end = time.monotonic() + args.timeout
    last_print = 0.0
    achieved_t = None

    try:
        while time.monotonic() < end:
            mav.send_heartbeat()
            tlm = mav.get_telemetry()
            now = time.monotonic()
            if now - last_print > 2.0:
                fix_label = FIX_LABELS.get(tlm.fix_type, f"?({tlm.fix_type})")
                log.info(
                    "fix_type=%d (%s) sats=%d lat=%.6f lon=%.6f",
                    tlm.fix_type, fix_label, tlm.sats_visible,
                    tlm.lat, tlm.lon,
                )
                last_print = now
            if tlm.fix_type >= target:
                if achieved_t is None:
                    achieved_t = now
                    log.info("✅ Fix %s obtenu — vérification stabilité (%.0fs)…",
                             FIX_LABELS.get(tlm.fix_type), args.keep_watching)
                if now - achieved_t >= args.keep_watching:
                    log.info("=" * 60)
                    log.info("✅ FIX RTK STABLE pendant %.0fs — drone prêt !",
                             args.keep_watching)
                    log.info("=" * 60)
                    mav.close()
                    return 0
            else:
                achieved_t = None  # reset si le fix retombe
            time.sleep(0.5)
    except KeyboardInterrupt:
        log.info("Interruption clavier")
    finally:
        mav.close()

    log.error("❌ TIMEOUT — fix_type cible non atteint dans %.0fs", args.timeout)
    return 1


if __name__ == "__main__":
    sys.exit(main())
