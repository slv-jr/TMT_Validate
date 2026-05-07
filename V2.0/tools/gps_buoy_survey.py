"""
Outil de relevé GPS des bouées — DÉSORMAIS OPTIONNEL.

⚠️ Le 9 mai 2026, les coordonnées GPS officielles des bouées sont fournies
par les organisateurs le matin de la course. Dans ce cas, il SUFFIT
d'éditer directement `config.BUOYS_GPS` avec ces coordonnées et de
redémarrer le service stormwings.

Cet outil reste utile dans deux cas :
    - Vérification croisée d'une coordonnée fournie (mesure embarquée
      avec le GPS RTK Here4 du drone, comparaison avec la valeur officielle).
    - Plan B si les coordonnées officielles arrivent en retard ou s'avèrent
      erronées et qu'il faut faire un relevé express sur l'eau.

Procédure :
    1. Embarquer le drone (ou un GPS RTK portatif) au plus près de chaque bouée.
    2. Lancer ce script avec le nom de la bouée :
        DRONE_ID=U1B1 python3 -m tools.gps_buoy_survey C
    3. Le script attend une stabilisation du fix (RTK > 30s recommandé).
    4. Il moyenne les positions sur 30 s puis affiche :
        - les coordonnées GPS (lat/lon) à coller dans config.BUOYS_GPS
        - les coordonnées locales (East/North) dérivées (information seulement)

Sortie : un fichier `tools/buoys_surveyed.json` qui sauvegarde toutes les
relevés successifs pour mise à jour finale du config.py.
"""

import argparse
import json
import logging
import os
import statistics
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from comms.mavlink_iface import MavlinkInterface
from navigation import geo_utils

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("buoy_survey")


def survey(buoy_name: str, duration_s: float = 30.0,
           require_rtk: bool = True) -> dict:
    log.info("Survey bouée '%s' — durée %.0fs (RTK requis: %s)",
             buoy_name, duration_s, require_rtk)
    mav = MavlinkInterface()
    if not mav.connect(timeout_s=15):
        raise RuntimeError("Pas de heartbeat MAVLink")

    # Attendre fix
    log.info("Attente fix GPS…")
    end_wait = time.monotonic() + 60.0
    while time.monotonic() < end_wait:
        mav.send_heartbeat()
        tlm = mav.get_telemetry()
        if tlm.has_gps_fix:
            log.info("Fix obtenu : type=%d sats=%d (RTK=%s)",
                     tlm.fix_type, tlm.sats_visible,
                     "OUI" if tlm.fix_type >= 6 else "NON")
            if require_rtk and tlm.fix_type < 6:
                log.warning("RTK pas encore actif, on attend encore…")
                time.sleep(2)
                continue
            break
        time.sleep(0.5)
    else:
        raise RuntimeError("Pas de fix GPS dans le délai imparti")

    # Échantillonnage
    log.info("Échantillonnage %.0fs…", duration_s)
    lats, lons = [], []
    end = time.monotonic() + duration_s
    last_print = 0
    while time.monotonic() < end:
        mav.send_heartbeat()
        tlm = mav.get_telemetry()
        if tlm.has_gps_fix:
            lats.append(tlm.lat)
            lons.append(tlm.lon)
        now = time.monotonic()
        if now - last_print > 5.0:
            log.info("  ... %d échantillons collectés", len(lats))
            last_print = now
        time.sleep(0.2)

    mav.close()

    if len(lats) < 30:
        raise RuntimeError(f"Trop peu d'échantillons : {len(lats)}")

    lat_mean = statistics.mean(lats)
    lon_mean = statistics.mean(lons)
    lat_std = statistics.stdev(lats) if len(lats) > 1 else 0.0
    lon_std = statistics.stdev(lons) if len(lons) > 1 else 0.0
    east, north = geo_utils.gps_to_local(lat_mean, lon_mean)

    result = {
        "buoy": buoy_name.upper(),
        "lat": lat_mean,
        "lon": lon_mean,
        "lat_std": lat_std,
        "lon_std": lon_std,
        "samples": len(lats),
        "local_east": east,
        "local_north": north,
        "rtk_was_required": require_rtk,
        "timestamp": time.time(),
    }

    log.info("=" * 60)
    log.info("RÉSULTAT bouée %s :", buoy_name.upper())
    log.info("  lat = %.7f° (σ=%.7f°)", lat_mean, lat_std)
    log.info("  lon = %.7f° (σ=%.7f°)", lon_mean, lon_std)
    log.info("  Local dérivé : (%.2f, %.2f) m E,N (info)", east, north)
    log.info("=" * 60)
    log.info("Ligne config.BUOYS_GPS à coller :")
    log.info('  "%s":  (%.7f, %.7f),', buoy_name.upper(), lat_mean, lon_mean)

    return result


def save_to_file(result: dict, path: str):
    if os.path.exists(path):
        with open(path, "r") as f:
            try:
                data = json.load(f)
            except Exception:
                data = {}
    else:
        data = {}
    data[result["buoy"]] = result
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    log.info("Sauvegardé dans %s", path)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Relevé GPS d'une bouée")
    parser.add_argument("buoy", help="Nom de la bouée (A, B, C, D, E, F, G, Z1, Z2)")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--no-rtk", action="store_true",
                        help="Accepter un fix non-RTK (par défaut RTK requis)")
    parser.add_argument("--output", default="tools/buoys_surveyed.json")
    args = parser.parse_args()

    result = survey(args.buoy, args.duration, require_rtk=not args.no_rtk)
    save_to_file(result, args.output)
