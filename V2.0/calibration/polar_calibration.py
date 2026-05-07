"""
Calibration de la polaire de vitesse sur l'eau (cf. README2 §"Polaire").

Procédure du J-1 (8 mai, eau calme) :
    Pour chaque angle TWA cible (40°, 50°, 60°, 70°, 90°, 120°, 150°, 180°) :
    1) Le bateau est mis sur une amure stable au cap correspondant.
    2) On enregistre 30 s de données (vitesse sol + vent réel reçu par LoRa).
    3) On calcule le ratio moyen V_bateau / V_vent.
    4) On agrège les passes dans une table JSON sauvegardée dans
       `config.POLAR_TABLE_PATH`. Au prochain démarrage de StormWings,
       `navigation/polar.py` chargera cette table calibrée.

Lancement :
    DRONE_ID=U1B1 python3 -m calibration.polar_calibration --wind-source lora

Modes de saisie du vent :
    --wind-source lora    : prend le vent réel reçu de la station Calypso
                            (recommandé, mode par défaut).
    --wind-source manual  : on demande à l'utilisateur de saisir une vitesse
                            de vent fixe (utile en l'absence de Calypso).

Le script attend que l'utilisateur :
    - confirme le cap stabilisé (le drone doit être sur l'allure cible
      depuis ≥ 10 s avant de lancer le timer)
    - laisse tourner 30 s
    - puis valide le passage à l'angle suivant
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import statistics
import sys
import time
from typing import Dict, List, Optional, Tuple

# Permettre l'import des sous-modules en mode développement
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from comms.mavlink_iface import MavlinkInterface
from comms.lora_iface import LoRaInterface

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger("polar_calibration")


# Angles cibles à explorer (cf. README2)
DEFAULT_ANGLES_DEG = [40, 50, 60, 70, 90, 120, 150, 180]
SAMPLE_DURATION_S = 30.0


def collect_pass(mav: MavlinkInterface,
                 lora: Optional[LoRaInterface],
                 wind_source: str,
                 manual_wind_ms: float,
                 duration_s: float = SAMPLE_DURATION_S) -> Tuple[float, float]:
    """Échantillonne `duration_s` : retourne (V_bateau_moy, V_vent_moy)."""
    samples_boat: List[float] = []
    samples_wind: List[float] = []
    end = time.monotonic() + duration_s
    last_print = 0.0
    while time.monotonic() < end:
        mav.send_heartbeat()
        tlm = mav.get_telemetry()
        samples_boat.append(tlm.ground_speed_ms)
        if wind_source == "lora" and lora is not None:
            wr = lora.get_wind_snapshot()
            if wr.last_received_t > 0 and not wr.sensor_offline:
                samples_wind.append(wr.speed_ms)
        else:
            samples_wind.append(manual_wind_ms)
        now = time.monotonic()
        if now - last_print > 5.0:
            remaining = end - now
            log.info(
                "  ... %.0fs restants (%d échant. boat / %d échant. wind)",
                remaining, len(samples_boat), len(samples_wind),
            )
            last_print = now
        time.sleep(0.2)

    v_boat = statistics.mean(samples_boat) if samples_boat else 0.0
    v_wind = (statistics.mean(samples_wind)
              if samples_wind else manual_wind_ms or 0.0)
    return v_boat, v_wind


def run_calibration(angles: List[int], wind_source: str,
                    manual_wind_ms: float, output_path: str):
    log.info("=" * 60)
    log.info("CALIBRATION POLAIRE — drone %s", config.DRONE_ID)
    log.info("=" * 60)
    log.info("Angles à mesurer : %s°", angles)
    log.info("Source vent      : %s", wind_source)
    if wind_source == "manual":
        log.info("Vent manuel      : %.1f m/s", manual_wind_ms)
    log.info("Durée par passe  : %.0f s", SAMPLE_DURATION_S)
    log.info("Sortie           : %s", output_path)
    log.info("=" * 60)

    mav = MavlinkInterface()
    if not mav.connect(timeout_s=15):
        log.error("Pas de heartbeat MAVLink — abort")
        return 1

    lora = None
    if wind_source == "lora":
        try:
            lora = LoRaInterface()
            if not lora.connect():
                log.warning("LoRa indisponible — fallback en saisie manuelle")
                lora = None
        except Exception as e:
            log.warning("LoRa init échoué : %s — fallback manuel", e)
            lora = None

    if wind_source == "lora" and lora is None:
        log.error("Mode 'lora' demandé mais LoRa absente. "
                  "Relancer avec --wind-source manual --manual-wind-ms <V>.")
        mav.close()
        return 1

    samples: Dict[int, Dict] = {}
    try:
        for angle in angles:
            log.info("─" * 50)
            log.info("► Mettre le drone à TWA ≈ %d° (vent à %d° du nez)",
                     angle, angle)
            log.info("  Stabiliser pendant ≥ 10 s puis valider :")
            input("  [Appuie ENTER quand le drone est stabilisé] ")
            log.info("  Échantillonnage %.0fs…", SAMPLE_DURATION_S)
            v_boat, v_wind = collect_pass(
                mav, lora, wind_source, manual_wind_ms,
            )
            ratio = v_boat / v_wind if v_wind > 0 else 0.0
            samples[angle] = {
                "boat_ms": v_boat,
                "wind_ms": v_wind,
                "ratio": ratio,
            }
            log.info(
                "  ✓ θ=%d° : V_boat=%.2f m/s · V_wind=%.2f m/s · ratio=%.3f",
                angle, v_boat, v_wind, ratio,
            )
    except KeyboardInterrupt:
        log.warning("Interruption — sauvegarde de ce qui a été collecté")

    mav.close()
    if lora is not None:
        try:
            lora.close()
        except Exception:
            pass

    if not samples:
        log.error("Aucun échantillon collecté")
        return 1

    # Construction de la table : on ajoute un point en 0° à 0 (zone morte)
    # et on conserve la valeur 35° à 0.05 (sortie de zone morte) si non mesurée
    table = [(0, 0.0), (config.POLAR_THETA_MIN_DEG, 0.05)]
    for angle in sorted(samples.keys()):
        table.append((float(angle), float(samples[angle]["ratio"])))
    # Si on n'a pas mesuré 180°, on extrapole sur le dernier point mesuré
    if 180 not in samples and table[-1][0] < 180:
        table.append((180.0, table[-1][1] * 0.85))
    table.sort(key=lambda x: x[0])

    payload = {
        "drone_id": config.DRONE_ID,
        "timestamp": time.time(),
        "wind_source": wind_source,
        "samples": samples,
        "table": [[a, r] for a, r in table],
    }
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)
    log.info("=" * 60)
    log.info("Table calibrée enregistrée dans %s :", output_path)
    for a, r in table:
        log.info("  θ=%6.1f°  →  ratio=%.3f", a, r)
    log.info("=" * 60)
    log.info("Au prochain démarrage de StormWings, navigation/polar.py la chargera.")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calibration polaire Joysway Focus V2 (cf. README2)",
    )
    parser.add_argument(
        "--angles", default="40,50,60,70,90,120,150,180",
        help="Liste d'angles TWA à mesurer (séparés par des virgules)",
    )
    parser.add_argument(
        "--wind-source", choices=["lora", "manual"], default="lora",
        help="Source du vent réel (défaut: lora)",
    )
    parser.add_argument(
        "--manual-wind-ms", type=float, default=4.5,
        help="Si --wind-source manual : vitesse du vent supposée (m/s)",
    )
    parser.add_argument(
        "--output", default=config.POLAR_TABLE_PATH,
        help="Chemin du JSON de sortie",
    )
    args = parser.parse_args()
    angles = [int(a.strip()) for a in args.angles.split(",") if a.strip()]
    sys.exit(run_calibration(
        angles=angles,
        wind_source=args.wind_source,
        manual_wind_ms=args.manual_wind_ms,
        output_path=args.output,
    ))
