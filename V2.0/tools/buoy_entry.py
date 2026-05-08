"""
Saisie des coordonnées GPS officielles des bouées (matin J0 — 9 mai).

L'outil adapte automatiquement la liste des bouées au parcours actif
(config.COURSE_NUMBER) :

    Parcours 1 (banane)      : 1, 2, 3, 4, P1, P2  (6 bouées)
    Parcours 2 (côtier court): A, B, C, D, E, Z1, Z2  (7 bouées)

Cet outil :
    1) Affiche les bouées attendues une par une.
    2) Demande les coordonnées en lat/lon décimal (ex: 43.0967000 5.9542853)
       OU en degrés-minutes-décimales (ex: 43°5.802'N 5°57.171'E)
       OU directement copier-coller depuis le PDF du briefing.
    3) Calcule les distances entre bouées (sanity check anti-faute de frappe).
    4) Écrit le résultat dans config.BUOYS_OVERRIDE_PATH (par défaut
       /etc/stormwings/buoys_today.json) qui sera lu au démarrage du service.

Usage :
    COURSE_NUMBER=2 python3 -m tools.buoy_entry         # parcours côtier court
    COURSE_NUMBER=1 python3 -m tools.buoy_entry         # parcours banane
    BUOYS_OVERRIDE_PATH=/tmp/test.json python3 -m tools.buoy_entry
    python3 -m tools.buoy_entry --all                   # saisir toutes (1+2)
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import re
import sys
from typing import Dict, Optional, Tuple

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from navigation import geo_utils

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("buoy_entry")


# Liste exhaustive des bouées possibles sur les 2 parcours retenus
ALL_BUOYS = ["1", "2", "3", "4", "P1", "P2",          # parcours banane
             "A", "B", "C", "D", "E", "Z1", "Z2"]     # parcours côtier court


def expected_buoys_for_course(course_num: int, all_buoys: bool = False):
    """Retourne la liste des bouées à saisir pour le parcours.

    Si `all_buoys=True`, retourne toutes les bouées des 2 parcours retenus.
    """
    if all_buoys:
        return list(ALL_BUOYS)
    return config.buoys_used_in_course(course_num)


# ────────────────────────────────────────────────────────────────────────
# Parsing tolérant
# ────────────────────────────────────────────────────────────────────────
_DECIMAL_RE = re.compile(r"^\s*([+-]?\d+(?:\.\d+)?)\s*[, ]\s*([+-]?\d+(?:\.\d+)?)\s*$")
_DM_RE = re.compile(
    r"^\s*(\d+)°\s*(\d+(?:\.\d+)?)['\u2032]?\s*([NSns])"
    r"\s+(\d+)°\s*(\d+(?:\.\d+)?)['\u2032]?\s*([EWew])\s*$"
)


def parse_coords(text: str) -> Optional[Tuple[float, float]]:
    """Tente plusieurs formats. Renvoie (lat, lon) en degrés ou None."""
    if not text:
        return None
    text = text.strip()
    # Format décimal
    m = _DECIMAL_RE.match(text)
    if m:
        return float(m.group(1)), float(m.group(2))
    # Format degrés-minutes décimales
    m = _DM_RE.match(text)
    if m:
        deg_lat = float(m.group(1))
        min_lat = float(m.group(2))
        ns = m.group(3).upper()
        deg_lon = float(m.group(4))
        min_lon = float(m.group(5))
        ew = m.group(6).upper()
        lat = deg_lat + min_lat / 60.0
        if ns == "S":
            lat = -lat
        lon = deg_lon + min_lon / 60.0
        if ew == "W":
            lon = -lon
        return lat, lon
    return None


# ────────────────────────────────────────────────────────────────────────
# Saisie interactive
# ────────────────────────────────────────────────────────────────────────
def prompt_buoy(name: str, default: Optional[Tuple[float, float]]):
    """Demande à l'utilisateur la coord d'une bouée."""
    default_str = (
        f"  défaut : {default[0]:.7f}, {default[1]:.7f}" if default else ""
    )
    print(f"\n► Bouée {name}{default_str}")
    print("  Format accepté : 43.0967 5.9543  OU  43°05.802'N 5°57.171'E")
    while True:
        text = input(f"  {name} > ").strip()
        if not text and default is not None:
            return default
        if text.lower() in ("skip", "s"):
            return None
        coords = parse_coords(text)
        if coords is None:
            print("  ❌ Format non reconnu — réessaie")
            continue
        lat, lon = coords
        if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
            print(f"  ❌ Hors limites (lat={lat}, lon={lon}) — réessaie")
            continue
        # Sanity check : distance à la valeur attendue
        if default is not None:
            d = geo_utils.distance_m(default, coords)
            if d > 500:
                print(
                    f"  ⚠ Différence > 500 m avec la valeur attendue "
                    f"({default[0]:.5f}, {default[1]:.5f}) — confirmer ? [o/N] ",
                    end="",
                )
                if input().strip().lower() not in ("o", "y", "oui", "yes"):
                    continue
        print(f"  ✓ {name} = ({lat:.7f}, {lon:.7f})")
        return coords


def display_summary(buoys: Dict[str, Tuple[float, float]],
                    expected: list):
    print("\n" + "═" * 60)
    print("RÉCAPITULATIF DES BOUÉES :")
    print("═" * 60)
    for name in expected:
        if name in buoys:
            lat, lon = buoys[name]
            print(f"  {name:>3} : ({lat:.7f}, {lon:.7f})")
        else:
            print(f"  {name:>3} : (non saisie — fallback config.py)")

    # Distances clés selon le parcours
    print("\nDistances entre bouées (m) :")
    pairs_banane = [("1", "2"), ("3", "4"), ("1", "3"), ("2", "4"),
                    ("P1", "P2"), ("2", "P1")]
    pairs_cotier = [("A", "B"), ("A", "C"), ("C", "D"), ("D", "E"),
                    ("E", "C"), ("Z1", "Z2")]
    pairs = pairs_banane if config.COURSE_NUMBER == 1 else pairs_cotier
    for a, b in pairs:
        if a in buoys and b in buoys:
            d = geo_utils.distance_m(buoys[a], buoys[b])
            print(f"  {a}-{b} : {d:7.1f} m")
    print("═" * 60)


def main():
    parser = argparse.ArgumentParser(description="Saisie bouées matin J0")
    parser.add_argument(
        "--output", default=config.BUOYS_OVERRIDE_PATH,
        help="Fichier JSON de sortie (par défaut config.BUOYS_OVERRIDE_PATH)",
    )
    parser.add_argument(
        "--use-defaults", action="store_true",
        help="Pré-remplir avec les valeurs config.BUOYS_GPS",
    )
    parser.add_argument(
        "--all", action="store_true",
        help="Saisir TOUTES les bouées des 2 parcours (utile en briefing).",
    )
    parser.add_argument(
        "--course", type=int, default=config.COURSE_NUMBER, choices=[1, 2],
        help="Numéro de parcours (1 banane ou 2 côtier court).",
    )
    args = parser.parse_args()

    expected = expected_buoys_for_course(args.course, all_buoys=args.all)

    print("═" * 60)
    if args.all:
        print("SAISIE DES BOUÉES — TOUS LES PARCOURS — BattleBoats 2026")
    else:
        print(f"SAISIE DES BOUÉES — Parcours N°{args.course} — BattleBoats 2026")
    print("═" * 60)
    print(f"Bouées à saisir : {expected}")
    print("Tapez 'skip' pour passer une bouée (utilise la valeur de config.py).")
    print("Tapez ENTER seul pour accepter la valeur par défaut.")
    print()

    buoys: Dict[str, Tuple[float, float]] = {}
    for name in expected:
        default = config.BUOYS_GPS.get(name) if args.use_defaults else None
        ref = config.BUOYS_GPS.get(name)
        coords = prompt_buoy(name, ref)
        if coords is not None:
            buoys[name] = coords

    display_summary(buoys, expected)

    if not buoys:
        print("\nAucune bouée saisie — rien à enregistrer.")
        return 1

    print(f"\nÉcriture vers {args.output} ?  [O/n] ", end="")
    if input().strip().lower() in ("n", "non", "no"):
        print("Annulé.")
        return 0

    out_dir = os.path.dirname(args.output)
    if out_dir and not os.path.exists(out_dir):
        try:
            os.makedirs(out_dir, exist_ok=True)
        except PermissionError:
            print(f"❌ Impossible de créer {out_dir} (sudo nécessaire ?)")
            return 1

    payload = {name: [lat, lon] for name, (lat, lon) in buoys.items()}
    try:
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)
    except PermissionError:
        print(f"❌ Permission refusée pour {args.output}")
        print("    Relancer avec sudo, ou exporter BUOYS_OVERRIDE_PATH=/tmp/...")
        return 1
    print(f"\n✅ Bouées écrites dans {args.output}")
    print("   Au prochain démarrage du service, config.py les chargera.")
    print("   Pour redémarrer immédiatement : sudo systemctl restart stormwings")
    return 0


if __name__ == "__main__":
    sys.exit(main())
