"""
Outil de diagnostic LoRa BattleBoats — vérifie la conformité au PDF v5.

Trois modes d'utilisation :

    # 1. Encoder une trame P|... à partir de valeurs MAVLink réelles
    python3 -m tools.lora_diag encode-pos \\
        --id U1B1 --lat 43.48256 --lon 6.49872 --hdg 185 --spd-ms 1.6464

    # 2. Décoder + interpréter une trame reçue (W|... ou P|...)
    python3 -m tools.lora_diag decode "W|245|63|1746787652"
    python3 -m tools.lora_diag decode "P|U1B1|4348256|649872|185|32"

    # 3. Écouter le canal BATTLEBOATS en live, parser tout en temps réel
    python3 -m tools.lora_diag listen --port /dev/ttyUSB0

Pour chaque trame reçue, le script affiche :
  - Format brut (vérifie que les champs respectent la spec PDF v5)
  - Interprétation humaine (direction cardinale, vitesse en kn, etc.)
  - Validation de conformité (caractères de remplissage, plages valides)

Conventions PDF v5 §3.2-3.3 :
  - WIND  : direction "d'OÙ vient le vent" — 0=N, 90=E, 180=S, 270=O
  - POS   : cap magnétique (idem convention) — direction du bateau
  - lat/lon : int32 ×1e5 (résolution ~1 m)
  - speed wind : uint16 ×10 m/s
  - speed boat : uint16 ×10 nœuds (1 m/s = 1.94384 kn)
"""

import argparse
import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from comms import protocol  # noqa: E402

log = logging.getLogger("lora_diag")


# ════════════════════════════════════════════════════════════════════════
# Helpers d'affichage
# ════════════════════════════════════════════════════════════════════════
_CARDINAL = [
    (0,   "N"),   (22,  "NNE"), (45,  "NE"),  (67,  "ENE"),
    (90,  "E"),   (112, "ESE"), (135, "SE"),  (157, "SSE"),
    (180, "S"),   (202, "SSW"), (225, "SW"),  (247, "WSW"),
    (270, "W"),   (292, "WNW"), (315, "NW"),  (337, "NNW"),
]


def cardinal(deg: float) -> str:
    """Renvoie la rose des vents (16 secteurs) pour un angle en degrés."""
    deg = deg % 360.0
    closest = min(_CARDINAL, key=lambda t: min(abs(deg - t[0]), 360 - abs(deg - t[0])))
    return closest[1]


def pretty_wind(msg: protocol.WindMessage) -> str:
    if msg.sensor_offline:
        return "[ORGA] WIND : capteur HORS LIGNE (W|000|00|...)"
    return (
        f"[ORGA] WIND : {msg.direction_deg:3d}° ({cardinal(msg.direction_deg)}) "
        f"souffle vers {cardinal((msg.direction_deg + 180) % 360)}, "
        f"vitesse {msg.speed_ms:.1f} m/s ({msg.speed_ms * 1.94384:.1f} kn), "
        f"timestamp {msg.timestamp}"
    )


def pretty_pos(msg: protocol.PositionMessage) -> str:
    if msg.no_fix:
        return f"[BOAT] {msg.boat_id}: PAS DE FIX GPS (P|...|0|0|0|0)"
    return (
        f"[BOAT] {msg.boat_id} @ ({msg.lat:.5f}, {msg.lon:.5f}) "
        f"cap {msg.heading_deg:3d}° ({cardinal(msg.heading_deg)}), "
        f"vitesse {msg.speed_knots:.1f} kn ({msg.speed_knots / 1.94384:.2f} m/s)"
    )


# ════════════════════════════════════════════════════════════════════════
# Validation de conformité
# ════════════════════════════════════════════════════════════════════════
def validate_wind_text(text: str) -> bool:
    """Vérifie qu'une trame W|... respecte exactement le format PDF v5."""
    parts = text.strip().split("|")
    ok = True
    if len(parts) != 4 or parts[0] != "W":
        print(f"  [ERR] Format invalide : 4 champs séparés par '|' attendus (W|dir|spd|ts)")
        return False
    # dir : 3 chiffres exactement
    if len(parts[1]) != 3 or not parts[1].isdigit():
        print(f"  [WARN] Champ 'dir' devrait être 3 chiffres zero-padded (ex: 045) : '{parts[1]}'")
        ok = False
    # spd : 2 chiffres minimum
    if not parts[2].isdigit():
        print(f"  [WARN] Champ 'spd' doit être un entier : '{parts[2]}'")
        ok = False
    elif len(parts[2]) < 2:
        print(f"  [WARN] Champ 'spd' devrait être ≥ 2 chiffres zero-padded : '{parts[2]}'")
    # ts : entier positif
    try:
        ts = int(parts[3])
        if ts < 1_500_000_000:
            print(f"  [WARN] timestamp suspicieusement bas : {ts}")
    except ValueError:
        print(f"  [ERR] timestamp invalide : '{parts[3]}'")
        ok = False
    # plages valides
    try:
        d = int(parts[1])
        if not (0 <= d <= 359):
            print(f"  [ERR] direction hors [0..359] : {d}")
            ok = False
    except ValueError:
        ok = False
    return ok


def validate_pos_text(text: str) -> bool:
    """Vérifie qu'une trame P|... respecte exactement le format PDF v5."""
    parts = text.strip().split("|")
    ok = True
    if len(parts) != 6 or parts[0] != "P":
        print(f"  [ERR] Format invalide : 6 champs séparés par '|' (P|id|lat|lon|hdg|spd)")
        return False
    # boat_id : doit être dans la liste officielle
    valid_ids = {"U1B1", "U1B2", "U1B3", "D2B1", "D2B2", "D2B3",
                 "I3B1", "I3B2", "I3B3", "E4B1", "E4B2", "E4B3"}
    if parts[1] not in valid_ids:
        print(f"  [WARN] ID '{parts[1]}' non officiel (cf. PDF §3.3) — OK pour test")
    # lat/lon : int signé
    for i, name in [(2, "lat"), (3, "lon")]:
        try:
            int(parts[i])
        except ValueError:
            print(f"  [ERR] {name} non entier : '{parts[i]}'")
            ok = False
    # Cas spécial PDF : P|<id>|0|0|0|0 = no-fix -> tolérance pour les champs "0"
    is_no_fix = (parts[2] == "0" and parts[3] == "0"
                 and parts[4] == "0" and parts[5] == "0")
    # hdg : 3 chiffres (sauf cas no-fix où "0" est explicitement attendu par le PDF)
    if not is_no_fix and (len(parts[4]) != 3 or not parts[4].lstrip("-").isdigit()):
        print(f"  [WARN] hdg devrait être 3 chiffres zero-padded (ex: 045) : '{parts[4]}'")
    try:
        h = int(parts[4])
        if not (0 <= h <= 359):
            print(f"  [ERR] hdg hors [0..359] : {h}")
            ok = False
    except ValueError:
        ok = False
    # spd : 2 chiffres min
    try:
        int(parts[5])
    except ValueError:
        print(f"  [ERR] spd non entier : '{parts[5]}'")
        ok = False
    return ok


# ════════════════════════════════════════════════════════════════════════
# Sous-commandes
# ════════════════════════════════════════════════════════════════════════
def cmd_encode_pos(args):
    msg = protocol.build_position_from_telemetry(
        boat_id=args.id, lat=args.lat, lon=args.lon,
        heading_deg=args.hdg, ground_speed_ms=args.spd_ms,
        has_fix=not args.no_fix,
    )
    text = msg.encode()
    print(f"Encodage P|... -> {text!r}")
    print(f"  -> {pretty_pos(msg)}")
    print(f"  Validation : {'OK [OK]' if validate_pos_text(text) else 'PROBLÈME'}")
    return 0


def cmd_encode_wind(args):
    msg = protocol.WindMessage(
        direction_deg=args.dir,
        speed_ms=args.spd,
        timestamp=args.ts or int(time.time()),
        sensor_offline=(args.dir == 0 and args.spd == 0.0),
    )
    text = msg.encode()
    print(f"Encodage W|... -> {text!r}")
    print(f"  -> {pretty_wind(msg)}")
    print(f"  Validation : {'OK [OK]' if validate_wind_text(text) else 'PROBLÈME'}")
    return 0


def cmd_decode(args):
    text = args.frame
    print(f"Trame brute : {text!r}")
    print()
    if text.startswith("W|"):
        ok = validate_wind_text(text)
        msg = protocol.WindMessage.parse(text)
        if msg is None:
            print("  [ERR] parse_message a retourné None — la trame n'est pas conforme.")
            return 1
        print(pretty_wind(msg))
    elif text.startswith("P|"):
        ok = validate_pos_text(text)
        msg = protocol.PositionMessage.parse(text)
        if msg is None:
            print("  [ERR] parse_message a retourné None")
            return 1
        print(pretty_pos(msg))
    else:
        print("  [ERR] Préfixe inconnu (attendu W| ou P|)")
        return 1
    print()
    print(f"Conformité PDF v5 : {'OK [OK]' if ok else 'WARN — voir messages au-dessus'}")
    return 0 if ok else 1


def cmd_listen(args):
    """Écoute le canal BATTLEBOATS en live et valide chaque trame."""
    try:
        from comms.lora_iface import LoRaInterface  # noqa: F401
        import meshtastic.serial_interface
        from pubsub import pub
    except ImportError as e:
        print(f"[ERR] Lib manquante : {e}\n-> pip install meshtastic pypubsub")
        return 1

    print(f"[LISTEN] Écoute du canal BATTLEBOATS sur {args.port}…")
    print(f"   (Ctrl-C pour quitter)")
    print()

    iface = meshtastic.serial_interface.SerialInterface(devPath=args.port)
    counts = {"W": 0, "P": 0, "?": 0, "ERR": 0}
    start_t = time.monotonic()

    def on_receive(packet, interface):
        try:
            decoded = packet.get("decoded", {})
            if decoded.get("portnum") != "TEXT_MESSAGE_APP":
                return
            text = decoded.get("text", "")
            if not text:
                return
            t = time.monotonic() - start_t
            print(f"\n[+{t:6.1f}s] {text!r}")
            if text.startswith("W|"):
                counts["W"] += 1
                ok = validate_wind_text(text)
                msg = protocol.WindMessage.parse(text)
                if msg:
                    print(f"  -> {pretty_wind(msg)}")
            elif text.startswith("P|"):
                counts["P"] += 1
                ok = validate_pos_text(text)
                msg = protocol.PositionMessage.parse(text)
                if msg:
                    print(f"  -> {pretty_pos(msg)}")
            else:
                counts["?"] += 1
                print(f"  [WARN] Préfixe inconnu (ni W| ni P|)")
        except Exception as e:
            counts["ERR"] += 1
            print(f"  [ERR] Exception : {e}")

    pub.subscribe(on_receive, "meshtastic.receive")

    try:
        while True:
            time.sleep(60)
            print(f"\n--- Stats {time.monotonic() - start_t:.0f}s : "
                  f"WIND={counts['W']}  POS={counts['P']}  "
                  f"unknown={counts['?']}  err={counts['ERR']} ---")
    except KeyboardInterrupt:
        print(f"\n\nFin écoute après {time.monotonic() - start_t:.0f}s")
        print(f"  WIND reçus  : {counts['W']}")
        print(f"  POS reçus   : {counts['P']}")
        print(f"  Inconnus    : {counts['?']}")
        print(f"  Erreurs     : {counts['ERR']}")
    finally:
        iface.close()
    return 0


# ════════════════════════════════════════════════════════════════════════
# main
# ════════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        prog="lora_diag",
        description="Diagnostic conformité LoRa BattleBoats v5",
    )
    sub = parser.add_subparsers(dest="cmd", required=True)

    # encode-pos
    p_pos = sub.add_parser("encode-pos", help="Encoder une trame P|...")
    p_pos.add_argument("--id", required=True, help="boat_id (ex: U1B1)")
    p_pos.add_argument("--lat", type=float, required=True)
    p_pos.add_argument("--lon", type=float, required=True)
    p_pos.add_argument("--hdg", type=float, required=True, help="cap magnétique en degrés")
    p_pos.add_argument("--spd-ms", type=float, required=True, help="vitesse fond m/s")
    p_pos.add_argument("--no-fix", action="store_true", help="Forcer le mode 'pas de fix'")
    p_pos.set_defaults(func=cmd_encode_pos)

    # encode-wind
    p_w = sub.add_parser("encode-wind", help="Encoder une trame W|...")
    p_w.add_argument("--dir", type=int, required=True,
                     help="direction d'OÙ vient le vent (0=N, 90=E, 180=S, 270=O)")
    p_w.add_argument("--spd", type=float, required=True, help="vitesse vent en m/s")
    p_w.add_argument("--ts", type=int, default=None, help="timestamp Unix (défaut: now)")
    p_w.set_defaults(func=cmd_encode_wind)

    # decode
    p_d = sub.add_parser("decode", help="Décoder + valider une trame reçue")
    p_d.add_argument("frame", help="Trame brute (W|... ou P|...)")
    p_d.set_defaults(func=cmd_decode)

    # listen
    p_l = sub.add_parser("listen", help="Écouter le canal LoRa en live")
    p_l.add_argument("--port", default="/dev/ttyUSB0", help="port série ESP32")
    p_l.set_defaults(func=cmd_listen)

    args = parser.parse_args()
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())


