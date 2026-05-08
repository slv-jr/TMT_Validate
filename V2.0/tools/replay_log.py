"""
Visualisation post-course d'un fichier de log CSV.

Trace la trajectoire effective sur le parcours actif (1 banane ou 2 côtier
court), avec les bouées et la route théorique. Utile pour debugger et
calibrer la polaire de vitesse.

Usage :
    python3 -m tools.replay_log logs/flight_U1B1_20260509_140532.csv
"""

import argparse
import os
import sys
import csv

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from navigation import geo_utils


def main():
    parser = argparse.ArgumentParser(description="Visualise un log de course")
    parser.add_argument("csv_path", help="Chemin du fichier CSV")
    parser.add_argument("--output", default=None,
                        help="Image de sortie (PNG). Par défaut: <csv>.png")
    parser.add_argument("--show", action="store_true",
                        help="Afficher la fenêtre matplotlib")
    args = parser.parse_args()

    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
    except ImportError:
        print("matplotlib requis : pip install matplotlib")
        sys.exit(1)

    # Lecture CSV
    east, north, heading, leg, speed, mode, role = [], [], [], [], [], [], []
    with open(args.csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                e = float(row["east_m"])
                n = float(row["north_m"])
            except (ValueError, KeyError):
                continue
            if e == 0 and n == 0:
                continue
            east.append(e)
            north.append(n)
            heading.append(float(row.get("heading_deg", 0)))
            speed.append(float(row.get("speed_ms", 0)))
            leg.append(int(row.get("leg_index", 0)))
            mode.append(row.get("control_mode", ""))
            role.append(row.get("role", ""))

    if not east:
        print("Aucune position valide trouvée")
        sys.exit(1)

    # Plot
    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_facecolor("#1a6896")
    fig.patch.set_facecolor("#0d3b5e")

    # Bouées
    colors = {
        "A": "gold", "B": "gold",
        "C": "yellow", "D": "yellow", "E": "yellow",
        "Z1": "red", "Z2": "red",
        # Parcours banane
        "1": "gold", "2": "gold", "3": "yellow", "4": "yellow",
        "P1": "red", "P2": "red",
    }
    for name, pos in config.BUOYS_LOCAL.items():
        color = colors.get(name, "white")
        circ = plt.Circle((pos.east, pos.north), 4, color=color, zorder=5)
        ax.add_patch(circ)
        ax.text(pos.east, pos.north, name, ha="center", va="center",
                fontsize=10, fontweight="bold", zorder=6)

    # Trajectoire (couleur selon vitesse)
    sc = ax.scatter(east, north, c=speed, s=2, cmap="viridis",
                    zorder=3, label=f"Trajectoire ({len(east)} pts)")
    plt.colorbar(sc, ax=ax, label="Vitesse (m/s)")

    # Porte départ/arrivée
    a = config.BUOYS_LOCAL["A"]
    b = config.BUOYS_LOCAL["B"]
    ax.plot([a.east, b.east], [a.north, b.north], "g-",
            linewidth=2, zorder=4, label="Porte A-B")

    # Position de départ et d'arrivée
    ax.plot(east[0], north[0], "g^", markersize=12, zorder=6, label="Départ")
    ax.plot(east[-1], north[-1], "rv", markersize=12, zorder=6, label="Fin")

    ax.set_xlabel("East (m)", color="white")
    ax.set_ylabel("North (m)", color="white")
    ax.set_title(
        f"Replay {os.path.basename(args.csv_path)}\n"
        f"Drone {config.DRONE_ID} — durée {len(east) / 10:.0f}s — "
        f"vitesse moy {sum(speed)/len(speed):.2f} m/s",
        color="white",
    )
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_color("white")
    ax.legend(facecolor="#0d3b5e", labelcolor="white")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    out = args.output or args.csv_path.replace(".csv", ".png")
    plt.savefig(out, dpi=150, bbox_inches="tight", facecolor="#0d3b5e")
    print(f"Image sauvegardée : {out}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
