"""
Simulateur 2D simple du parcours N°3 (sans matériel).

Permet de vérifier que la stack logicielle (waypoints + VMG + layline +
champ de potentiel) génère une trajectoire COHÉRENTE avant les essais
sur l'eau.

Le bateau virtuel et toutes les positions du parcours sont en GPS (lat, lon),
exactement comme le runtime. Seul le rendu matplotlib utilise une
conversion locale (purement esthétique).

Modèle physique très simplifié :
    - Vent uniforme (configurable)
    - Bateau ponctuel, vitesse = polaire(TWA, V_vent)
    - Pas de gîte, pas de courant, pas de bruit GPS

Usage :
    python3 -m tools.simulator [--wind-dir 90] [--wind-speed 5]
"""

import argparse
import math
import os
import sys
import time

# Forcer UTF-8 sur stdout/stderr (Windows utilise cp1252 par défaut, ce qui
# fait crasher l'affichage des emojis du résumé final).
try:
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
except (AttributeError, ValueError):
    pass

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from navigation import geo_utils, layline, polar, potential_field, vmg
from navigation.heading_pid import HeadingPID
from navigation.waypoints import CourseManager


class SimBoat:
    """Bateau virtuel — position en GPS (lat, lon)."""

    def __init__(self, start_pos_gps):
        self.pos = list(start_pos_gps)         # [lat, lon]
        self.heading_deg = 90.0                # cap initial vers l'est
        self.speed_ms = 0.0
        self.last_tack_t = 0.0
        self.history = [tuple(self.pos)]

    def step(self, target_heading_deg: float, wind_dir: float, wind_speed: float,
             dt: float):
        # Yaw rate limité (le bateau tourne progressivement)
        diff = geo_utils.angle_diff_deg(target_heading_deg, self.heading_deg)
        max_rate = 60.0    # °/s
        delta = max(-max_rate * dt, min(max_rate * dt, diff))
        self.heading_deg = (self.heading_deg + delta) % 360.0

        # Vitesse selon la polaire
        twa = abs(geo_utils.angle_diff_deg(self.heading_deg, wind_dir))
        v_target = polar.boat_speed_predicted(twa, wind_speed)
        # Inertie (montée en vitesse progressive)
        self.speed_ms += (v_target - self.speed_ms) * 0.3

        # Avancer : déplacement (east, north) en mètres puis nouvelle position GPS
        ux, uy = geo_utils.cap_to_unit_vector(self.heading_deg)
        new_pos = geo_utils.move_meters(
            tuple(self.pos),
            ux * self.speed_ms * dt,
            uy * self.speed_ms * dt,
        )
        self.pos = list(new_pos)
        self.history.append(tuple(self.pos))


def simulate(wind_dir: float, wind_speed: float, max_time_s: float = 1800.0,
             plot: bool = True):
    print(f"=== Simulation parcours N°3 ===")
    print(f"Vent : {wind_dir}° à {wind_speed:.1f} m/s")
    print(f"Drone : {config.DRONE_ID} | Bouées : {list(config.BUOYS_GPS)}")

    # Position initiale : 30 m à l'est de la bouée A (côté ouvert de la porte)
    start_gps = geo_utils.move_meters(config.BUOYS_GPS["A"], 30.0, -15.0)
    boat = SimBoat(start_pos_gps=start_gps)
    course = CourseManager()
    pid = HeadingPID()

    dt = 0.1
    t = 0.0
    log_interval = 5.0
    last_log = -log_interval

    while t < max_time_s and not course.race_finished:
        boat_pos = tuple(boat.pos)
        # Le simulateur a une "vérité GPS" parfaite → équivalent RTK_FIXED
        course.update_and_validate(boat_pos, rtk_fixed=True)
        plan = course.plan(boat_pos)
        if plan is None:
            break

        # Cap optimal VMG
        advice = vmg.compute_optimal_heading(
            boat_pos=boat_pos,
            waypoint=plan.target_pos,
            true_wind_dir_deg=wind_dir,
            true_wind_speed_ms=wind_speed,
            current_heading_deg=boat.heading_deg,
        )
        target = advice.target_heading_deg

        # Layline en upwind
        if advice.regime == "UPWIND":
            ll = layline.evaluate(
                boat_pos=boat_pos,
                boat_heading_deg=boat.heading_deg,
                target_buoy_pos=plan.final_pos,
                true_wind_dir_deg=wind_dir,
                true_wind_speed_ms=wind_speed,
            )
            if (ll.should_tack
                    and (t - boat.last_tack_t) > config.MIN_TIME_BETWEEN_TACKS_S):
                target = (advice.tack_options[1] if ll.on_starboard_tack
                          else advice.tack_options[0])
                boat.last_tack_t = t

        # Champ de potentiel — exclure la bouée tout juste validée
        last_validated_buoy = None
        if course.current_idx > 0:
            prev_leg = course.legs[course.current_idx - 1]
            if prev_leg.completed and prev_leg.side != "gate":
                last_validated_buoy = prev_leg.buoy
        nearby_buoys = [b for b in config.BUOYS_GPS
                        if b != last_validated_buoy
                        and geo_utils.distance_m(boat_pos,
                                                 geo_utils.buoy_gps(b)) < 8.0]
        rep = potential_field.build_repulsors([], nearby_buoys)
        if rep:
            f = potential_field.total_force(boat_pos, plan.target_pos, rep, 2.0)
            pf_h = potential_field.force_to_heading(f)
            d = abs(geo_utils.angle_diff_deg(pf_h, target))
            if d > 30:
                # Sécurité : ne pas envoyer le bateau en zone morte
                pf_h = potential_field.safe_heading_against_wind(
                    pf_h, wind_dir, config.POLAR_THETA_MIN_DEG,
                )
                target = pf_h

        # Avancer
        boat.step(target, wind_dir, wind_speed, dt)
        t += dt

        if t - last_log >= log_interval:
            print(
                f"  t={t:6.1f}s  étape={course.current_idx}/{len(course.legs):<2} "
                f"({plan.leg.name})  "
                f"pos=({boat.pos[0]:.6f},{boat.pos[1]:.6f})  "
                f"hdg={boat.heading_deg:5.1f}°  v={boat.speed_ms:.2f}m/s  "
                f"d_wp={plan.distance_m:.1f}m  régime={advice.regime}",
            )
            last_log = t

    print()
    if course.race_finished:
        print(f"✅ Course terminée en {t:.1f}s")
    else:
        print(f"⚠ Timeout — étape atteinte: {course.current_idx}/{len(course.legs)}")
    total_dist = sum(geo_utils.distance_m(boat.history[i], boat.history[i+1])
                     for i in range(len(boat.history)-1))
    print(f"Distance parcourue : {total_dist:.0f} m")

    if plot:
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("matplotlib non disponible — pas de plot")
            return
        # Pour le rendu graphique, on convertit en repère local (E, N)
        # — uniquement esthétique, sans incidence sur la simulation.
        fig, ax = plt.subplots(figsize=(14, 10))
        ax.set_facecolor("#1a6896")
        fig.patch.set_facecolor("#0d3b5e")
        # Bouées
        for name, gps_pos in config.BUOYS_GPS.items():
            local = geo_utils.gps_to_local(*gps_pos)
            ax.add_patch(plt.Circle(local, 4, color="yellow", zorder=5))
            ax.text(local[0], local[1], name, ha="center", va="center",
                    fontsize=10, fontweight="bold", zorder=6)
        # Trajectoire (conversion GPS → local pour matplotlib)
        history_local = [geo_utils.gps_to_local(p[0], p[1]) for p in boat.history]
        xs = [p[0] for p in history_local]
        ys = [p[1] for p in history_local]
        ax.plot(xs, ys, "y-", linewidth=1.5, zorder=3)
        # Porte
        a = geo_utils.gps_to_local(*config.BUOYS_GPS["A"])
        b = geo_utils.gps_to_local(*config.BUOYS_GPS["B"])
        ax.plot([a[0], b[0]], [a[1], b[1]], "g-", linewidth=2, zorder=4)
        # Vent
        ux, uy = geo_utils.cap_to_unit_vector((wind_dir + 180) % 360)  # vers où il VA
        ax.annotate("", xy=(150 + 30*ux, 50 + 30*uy), xytext=(150, 50),
                    arrowprops=dict(arrowstyle="->", color="cyan", lw=2))
        ax.text(150, 60, f"Vent {wind_dir}°\n{wind_speed:.1f} m/s",
                color="cyan", ha="center", fontsize=10)
        ax.set_aspect("equal")
        ax.set_xlabel("East (m) — vue locale", color="white")
        ax.set_ylabel("North (m) — vue locale", color="white")
        ax.set_title(
            f"Simulation Parcours N°3 — {config.DRONE_ID} (runtime en GPS)",
            color="white", fontsize=12,
        )
        ax.tick_params(colors="white")
        out_path = os.path.join(os.path.dirname(__file__), "sim_output.png")
        plt.savefig(out_path,
                    dpi=150, bbox_inches="tight", facecolor="#0d3b5e")
        print(f"Image : {out_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--wind-dir", type=float, default=90.0,
                        help="Direction d'OÙ vient le vent (°), défaut 90 (Est)")
    parser.add_argument("--wind-speed", type=float, default=5.0,
                        help="Vitesse vent en m/s, défaut 5")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()
    simulate(args.wind_dir, args.wind_speed, plot=not args.no_plot)
