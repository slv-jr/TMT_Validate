"""
StormWings — point d'entrée principal (cf. README2).

Boucle 10 Hz :
    1. Lecture télémétrie (MAVLink) + estimation vent (LoRa Calypso)
    2. Détection mode dégradé (GPS, RTK, LoRa, MAVLink, batterie)
    3. Bascule MANUEL ↔ AUTO selon CH3
    4. AUTO → décision navigation (priorité décroissante) :
        a) PENALITE active → on suit la séquence Z1/Z2/Z1
        b) STALL détecté   → on déclenche la manœuvre de dégagement
        c) Sinon           → VMG / layline / champ de potentiel / PID
    5. Push override CH1/CH2 (et CH4 boost) en continu à 10 Hz
    6. Comm LoRa : broadcast P|... selon le slot TDMA (60 s décalés)
    7. Logging CSV horodaté

Lancement :
    DRONE_ID=U1B1 python3 main.py        # drone Scout (avec boost)
    DRONE_ID=U1B2 python3 main.py        # drone Optimizer
    DRONE_ID=U1B3 python3 main.py        # drone Safety
"""

from __future__ import annotations

import logging
import math
import os
import signal
import sys
import time
from typing import Dict, Optional, Tuple

# Permettre l'import des sous-modules en mode développement
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config
from boost.boost_controller import BoostController, BoostState
from comms.lora_iface import LoRaInterface
from comms.mavlink_iface import MavlinkInterface
from navigation import geo_utils, layline, polar, potential_field, vmg
from navigation.heading_pid import HeadingPID
from navigation.state_machine import NavState, NavStateMachine
from navigation.waypoints import CourseManager
from safety.degraded_modes import DegradedManager, DegradedMode
from safety.logger import FlightLogger
from safety.mode_switch import ControlMode, ModeSwitch
from safety.penalty_manager import PenaltyManager, PenaltyMode
from safety.stall_detector import StallDetector, StallLevel
from swarm.roles import RoleManager
from swarm.tdma import TDMAScheduler
from wind.wind_estimator import WindEstimator

GPSPos = Tuple[float, float]


# ──────────────────────────────────────────────────────────────────
# Logging setup
# ──────────────────────────────────────────────────────────────────
def setup_logging():
    log_dir = config.LOG_DIR
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, f"stormwings_{config.DRONE_ID}.log")
    logging.basicConfig(
        level=getattr(logging, config.LOG_LEVEL, logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        handlers=[
            logging.FileHandler(log_path),
            logging.StreamHandler(sys.stdout),
        ],
    )


# ──────────────────────────────────────────────────────────────────
# Application
# ──────────────────────────────────────────────────────────────────
class StormWingsApp:
    """Application principale."""

    def __init__(self):
        self.log = logging.getLogger("StormWings")
        self.running = False

        # Composants matériels
        self.mav = MavlinkInterface()
        self.lora: Optional[LoRaInterface] = None

        # Composants logiques
        self.nav_sm = NavStateMachine()
        self.course = CourseManager()
        self.heading_pid = HeadingPID()
        self.mode_switch: Optional[ModeSwitch] = None
        self.wind = WindEstimator()
        self.roles = RoleManager()
        self.tdma = TDMAScheduler()
        self.degraded = DegradedManager()
        self.boost = BoostController(self.mav)
        self.flight_log = FlightLogger()
        self.penalty = PenaltyManager()
        self.stall = StallDetector()

        # Compteurs / timing
        self._last_tack_t: float = 0.0
        self._last_override_push_t: float = 0.0
        self._last_heartbeat_t: float = 0.0
        self._last_sail_pct: float = 50.0
        self._target_heading_deg: float = 0.0
        self._last_rudder_cmd_deg: float = 0.0
        self._last_role_eval_t: float = 0.0
        self._last_lora_msg_t: float = time.monotonic()
        # Tracking adversaires : dernière position connue par bateau
        self._last_adversary_pos: Dict[str, Tuple[GPSPos, float]] = {}

    # ─────────────────────────────────────────────
    # Setup / Teardown
    # ─────────────────────────────────────────────
    def setup(self) -> bool:
        self.log.info("=" * 60)
        self.log.info("StormWings %s — démarrage", config.DRONE_ID)
        self.log.info("Rôle %s | Strategy %s | Boost %s",
                      config.DEFAULT_ROLE, config.STRATEGY,
                      config.HAS_BOOST_MOTOR)
        self.log.info("RTK radius=%.1f m / GPS radius=%.1f m | "
                      "BOOST_MAX=%.0fs/%d actions",
                      config.CAPTURE_RADIUS_RTK, config.CAPTURE_RADIUS_GPS,
                      config.BOOST_MAX_S, config.BOOST_ACTIONS_MAX)
        self.log.info("=" * 60)

        # MAVLink (obligatoire)
        if not self.mav.connect(timeout_s=30.0):
            self.log.error("Impossible de se connecter au Cube Orange+")
            return False
        self.mode_switch = ModeSwitch(self.mav)

        # LoRa (optionnel mais fortement recommandé)
        try:
            self.lora = LoRaInterface()
            if not self.lora.connect():
                self.log.warning(
                    "LoRa indisponible — navigation sans coordination essaim"
                )
                self.lora = None
            else:
                # Branchement des callbacks
                self.lora.on_wind = self._on_wind_received
                self.lora.on_position = self._on_position_received
        except Exception as e:
            self.log.warning("LoRa init échoué : %s", e)
            self.lora = None

        # Logger CSV
        self.flight_log.open()

        # Initialisation des composants
        self.nav_sm.transition(NavState.INIT, "boot")
        return True

    def teardown(self):
        self.log.info("Arrêt en cours…")
        try:
            self.mav.clear_all_overrides()
        except Exception:
            pass
        try:
            self.flight_log.close()
        except Exception:
            pass
        try:
            if self.lora is not None:
                self.lora.close()
        except Exception:
            pass
        try:
            self.mav.close()
        except Exception:
            pass
        self.log.info("StormWings %s — arrêt complet", config.DRONE_ID)

    # ─────────────────────────────────────────────
    # Callbacks LoRa
    # ─────────────────────────────────────────────
    def _on_wind_received(self, msg):
        self._last_lora_msg_t = time.monotonic()
        self.wind.push_calypso(
            direction_deg=msg.direction_deg,
            speed_ms=msg.speed_ms,
            timestamp=msg.timestamp,
            sensor_offline=msg.sensor_offline,
        )

    def _on_position_received(self, msg):
        """Callback LoRa : un voisin (allié ou ennemi) annonce sa position."""
        self._last_lora_msg_t = time.monotonic()
        if msg.no_fix:
            return
        if config.is_teammate(msg.boat_id):
            spd_ms = msg.speed_knots / 1.94384
            self.roles.update_teammate(
                boat_id=msg.boat_id,
                pos_gps=(msg.lat, msg.lon),
                speed_ms=spd_ms,
                has_fix=not msg.no_fix,
            )
        elif config.is_enemy(msg.boat_id):
            self._last_adversary_pos[msg.boat_id] = (
                (msg.lat, msg.lon), time.monotonic(),
            )

    # ─────────────────────────────────────────────
    # Boucle principale
    # ─────────────────────────────────────────────
    def run(self):
        if self.mode_switch is None:
            raise RuntimeError("setup() doit être appelé avant run()")

        self.running = True
        next_tick = time.monotonic()

        try:
            while self.running:
                self._tick()
                # Cadence 10 Hz
                next_tick += config.NAV_DT
                sleep = next_tick - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_tick = time.monotonic()
        except KeyboardInterrupt:
            self.log.info("Interruption clavier")
        finally:
            self.teardown()

    def _tick(self):
        now = time.monotonic()

        # ===== 1. LECTURE TÉLÉMÉTRIE =====
        tlm = self.mav.get_telemetry()
        wind_est = self.wind.estimate()
        rtk_fixed = tlm.has_gps_fix and tlm.fix_type >= 5

        boat_pos: GPSPos = (
            (tlm.lat, tlm.lon) if tlm.has_gps_fix else (0.0, 0.0)
        )

        # ===== 2. HEARTBEAT GCS (anti-failsafe) =====
        if now - self._last_heartbeat_t >= 1.0:
            self.mav.send_heartbeat()
            self._last_heartbeat_t = now

        # ===== 3. ÉTAT DÉGRADÉ =====
        last_gps_age = (now - tlm.last_gps_t) if tlm.last_gps_t > 0 else 1e9
        lora_msg_age = now - self._last_lora_msg_t
        adversary_silent = self._has_silent_adversary(now)
        deg = self.degraded.update(
            has_gps_fix=tlm.has_gps_fix,
            gps_fix_type=tlm.fix_type,
            last_gps_age_s=last_gps_age,
            last_lora_msg_age_s=lora_msg_age,
            wind_age_s=wind_est.age_s,
            wind_confident=wind_est.confident,
            battery_pct=tlm.battery_remaining_pct,
            mavlink_alive=self.mav.is_alive(),
            stall_detected=False,   # rempli après évaluation StallDetector
            adversary_silent=adversary_silent,
        )

        # Si MAVLink perdu → on n'envoie plus rien (sécurité)
        if deg.has(DegradedMode.MAVLINK_LOST):
            self.log.warning("MAVLink perdu — pause des commandes")
            self.nav_sm.transition(NavState.DEGRADE, "mavlink_lost")
            self._comm_step(tlm, now)
            self._log_step(tlm, wind_est, deg, now)
            return

        # ===== 4. BASCULE MANUEL/AUTO =====
        mode_status = self.mode_switch.update()

        # ===== 5. ROUTAGE ÉTAT NAVIGATION =====
        if mode_status.mode == ControlMode.MANUAL:
            self.nav_sm.transition(NavState.REPRISE_RC, "ch3 haut")
            self._update_self_state(boat_pos, tlm)
            # Si une pénalité est active, le manager doit savoir que le
            # pilote a bien pris la main (sous-état MANUAL)
            if self.penalty.is_active and tlm.has_gps_fix:
                self.penalty.update(boat_pos, control_mode_is_manual=True,
                                    rtk_fixed=rtk_fixed)
        elif mode_status.mode == ControlMode.UNKNOWN:
            self.nav_sm.transition(NavState.ATTENTE, "ch3 inconnu")
            self.mav.clear_all_overrides()
        else:
            # AUTO
            self._auto_step(boat_pos, tlm, wind_est, rtk_fixed, now)

        # ===== 6. ENVOI DES OVERRIDES (10 Hz) =====
        if self.mode_switch.is_auto:
            if now - self._last_override_push_t >= 1.0 / config.OVERRIDE_REFRESH_HZ:
                self.mav.push_overrides()
                self._last_override_push_t = now

        # ===== 7. COMM LORA =====
        self._comm_step(tlm, now)

        # ===== 8. LOGGING =====
        self._log_step(tlm, wind_est, deg, now)

    # ─────────────────────────────────────────────
    # Étape AUTO complète (cœur de la stratégie)
    # ─────────────────────────────────────────────
    def _auto_step(self, boat_pos: GPSPos, tlm, wind_est, rtk_fixed: bool,
                   now: float):
        # Mise à jour de l'état du parcours (rayon adaptatif RTK/GPS)
        if tlm.has_gps_fix:
            self.course.update_and_validate(boat_pos, rtk_fixed=rtk_fixed)
            self._update_self_state(boat_pos, tlm)

        # Réévaluation du rôle
        if now - self._last_role_eval_t >= config.ROLE_REEVAL_PERIOD_S:
            self.roles.reevaluate()
            self._last_role_eval_t = now

        # ── PRIORITÉ 1 — pénalité active ──
        if self.penalty.is_active and tlm.has_gps_fix:
            self.nav_sm.transition(NavState.PENALITE, "pénalité en cours")
            pst = self.penalty.update(boat_pos, control_mode_is_manual=False,
                                      rtk_fixed=rtk_fixed)
            if pst.finished:
                self.log.warning(
                    "[PENALTY] Terminée — reprise du parcours à l'étape %d",
                    self.penalty.interrupted_leg_idx,
                )
                self.penalty.reset()
            elif pst.target_pos is not None:
                # On pilote vers la bouée pénalité (réutilise la même chaîne
                # VMG/anti-collision/PID que le parcours normal)
                self._navigate_to(
                    boat_pos=boat_pos,
                    waypoint=pst.target_pos,
                    final_pos=pst.target_pos,
                    tlm=tlm, wind_est=wind_est, now=now,
                    label=f"penalty:{pst.current_leg_idx+1}/{pst.total_legs}",
                )
            return

        # Course terminée ?
        if self.course.race_finished:
            self.nav_sm.transition(NavState.FIN_COURSE, "porte arrivée")
            self.mav.set_rudder_pwm(config.RUDDER_PWM_TRIM)
            self.mav.set_sail_percent(50.0)
            return

        # Pas encore démarrée ?
        if not self.course.race_started:
            self.nav_sm.transition(NavState.ATTENTE, "pré-départ")
            self._auto_idle(tlm, wind_est)
            return

        # ── PRIORITÉ 2 — stall (blocage comportemental) ──
        plan = self.course.plan(boat_pos)
        if plan is None:
            self.log.warning("Pas de plan disponible")
            return
        stall_status = self.stall.update(
            boat_speed_ms=tlm.ground_speed_ms,
            rudder_cmd_deg=self._last_rudder_cmd_deg,
            distance_to_wp_m=plan.distance_m,
        )
        if stall_status.is_stalled:
            self.nav_sm.transition(NavState.STALL_RECOVERY,
                                   f"stall {stall_status.level.value}")
            self._handle_stall(stall_status, plan, boat_pos, tlm, wind_est, now)
            return

        # ── PRIORITÉ 3 — navigation normale vers le waypoint ──
        self._navigate_to(
            boat_pos=boat_pos,
            waypoint=plan.target_pos,
            final_pos=plan.final_pos,
            tlm=tlm, wind_est=wind_est, now=now,
            label=plan.leg.name,
        )

    def _navigate_to(self, boat_pos: GPSPos, waypoint: GPSPos, final_pos: GPSPos,
                     tlm, wind_est, now: float, label: str):
        """Stratégie commune de navigation (utilisée pour course et pénalité)."""
        # ── Calcul cap optimal VMG ──
        advice = vmg.compute_optimal_heading(
            boat_pos=boat_pos,
            waypoint=waypoint,
            true_wind_dir_deg=wind_est.direction_deg,
            true_wind_speed_ms=wind_est.speed_ms,
            current_heading_deg=tlm.heading_deg,
        )

        if advice.regime == "UPWIND":
            self.nav_sm.transition(NavState.REMONTEE_VENT, label)
        elif advice.regime == "DOWNWIND":
            self.nav_sm.transition(NavState.DESCENTE, label)
        elif advice.regime == "REACHING":
            self.nav_sm.transition(NavState.REACHING, label)

        target_heading = advice.target_heading_deg

        # ── Layline / décision tack ──
        if advice.regime == "UPWIND":
            ll = layline.evaluate(
                boat_pos=boat_pos,
                boat_heading_deg=tlm.heading_deg,
                target_buoy_pos=final_pos,
                true_wind_dir_deg=wind_est.direction_deg,
                true_wind_speed_ms=wind_est.speed_ms,
            )
            time_since_last_tack = now - self._last_tack_t
            if (ll.should_tack
                    and time_since_last_tack > config.MIN_TIME_BETWEEN_TACKS_S):
                target_heading = (
                    advice.tack_options[1] if ll.on_starboard_tack
                    else advice.tack_options[0]
                )
                self._last_tack_t = now
                self.log.info("[NAV] Tack forcé (layline atteinte) → cap %.0f°",
                              target_heading)

        # ── Anti-collision champ de potentiel ──
        repulsors = self._build_repulsors(boat_pos, now)

        if potential_field.emergency_avoidance_needed(
            boat_pos, repulsors, threshold_m=3.0,
        ):
            self.nav_sm.transition(NavState.EVITEMENT_URGENCE, "obstacle <3m")
            f = potential_field.total_force(
                boat_pos=boat_pos, waypoint=waypoint,
                repulsors=repulsors, attractive_gain=0.5,
            )
            target_heading = potential_field.force_to_heading(f)
            target_heading = potential_field.safe_heading_against_wind(
                target_heading, wind_est.direction_deg,
                config.POLAR_THETA_MIN_DEG,
            )
        elif repulsors:
            f = potential_field.total_force(
                boat_pos=boat_pos, waypoint=waypoint,
                repulsors=repulsors, attractive_gain=2.0,
            )
            pf_heading = potential_field.force_to_heading(f)
            diff = geo_utils.angle_diff_deg(pf_heading, target_heading)
            if abs(diff) > 30:
                target_heading = potential_field.safe_heading_against_wind(
                    pf_heading, wind_est.direction_deg,
                    config.POLAR_THETA_MIN_DEG,
                )

        # ── PID cap → angle safran ──
        rudder_cmd_deg = self.heading_pid.compute(
            target_heading_deg=target_heading,
            current_heading_deg=tlm.heading_deg,
        )
        self.mav.set_rudder_angle_deg(rudder_cmd_deg)
        self._last_rudder_cmd_deg = rudder_cmd_deg
        self._target_heading_deg = target_heading

        # ── Réglage voile selon angle vent apparent ──
        awa_deg, _ = vmg.true_to_apparent_wind(
            true_wind_dir_deg=wind_est.direction_deg,
            true_wind_speed_ms=wind_est.speed_ms,
            boat_heading_deg=tlm.heading_deg,
            boat_speed_ms=tlm.ground_speed_ms,
        )
        sail_pct = polar.sail_trim_percent(awa_deg)
        sail_pct = 0.7 * self._last_sail_pct + 0.3 * sail_pct
        self._last_sail_pct = sail_pct
        self.mav.set_sail_percent(sail_pct)

        # ── Boost auto ──
        role_mods = self.roles.role_modifies_strategy()
        in_dead = polar.is_dead_zone(
            geo_utils.angle_diff_deg(target_heading, wind_est.direction_deg)
        )
        self.boost.update(
            boat_speed_ms=tlm.ground_speed_ms,
            wind_speed_ms=wind_est.speed_ms,
            battery_pct=tlm.battery_remaining_pct,
        )
        self.boost.auto_check(
            boat_speed_ms=tlm.ground_speed_ms,
            wind_speed_ms=wind_est.speed_ms,
            battery_pct=tlm.battery_remaining_pct,
            in_dead_zone=in_dead,
            role_authorizes=role_mods["use_boost_authorized"],
        )

    def _handle_stall(self, stall_status, plan, boat_pos, tlm, wind_est,
                      now: float):
        """Réaction palier par palier (cf. README2 §"Détection blocage")."""
        level = stall_status.level
        # Cap "fuite" : virer franchement de bord
        # On choque à fond la voile et on tente le bord opposé du cap actuel
        opposite = (tlm.heading_deg + 180.0) % 360.0
        # Mais en évitant la zone morte
        target_heading = potential_field.safe_heading_against_wind(
            opposite, wind_est.direction_deg, config.POLAR_THETA_MIN_DEG,
        )
        rudder_cmd = self.heading_pid.compute(
            target_heading_deg=target_heading,
            current_heading_deg=tlm.heading_deg,
        )
        self.mav.set_rudder_angle_deg(rudder_cmd)
        self._last_rudder_cmd_deg = rudder_cmd
        self._target_heading_deg = target_heading
        # Voile choquée à fond (recherche de poussée même mauvaise)
        self.mav.set_sail_percent(80.0)
        self._last_sail_pct = 80.0

        # Escalade : niveau MEDIUM = boost si dispo et autorisé
        if level == StallLevel.MEDIUM:
            role_mods = self.roles.role_modifies_strategy()
            if (self.boost.has_boost
                    and role_mods["use_boost_authorized"]
                    and self.boost.can_boost()):
                self.boost.request_boost(
                    boat_speed_ms=tlm.ground_speed_ms,
                    wind_speed_ms=wind_est.speed_ms,
                    battery_pct=tlm.battery_remaining_pct,
                    reason="stall-medium",
                )

        if level == StallLevel.HARD:
            # Le drone est vraiment coincé — log warning seulement
            self.log.warning(
                "[STALL] Niveau HARD persistant (%.1fs) — proposer reprise RC",
                stall_status.stalled_since_s,
            )

    def _auto_idle(self, tlm, wind_est):
        """En attente de départ : safran droit, voile mi-position."""
        self.mav.set_rudder_pwm(config.RUDDER_PWM_TRIM)
        self.mav.set_sail_percent(60.0)
        self._last_rudder_cmd_deg = 0.0

    def _update_self_state(self, boat_pos: GPSPos, tlm):
        self.roles.update_self(
            leg_index=self.course.current_idx,
            pos_gps=boat_pos,
            speed_ms=tlm.ground_speed_ms,
            battery_pct=tlm.battery_remaining_pct,
            has_fix=tlm.has_gps_fix,
        )

    # ─────────────────────────────────────────────
    # Construction des répulseurs (alliés + ennemis + bouées)
    # ─────────────────────────────────────────────
    def _build_repulsors(self, boat_pos: GPSPos, now: float):
        all_neighbors_pos = []
        if self.lora is not None:
            for bid, ns in self.lora.get_active_neighbors().items():
                if ns.has_fix:
                    all_neighbors_pos.append((bid, (ns.lat, ns.lon)))
        # Adversaires silencieux : on les traite comme obstacles fixes à
        # leur dernière position connue tant qu'on est dans le timeout
        for bid, (pos, t) in list(self._last_adversary_pos.items()):
            if now - t > 5 * config.ADVERSARY_SILENT_TIMEOUT_S:
                # Trop ancien — on oublie
                del self._last_adversary_pos[bid]
                continue
            if (bid, pos) not in [(b, p) for b, p in all_neighbors_pos]:
                all_neighbors_pos.append((bid + ":silent", pos))

        # Bouées proches, sauf celle qu'on vient de valider
        last_validated_buoy = None
        if self.course.current_idx > 0:
            prev_leg = self.course.legs[self.course.current_idx - 1]
            if prev_leg.completed and prev_leg.side != "gate":
                last_validated_buoy = prev_leg.buoy
        nearby_buoys = []
        for buoy_name in config.BUOYS_GPS:
            if buoy_name == last_validated_buoy:
                continue
            bp = geo_utils.buoy_gps(buoy_name)
            if geo_utils.distance_m(boat_pos, bp) < 10.0:
                nearby_buoys.append(buoy_name)

        return potential_field.build_repulsors(
            other_drones_positions=all_neighbors_pos,
            nearby_buoys=nearby_buoys,
        )

    def _has_silent_adversary(self, now: float) -> bool:
        """True si au moins 1 adversaire connu n'a pas émis depuis > timeout."""
        for bid, (_, t) in self._last_adversary_pos.items():
            age = now - t
            if config.ADVERSARY_SILENT_TIMEOUT_S < age < 5 * config.ADVERSARY_SILENT_TIMEOUT_S:
                return True
        return False

    # ─────────────────────────────────────────────
    # API publique : déclencher une pénalité
    # ─────────────────────────────────────────────
    def request_penalty(self, reason: str = "manual"):
        """Hook externe (CLI, GCS, GPIO bouton) pour déclencher une pénalité.

        L'option par défaut suspend le parcours à l'étape courante et lance
        la séquence Z1/Z2/Z1.
        """
        self.log.warning("[PENALTY] Déclenchée (raison=%s)", reason)
        self.penalty.start(interrupted_leg_idx=self.course.current_idx)

    # ─────────────────────────────────────────────
    # Communication LoRa
    # ─────────────────────────────────────────────
    def _comm_step(self, tlm, now: float):
        if self.lora is None:
            return
        if not self.tdma.should_transmit():
            return
        try:
            self.lora.broadcast_position(
                lat=tlm.lat,
                lon=tlm.lon,
                heading_deg=tlm.heading_deg,
                speed_ms=tlm.ground_speed_ms,
                has_fix=tlm.has_gps_fix,
            )
        except Exception as e:
            self.log.warning("LoRa broadcast échoué : %s", e)

    # ─────────────────────────────────────────────
    # Logging
    # ─────────────────────────────────────────────
    def _log_step(self, tlm, wind_est, deg, now: float):
        try:
            leg = self.course.current_leg
            boat_pos_gps = (
                (tlm.lat, tlm.lon) if tlm.has_gps_fix else (0.0, 0.0)
            )
            plan = self.course.plan(boat_pos_gps)
            if tlm.has_gps_fix:
                east, north = geo_utils.gps_to_local(tlm.lat, tlm.lon)
            else:
                east, north = 0.0, 0.0
            row = {
                "nav_state": self.nav_sm.state.value,
                "control_mode": self.mode_switch.update().mode.value
                if self.mode_switch else "?",
                "role": self.roles.my_role,
                "leg_index": self.course.current_idx,
                "leg_name": leg.name if leg else "—",
                "lat": tlm.lat,
                "lon": tlm.lon,
                "east_m": east,
                "north_m": north,
                "fix_type": tlm.fix_type,
                "rtk_fixed": tlm.has_gps_fix and tlm.fix_type >= 5,
                "heading_deg": tlm.heading_deg,
                "speed_ms": tlm.ground_speed_ms,
                "roll_deg": tlm.roll_deg,
                "wp_distance_m": plan.distance_m if plan else 0.0,
                "wp_bearing_deg": plan.bearing_deg if plan else 0.0,
                "target_heading_deg": self._target_heading_deg,
                "rudder_cmd_deg": self._last_rudder_cmd_deg,
                "rudder_pwm": self.mav._override_rudder_pwm,
                "sail_pwm": self.mav._override_sail_pwm,
                "sail_pct": self._last_sail_pct,
                "wind_dir_deg": wind_est.direction_deg,
                "wind_speed_ms": wind_est.speed_ms,
                "wind_age_s": wind_est.age_s,
                "wind_source": wind_est.source,
                "battery_pct": tlm.battery_remaining_pct,
                "boost_state": self.boost.state.value,
                "boost_seconds_used": (
                    self.boost.update(
                        boat_speed_ms=tlm.ground_speed_ms,
                        wind_speed_ms=wind_est.speed_ms,
                        battery_pct=tlm.battery_remaining_pct,
                    ).seconds_used
                ),
                "boost_actions_used": self.boost._activations_used,
                "penalty_mode": self.penalty.mode.value,
                "penalty_progress": self.penalty.progress_str(),
                "neighbors_count": len(
                    self.lora.get_team_neighbors() if self.lora else {}
                ),
                "degraded_modes": "|".join(m.value for m in deg.active_modes),
            }
            self.flight_log.log(row)
        except Exception as e:
            self.log.debug("Log err : %s", e)

    # ─────────────────────────────────────────────
    # Signaux Unix
    # ─────────────────────────────────────────────
    def install_signal_handlers(self):
        def handler(signum, frame):
            self.log.info("Signal %d reçu — arrêt", signum)
            self.running = False
        signal.signal(signal.SIGINT, handler)
        if hasattr(signal, "SIGTERM"):
            signal.signal(signal.SIGTERM, handler)


# ──────────────────────────────────────────────────────────────────
# Entrée
# ──────────────────────────────────────────────────────────────────
def main():
    setup_logging()
    app = StormWingsApp()
    app.install_signal_handlers()
    if not app.setup():
        sys.exit(1)
    app.run()


if __name__ == "__main__":
    main()
