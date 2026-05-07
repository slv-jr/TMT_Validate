"""
Tests unitaires de la logique pure (sans matériel).

Lancer en local sur un PC pour vérifier la cohérence des modules :
    python3 -m tests.test_logic_unit
"""

import math
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from navigation import geo_utils, layline, polar, potential_field, vmg
from navigation.heading_pid import HeadingPID
from navigation.waypoints import CourseManager
from comms.protocol import (
    PositionMessage, WindMessage, parse_message, build_position_from_telemetry,
)
from safety.stall_detector import StallDetector, StallLevel
from safety.penalty_manager import PenaltyManager, PenaltyMode
from safety.degraded_modes import DegradedManager, DegradedMode


# ════════════════════════════════════════════════════════════════════════
# GEO
# ════════════════════════════════════════════════════════════════════════
class TestGeoUtils(unittest.TestCase):

    def test_distance_haversine(self):
        p1 = (43.0967, 5.9533)
        p2 = (43.0977, 5.9533)
        d = geo_utils.distance_m(p1, p2)
        self.assertAlmostEqual(d, 111.0, delta=2.0)
        self.assertAlmostEqual(geo_utils.distance_m(p1, p1), 0.0)

    def test_bearing_cardinal(self):
        p_origin = (43.0967, 5.9533)
        p_north = geo_utils.move_meters(p_origin, 0.0, 100.0)
        self.assertAlmostEqual(geo_utils.bearing_deg(p_origin, p_north), 0.0, delta=0.1)
        p_east = geo_utils.move_meters(p_origin, 100.0, 0.0)
        self.assertAlmostEqual(geo_utils.bearing_deg(p_origin, p_east), 90.0, delta=0.1)
        p_south = geo_utils.move_meters(p_origin, 0.0, -100.0)
        self.assertAlmostEqual(geo_utils.bearing_deg(p_origin, p_south), 180.0, delta=0.1)
        p_west = geo_utils.move_meters(p_origin, -100.0, 0.0)
        self.assertAlmostEqual(geo_utils.bearing_deg(p_origin, p_west), 270.0, delta=0.1)

    def test_offset_meters_roundtrip(self):
        p1 = (43.0967, 5.9533)
        for east, north in [(100, 0), (0, 100), (-50, 30), (200, -150)]:
            p2 = geo_utils.move_meters(p1, east, north)
            de, dn = geo_utils.offset_meters(p1, p2)
            self.assertAlmostEqual(de, east, delta=0.05)
            self.assertAlmostEqual(dn, north, delta=0.05)

    def test_destination_point(self):
        p = (43.0967, 5.9533)
        p2 = geo_utils.destination_point(p, 90.0, 100.0)
        d = geo_utils.distance_m(p, p2)
        self.assertAlmostEqual(d, 100.0, delta=0.5)

    def test_gps_local_roundtrip(self):
        lat0 = config.ORIGIN_LAT + 0.001
        lon0 = config.ORIGIN_LON + 0.001
        e, n = geo_utils.gps_to_local(lat0, lon0)
        lat2, lon2 = geo_utils.local_to_gps(e, n)
        self.assertAlmostEqual(lat0, lat2, places=6)
        self.assertAlmostEqual(lon0, lon2, places=6)

    def test_angle_diff(self):
        self.assertAlmostEqual(geo_utils.angle_diff_deg(10, 350), 20.0)
        self.assertAlmostEqual(geo_utils.angle_diff_deg(350, 10), -20.0)
        self.assertAlmostEqual(abs(geo_utils.angle_diff_deg(180, 0)), 180.0)


# ════════════════════════════════════════════════════════════════════════
# CONFIG
# ════════════════════════════════════════════════════════════════════════
class TestConfig(unittest.TestCase):

    def test_capture_radius_rtk_vs_gps(self):
        rtk = config.capture_radius_for_fix(rtk_fixed=True)
        gps = config.capture_radius_for_fix(rtk_fixed=False)
        self.assertEqual(rtk, config.CAPTURE_RADIUS_RTK)
        self.assertEqual(gps, config.CAPTURE_RADIUS_GPS)
        self.assertLess(rtk, gps,
                        msg="Le rayon RTK doit être PLUS PETIT que le rayon GPS")

    def test_drone_profile_consistency(self):
        # Vérifier que les 3 drones ont des slots TDMA distincts
        slots = set(config.TDMA_SLOTS.values())
        self.assertEqual(len(slots), 3)
        self.assertEqual(slots, {0, 500, 1000})


# ════════════════════════════════════════════════════════════════════════
# POLAR
# ════════════════════════════════════════════════════════════════════════
class TestPolar(unittest.TestCase):

    def test_dead_zone(self):
        v = polar.boat_speed_predicted(20.0, 5.0)
        self.assertAlmostEqual(v, 0.0)

    def test_speed_increases_with_wind(self):
        v1 = polar.boat_speed_predicted(60.0, 3.0)
        v2 = polar.boat_speed_predicted(60.0, 6.0)
        self.assertGreater(v2, v1)

    def test_optimal_upwind(self):
        opt = polar.optimal_upwind_angle(5.0)
        self.assertGreater(opt, 35.0)
        self.assertLess(opt, 60.0)

    def test_sail_trim_monotonic(self):
        prev = polar.sail_trim_percent(0)
        for awa in range(10, 180, 10):
            curr = polar.sail_trim_percent(awa)
            self.assertGreaterEqual(curr, prev - 0.01)
            prev = curr


# ════════════════════════════════════════════════════════════════════════
# VMG
# ════════════════════════════════════════════════════════════════════════
class TestVMG(unittest.TestCase):

    def test_upwind_returns_two_options(self):
        boat = (43.0967, 5.9533)
        waypoint = geo_utils.move_meters(boat, 50.0, 0.0)
        advice = vmg.compute_optimal_heading(
            boat_pos=boat, waypoint=waypoint,
            true_wind_dir_deg=90.0, true_wind_speed_ms=5.0,
            current_heading_deg=45.0,
        )
        self.assertEqual(advice.regime, "UPWIND")
        self.assertTrue(advice.needs_tacking)

    def test_reaching(self):
        boat = (43.0967, 5.9533)
        waypoint = geo_utils.move_meters(boat, 0.0, 50.0)
        advice = vmg.compute_optimal_heading(
            boat_pos=boat, waypoint=waypoint,
            true_wind_dir_deg=270.0, true_wind_speed_ms=5.0,
            current_heading_deg=0.0,
        )
        self.assertEqual(advice.regime, "REACHING")

    def test_apparent_wind(self):
        awa, awa_speed = vmg.true_to_apparent_wind(
            true_wind_dir_deg=90.0, true_wind_speed_ms=5.0,
            boat_heading_deg=0.0, boat_speed_ms=0.0,
        )
        self.assertAlmostEqual(awa_speed, 5.0)
        self.assertAlmostEqual(awa, 90.0, places=1)


# ════════════════════════════════════════════════════════════════════════
# PROTOCOL
# ════════════════════════════════════════════════════════════════════════
class TestProtocol(unittest.TestCase):

    def test_position_roundtrip(self):
        msg = PositionMessage("U1B1", 43.48256, 6.49872, 185, 3.2, no_fix=False)
        text = msg.encode()
        self.assertTrue(text.startswith("P|U1B1"))
        parsed = parse_message(text)
        self.assertIsInstance(parsed, PositionMessage)
        self.assertEqual(parsed.boat_id, "U1B1")
        self.assertAlmostEqual(parsed.lat, 43.48256, places=4)
        self.assertAlmostEqual(parsed.lon, 6.49872, places=4)
        self.assertEqual(parsed.heading_deg, 185)

    def test_wind_roundtrip(self):
        msg = WindMessage(245, 6.3, 1746787652, sensor_offline=False)
        text = msg.encode()
        parsed = parse_message(text)
        self.assertIsInstance(parsed, WindMessage)
        self.assertEqual(parsed.direction_deg, 245)
        self.assertAlmostEqual(parsed.speed_ms, 6.3, places=1)

    def test_no_fix(self):
        msg = build_position_from_telemetry("U1B2", 0, 0, 0, 0, has_fix=False)
        self.assertTrue(msg.no_fix)
        text = msg.encode()
        self.assertEqual(text, "P|U1B2|0|0|0|0")

    def test_negative_lat_lon(self):
        msg = PositionMessage("U1B1", -43.48256, -6.49872, 0, 0, no_fix=False)
        text = msg.encode()
        parsed = parse_message(text)
        self.assertAlmostEqual(parsed.lat, -43.48256, places=4)
        self.assertAlmostEqual(parsed.lon, -6.49872, places=4)


# ════════════════════════════════════════════════════════════════════════
# COURSE — rayon adaptatif RTK / GPS
# ════════════════════════════════════════════════════════════════════════
class TestCourse(unittest.TestCase):

    def test_initial_state(self):
        cm = CourseManager()
        self.assertEqual(cm.current_idx, 0)
        self.assertFalse(cm.race_started)
        self.assertEqual(cm.current_leg.name, "DEPART")

    def test_progression(self):
        cm = CourseManager()
        a = config.BUOYS_GPS["A"]
        b = config.BUOYS_GPS["B"]
        gate_mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
        before = geo_utils.move_meters(gate_mid, 30.0, 0.0)
        after = geo_utils.move_meters(gate_mid, -30.0, 0.0)
        cm.update_and_validate(before)
        cm.update_and_validate(after)
        self.assertTrue(cm.race_started)
        self.assertEqual(cm.current_leg.name, "ETAPE_1_C")

    def test_capture_radius_strict_in_rtk(self):
        """En RTK, un point à 5 m d'une bouée NE doit PAS la valider (rayon 4 m)."""
        cm = CourseManager()
        # On force le mode "post-départ"
        cm.current_idx = 1     # ETAPE_1_C
        cm.race_started = True
        c = config.BUOYS_GPS["C"]
        # Approche à 5 m de C (au-delà du rayon RTK 4 m)
        before = geo_utils.move_meters(c, 8.0, 0.0)
        after = geo_utils.move_meters(c, 5.5, 0.0)
        cm.update_and_validate(before, rtk_fixed=True)
        cm.update_and_validate(after, rtk_fixed=True)
        self.assertEqual(cm.current_leg.name, "ETAPE_1_C",
                         msg="Étape ne doit pas être validée à 5.5 m en RTK")

    def test_capture_radius_lax_in_gps(self):
        """En GPS standard (rayon 7 m), un point à 5 m DOIT valider."""
        cm = CourseManager()
        cm.current_idx = 1
        cm.race_started = True
        c = config.BUOYS_GPS["C"]
        # Approche au bord — pour valider il faut aussi le bon côté (starboard
        # = bouée à droite → trajectoire passant à GAUCHE de la bouée, càd
        # cross < 0). Trajet sud→nord avec bouée à l'EST (offset E +5m).
        before = geo_utils.move_meters(c, -5.5, -3.0)   # SO de C
        after = geo_utils.move_meters(c, -5.5, 3.0)     # NO de C, à 5.5 m
        cm.update_and_validate(before, rtk_fixed=False)
        cm.update_and_validate(after, rtk_fixed=False)
        # En GPS, le rayon est 7 m → 5.5 m doit valider l'étape
        self.assertNotEqual(
            cm.current_leg.name, "ETAPE_1_C",
            msg="Étape doit être validée à 5.5 m en GPS standard",
        )


# ════════════════════════════════════════════════════════════════════════
# PID
# ════════════════════════════════════════════════════════════════════════
class TestPID(unittest.TestCase):

    def test_zero_error(self):
        pid = HeadingPID()
        out = pid.compute(target_heading_deg=90.0, current_heading_deg=90.0)
        self.assertAlmostEqual(out, 0.0)

    def test_positive_error_right_rudder(self):
        pid = HeadingPID()
        out = pid.compute(target_heading_deg=100.0, current_heading_deg=90.0)
        self.assertGreater(out, 0)

    def test_saturation(self):
        pid = HeadingPID()
        out = pid.compute(target_heading_deg=180.0, current_heading_deg=0.0)
        self.assertLessEqual(abs(out), config.RUDDER_ANGLE_MAX_DEG + 0.01)


# ════════════════════════════════════════════════════════════════════════
# CHAMP DE POTENTIEL
# ════════════════════════════════════════════════════════════════════════
class TestPotentialField(unittest.TestCase):

    def test_attractive_only(self):
        boat = (43.0967, 5.9533)
        waypoint = geo_utils.move_meters(boat, 10.0, 0.0)
        f = potential_field.attractive_force(boat, waypoint, gain=1.0)
        self.assertAlmostEqual(f[0], 1.0, places=2)
        self.assertAlmostEqual(f[1], 0.0, places=2)

    def test_repulsive_close(self):
        boat = (43.0967, 5.9533)
        rep_pos = geo_utils.move_meters(boat, 2.0, 0.0)
        rep = potential_field.Repulsor(pos=rep_pos, radius_m=4.0, gain=10.0)
        f = potential_field.repulsive_force(boat, rep)
        self.assertLess(f[0], 0)


# ════════════════════════════════════════════════════════════════════════
# STALL DETECTOR
# ════════════════════════════════════════════════════════════════════════
class TestStallDetector(unittest.TestCase):

    def test_no_stall_if_only_one_condition(self):
        sd = StallDetector()
        # Vitesse basse uniquement (1 condition / 3) — pas bloqué
        st = sd.update(boat_speed_ms=0.05, rudder_cmd_deg=2.0,
                       distance_to_wp_m=42.0)
        self.assertFalse(st.is_stalled)

    def test_stall_after_duration(self):
        sd = StallDetector()
        # 3 conditions vraies mais sans avoir tenu la durée → encore "False"
        st1 = sd.update(boat_speed_ms=0.05, rudder_cmd_deg=20.0,
                        distance_to_wp_m=42.0)
        self.assertFalse(st1.is_stalled,
                         msg="Pas encore officiellement stalled (durée < 3s)")
        # Forcer dans le passé pour simuler le délai
        sd._stall_start_t = time.monotonic() - (config.STALL_DURATION_S + 0.5)
        # Refaire un update — le distance_frozen demande l'historique cohérent
        # On bourre l'historique avec des distances quasi-identiques
        sd._history.clear()
        t = time.monotonic()
        for k in range(int(config.STALL_WINDOW_S * 5)):
            sd._history.append((t - config.STALL_WINDOW_S + k * 0.2, 42.0))
        st2 = sd.update(boat_speed_ms=0.05, rudder_cmd_deg=20.0,
                        distance_to_wp_m=42.0)
        self.assertTrue(st2.is_stalled)
        self.assertIn(st2.level, (StallLevel.LIGHT, StallLevel.MEDIUM, StallLevel.HARD))

    def test_reset_on_resolve(self):
        sd = StallDetector()
        # Simule un blocage déjà détecté
        sd._stall_start_t = time.monotonic() - 5.0
        # Boat repart → on doit ressortir
        st = sd.update(boat_speed_ms=2.0, rudder_cmd_deg=1.0,
                       distance_to_wp_m=10.0)
        self.assertFalse(st.is_stalled)
        self.assertEqual(sd._stall_start_t, 0.0)


# ════════════════════════════════════════════════════════════════════════
# PENALTY MANAGER
# ════════════════════════════════════════════════════════════════════════
class TestPenaltyManager(unittest.TestCase):

    def test_inactive_by_default(self):
        pm = PenaltyManager()
        self.assertFalse(pm.is_active)
        self.assertEqual(pm.mode, PenaltyMode.INACTIVE)

    def test_start_enters_wait_mode(self):
        pm = PenaltyManager()
        pm.start(interrupted_leg_idx=4)
        self.assertEqual(pm.mode, PenaltyMode.WAIT)
        self.assertTrue(pm.is_active)
        self.assertEqual(pm.interrupted_leg_idx, 4)

    def test_wait_to_manual_when_pilot_takes_over(self):
        pm = PenaltyManager()
        pm.start()
        boat = config.BUOYS_GPS["Z1"]
        st = pm.update(boat_pos=boat, control_mode_is_manual=True,
                       rtk_fixed=True)
        self.assertEqual(st.mode, PenaltyMode.MANUAL)

    def test_wait_to_auto_after_timeout(self):
        pm = PenaltyManager()
        pm.start()
        # Forcer le timeout
        pm._t0 = time.monotonic() - (config.PENALTY_DECISION_TIMEOUT_S + 0.5)
        st = pm.update(boat_pos=config.BUOYS_GPS["A"],
                       control_mode_is_manual=False, rtk_fixed=True)
        self.assertEqual(st.mode, PenaltyMode.AUTO)

    def test_auto_advances_through_legs(self):
        pm = PenaltyManager()
        pm.start()
        pm._mode = PenaltyMode.AUTO
        pm._t0 = time.monotonic() - 10
        # Étape 1 = Z1, on se place dessus
        st = pm.update(boat_pos=config.BUOYS_GPS["Z1"],
                       control_mode_is_manual=False, rtk_fixed=True)
        # On doit avoir avancé d'une étape OU être en cours de validation
        self.assertGreaterEqual(st.current_leg_idx, 1)

    def test_reset(self):
        pm = PenaltyManager()
        pm.start()
        pm.reset()
        self.assertFalse(pm.is_active)


# ════════════════════════════════════════════════════════════════════════
# DEGRADED MODES — RTK / ADVERSARY_SILENT
# ════════════════════════════════════════════════════════════════════════
class TestDegradedModes(unittest.TestCase):

    def test_nominal(self):
        dm = DegradedManager()
        st = dm.update(
            has_gps_fix=True, gps_fix_type=6,
            last_gps_age_s=0.5, last_lora_msg_age_s=2.0,
            wind_age_s=10.0, wind_confident=True,
            battery_pct=80.0, mavlink_alive=True,
        )
        self.assertEqual(st.severity, 0)
        self.assertTrue(st.rtk_fixed)

    def test_rtk_degraded(self):
        dm = DegradedManager()
        st = dm.update(
            has_gps_fix=True, gps_fix_type=3,        # 3D fix mais pas RTK
            last_gps_age_s=0.5, last_lora_msg_age_s=2.0,
            wind_age_s=10.0, wind_confident=True,
            battery_pct=80.0, mavlink_alive=True,
        )
        self.assertTrue(st.has(DegradedMode.RTK_DEGRADED))
        self.assertFalse(st.rtk_fixed)

    def test_lora_lost_short_threshold(self):
        dm = DegradedManager()
        st = dm.update(
            has_gps_fix=True, gps_fix_type=6,
            last_gps_age_s=0.5,
            last_lora_msg_age_s=15.0,    # > 10 s → LORA_LOST
            wind_age_s=10.0, wind_confident=True,
            battery_pct=80.0, mavlink_alive=True,
        )
        self.assertTrue(st.has(DegradedMode.LORA_LOST))

    def test_adversary_silent(self):
        dm = DegradedManager()
        st = dm.update(
            has_gps_fix=True, gps_fix_type=6,
            last_gps_age_s=0.5, last_lora_msg_age_s=2.0,
            wind_age_s=10.0, wind_confident=True,
            battery_pct=80.0, mavlink_alive=True,
            adversary_silent=True,
        )
        self.assertTrue(st.has(DegradedMode.ADVERSARY_SILENT))


if __name__ == "__main__":
    unittest.main(verbosity=2)
