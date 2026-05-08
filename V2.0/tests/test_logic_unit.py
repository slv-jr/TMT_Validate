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
        # Régate à 2 drones : on doit avoir exactement 2 slots TDMA distincts
        slots = set(config.TDMA_SLOTS.values())
        self.assertEqual(len(slots), 2,
                         msg="Régate à 2 drones : 2 slots TDMA attendus")
        self.assertEqual(set(config.TEAM_BOATS), {"U1B1", "U1B2"})

    def test_modes_essai_regate_helpers(self):
        # is_essai() et is_regate() sont mutuellement exclusifs
        self.assertNotEqual(config.is_essai(), config.is_regate())

    def test_courses_definition(self):
        # Les 2 parcours doivent commencer et finir par une porte
        for cn in (1, 2):
            legs = config.course_legs_for(cn)
            self.assertEqual(legs[0]["side"], "gate",
                             msg=f"Parcours {cn} : 1ère étape doit être une porte")
            self.assertEqual(legs[-1]["side"], "gate",
                             msg=f"Parcours {cn} : dernière étape doit être une porte")
            self.assertEqual(legs[0]["name"], "DEPART")
            self.assertEqual(legs[-1]["name"], "ARRIVEE")

    def test_only_two_courses_supported(self):
        """Le parcours 3 (côtier long) a été supprimé — seuls 1 et 2 existent."""
        self.assertEqual(set(config._COURSES_LEGS.keys()), {1, 2})
        # Les bouées F, G, A-E, Z1, Z2 ne sont plus dans BUOYS_GPS (briefing du 8/5)
        for legacy in ("A", "B", "C", "D", "E", "F", "G", "Z1", "Z2"):
            self.assertNotIn(legacy, config.BUOYS_GPS,
                             msg=f"{legacy} ne doit plus exister")

    def test_course_1_has_4_buoys_plus_penalty(self):
        used = config.buoys_used_in_course(1)
        # Doit contenir au moins 1, 2, 3, 4 + P1, P2
        for name in ["1", "2", "3", "4", "P1", "P2"]:
            self.assertIn(name, used,
                          msg=f"Parcours 1 doit utiliser {name}")
        # La bouée 5 est exclusive au parcours 2
        self.assertNotIn("5", used)

    def test_course_2_has_5_buoys_plus_penalty(self):
        """Nouveau parcours 2 (briefing du 8/5/2026) : bouées 1, 2, 3, 4, 5
        + pénalité P1/P2 (commune avec le parcours 1)."""
        used = config.buoys_used_in_course(2)
        for name in ["1", "2", "3", "4", "5", "P1", "P2"]:
            self.assertIn(name, used,
                          msg=f"Parcours 2 doit utiliser {name}")
        # Plus aucune bouée côtier historique
        for legacy in ["A", "B", "C", "D", "E", "Z1", "Z2"]:
            self.assertNotIn(legacy, used)

    def test_course_2_geometry(self):
        """Parcours 2 = porte départ + 2 tours de 4 bouées + porte arrivée."""
        legs = config.course_legs_for(2)
        self.assertEqual(legs[0]["name"], "DEPART")
        self.assertEqual(legs[0]["buoy"], "12")
        self.assertEqual(legs[-1]["name"], "ARRIVEE")
        self.assertEqual(legs[-1]["buoy"], "12")
        # 1 départ + 4 bouées × 2 tours + 1 arrivée = 10 étapes
        self.assertEqual(len(legs), 10)
        # Séquence par tour : 5 → 4 → 3 → 1
        sequence_t1 = [legs[i]["buoy"] for i in range(1, 5)]
        sequence_t2 = [legs[i]["buoy"] for i in range(5, 9)]
        self.assertEqual(sequence_t1, ["5", "4", "3", "1"])
        self.assertEqual(sequence_t2, ["5", "4", "3", "1"])

    def test_penalty_legs_unified(self):
        """PENALTY_LEGS pointe sur P1/P2 quel que soit le parcours (briefing 8/5)."""
        buoys = {leg["buoy"] for leg in config.PENALTY_LEGS}
        self.assertEqual(buoys, {"P1", "P2"})
        self.assertEqual(len(config.PENALTY_LEGS), 3,
                         msg="P1 → P2 → P1 = 3 legs")


# ════════════════════════════════════════════════════════════════════════
# PARCOURS 1 — Spirale alternée (3→1→4 ou 4→2→3)
# ════════════════════════════════════════════════════════════════════════
class TestCourse1Spiral(unittest.TestCase):
    """Vérifie que le choix T/B est figé au départ et que la séquence
    suit bien le zigzag spiralé attendu par le règlement."""

    def setUp(self):
        # Forcer le contexte parcours 1 le temps des tests
        self._old_course = config.COURSE_NUMBER
        config.COURSE_NUMBER = 1
        config.COURSE_LEGS = config.COURSE_1_LEGS

    def tearDown(self):
        config.COURSE_NUMBER = self._old_course
        config.COURSE_LEGS = config._COURSES_LEGS[self._old_course]

    def _cross_start_gate(self, cm: CourseManager, wind_dir_deg: float):
        """Simule un franchissement net de la porte de départ 1-2."""
        b1 = config.BUOYS_GPS["1"]
        b2 = config.BUOYS_GPS["2"]
        gate_mid = ((b1[0] + b2[0]) / 2.0, (b1[1] + b2[1]) / 2.0)
        before = geo_utils.move_meters(gate_mid, 30.0, 0.0)
        after = geo_utils.move_meters(gate_mid, -30.0, 0.0)
        cm.update_and_validate(before, wind_dir_deg=wind_dir_deg)
        cm.update_and_validate(after, wind_dir_deg=wind_dir_deg)

    def test_choice_T_when_buoy_3_more_upwind(self):
        """Vent venant du NE (45°) → bouée 3 (à l'est) plus exposée → côté T."""
        cm = CourseManager()
        self._cross_start_gate(cm, wind_dir_deg=45.0)
        self.assertEqual(cm.course1_chosen_side, "T")
        # Séquence figée : 3 → 1 → 4
        self.assertEqual(cm.legs[1].buoy, "3")
        self.assertEqual(cm.legs[2].buoy, "1")
        self.assertEqual(cm.legs[3].buoy, "4")
        self.assertEqual(cm.legs[1].side, "starboard")
        self.assertEqual(cm.legs[2].side, "port")
        self.assertEqual(cm.legs[3].side, "port")
        # L'arrivée reste une porte
        self.assertEqual(cm.legs[4].side, "gate")

    def test_choice_B_when_buoy_4_more_upwind(self):
        """Vent venant du NW (315°) → bouée 4 (à l'ouest) plus exposée → côté B."""
        cm = CourseManager()
        self._cross_start_gate(cm, wind_dir_deg=315.0)
        self.assertEqual(cm.course1_chosen_side, "B")
        self.assertEqual(cm.legs[1].buoy, "4")
        self.assertEqual(cm.legs[2].buoy, "2")
        self.assertEqual(cm.legs[3].buoy, "3")
        self.assertEqual(cm.legs[1].side, "port")
        self.assertEqual(cm.legs[2].side, "starboard")
        self.assertEqual(cm.legs[3].side, "starboard")

    def test_choice_frozen_after_start(self):
        """Une fois choisi, le côté ne change PAS même si le vent évolue."""
        cm = CourseManager()
        self._cross_start_gate(cm, wind_dir_deg=45.0)   # → T
        # On simule une fluctuation du vent qui aurait basculé en B
        cm.update_and_validate(
            geo_utils.move_meters(config.BUOYS_GPS["3"], 50.0, 0.0),
            wind_dir_deg=315.0,    # vent qui pointe vers la 4
        )
        self.assertEqual(cm.course1_chosen_side, "T",
                         msg="Le choix initial doit rester figé")
        self.assertEqual(cm.legs[1].buoy, "3")

    def test_no_wind_falls_back_to_T(self):
        """Sans données vent au départ → fallback "T" (3→1→4)."""
        cm = CourseManager()
        self._cross_start_gate(cm, wind_dir_deg=None)
        self.assertEqual(cm.course1_chosen_side, "T")

    def test_full_sequence_T_3_1_4(self):
        """Séquence complète côté T : DEPART → 3 → 1 → 4 → ARRIVEE."""
        cm = CourseManager()
        self._cross_start_gate(cm, wind_dir_deg=45.0)
        # Au départ on doit être à l'étape 1 (BOUEE_HAUTE_T1 = 3)
        self.assertEqual(cm.current_idx, 1)
        self.assertEqual(cm.current_leg.buoy, "3")
        self.assertEqual(cm.current_leg.name, "BOUEE_HAUTE_T1")
        # Vérifier l'ordre complet
        self.assertEqual(
            [l.buoy for l in cm.legs],
            ["12", "3", "1", "4", "12"],
            msg="Séquence T doit être 12 → 3 → 1 → 4 → 12",
        )

    def test_full_sequence_B_4_2_3(self):
        """Séquence complète côté B : DEPART → 4 → 2 → 3 → ARRIVEE."""
        cm = CourseManager()
        self._cross_start_gate(cm, wind_dir_deg=315.0)
        self.assertEqual(
            [l.buoy for l in cm.legs],
            ["12", "4", "2", "3", "12"],
            msg="Séquence B doit être 12 → 4 → 2 → 3 → 12",
        )

    def test_reset_clears_choice(self):
        """reset() doit remettre le côté à None et restaurer les legs initiaux."""
        cm = CourseManager()
        self._cross_start_gate(cm, wind_dir_deg=45.0)
        self.assertIsNotNone(cm.course1_chosen_side)
        cm.reset()
        self.assertIsNone(cm.course1_chosen_side)
        self.assertEqual(cm.legs[1].buoy, "34", msg="reset doit restaurer la porte 34")
        self.assertEqual(cm.legs[1].side, "gate")


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

    # ── Conformité bit-à-bit avec les exemples du PDF v5 ──────────────
    def test_pdf_v5_position_examples(self):
        """Cf. BATTLEBOATS_LORA_PROTOCOL_v5.pdf §3.3 (référence rapide)."""
        cases = [
            (PositionMessage("U1B1", 43.48256, 6.49872, 185, 3.2, no_fix=False),
             "P|U1B1|4348256|649872|185|32"),
            (PositionMessage("D2B2", 43.47100, 6.50134, 92, 1.8, no_fix=False),
             "P|D2B2|4347100|650134|092|18"),
            (PositionMessage("I3B3", 43.48500, 6.49500, 270, 2.5, no_fix=False),
             "P|I3B3|4348500|649500|270|25"),
            (PositionMessage("E4B1", 43.47800, 6.50200, 45, 1.0, no_fix=False),
             "P|E4B1|4347800|650200|045|10"),
            (PositionMessage("U1B2", 0, 0, 0, 0, no_fix=True),
             "P|U1B2|0|0|0|0"),
        ]
        for msg, expected in cases:
            self.assertEqual(msg.encode(), expected,
                             msg=f"Encoding non conforme PDF v5 pour {expected}")

    def test_pdf_v5_wind_examples(self):
        """Cf. BATTLEBOATS_LORA_PROTOCOL_v5.pdf §3.2 (référence rapide)."""
        cases = [
            (WindMessage(245, 6.3, 1746787652, sensor_offline=False),
             "W|245|63|1746787652"),
            (WindMessage(90, 1.5, 1746787657, sensor_offline=False),
             "W|090|15|1746787657"),
            (WindMessage(0, 0.0, 1746787662, sensor_offline=True),
             "W|000|00|1746787662"),
        ]
        for msg, expected in cases:
            self.assertEqual(msg.encode(), expected,
                             msg=f"Encoding non conforme PDF v5 pour {expected}")

    def test_build_from_mavlink_speed_conversion(self):
        """Vérifie la conversion m/s → nœuds spécifiée dans le PDF v5 §3.3."""
        # PDF : 32 = 3.2 kn → on attend bateau à ~1.65 m/s (= 3.2/1.94384)
        msg = build_position_from_telemetry(
            "U1B1", 43.48256, 6.49872,
            heading_deg=185, ground_speed_ms=1.6464,
            has_fix=True,
        )
        self.assertEqual(msg.encode(), "P|U1B1|4348256|649872|185|32")

    # ── Convention de direction PDF v5 §3.2 ────────────────────────────
    # "0° = Nord · 90° = Est · 180° = Sud · 270° = Ouest"
    # Direction = "d'OÙ vient le vent" (convention météo)
    def test_wind_convention_north(self):
        """W|000|... = vent venant du Nord (souffle vers le Sud)."""
        msg = WindMessage.parse("W|000|50|1700000000")
        self.assertEqual(msg.direction_deg, 0)
        self.assertAlmostEqual(msg.speed_ms, 5.0, places=1)
        self.assertFalse(msg.sensor_offline)

    def test_wind_convention_east(self):
        """W|090|... = vent venant de l'Est (souffle vers l'Ouest)."""
        msg = WindMessage.parse("W|090|50|1700000000")
        self.assertEqual(msg.direction_deg, 90)
        self.assertFalse(msg.sensor_offline)

    def test_wind_convention_south(self):
        """W|180|... = vent venant du Sud."""
        msg = WindMessage.parse("W|180|50|1700000000")
        self.assertEqual(msg.direction_deg, 180)

    def test_wind_convention_west(self):
        """W|270|... = vent venant de l'Ouest."""
        msg = WindMessage.parse("W|270|50|1700000000")
        self.assertEqual(msg.direction_deg, 270)

    def test_wind_offline_marker(self):
        """W|000|00|... uniquement = capteur orga hors ligne."""
        msg_offline = WindMessage.parse("W|000|00|1700000000")
        self.assertTrue(msg_offline.sensor_offline)
        # Bord cas : vent du Nord à 0 m/s = aussi détecté comme offline (cf. PDF)
        # mais vent du Nord à 0.1 m/s n'est PAS offline
        msg_calm_north = WindMessage.parse("W|000|01|1700000000")
        self.assertFalse(msg_calm_north.sensor_offline)
        self.assertEqual(msg_calm_north.direction_deg, 0)
        self.assertAlmostEqual(msg_calm_north.speed_ms, 0.1, places=1)

    def test_wind_speed_decimal(self):
        """spd ×10 m/s : 63 → 6.3 m/s, 15 → 1.5 m/s, 100 → 10.0 m/s."""
        cases = [(63, 6.3), (15, 1.5), (100, 10.0), (1, 0.1)]
        for s_x10, expected in cases:
            msg = WindMessage.parse(f"W|180|{s_x10:02d}|1700000000")
            self.assertAlmostEqual(msg.speed_ms, expected, places=2)

    def test_position_heading_convention_cardinal(self):
        """hdg suit la même convention que dir_vent : 0=N, 90=E, 180=S, 270=O."""
        for hdg, label in [(0, "N"), (90, "E"), (180, "S"), (270, "O")]:
            msg = PositionMessage.parse(f"P|U1B1|4300000|600000|{hdg:03d}|10")
            self.assertEqual(msg.heading_deg, hdg,
                             msg=f"Cap {hdg}° = {label} doit être préservé")

    def test_position_speed_kn_decimal(self):
        """spd ×10 nœuds : 32 → 3.2 kn, 100 → 10.0 kn."""
        cases = [(32, 3.2), (100, 10.0), (5, 0.5)]
        for spd_x10, expected_kn in cases:
            msg = PositionMessage.parse(f"P|U1B1|4300000|600000|180|{spd_x10:02d}")
            self.assertAlmostEqual(msg.speed_knots, expected_kn, places=2)

    def test_position_lat_lon_resolution(self):
        """lat/lon ×1e5 = résolution ~1 m sur la latitude (cf. PDF §3.3)."""
        # Différence de 1 unité sur le champ lat = 0.00001° ≈ 1.11 m
        msg1 = PositionMessage.parse("P|U1B1|4348256|649872|180|10")
        msg2 = PositionMessage.parse("P|U1B1|4348257|649872|180|10")
        self.assertAlmostEqual(msg1.lat, 43.48256, places=5)
        self.assertAlmostEqual(msg2.lat, 43.48257, places=5)
        diff_m = (msg2.lat - msg1.lat) * 111_320.0   # 1° ≈ 111.32 km
        self.assertAlmostEqual(diff_m, 1.11, places=1)

    def test_position_negative_coords(self):
        """Hémisphère sud / longitude ouest : lat/lon int32 peuvent être négatifs."""
        msg = PositionMessage.parse("P|U1B1|-4348256|-649872|180|10")
        self.assertAlmostEqual(msg.lat, -43.48256, places=5)
        self.assertAlmostEqual(msg.lon, -6.49872, places=5)
        # Round-trip
        self.assertEqual(msg.encode(), "P|U1B1|-4348256|-649872|180|10")

    def test_full_chain_mavlink_to_lora(self):
        """Chaîne complète : valeurs MAVLink réelles → encodage LoRa conforme."""
        # Cas concret : drone à Toulon, cap 185° (sud-sud-ouest), 3.2 kn = 1.6464 m/s
        # MAVLink GLOBAL_POSITION_INT.lat = 4348256 (×1e7 / 100 dans notre code → 43.48256)
        # Ici on simule la sortie de MavlinkInterface.get_telemetry()
        lat_mavlink = 4348256_00 / 1e7    # 43.48256
        lon_mavlink = 6498720_0 / 1e7     # 6.49872
        msg = build_position_from_telemetry(
            boat_id="U1B1",
            lat=lat_mavlink,
            lon=lon_mavlink,
            heading_deg=185.0,        # GLOBAL_POSITION_INT.hdg / 100
            ground_speed_ms=1.6464,   # hypot(vx, vy) cm/s → m/s
            has_fix=True,
        )
        self.assertEqual(msg.encode(), "P|U1B1|4348256|649872|185|32",
                         msg="Chaîne MAVLink → LoRa doit produire le format PDF")


# ════════════════════════════════════════════════════════════════════════
# COURSE — rayon adaptatif RTK / GPS (testé sur le parcours 2 "5 bouées")
# ════════════════════════════════════════════════════════════════════════
class TestCourse(unittest.TestCase):
    """Ces tests s'appuient sur la séquence du parcours 2 du briefing 8/5 :
    porte 1-2 → 5 (port) → 4 (port) → 3 (port) → 1 (port) → … (2 tours).
    On force donc temporairement COURSE_NUMBER=2 quel que soit l'env."""

    def setUp(self):
        self._old_course = config.COURSE_NUMBER
        config.COURSE_NUMBER = 2
        config.COURSE_LEGS = config._COURSES_LEGS[2]

    def tearDown(self):
        config.COURSE_NUMBER = self._old_course
        config.COURSE_LEGS = config._COURSES_LEGS[self._old_course]

    def test_initial_state(self):
        cm = CourseManager()
        self.assertEqual(cm.current_idx, 0)
        self.assertFalse(cm.race_started)
        self.assertEqual(cm.current_leg.name, "DEPART")

    def test_progression(self):
        """Franchir la porte 1-2 → on enchaîne sur T1_5 (bouée 5)."""
        cm = CourseManager()
        b1 = config.BUOYS_GPS["1"]
        b2 = config.BUOYS_GPS["2"]
        gate_mid = ((b1[0] + b2[0]) / 2, (b1[1] + b2[1]) / 2)
        # Trajectoire qui traverse le segment 1-2 d'est en ouest.
        before = geo_utils.move_meters(gate_mid, 30.0, 0.0)
        after = geo_utils.move_meters(gate_mid, -30.0, 0.0)
        cm.update_and_validate(before)
        cm.update_and_validate(after)
        self.assertTrue(cm.race_started)
        self.assertEqual(cm.current_leg.name, "T1_5")

    def test_capture_radius_strict_in_rtk(self):
        """En RTK, un point à 5.5 m d'une bouée NE doit PAS la valider (rayon 4 m)."""
        cm = CourseManager()
        cm.current_idx = 1     # T1_5
        cm.race_started = True
        b5 = config.BUOYS_GPS["5"]
        before = geo_utils.move_meters(b5, 8.0, 0.0)
        after = geo_utils.move_meters(b5, 5.5, 0.0)
        cm.update_and_validate(before, rtk_fixed=True)
        cm.update_and_validate(after, rtk_fixed=True)
        self.assertEqual(cm.current_leg.name, "T1_5",
                         msg="Étape ne doit pas être validée à 5.5 m en RTK")

    def test_capture_radius_lax_in_gps(self):
        """En GPS standard (rayon 7 m), un point à 5.5 m du bon côté DOIT valider."""
        cm = CourseManager()
        cm.current_idx = 1     # T1_5 (bouée 5, side=port → bateau passe à droite)
        cm.race_started = True
        b5 = config.BUOYS_GPS["5"]
        # Côté "port" = bouée à BÂBORD du bateau → trajectoire passant à
        # DROITE de la bouée (offset east > 0, cross > 0). Trajet sud→nord.
        before = geo_utils.move_meters(b5, 5.5, -3.0)
        after = geo_utils.move_meters(b5, 5.5, 3.0)
        cm.update_and_validate(before, rtk_fixed=False)
        cm.update_and_validate(after, rtk_fixed=False)
        self.assertNotEqual(
            cm.current_leg.name, "T1_5",
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
        boat = config.BUOYS_GPS["P1"]
        st = pm.update(boat_pos=boat, control_mode_is_manual=True,
                       rtk_fixed=True)
        self.assertEqual(st.mode, PenaltyMode.MANUAL)

    def test_wait_to_auto_after_timeout(self):
        pm = PenaltyManager()
        pm.start()
        pm._t0 = time.monotonic() - (config.PENALTY_DECISION_TIMEOUT_S + 0.5)
        st = pm.update(boat_pos=config.BUOYS_GPS["1"],
                       control_mode_is_manual=False, rtk_fixed=True)
        self.assertEqual(st.mode, PenaltyMode.AUTO)

    def test_auto_advances_through_legs(self):
        pm = PenaltyManager()
        pm.start()
        pm._mode = PenaltyMode.AUTO
        pm._t0 = time.monotonic() - 10
        # 1ère étape de pénalité (P1 commun aux 2 parcours depuis le briefing 8/5)
        first_buoy = config.PENALTY_LEGS[0]["buoy"]
        st = pm.update(boat_pos=config.BUOYS_GPS[first_buoy],
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


# ════════════════════════════════════════════════════════════════════════
# WIND ESTIMATOR — modes ESSAI / RÉGATE
# ════════════════════════════════════════════════════════════════════════
class TestWindEstimator(unittest.TestCase):

    def setUp(self):
        # Sauvegarder le mode courant pour le restaurer après chaque test
        self._saved_mode = config.MODE

    def tearDown(self):
        config.MODE = self._saved_mode

    def test_essai_mode_returns_fixed_wind(self):
        """En ESSAI, l'estimator retourne immédiatement WIND_ESSAI_*."""
        from wind.wind_estimator import WindEstimator
        config.MODE = "ESSAI"
        we = WindEstimator()
        est = we.estimate()
        self.assertEqual(est.source, "ESSAI")
        self.assertTrue(est.confident)
        self.assertAlmostEqual(est.direction_deg, config.WIND_ESSAI_DIR_DEG)
        self.assertAlmostEqual(est.speed_ms, config.WIND_ESSAI_SPEED_MS)

    def test_essai_ignores_orga_messages(self):
        """En ESSAI, push_orga_wind ne change PAS l'estimation."""
        from wind.wind_estimator import WindEstimator
        config.MODE = "ESSAI"
        we = WindEstimator()
        # Push un vent orga "270° 8 m/s" — doit être ignoré
        we.push_orga_wind(270, 8.0, int(time.time()), False)
        est = we.estimate()
        self.assertEqual(est.source, "ESSAI")
        self.assertNotEqual(est.direction_deg, 270)

    def test_regate_uses_orga_when_received(self):
        """En RÉGATE, push_orga_wind alimente l'estimation."""
        from wind.wind_estimator import WindEstimator
        config.MODE = "REGATE"
        we = WindEstimator()
        we.push_orga_wind(245, 6.3, int(time.time()), False)
        est = we.estimate()
        self.assertEqual(est.source, "ORGA")
        self.assertTrue(est.confident)
        self.assertAlmostEqual(est.direction_deg, 245.0, delta=1.0)

    def test_regate_fallback_when_silent(self):
        """En RÉGATE sans message orga, on utilise le fallback statique."""
        from wind.wind_estimator import WindEstimator
        config.MODE = "REGATE"
        we = WindEstimator()
        # Pas de push → fallback
        est = we.estimate()
        self.assertEqual(est.source, "FALLBACK")
        self.assertFalse(est.confident)
        self.assertAlmostEqual(est.direction_deg, config.WIND_FALLBACK_DIR_DEG)


# ════════════════════════════════════════════════════════════════════════
# GEO_UTILS — portes génériques (AB / 12 / 34)
# ════════════════════════════════════════════════════════════════════════
class TestGenericGates(unittest.TestCase):

    def test_buoy_gps_handles_12_gate(self):
        """buoy_gps('12') = milieu géométrique de la porte 1-2 (départ commun)."""
        b1 = config.BUOYS_GPS["1"]
        b2 = config.BUOYS_GPS["2"]
        mid = geo_utils.buoy_gps("12")
        self.assertAlmostEqual(mid[0], (b1[0] + b2[0]) / 2.0, places=6)
        self.assertAlmostEqual(mid[1], (b1[1] + b2[1]) / 2.0, places=6)

    def test_buoy_gps_handles_34_gate(self):
        """buoy_gps('34') doit fonctionner pour la porte au vent du parcours 1."""
        b3 = config.BUOYS_GPS["3"]
        b4 = config.BUOYS_GPS["4"]
        mid = geo_utils.buoy_gps("34")
        self.assertAlmostEqual(mid[0], (b3[0] + b4[0]) / 2.0, places=6)

    def test_buoy_gps_handles_single_buoy_5(self):
        """buoy_gps('5') retourne la bouée 5 (parcours 2 du briefing 8/5)."""
        b5 = config.BUOYS_GPS["5"]
        pos = geo_utils.buoy_gps("5")
        self.assertEqual(pos, b5)

    def test_gate_endpoints_with_arg(self):
        """gate_endpoints_gps prend un argument et retourne les 2 bouées."""
        a, b = geo_utils.gate_endpoints_gps("12")
        self.assertEqual(a, config.BUOYS_GPS["1"])
        self.assertEqual(b, config.BUOYS_GPS["2"])


# ════════════════════════════════════════════════════════════════════════
# TACTIQUE — exploitation des positions LoRa reçues
# ════════════════════════════════════════════════════════════════════════
class TestTactical(unittest.TestCase):

    def _fake_neighbor(self, lat, lon, has_fix=True):
        """Crée un faux NeighborState minimal."""
        from comms.lora_iface import NeighborState
        return NeighborState(boat_id="X", lat=lat, lon=lon,
                             has_fix=has_fix, last_seen_t=time.monotonic())

    def test_tactical_no_enemies(self):
        from swarm.tactical import compute
        snap = compute(boat_pos=(43.0967, 5.9533), heading_deg=0.0,
                       target_pos=(43.0977, 5.9533),
                       team_neighbors={}, enemy_neighbors={})
        self.assertEqual(snap.enemies_total, 0)
        self.assertEqual(snap.enemies_ahead, 0)
        self.assertFalse(snap.blocked_target)

    def test_tactical_enemies_ahead(self):
        from swarm.tactical import compute
        # Cible 100 m au nord ; ennemi à 50 m au nord (entre nous et cible) → "devant"
        boat = (43.0967, 5.9533)
        target = geo_utils.move_meters(boat, 0.0, 100.0)
        enemy = geo_utils.move_meters(boat, 0.0, 50.0)
        enemies = {"D2B1": self._fake_neighbor(*enemy)}
        snap = compute(boat_pos=boat, heading_deg=0.0,
                       target_pos=target,
                       team_neighbors={}, enemy_neighbors=enemies)
        self.assertEqual(snap.enemies_total, 1)
        self.assertEqual(snap.enemies_ahead, 1)

    def test_tactical_blocked_target(self):
        """Si ≥2 ennemis dans le rayon de la cible, blocked_target=True."""
        from swarm.tactical import compute
        boat = (43.0967, 5.9533)
        target = geo_utils.move_meters(boat, 0.0, 100.0)
        # 2 ennemis dans le rayon de 30 m autour de target
        e1 = geo_utils.move_meters(target, 5.0, 0.0)
        e2 = geo_utils.move_meters(target, -5.0, 5.0)
        enemies = {
            "D2B1": self._fake_neighbor(*e1),
            "I3B2": self._fake_neighbor(*e2),
        }
        snap = compute(boat_pos=boat, heading_deg=0.0,
                       target_pos=target,
                       team_neighbors={}, enemy_neighbors=enemies)
        self.assertTrue(snap.blocked_target)

    def test_role_strategy_defensive_when_blocked(self):
        """role_modifies_strategy passe en mode défensif si cible encombrée."""
        from swarm.roles import RoleManager
        from swarm.tactical import TacticalSnapshot
        rm = RoleManager()
        rm.my_role = config.ROLE_OPTIMIZER
        normal = rm.role_modifies_strategy()
        self.assertFalse(normal["defensive"])
        blocked = rm.role_modifies_strategy(
            tactical=TacticalSnapshot(enemies_total=3, blocked_target=True),
        )
        self.assertTrue(blocked["defensive"])
        self.assertGreater(blocked["buoy_clearance_factor"],
                           normal["buoy_clearance_factor"])


# ════════════════════════════════════════════════════════════════════════
# RÉSEAU LORA BATTLEBOATS — config consolidée
# ════════════════════════════════════════════════════════════════════════
class TestLoraNetworkConfig(unittest.TestCase):

    def test_channel_constants_present(self):
        """Les constantes du canal BATTLEBOATS doivent exister."""
        self.assertEqual(config.LORA_CHANNEL_NAME, "BATTLEBOATS")
        self.assertEqual(config.LORA_REGION, "EU_868")
        self.assertEqual(config.LORA_NODE_WIND, "WIND")
        self.assertEqual(config.LORA_NODE_ORG, "ORG")

    def test_broadcast_period_60s(self):
        """Le règlement impose 60 s entre 2 P|... successifs."""
        self.assertEqual(config.LORA_BROADCAST_POSITION_PERIOD_S, 60.0)
        self.assertEqual(config.LORA_BROADCAST_PERIOD_S, 60.0)

    def test_my_tdma_offset_2_drones(self):
        """U1B1 émet à T+0, U1B2 à T+30 (étalement intra-équipe)."""
        from swarm.tdma import my_offset_s_within_minute
        # On ne peut tester que pour DRONE_NUM courant, mais on vérifie la
        # formule (DRONE_NUM-1)*30 reste cohérente.
        offset = my_offset_s_within_minute()
        self.assertIn(offset, (0.0, 30.0))

    def test_team_and_enemy_lists_disjoint(self):
        team = set(config.TEAM_BOATS)
        enemy = set(config.ENEMY_BOATS)
        self.assertEqual(team & enemy, set(), msg="TEAM ∩ ENEMY doit être vide")
        # Pas d'identifiant orga dans les listes des bateaux
        self.assertNotIn(config.LORA_NODE_WIND, team | enemy)
        self.assertNotIn(config.LORA_NODE_ORG, team | enemy)


if __name__ == "__main__":
    unittest.main(verbosity=2)
