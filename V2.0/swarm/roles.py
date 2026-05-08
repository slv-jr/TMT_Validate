"""
Attribution dynamique des rôles dans l'essaim UTT — régate à 2 drones.

Deux rôles (cf. config._DRONE_PROFILES) :
    - SCOUT     : éclaireur, départ T+0, mesure le vent au largue, agressif
    - OPTIMIZER : optimise le VMG, départ T+30s, profite des data du Scout

Réévalués toutes les ROLE_REEVAL_PERIOD_S secondes par une matrice de score.
Critères :
    - Position dans le parcours (le drone le plus avancé devient Scout)
    - Batterie restante (privilégie les drones bien chargés)
    - Vitesse instantanée (le plus rapide tend à devenir Scout)

NB : si seul 1 drone est connu, on conserve le rôle par défaut du profil.
"""

import logging
import time
from dataclasses import dataclass
from typing import Dict, Tuple

import config
from navigation import geo_utils

log = logging.getLogger(__name__)


@dataclass
class DroneState:
    boat_id: str
    leg_index: int = 0          # progression dans le parcours
    pos_gps: Tuple[float, float] = (0.0, 0.0)   # (lat_deg, lon_deg)
    speed_ms: float = 0.0
    battery_pct: float = 100.0
    has_fix: bool = False
    role: str = config.ROLE_OPTIMIZER


class RoleManager:
    """Gestionnaire de rôle pour le drone local."""

    def __init__(self):
        self.my_role: str = config.DEFAULT_ROLE
        self._last_eval_t: float = 0.0
        self._self_state: DroneState = DroneState(
            boat_id=config.DRONE_ID,
            role=config.DEFAULT_ROLE,
        )
        self._teammate_states: Dict[str, DroneState] = {}

    # ─────────────────────────────────────────────
    # Mise à jour des états
    # ─────────────────────────────────────────────
    def update_self(self, leg_index: int, pos_gps: Tuple[float, float],
                    speed_ms: float, battery_pct: float, has_fix: bool):
        self._self_state.leg_index = leg_index
        self._self_state.pos_gps = pos_gps
        self._self_state.speed_ms = speed_ms
        self._self_state.battery_pct = battery_pct
        self._self_state.has_fix = has_fix

    def update_teammate(self, boat_id: str, pos_gps: Tuple[float, float],
                        speed_ms: float, has_fix: bool,
                        leg_index_estimate: int = -1):
        """À appeler quand on reçoit une PositionMessage d'un coéquipier."""
        ts = self._teammate_states.get(boat_id)
        if ts is None:
            ts = DroneState(
                boat_id=boat_id,
                role=config.DEFAULT_ROLE,
            )
            self._teammate_states[boat_id] = ts
        ts.pos_gps = pos_gps
        ts.speed_ms = speed_ms
        ts.has_fix = has_fix
        if leg_index_estimate >= 0:
            ts.leg_index = leg_index_estimate
        else:
            ts.leg_index = self._estimate_progress_from_position(pos_gps)

    @staticmethod
    def _estimate_progress_from_position(pos_gps: Tuple[float, float]) -> int:
        """Heuristique : trouve l'étape la plus proche dans le parcours actif."""
        best_idx = 0
        best_d = 1e9
        for i, leg in enumerate(config.COURSE_LEGS):
            try:
                bp = geo_utils.buoy_gps(leg["buoy"])
            except KeyError:
                continue
            d = geo_utils.distance_m(pos_gps, bp)
            if d < best_d:
                best_d = d
                best_idx = i
        return best_idx

    # ─────────────────────────────────────────────
    # Calcul du score / attribution
    # ─────────────────────────────────────────────
    def _score_for_role(self, drone: DroneState, role: str) -> float:
        """Plus le score est élevé, plus le drone est adapté à ce rôle."""
        score = 0.0
        if role == config.ROLE_SCOUT:
            # Le Scout est en tête : récompense la position avancée et la vitesse
            score += drone.leg_index * 100.0
            score += drone.speed_ms * 5.0
            score += drone.battery_pct * 0.3
        elif role == config.ROLE_OPTIMIZER:
            # L'Optimizer maximise le VMG : récompense la batterie + une vitesse
            # stable (pas trop élevée = signe d'optimisation), pénalise être en tête
            score += drone.battery_pct * 1.0
            score += min(drone.speed_ms, 3.0) * 10.0
            # Préfère un drone "au milieu" : pas en tête ni dernier
            score += -abs(drone.leg_index - len(config.COURSE_LEGS) / 2.0) * 5.0
        return score

    def reevaluate(self) -> str:
        """Lance une réévaluation des rôles. Retourne le nouveau rôle local.

        Avec 2 drones : on attribue Scout au plus avancé/rapide, Optimizer à
        l'autre. Si on n'a pas encore reçu de message du coéquipier (départ
        non encore commencé), on garde le rôle par défaut du profil.
        """
        now = time.monotonic()
        if now - self._last_eval_t < config.ROLE_REEVAL_PERIOD_S:
            return self.my_role
        self._last_eval_t = now

        all_drones: Dict[str, DroneState] = dict(self._teammate_states)
        all_drones[config.DRONE_ID] = self._self_state

        if len(all_drones) < 2:
            return self.my_role

        # Attribution Hungarian-like : on choisit Scout (le meilleur à ce poste),
        # le drone restant devient Optimizer.
        roles_order = [config.ROLE_SCOUT, config.ROLE_OPTIMIZER]
        assignments: Dict[str, str] = {}
        remaining_drones = list(all_drones.keys())
        for role in roles_order:
            if not remaining_drones:
                break
            best_id = max(
                remaining_drones,
                key=lambda did: self._score_for_role(all_drones[did], role),
            )
            assignments[best_id] = role
            remaining_drones.remove(best_id)

        new_role = assignments.get(config.DRONE_ID, config.ROLE_OPTIMIZER)
        if new_role != self.my_role:
            log.info("[ROLES] Bascule %s → %s", self.my_role, new_role)
            self.my_role = new_role
        return new_role

    # ─────────────────────────────────────────────
    # Stratégie modulée par rôle (+ tactique)
    # ─────────────────────────────────────────────
    def role_modifies_strategy(self,
                               tactical=None) -> dict:
        """Retourne des paramètres de modulation pour la stratégie.

        SCOUT (U1B1) :
            - Accepte plus de risque, favorise vitesse brute
            - Tack plus agressif (plus rapide à virer)
            - Marge bouée minimale (frôle pour gagner du temps)

        OPTIMIZER (U1B2) :
            - Maximise VMG strict
            - Tack standard
            - Marge bouée standard
            - Quand il a reçu ≥1 broadcast du Scout, ajuste la polaire
              (cette logique est dans wind_estimator.py)

        Si `tactical` (TacticalSnapshot) est fourni, on bascule en mode
        "défensif" quand la cible est encombrée par ≥2 ennemis : marge
        bouée augmentée, tack moins agressif. C'est appelé depuis main.py.
        """
        if self.my_role == config.ROLE_SCOUT:
            base = {
                "buoy_clearance_factor": 1.0,
                "tack_eagerness": 1.2,
                "use_scout_wind": False,
            }
        else:
            # OPTIMIZER (défaut)
            base = {
                "buoy_clearance_factor": 1.0,
                "tack_eagerness": 1.0,
                "use_scout_wind": True,
            }

        if tactical is not None and getattr(tactical, "blocked_target", False):
            base["buoy_clearance_factor"] *= 1.5      # +50 % de marge
            base["tack_eagerness"] *= 0.8             # tack moins fréquent
            base["defensive"] = True
        else:
            base["defensive"] = False
        return base
