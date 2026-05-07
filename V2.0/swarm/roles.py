"""
Attribution dynamique des rôles dans l'essaim UTT.

Trois rôles (cf. dossier technique §VII.2) :
    - SCOUT     : éclaireur, en tête, mesure le vent et confirme les bouées
    - OPTIMIZER : optimise le VMG, suit le scout
    - SAFETY    : arrière-garde, surveillance, redondance

Réévalués toutes les ROLE_REEVAL_PERIOD_S secondes par une matrice de score.
Critères :
    - Position dans le parcours (le drone le plus avancé devient Scout)
    - Batterie restante (Safety prend le drone à plus faible énergie)
    - Vitesse instantanée (le plus rapide tend à devenir Scout)
"""

import logging
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

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
        # Notre état interne (rempli par la boucle principale)
        self._self_state: DroneState = DroneState(
            boat_id=config.DRONE_ID,
            role=config.DEFAULT_ROLE,
        )
        # États perçus des coéquipiers (depuis LoRa)
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
        """À appeler quand on reçoit une PositionMessage d'un coéquipier.

        leg_index_estimate : si on connaît la progression du voisin par un
        message d'extension (non standard), sinon on l'estime ici.
        """
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
        """Heuristique : trouve la prochaine bouée non franchie en se
        basant sur la position GPS du voisin et la séquence du parcours.
        Sans info exacte, on retourne l'étape la plus proche.
        """
        best_idx = 0
        best_d = 1e9
        for i, leg in enumerate(config.COURSE_3_LEGS):
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
            score += drone.leg_index * 100.0          # plus avancé = mieux
            score += drone.speed_ms * 5.0             # plus rapide = mieux
            score += drone.battery_pct * 0.3
        elif role == config.ROLE_OPTIMIZER:
            score += drone.battery_pct * 1.0
            score += min(drone.speed_ms, 3.0) * 10.0  # vitesse stable, pas excessive
            # Préfère un drone "au milieu" : pas en tête ni dernier
            score += -abs(drone.leg_index - len(config.COURSE_3_LEGS) / 2.0) * 5.0
        elif role == config.ROLE_SAFETY:
            score += (100.0 - drone.battery_pct) * 0.5  # on lui donne le moins chargé
            score += -drone.leg_index * 20.0             # plutôt arrière du peloton
        return score

    def reevaluate(self) -> str:
        """Lance une réévaluation des rôles. Retourne le nouveau rôle local."""
        now = time.monotonic()
        if now - self._last_eval_t < config.ROLE_REEVAL_PERIOD_S:
            return self.my_role
        self._last_eval_t = now

        # Liste de tous les drones connus de l'équipe
        all_drones: Dict[str, DroneState] = dict(self._teammate_states)
        all_drones[config.DRONE_ID] = self._self_state

        if len(all_drones) < 2:
            # Pas assez d'info — on garde le rôle par défaut
            return self.my_role

        # Hungarian-like simple : on attribue les rôles dans l'ordre Scout,
        # Optimizer, Safety en choisissant à chaque fois le meilleur drone non encore assigné.
        roles_order = [config.ROLE_SCOUT, config.ROLE_OPTIMIZER, config.ROLE_SAFETY]
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
    # Stratégie modulée par rôle
    # ─────────────────────────────────────────────
    def role_modifies_strategy(self) -> dict:
        """Retourne des paramètres de modulation pour la stratégie.

        Exemple :
            SCOUT     : accepte plus de risque, favorise vitesse brute
            OPTIMIZER : maximise VMG strict
            SAFETY    : marge accrue sur les bouées, virages anticipés
        """
        if self.my_role == config.ROLE_SCOUT:
            return {
                "buoy_clearance_factor": 1.0,
                "tack_eagerness": 1.2,
            }
        elif self.my_role == config.ROLE_SAFETY:
            return {
                "buoy_clearance_factor": 1.5,    # passer plus large
                "tack_eagerness": 0.8,           # moins de tacks risqués
            }
        # OPTIMIZER (défaut)
        return {
            "buoy_clearance_factor": 1.0,
            "tack_eagerness": 1.0,
        }
