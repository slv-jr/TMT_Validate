"""
PID de cap : convertit l'erreur (cap_voulu - cap_réel) en angle de safran.

L'angle de safran est ensuite mappé vers une commande PWM par
mavlink_iface.set_rudder_angle_deg(...).

Anti-windup et limitation de la sortie en place pour éviter le cumul
d'erreur intégrale lorsque le bateau ne peut pas physiquement réagir
(zone morte, cap très différent, etc.).
"""

import logging
import time
from dataclasses import dataclass

import config
from navigation import geo_utils

log = logging.getLogger(__name__)


@dataclass
class PIDState:
    integral: float = 0.0
    last_error: float = 0.0
    last_t: float = 0.0


class HeadingPID:
    """PID de cap. Sortie : angle de safran en degrés (signé)."""

    def __init__(self,
                 kp: float = config.HEADING_PID_KP,
                 ki: float = config.HEADING_PID_KI,
                 kd: float = config.HEADING_PID_KD,
                 max_output_deg: float = config.RUDDER_ANGLE_MAX_DEG):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output_deg = max_output_deg
        self.state = PIDState()

    def reset(self):
        self.state = PIDState()

    def compute(self, target_heading_deg: float, current_heading_deg: float) -> float:
        """Retourne l'angle de safran à appliquer (degrés signés).

        Convention : positif = barre à droite (bateau tourne vers la droite).
        """
        now = time.monotonic()
        dt = (now - self.state.last_t) if self.state.last_t > 0.0 else 0.1
        dt = max(0.01, min(0.5, dt))

        # Erreur signée dans [-180, +180]
        error = geo_utils.angle_diff_deg(target_heading_deg, current_heading_deg)

        # Composante intégrale avec anti-windup
        self.state.integral += error * dt
        # Saturer l'intégrale pour éviter le windup
        max_int = self.max_output_deg / max(self.ki, 1e-3)
        self.state.integral = max(-max_int, min(max_int, self.state.integral))

        # Composante dérivée
        derivative = (error - self.state.last_error) / dt

        output = (self.kp * error
                  + self.ki * self.state.integral
                  + self.kd * derivative)

        # Saturation
        output = max(-self.max_output_deg, min(self.max_output_deg, output))

        # Mémorisation pour le prochain tick
        self.state.last_error = error
        self.state.last_t = now

        return output
