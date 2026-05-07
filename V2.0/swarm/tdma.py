"""
TDMA scheduling pour les transmissions LoRa internes UTT.

Note : Le protocole BattleBoats officiel impose 60s entre 2 P|... (cf.
BATTLEBOATS_LORA_PROTOCOL_v5.pdf §3.3). Ce TDMA fin (slots de 500ms) est
une couche INTERNE à l'équipe UTT pour éviter les collisions d'air entre
nos 3 drones quand on échange des informations supplémentaires (par
exemple à l'avenir des messages de coordination étendus).

Pour le protocole officiel, on respecte simplement la cadence de 60s en
décalant l'envoi de chaque drone selon son numéro :
    - U1B1 (D1) émet à T+0s, T+60s, T+120s, …
    - U1B2 (D2) émet à T+20s, T+80s, T+140s, …
    - U1B3 (D3) émet à T+40s, T+100s, T+160s, …
Cela évite que les 3 drones de l'équipe émettent en même temps et
saturent le canal LoRa avec des positions UTT.
"""

import time
import logging

import config

log = logging.getLogger(__name__)


def my_offset_s_within_minute() -> float:
    """Retourne le décalage (en secondes) DANS UNE MINUTE auquel notre
    drone émet son P|... officiel.

    On répartit les 3 drones sur 0/20/40s pour étaler les broadcasts.
    """
    return (config.DRONE_NUM - 1) * 20.0


class TDMAScheduler:
    """Décide à quel moment l'on doit émettre notre broadcast P|..."""

    def __init__(self):
        self._next_tx_t: float = 0.0
        self._period_s: float = config.LORA_BROADCAST_PERIOD_S
        self._offset_s: float = my_offset_s_within_minute()

    def init_schedule(self, reference_unix_time: int) -> None:
        """Initialise le planning à partir d'un temps de référence (typiquement
        Unix time GPS). On vise reference_t + offset, puis +period, +2·period, …
        """
        # Trouver le prochain top horloge cohérent
        now = time.time()
        # L'offset est appliqué dans la période de 60s
        # On cale notre slot sur reference_unix_time arrondi à la prochaine minute
        ref_modulo = reference_unix_time % int(self._period_s)
        # Heure unix de notre prochain slot
        sec_to_next = (self._offset_s - ref_modulo) % self._period_s
        self._next_tx_t = time.monotonic() + sec_to_next
        log.info(
            "[TDMA] Schedule initial : prochain TX dans %.1fs "
            "(offset=%.0fs, période=%.0fs)",
            sec_to_next, self._offset_s, self._period_s,
        )

    def should_transmit(self) -> bool:
        """Retourne True si c'est l'heure d'émettre.
        Avance automatiquement le prochain slot."""
        if self._next_tx_t == 0.0:
            # Pas encore initialisé — initialisation paresseuse
            self.init_schedule(int(time.time()))
        now = time.monotonic()
        if now >= self._next_tx_t:
            self._next_tx_t += self._period_s
            return True
        return False

    def time_to_next_tx_s(self) -> float:
        if self._next_tx_t == 0.0:
            return 0.0
        return max(0.0, self._next_tx_t - time.monotonic())
