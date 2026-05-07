# Checklist J0 — Course du 9 mai 2026

> **Format imprimable** — 1 page recto-verso si possible. À garder sur soi le jour J.
> Cocher au stylo. Si ❌ : voir `docs/DEPLOYMENT.md` § "Annexe Dépannage".

---

## ☀️ Veille — soirée 8 mai

```
[ ] Batteries LiPo chargées (×3)        ─ tension > 12.5V au repos
[ ] microSD Cube en place (×3)
[ ] Antennes LoRa vissées (×3)          ─ NE PAS oublier (PA cramé sinon)
[ ] Polaire calibrée (×3)               ─ /etc/stormwings/polar_table.json
[ ] Radio J4C05 chargée                 ─ piles AA neuves
[ ] PC portable chargé + Mission Planner installé
[ ] WiFi terrain testé (clé 4G en backup)
[ ] Capture du Readme2 + cap. de course imprimée
[ ] Multimètre + tournevis dans le sac
```

---

## 🌅 Briefing matin (9h00 → 10h00)

```
[ ] Récupérer GPS officiels des 9 bouées (A B C D E F G Z1 Z2)
[ ] Confirmer sens du parcours (tribord ou bâbord amure au départ ?)
[ ] Confirmer BOOST_MAX_S officiel (modifier config.py si ≠ 30s)
[ ] Récupérer fréquence vent (vérifier que la station Calypso émet)
[ ] Numéros d'urgence des juges
```

---

## 🛠️ Avant mise à l'eau (par drone, 15 min)

### Drone __________ (U1B__) ─ rôle initial __________

```
[ ] Batterie connectée                  ─ LED Cube allumée
[ ] Pi boot OK                          ─ ssh admin@stormwings-u1b_.local
[ ] DRONE_ID correct                    ─ cat /etc/stormwings/drone_id.env
[ ] Bouées chargées                     ─ /etc/stormwings/buoys_today.json
[ ] Test fix RTK                        ─ python3 -m tests.test_rtk
        ─► fix_type ≥ 5 stable 10s     ─►
[ ] Test connexion MAVLink+LoRa+RC      ─ python3 -m tests.test_connexion
[ ] Test servos (CH3 BAS)               ─ python3 -m tests.test_servos
        ─► safran + voile bougent OK   ─►
[ ] Test bascule manuel/auto (CH3)      ─ python3 -m tests.test_bascule
[ ] Service stormwings démarré          ─ sudo systemctl start stormwings
[ ] Logs OK (1 ligne / 100ms)           ─ tail -f logs/flight_*.csv
```

### Vérifications RC

```
[ ] CH3 HAUT (>1700µs) au démarrage     ─ pilote a la main
[ ] Stick safran : centre franc
[ ] Stick voile : position basse (voile fermée)
[ ] Levier mode : 3 positions distinctes
[ ] (U1B1) Levier boost : position basse
```

---

## 🚤 Mise à l'eau

```
[ ] Drone à l'eau, flotte horizontalement
[ ] Quille pas envasée
[ ] Voile gonflée par le vent
[ ] Pilote sur la berge prêt avec radio
[ ] Pi reçoit les positions des autres drones (LoRa OK)
        ─► journalctl -f sur 1 Pi : "[LoRa-RX] POS U1B__"
[ ] Confirmer le rôle attribué dans les logs
        ─► "[ROLES] Drone moi=SCOUT  voisin1=OPTIMIZER  voisin2=SAFETY"
```

---

## 🏁 Procédure de départ

```
T-3 min  [ ] Tous les drones positionnés zone départ
         [ ] CH3 HAUT — pilotage manuel pour positionnement final
         [ ] Demander confirmation visuelle des juges

T-1 min  [ ] CH3 BAS sur U1B1, puis U1B2, puis U1B3 (séquentiel)
         [ ] Vérifier dans logs : "[MODE] MANUAL → AUTO"
         [ ] Vérifier dans logs : "[NAV] ATTENTE → REMONTEE_VENT"

T-0      [ ] Top départ donné par les juges
         [ ] Drones démarrent automatiquement après leur offset TDMA
              ─► U1B1 : t=0s    (Scout)
              ─► U1B2 : t=20s   (Optimizer)
              ─► U1B3 : t=40s   (Safety)
```

---

## 📡 Pendant la course — surveillance

> Sur PC portable, **1 SSH par drone** dans 3 fenêtres `tmux` ou Terminal :

```
[ ] sudo journalctl -u stormwings -f         (×3 fenêtres)
```

### Signaux d'alerte à surveiller

| Log à voir                              | Action                                    |
|-----------------------------------------|-------------------------------------------|
| `[DEGRADED] RTK_DEGRADED`               | OK, le drone passe en radius=7m, on continue |
| `[DEGRADED] LORA_LOST`                  | Communication coupée — surveiller       |
| `[DEGRADED] WIND_STALE`                 | Plus de vent reçu — fallback boussole   |
| `[STALL] détecté niveau LIGHT`          | Drone décroche, il va corriger seul     |
| `[STALL] détecté niveau HARD`           | Reprendre la main si dérive             |
| `[PENALTY] received from juge`          | 5 sec pour décider AUTO/MANUEL          |
| `[BOOST] activé pour Xs (reste Ys)`     | Boost engagé, normal                     |
| `[NAV] ARRIVÉE`                         | Drone a fini son parcours !              |

---

## 🚨 Procédures d'urgence

### Le drone fait n'importe quoi

```
1. CH3 HAUT immédiatement (reprise manuelle)
2. Stick safran centré + stick voile choquée → drone s'arrête
3. Récupérer à la rame ou attendre dérive vers la berge
```

### Pénalité signalée par les juges

```
─► Pi détecte "[PENALTY] reçue"
─► 5 sec de fenêtre pour CH3 HAUT (manuel)
   • Si CH3 HAUT dans les 5s   ─► tu fais le tour à la radio (max 30s)
   • Si CH3 reste BAS          ─► drone fait Z1→Z2→Z1 tout seul
```

### Perte LoRa (drone solo)

```
─► [DEGRADED] LORA_LOST + ADVERSARY_SILENT
─► Le drone bascule en mode prudent (radius=7m, vitesse réduite)
─► Continuer la course, ne pas reprendre la main sauf nécessité
```

### Perte MAVLink (Pi ne parle plus au Cube)

```
─► [DEGRADED] MAVLINK_LOST
─► Le Cube garde le dernier override 3s puis revient en RC pur
─► CH3 HAUT immédiatement, repasser en pilotage radio
─► Si possible : SSH sur Pi et "sudo systemctl restart stormwings"
```

### Batterie faible

```
─► [DEGRADED] LOW_BATTERY (<11.0V)
─► Drone réduit la vitesse + désactive boost
─► Si tension critique (<10.5V) : reprendre la main et rallier la berge
```

---

## 🏆 Après la course — débrief

```
[ ] Récupérer la microSD du Cube (logs ArduPilot)
[ ] scp logs/ vers le PC depuis chaque Pi
[ ] Visualiser la trajectoire
        ─► python3 -m tools.replay_log logs_U1B_/flight_*.csv
[ ] Notes : ce qui a marché / pas marché
[ ] Si 2ème manche : sudo systemctl restart stormwings sur les 3 Pi
        (remet le compteur boost à zéro)
```

---

## 📞 Contacts utiles

```
Juges course      ____________________
Station base RTK  ____________________
Station vent      ____________________
Coéquipier 1      ____________________
Coéquipier 2      ____________________
Coéquipier 3      ____________________
```

---

## 🔢 Valeurs de référence — à connaître par cœur

| Paramètre                         | Valeur                |
|-----------------------------------|-----------------------|
| Capture WAYPOINT (RTK)            | **4 m**               |
| Capture WAYPOINT (GPS dégradé)    | **7 m**               |
| Boost total                       | **30 s** (par défaut) |
| Boost actions max                 | **3**                 |
| Pénalité — fenêtre pilote         | **5 s**               |
| Pénalité — limite manuel          | **30 s**              |
| Stall — durée détection           | **10 s**              |
| Vitesse stall                     | **< 0.3 m/s**         |
| Slot TDMA                         | **0/500/1000 ms**     |
| Heartbeat LoRa course             | **20 s**              |
| Timeout LoRa                      | **10 s**              |

---

> 🍀 **BONNE COURSE — UTT — Challenge SWARMz BattleBoats 2026**

*StormWings v2.0 · Édition course du 9 mai 2026*
