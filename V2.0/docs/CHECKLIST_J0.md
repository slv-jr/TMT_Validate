# Checklist J0 — Course du 9 mai 2026

> **Format imprimable** — 1 page recto-verso si possible. À garder sur soi le jour J.
> Cocher au stylo. Si ❌ : voir `docs/DEPLOYMENT.md` § "Annexe Dépannage".
> 
> Régate à **2 drones** UTT : `U1B1` (Scout) + `U1B2` (Optimizer). U1B3 optionnel.

---

## ☀️ Veille — soirée 8 mai

```
[ ] Batteries LiPo chargées (×2 + spares) ─ tension > 12.5V au repos
[ ] microSD Cube en place (×2)
[ ] Antennes LoRa vissées (×2)            ─ NE PAS oublier (PA cramé sinon)
[ ] Polaire calibrée (×2)                 ─ /etc/stormwings/polar_table.json
[ ] Radio J4C05 chargée                   ─ piles AA neuves
[ ] PC portable chargé + Mission Planner installé
[ ] WiFi terrain testé (clé 4G en backup)
[ ] Capture du Readme2 + cap. de course imprimée
[ ] Multimètre + tournevis dans le sac
```

---

## 🌅 Briefing matin (9h00 → 10h00)

```
[ ] Identifier le PARCOURS du jour (1 banane / 2 côtier court)
        ─► sera utilisé pour COURSE_NUMBER=__ au lancement

[ ] Récupérer GPS officiels selon le parcours :
    • Parcours 1 : 1, 2, 3, 4, P1, P2 (6 bouées)
    • Parcours 2 : A, B, C, D, E, Z1, Z2 (7 bouées)

[ ] Confirmer sens du parcours (tribord ou bâbord amure au départ ?)
[ ] Récupérer prévision vent pour régler WIND_FALLBACK_DIR/SPD
[ ] Vérifier que le nœud WIND orga émet (W|... toutes les 60s)
[ ] Numéros d'urgence des juges
```

---

## 🛠️ Avant mise à l'eau (par drone, 15 min)

### Drone __________ (U1B__) ─ rôle initial __________

```
[ ] Batterie connectée                  ─ LED Cube allumée
[ ] Pi boot OK                          ─ ssh admin@stormwings-u1b_.local
[ ] DRONE_ID correct                    ─ cat /etc/stormwings/drone_id.env
[ ] Variables MODE et COURSE_NUMBER     ─ STORMWINGS_MODE=REGATE COURSE_NUMBER=__
[ ] Bouées chargées                     ─ /etc/stormwings/buoys_today.json
[ ] Test fix RTK                        ─ python3 -m tests.test_rtk
        ─► fix_type ≥ 5 stable 10s     ─►
[ ] Test connexion MAVLink+LoRa+RC      ─ python3 -m tests.test_connexion
[ ] Test servos (levier mode BAS)       ─ python3 -m tests.test_servos
        ─► safran + voile bougent OK   ─►
[ ] Test bascule manuel/auto (levier)   ─ python3 -m tests.test_bascule
[ ] Diag config attendu :               ─ python3 config.py
        ─► MODE=REGATE COURSE=__ Rôle=__ Départ T+__s
[ ] Service stormwings démarré          ─ sudo systemctl start stormwings
[ ] Logs OK (1 ligne / 100ms)           ─ tail -f logs/flight_*.csv
```

### Vérifications RC (canaux MAVLink avec setup NouvelEncodeur)

```
[ ] Levier mode HAUT (chan6 > 1700µs)   ─ pilote a la main
[ ] Stick safran (chan4) : centre franc
[ ] Stick voile (chan5) : position basse (voile fermée)
[ ] Levier mode (chan6) : 3 positions distinctes ~950 / ~1500 / ~2050 µs
```

---

## 🚤 Mise à l'eau

```
[ ] Drone à l'eau, flotte horizontalement
[ ] Quille pas envasée
[ ] Voile gonflée par le vent
[ ] Pilote sur la berge prêt avec radio
[ ] Pi reçoit les positions de l'autre drone (LoRa OK)
        ─► journalctl -f sur 1 Pi : "[LoRa-RX] POS U1B__"
[ ] Pi reçoit le vent orga (mode REGATE)
        ─► journalctl -f : "[LoRa-RX] WIND dir=__° spd=__m/s"
[ ] Confirmer le rôle attribué dans les logs
        ─► "[ROLES] Drone moi=SCOUT  voisin=OPTIMIZER" (ou inverse)
```

---

## 🏁 Procédure de départ (régate à 2 drones)

```
T-3 min  [ ] U1B1 + U1B2 positionnés zone départ
         [ ] Levier mode HAUT — pilotage manuel pour positionnement final
         [ ] U1B1 (Scout) calé à ~10 m de la porte (proche)
         [ ] U1B2 (Optimizer) calé à ~50 m derrière U1B1
         [ ] Demander confirmation visuelle des juges

T-1 min  [ ] Levier mode BAS sur U1B1 puis U1B2 (ordre, pas simultané)
         [ ] Vérifier dans logs : "[MODE] MANUAL → AUTO (chan6=...)"
         [ ] U1B1 (Scout) cap direct vers la porte → "[NAV] ATTENTE → cap vers porte"
         [ ] U1B2 (Optimizer) entre en LOITER → "[NAV] loiter pré-départ (T+0/30s)"

T-0      [ ] Top départ donné par les juges
         [ ] U1B1 part dès le top (RACE_START_OFFSET_S=0)
         [ ] U1B2 attend 30 s en cercle (RACE_START_OFFSET_S=30)
              ─► reçoit ≥ 1 broadcast P|U1B1 → calibre sa polaire
              ─► à T+30s : franchit la porte avec vent affiné

T+30s    [ ] U1B2 quitte le loiter et fonce vers la porte
         [ ] Logs : "[NAV] loiter → REMONTEE_VENT" sur U1B2
```

---

## 📡 Pendant la course — surveillance

> Sur PC portable, **1 SSH par drone** dans 2 fenêtres `tmux` ou Terminal :

```
[ ] sudo journalctl -u stormwings -f         (×2 fenêtres)
```

### Signaux d'alerte à surveiller

| Log à voir                              | Action                                    |
|-----------------------------------------|-------------------------------------------|
| `[DEGRADED] RTK_DEGRADED`               | OK, le drone passe en radius=7m, on continue |
| `[DEGRADED] LORA_LOST`                  | Communication coupée — surveiller       |
| `[DEGRADED] WIND_STALE`                 | Plus de vent orga reçu — fallback statique|
| `[WIND] orga dir=__ spd=__ ajouté`      | OK, vent orga reçu (toutes les 60s)      |
| `[STALL] détecté niveau LIGHT`          | Drone décroche, il va corriger seul     |
| `[STALL] détecté niveau HARD`           | Reprendre la main si dérive             |
| `[COURSE] Bouée X validée`              | Étape franchie ✓                         |
| `[NAV] FIN_COURSE`                      | Drone a fini son parcours !              |

---

## 🚨 Procédures d'urgence

### Le drone dévie de la trajectoire (PÉNALITÉ probable)

```
1. Pilote constate visuellement la dérive (n'enroule pas la bonne bouée,
   passe du mauvais côté, etc.)
2. Levier mode HAUT immédiatement → prise de contrôle manuel
3. Repositionner le drone sur la trajectoire
4. Faire le tour de pénalité À LA RADIO :
   • Parcours 1 : porte 2-P1 → enrouler P2 BÂBORD → enrouler P1 BÂBORD
   • Parcours 2 : porte A-Z1 → enrouler Z2 BÂBORD → enrouler Z1 BÂBORD
5. Limite : 30s max RC (au-delà l'auto reprend)
6. Levier mode BAS → drone reprend le parcours à l'étape COURANTE
```

### Le drone fait n'importe quoi

```
1. Levier mode HAUT immédiatement (reprise manuelle)
2. Stick safran centré + stick voile choquée → drone s'arrête
3. Récupérer à la rame ou attendre dérive vers la berge
```

### Perte LoRa (drone solo)

```
─► [DEGRADED] LORA_LOST + ADVERSARY_SILENT
─► Le drone passe en mode prudent (radius=7m, vitesse réduite)
─► Vent orga muet > 30s → fallback statique WIND_FALLBACK_DIR/SPD
─► Continuer la course, ne pas reprendre la main sauf nécessité
```

### Perte MAVLink (Pi ne parle plus au Cube)

```
─► [DEGRADED] MAVLINK_LOST
─► Le Cube garde le dernier override 3s puis revient en RC pur
─► Levier mode HAUT immédiatement, repasser en pilotage radio
─► Si possible : SSH sur Pi et "sudo systemctl restart stormwings"
```

### Batterie faible

```
─► [DEGRADED] LOW_BATTERY (<11.0V)
─► Drone log warning + reprise RC suggérée à l'opérateur
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
[ ] Si 2ème manche : adapter COURSE_NUMBER au nouveau parcours
        ─► sudo systemctl restart stormwings sur les 2 Pi
```

---

## 📞 Contacts utiles

```
Juges course      ____________________
Station base RTK  ____________________
Station vent orga ____________________
Coéquipier 1      ____________________
Coéquipier 2      ____________________
```

---

## 🔢 Valeurs de référence — à connaître par cœur

| Paramètre                         | Valeur                |
|-----------------------------------|-----------------------|
| Capture WAYPOINT (RTK)            | **4 m**               |
| Capture WAYPOINT (GPS dégradé)    | **7 m**               |
| Pénalité — fenêtre pilote         | **5 s**               |
| Pénalité — limite manuel          | **30 s**              |
| Stall — durée détection           | **3 s**               |
| Vitesse stall                     | **< 0.15 m/s**        |
| Slot TDMA (2 drones)              | **0 / 750 ms**        |
| Décalage départ Optimizer         | **30 s**              |
| Loiter rayon (U1B2 pré-départ)    | **25 m**              |
| Loiter offset porte (U1B2)        | **50 m**              |
| Heartbeat LoRa course             | **60 s**              |
| Timeout LoRa neighbor             | **180 s** (3× période)|
| Timeout WIND orga (fallback)      | **30 s**              |

---

## 🔧 Commandes de lancement les plus courantes

```bash
# Course officielle parcours 2 (côtier court) — Scout
DRONE_ID=U1B1 STORMWINGS_MODE=REGATE COURSE_NUMBER=2 python3 main.py

# Course officielle parcours 2 — Optimizer
DRONE_ID=U1B2 STORMWINGS_MODE=REGATE COURSE_NUMBER=2 python3 main.py

# Course parcours 1 (banane, spirale alternée)
DRONE_ID=U1B1 STORMWINGS_MODE=REGATE COURSE_NUMBER=1 python3 main.py

# Test à sec parcours 2 avec vent simulé
DRONE_ID=U1B1 STORMWINGS_MODE=ESSAI COURSE_NUMBER=2 \
    WIND_DIR_DEG=270 WIND_SPEED_MS=4.5 python3 main.py

# Saisie bouées matin J0 (parcours actif)
COURSE_NUMBER=2 python3 -m tools.buoy_entry

# Saisie de TOUTES les bouées des 2 parcours (briefing matin)
python3 -m tools.buoy_entry --all
```

---

> 🍀 **BONNE COURSE — UTT — Challenge SWARMz BattleBoats 2026**

*StormWings v2.0 · Édition course du 9 mai 2026 · Régate à 2 drones · 2 parcours (1 banane, 2 côtier court)*
