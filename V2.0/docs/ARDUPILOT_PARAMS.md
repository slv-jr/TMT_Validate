# Paramètres ArduPilot — Explications détaillées

> Référence de chaque paramètre du fichier [`ardupilot_params.txt`](./ardupilot_params.txt) à charger dans Mission Planner. À lire pour comprendre **pourquoi** chaque valeur est là (et donc savoir laquelle modifier en cas de souci).

Plate-forme cible : **Cube Orange+ Plus** sous **ArduRover 4.6.3+**.

---

## Comment charger ces paramètres

### Option 1 — Chargement en bloc (recommandé)

```
Mission Planner ──► onglet CONFIG
                    ──► Full Parameter Tree
                        ──► Load From File (en bas à droite)
                            ──► sélectionner V2.0/docs/ardupilot_params.txt
                                ──► Compare      (visualise le diff)
                                    ──► Write Params (applique)
                                        ──► reboot le Cube
```

### Option 2 — Modification ponctuelle

```
Mission Planner ──► CONFIG ──► Full Parameter List
                    ──► (recherche par nom)
                        ──► modifier la valeur
                            ──► Write Params
```

---

## 1. Plate-forme

### `FRAME_CLASS = 2`
Classe de châssis = **Boat**. Active la cinématique adaptée à un bateau (pas de roues motrices, pas de differential steering). Indispensable.

### `SAIL_ENABLE = 1`
Active le **module sailboat** d'ArduRover. Permet à l'autopilote de calculer l'angle optimal de voile en fonction du vent apparent.

> StormWings n'utilise PAS la navigation auto d'ArduPilot — on pilote tout depuis le Pi via `RC_CHANNELS_OVERRIDE`. Mais activer `SAIL_ENABLE` autorise le réglage de `SERVO2_FUNCTION=89` (MainSail) qui est utile.

---

## 2. Communication MAVLink Cube ↔ Pi

### `SERIAL2_PROTOCOL = 2`
Protocole **MAVLink2** sur le port TELEM2 (le port branché sur le Pi).
- 1 = MAVLink1 (legacy, ne pas utiliser)
- 2 = MAVLink2 (avec checksum + extensions)

### `SERIAL2_BAUD = 57`
Vitesse en kbaud **× 1000**. Donc `57` ⇒ **57600 bauds**. Valeur compatible avec un câble UART de 30+ cm sans souci. Plus rapide (115200) possible mais inutile pour notre débit (~10 Hz de télémétrie).

> Côté Pi, dans `config.py` : `MAVLINK_BAUD = 57600` doit matcher.

---

## 3. Servos

> Setup matériel : encodeur **Arduino Nano** ArduPPM v2.3.16. Le PPM produit donne :
> - `chan4_raw` ← CH1 récepteur Joysway → safran
> - `chan5_raw` ← CH2 récepteur          → voile
> - `chan6_raw` ← CH5 récepteur          → levier 3 positions (mode)
>
> Voir `SwarmZ_fichier_Orga/NouvelEncodeur/` pour la chaîne physique complète.

### `SERVO1_FUNCTION = 26` ⚠️ **PARAMÈTRE CRITIQUE**

**GroundSteering** sur MAIN OUT 1 (servo de safran).

| Valeur | Comportement                                        | Conséquence StormWings |
|--------|-----------------------------------------------------|------------------------|
| 0      | Désactivé                                           | ❌ Servo ne bouge jamais |
| 1      | RCPassThru → utilise RC channel **1** brut          | ❌ chan1 saturé (pull-up Nano) |
| **26** | **GroundSteering** : passthrough proportionnel de RCMAP_ROLL en MANUAL | ✅ Cible attendue (RCMAP_ROLL=4 → chan4) |
| 54     | k_rcin4 → utilise RC channel 4 brut                 | Alternative possible (équivalent) |

> **Pourquoi 26 et pas 1 ?** Avec le setup NouvelEncodeur, le safran arrive sur **chan4**, pas chan1. Et `SERVO1_FUNCTION=1` lit chan1 (pull-up Nano = saturé). On utilise donc `26` qui prend l'input via `RCMAP_ROLL=4`. En mode MANUAL, GroundSteering est un simple passthrough proportionnel — ArduPilot ne fait pas de PID en MANUAL pour les rovers.
>
> Les overrides MAVLink envoyés par le Pi sur **chan4** sont vus par GroundSteering comme un "stick déplacé" et passent au servo.

### `SERVO1_MIN/MAX/TRIM = 1100 / 1900 / 1500`
Plage µs du gouvernail. Centre à **1500 µs**, butée gauche à 1100, butée droite à 1900. Soit ±400 µs.

> Si le safran ne va pas en butée mécanique, augmenter à 1000/2000. Si au contraire il "frappe" le bout, réduire à 1200/1800.

### `SERVO2_FUNCTION = 89`
**MainSail** sur MAIN OUT 2. ArduRover gère la position du winch de voile en mode AUTO ; en mode MANUAL c'est un passthrough de `RCMAP_THROTTLE = 5` (= chan5). Cohérent avec `SAIL_ENABLE=1`.

> Les overrides MAVLink du Pi sur chan5 sont relayés par MainSail au servo.

### `SERVO3_FUNCTION = 0`, `SERVO4_FUNCTION = 0`, `SERVO5_FUNCTION = 0`, `SERVO6_FUNCTION = 0`
MAIN OUT 3/4/5/6 inutilisés. La sortie 5 du rail sert à l'alim BEC (5V vers les servos), pas à un signal PWM. Seules MAIN OUT 1 (safran) et MAIN OUT 2 (voile) sont câblées sur la carte UTT.

---

## 4. RC IN

### `RC_PROTOCOLS = 1`
**PPM uniquement**. L'encodeur Arduino Nano (ArduPPM v2.3.16) convertit les PWM individuels du récepteur Joysway J5C01R en un flux PPM combiné transmis au Cube via MAIN OUT 1 du Nano (D10).

> Si tu vois `No RC` dans Mission Planner alors que la radio est allumée :
> 1. récepteur J5C01R alimenté ? (LED allumée)
> 2. Nano alimenté ? (5V depuis le Cube via RCIN, cf. `docs/CABLAGE.md`)
> 3. câble PPM (D10 du Nano → RCIN signal du Cube) bien branché ?

### `RCMAP_ROLL = 4`, `RCMAP_THROTTLE = 5`, `RCMAP_PITCH = 6`
Mapping ArduPilot ↔ canaux PPM. Le firmware ArduPPM ne sort pas les signaux dans l'ordre 1→8 — il sort le CH1 récepteur sur PPM ch4, CH2 sur ch5, CH5 sur ch6 (par design du firmware). Ces `RCMAP_*` rétablissent le mapping logique côté ArduRover.

> Si tu pars sur le setup BCUBE (ancien, pas recommandé), il faut au contraire `RCMAP_ROLL=1, RCMAP_THROTTLE=3` car le BCUBE conserve l'ordre PPM ch1=CH1.

### `MODE_CH = 6`
Le **canal 6 PPM** (qui correspond au levier 3 positions de la J4C05 → CH5 récepteur → D7 Nano) sélectionne le mode ArduRover. Permet d'avoir 6 modes mappés sur les positions du levier.

### `MODE1..MODE6 = 0`

Tous les modes ArduRover = **MANUAL (0)**.

> 🎯 Pourquoi tout en MANUAL ? Parce que **la bascule auto/manuel se fait côté Pi**, pas côté ArduPilot. Le Pi lit `RC_CHANNELS.chan6_raw` (constante `config.CH_MODE`) :
> - `< 1300 µs` ⇒ Pi prend la main, envoie `RC_CHANNELS_OVERRIDE` sur chan4/chan5
> - `> 1500 µs` ⇒ Pi cesse les overrides, ArduRover redevient passthrough RC pur
>
> Du point de vue d'ArduPilot, on est **toujours** en MANUAL, ce qui simplifie tout et évite les comportements GUIDED/HOLD imprévus.

---

## 5. Failsafes

### `FS_THR_ENABLE = 0` ⚠️ **À considérer pour la course**
Désactive le failsafe radio. Avec `0`, perte RC = pas d'action. Avec `1` + `FS_THR_VALUE`, le Cube bascule en mode `FS_ACTION`.

> **Recommandation course** : laisser à 0. La logique de failsafe est gérée côté Pi (mode `LORA_LOST`, `MAVLINK_LOST`, etc.) avec une réaction adaptée au sailboat (loiter, return, etc.). Activer le failsafe ArduPilot peut faire entrer en HOLD au pire moment.

### `FS_GCS_ENABLE = 0` ⚠️ **NE PAS ACTIVER**
**Désactive obligatoirement le failsafe GCS**. Sinon le Cube exige des heartbeats du Pi (1 Hz minimum) et bascule en HOLD si le Pi a un microlag → drone bloqué en pleine course.

### `BRD_SAFETY_DEFLT = 0`
**Pas de safety switch** au boot. Pas de bouton physique à presser pour armer.

---

## 6. GPS HERE4 (CAN1)

### `GPS_TYPE = 9`
**DroneCAN (UAVCAN)**. Le HERE4 communique en CAN, pas en UART.

### `CAN_P1_DRIVER = 1`
Active le **driver CAN port 1** (le port physique CAN1 du Cube).

### `CAN_P1_BITRATE = 1000000`
**1 Mbps** — vitesse standard CAN.

### `CAN_D1_PROTOCOL = 1`
**DroneCAN** sur le driver CAN1.

### `CAN_D1_UC_NODE = 10`
ID du nœud DroneCAN du Cube. Le HERE4 prendra automatiquement un autre ID.

### `CAN_D1_UC_ESC_BM = 0`
Pas d'ESC sur CAN (on utilise le PWM standard pour les servos).

> 📡 Pour vérifier que le HERE4 est bien détecté : Mission Planner → `Initial Setup → Optional Hardware → DroneCAN/UAVCAN → SLCAN Mode CAN1`. Tu dois voir un nœud `Here4` apparaître.

---

## 7. Compass / IMU

### `COMPASS_ENABLE = 1` + `COMPASS_USE = 1`
Active la boussole et l'utilise pour l'estimation du cap (EKF3).

### `COMPASS_PRIO1_ID = 0`
À configurer **APRÈS** la calibration boussole. Pendant la calibration, Mission Planner va proposer la boussole interne ou celle du HERE4. Sélectionner la **boussole du HERE4** comme prio 1 (elle est loin des perturbations magnétiques du bateau).

> Sans calibration de boussole, le drone n'aura pas un cap fiable et la nav VMG va dériver.

---

## 8. Logging

### `LOG_BACKEND_TYPE = 1`
Logs ArduPilot écrits sur la **microSD du Cube** (à insérer dans le slot SD du Cube — séparée de celle du Pi).

### `LOG_FILE_DSRMROT = 1`
**Rotation des logs à chaque disarm**. Évite d'avoir un seul fichier énorme. Pratique pour analyser une course post-mortem.

> Pour récupérer les logs ArduPilot : retirer la microSD du Cube après la course → ouvrir avec **Mission Planner → Data Flash Logs → Review a Log**.

---

## 9. Paramètres optionnels — réglages fins

Ces paramètres ne sont **pas** dans le fichier de base mais peuvent être ajustés sur l'eau si besoin.

| Paramètre              | Valeur recommandée | Usage                                            |
|------------------------|--------------------|--------------------------------------------------|
| `RC1_DZ`               | 30                 | Dead zone du stick safran (µs)                   |
| `RC2_DZ`               | 30                 | Dead zone du stick voile                         |
| `EK3_GPS_TYPE`         | 0                  | EKF3 utilise GPS3D + RTK                         |
| `EK3_SRC1_POSXY`       | 3                  | Source position horizontale = GPS                |
| `EK3_SRC1_VELXY`       | 3                  | Source vitesse horizontale = GPS                 |
| `EK3_SRC1_YAW`         | 1                  | Source cap = compass                             |
| `BATT_MONITOR`         | 4                  | Monitoring batterie sur power module (si câblé)  |
| `BATT_LOW_VOLT`        | 11.0               | Seuil batterie faible (3S LiPo)                  |
| `INS_LOG_BAT_MASK`     | 1                  | Logger toutes les IMU                            |

---

## 10. Validation après chargement

Après `Write Params` + reboot, **vérifier dans Mission Planner** :

```
[ ] HUD affiche le numsat ≥ 8 (et idéalement gpsstatus = 6 = RTK Fixed)
[ ] HUD affiche le heading qui change quand on tourne le drone
[ ] Onglet Status → ch4in change quand on bouge le stick gauche/droite (safran)
[ ] Onglet Status → ch5in change quand on bouge le stick haut/bas    (voile)
[ ] Onglet Status → ch6in montre 3 plages distinctes (~950 / ~1500 / ~2050)
                    selon la position du levier 3 positions
[ ] Onglet Status → ch1out suit ch4in (GroundSteering passthrough en MANUAL)
[ ] Onglet Status → ch2out suit ch5in (MainSail passthrough en MANUAL)
[ ] Pas de "Bad Compass" ni "Bad GPS" en bas du HUD
```

Si tout est ✅, ArduPilot est prêt pour le Pi.

---

## 11. Procédure de calibration (à faire **APRÈS** le Write Params)

Dans **Mission Planner → Setup → Mandatory Hardware** :

1. **Accel Calibration** — `Calibrate Accel` puis suivre les 6 orientations (sur le côté droit, gauche, dos, ventre, nez bas, nez haut). Chaque pose ~3 s, presser une touche.
2. **Compass Calibration** — `Onboard Mag Calibration → Start`. Faire tourner le drone dans tous les sens en 3D pendant 60-90 s jusqu'à 100%. Sélectionner la **boussole du HERE4** en `prio 1`.
3. **Radio Calibration** — `Calibrate Radio`. Bouger les sticks à fond dans tous les sens, puis OK. Vérifier les valeurs de `min/max` pour chaque canal.

> Sans ces 3 calibrations, le drone démarrera mais le cap dérivera et la mise à plat sera fausse.

---

## 12. Aide-mémoire — paramètres à NE JAMAIS toucher

Une fois validé, **ne pas modifier** sans raison forte :

- `SERVO1_FUNCTION = 26` (GroundSteering, lit RCMAP_ROLL=4)
- `SERVO2_FUNCTION = 89` (MainSail, lit RCMAP_THROTTLE=5)
- `RCMAP_ROLL = 4`, `RCMAP_THROTTLE = 5` (matchent le PPM Nano)
- `MODE_CH = 6` (le levier 3 positions sort sur PPM ch6)
- `MODE1..MODE6 = 0` (la bascule est côté Pi, pas ArduPilot)
- `FS_GCS_ENABLE = 0` (sinon HOLD intempestif au moindre microlag du Pi)
- `SYSID_THISMAV = 1` (sinon le Pi ne peut pas demander les streams)
- `RC_PROTOCOLS = 1` (PPM)
- `SR2_RC_CHAN = 10` (le stream RC à 10 Hz sur TELEM2 est INDISPENSABLE)

Tout le reste est ajustable selon les conditions du jour.

---

*StormWings v2.0 — Paramètres ArduPilot · Mai 2026*
