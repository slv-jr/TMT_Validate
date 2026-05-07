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

### `SERVO1_FUNCTION = 1` ⚠️ **PARAMÈTRE CRITIQUE**

Mode **RCPassThru** sur le servo de safran.

| Valeur | Comportement                                        | Conséquence StormWings |
|--------|-----------------------------------------------------|------------------------|
| 0      | Désactivé                                           | ❌ Servo ne bouge jamais |
| **1**  | **RCPassThru** : transmet RC ou override Pi         | ✅ Cible attendue       |
| 26     | GroundSteering : ArduPilot recalcule en permanence  | ❌ Override Pi écrasé   |

> **Si tes overrides Pi ne fonctionnent pas, vérifier ce paramètre EN PREMIER.**

### `SERVO1_MIN/MAX/TRIM = 1100 / 1900 / 1500`
Plage µs du gouvernail. Centre à **1500 µs**, butée gauche à 1100, butée droite à 1900. Soit ±400 µs.

> Si le safran ne va pas en butée mécanique, augmenter à 1000/2000. Si au contraire il "frappe" le bout, réduire à 1200/1800.

### `SERVO2_FUNCTION = 89`
**MainSail** — ArduRover gère la position du winch de voile. Cohérent avec `SAIL_ENABLE=1`.

> Pour pilotage **passthrough total** depuis le Pi (notre cas) : mettre `SERVO2_FUNCTION = 1`. Sinon ArduRover peut interférer avec nos commandes voile.

### `SERVO5_FUNCTION = 89`
Optionnel — pour une foc séparée. Si pas de foc, laisser à 0 sans conséquence.

### `SERVO4_FUNCTION = 1`
**Boost (CH4)** — RCPassThru pour permettre au Pi d'overrider le moteur de boost via `RC_CHANNELS_OVERRIDE`. **Uniquement pertinent sur U1B1**.

---

## 4. RC IN

### `RC_PROTOCOLS = 1`
**PPM uniquement**. L'encodeur BCUBE convertit les PWM individuels du récepteur en un flux PPM combiné.

> Si tu vois `No RC` dans Mission Planner alors que la radio est allumée, c'est probablement ici (ou un câble RC IN débranché).

### `MODE_CH = 3`
Le **canal 3** (levier de la radio) sélectionne le mode. Permet d'avoir 6 modes mappés sur les positions du levier.

### `MODE1..MODE6 = 0`

Tous les modes ArduRover = **MANUAL (0)**.

> 🎯 Pourquoi tout en MANUAL ? Parce que **la bascule auto/manuel se fait côté Pi**, pas côté ArduPilot. Le Pi lit `RC_CHANNELS.chan3_raw` :
> - `< 1300 µs` ⇒ Pi prend la main, envoie `RC_CHANNELS_OVERRIDE`
> - `> 1700 µs` ⇒ Pi cesse les overrides, le Cube redevient passthrough RC pur
>
> Du point de vue d'ArduPilot, on est **toujours** en MANUAL, ce qui simplifie tout.

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
[ ] Onglet Status → ch1in / ch2in / ch3in changent quand on bouge la radio
[ ] Onglet Status → ch1out / ch2out suivent ch1in / ch2in (RCPassThru OK)
[ ] Pas de "Bad Compass" ni "Bad GPS" en bas du HUD
[ ] CH3 affiche 3 plages distinctes (~950, ~1500, ~2050) selon la position du levier
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

- `SERVO1_FUNCTION` — toujours 1 (RCPassThru)
- `MODE1..MODE6` — toujours 0 (la bascule est côté Pi)
- `FS_GCS_ENABLE` — toujours 0 (sinon HOLD intempestif)
- `RC_PROTOCOLS` — toujours 1 (PPM)

Tout le reste est ajustable selon les conditions du jour.

---

*StormWings v2.0 — Paramètres ArduPilot · Mai 2026*
