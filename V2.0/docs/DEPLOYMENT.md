# Guide de déploiement StormWings — du kit à la ligne de départ

> **Objectif** : t'amener du carton ouvert le J-7 à la course du 9 mai dans l'ordre **exact** des opérations, avec les commandes à copier-coller. Chaque section indique sur quelle machine tu travailles : `[PC]`, `[Pi]` ou `[Mission Planner]`.

À répéter pour chaque drone (U1B1, U1B2, U1B3) — la seule chose qui change est le `DRONE_ID`.

---

## Vue d'ensemble : 7 phases

```
PHASE 1 ─ Préparation des Raspberry Pi (PC, ~30 min × 3)
PHASE 2 ─ Câblage du drone (1h × 3 = 3h)
PHASE 3 ─ Configuration ArduPilot via Mission Planner (45 min × 3)
PHASE 4 ─ Déploiement du code sur les Pi (15 min × 3)
PHASE 5 ─ Tests au sol (bench, bateau hors de l'eau) (1h × 3)
PHASE 6 ─ J-1 : entraînement sur l'eau, calibrations (4h)
PHASE 7 ─ J0 : matin de course
```

Compte **2 jours pleins** pour les phases 1-5 si tu pars de zéro. **Ne saute pas** les tests au sol, c'est ce qui fait gagner la course.

---

## PHASE 1 — Préparation des Raspberry Pi `[PC]`

### 1.1. Flasher la microSD avec Raspberry Pi OS

Sur ton PC, télécharger **Raspberry Pi Imager** : https://www.raspberrypi.com/software/

| Réglage Imager                 | Valeur                                       |
|--------------------------------|----------------------------------------------|
| Device                         | **Raspberry Pi 4**                           |
| OS                             | **Raspberry Pi OS Lite (64-bit)** (Bookworm) |
| Storage                        | microSD ≥ 32 GB                              |

Cliquer sur la roue dentée **AVANT** d'écrire :

- **Set hostname** : `stormwings-u1b1` (puis `u1b2`, `u1b3`)
- **Enable SSH** : ✅ + créer utilisateur `admin` avec mot de passe robuste
- **Configure WiFi** : ton réseau local (pour la phase 1 uniquement)
- **Locale** : Europe/Paris, clavier fr

Écrire la SD et l'insérer dans le Pi.

### 1.2. Premier boot + SSH

```bash
# [PC] — attendre ~2 min après mise sous tension du Pi
ssh admin@stormwings-u1b1.local
# (premier boot ~5 min ; si .local ne résout pas, trouver l'IP via le routeur)
```

### 1.3. Mise à jour + outils de base

```bash
# [Pi]
sudo apt update && sudo apt upgrade -y
sudo apt install -y git python3-pip python3-venv
```

### 1.4. Cloner le dépôt

```bash
# [Pi]
mkdir -p ~/code && cd ~/code
git clone <ton_url_de_repo> stormwings
cd stormwings/V2.0
```

> Si le dépôt est local (pas encore push), le plus simple est de le copier en **scp** depuis ton PC :
> ```powershell
> # [PC PowerShell]
> scp -r D:\touon\V2.0 admin@stormwings-u1b1.local:~/code/stormwings/
> ```

### 1.5. Installation système (UART, deps, service)

```bash
# [Pi]
cd ~/code/stormwings/V2.0
sudo bash scripts/install.sh
# Répondre OUI à la question "service systemd pour démarrage auto"
sudo reboot
```

Le script `install.sh` :
- désactive le Bluetooth (libère `/dev/serial0` pour MAVLink)
- active l'UART hardware sur GPIO14/15
- supprime le shell série (sinon ça pollue le port)
- installe `pymavlink`, `meshtastic`, `pypubsub`
- installe le service systemd

### 1.6. Définir le DRONE_ID

```bash
# [Pi] — APRÈS le reboot
# Sur U1B1 :
sudo mkdir -p /etc/stormwings
echo 'DRONE_ID=U1B1' | sudo tee /etc/stormwings/drone_id.env
# Sur U1B2 → DRONE_ID=U1B2  ;  sur U1B3 → DRONE_ID=U1B3
```

### 1.7. Vérification rapide

```bash
# [Pi]
ls -l /dev/serial0       # doit exister
cd ~/code/stormwings/V2.0
DRONE_ID=U1B1 python3 config.py
# → doit afficher la config du drone
```

✅ **Phase 1 terminée** : SD flashée, SSH OK, dépôt cloné, code installé, DRONE_ID fixé. Répéter pour les 2 autres Pi.

---

## PHASE 2 — Câblage du drone

> Référence détaillée : `docs/CABLAGE.md` + le PoC SwarmZ officiel `SwarmZ_fichier_Orga/PoC Full/README.md`.

**⚠️ Avant de commencer** : tout débranché, batterie hors du drone. On ne câble jamais sous tension.

### 2.1. Bloc alimentation

```
Batterie LiPo
   ↓
BEC 5V 3A (ou Castle CC BEC) ─────┬──► PWR IN du Cube Orange+
                                   ├──► +5V du récepteur Joysway J5C01R
                                   └──► MAIN OUT 5 du Cube (alim rail servos)

(Câble en Y partout — GND COMMUN obligatoire)
```

> **Erreur n°1 à éviter** : oublier d'alimenter le **rail MAIN OUT 5** → les servos ne reçoivent rien et personne ne comprend pourquoi.

### 2.2. Récepteur RC → Encodeur BCUBE → Cube RCIN

| Source              | Destination          | Couleur     | Notes                  |
|---------------------|----------------------|-------------|------------------------|
| J5C01R port 1       | BCUBE IN1            | 3 fils      | Stick safran (CH1)     |
| J5C01R port 2       | BCUBE IN2 (signal)   | Orange      | Stick voile (CH2)      |
| J5C01R port 3       | BCUBE IN3 (signal)   | Jaune       | **Levier mode CH3**    |
| J5C01R port 4       | BCUBE IN4 (signal)   | Vert        | Boost CH4 (U1B1 only)  |
| BCUBE prise 4 pins  | Cube RC IN           | 3 fils      | GND-PPM-+5V            |

> **Cavalier non soudé** sur l'encodeur BCUBE — il sera alimenté par le Cube via RCIN.
> **MUX non connecté** côté Cube.

### 2.3. Cube TELEM2 → Raspberry Pi UART

Connecteur JST-GH 6 pins sur le Cube. Seulement 3 fils utilisés.

| Cube TELEM2     | → | Raspberry Pi GPIO    |
|-----------------|---|----------------------|
| Pin 6 (GND)     | → | Pin 6 (GND)          |
| Pin 2 (TX)      | → | Pin 10 (RXD/GPIO15)  |
| Pin 3 (RX)      | → | Pin 8 (TXD/GPIO14)   |

> **Pin 1 (5V) NON connecté** — le Pi a sa propre alimentation.
> **TX↔RX croisés** — sinon ça ne parle pas.

### 2.4. Servos sur MAIN OUT

| MAIN OUT | Servo                    | Notes                                      |
|----------|--------------------------|--------------------------------------------|
| OUT 1    | Safran (gouvernail)      | **Connecteur retourné 180°** sur Cube Orange+ Plus |
| OUT 2    | Voile (winch)            | Idem retourner                             |
| OUT 4    | Boost (U1B1 uniquement)  | Idem retourner                             |
| OUT 5    | (BEC 5V — alim rail)     | NON un servo, c'est l'alim                 |

> **Erreur n°2 à éviter** : ne pas retourner les connecteurs servo sur Cube Orange Plus (GND est en haut au lieu d'en bas). Vérifier au multimètre : +5V entre fil rouge (milieu) et GND.

### 2.5. ESP32 LoRa V3 (Meshtastic)

Brancher l'ESP32 LoRa V3 sur **un port USB du Pi** (USB-C). Il sera vu comme `/dev/ttyUSB0` (ou `ttyACM0`). Pas d'autre câblage.

### 2.6. HERE4 RTK GPS

Branché sur **CAN1** du Cube. Le HERE4 reçoit ses corrections RTCM3 d'une station de base externe (typiquement la station-sol des organisateurs ou une 2ème antenne). Aucune action côté Pi.

### 2.7. Schéma global de connexion

```
Batterie ──► BEC 5V ──┬──► Cube PWR IN
                       ├──► J5C01R alim
                       └──► MAIN OUT 5 (rail servos)

J5C01R ──PWM──► BCUBE ──PPM──► Cube RC IN

Cube TELEM2 ──UART──► Pi GPIO 14/15
Cube CAN 1  ──CAN──► HERE4 RTK
Cube MAIN OUT 1 ──PWM──► Servo safran
Cube MAIN OUT 2 ──PWM──► Servo voile
Cube MAIN OUT 4 ──PWM──► Boost (U1B1)

Pi USB ──► ESP32 LoRa V3
Pi GPIO ──UART──► Cube TELEM2
```

✅ **Phase 2 terminée** : tout câblé, GND commun vérifié. Tu peux maintenant mettre sous tension la batterie.

---

## PHASE 3 — Configuration ArduPilot `[Mission Planner]`

> À faire avec un PC connecté au Cube via le câble USB du Cube (la prise frontale, pas TELEM2).

### 3.1. Installer Mission Planner

https://firmware.ardupilot.org/Tools/MissionPlanner/ — version stable actuelle.

### 3.2. Vérifier le firmware

Connecter le Cube en USB → `Setup → Install Firmware` → choisir **ArduRover 4.6.3** (ou plus récent stable).

### 3.3. Charger le fichier de paramètres

```
Mission Planner → Config → Full Parameter Tree
    → Load From File
    → Sélectionner V2.0/docs/ardupilot_params.txt
    → Compare → Write Params
```

> Voir `docs/ARDUPILOT_PARAMS.md` pour l'explication ligne par ligne.

### 3.4. Calibrations OBLIGATOIRES (dans cet ordre)

1. **Accéléromètre** : `Setup → Mandatory Hardware → Accel Calibration → Calibrate Accel`
   (poser le drone dans 6 orientations comme indiqué)
2. **Boussole** : `Setup → Mandatory Hardware → Compass → Onboard Mag Calibration`
   (faire tourner le drone dans tous les sens 60-90 s)
3. **Radio** : `Setup → Mandatory Hardware → Radio Calibration → Calibrate Radio`
   - Bouger les sticks à fond dans tous les sens
   - **Vérifier impérativement** : CH1 (safran) varie ~1100→1900 µs, CH2 (voile) idem, CH3 doit montrer **3 positions distinctes** quand tu bouges le levier (~900 / ~1500 / ~2000 µs)

### 3.5. Vérification servos passthrough

Avec CH3 en position **HAUTE** (mode MANUAL), bouger le stick CH1 :
- Le safran doit suivre le stick **immédiatement**.
- Si non → revérifier `SERVO1_FUNCTION=1` (RCPassThru) et le sens du connecteur.

### 3.6. Vérification fix RTK (en extérieur)

Sortir le drone dehors, ciel ouvert. Dans `Flight Data → Status` :
- `gpsstatus` = **6** (RTK Fixed) idéalement
- `numsat` ≥ 12

> Si le RTK ne fixe pas, vérifier que la station de base RTCM3 émet bien (ou activer le mode SBAS au pire).

✅ **Phase 3 terminée** : Cube paramétré, calibré, fix RTK obtenu. Le levier CH3 fonctionne en bascule.

---

## PHASE 4 — Déploiement du code sur les Pi `[Pi]`

Si la phase 1.4 a déjà cloné le code, ici on synchronise les dernières modifs et on vérifie que tout démarre.

### 4.1. Mettre à jour le code (si besoin)

```bash
# [Pi]
cd ~/code/stormwings/V2.0
git pull       # OU re-scp depuis le PC si pas pushé
```

### 4.2. Vérification importation

```bash
# [Pi]
DRONE_ID=U1B1 python3 -c "
import config
from main import StormWingsApp
print('OK', config.DRONE_ID, config.DEFAULT_ROLE)
"
```

Aucune erreur attendue.

### 4.3. Lancer la suite de tests unitaires (logique pure)

```bash
# [Pi]
cd ~/code/stormwings/V2.0
DRONE_ID=U1B1 python3 -m tests.test_logic_unit
# → "Ran 46 tests in 0.0Xs / OK"
```

### 4.4. Activer le service systemd

```bash
# [Pi] — pour redémarrage auto au boot et logs centralisés
sudo systemctl enable stormwings
# (ne pas le démarrer tout de suite — on fera ça après les tests bench)
```

✅ **Phase 4 terminée** : code à jour, tests unitaires passés.

---

## PHASE 5 — Tests au sol (bench, bateau hors de l'eau) `[Pi]`

> **Bateau sur ses bers ou tenu manuellement**. La RC sous tension. La batterie du drone branchée.

### 5.1. Test connexion (MAVLink + LoRa)

```bash
# [Pi]
DRONE_ID=U1B1 python3 -m tests.test_connexion
```

Ce que tu dois voir :
- `[MAV] Connecté` puis lecture des coordonnées GPS, vitesse, cap
- `RC: ch1=1500 ch2=1500 ch3=1900` qui changent quand tu bouges la radio
- `Vent=...° ...m/s` quand la station Calypso émet (peut prendre 60 s)

### 5.2. Test fix RTK

```bash
# [Pi] — en extérieur, ciel ouvert
DRONE_ID=U1B1 python3 -m tests.test_rtk
# → ✅ FIX RTK STABLE pendant 10s — drone prêt !
```

Si ça timeout, sortir plus à découvert et attendre.

### 5.3. Test servos (CH3 BAS = mode AUTO)

⚠️ **Tenir le bateau ou le bloquer sur ses bers**.

Mettre le levier CH3 **EN BAS** sur la radio.

```bash
# [Pi]
DRONE_ID=U1B1 python3 -m tests.test_servos
```

Tu dois voir :
- Safran : centre → gauche → centre → droite → centre
- Voile : bordée → mi-ouverte → choquée → mi-ouverte
- Sweep progressif

Si rien ne bouge :
- Vérifier l'alimentation rail MAIN OUT 5
- Vérifier `SERVO1_FUNCTION=1`
- Vérifier le sens du connecteur servo

### 5.4. Test bascule manuel/auto

```bash
# [Pi]
DRONE_ID=U1B1 python3 -m tests.test_bascule
```

Bouger le levier CH3 :
- **CH3 BAS** → le safran fait des allers-retours auto. Logs : `[AUTO] CH3=...`
- **CH3 HAUT** → le safran cesse. Le stick CH1 reprend le contrôle. Logs : `[MANUAL] CH3=...`

Ctrl+C pour arrêter.

### 5.5. Lancement de la boucle complète (en mode bench)

```bash
# [Pi]
sudo systemctl start stormwings
sudo journalctl -u stormwings -f
```

Tu dois voir le log de la boucle principale tous les 100 ms. Pour arrêter :
```bash
sudo systemctl stop stormwings
```

### 5.6. Vérifier que les logs CSV s'écrivent

```bash
# [Pi]
ls -la ~/code/stormwings/V2.0/logs/
# → flight_U1B1_YYYYMMDD_HHMMSS.csv (≈ 1 ligne / 100 ms)
```

✅ **Phase 5 terminée** : tout fonctionne au sol. Le drone est PRÊT pour l'eau.

---

## PHASE 6 — J-1 : entraînement sur l'eau (8 mai)

L'ordre des tests sur l'eau est crucial — on commence simple et on monte en complexité.

### 6.1. Mise à l'eau + checklist sécurité (15 min × drone)

- [ ] Batterie chargée (mesurée > 12.0 V)
- [ ] CH3 **HAUT** au démarrage (RC reprend la main par défaut)
- [ ] Fix RTK confirmé sur le Pi (`tests/test_rtk.py` ou bandeau Mission Planner)
- [ ] LoRa : la station Calypso émet bien (vérifier dans le log Pi : `[LoRa-RX] WIND ...`)

### 6.2. Vérif manuelle (CH3 HAUT) — 5 min

Faire une boucle simple en pilotant à la radio. Vérifier :
- Le safran réagit bien et dans le bon sens
- La voile s'ouvre/ferme correctement
- Le bateau n'a pas de défaut hydrodynamique

### 6.3. Activation auto pour la première fois (CH3 BAS) — 10 min

Le drone va commencer à agir tout seul. Test à courte distance, **garder la main pour rebasculer CH3 HAUT à tout moment**.

Vérifier dans `journalctl -u stormwings -f` :
- `[NAV] ATTENTE → REMONTEE_VENT` (par exemple)
- Le cap cible se met à jour
- Pas de comportement erratique

### 6.4. Calibration polaire — 1h30

C'est la calibration **la plus importante** pour la performance. Vent stable nécessaire (3-6 m/s idéal).

```bash
# [Pi]
DRONE_ID=U1B1 python3 -m calibration.polar_calibration --wind-source lora
```

Le script va te demander 8 passes de 30 s à différents angles (40°, 50°, ..., 180°). Pour chaque :
1. Mettre le drone sur l'allure cible (radio en MANUEL pendant la stabilisation)
2. Quand il navigue stable au TWA voulu depuis ≥ 10 s, repasser CH3 BAS et taper ENTER
3. Le script échantillonne 30 s en mode auto
4. Recommencer pour l'angle suivant

Sortie : `/etc/stormwings/polar_table.json` qui sera chargé automatiquement au prochain démarrage de StormWings.

### 6.5. Réglage PID gouvernail — 30 min

Si tu observes des oscillations de cap (le drone "serpente" autour de sa cible) :
- Réduire `HEADING_PID_KP` de 2.0 → 1.5 dans `config.py`
- Augmenter `HEADING_PID_KD` de 0.5 → 0.8

Si le drone est trop "mou" pour atteindre son cap :
- Augmenter `HEADING_PID_KP` à 2.5

```bash
# [Pi]
sudo nano ~/code/stormwings/V2.0/config.py
sudo systemctl restart stormwings
```

### 6.6. Test essaim 3 drones simultanés — 1h

Lancer les 3 drones en eau libre. Vérifier dans les logs :
- Chaque drone voit les 2 autres en `[LoRa-RX] POS ...`
- Pas de collision en approche bouée (le `potential_field.py` doit les écarter)
- Les rôles se réattribuent dynamiquement (`[ROLES] ...`)

### 6.7. Test séquence pénalité — 30 min

```bash
# [Pi] — sur un drone
# Forcer une pénalité via Python (à intégrer ou mettre une touche RC)
DRONE_ID=U1B1 python3 -c "
import time
from main import StormWingsApp
app = StormWingsApp()
app.setup()
import threading
threading.Timer(20, lambda: app.request_penalty('test')).start()
app.run()
"
```

Le drone doit aller automatiquement faire le tour Z1 → Z2 → Z1 puis reprendre.

✅ **Phase 6 terminée** : les 3 drones naviguent en autonomie, polaire calibrée, pénalité validée. **Tu es prêt pour le 9 mai**.

---

## PHASE 7 — J0 : matin de course (9 mai)

### 7.1. Briefing organisateurs (~9h30)

Récupérer les coordonnées GPS officielles des 9 bouées. Format probable : décimal ou DMS.

### 7.2. Saisie des bouées sur les 3 Pi

Sur chaque Pi (un seul suffit si tu fais un scp ensuite) :

```bash
# [Pi]
cd ~/code/stormwings/V2.0
sudo BUOYS_OVERRIDE_PATH=/etc/stormwings/buoys_today.json python3 -m tools.buoy_entry
```

Le script t'affiche chaque bouée, tu colles ses coordonnées (le format décimal `43.0967 5.9533` et DMS `43°05.802'N 5°57.171'E` sont reconnus). À la fin : récap avec les distances entre bouées (sanity check).

Ensuite, **diffuser le fichier sur les 2 autres Pi** :

```bash
# [Pi1]
scp /etc/stormwings/buoys_today.json admin@stormwings-u1b2.local:/tmp/buoys_today.json
ssh admin@stormwings-u1b2.local 'sudo mv /tmp/buoys_today.json /etc/stormwings/'
# Idem pour U1B3
```

### 7.3. Confirmer le BOOST_MAX_S

Si les organisateurs annoncent une autre valeur que 30 s :

```bash
# [Pi sur U1B1 — seul à embarquer le boost]
sudo sed -i 's/^BOOST_MAX_S: float = .*/BOOST_MAX_S: float = 45.0/' \
    ~/code/stormwings/V2.0/config.py
```

### 7.4. Pré-vol final

Sur **chaque drone** :

```bash
# [Pi]
DRONE_ID=U1B1 python3 -m tests.test_rtk          # fix RTK confirmé
sudo systemctl restart stormwings
sudo journalctl -u stormwings -f | head -20
# Vérifier : "Capture radius=4.0 RTK / 7.0 GPS"
#            "Boost max=45.0s/3 actions" (ou la valeur du jour)
```

### 7.5. Mise à l'eau

CH3 **HAUT** = pilotage RC manuel pour la mise à l'eau. Quand le drone est positionné dans la zone de départ, basculer CH3 **BAS** : le Pi prend la main. Logs :

```
[MODE] Bascule MANUAL → AUTO (CH3=950µs)
[NAV] ATTENTE → REMONTEE_VENT (vmg)
```

### 7.6. Pendant la course

- Surveiller `journalctl -u stormwings -f` (sur 1 Pi via SSH WiFi terrain)
- Garder la radio prête pour reprendre la main si nécessaire
- Si pénalité notifiée par les juges : on a 5 s pour basculer CH3 HAUT — sinon le drone fait le tour Z1/Z2/Z1 tout seul

### 7.7. Après la course

Récupérer les CSV de log :

```bash
# [PC]
scp -r admin@stormwings-u1b1.local:~/code/stormwings/V2.0/logs/ ./logs_U1B1/
# Idem pour U1B2, U1B3
```

Visualiser :

```bash
# [PC]
cd D:/touon/V2.0
python3 -m tools.replay_log logs_U1B1/flight_U1B1_*.csv
```

✅ **Course terminée**. Si une 2ème course est prévue, **resetter le boost** :

```bash
# [Pi]
sudo systemctl restart stormwings   # remet le compteur boost à zéro
```

---

## Annexe — Dépannage rapide

| Symptôme                                      | Cause probable                              | Solution                                       |
|-----------------------------------------------|---------------------------------------------|------------------------------------------------|
| `[MAV] Pas de heartbeat`                      | UART non câblé / TX-RX pas croisés          | Vérifier câbles TELEM2 + `SERIAL2_PROTOCOL=2`  |
| Servos ne bougent jamais                      | Rail MAIN OUT non alimenté                  | Brancher BEC sur OUT 5                         |
| Servo bouge 1 fois puis bloque                | `SERVO1_FUNCTION ≠ 1`                       | Mettre `SERVO1_FUNCTION=1` (RCPassThru)        |
| CH3 jamais reçu                               | RC éteinte / encodeur PPM débranché         | Allumer J4C05 ; vérifier RCIN du Cube          |
| Bascule MANUAL/HOLD en boucle                 | `FS_GCS_ENABLE=1`                           | Mettre `FS_GCS_ENABLE=0`                       |
| `[LoRa] Échec ouverture /dev/ttyUSB0`         | ESP32 débranché / dans `/dev/ttyACM0`       | `ls /dev/tty*` puis adapter `LORA_PORT`        |
| Le drone "serpente" en cap                    | PID trop agressif                           | Réduire `HEADING_PID_KP`, augmenter `KD`       |
| RTK ne fixe pas                               | Pas de RTCM3 / antenne masquée              | Vérifier station de base ; ciel ouvert         |
| Service stormwings redémarre en boucle        | Erreur Python / dépendance manquante        | `sudo journalctl -u stormwings -n 100`         |

---

## Annexe — Commandes utiles

```bash
# Voir le log temps réel
sudo journalctl -u stormwings -f

# Voir les 100 dernières lignes
sudo journalctl -u stormwings -n 100

# Redémarrer après modif de config
sudo systemctl restart stormwings

# Désactiver le service (mode manuel)
sudo systemctl stop stormwings
DRONE_ID=U1B1 python3 ~/code/stormwings/V2.0/main.py

# Lister les ports série visibles (debug LoRa/MAVLink)
ls -l /dev/serial* /dev/ttyUSB* /dev/ttyACM*

# Vérifier la config active
DRONE_ID=U1B1 python3 ~/code/stormwings/V2.0/config.py
```

---

*StormWings v2.0 — Guide de déploiement · Mai 2026*
