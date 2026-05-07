# Câblage StormWings — Schéma détaillé

> Ce document est la **référence visuelle** du câblage des 3 drones UTT pour le Challenge SWARMz BattleBoats 2026. Source de vérité : le setup officiel **NouvelEncodeur** des organisateurs (Joysway J5C01R → Arduino Nano ArduPPM v2.3.16 → Cube Orange+) + l'ajout StormWings d'un ESP32 LoRa V3 sur USB.
>
> Référence complète des organisateurs : `SwarmZ_fichier_Orga/NouvelEncodeur/cablage_ppm_battleboats.html`.

---

## Vue d'ensemble

```
┌─────────────────────────────────────────────────────────────────────┐
│                         À BORD DU DRONE                              │
│                                                                      │
│     ┌──────────┐                                                    │
│     │ Batterie │                                                    │
│     │   LiPo   │                                                    │
│     └────┬─────┘                                                    │
│          │                                                           │
│     ┌────▼─────┐    Y       ┌──────────┐                            │
│     │  BEC 5V  ├───────────►│ Cube PWR │                            │
│     │   3 A    ├──────┬────►│   IN     │                            │
│     └──────────┘      │      └──────────┘                            │
│                       │      ┌──────────┐                            │
│                       └─────►│ J5C01R   │                            │
│                       │      │ alim     │                            │
│                       │      └──────────┘                            │
│                       │      ┌──────────┐                            │
│                       └─────►│ MAIN OUT │ rail servos                │
│                              │   5      │                            │
│                              └──────────┘                            │
│                                                                      │
│     ┌──────────┐  PWM    ┌────────┐  PPM    ┌──────────┐            │
│     │ J5C01R   ├────────►│ Arduino├────────►│ Cube RC  │            │
│     │ Récept.  │ CH1/2/5 │ Nano   │  D10    │   IN     │            │
│     │ 5 canaux │         │ ArduPPM│         │          │            │
│     └──────────┘         └────────┘         └──────────┘            │
│                                                                      │
│     ┌──────────┐  CAN    ┌──────────┐                                │
│     │ HERE4    ├────────►│ Cube CAN1│                                │
│     │ RTK GPS  │         └──────────┘                                │
│     └──────────┘                                                     │
│                                                                      │
│     ┌──────────┐ UART    ┌──────────┐                                │
│     │ Cube     ├────────►│ Pi GPIO  │ (3 fils croisés)               │
│     │ TELEM2   │         │ 14/15    │                                │
│     └──────────┘         └─────┬────┘                                │
│                                │                                     │
│     ┌──────────┐         USB   │                                     │
│     │ ESP32    ├──────────────►│ Pi USB                              │
│     │ LoRa V3  │               │                                     │
│     └──────────┘               │                                     │
│                                ▼                                     │
│     ┌──────────────────────────────┐                                 │
│     │      Raspberry Pi 5          │                                 │
│     │      (cerveau StormWings)    │                                 │
│     └──────────────────────────────┘                                 │
│                                                                      │
│     ┌──────────┐  PWM             ┌──────────┐                       │
│     │ Cube     ├─────────────────►│ Servo    │ Safran                │
│     │ MAIN OUT1│                  │ gouv.    │                       │
│     └──────────┘                  └──────────┘                       │
│     ┌──────────┐  PWM             ┌──────────┐                       │
│     │ Cube     ├─────────────────►│ Servo    │ Voile                 │
│     │ MAIN OUT2│                  │ winch    │                       │
│     └──────────┘                  └──────────┘                       │
└─────────────────────────────────────────────────────────────────────┘
```

---

## A — Bloc alimentation (cœur du système)

### Câble Y depuis le BEC 5V

```
        BEC 5V 3A  (sortie)
              │
              ├── 1 ──► Cube PWR IN (rouge + noir, 3 fils JST)
              │
              ├── 2 ──► J5C01R alim dédiée (rouge + noir, 3 fils)
              │
              └── 3 ──► MAIN OUT 5 du Cube (rouge milieu + noir)
                         (alimente le rail servos 1-8)
```

> **GND COMMUN OBLIGATOIRE** — BEC, récepteur, encodeur, Cube et Pi doivent tous partager la masse. Sans ça, tu auras des comportements erratiques voire des destructions.

> **MAIN OUT 5 alimenté séparément** — Le Cube **ne fournit pas** de 5V sur le rail des servos par défaut. Il **faut** brancher le BEC sur OUT 5 (ou n'importe quel OUT libre) sinon les servos ne reçoivent rien.

---

## B — Récepteur Joysway J5C01R → Arduino Nano (ArduPPM)

Le récepteur sort 5 PWM individuels (CH1 à CH5). Seuls **CH1, CH2 et CH5** sont utilisés dans le setup officiel BattleBoats — CH3 et CH4 sont **volontairement laissés vides**. L'Arduino Nano flashé ArduPPM v2.3.16 convertit ces PWM en un flux PPM combiné pour le Cube.

### Branchements signal seul

| J5C01R port | Couleur fil | Nano pin | PPM canal côté Cube | Usage |
|-------------|-------------|----------|---------------------|-------|
| **CH1**     | 3 fils      | **D3**   | chan4 (RCMAP_ROLL=4)     | Stick safran (gouvernail) |
| **CH2**     | 🟠 Orange   | **D4**   | chan5 (RCMAP_THROTTLE=5) | Stick voile (winch)       |
| CH3         | —           | —        | —                        | **Non utilisé**           |
| CH4         | —           | —        | —                        | **Non utilisé**           |
| **CH5**     | 🟡 Jaune    | **D7**   | chan6 (MODE_CH=6)        | **Levier mode AUTO/MAN**  |
| GND         | Noir        | GND      | —                        | Masse commune             |
| (alim)      | (batterie)  | —        | —                        | Alim récepteur (par BEC)  |

> ⚠️ Le **levier de mode est sur CH5** du récepteur — pas CH3. Bien identifier physiquement le canal correspondant au levier 3 positions sur ta J4C05 avant de souder.

### Pull-ups obligatoires sur le Nano

Le firmware ArduPPM scrute les inputs avec interruptions. Les pins inutilisées **doivent** être tirées au +5V pour éviter les fausses interruptions :

| Pin Nano | À relier à |
|----------|------------|
| RX0      | +5V        |
| TX1      | +5V        |
| D2       | +5V        |

> Sans ces pull-ups, des fausses interruptions parasites font sauter les servos de manière erratique.

---

## C — Arduino Nano → Cube Orange+ RC IN

Câble 3 fils du Nano vers le port RCIN du Cube :

| Nano                | Cube RC IN (JST-GH 3 pins) | Couleur recommandée |
|---------------------|----------------------------|---------------------|
| **D10 (OC1B)**      | Pin 1 · **Signal**         | Blanc / vert        |
| GND                 | Pin 3 · **GND**            | Noir                |
| **+5V (entrée)**    | Pin 2 · **+5V**            | Rouge               |

> Le Nano est **alimenté par le Cube** (via la pin 2 du RCIN, et non par le BEC servo). Cela évite les brownouts du Nano lors des appels de courant des servos.

> ⚠️ **Polarité RCIN sur Cube Orange+** : la masse est sur la rangée du **haut** (inverse de la convention habituelle). Vérifier l'orientation du connecteur JST-GH 3 pins.

### Flash et configuration du Nano (à faire UNE fois avant câblage)

Voir `SwarmZ_fichier_Orga/NouvelEncodeur/cablage_ppm_battleboats.html` §04 :
- Firmware : `ArduPPM_v2.3.16_ATMega328p_for_ArduPlane.hex`
- Source : https://download.ardupilot.org/downloads/wiki/advanced_user_tools/
- Outil : `avrdude` (inclus dans Arduino IDE 2.x)
- Baud rate : **115200** (clones CH340)
- Commande type :

```bash
avrdude -p atmega328p -c arduino -P COM15 -b 115200 \
  -U flash:w:"ArduPPM_v2.3.16_ATMega328p_for_ArduPlane.hex":i
```

Une fois flashé, débrancher l'USB du Nano (il ne sert plus, le Nano est alimenté par le Cube).

---

## D — Cube TELEM2 → Raspberry Pi GPIO

Connecteur **JST-GH 6 pins** côté Cube. Niveaux 3.3 V des deux côtés — **pas besoin** de level shifter.

### Pinout TELEM2

```
Cube TELEM2 JST-GH (6 pins, vue de face)
┌───┬───┬───┬───┬───┬───┐
│ 6 │ 5 │ 4 │ 3 │ 2 │ 1 │
│GND│RTS│CTS│ RX│ TX│+5V│
└───┴───┴───┴───┴───┴───┘
```

### Connexions vers le Pi (3 fils seulement)

| Cube TELEM2  | Pin GPIO Pi | Pin physique Pi | Couleur recommandée |
|--------------|-------------|-----------------|---------------------|
| Pin 6 (GND)  | GND         | **Pin 6**       | Noir                |
| Pin 2 (TX)   | GPIO15 RXD  | **Pin 10**      | Vert                |
| Pin 3 (RX)   | GPIO14 TXD  | **Pin 8**       | Bleu                |

**NON connectés** (laisser les fils dans l'air ou couper) :
- Pin 1 (+5V) — le Pi a sa propre alim
- Pin 4 (CTS), Pin 5 (RTS) — pas de contrôle de flux

> 🔁 **TX↔RX croisés** — c'est NORMAL et OBLIGATOIRE. TX du Cube va sur RX du Pi (et vice-versa). Si tu mets TX↔TX, ils ne se parlent pas.

---

## E — Sorties servo MAIN OUT

### Cube Orange+ Plus — connecteurs INVERSÉS

| Position broche | Cube Orange standard | **Cube Orange Plus**       |
|-----------------|----------------------|----------------------------|
| Haut (intérieur PCB) | Signal (S)      | **GND (−)**                |
| Milieu          | +5V (+)              | +5V (+)                    |
| Bas (bord PCB)  | GND (−)              | **Signal (S)**             |

> 🔄 **Tous les connecteurs servo et BEC doivent être retournés à 180°** sur Cube Orange+ Plus. Vérifier au multimètre **+5V entre la broche du milieu et la masse boîtier**.

### Affectation des sorties

| MAIN OUT | Servo / Composant      | SERVO_FUNCTION       | Source PWM      | PWM neutre | Drones concernés |
|----------|------------------------|----------------------|-----------------|------------|------------------|
| OUT 1    | Safran (gouvernail)    | **26 (GroundSteering)** | RCMAP_ROLL=chan4 | 1500 µs   | U1B1, U1B2, U1B3 |
| OUT 2    | Voile (winch)          | 89 (MainSail)        | RCMAP_THROTTLE=chan5 | 1500 µs | U1B1, U1B2, U1B3 |
| OUT 3    | (libre)                | 0                    | —               | —          | —                |
| OUT 4    | (libre)                | 0                    | —               | —          | —                |
| OUT 5    | (BEC 5V — alim rail)   | —                    | —               | —          | tous (alim seule)|

> 🔴 **Pourquoi GroundSteering (26) et pas RCPassThru (1) ?**
> Avec le PPM Nano, le safran arrive sur **chan4**, pas chan1. RCPassThru (function 1) lit chan1 qui est saturé par les pull-ups du Nano. GroundSteering (function 26) lit `RCMAP_ROLL` (= chan4 dans notre setup) et passe en passthrough proportionnel en mode MANUAL — c'est exactement ce qu'on veut.

---

## F — HERE4 RTK → Cube CAN1

Câble livré avec le HERE4. Branchement simple :

```
HERE4 RTK ──[câble CAN propriétaire]──► Cube CAN 1
```

Côté ArduPilot, `GPS_TYPE=9` (DroneCAN) et `CAN_P1_DRIVER=1` configurés dans `ardupilot_params.txt`.

> 📡 **Corrections RTCM3** : le HERE4 reçoit ses corrections par sa propre antenne RTK depuis une station de base externe (typiquement la station-sol des organisateurs). Ce n'est PAS le rôle du Pi.

---

## G — ESP32 LoRa V3 → Pi USB

Pas de câblage particulier : l'ESP32 LoRa V3 est branché en **USB-C** sur n'importe quel port USB du Pi. Il sera énuméré comme :

```bash
ls /dev/ttyUSB* /dev/ttyACM*
# /dev/ttyUSB0  (typique)
# OU /dev/ttyACM0 (selon firmware)
```

Si le port n'est pas `/dev/ttyUSB0`, mettre à jour `LORA_PORT` dans `config.py` ou exporter `LORA_PORT=/dev/ttyACM0` avant lancement.

> 🔋 **Alimentation** : l'ESP32 est alimenté par le Pi via l'USB. Pas de BEC séparé.

> 📻 **Antenne** : visser l'antenne LoRa **AVANT** de mettre sous tension (sinon tu peux endommager le PA).

---

## H — Récapitulatif des points critiques

### 6 erreurs qui font perdre 2 heures

1. ⚡ **GND non commun** entre BEC, récepteur, Nano et Cube → comportement erratique. Dérouler du fil noir entre tous les blocs.
2. 🔌 **Rail MAIN OUT non alimenté** par le BEC → les servos ne bougent pas. Brancher le BEC sur OUT 5.
3. 🔄 **Connecteurs servo non retournés** sur Cube Orange Plus → +5V sur la broche signal → servo cramé. Toujours vérifier au multimètre.
4. 🔁 **TX-RX pas croisés** entre Cube TELEM2 et Pi GPIO → pas de heartbeat MAVLink. Inverser.
5. ⚠️ **Pull-ups manquants** sur RX0/TX1/D2 du Nano → fausses interruptions, servos qui sautent. Tirer ces 3 pins au +5V.
6. ❌ **Levier mode câblé sur D5 ou D6** au lieu de D7 → côté Cube, le canal mode arrivera en chan4 ou chan5 au lieu de chan6, et la lecture côté Pi (`config.CH_MODE = 6`) restera à 0. Bien câbler **CH5 récepteur → D7 Nano**.

### Vérifications au multimètre AVANT d'allumer

| Point de mesure                     | Valeur attendue            |
|-------------------------------------|----------------------------|
| GND commun (Cube ↔ Pi ↔ BEC)        | 0 Ω entre les masses       |
| +5V sur Cube PWR IN                 | 4.9 - 5.2 V                |
| +5V sur MAIN OUT 5 (broche milieu)  | 4.9 - 5.2 V                |
| Sens connecteur servo OUT 1         | +5V au MILIEU, GND en HAUT |
| TX Cube TELEM2 (pin 2) ↔ RX Pi (8)  | continuité                 |
| RX Cube TELEM2 (pin 3) ↔ TX Pi (10) | continuité                 |

---

## Annexe — Identifier les ports Pi GPIO

```
                3V3  (1) (2)  5V
              GPIO2  (3) (4)  5V
              GPIO3  (5) (6)  GND       ← TELEM2 Pin 6 (GND)
              GPIO4  (7) (8)  GPIO14    ← TELEM2 Pin 3 (RX du Cube)
                GND  (9) (10) GPIO15    ← TELEM2 Pin 2 (TX du Cube)
            GPIO17 (11) (12) GPIO18
                       ...
```

Sur Pi 5, les pins 6, 8 et 10 sont sur le coin haut-gauche du connecteur 40 broches (côté carte SD) — pinout 40 broches identique au Pi 4B.

---

*StormWings v2.0 — Câblage · Mai 2026 · Source : SwarmZ NouvelEncodeur (officiel) + ajouts UTT*
