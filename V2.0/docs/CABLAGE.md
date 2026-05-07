# Câblage StormWings — Schéma détaillé

> Ce document est la **référence visuelle** du câblage des 3 drones UTT pour le Challenge SWARMz BattleBoats 2026. Source de vérité : le PoC SwarmZ officiel + les ajouts spécifiques StormWings (CH4 boost sur U1B1, ESP32 LoRa V3 USB).

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
│     │ J5C01R   ├────────►│ BCUBE  ├────────►│ Cube RC  │            │
│     │ Récept.  │ ch1-4   │ Encod. │         │   IN     │            │
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
│     │      Raspberry Pi 4B         │                                 │
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
│     ┌──────────┐  PWM             ┌──────────┐                       │
│     │ Cube     ├─────────────────►│ Moteur   │ Boost (U1B1 only)     │
│     │ MAIN OUT4│                  │ ESC      │                       │
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

## B — Récepteur Joysway J5C01R → Encodeur BCUBE

Le récepteur sort des **PWM individuels** par canal. Le port RCIN du Cube attend du **PPM**. L'encodeur BCUBE fait la traduction.

### Branchements signal seul (les fils rouges/noirs côté récepteur ne sont **pas** tirés vers l'encodeur)

| J5C01R port | Couleur fil  | BCUBE entrée | Canal | Usage                  |
|-------------|--------------|--------------|-------|------------------------|
| **1**       | 3 fils       | IN1          | CH1   | Stick safran           |
| **2**       | 🟠 Orange    | IN2          | CH2   | Stick voile            |
| **3**       | 🟡 Jaune     | IN3          | CH3   | **Levier mode AUTO/MAN** |
| **4**       | 🟢 Vert      | IN4          | CH4   | Boost (U1B1 seulement) |
| **5**       | (batterie)   | (non utilisé)| —     | Alim récepteur         |

> ⚠️ Sur **U1B2** et **U1B3**, on **n'utilise pas CH4** (pas de boost). Câbler quand même pour pouvoir échanger les drones en cas de panne.

### Cavalier d'alimentation BCUBE

**LAISSER NON SOUDÉ** — c'est la configuration par défaut. L'encodeur sera alimenté par le Cube via le port RCIN.

---

## C — Encodeur BCUBE → Cube Orange+ RC IN

Câble 3 fils depuis la prise 4 pins de l'encodeur :

| BCUBE prise 4 pins | Cube RC IN | Couleur     |
|--------------------|------------|-------------|
| GND                | GND        | Noir        |
| PPM (signal)       | Signal     | Blanc/Orange|
| +5V                | +5V        | Rouge       |
| MUX                | **NON connecté** | — (pas utilisé) |

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

| MAIN OUT | Servo / Composant      | SERVO_FUNCTION | PWM neutre | Drones concernés |
|----------|------------------------|----------------|------------|------------------|
| OUT 1    | Safran (gouvernail)    | **1 (RCPassThru)** | 1500 µs | U1B1, U1B2, U1B3 |
| OUT 2    | Voile (winch)          | 89 (MainSail)  | 1500 µs    | U1B1, U1B2, U1B3 |
| OUT 3    | (libre)                | 0              | —          | —                |
| OUT 4    | Moteur boost (ESC)     | 1 (RCPassThru) | 1000 µs    | **U1B1 seul**    |
| OUT 5    | (BEC 5V — alim rail)   | —              | —          | tous (alim seule)|

> 🔴 **SERVO1_FUNCTION = 1 (RCPassThru) est OBLIGATOIRE.**
> Avec FUNCTION=0, le servo est désactivé.
> Avec FUNCTION=26 (GroundSteering), ArduPilot recalcule en permanence et écrase les overrides du Pi → le drone ne suit plus la consigne du Pi.

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

### 5 erreurs qui font perdre 2 heures

1. ⚡ **GND non commun** entre BEC, récepteur et Cube → comportement erratique. Dérouler du fil noir entre tous les blocs.
2. 🔌 **Rail MAIN OUT non alimenté** par le BEC → les servos ne bougent pas. Brancher OUT 5.
3. 🔄 **Connecteurs servo non retournés** sur Cube Orange Plus → +5V sur la broche signal → servo cramé. Toujours vérifier au multimètre.
4. 🔁 **TX-RX pas croisés** entre Cube et Pi → pas de heartbeat. Inverser.
5. ❌ **Cavalier d'alim soudé** sur l'encodeur BCUBE → l'encodeur attend une alim externe qui n'est pas branchée. Laisser non soudé.

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

Sur Pi 4B, les pins 6, 8 et 10 sont sur le coin haut-gauche du connecteur 40 broches (côté carte SD).

---

*StormWings v2.0 — Câblage · Mai 2026 · Source : PoC SwarmZ Full + ajouts UTT*
