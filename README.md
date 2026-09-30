# 🤖 Robot d'Assistance pour Personnes à Mobilité Réduite

Système robotique d'assistance monté sur fauteuil roulant, conçu pour aider les personnes à mobilité réduite dans leurs tâches quotidiennes : préhension et déplacement d'objets, interactions vocales, détection d'obstacles et navigation assistée.

> Projet réalisé chez **Flaster Group** (Agadir, Maroc) — Juin à Septembre 2023

![Status](https://img.shields.io/badge/Status-Complété-brightgreen)
![Arduino](https://img.shields.io/badge/Arduino-Mega%202560-blue)
![Python](https://img.shields.io/badge/Python-3.10-yellow)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

---

## 🎯 Objectif

Concevoir un robot d'assistance abordable et modulaire, fixé sur un fauteuil roulant, capable de :
- **Attraper et déplacer des objets** (bouteille, téléphone, télécommande, livre)
- **Détecter les obstacles** et alerter l'utilisateur
- **Interagir par la voix** (commandes vocales et retour audio)
- **Être piloté** via une télécommande sans fil ou une interface tactile

## 🏗 Architecture du système

```
┌─────────────────────────────────────────────────┐
│              INTERFACE UTILISATEUR               │
│  ┌──────────┐  ┌──────────┐  ┌───────────────┐  │
│  │Joystick  │  │  Écran   │  │  Commandes    │  │
│  │2 axes    │  │  OLED    │  │  vocales      │  │
│  └────┬─────┘  └────┬─────┘  └──────┬────────┘  │
│       │              │               │           │
│  ┌────▼──────────────▼───────────────▼────────┐  │
│  │         ARDUINO MEGA 2560                   │  │
│  │  ┌─────────┐ ┌──────────┐ ┌─────────────┐  │  │
│  │  │Moteurs  │ │Capteurs  │ │Communication│  │  │
│  │  │& Servos │ │Ultrason  │ │  HC-05 BT   │  │  │
│  │  └────┬────┘ └────┬─────┘ └──────┬──────┘  │  │
│  └───────┼───────────┼──────────────┼──────────┘  │
│          │           │              │              │
│  ┌───────▼───┐ ┌─────▼─────┐ ┌─────▼──────────┐  │
│  │Bras 3DDL  │ │4x HC-SR04 │ │ App Python     │  │
│  │+ Pince    │ │avant/arr  │ │ IHM Tkinter    │  │
│  │servos SG90│ │gauche/dr  │ │ + Speech Reco  │  │
│  └───────────┘ └───────────┘ └────────────────┘  │
└─────────────────────────────────────────────────┘
```

## 🔧 Matériel utilisé

| Composant | Modèle | Rôle |
|-----------|--------|------|
| Microcontrôleur | Arduino Mega 2560 | Contrôle central |
| Bras robotique | 3 DDL + pince (4x servos SG90) | Préhension d'objets |
| Capteurs distance | 4x HC-SR04 | Détection d'obstacles |
| Moteurs DC | 2x moteur 12V + L298N | Déplacement du fauteuil |
| Bluetooth | HC-05 | Communication avec l'IHM |
| Joystick | Analogique 2 axes | Pilotage manuel |
| Écran | OLED SSD1306 128x64 | Affichage état/alertes |
| Audio | Module DFPlayer Mini + haut-parleur | Retour vocal |
| Alimentation | Batterie LiPo 11.1V 5000mAh | Autonomie ~3h |

## 📁 Structure du projet

```
robot-assistance-pmr/
├── arduino/
│   ├── main.ino                  # Programme principal Arduino
│   ├── arm_controller.h          # Contrôle du bras 3DDL + pince
│   ├── obstacle_detection.h      # Détection d'obstacles (4 capteurs)
│   ├── motor_driver.h            # Pilotage moteurs DC (L298N)
│   ├── bluetooth_com.h           # Communication Bluetooth
│   ├── display_oled.h            # Affichage OLED
│   └── audio_feedback.h          # Retour sonore DFPlayer
├── ihm/
│   ├── app.py                    # Interface graphique principale
│   ├── bluetooth_serial.py       # Liaison série Bluetooth
│   ├── voice_control.py          # Reconnaissance vocale
│   └── config.py                 # Configuration
├── docs/
│   └── wiring_diagram.md         # Schéma de câblage
├── tests/
│   ├── test_arm.py               # Test unitaire bras
│   └── test_obstacles.py         # Test capteurs
├── requirements.txt
└── README.md
```

## 🚀 Installation

### Arduino
1. Ouvrir `arduino/main.ino` dans l'IDE Arduino
2. Installer les librairies : `Servo`, `Wire`, `Adafruit_SSD1306`, `DFRobotDFPlayerMini`, `SoftwareSerial`
3. Sélectionner **Arduino Mega 2560**, téléverser

### IHM Python
```bash
pip install -r requirements.txt
python ihm/app.py
```

## 🎮 Commandes

### Joystick
| Direction | Action |
|-----------|--------|
| Avant | Avancer le fauteuil |
| Arrière | Reculer |
| Gauche/Droite | Tourner |
| Bouton | Ouvrir/fermer la pince |

### Commandes vocales
| Commande | Action |
|----------|--------|
| "Attrape" | Bras descend et ferme la pince |
| "Lâche" | Ouvre la pince |
| "Monte" | Bras en position haute |
| "Descend" | Bras en position basse |
| "Stop" | Arrêt d'urgence |
| "Aide" | Annonce l'état du système |

## 📝 Licence
MIT — Libre d'utilisation et de modification.
