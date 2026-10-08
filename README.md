# Robot d'Assistance pour Personnes à Mobilité Réduite

Système robotique monté sur fauteuil roulant pour aider les personnes à mobilité réduite dans leurs tâches quotidiennes : préhension d'objets, détection d'obstacles, interactions vocales et pilotage par télécommande.

Projet réalisé chez Flaster Group, de Juin à Septembre 2023

## Fonctionnalités

- Bras robotique 3 DDL + pince pour attraper et déplacer des objets courants (bouteille, téléphone, livre)
- Détection d'obstacles sur 4 côtés par capteurs ultrason HC-SR04
- Pilotage par joystick analogique 2 axes
- Commandes vocales en français (reconnaissance via Google Speech API)
- Affichage d'état sur écran OLED
- Retour audio par haut-parleur (DFPlayer Mini)
- Communication Bluetooth (HC-05) avec interface PC
- Arrêt d'urgence

## Matériel

- Arduino Mega 2560
- 4 servomoteurs SG90 (bras + pince)
- 4 capteurs ultrason HC-SR04
- 2 moteurs DC 12V + pont en H L298N
- Module Bluetooth HC-05
- Écran OLED SSD1306 128x64
- Joystick analogique
- Batterie LiPo 11.1V 5000mAh

## Structure du projet

```
arduino/
  main.ino               Programme principal Arduino
  arm_controller.h       Contrôle du bras 3DDL + pince
  obstacle_detection.h   Détection d'obstacles (4 capteurs)
  motor_driver.h         Pilotage moteurs DC (L298N)
ihm/
  app.py                 Interface graphique (Tkinter)
  bluetooth_serial.py    Communication série Bluetooth
  voice_control.py       Reconnaissance vocale
```

## Commandes vocales

| Commande | Action |
|----------|--------|
| attrape | Bras descend et ferme la pince |
| lâche | Ouvre la pince |
| monte / descend | Position haute / basse du bras |
| stop | Arrêt d'urgence |
| home | Position de repos |

## Installation

Arduino : ouvrir `arduino/main.ino`, installer les librairies Servo, Wire, Adafruit_SSD1306, SoftwareSerial, téléverser sur Mega 2560.

IHM Python :
```
pip install -r requirements.txt
python ihm/app.py
```

## Technologies

Arduino, Python, Tkinter, Bluetooth (HC-05), servomoteurs, capteurs ultrason, OLED I2C, reconnaissance vocale
