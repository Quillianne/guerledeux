# Projet de Navigation avec IMU et GPS

Ce projet vise à programmer un bateau équipé de deux moteurs, d'un capteur IMU et d'un GPS. Il permet de calibrer le magnétomètre et l'accéléromètre, d'estimer les angles d'Euler et de suivre un cap donné.

## Arborescence des fichiers

```
├── calibration
│   ├── gyro_calib.py              # Calibration du gyroscope
│   ├── magnetometer_calib.py      # Calibration du magnétomètre et de l'accéléromètre
│   ├── __init__.py
│
├── drivers-ddboat-v2
│   ├── arduino_driver_v2.py       # Interface de communication avec la carte Arduino pour les moteurs
│   ├── gps_driver_v2.py           # Interface de communication avec le GPS
│   ├── imu9_driver_v2.py          # Interface de communication avec l'IMU (accéléromètre, gyroscope, magnétomètre)
│   ├── radio_driver_v2.py         # Interface pour la communication radio
│   ├── __init__.py
│
├── test
│   ├── calibrate_and_read_euler.py # Script testant la calibration et affichant les angles d'Euler en continu
│   ├── follow_heading.py           # Script de navigation maintenant un cap donné
│   ├── __init__.py
│
├── utils
│   ├── ddlib.py                   # Bibliothèque pour la gestion de l'IMU et de la navigation
│   ├── geo_conversion.py          # Fonctions de conversion géographique (GPS -> coordonnées locales)
│   ├── roblib.py                   # Librairie mathématique pour la gestion des rotations et transformations
│   ├── __init__.py
│
├── settings.py                     # Fichier de configuration (chemins des fichiers de calibration, constantes)
├── main.py                         # Programme principal exécutant la calibration et affichant les angles d'Euler
├── README.md                       # Documentation du projet
```

## Explication des fichiers

### Calibration

- **`magnetometer_calib.py`** : Calibre le magnétomètre et l'accéléromètre en mesurant les valeurs brutes dans différentes positions et en calculant les matrices de transformation.
- **`gyro_calib.py`** : Calibre le gyroscope en enregistrant une série de mesures lorsque l'IMU est immobile pour estimer un biais.

### Drivers

- **`arduino_driver_v2.py`** : Envoie des commandes aux moteurs via une carte Arduino.
- **`gps_driver_v2.py`** : Récupère les données de positionnement GPS.
- **`imu9_driver_v2.py`** : Interface avec l'IMU, permettant d'accéder aux valeurs brutes des capteurs.
- **`radio_driver_v2.py`** : Gestion de la communication radio avec le bateau.

### Tests et scripts

- **`calibrate_and_read_euler.py`** : Vérifie si une calibration est disponible, sinon l’effectue, et affiche les angles d'Euler en temps réel.
- **`follow_heading.py`** : Programme de navigation permettant de maintenir un cap donné en ajustant la vitesse des moteurs en fonction de l'IMU.

### Utilitaires

- **`ddlib.py`** : Contient la classe `IMU` pour récupérer les angles d'Euler, la classe `Navigation` permettant de suivre un cap et `GPS` pour recupérer les coordonnées cartésiennes notamment
- **`geo_conversion.py`** : Convertit des coordonnées GPS (latitude, longitude) en coordonnées cartésiennes locales.
- **`roblib.py`** : Librairie pour les rotations et transformations mathématiques, utilisée dans le calcul des angles d'Euler.

### Fichiers de configuration

- **`settings.py`** : Définit les fichiers de calibration (`CALIBRATION_FILE`, `GYRO_CALIBRATION_FILE`) et la constante `DT` pour l'intervalle de mise à jour des capteurs.

### Programme principal

- **`main.py`** : Vérifie et charge les calibrations, initialise l'IMU, puis affiche les angles d'Euler en continu. (tout comme `test/calibrate_and_read_euler.py`)

## Utilisation

1. **Lancer la calibration** :
   ```bash
   python test/calibrate_and_read_euler.py
   ```
   Si une calibration est déjà enregistrée, elle sera chargée automatiquement et affichera le heading en continu

2. **Suivre un cap** :
   ```bash
   python test/follow_heading.py
   ```
   Cela permet au bateau de maintenir un cap donné (modifiable dans le script).

