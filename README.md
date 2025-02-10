# Projet Guerledan

Ce projet a pour but de contrôler un bateau équipé de deux moteurs, d’un IMU (accélération, gyroscope, magnétomètre) et d’un GPS, et récuperer les angles d'euler pour ensuite se régaler sur le lac

## Structure du projet

```bash
.
├── __init__.py               # Indique que ce dossier est un package Python
├── main.py                   # Script principal (point d'entrée)
├── calibration/
│   ├── __init__.py           # Marqueur de sous-package Python
│   └── magnetometer_calib.py # Code de calibration du magnétomètre
└── utils/
    ├── __init__.py           # Marqueur de sous-package Python
    ├── geo_conversion.py     # Fonctions utilitaires pour la conversion géographique
    └── ddlib.py              # Fonctions diverses et librairies générales
```