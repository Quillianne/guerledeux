import numpy as np
import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from settings import POINT_BASE, RHO
def convert_to_decimal_degrees(ddmmss, direction):
    """
    Convertit une latitude/longitude au format DDMM.MMMM 
    (degrés + minutes décimales) en degrés décimaux.

    Paramètres
    ----------
    ddmmss : float ou str
        La valeur de latitude/longitude au format DDMM.MMMM.
        Par exemple 4807.038 indique 48 degrés et 07.038 minutes.
    direction : str
        La direction, 'N', 'S', 'E' ou 'W'. 
        Si direction vaut 'S' ou 'W', la valeur sera négative.

    Retour
    ------
    float
        La latitude/longitude en degrés décimaux.
    """
    # Convertir en float si la valeur est passée en chaîne
    ddmmss = float(ddmmss)

    # Séparer les degrés et les minutes
    degrees = int(ddmmss // 100)   # Partie entière correspondant aux degrés
    minutes = ddmmss % 100        # Reste correspondant aux minutes

    # Conversion des minutes en degrés décimaux
    decimal_degrees = degrees + (minutes / 60.0)

    # Gestion de la direction (Sud ou Ouest => valeur négative)
    if direction.upper() in ['S', 'W']:
        decimal_degrees = -decimal_degrees

    return decimal_degrees

def deg_to_rad(deg):
    """Convertit les degrés en radians."""
    return deg * np.pi / 180

def rad_to_deg(rad):
    return rad * 180.0 / np.pi

def conversion_cartesien_spherique(coord_xy, lat_m=POINT_BASE[0], long_m=POINT_BASE[1], rho=RHO):
    """
    Inverse de conversion_spherique_cartesien :
    À partir des coordonnées locales (X, Y),
    retourne (latitude, longitude) en degrés du point correspondant.
    
    :param coord_xy: tuple (X, Y) en coordonnées locales
    :param lat_m, long_m: coord. du point de référence (en degrés)
    :param rho: rayon (ou grand rayon) de la sphère utilisée
    :return: (latitude, longitude) en degrés du point P
    """

    X, Y = coord_xy

    # Convertir la référence (lat_m, long_m) en radians
    lat_m_rad = deg_to_rad(lat_m)
    long_m_rad = deg_to_rad(long_m)

    # Coordonnées cartésiennes (x_m, y_m) du point M
    x_m = rho * np.cos(lat_m_rad) * np.cos(long_m_rad)
    y_m = rho * np.cos(lat_m_rad) * np.sin(long_m_rad)

    # Retrouver (x_p, y_p) du point P
    # Rappel : X = x_m - x_p ; Y = y_p - y_m
    x_p = x_m - X
    y_p = y_m + Y

    # Calcul de la longitude (radians)
    long_p_rad = np.arctan2(y_p, x_p)

    # Calcul de la latitude (radians) via cos(lat_p)
    # cos(lat_p) = sqrt(x_p^2 + y_p^2) / rho
    r_xy = np.sqrt(x_p**2 + y_p**2)
    cos_lat_p = r_xy / rho

    # On borne cos_lat_p à 1 (dans le cas d'imprécisions numériques >1)
    if cos_lat_p > 1.0:
        cos_lat_p = 1.0
    elif cos_lat_p < -1.0:
        cos_lat_p = -1.0

    lat_p_rad = np.arccos(cos_lat_p)
    
    # Choix du signe pour lat_p :
    # hypothèse : on reste dans le même hémisphère que lat_m
    # Si lat_m est négatif, on rend lat_p négatif, etc.
    if lat_m_rad < 0:
        lat_p_rad = -lat_p_rad

    # Conversion en degrés
    lat_p_deg  = rad_to_deg(lat_p_rad)
    long_p_deg = rad_to_deg(long_p_rad)

    return (lat_p_deg, long_p_deg)

def conversion_spherique_cartesien(point, lat_m=POINT_BASE[0], long_m=POINT_BASE[1], rho=RHO):
    """
    Convertit les coordonnées GPS (latitude, longitude) en coordonnées cartésiennes locales
    par rapport à un point M défini par lat_m et long_m, en ne retournant que x et y.
    """
    # Convertir les latitudes et longitudes en radians
    lat_m_rad = deg_to_rad(lat_m)
    long_m_rad = deg_to_rad(long_m)
    lat_rad = deg_to_rad(point[0])
    long_rad = deg_to_rad(point[1])

    # Conversion des coordonnées du point M (centre) en cartésiennes 2D (x_m, y_m)
    x_m = rho * np.cos(lat_m_rad) * np.cos(long_m_rad)
    y_m = rho * np.cos(lat_m_rad) * np.sin(long_m_rad)

    # Conversion des coordonnées du point P en cartésiennes 2D (x_p, y_p)
    x_p = rho * np.cos(lat_rad) * np.cos(long_rad)
    y_p = rho * np.cos(lat_rad) * np.sin(long_rad)

    # Calcul des coordonnées relatives par rapport au point M
    x = x_p - x_m
    y = y_p - y_m

    return -x, y