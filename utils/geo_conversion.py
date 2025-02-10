import numpy as np

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

def conversion_spherique_cartesien(point, lat_m=48.1996872, long_m=-3.0153766, rho=6371000):
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