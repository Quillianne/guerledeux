import os, sys
import numpy as np 

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from utils import geo_conversion

def circle_trajectory(t, R = 10, M = [48.1996457, -3.0152944]):
    """
    dessine un cercle de rayon R autour du point M
    """
    a0, a1 = geo_conversion.conversion_spherique_cartesien(M[0], M[1])

    x = R*np.sin(t) + a0
    y = R*np.cos(t) + a1

    return x,y

def circle_trajectory_dot(t, R = 10):  #fonction qui retourne la dérivé du point a rejoindre à l'instant t (cartesien)    
    """
    fonction qui retourne la dérivée du cercle
    """
    x_dot = R*np.cos(t)
    y_dot = -R*np.sin(t)

    return x_dot, y_dot