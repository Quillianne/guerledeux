import os, sys
import numpy as np 

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from utils import geo_conversion
from settings import POINT_BOUEE, FREQUENCE_CIRCLE, RAYON_CIRCLE


def circle_trajectory(t, R = RAYON_CIRCLE, f = FREQUENCE_CIRCLE, M = POINT_BOUEE):  #fonction qui retourne le point a rejoindre à l'instant t (cartesien)
    """
    dessine un cercle de rayon R autour du point M
    """
    a0, a1 = geo_conversion.conversion_spherique_cartesien(M[0], M[1])

    x = R*np.sin(f*t) + a0
    y = R*np.cos(f*t) + a1

    return x,y

def circle_trajectory_dot(t, f = FREQUENCE_CIRCLE, R = RAYON_CIRCLE):  #fonction qui retourne la dérivé du point a rejoindre à l'instant t (cartesien)    
    """
    fonction qui retourne la dérivée du cercle
    """
    x_dot = R*f*np.cos(f*t)
    y_dot = -R*f*np.sin(f*t)

    return x_dot, y_dot