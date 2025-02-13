from roblib import *
import numpy as np

def f(x, u):
    x = x.flatten()
    u = u.flatten()
    return np.array([[x[3] * np.cos(x[2])],
                     [x[3] * np.sin(x[2])], 
                     [u[0]], 
                     [u[1]]])

def control(x, m, X, seuil_proximite = 8, seuil_eloignement = 20):
    posx, posy, vitesse, cap = x.flatten()
    force_x, force_y = 0, 0
    
    for i in range(m):
        x_m = X[i].flatten()
        distance = np.linalg.norm([posx - x_m[0], posy - x_m[1]])
        if distance < 1e-6:
            continue  # éviter les erreurs de division
        direction = np.array([x_m[0] - posx, x_m[1] - posy]) / distance
        
        if distance < seuil_proximite:
            new_cap = x_m[3]
        elif distance > seuil_eloignement:
            # Se rapprocher en appliquant une force vers le bateau
            force_x += direction[0] * (distance - seuil_eloignement)
            force_y += direction[1] * (distance - seuil_eloignement)
        else:
            new_cap = cap
    # Calcul du nouveau cap basé sur la force résultante
    if np.linalg.norm([force_x, force_y]) > 1e-6:
        new_cap += np.arctan2(force_y, force_x)
    
    # Ajustement de la vitesse
    new_vitesse = min(max(vitesse + np.linalg.norm([force_x, force_y]), 0), 10)  # bornée entre 0 et 10
    
    return np.array([new_vitesse, new_cap])

# paramètres
a = 0.1
m = 6  # nombre de bateaux
X = 10 * np.random.rand(m, 4, 1)
dt = 0.01

ax = init_figure(-5, 15, -5, 15)

# simulation
for t in np.arange(0, 2, dt):
    clear(ax)
    for i in range(m):
        u = control(X[i], m, X)
        X[i] = X[i] + dt * f(X[i], u)
        draw_tank(X[i][:3], r=0.05)
    pause(0.01)
show()
