import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

# === Paramètres globaux de la simulation ===

N = 5  # Nombre de bateaux simulés
np.random.seed(0)  # Pour avoir des résultats reproductibles (facultatif)

# Positions initiales aléatoires dans un carré [0,100]x[0,100]
positions = np.random.rand(N, 2) * 100

# Vitesse (échelle max en m/s ou en unité/s)
max_speed = 5.0

# Heading initial (en degrés, 0 = axe X)
headings = np.random.rand(N) * 360  # entre 0° et 360°

# Paramètres pour l'algo attraction/répulsion
safe_distance = 15.0         # Distance en-dessous de laquelle on repousse
repulsion_weight = 1.0
attraction_weight = 1.0

# Paramètres pour le suivi
# Ici, boat i suit boat (i-1) modulo N (vous pouvez inverser pour que i suive i+1).
follow_distance = 10.0
follow_weight   = 1.5

# Pour le pilotage "proportionnel" entre heading actuel et heading cible
Kp = 0.5  # Gain proportionnel pour la correction d'angle

# Intervalle de temps (s) entre chaque pas de simulation
dt = 0.1


def compute_forces(positions, headings):
    """
    Calcule la force totale sur chaque bateau.
    - Attraction/répulsion vis-à-vis des autres
    - Suivi du "bateau de devant" (index circulaire).
    
    Retourne un tableau Nx2 contenant la force (Fx, Fy) pour chaque bateau.
    """
    n = len(positions)
    forces = np.zeros((n, 2))
    
    for i in range(n):
        force_i = np.array([0.0, 0.0])
        pos_i = positions[i]

        # -- 1) Attraction/répulsion avec tous les autres bateaux --
        for j in range(n):
            if i == j:
                continue
            pos_j = positions[j]
            delta = pos_j - pos_i
            dist = np.linalg.norm(delta)

            if dist < 1e-6:
                # Évite division par zéro si deux bateaux sont au même endroit
                continue

            # Si trop proche => repousse
            if dist < safe_distance:
                direction = delta / dist
                intensity = repulsion_weight * (safe_distance - dist)
                force_i -= intensity * direction
            else:
                # Sinon, attire
                direction = delta / dist
                intensity = attraction_weight * (dist - safe_distance)
                force_i += intensity * direction

        # -- 2) Force de suivi --
        # On veut que le bateau i suive le bateau (i-1) mod n
        # (Si vous voulez l’inverse, vous pouvez faire (i+1)%n)
        leader_index = (i - 1) % n
        pos_leader = positions[leader_index]
        heading_leader_deg = headings[leader_index]
        heading_leader_rad = math.radians(heading_leader_deg)

        # Direction du leader en coordonnées cartésiennes
        # (cos, -sin) si on garde la même convention Y
        dir_leader = np.array([
            math.cos(heading_leader_rad),
            -math.sin(heading_leader_rad)
        ])
        
        # Position idéale pour se placer derrière le leader : 
        # un certain follow_distance derrière lui (dans la direction de son heading).
        desired_pos = pos_leader - follow_distance * dir_leader
        
        delta_follow = desired_pos - pos_i
        force_follow = follow_weight * delta_follow
        
        # On ajoute cette force de suivi à la force_i
        force_i += force_follow

        # Stocke la force totale pour le bateau i
        forces[i] = force_i
    
    return forces


def update(frame):
    """
    Fonction appelée à chaque frame pour :
    1. Calculer forces
    2. Mettre à jour headings et vitesses
    3. Mettre à jour positions
    4. Mettre à jour l'affichage (scatter + quiver)
    """
    global positions, headings

    # 1) Calcule de la force sur chaque bateau
    forces = compute_forces(positions, headings)

    # 2) Détermination du heading-cible = angle de la force (Fx, Fy)
    #    + correction proportionnelle
    for i in range(N):
        fx, fy = forces[i]
        # Angle cible (en degrés)
        target_heading = -np.degrees(math.atan2(fy, fx))
        
        current_heading = headings[i]
        error = current_heading - target_heading
        # Ramener l’erreur dans [-180, 180]
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Loi P
        correction = Kp * error
        new_heading = current_heading - correction
        headings[i] = new_heading % 360

    # 3) Mise à jour des positions
    #    Vitesse ~ norme de la force (limitée à max_speed)
    speeds = np.linalg.norm(forces, axis=1)
    speeds = np.clip(speeds, 0, max_speed)
    
    for i in range(N):
        rad = math.radians(headings[i])
        vx = speeds[i] * math.cos(rad)
        vy = -speeds[i] * math.sin(rad)
        positions[i, 0] += vx * dt
        positions[i, 1] += vy * dt

    # 4) Mise à jour de l’affichage
    scatter.set_offsets(positions)
    U = speeds * np.cos(np.radians(headings))
    V = -speeds * np.sin(np.radians(headings))
    quiver.set_offsets(positions)
    quiver.set_UVC(U, V)
    
    # Retour des éléments mis à jour
    return scatter, quiver


# === Mise en place de la figure Matplotlib ===
fig, ax = plt.subplots(figsize=(7, 7))
ax.set_xlim(0, 100)
ax.set_ylim(0, 100)
ax.set_aspect('equal')
ax.set_title("Attraction/Répulsion + Suivi (Simulation)")

# Trace initial (scatter)
scatter = ax.scatter(positions[:, 0], positions[:, 1], c='blue', s=40)

# Flèches (quiver) pour visualiser heading + intensité
U_init = np.zeros(N)
V_init = np.zeros(N)
quiver = ax.quiver(positions[:,0], positions[:,1], U_init, V_init,
                   angles='xy', scale_units='xy', scale=1, color='red', width=0.005)

# Animation
ani = animation.FuncAnimation(
    fig, update, 
    frames=300,     # 300 itérations
    interval=50,    # 50 ms = 20 FPS environ
    blit=False
)

plt.show()
