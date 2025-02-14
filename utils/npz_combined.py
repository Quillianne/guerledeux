import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import itertools

def load_trajectory(npz_file):
    """
    Charge et nettoie la trajectoire depuis un fichier NPZ.
    Retourne deux arrays (targets, boats) de forme (N,2) si possible.
    """
    data = np.load(npz_file, allow_pickle=True)
    history = data["history"]  # Chaque élément est un tuple: ( (x,y), (px,py) )
    
    cleaned_history = []
    for (target, boat) in history:
        if target is None or boat is None:
            continue
        if not isinstance(target, np.ndarray) or not isinstance(boat, np.ndarray):
            continue
        if target.shape == (2,) and boat.shape == (2,):
            if (target[0] is None) or (target[1] is None) or (boat[0] is None) or (boat[1] is None):
                continue
            cleaned_history.append((target, boat))
    
    if not cleaned_history:
        raise ValueError(f"Aucune donnée valide dans {npz_file}")
    
    # On échange les coordonnées pour passer de (x,y) à (y,x) si besoin (selon votre usage)
    targets = np.array([(h[0][1], h[0][0]) for h in cleaned_history])
    boats   = np.array([(h[1][1], h[1][0]) for h in cleaned_history])
    return targets, boats

def create_combined_trajectory_gif(npz_files, gif_output="combined_trajectory.gif", downsample=1):
    """
    Charge plusieurs fichiers NPZ (chacun correspondant à un bateau),
    et crée une animation GIF affichant toutes les trajectoires.
    
    Pour chaque bateau, la trajectoire est affichée jusqu'à sa fin,
    puis la dernière position est maintenue jusqu'à la fin de l'animation.
    """
    trajectories = []  # Liste de dicts pour chaque bateau
    for file in npz_files:
        try:
            targets, boats = load_trajectory(file)
            trajectories.append({
                "file": file,
                "targets": targets,
                "boats": boats,
                "length": len(targets)
            })
        except ValueError as e:
            print(e)
    
    if not trajectories:
        print("Aucune trajectoire valide n'a été chargée.")
        return

    # Déterminer le nombre total de frames (selon le bateau le plus long)
    max_frames = max(traj["length"] for traj in trajectories)
    
    # Définir les indices de frames à utiliser (downsample + dernière frame)
    frame_indices = list(range(0, max_frames, downsample))
    if frame_indices[-1] != max_frames - 1:
        frame_indices.append(max_frames - 1)
    
    # Calcul des bornes globales pour l'affichage
    all_x = []
    all_y = []
    for traj in trajectories:
        t = traj["targets"]
        b = traj["boats"]
        all_x.extend(t[:,0])
        all_x.extend(b[:,0])
        all_y.extend(t[:,1])
        all_y.extend(b[:,1])
    
    x_min, x_max = min(all_x), max(all_x)
    y_min, y_max = min(all_y), max(all_y)
    x_margin = 0.05 * (x_max - x_min) if (x_max - x_min) != 0 else 1
    y_margin = 0.05 * (y_max - y_min) if (y_max - y_min) != 0 else 1

    # Création de la figure
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(x_min - x_margin, x_max + x_margin)
    ax.set_ylim(y_min - y_margin, y_max + y_margin)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Animation des trajectoires (downsample={})".format(downsample))

    # Préparation des objets graphiques pour chaque bateau.
    # On utilisera un cycle de couleurs pour différencier les bateaux.
    color_cycle = itertools.cycle(plt.rcParams['axes.prop_cycle'].by_key()['color'])
    for traj in trajectories:
        c = next(color_cycle)
        # Trajectoire de la cible en noir, indépendamment du bateau
        traj["target_line"], = ax.plot([], [], '-', color='black', lw=1)
        # Trajectoire du bateau avec sa couleur
        traj["boat_line"],   = ax.plot([], [], '--', color=c, lw=1, label=f"Bateau {traj['file']}")
        # Position courante de la cible en gris avec un marqueur plus petit
        traj["current_target"], = ax.plot([], [], 'o', color='gray', markersize=4)
        # Position courante du bateau avec sa couleur et un marqueur plus grand
        traj["current_boat"],   = ax.plot([], [], 's', color=c, markersize=6)
    
    #ax.legend(loc='best')

    def init():
        """Initialise les lignes et marqueurs pour tous les bateaux."""
        for traj in trajectories:
            traj["target_line"].set_data([], [])
            traj["boat_line"].set_data([], [])
            traj["current_target"].set_data([], [])
            traj["current_boat"].set_data([], [])
        return [traj[obj] for traj in trajectories for obj in ["target_line", "boat_line", "current_target", "current_boat"]]

    def update(frame_idx):
        """
        Pour chaque frame de l'animation, affiche pour chaque bateau
        la trajectoire jusqu'à l'indice correspondant (ou la dernière si terminé).
        """
        global_frame = frame_indices[frame_idx]
        for traj in trajectories:
            length = traj["length"]
            # Déterminer jusqu'où on peut aller dans la trajectoire de ce bateau
            current_index = global_frame if global_frame < length else length - 1
            
            # Extraire les positions jusqu'au current_index
            used_indices = list(range(0, current_index+1))
            t = traj["targets"]
            b = traj["boats"]
            
            # Mettre à jour la ligne de la cible
            traj["target_line"].set_data(t[used_indices, 0], t[used_indices, 1])
            # Mettre à jour la ligne du bateau
            traj["boat_line"].set_data(b[used_indices, 0], b[used_indices, 1])
            # Mettre à jour les marqueurs de position courante
            traj["current_target"].set_data(t[current_index, 0], t[current_index, 1])
            traj["current_boat"].set_data(b[current_index, 0], b[current_index, 1])
        return [traj[obj] for traj in trajectories for obj in ["target_line", "boat_line", "current_target", "current_boat"]]

    ani = FuncAnimation(
        fig,
        update,
        frames=len(frame_indices),
        init_func=init,
        blit=True,
        interval=20  # durée en ms par frame (ajustez si nécessaire)
    )

    ani.save(gif_output, writer='pillow', fps=30)
    print(f"Animation sauvegardée dans {gif_output}")
    plt.close(fig)

if __name__ == "__main__":
    # Liste des fichiers de trajectoires
    files = [
        "log/trajectory02.npz",
        "log/trajectory04.npz",
        "log/trajectory05.npz",
        "log/trajectory09.npz",
        "log/trajectory12.npz",
    ]
    create_combined_trajectory_gif(files, "log/combined_trajectory.gif", downsample=5)
