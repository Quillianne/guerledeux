import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def create_trajectory_gif(npz_file, gif_output="trajectory.gif", downsample=1):
    """
    Load history from an NPZ file and create a GIF of the evolution of:
      - Target (x,y) in black
      - Boat (px, py) in green
    Also shows the final positions with markers, with the option
    to downsample intermediate frames by 'downsample' factor.
    """
    data = np.load(npz_file, allow_pickle=True)
    history = data["history"]  # Array of tuples: ( (x,y), (px,py) )

    # Filter out any frames where target or boat is None or not numeric
    cleaned_history = []
    for (target, boat) in history:
        # Basic sanity checks
        if target is None or boat is None:
            continue
        if not isinstance(target, np.ndarray) or not isinstance(boat, np.ndarray):
            continue
        if target.shape == (2,) and boat.shape == (2,):
            # Also check we have no None entries inside
            if (target[0] is None) or (target[1] is None) \
               or (boat[0] is None) or (boat[1] is None):
                continue
            cleaned_history.append((target, boat))

    if not cleaned_history:
        print("No valid (non-None) frames found in history.")
        return

    # Convert to arrays
    targets = np.array([(h[0][1],h[0][0]) for h in cleaned_history])  # shape: (N, 2)
    boats   = np.array([(h[1][1],h[1][0]) for h in cleaned_history])  # shape: (N, 2)

    # Determine downsampled frames (indices)
    # Example: downsample=10 => show frames 0,10,20,... plus the final
    total_frames = len(cleaned_history)
    frame_indices = list(range(0, total_frames, downsample))
    if frame_indices[-1] != total_frames - 1:
        frame_indices.append(total_frames - 1)

    # For convenience, define a quick "used_indices" array for each frame
    # so we can show the trajectory *up to* that point in the downsample.
    # Each animation frame is an integer f in [0, len(frame_indices)-1].
    # The actual data index is frame_indices[f].
    # We only plot frames corresponding to the *downsampled* set.
    
    # Now for overall min/max, we still consider all points or only the subset?
    # Typically, you'd want the axis to account for all points, not just downsampled.
    all_x = np.concatenate((targets[:, 0], boats[:, 0]))
    all_y = np.concatenate((targets[:, 1], boats[:, 1]))

    x_min, x_max = np.min(all_x), np.max(all_x)
    y_min, y_max = np.min(all_y), np.max(all_y)

    # Add 5% margin
    x_range = x_max - x_min
    y_range = y_max - y_min
    x_margin = 0.05 * x_range
    y_margin = 0.05 * y_range

    # Final positions (persistently displayed)
    final_target = targets[-1]
    final_boat   = boats[-1]

    # Create figure
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(x_min - x_margin, x_max + x_margin)
    ax.set_ylim(y_min - y_margin, y_max + y_margin)
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("Trajectory Animation (Downsample={})".format(downsample))

    # Line objects for the evolving trajectory
    target_line, = ax.plot([], [], 'k-', lw=1, label='Target trajectory')
    boat_line,   = ax.plot([], [], 'g-', lw=1, label='Boat trajectory')

    # Markers for the *current* point
    current_target, = ax.plot([], [], 'ko', markersize=5)
    current_boat,   = ax.plot([], [], 'go', markersize=5)


    ax.legend(loc='best')

    def init():
        """Initialize empty data for animation."""
        target_line.set_data([], [])
        boat_line.set_data([], [])
        current_target.set_data([], [])
        current_boat.set_data([], [])
        return (target_line, boat_line, current_target, current_boat)

    def update(frame):
        """
        For the f-th step in the animation, the actual data index is frame_indices[f].
        We'll plot all downsampled indices up to f, so the line is 'stepping' along.
        """
        data_index = frame_indices[frame]
        used_indices = frame_indices[:frame+1]  # all indices up to current frame

        # Extract x,y up to current downsampled point
        # (so the line is only drawn through the downsampled positions)
        t_x = targets[used_indices, 0]
        t_y = targets[used_indices, 1]
        b_x = boats[used_indices, 0]
        b_y = boats[used_indices, 1]

        target_line.set_data(t_x, t_y)
        boat_line.set_data(b_x, b_y)

        # Mark the current point (the last in used_indices)
        current_target.set_data(targets[data_index, 0], targets[data_index, 1])
        current_boat.set_data(boats[data_index, 0], boats[data_index, 1])

        return (target_line, boat_line, current_target, current_boat)

    # Create the animation with frames = number of downsampled frames
    ani = FuncAnimation(
        fig,
        update,
        frames=len(frame_indices),
        init_func=init,
        blit=True,
        interval=10  # ms per frame
    )

    # Save as GIF
    ani.save(gif_output, writer='pillow', fps=20)
    print(f"Animation saved to {gif_output}")

    plt.close(fig)


if __name__ == "__main__":
    # Example usage: downsample by 10
    create_trajectory_gif("trajectory.npz", "trajectory.gif", downsample=10)
