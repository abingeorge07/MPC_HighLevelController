"""
Docstring for visuals.generate_trajectory.py
Generates and saves a trajectory plot from logged MPC data.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import json
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


# def plot_obstacle(Ox, Oy, length, width, alpha, ax):
#     """
#     Plots a rectangle obstacle at the given center with rotation (yaw).
#     """

#     # Rectangle centered at origin
#     rect = Rectangle(
#         (-length / 2, -width / 2),
#         length,
#         width,
#         fill=True,
#         edgecolor='black',
#         facecolor='blue',
#         linewidth=1.5
#     )

#     # Apply rotation + translation
#     transform = (
#         Affine2D()
#         .rotate(alpha)
#         .translate(Ox, Oy)
#         + ax.transData
#     )

#     rect.set_transform(transform)
    
#     ax.add_patch(rect)

def plot_obstacle(Ox, Oy, length, width, alpha, ax):
    """
    Plot a rotated rectangle obstacle in DATA coordinates.
    alpha is yaw in radians.
    """

    rect = Rectangle(
        (Ox - length / 2, Oy - width / 2),
        length,
        width,
        angle=np.degrees(alpha),   # Rectangle expects degrees
        edgecolor="black",
        facecolor="blue",
        linewidth=1.5,
        zorder=2
    )

    ax.add_patch(rect)


def plot_all_obstacles(obstacle_data, ax): 
    """ Plots obstacles from the given obstacle data. """ 
    
    xmin, xmax = np.inf, -np.inf
    ymin, ymax = np.inf, -np.inf

    for data in obstacle_data: 
        Ox = data["position"][0] 
        Oy = data['position'][1] 
        length = data['size'][0] 
        width = data['size'][1] 
        alpha = data['euler'][2] 
        # label = data.get('label', 'Obstacle') 
        plot_obstacle(Ox, Oy, length, width, alpha, ax)

        xmin = min(xmin, Ox - length / 2)
        xmax = max(xmax, Ox + length / 2)
        ymin = min(ymin, Oy - width / 2)
        ymax = max(ymax, Oy + width / 2)

    return xmin, xmax, ymin, ymax
        

if __name__ == "__main__":

    # Load json file
    json_file = "../Obstacle_Info/obstacle.json"
    with open(json_file, 'r') as f:
        obstacle_data = json.load(f) 

    # Get the figure 
    fig, ax = plt.subplots()

    # Plot the obstacles
    obs_x_min, obs_x_max, obs_y_min, obs_y_max = plot_all_obstacles(obstacle_data, ax)

    # Load logged data
    log_data = pd.read_csv("mpc_log.csv")

    x = log_data['x'].to_numpy()
    y = log_data['y'].to_numpy()
    theta = log_data['theta'].to_numpy()

    # Find the figure limits
    x_min = min(np.min(x), obs_x_min)
    x_max = max(np.max(x), obs_x_max)
    y_min = min(np.min(y), obs_y_min)
    y_max = max(np.max(y), obs_y_max)


    # Plot trajectory
    ax.plot(x, y, 'b--', linewidth=1, label="Trajectory")

    # Arrow directions
    arrow_length = 0.05  # adjust for visibility
    u = arrow_length * np.cos(theta)
    v = arrow_length * np.sin(theta)

    # Plot orientation arrows
    ax.quiver(
        x, y, u, v,
        angles='xy',
        scale_units='xy',
        color='red',
        width=0.008,
        headwidth=2,
        headlength=4,
        label="Heading"
    )

    # Formatting
    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_title('MPC Trajectory with Orientation')
    ax.axis('equal')
    ax.grid(False)
    ax.legend()


    # Set axis limits with some padding
    padding = 1.0
    ax.set_aspect("equal")
    ax.set_xlim(x_min - padding, x_max + padding)
    ax.set_ylim(y_min - padding, y_max + padding) 

    plt.show()

    # Save and show
    fig.savefig('trajectory_plot.png', dpi=150, bbox_inches='tight')
