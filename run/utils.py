from run.parameters import *

import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation
from matplotlib.collections import EllipseCollection
import numpy as np 


def humble_plot(trajectories, obstacles:list[np.ndarray]=[]):
    n_agents = trajectories.shape[1]

    for i in range(n_agents): 
        plt.plot(trajectories[:,i,0], trajectories[:,i,1])

    for j in range(len(obstacles)): 
        obs = obstacles[j]
        plt.plot(obs[:, 0], obs[:, 1], color="black", lw=2) 
    
    plt.gca().set_aspect("equal")
    plt.show() 

def animate_run(trajectories:np.ndarray, actives:np.ndarray, obstacles:list[np.ndarray]=[], interval=50, save=None):
    """Animates a run step by step. save="run.mp4" / "run.gif" writes the file instead of showing it."""

    fig, ax = plt.subplots()
    n_frames = trajectories.shape[0]
    n_agents = trajectories.shape[1]
    actives = np.asarray(actives, dtype=bool)

    # fixed limits over the whole run (agents and obstacles), so the view does not jump
    m = 2 * RADIUS
    points = np.concatenate([trajectories.reshape(-1, 2), *obstacles]) if obstacles else trajectories.reshape(-1, 2)
    ax.set_xlim(points[:, 0].min() - m, points[:, 0].max() + m)
    ax.set_ylim(points[:, 1].min() - m, points[:, 1].max() + m)
    ax.set_aspect("equal")

    # static obstacles
    for obs in obstacles:
        ax.plot(obs[:, 0], obs[:, 1], color="black", lw=2)

    # one color per agent id, kept the same over the whole run
    colors = plt.get_cmap("tab20")(np.arange(n_agents) % 20)

    # trajectories drawn so far, one faded line per agent
    trails = [ax.plot([], [], color=colors[n], lw=1, alpha=0.4)[0] for n in range(n_agents)]

    # agents as disks of their true radius (data units)
    disks = EllipseCollection([], [], [], units="xy", offsets=np.empty((0, 2)), offset_transform=ax.transData)
    ax.add_collection(disks)
    title = ax.set_title("")

    def update(i):
        # only active agents are drawn as disks
        a = actives[i]
        n_active = int(a.sum())
        disks.set_offsets(trajectories[i, a])
        disks.set_widths(np.full(n_active, 2 * RADIUS))
        disks.set_heights(np.full(n_active, 2 * RADIUS))
        disks.set_angles(np.zeros(n_active))
        disks.set_facecolors(colors[a])

        for n, trail in enumerate(trails):
            trail.set_data(trajectories[:i+1, n, 0], trajectories[:i+1, n, 1])

        title.set_text(f"step {i}/{n_frames-1}   t = {i*TIME_STEP:.1f} s   active {n_active}/{n_agents}")
        return disks, title, *trails

    anim = FuncAnimation(fig, update, frames=n_frames, interval=interval, blit=False)
    if save:
        anim.save(save, fps=1000 // interval)
    else:
        plt.show()
    return anim