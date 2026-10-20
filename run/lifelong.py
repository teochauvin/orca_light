from run._ext import rvo2 
from run.parameters import * 
from run.utils import animate_run, humble_plot

import numpy as np 

if __name__ == "__main__": 

    # Example parameters 
    n_agents = 200
    area_side = 50.0 
    half = area_side/2 

    # Random positions with no initial collisions 
    initial_positions = np.empty((n_agents, 2))
    for n in range(n_agents):
        for _ in range(1_000):
            p = np.random.uniform(-half, half, size=2)
            if n == 0 or np.linalg.norm(initial_positions[:n] - p, axis=1).min() >= 2 * RADIUS:
                initial_positions[n] = p
                break

    initial_actives = np.ones(n_agents, dtype=bool)
    goals = np.random.uniform(-half, half, size=(n_agents, 2))

    # Historical data 
    positions = [] 
    actives = []

    # Initialize the simulator 
    simulator = rvo2.RVOSimulator() 
    simulator.setTimeStep(TIME_STEP) 

    # Initialize all agents 
    for i in range(n_agents): 

        simulator.addAgent(
            rvo2.Vector2(*initial_positions[i]), 
            NEIGHBOR_DIST, 
            MAX_NEIGHBORS,
            TIME_HORIZON,
            TIME_HORIZON_OBST,
            RADIUS,
            MAX_SPEED) 

        # All agents are active at t=0.
        # i.e. all agents start at the same time t=0 
        simulator.setAgentActiveFlag(i, True)  

    # The simulation loop 
    for k in range(MAX_SIMULATION_STEPS): 

        # Get all agents positions 
        p = np.array([(q.x, q.y) for q in map(simulator.getAgentPosition, range(n_agents))])
        a = np.array([a for a in map(simulator.getAgentActiveFlag, range(n_agents))])

        # Update historical data 
        positions.append(p)
        actives.append(a)

        # Destination 
        to_goal = goals - p 
        dist = np.linalg.norm(to_goal, axis=1)

        # For all agents that are active and that just arrived at destination 
        for i in np.flatnonzero(a & (dist < EPS)):  

            # New goal 
            goals[i] = np.random.uniform(-half, half, size=2)


        # If all agents arrived at destination, then the simulation 
        if not a.any(): 
            break 

        # Avoid overshooting the target 
        speed = np.minimum(MAX_SPEED, dist/TIME_STEP) 
        v_pref = (to_goal / np.maximum(1e-12, dist)[:, None]) * speed[:, None] 

        # inactive agents do not have a preferred velocity
        v_pref[~a] = 0 

        # Set up the preferred velocities for each agent
        for i, (vx, vy) in enumerate(v_pref):
            simulator.setAgentPrefVelocity(i, rvo2.Vector2(vx, vy))

        # Do one simulation step 
        simulator.doStep()

    # Convert in numpy array shape = (K frames, N agents, D dimensions)
    positions = np.asarray(positions)
    actives = np.asarray(actives)

    # Plot 
    humble_plot(positions)

    # Animate 
    animate_run(positions, actives, save="lifelong.gif")



    