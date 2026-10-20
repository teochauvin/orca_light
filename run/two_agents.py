from run._ext import rvo2 
from run.parameters import * 
from run.utils import animate_run, humble_plot

import numpy as np 
import matplotlib.pyplot as plt 

if __name__ == "__main__": 

    # Example parameters 
    n_agents = 2
    initial_actives = np.ones(n_agents, dtype=bool)
    initial_positions = np.array([[-10.0, 0.0], [7.0, -7.0]]) 
    goals = np.array([[10.0, 0.0], [-7.0, 7.0]]) 

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

            # The agent is set inactive
            a[i] = 0 
            simulator.setAgentActiveFlag(i, False) 

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
    animate_run(positions, actives)


    