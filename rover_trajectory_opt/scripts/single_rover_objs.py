import numpy as np
import matplotlib.pyplot as plt

from tomma.multi_agent_optimization import MultiAgentOptimization
from tomma.dubins_dynamics import DubinsDynamics, CONTROL_LIN_ACC_ANG_VEL

from create_and_store_trajectories import create_and_store_trajectories

# parameters

num_rovers = 1
num_timesteps = 100
min_allowable_dist = 1.5

x_bounds = np.array([[-5.0, 5.], [-5.0, 5.0], [-.5, 0.6], [-np.inf, np.inf]])
u_bounds = np.array([[-0.25, 0.25], [-0.785, 0.785]])

obstacles = np.array([
    [0.5, -3.0, 1.],
    [-.9, -2.6, 1.],
    [2.2, 1.8, 1.],
    [1.8, 2.7, 1.],
    [-3.8, 1.6, 1.],
    [-3., 2.7, 1.]
])

waypoints = np.array([
    [
        [-3., -3, 0],
        [3, -3, 0.],
        [0, 3., np.pi/2],
        [-3., -3., 0.]
    ],
])
output_file = '/home/swarm/data/trajectories/single.json'

# setup

planner = MultiAgentOptimization(dynamics=DubinsDynamics(control=CONTROL_LIN_ACC_ANG_VEL), 
                                num_agents=num_rovers, 
                                num_timesteps=num_timesteps,
                                min_allowable_dist=min_allowable_dist,
                                x_bounds=x_bounds,
                                u_bounds=u_bounds)
for obstacle in obstacles:
    planner.add_obstacle(position=obstacle[:2], radius=obstacle[2])

create_and_store_trajectories(planner=planner, num_rovers=num_rovers, waypoints=waypoints, output_file=output_file)
plt.show()
