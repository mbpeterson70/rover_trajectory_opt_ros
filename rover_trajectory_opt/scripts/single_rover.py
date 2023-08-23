import numpy as np
import matplotlib.pyplot as plt

from tomma.multi_agent_optimization import MultiAgentOptimization
from tomma.dubins_dynamics import DubinsDynamics, CONTROL_LIN_ACC_ANG_VEL

from create_and_store_trajectories import create_and_store_trajectories

# parameters

num_rovers = 1
num_timesteps = 100
min_allowable_dist = 1.5

x_bounds = np.array([[-7.0, 7.], [-7.0, 7.0], [-.5, 1.0], [-np.inf, np.inf]])
u_bounds = np.array([[-1.0, 1.], [-0.785, 0.785]])

obstacles = []

waypoints = np.array([
    [
        [-5., -5., np.pi/4],
        [5, 5, np.pi/4],
        [5, -5, 3*np.pi/4],
        [-5, 5, 3*np.pi/4],
        [-5., -5., np.pi/4]
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