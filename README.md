# Rover Trajectory Optimization ROS Package

For use with [Casadi Trajectory Optimization](https://github.com/mbpeterson70/casadi_trajectory_optimization/tree/main)

![animation](media/multi_ani.gif)
![video](media/multi_vid.gif)

## Install

Install with the typical `catkin build` ROS installation process.

## Nodes

* The [trajectory generator node](rover_trajectory_opt/nodes/trajectory_generator_node.py) generates trajectories from the rovers' current states to an end desired state.
* The [MPC node](rover_trajectory_opt/nodes/mpc_node.py) runs model predictive control to follow the trajectory.
