#!/usr/bin/env python3

"""
This node generates and publishes a trajectory for rovers 
to navigate to a desired final pose
"""

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_srvs.srv import Trigger
from rover_trajectory_msgs.msg import RoverState

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from casadi_trajectory_optimization.dubins_dynamics import DubinsDynamics, CONTROL_LIN_ACC_ANG_VEL
from casadi_trajectory_optimization.multi_agent_planner import MultiAgentPlanner

class TrajectoryGeneratorNode():
    def __init__(self):
        self.node_name = rospy.get_name()
        
        # Params
        self.rovers = rospy.get_param("~trajectory_generator/rovers")
        self.dt = rospy.get_param("~trajectory_generator/dt")
        num_timesteps = rospy.get_param("~trajectory_generator/num_timesteps")
        self.goal_states = {f'{rover}': np.array([rospy.get_param(f"~trajectory_generator/goal_states/{rover}")]) for rover in self.rovers}
        self.xf_rover_states = dict()
        for rover, xf in self.goal_states.items():
            xf_rover_state = RoverState()
            xf_rover_state.x, xf_rover_state.y, xf_rover_state.v, xf_rover_state.theta = xf.reshape(-1).tolist()
            self.xf_rover_states[rover] = xf_rover_state
        x_bounds = np.array(rospy.get_param(f"~trajectory_generator/x_bounds"))
        u_bounds = np.array(rospy.get_param(f"~trajectory_generator/u_bounds"))
        self.x_bounds = np.zeros(x_bounds.shape)
        self.u_bounds = np.zeros(u_bounds.shape)
        for i in range(self.x_bounds.shape[0]):
            for j in range(self.x_bounds.shape[1]):
                self.x_bounds[i,j] = float(x_bounds[i,j])
        for i in range(self.u_bounds.shape[0]):
            for j in range(self.u_bounds.shape[1]):
                self.u_bounds[i,j] = float(u_bounds[i,j])
        # self.publish_control = rospy.get_param("~trajectory_publisher/publish_control")
        
        # Internal variables
        self.states = {f'{rover}': np.nan*np.ones(4) for rover in self.rovers}
        self.planner = MultiAgentPlanner(dynamics=DubinsDynamics(control=CONTROL_LIN_ACC_ANG_VEL), 
                                         num_agents=len(self.rovers), 
                                         num_timesteps=num_timesteps)
        self.traj_planned = False
        self.t = 0.0
        self.rover_idx = {f'{rover}': i for i, rover in enumerate(self.rovers)}
        
        # Pub & Sub
        self.sub_pose = [rospy.Subscriber(f"{rover}/world", PoseStamped, self.pose_cb, queue_size=1, callback_args=rover) for rover in self.rovers]
        self.sub_twist = [rospy.Subscriber(f"{rover}/mocap/twist", TwistStamped, self.twist_cb, queue_size=1, callback_args=rover) for rover in self.rovers]
        self.pub_traj = {f'{rover}': rospy.Publisher(f"{rover}/trajectory", RoverState, queue_size=1) for rover in self.rovers}
        # self.pub_auto_cmd = {f'{rover}': rospy.Publisher(f"{rover}/cmd_vel_auto", Twist, queue_size=1) for rover in self.rovers}
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop_cb)
                
    def loop_cb(self, event):
        if self.traj_planned:
            for rover in self.rovers:
                if self.t > self.tf:
                    self.pub_traj[rover].publish(self.xf_rover_states[rover])
                    continue
                rover_state = RoverState()
                rover_state.x = np.interp(self.t, xp=self.t_traj, fp=self.x_traj[rover][0,:-1])
                rover_state.y = np.interp(self.t, xp=self.t_traj, fp=self.x_traj[rover][1,:-1])
                rover_state.v = np.interp(self.t, xp=self.t_traj, fp=self.x_traj[rover][2,:-1])
                rover_state.theta = np.interp(self.t, xp=self.t_traj, fp=self.x_traj[rover][3,:-1])
                self.pub_traj[rover].publish(rover_state)

                # if self.t > self.tf:
                #     self.pub_auto_cmd[rover].publish(Twist())
                #     continue
                # v_cmd = np.interp(self.t, xp=self.t_traj, fp=self.x_traj[rover][2,:-1])
                # th_dot_cmd = np.interp(self.t, xp=self.t_traj, fp=self.u_traj[rover][1,:])
                # cmd_vel = Twist()
                # cmd_vel.linear.x = v_cmd
                # cmd_vel.angular.z = th_dot_cmd
                # self.pub_auto_cmd[rover].publish(cmd_vel)
            self.t += self.dt
            return
        else:
            if self.all_states_received():
                # plan trajectory
                print('Planning trajectory...')
                x0 = np.zeros((len(self.rovers), 4))
                xf = np.zeros((len(self.rovers), 4))
                for i, rover in enumerate(self.rovers):
                    x0[i,:] = self.states[rover]
                    xf[i,:] = self.goal_states[rover]
                self.planner.setup_min_time_opt(x0, xf, tf_guess=10.0, x_bounds=self.x_bounds, u_bounds=self.u_bounds)
                self.planner.opti.subject_to(self.planner.tf > 1.)
                x, u, tf = self.planner.solve_opt()
                print('Executing trajectory...')
                self.x_traj = {f'{rover}': x[i] for i, rover in enumerate(self.rovers)}
                self.u_traj = {f'{rover}': u[i] for i, rover in enumerate(self.rovers)}
                self.tf = tf
                self.t_traj = np.linspace(0.0, self.tf, self.planner.N)
                self.traj_planned = True
            else:
                return # wait for all starting positions to be known
        
    def pose_cb(self, pose_stamped, rover):
        self.states[rover][0] = pose_stamped.pose.position.x
        self.states[rover][1] = pose_stamped.pose.position.y
        quat = pose_stamped.pose.orientation
        theta_unwrapped = Rot.from_quat([quat.x, quat.y, quat.w, quat.z]).as_euler('xyz')[2] + np.pi # add pi because of how theta is defined in Dubins dynamis
        self.states[rover][3] = -((theta_unwrapped + np.pi) % (2 * np.pi) - np.pi) # wrap
        
    def twist_cb(self, twist_stamped, rover):
        theta = self.states[rover][3]
        if np.isnan(theta):
            return
        self.states[rover][2] = twist_stamped.twist.linear.x*np.cos(theta) + twist_stamped.twist.linear.y*np.sin(theta)
        # TODO: did I get that right?
        
    def all_states_received(self):
        for rover in self.rovers:
            if np.any(np.isnan(self.states[rover])):
                return False
        return True
            
    
if __name__ == '__main__':
    rospy.init_node('trajectory_generator_node', anonymous=False)
    node = TrajectoryGeneratorNode()
    rospy.spin()
