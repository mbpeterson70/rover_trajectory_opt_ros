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

from casadi_trajectory_optimization.dubins_dynamics import DubinsDynamics, CONTROL_LIN_ACC_ANG_VEL, CONTROL_LIN_VEL_ANG_VEL
from casadi_trajectory_optimization.multi_agent_planner import MultiAgentPlanner

class ModelPredictiveControlNode():
    def __init__(self):
        self.node_name = rospy.get_name()
        
        # Params
        self.dt = rospy.get_param("~mpc/dt")
        self.mpc_num_timesteps = rospy.get_param("~mpc/num_timesteps")
        self.mpc_tf = rospy.get_param("~mpc/tf")
        x_bounds = np.array(rospy.get_param(f"~mpc/x_bounds"))
        u_bounds = np.array(rospy.get_param(f"~mpc/u_bounds"))
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
        self.state = np.nan*np.ones(4)
        self.ref_state = np.nan*np.ones(4)
        self.planner = MultiAgentPlanner(dynamics=DubinsDynamics(control=CONTROL_LIN_VEL_ANG_VEL), 
                                         num_agents=1, 
                                         num_timesteps=self.mpc_num_timesteps)
        
        # Pub & Sub
        self.sub_pose = rospy.Subscriber(f"world", PoseStamped, self.pose_cb, queue_size=1)
        self.sub_twist = rospy.Subscriber(f"mocap/twist", TwistStamped, self.twist_cb, queue_size=1)
        self.sub_ref = rospy.Subscriber(f"trajectory", RoverState, self.ref_cb, queue_size=1)
        self.pub_auto_cmd = rospy.Publisher("cmd_vel_auto", Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop_cb)
                
    def loop_cb(self, event):
        if self.states_received() and self.ref_received():
            x0 = np.concatenate([self.state[0:2], [self.state[3]]]).reshape((1,3))
            xf = np.concatenate([self.ref_state[0:2], [self.ref_state[3]]]).reshape((1,3))
            # print(self.states_received())
            # print(self.state.reshape((1, 4)))
            # print(xf)
            # print(self.mpc_tf)
            self.planner.setup_mpc_opt(x0, xf, tf=self.mpc_tf, x_bounds=self.x_bounds, u_bounds=self.u_bounds)
            # self.planner.opti.subject_to(self.planner.tf > 1.)
            x, u, tf = self.planner.solve_opt()
            v_cmd = u[0][0,0]
            th_dot_cmd = u[0][1,0]
            cmd_vel = Twist()
            cmd_vel.linear.x = v_cmd
            cmd_vel.angular.z = th_dot_cmd
            print(v_cmd)
            self.pub_auto_cmd.publish(cmd_vel)
        else:
            self.pub_auto_cmd.publish(Twist())
        
    def pose_cb(self, pose_stamped):
        self.state[0] = pose_stamped.pose.position.x
        self.state[1] = pose_stamped.pose.position.y
        quat = pose_stamped.pose.orientation
        theta_unwrapped = Rot.from_quat([quat.x, quat.y, quat.w, quat.z]).as_euler('xyz')[2] + np.pi # add pi because of how theta is defined in Dubins dynamis
        self.state[3] = -((theta_unwrapped + np.pi) % (2 * np.pi) - np.pi) # wrap
        
    def twist_cb(self, twist_stamped):
        theta = self.state[3]
        if np.isnan(theta):
            return
        self.state[2] = twist_stamped.twist.linear.x*np.cos(theta) + twist_stamped.twist.linear.y*np.sin(theta)
        # TODO: did I get that right?

    def ref_cb(self, rover_state):
        self.ref_state = np.array([rover_state.x, rover_state.y, rover_state.v, rover_state.theta])
        
    def states_received(self):
        print(self.state)
        return not np.any(np.isnan(self.state))
        
    def ref_received(self):
        print(self.ref_state)
        return not np.any(np.isnan(self.ref_state))
            
    
if __name__ == '__main__':
    rospy.init_node('mpc_node', anonymous=False)
    node = ModelPredictiveControlNode()
    rospy.spin()
