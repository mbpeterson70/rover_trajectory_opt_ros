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
from scipy.interpolate import interp1d

from tomma.dubins_dynamics import DubinsDynamics, CONTROL_LIN_ACC_ANG_VEL, CONTROL_LIN_VEL_ANG_VEL
from tomma.multi_agent_optimization import MultiAgentOptimization

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
        self.u_diff_bounds = np.array([3., 3.])
        self.R = np.diag([.05, .05])
        for i in range(self.x_bounds.shape[0]):
            for j in range(self.x_bounds.shape[1]):
                self.x_bounds[i,j] = float(x_bounds[i,j])
        for i in range(self.u_bounds.shape[0]):
            for j in range(self.u_bounds.shape[1]):
                self.u_bounds[i,j] = float(u_bounds[i,j])
        # self.publish_control = rospy.get_param("~trajectory_publisher/publish_control")
        
        # Internal variables
        self.state = np.nan*np.ones(5)
        self.ref_times = []
        self.ref_state = []
        self.planner = MultiAgentOptimization(dynamics=DubinsDynamics(control=CONTROL_LIN_VEL_ANG_VEL), 
                                         num_agents=1, 
                                         num_timesteps=self.mpc_num_timesteps)
        
        # Pub & Sub
        self.sub_pose = rospy.Subscriber(f"world", PoseStamped, self.pose_cb, queue_size=1)
        self.sub_twist = rospy.Subscriber(f"mocap/twist", TwistStamped, self.twist_cb, queue_size=1)
        self.sub_ref = rospy.Subscriber(f"trajectory", RoverState, self.ref_cb, queue_size=1)
        self.pub_auto_cmd = rospy.Publisher("cmd_vel_auto", Twist, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop_cb)
        self.last_ref_time = None
        self.last_pose_time = None
        self.no_msg_time = .2
                
    def loop_cb(self, event):
        if self.states_received() and self.ref_received():
            if (rospy.Time.now() - self.last_pose_time).to_sec() > self.no_msg_time or \
                (rospy.Time.now() - self.last_ref_time).to_sec() > self.no_msg_time:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.
                cmd_vel.angular.z = 0.
                self.pub_auto_cmd.publish(cmd_vel)
                return
            x0 = np.concatenate([self.state[0:2], [self.state[3]]]).reshape((1,3))
            xf = np.concatenate([self.ref_state[-1][0:2], [self.ref_state[-1][3]]]).reshape((1,3))
            waypoints = self._get_waypoints()
            # print(self.states_received())
            # print(self.state.reshape((1, 4)))
            # print(xf)
            # print(self.mpc_tf)
            self.planner.setup_mpc_opt(x0, xf, R=self.R, tf=self.mpc_tf, waypoints=waypoints, x_bounds=self.x_bounds, u_bounds=self.u_bounds)
            self.planner.add_u_diff_bounds(self.u_diff_bounds)
            # constrain initial forward velocity
            self.planner.add_u0_constraint(np.array([self.state[2], self.state[4]]))
            # self.planner.opti.subject_to(self.planner.tf > 1.)
            try:
                x, u, t = self.planner.solve_opt()
            
                # use second command since first is constrained to be current 
                v_cmd = u[0][0,1]
                # TODO: [0,1] ? since theta_dot init not constrained
                th_dot_cmd = u[0][1,1]
                cmd_vel = Twist()
                cmd_vel.linear.x = v_cmd
                cmd_vel.angular.z = th_dot_cmd
                # print(v_cmd)
                self.pub_auto_cmd.publish(cmd_vel)
            except:
                print('planner failed')
        else:
            self.pub_auto_cmd.publish(Twist())
        
    def pose_cb(self, pose_stamped):
        self.last_pose_time = rospy.Time.now()
        self.state[0] = pose_stamped.pose.position.x
        self.state[1] = pose_stamped.pose.position.y
        quat = pose_stamped.pose.orientation
        theta_unwrapped = Rot.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]
        self.state[3] = ((theta_unwrapped + np.pi) % (2 * np.pi) - np.pi) # wrap
        
    def twist_cb(self, twist_stamped):
        theta = self.state[3]
        if np.isnan(theta):
            return
        self.state[2] = twist_stamped.twist.linear.x*np.cos(theta) + twist_stamped.twist.linear.y*np.sin(theta)
        self.state[4] = twist_stamped.twist.angular.z
        # TODO: did I get that right?

    def ref_cb(self, rover_state):
        self.last_ref_time = rospy.Time.now()
        theta_ref = rover_state.theta
        # TODO: don't use euler angles in trajectory optimization
        if self.state[3] - theta_ref > np.pi:
            theta_ref += 2*np.pi
        elif theta_ref - self.state[3] > np.pi:
            theta_ref -= 2*np.pi
        self.ref_state.append(np.array([rover_state.x, rover_state.y, rover_state.v, theta_ref]))
        self.ref_times.append(rover_state.t)
        
    def states_received(self):
        print(self.state)
        return not np.any(np.isnan(self.state))
        
    def ref_received(self):
        print(self.ref_state)
        # make sure we have enough reference times to make up a trajectory
        return len(self.ref_times) > 0 and \
            self.ref_times[-1] - self.ref_times[0] > self.mpc_tf
            
    def _get_waypoints(self):
        tf = self.ref_times[-1]
        while self.ref_times[1] < tf - self.mpc_tf:
            self.ref_times = self.ref_times[1:]
            self.ref_state = self.ref_state[1:]
        
        t0 = tf - self.mpc_tf
        waypoints = {}

        trajectory_interp = interp1d(self.ref_times, self.ref_state, axis=0)
        for i in range(1, self.mpc_num_timesteps):
            waypoint = trajectory_interp(t0 + i*self.mpc_tf/self.mpc_num_timesteps)
            waypoints[i] = np.concatenate([waypoint[0:2], [waypoint[3]]]).reshape((1,3))
            
        return waypoints
        
            
    
if __name__ == '__main__':
    rospy.init_node('mpc_node', anonymous=False)
    node = ModelPredictiveControlNode()
    rospy.spin()
