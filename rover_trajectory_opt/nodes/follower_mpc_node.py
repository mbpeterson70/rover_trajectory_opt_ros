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

from tomma.dubins_dynamics import DubinsDynamics, CONTROL_LIN_ACC_ANG_VEL, CONTROL_LIN_VEL_ANG_VEL
from tomma.multi_agent_optimization import MultiAgentOptimization

from robot_utils.robot_data import ArrayData
from robot_utils.exceptions import NoDataNearTimeException

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
        self.ref_states = ArrayData(time_array=None, data_array=None, interp=True, time_tol=1.0)
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
            xf = np.concatenate([self.ref_states._data[-1][0:2], [self.ref_states._data[-1][3]]]).reshape((1,3))
            
            waypoints = {}
            Q_waypoints = {}
            for i in range(self.mpc_num_timesteps):
                t = self.ref_states.times[-1] - self.mpc_tf + i*self.mpc_num_timesteps/self.mpc_tf
                try:
                    wp = self.ref_states.data(t)
                except NoDataNearTimeException as ex:
                    continue
                waypoints[i] = np.concatenate([wp[0:2], [wp[3]]]).reshape((1,3))
                Q_waypoints[i] = np.eye(3)*i
            Qf = np.eye(3)*self.mpc_num_timesteps
            
            self.planner.setup_mpc_opt(x0, xf, R=self.R, tf=self.mpc_tf, waypoints=waypoints, Q_waypoints=Q_waypoints, Qf=Qf, x_bounds=self.x_bounds, u_bounds=self.u_bounds)
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
        while self.state[3] - theta_ref > np.pi:
            theta_ref += 2*np.pi
        while theta_ref - self.state[3] > np.pi:
            theta_ref -= 2*np.pi
        ref_state = np.array([rover_state.x, rover_state.y, rover_state.v, theta_ref])
        if self.ref_states.times is None:
            self.ref_states.times = np.array([rover_state.t])
            self.ref_states._data = ref_state.reshape((1,4))
        else:
            self.ref_states.times = np.append(self.ref_states.times, rover_state.t)
            self.ref_states._data = np.append(self.ref_states._data, ref_state.reshape((1,4)), axis=0)
        
    def states_received(self):
        print(self.state)
        return not np.any(np.isnan(self.state))
        
    def ref_received(self):
        if self.ref_states._data is not None:
            print(self.ref_states._data[-1])
        else:
            print(None)
        return self.ref_states._data is not None
            
    
if __name__ == '__main__':
    rospy.init_node('mpc_node', anonymous=False)
    node = ModelPredictiveControlNode()
    rospy.spin()
