#!/usr/bin/env python3

"""
This node generates and publishes a trajectory for rovers 
to navigate to a desired final pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Odometry
from rover_trajectory_msgs.msg import RoverState

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from tomma.dubins_dynamics import DubinsDynamics, CONTROL_LIN_VEL_ANG_VEL
from tomma.multi_agent_optimization import MultiAgentOptimization

from robotdatapy.data import ArrayData
from robotdatapy.exceptions import NoDataNearTimeException


class ModelPredictiveControlNode(Node):
    def __init__(self):
        super().__init__('mpc_node')

        # Params
        self.declare_parameter("mpc/dt", 0.2)
        self.declare_parameter("mpc/num_timesteps", 5)
        self.declare_parameter("mpc/tf", 1.0)
        # TODO: make param
        # self.declare_parameter("mpc/x_bounds", [-10.0, 10, -10.0, 10.0, -float('inf'), float('inf')])
        # self.declare_parameter("mpc/u_bounds", [-1.0, 1.0, -1.5, 1.5])

        self.dt = self.get_parameter("mpc/dt").value
        self.mpc_num_timesteps = self.get_parameter("mpc/num_timesteps").value
        self.mpc_tf = self.get_parameter("mpc/tf").value

        # x_bounds = np.array(self.get_parameter("mpc/x_bounds").value).reshape((3,2))
        # u_bounds = np.array(self.get_parameter("mpc/u_bounds").value).reshape((2,2))
        x_bounds = np.array([[-np.inf, np.inf], [-np.inf, np.inf], [-np.inf, np.inf]])
        u_bounds = np.array([[-1.0, 1.0], [-1.5, 1.5]])
        self.x_bounds = np.zeros(x_bounds.shape)
        self.u_bounds = np.zeros(u_bounds.shape)
        self.u_diff_bounds = np.array([3.0, 3.0])
        self.R = np.diag([0.05, 0.05])

        for i in range(self.x_bounds.shape[0]):
            for j in range(self.x_bounds.shape[1]):
                self.x_bounds[i, j] = float(x_bounds[i, j])
        for i in range(self.u_bounds.shape[0]):
            for j in range(self.u_bounds.shape[1]):
                self.u_bounds[i, j] = float(u_bounds[i, j])

        # Internal variables
        self.state = np.nan * np.ones(5)
        self.ref_states = ArrayData(time_array=None, data_array=None, interp=True, time_tol=1.0)
        self.planner = MultiAgentOptimization(
            dynamics=DubinsDynamics(control=CONTROL_LIN_VEL_ANG_VEL),
            num_agents=1,
            num_timesteps=self.mpc_num_timesteps,
        )

        # Pub & Sub
        # self.sub_pose = self.create_subscription(PoseStamped, "world", self.pose_cb, 10)
        # self.sub_twist = self.create_subscription(TwistStamped, "mocap/twist", self.twist_cb, 10)
        # self.sub_twist = self.create_subscription(TwistStamped, "odom", self.odom_cb, 10) # previous ros2
        self.sub_twist = self.create_subscription(Odometry, "odom", self.odom_cb, 10) # new ros2
        self.sub_ref = self.create_subscription(RoverState, "trajectory", self.ref_cb, 10)
        self.pub_auto_cmd = self.create_publisher(Twist, "cmd_vel_auto", 10)

        # Timer
        self.timer = self.create_timer(self.dt, self.loop_cb)
        self.last_ref_time = None
        self.last_pose_time = None
        self.no_msg_time = 0.5
                
    def loop_cb(self):
        # self.get_logger().info(f'self state is: {self.state}')
        # self.get_logger().info(f'beforeif statement, inside loop_cb222; states_recieved: {self.states_received()}; ref_recieved: {self.ref_received()}')
        if self.states_received() and self.ref_received():
            # for the conversion to seconds, it is noted in the rclcpp docs that significant precision
            # loss is possible depending on sizeof(double), so convert nanoseconds directly instead
            if (self.get_clock().now() - self.last_pose_time).nanoseconds / 1e9 > self.no_msg_time or \
                (self.get_clock().now() - self.last_ref_time).nanoseconds / 1e9 > self.no_msg_time:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.
                cmd_vel.angular.z = 0.
                self.pub_auto_cmd.publish(cmd_vel)
                # self.get_logger().info('publishing from here111')
                return
            # ref_state is in the form [x, y, v, theta]
            # x: x, y, theta
            x0 = np.concatenate([self.state[0:2], [self.state[3]]]).reshape((1,3))
            xf = np.concatenate([self.ref_states._data[-1][0:2], [self.ref_states._data[-1][3]]]).reshape((1,3))
            
            waypoints = {}
            Q_waypoints = {}

            # TODO: deleting the following if statement w/o testing...
            # seems like it was supposed to be for 
            # making sure the velocity was high enough, but xf.item(1) is y?
            # if xf.item(1) > .1:
            for i in range(self.mpc_num_timesteps):
                t = self.ref_states.times[-1] - self.mpc_tf + i*self.mpc_num_timesteps/self.mpc_tf
                try:
                    wp = self.ref_states.data(t)
                except NoDataNearTimeException as ex:
                    continue
                waypoints[i] = np.concatenate([wp[0:2], [wp[3]]]).reshape((1,3))
                Q_waypoints[i] = np.eye(3)*i
            Qf = np.eye(3)*self.mpc_num_timesteps
            
            self.planner.setup_mpc_opt(
                x0, 
                xf, 
                R=self.R, 
                tf=self.mpc_tf, 
                waypoints=waypoints, 
                Q_waypoints=Q_waypoints, 
                Qf=Qf, 
                x_bounds=self.x_bounds, 
                u_bounds=self.u_bounds
            )
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
            except Exception:
                self.get_logger().error("Planner failed")
        else:
            # self.get_logger().info('inside else statement333')
            self.pub_auto_cmd.publish(Twist())
        
    # def pose_cb(self, pose_stamped):
    #     self.last_pose_time = self.get_clock().now()
    #     self.state[0] = pose_stamped.pose.position.x
    #     self.state[1] = pose_stamped.pose.position.y
    #     quat = pose_stamped.pose.orientation
    #     theta_unwrapped = Rot.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]
    #     self.state[3] = ((theta_unwrapped + np.pi) % (2 * np.pi) - np.pi) # wrap
        
    # def twist_cb(self, twist_stamped):
    #     theta = self.state[3]
    #     if np.isnan(theta):
    #         return
    #     self.state[2] = twist_stamped.twist.linear.x*np.cos(theta) + twist_stamped.twist.linear.y*np.sin(theta)
    #     self.state[4] = twist_stamped.twist.angular.z
    #     # TODO: did I get that right?

    def odom_cb(self, odom_msg: Odometry):
        # self.get_logger().info('inside callback for odom444')
        quat = odom_msg.pose.pose.orientation
        theta_unwrapped = Rot.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]
        self.state[3] = ((theta_unwrapped + np.pi) % (2 * np.pi) - np.pi) # wrap
        theta = self.state[3]
        
        # if np.isnan(theta):
        #     # self.get_logger().info('theta is nan, returning555')
        #     return
        
        self.last_pose_time = self.get_clock().now()
        self.state[0] = odom_msg.pose.pose.position.x
        self.state[1] = odom_msg.pose.pose.position.y
        
        self.state[2] = odom_msg.twist.twist.linear.x*np.cos(theta) + odom_msg.twist.twist.linear.y*np.sin(theta)
        self.state[4] = odom_msg.twist.twist.angular.z
        print(np.round(self.state, 2))
        # TODO: did I get that right?

    def ref_cb(self, rover_state):
        # self.get_logger().info('inside ref_cb callback66A')
        self.last_ref_time = self.get_clock().now()
        theta_ref = rover_state.theta
        # TODO: don't use euler angles in trajectory optimization
        while self.state[3] - theta_ref > np.pi:
            theta_ref += 2*np.pi
        while theta_ref - self.state[3] > np.pi:
            theta_ref -= 2*np.pi
        ref_state = np.array([rover_state.x, rover_state.y, rover_state.v, theta_ref])
        # ref_state is in the form [x, y, v, theta]
        if self.ref_states.times is None:
            self.ref_states.times = np.array([rover_state.t])
            self.ref_states._data = ref_state.reshape((1,4))
        else:
            self.ref_states.times = np.append(self.ref_states.times, rover_state.t)
            self.ref_states._data = np.append(self.ref_states._data, ref_state.reshape((1,4)), axis=0)
        
    def states_received(self):
        # print(np.round(self.state,2))
        return not np.any(np.isnan(self.state))
        
    def ref_received(self):
        if self.ref_states._data is not None:
            print(self.ref_states._data[-1])
        else:
            print(None)
        return self.ref_states._data is not None
            
    
def main(args=None):
    rclpy.init(args=args)
    node = ModelPredictiveControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()