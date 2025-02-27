#!/usr/bin/env python3

"""
This node generates and publishes a trajectory for rovers 
to navigate to a desired final pose
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_srvs.srv import Trigger
from rover_trajectory_msgs.msg import RoverState
from nav_msgs.msg import Odometry


import json
import numpy as np
from scipy.spatial.transform import Rotation as Rot

# from robot_utils.robot_data.array_data import ArrayData # this seems to be a depracated reference to the robotdatapy library
from robotdatapy.data import ArrayData


class TrajectoryPublisherNode(Node):
    def __init__(self):
        super().__init__('trajectory_publisher_node')

        # Params
        # self.declare_parameter("trajectory_publisher/rovers")
        # self.declare_parameter("trajectory_publisher/dt")
        # self.declare_parameter("trajectory_publisher/trajectory_file")
        # self.declare_parameter("trajectory_publisher/is_cyclic")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rovers', None),
                ('dt', None),
                ('trajectory_file', None),
                ('is_cyclic', False), # TODO: what should the default value of this be?
            ]
        )
        
        # self.rovers = self.get_parameter("trajectory_publisher/rovers")
        self.rovers = self.get_parameter("rovers").value # rover names

        # self.dt = self.get_parameter("trajectory_publisher/dt")
        self.dt = self.get_parameter("dt").value # how often to publish trajectory

        # self.trajectory_file = self.get_parameter("trajectory_publisher/trajectory_file")
        self.trajectory_file = self.get_parameter("trajectory_file").value

        # self.is_cyclic = self.get_parameter("trajectory_publisher/is_cyclic")
        self.is_cyclic = self.get_parameter("is_cyclic").value 

        
        # Internal variables
        self.num_rovers = len(self.rovers)
        self.states = {f'{rover}': np.nan*np.ones(4) for rover in self.rovers}
        # self.traj_planned = False
        self.t = 0.0
        self.seg_idx = 0
        self.rover_idx = {f'{rover}': i for i, rover in enumerate(self.rovers)}
        
        # load in trajectory data
        f = open(self.trajectory_file)
        data = json.load(f)
        f.close()
        traj_array = np.array(data['trajectory'])
        time_array = np.array(data['time'])
        assert time_array.shape[0] == traj_array.shape[0]
        assert self.num_rovers == traj_array.shape[1]
        self.num_segs = time_array.shape[0]
        
        self.trajectories = []
        for i in range(self.num_segs):
            self.trajectories.append([])
            for j in range(self.num_rovers):
                self.trajectories[i].append(
                    # ArrayData(time_array=time_array[i,:], data_array=traj_array[i,j,:,:], interp=True)
                    ArrayData(time_array=time_array[i,:], data_array=traj_array[i,j,:,:], interp=True, time_tol=0.2) # manually set time tolerance
                )
        self.tfs = time_array[:,-1]
        
        # might have to replace pose and twist sub with odom sub
        # Pub & Sub
        # self.sub_pose = [self.create_subscription(f"{rover}/world", PoseStamped, self.pose_cb, queue_size=1, callback_args=rover) for rover in self.rovers]
        # self.sub_twist = [self.create_subscription(f"{rover}/mocap/twist", TwistStamped, self.twist_cb, queue_size=1, callback_args=rover) for rover in self.rovers]
        self.sub_twist = [self.create_subscription(Odometry, f"{rover}/odom", lambda odom_msg: self.odom_cb(odom_msg, rover), 10) for rover in self.rovers]
        self.pub_traj = {f'{rover}': self.create_publisher(RoverState, f"{rover}/trajectory", 10) for rover in self.rovers}
        self.pub_traj_pose = {f'{rover}': self.create_publisher(PoseStamped, f"{rover}/trajectory_pose", 10) for rover in self.rovers}
        # self.pub_auto_cmd = {f'{rover}': self.create_publisher(f"{rover}/cmd_vel_auto", Twist, queue_size=1) for rover in self.rovers}
        self.timer = self.create_timer(self.dt, self.loop_cb)
                
    def loop_cb(self):
        # check if trajectory is finished
        if self.t > self.tfs[self.seg_idx]:
            if self.seg_idx + 1 < self.num_segs:
                self.seg_idx += 1
                self.t = 0.0
            elif self.is_cyclic:
                self.seg_idx = 0
                self.t = 0.0
            else:
                assert False, "This option is currently not supported."
        
        for rover in self.rovers:
            rover_state = RoverState()
            rover_state.t = self.t
            rover_state.tf = self.tfs[self.seg_idx]
            traj = self.trajectories[self.seg_idx][self.rover_idx[rover]].data(self.t)
            rover_state.x = traj[0]
            rover_state.y = traj[1]
            rover_state.v = traj[2]
            rover_state.theta = traj[3]
            self.pub_traj[rover].publish(rover_state)

            pose = PoseStamped()
            pose.pose.position.x = rover_state.x
            pose.pose.position.y = rover_state.y
            quat = Rot.from_euler('xyz', [0, 0, rover_state.theta]).as_quat()
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat.tolist()
            pose.header.frame_id = "world"
            self.pub_traj_pose[rover].publish(pose)

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
        if self.t - int(self.t) <= 0.02:
            self.get_logger().info(f'time: {self.t}, rover state: {rover_state.x, rover_state.y, rover_state.v}')
        return
        
    def odom_cb(self, odom_msg: Odometry, rover):
        self.states[rover][0] = odom_msg.pose.pose.position.x
        self.states[rover][1] = odom_msg.pose.pose.position.y
        quat = odom_msg.pose.pose.orientation
        theta_unwrapped = Rot.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]
        #should there be/not be a negative sign here (below)? there is no neg in mpc.py but there was in the ros1 implementation of this file
        self.states[rover][3] = ((theta_unwrapped + np.pi) % (2 * np.pi) - np.pi) # wrap 

        theta = self.states[rover][3]
        if np.isnan(theta):
            return
        
        self.states[rover][2] = odom_msg.twist.twist.linear.x*np.cos(theta) + odom_msg.twist.twist.linear.y*np.sin(theta)

        # should something be updating self.states[rover][4] here?

        return        
    # def pose_cb(self, pose_stamped, rover):
    #     self.states[rover][0] = pose_stamped.pose.position.x
    #     self.states[rover][1] = pose_stamped.pose.position.y
    #     quat = pose_stamped.pose.orientation
    #     theta_unwrapped = Rot.from_quat([quat.x, quat.y, quat.w, quat.z]).as_euler('xyz')[2] + np.pi # add pi because of how theta is defined in Dubins dynamis
    #     self.states[rover][3] = -((theta_unwrapped + np.pi) % (2 * np.pi) - np.pi) # wrap
        
    # def twist_cb(self, twist_stamped, rover):
    #     theta = self.states[rover][3]
    #     if np.isnan(theta):
    #         return
    #     self.states[rover][2] = twist_stamped.twist.linear.x*np.cos(theta) + twist_stamped.twist.linear.y*np.sin(theta)
    #     # TODO: did I get that right?
            

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
