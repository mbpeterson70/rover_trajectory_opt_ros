#!/usr/bin/env python3

"""
This node generates and publishes a trajectory for rovers 
to navigate to a desired final pose
"""

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_srvs.srv import Trigger
from rover_trajectory_msgs.msg import RoverState

import json
import numpy as np
from scipy.spatial.transform import Rotation as Rot

from robot_utils.robot_data.array_data import ArrayData

class TrajectoryPublisherNode():
    def __init__(self):
        self.node_name = rospy.get_name()
        
        # Params
        self.rovers = rospy.get_param("~trajectory_publisher/rovers")
        self.dt = rospy.get_param("~trajectory_publisher/dt")
        self.trajectory_file = rospy.get_param("~trajectory_publisher/trajectory_file")
        self.is_cyclic = rospy.get_param("~trajectory_publisher/is_cyclic")
        
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
                    ArrayData(time_array=time_array[i,:], data_array=traj_array[i,j,:,:], interp=True)
                )
        self.tfs = time_array[:,-1]
        
        # Pub & Sub
        self.sub_pose = [rospy.Subscriber(f"{rover}/world", PoseStamped, self.pose_cb, queue_size=1, callback_args=rover) for rover in self.rovers]
        self.sub_twist = [rospy.Subscriber(f"{rover}/mocap/twist", TwistStamped, self.twist_cb, queue_size=1, callback_args=rover) for rover in self.rovers]
        self.pub_traj = {f'{rover}': rospy.Publisher(f"{rover}/trajectory", RoverState, queue_size=1) for rover in self.rovers}
        self.pub_traj_pose = {f'{rover}': rospy.Publisher(f"{rover}/trajectory_pose", PoseStamped, queue_size=1) for rover in self.rovers}
        # self.pub_auto_cmd = {f'{rover}': rospy.Publisher(f"{rover}/cmd_vel_auto", Twist, queue_size=1) for rover in self.rovers}
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop_cb)
                
    def loop_cb(self, event):
        # check if trajectory is finished
        if self.t > self.tfs[self.seg_idx]:
            if self.seg_idx + 1 < self.num_segs:
                self.seg_idx += 1
                self.t = 0
            elif self.is_cyclic:
                self.seg_idx = 0
                self.t = 0
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
        return
        
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
            
    
if __name__ == '__main__':
    rospy.init_node('trajectory_publisher_node', anonymous=False)
    node = TrajectoryPublisherNode()
    rospy.spin()
