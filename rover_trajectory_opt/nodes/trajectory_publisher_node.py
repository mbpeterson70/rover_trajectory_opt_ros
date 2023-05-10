#!/usr/bin/env python3

"""
This node publishes a trajectory received as a ROS message
or a static trajectory loaded into the node
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

import numpy as np

class TrajectoryPublisherNode():
    def __init__(self):
        self.node_name = rospy.get_name()
        
        # Params
        self.ugvs = rospy.get_param("~ugvs")
        self.x_bounds = [rospy.get_param("~xmin"),
                         rospy.get_param("~xmax")]
        self.y_bounds = [rospy.get_param("~ymin"),
                         rospy.get_param("~ymax")]
        self.ugv_radius = rospy.get_param("~ugv_radius")
        dt = rospy.get_param("~dt")
        
        # Internal Variables
        self.ugv_positions = {f'{ugv}': np.nan*np.ones((2,1)) for ugv in self.ugvs}
        
        # Pub & Sub
        self.sub_pose = [rospy.Subscriber(f"{ugv}/world", PoseStamped, self.pose_cb, queue_size=1, callback_args=({'ugv': ugv})) for ugv in self.ugvs]
        self.timer = rospy.Timer(rospy.Duration(dt), self.loop_cb)
        
        # Serv
        self.serv_disarm = {f'{ugv}': rospy.ServiceProxy(f'{ugv}/disarm', Trigger) for ugv in self.ugvs}
        
    def loop_cb(self, event):
        for ugv in self.ugvs:
            if self.ugv_positions[ugv].item(0) < self.x_bounds[0] or \
                self.ugv_positions[ugv].item(0) > self.x_bounds[1] or \
                self.ugv_positions[ugv].item(1) < self.y_bounds[0] or \
                self.ugv_positions[ugv].item(1) > self.y_bounds[1]:
                self.serv_disarm[ugv]()
        for ugv1 in self.ugvs:
            for ugv2 in self.ugvs:
                if ugv1 == ugv2: continue
                if np.linalg.norm(self.ugv_positions[ugv1] - self.ugv_positions[ugv2]) < self.ugv_radius:
                    self.serv_disarm[ugv1]()
                    self.serv_disarm[ugv2]()
        
    def pose_cb(self, pose_stamped, ugv):
        self.ugv_positions[ugv] = np.array([[pose_stamped.pose.position.x], 
                                                [pose_stamped.pose.position.y]])
    
if __name__ == '__main__':
    rospy.init_node('emergency_stop_node', anonymous=False)
    node = EmergencyStopNode()
    rospy.spin()
