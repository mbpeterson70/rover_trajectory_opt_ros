#!/usr/bin/env python3
#
# This node generates and publishes a trajectory for rovers 
# to navigate to a desired final pose

# ros imports
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_srvs.srv import Trigger
from rover_trajectory_msgs.msg import RoverState

# python imports
import numpy as np
from scipy.spatial.transform import Rotation as Rot

# trajectory optimization imports: https://github.com/mbpeterson70/tomma/
from tomma.dubins_dynamics import DubinsDynamics, CONTROL_LIN_ACC_ANG_VEL
from tomma.multi_agent_optimization import MultiAgentOptimization

class TrajectoryGeneratorNode():
    def __init__(self):
        self.node_name = rospy.get_name()
        
        # Params
        self.rovers = rospy.get_param("~trajectory_generator/rovers") # rover names
        self.dt = rospy.get_param("~trajectory_generator/dt") # how often to publish trajectory
        num_timesteps = rospy.get_param( # number of timesteps to use in trajectory optimization
            "~trajectory_generator/num_timesteps") 
        min_allowable_dist = rospy.get_param( # allowable distance between rovers
            "~trajectory_generator/min_allowable_dist")
        self.goal_states = {f'{rover}': # dictionary mapping rover names to goal states
            np.array([rospy.get_param(f"~trajectory_generator/goal_states/{rover}")]) for rover in self.rovers}
        self.xf_rover_states = dict()
        for rover, xf in self.goal_states.items():
            xf_rover_state = RoverState()
            xf_rover_state.x, xf_rover_state.y, xf_rover_state.v, xf_rover_state.theta = xf.reshape(-1).tolist()
            self.xf_rover_states[rover] = xf_rover_state
        x_bounds = np.array(rospy.get_param(f"~trajectory_generator/x_bounds")) # state boundaries
        u_bounds = np.array(rospy.get_param(f"~trajectory_generator/u_bounds")) # input boundaries
        # reformatting boundaries for input into tomma
        self.x_bounds = np.zeros(x_bounds.shape) 
        self.u_bounds = np.zeros(u_bounds.shape)
        for i in range(self.x_bounds.shape[0]):
            for j in range(self.x_bounds.shape[1]):
                self.x_bounds[i,j] = float(x_bounds[i,j])
        for i in range(self.u_bounds.shape[0]):
            for j in range(self.u_bounds.shape[1]):
                self.u_bounds[i,j] = float(u_bounds[i,j])
        
        # Internal variables
        self.states = {f'{rover}': np.nan*np.ones(4) for rover in self.rovers}
        self.planner = MultiAgentOptimization(dynamics=DubinsDynamics(control=CONTROL_LIN_ACC_ANG_VEL), 
                                         num_agents=len(self.rovers), 
                                         num_timesteps=num_timesteps,
                                         min_allowable_dist=min_allowable_dist)
        self.traj_planned = False
        self.t = 0.0
        self.rover_idx = {f'{rover}': i for i, rover in enumerate(self.rovers)}
        
        # Pub & Sub
        self.sub_pose = [ # pose subscriber for each rover, passes the rover identifier into callback
            rospy.Subscriber(f"{rover}/world", PoseStamped, self.pose_cb, queue_size=1, callback_args=rover)
        for rover in self.rovers]
        self.sub_twist = [ # twist subscriber for each rover
            rospy.Subscriber(f"{rover}/mocap/twist", TwistStamped, self.twist_cb, queue_size=1, callback_args=rover) 
        for rover in self.rovers]
        self.pub_traj = { # dictionary of trajectory publishers for each rover
            f'{rover}': rospy.Publisher(f"{rover}/trajectory", RoverState, queue_size=1) 
        for rover in self.rovers}
        self.pub_traj_pose = { # dictionary of trajectory pose publishers for each rover (for visualization)
            f'{rover}': rospy.Publisher(f"{rover}/trajectory_pose", PoseStamped, queue_size=1) for rover in self.rovers
        }
        self.timer = rospy.Timer(rospy.Duration(self.dt), self.loop_cb)
                
    def loop_cb(self, event):
        """
        Loops every self.dt seconds to publish trajectory to listening robots

        Args:
            event (rospy.TimerEvent): not used
        """
        # if trajectory has been planned, publish trajectory
        if self.traj_planned:
            for rover in self.rovers: # loop through each rover
                if self.t > self.tf: # continue to publish final state if trajectory time has finished
                    self.pub_traj[rover].publish(self.xf_rover_states[rover])
                    continue
                
                # setup and publish RoverState msg
                rover_state = RoverState()
                rover_state.t = self.t
                rover_state.tf = self.tf
                rover_state.x = np.interp(self.t, xp=self.t_traj, fp=self.x_traj[rover][0,:])
                rover_state.y = np.interp(self.t, xp=self.t_traj, fp=self.x_traj[rover][1,:])
                rover_state.v = np.interp(self.t, xp=self.t_traj, fp=self.x_traj[rover][2,:])
                rover_state.theta = np.interp(self.t, xp=self.t_traj, fp=self.x_traj[rover][3,:])
                self.pub_traj[rover].publish(rover_state)

                # setup and publish rover trajectory pose for visualization (rviz)
                pose = PoseStamped()
                pose.pose.position.x = rover_state.x
                pose.pose.position.y = rover_state.y
                quat = Rot.from_euler('xyz', [0, 0, rover_state.theta]).as_quat()
                pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = quat.tolist()
                pose.header.frame_id = "world"
                self.pub_traj_pose[rover].publish(pose)

            self.t += self.dt
            return
        else:
            # if all states have been received, plan trajectory, otherwise, continue waiting
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
                x, u, self.t_traj = self.planner.solve_opt()
                print('Executing trajectory...')               
                self.x_traj = {f'{rover}': x[i] for i, rover in enumerate(self.rovers)}
                self.u_traj = {f'{rover}': u[i] for i, rover in enumerate(self.rovers)}
                self.tf = self.t_traj[-1]
                self.traj_planned = True
            else:
                return # wait for all starting positions to be known
        
    def pose_cb(self, pose_stamped, rover):
        """
        Stores the most recent pose (with theta wrapping)

        Args:
            pose_stamped (PoseStamped): rover pose
            rover (any): rover ID
        """
        self.states[rover][0] = pose_stamped.pose.position.x
        self.states[rover][1] = pose_stamped.pose.position.y
        quat = pose_stamped.pose.orientation
        theta_unwrapped = Rot.from_quat([quat.x, quat.y, quat.w, quat.z]).as_euler('xyz')[2] + np.pi # add pi because of how theta is defined in Dubins dynamis
        self.states[rover][3] = -((theta_unwrapped + np.pi) % (2 * np.pi) - np.pi) # wrap
        
    def twist_cb(self, twist_stamped, rover):
        """
        Stores the most recent linear/angular velcoties

        Args:
            twist_stamped (TwistStamped): rover twist
            rover (any): rover ID
        """
        theta = self.states[rover][3]
        if np.isnan(theta):
            return
        self.states[rover][2] = twist_stamped.twist.linear.x*np.cos(theta) + twist_stamped.twist.linear.y*np.sin(theta)
        # TODO: did I get that right?
        
    def all_states_received(self):
        """
        Check whether each rover's state has been recieved
        """
        for rover in self.rovers:
            if np.any(np.isnan(self.states[rover])):
                return False
        return True
            
    
if __name__ == '__main__':
    rospy.init_node('trajectory_generator_node', anonymous=False)
    node = TrajectoryGeneratorNode()
    rospy.spin()
