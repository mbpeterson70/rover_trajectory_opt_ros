#!/usr/bin/env python3
#
# This node generates and publishes a trajectory for rovers 
# to navigate to a desired final pose

# ros imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from rover_trajectory_msgs.msg import RoverState
from nav_msgs.msg import Odometry


# python imports
import numpy as np
from scipy.spatial.transform import Rotation as Rot

# trajectory optimization imports: https://github.com/mbpeterson70/tomma/
from tomma.dubins_dynamics import DubinsDynamics, CONTROL_LIN_ACC_ANG_VEL
from tomma.multi_agent_optimization import MultiAgentOptimization

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')
        # self.node_name = rospy.get_name()
        # Params
        # self.declare_parameter('rovers')
        # self.declare_parameter('dt')
        # self.declare_parameter('num_timesteps')
        # self.declare_parameter('min_allowable_dist')
        # self.declare_parameter('x_bounds')
        # self.declare_parameter('u_bounds')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rovers', None),
                ('dt', None),
                ('num_timesteps', None),
                ('min_allowable_dist', None),
                ('x_bounds', None),
                ('u_bounds', None),
                ('x_bounds.n', 0),
                ('u_bounds.n', 0),
            ]
        )
        self.rovers = self.get_parameter("rovers").value # rover names
        # self.rovers = self.get_parameter("rovers").value # rover names
        # self.get_logger().info(f'the contents of self.rovers: {self.rovers} ')
        {self.declare_parameter(f'goal_states.{rover}') for rover in self.rovers}
        
        self.dt = self.get_parameter("dt").value # how often to publish trajectory
        num_timesteps = self.get_parameter( # number of timesteps to use in trajectory optimization
            "num_timesteps").value 
        min_allowable_dist = self.get_parameter( # allowable distance between rovers
            "min_allowable_dist").value
        self.goal_states = {f'{rover}': # dictionary mapping rover names to goal states
            np.array([self.get_parameter(f"goal_states.{rover}").value]) for rover in self.rovers}
        self.xf_rover_states = dict()
        # self.get_logger().info(f'goal state items are self.goal_states.items(): {self.goal_states.items()}\n')
        # self.get_logger().info(f'goal_states.RedRover has contents: {self.get_parameter("goal_states.RedRover")}\n')
        
        for rover, xf in self.goal_states.items():
            # self.get_logger().info(f'goal state items are rover: {rover} and xf: {xf}')
            xf_rover_state = RoverState()
            xf_rover_state.x, xf_rover_state.y, xf_rover_state.v, xf_rover_state.theta = xf.reshape(-1).tolist()
            self.xf_rover_states[rover] = xf_rover_state
            
        
        # x_bounds = np.array(self.get_parameter(f"trajectory_generator/x_bounds").value) # state boundaries
        # u_bounds = np.array(self.get_parameter(f"trajectory_generator/u_bounds").value) # input boundaries
        # u_bounds = np.array([[-1.0, 1.0], [-1.5, 1.5]]) # placeholder
        x_bound_num = self.get_parameter('x_bounds.n').value
        {self.declare_parameter(f'x_bounds.xb{i}') for i in range(x_bound_num)}
        x_bounds = np.array([self.get_parameter(f'x_bounds.xb{i}').value for i in range(x_bound_num)])

        u_bound_num = self.get_parameter('u_bounds.n').value
        {self.declare_parameter(f'u_bounds.ub{i}') for i in range(u_bound_num)}
        u_bounds = np.array([self.get_parameter(f'u_bounds.ub{i}').value for i in range(u_bound_num)])



        # reformatting boundaries for input into tomma
        self.x_bounds = np.zeros(x_bounds.shape) 
        self.u_bounds = np.zeros(u_bounds.shape)
        # self.get_logger().info(f'\n\n\nthe contents of self.x_bounds: {self.x_bounds} \n\n\n')
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
        
        # might have to change the mocap/twist sub to a /odom sub
        # Pub & Sub
        # self.sub_pose = [ # pose subscriber for each rover, passes the rover identifier into callback
        #     self.create_subscription(PoseStamped, f"{rover}/world", lambda pose_stamped: self.pose_cb(pose_stamped, rover), 10)
        # for rover in self.rovers]
        # self.sub_twist = [ # twist subscriber for each rover
        #     self.create_subscription(TwistStamped, f"{rover}/mocap/twist",  lambda twist_stamped: self.twist_cb(twist_stamped, rover), 10) 
        # for rover in self.rovers]

        self.sub_twist = [self.create_subscription(Odometry, f"{rover}/odom", lambda odom_msg: self.odom_cb(odom_msg, rover), 10) for rover in self.rovers]
        self.pub_traj = { # dictionary of trajectory publishers for each rover
            f'{rover}': self.create_publisher(RoverState, f"{rover}/trajectory", 10) 
        for rover in self.rovers}
        self.pub_traj_pose = { # dictionary of trajectory pose publishers for each rover (for visualization)
            f'{rover}': self.create_publisher(PoseStamped, f"{rover}/trajectory_pose", 10) for rover in self.rovers
        }
        self.timer = self.create_timer(self.dt, self.loop_cb)
                
    def loop_cb(self):
        """
        Loops every self.dt seconds to publish trajectory to listening robots

        Args:
            None
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
        
    # def pose_cb(self, pose_stamped, rover):
    #     """
    #     Stores the most recent pose (with theta wrapping)

    #     Args:
    #         pose_stamped (PoseStamped): rover pose
    #         rover (any): rover ID
    #     """
    #     self.states[rover][0] = pose_stamped.pose.position.x
    #     self.states[rover][1] = pose_stamped.pose.position.y
    #     quat = pose_stamped.pose.orientation
    #     theta_unwrapped = Rot.from_quat([quat.x, quat.y, quat.w, quat.z]).as_euler('xyz')[2] + np.pi # add pi because of how theta is defined in Dubins dynamis
    #     self.states[rover][3] = -((theta_unwrapped + np.pi) % (2 * np.pi) - np.pi) # wrap
        
    # def twist_cb(self, twist_stamped, rover):
    #     """
    #     Stores the most recent linear/angular velcoties

    #     Args:
    #         twist_stamped (TwistStamped): rover twist
    #         rover (any): rover ID
    #     """
    #     theta = self.states[rover][3]
    #     if np.isnan(theta):
    #         return
    #     self.states[rover][2] = twist_stamped.twist.linear.x*np.cos(theta) + twist_stamped.twist.linear.y*np.sin(theta)
    #     # TODO: did I get that right?
        

    def odom_cb(self, odom_msg: Odometry, rover):
        self.states[rover][0] = odom_msg.pose.pose.position.x
        self.states[rover][1] = odom_msg.pose.pose.position.y
        quat = odom_msg.pose.pose.orientation
        theta_unwrapped = Rot.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz')[2]
        self.states[rover][3] = ((theta_unwrapped + np.pi) % (2 * np.pi) - np.pi) # wrap

        theta = self.states[rover][3]
        if np.isnan(theta):
            return
        
        self.states[rover][2] = odom_msg.twist.twist.linear.x*np.cos(theta) + odom_msg.twist.twist.linear.y*np.sin(theta)

        # should something be updating self.states[rover][4] here?

        return

    def all_states_received(self):
        """
        Check whether each rover's state has been recieved
        """
        for rover in self.rovers:
            if np.any(np.isnan(self.states[rover])):
                return False
        return True
            

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()