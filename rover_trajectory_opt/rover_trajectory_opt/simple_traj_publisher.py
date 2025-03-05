import rclpy
from rclpy.node import Node
from rover_trajectory_msgs.msg import RoverState
import numpy as np

# from std_msgs.msg import String


class SimpleTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(RoverState, '/RedRover/trajectory', 10)
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.t = 0.

    def timer_callback(self):
        # # msg = String()
        # # msg.data = 'Hello World: %d' % self.i
        # rover_state = RoverState()
        # rover_state.t = self.t
        # # rover_state.tf = 1.
        # rover_state.x = self.t/2
        # rover_state.y = 0.
        # rover_state.v = 0.5
        # rover_state.theta = 0.
        # self.publisher_.publish(rover_state)
        # # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1
        # self.t += self.timer_period
        
        
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        rover_state = RoverState()
        rover_state.t = self.t
        # rover_state.tf = 1.
        if self.t < 5:
            rover_state.x = self.t/2
            rover_state.y = 0.
            rover_state.v = 0.5
            rover_state.theta = 0.
            self.publisher_.publish(rover_state)
            # self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1
            self.t += self.timer_period
        else:
            ang = np.pi/20
            rover_state.x = 5/2+self.t/2*np.cos(ang)
            rover_state.y = self.t/2*np.sin(ang)
            rover_state.v = 0.5
            rover_state.theta = ang
            self.publisher_.publish(rover_state)
            # self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1
            self.t += self.timer_period

        # # msg = String()
        # # msg.data = 'Hello World: %d' % self.i
        # r = 1 # circle radius
        # rover_state = RoverState()
        # rover_state.t = self.t
        # # rover_state.tf = 1.
        # rover_state.v = 0.5
        # rover_state.x = r*np.cos(self.t*r*rover_state.v)-1
        # rover_state.y = -r*np.sin(self.t*r*rover_state.v)
        # rover_state.theta = self.t*r*rover_state.v
        # self.publisher_.publish(rover_state)
        # # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1
        # self.t += self.timer_period


def main(args=None):
    rclpy.init(args=args)

    simple_traj_publisher = SimpleTrajectoryPublisher()

    rclpy.spin(simple_traj_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    simple_traj_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()