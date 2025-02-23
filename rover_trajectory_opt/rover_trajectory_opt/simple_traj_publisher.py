import rclpy
from rclpy.node import Node
from rover_trajectory_msgs.msg import RoverState

# from std_msgs.msg import String


class SimpleTrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(RoverState, '/trajectory', 10)
        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0
        self.t = 0.

    def timer_callback(self):
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        rover_state = RoverState()
        rover_state.t = self.t
        # rover_state.tf = 1.
        rover_state.x = self.t/2
        rover_state.y = 0.
        rover_state.v = 0.5
        rover_state.theta = 0.
        self.publisher_.publish(rover_state)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        self.t += self.timer_period

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