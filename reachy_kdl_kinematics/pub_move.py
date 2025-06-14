import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('movement_publisher')
        self.publisher_ = self.create_publisher(String, 'move', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.origem_x = -0.2
        self.origem_y = 0.
        self.raio = 0.2

    def timer_callback(self):
        A = np.array([
        [0, 0, -1, 0.3],
        [0, 1, 0, self.origem_x+self.raio*np.cos(self.i)],  
        [1, 0, 0, self.origem_y+self.raio*np.sin(self.i)],
        [0, 0, 0, 1],  
        ])


        msg = String()
        msg.data = msg.data = np.array2string(A, separator=', ')
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 0.01


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()