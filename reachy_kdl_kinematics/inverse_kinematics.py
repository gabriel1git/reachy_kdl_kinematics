import rclpy
import numpy as np
from spatialmath import *
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/r_arm/target_pose', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period, self.timer_callback2)
        self.i, self.indice = 0,0

        #read joint data
        self.subcriber = self.create_subscription(
            Float64MultiArray,
            'r_arm_forward_position_controller/commands',
            self.listener_data,
            10)
        self.subcriber  # prevent unused variable warning

        self.j1 = 0.
        self.j2 = 0.
        self.j3 = 0.
        self.j4 = -np.pi/2
        self.j5 = 0.
        self.j6 = 0.
        self.j7 = 0.

        A = np.array([
        [0, 0, -1, 0.3],
        [0, 1, 0, -0.4],  
        [1, 0, 0, -0.3],
        [0, 0, 0, 1],  
        ])

        B = np.array([
        [0, 0, -1, 0.3],
        [0, 1, 0, -0.4],  
        [1, 0, 0, 0.0],
        [0, 0, 0, 1],  
        ])

        C = np.array([
        [0, 0, -1, 0.3],
        [0, 1, 0, -0.1],  
        [1, 0, 0, 0.0],
        [0, 0, 0, 1],
        ])

        D = np.array([
        [0, 0, -1, 0.3],
        [0, 1, 0, -0.1],  
        [1, 0, 0, -0.3],
        [0, 0, 0, 1],  
        ])

        xA, yA, zA = A[:3, 3]
        xB, yB, zB = B[:3, 3]
        xC, yC, zC = C[:3, 3]
        xD, yD, zD = D[:3, 3]

        self.x = [xA, xB, xC, xD]
        self.y = [yA, yB, yC, yD]
        self.z = [zA, zB, zC, zD]

        qA = Rotation.from_matrix(A[:3, :3]).as_quat()
        qB = Rotation.from_matrix(B[:3, :3]).as_quat()
        qC = Rotation.from_matrix(C[:3, :3]).as_quat()
        qD = Rotation.from_matrix(D[:3, :3]).as_quat()

        self.q = [qA, qB, qC, qD]

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.x[self.indice]
        msg.pose.position.y = self.y[self.indice]
        msg.pose.position.z = self.z[self.indice]

        msg.pose.orientation.x = self.q[0][0]
        msg.pose.orientation.y = self.q[0][1]
        msg.pose.orientation.z = self.q[0][2]
        msg.pose.orientation.w = self.q[0][3]

        if self.i == 50:
            self.indice = 1
        elif self.i == 100:
            self.indice = 2
        elif self.i == 150:
            self.indice = 3
        elif self.i == 200:
            self.indice = 0
            self.i = 0
        
        self.i += 1

        self.publisher_.publish(msg)

    def listener_data(self,msg):
        self.j1 = msg.data[0]
        self.j2 = msg.data[1]
        self.j3 = msg.data[2]
        self.j4 = msg.data[3]
        self.j5 = msg.data[4]
        self.j6 = msg.data[5]
        self.j7 = msg.data[6]

    def timer_callback2(self):
        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw','r_elbow_pitch','r_forearm_yaw','r_wrist_pitch','r_wrist_roll','r_gripper']
        joint_state.position = [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6, self.j7, 0.]
        self.get_logger().info('Publishing: "%s"' % joint_state.position)
        self.joint_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()