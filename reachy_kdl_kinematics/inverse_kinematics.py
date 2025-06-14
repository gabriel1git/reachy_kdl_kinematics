import rclpy
import numpy as np
from spatialmath import *
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
#from roboticstoolbox import quintic
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/r_arm/target_pose', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = .2
        timer_period2 = .1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.timer2 = self.create_timer(timer_period2, self.timer_callback2)
        self.i, self.indice = 0,0

        self.sub_move = self.create_subscription(
            String, 
            'move',
            self.listener_move,
            10)

        #read joint data
        self.subcriber = self.create_subscription(
            Float64MultiArray,
            'r_arm_forward_position_controller/commands',
            self.listener_data,
            10)
        self.subcriber  # prevent unused variable warning

        self.j1 = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        self.j2 = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        self.j3 = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        self.j4 = [-np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2, -np.pi/2]
        self.j5 = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        self.j6 = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]
        self.j7 = [0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]

        self.q1 = 0.
        self.q2 = 0.
        self.q3 = 0.
        self.q4 = 0.
        self.q5 = 0.
        self.q6 = 0.
        self.q7 = 0.

        self.i = 0
        self.ii = 0
        self.qant = [0., 0., 0., -np.pi/2, 0., 0., 0.]
        
        self.T = [0.,0.,0.,0.,0.,0.,0.]
        self.joint_state = JointState()

        self.A = np.array([
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

        xA, yA, zA = self.A[:3, 3]
        #xB, yB, zB = B[:3, 3]
        #xC, yC, zC = C[:3, 3]
        #xD, yD, zD = D[:3, 3]

        self.x = [xA]#, xB, xC, xD]
        self.y = [yA]#, yB, yC, yD]
        self.z = [zA]#, zB, zC, zD]

        qA = Rotation.from_matrix(self.A[:3, :3]).as_quat()
        #qB = Rotation.from_matrix(B[:3, :3]).as_quat()
        #qC = Rotation.from_matrix(C[:3, :3]).as_quat()
        #qD = Rotation.from_matrix(D[:3, :3]).as_quat()

        self.q = [qA]
    def listener_move(self, msg):
        self.A = np.array(eval(msg.data))
        xA, yA, zA = self.A[:3, 3]
        self.x = [xA]#, xB, xC, xD]
        self.y = [yA]#, yB, yC, yD]
        self.z = [zA]#, zB, zC, zD]
        qA = Rotation.from_matrix(self.A[:3, :3]).as_quat()
        self.q = [qA]

    def quintic_trajectory_array(self, q0, qf, N=10):
        """
        Gera N pontos de uma interpolação quintic de q0 até qf.
        """
        t_vals = np.linspace(0, 1, N)
        return [q0 + (qf - q0)*(10*t**3 - 15*t**4 + 6*t**5) for t in t_vals]

    def timer_data(self):
        q = [self.q1, self.q2, self.q3, self.q4, self.q5, self.q6, self.q7]
        #aux = [0.,0.,0.,-np.pi/2.,0.,0.,0.]
        #q = [aux[0], aux[1], aux[2], aux[3], aux[4], aux[5], aux[6]]#self.ik()
        
        self.j1 = self.quintic_trajectory_array(self.qant[0], q[0], 10)
        self.j2 = self.quintic_trajectory_array(self.qant[1], q[1], 10)
        self.j3 = self.quintic_trajectory_array(self.qant[2], q[2], 10)
        self.j4 = self.quintic_trajectory_array(self.qant[3], q[3], 10)
        self.j5 = self.quintic_trajectory_array(self.qant[4], q[4], 10)
        self.j6 = self.quintic_trajectory_array(self.qant[5], q[5], 10)
        self.j7 = self.quintic_trajectory_array(self.qant[6], q[6], 10)
        self.qant = q

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

        self.publisher_.publish(msg)

    def listener_data(self,msg):
        self.q1 = msg.data[0]
        self.q2 = msg.data[1]
        self.q3 = msg.data[2]
        self.q4 = msg.data[3]
        self.q5 = msg.data[4]
        self.q6 = msg.data[5]
        self.q7 = msg.data[6]

    def timer_callback2(self):
        joint_state = JointState()
        now = self.get_clock().now()
        joint_state.header.stamp = now.to_msg()
        joint_state.name = ['r_shoulder_pitch', 'r_shoulder_roll', 'r_arm_yaw','r_elbow_pitch','r_forearm_yaw','r_wrist_pitch','r_wrist_roll','r_gripper']
        joint_state.position = [self.j1[self.ii], self.j2[self.ii], self.j3[self.ii], self.j4[self.ii], self.j5[self.ii], self.j6[self.ii], self.j7[self.ii], 0.]
        self.joint_pub.publish(joint_state)

        self.ii = self.ii + 1
        if self.ii == 10:
            self.timer_data()
            self.ii = 0


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()