import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.send_pose)



    def send_pose(self):
        msg = JointState()                                     # 🏷️ msg
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'left_finger_joint', 'right_finger_joint']
        msg.position = [0.0, 0.5, 1.0, 0.0, 0.0]               # 🏷️ your pose values
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = PosePublisher()                                     # 🏷️ node
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()