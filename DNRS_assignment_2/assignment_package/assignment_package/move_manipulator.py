import rclpy
import rclpy.time
import rclpy.publisher
import numpy as np
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Header


class MoveManipulator(Node):
    def __init__(self, timer_period: float, omega: float):
        super().__init__("move_manipulator")
        self.start = self.get_clock().now()
        self.omega = omega
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.robot_state = JointState()
        self.robot_state.header = Header()
        self.robot_state.header.stamp = self.start.to_msg()
        self.robot_state.name = [
            "body_1-base-joint",
            "body_2-body_1-joint",
            "body_3-body_2-joint",
            "body_4-body_3-joint",
            "body_5-body_4-joint",
            "gripper-body_5-joint",
        ]
        self.robot_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pub.publish(self.robot_state)

    def timer_callback(self):
        now = self.get_clock().now()
        self.get_logger().info(
            ", ".join(
                f"q_{i + 1} = {v:.4f}" for i, v in enumerate(self.robot_state.position)
            )
        )
        self.robot_state.position = self.get_joint_state(now)
        self.robot_state.header.stamp = now.to_msg()
        self.pub.publish(self.robot_state)

    def get_joint_state(self, now: rclpy.time.Time) -> list[float]:
        t = (now.nanoseconds - self.start.nanoseconds) / 1e9
        return (np.pi * np.sin(self.omega * t * np.ones(6))).tolist()


def main(args=None):
    rclpy.init(args=args)
    node = MoveManipulator(0.1, 0.1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
