import rclpy
from rclpy.node import Node
from rokey_interfaces.msg import TaskState
import time

class TestFeedbackNode(Node):
    def __init__(self):
        super().__init__('test_feedback_node')
        self.publisher = self.create_publisher(TaskState, '/task_state', 10)
        self.timer = self.create_timer(2.0, self.send_next_state)
        self.state_index = 0

        # 시퀀스 정의
        self.state_sequence = [
            ("start", ""),
            ("searching", ""),
            ("grasped", "tumblr"),
            ("done", "")
        ]

        self.get_logger().info(" Test start")

    def send_next_state(self):
        if self.state_index >= len(self.state_sequence):
            self.get_logger().info(" Test complete.")
            rclpy.shutdown()
            return

        state, obj = self.state_sequence[self.state_index]
        msg = TaskState()
        msg.state = state
        msg.object_name = obj

        self.publisher.publish(msg)
        self.get_logger().info(f"send state: {state}, object: {obj}")
        self.state_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = TestFeedbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
