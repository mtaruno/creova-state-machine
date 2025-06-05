#!/usr/bin/env python3
"""
final_detection_subscriber.py
-----------------------------
Listens on /final_detection, prints the task_id and (x, y) every time a
FinalDetection message arrives.

Drop this file into any Python-based package (e.g. creova_state_machine/scripts/)
and add an entry-point in setup.py, or just run it with python3 <path>.py after
sourcing your workspace.

Message definition assumed (FinalDetection.msg)
    int32   task_id
    float64 x
    float64 y
"""
import rclpy
from rclpy.node import Node
from trans_msgs.msg import FinalDetection   # auto-generated from your .msg

class FinalDetectionSubscriber(Node):
    def __init__(self) -> None:
        super().__init__("final_detection_subscriber")

        # QoS depth 10 is plenty for small, infrequent messages
        self.create_subscription(
            FinalDetection,
            "/final_detection",           # topic name
            self.callback,
            10
        )
        self.get_logger().info("Listening on /final_detection")

    # ---------- callback -------------------------------------------------
    def callback(self, msg: FinalDetection) -> None:  # noqa: D401
        self.get_logger().info(
            f"Task {msg.task_id:>4}  →  x = {msg.x:.3f},  y = {msg.y:.3f}"
        )

# ---------- main --------------------------------------------------------
def main() -> None:            # noqa: D401
    rclpy.init()
    node = FinalDetectionSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
