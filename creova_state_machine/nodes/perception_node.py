#!/usr/bin/env python3
"""
perception_node.py
──────────────────
Subscriber for the topic `/detections_3d_transformed` (yolo_msgs/DetectionArray).

For every DetectionArray it logs how many detections were received and the
(x, y, z) coordinates of each 3-D bounding-box centre.

Usage
-----
# 1.  Place this file in your package’s scripts/ directory.
# 2.  Make it executable:  chmod +x perception_node.py
# 3.  Re-build and source your workspace.
# 4.  Run:  ros2 run <your_pkg> perception_node.py
"""

import rclpy
from rclpy.node import Node
from yolo_msgs.msg import DetectionArray


class PerceptionNode(Node):
    """Logs transformed 3-D YOLO detections."""

    def __init__(self) -> None:
        super().__init__("perception_node")  # Node name as requested

        self.create_subscription(
            DetectionArray,
            "/detections_3d_transformed",   # topic published by the transformer
            self._callback,
            10                               # QoS depth
        )

        self.get_logger().info("🟣 perception_node listening on /detections_3d_transformed")

    # ------------------------------------------------------------------
    def _callback(self, msg: DetectionArray) -> None:
        n = len(msg.detections)
        if n == 0:
            return

        self.get_logger().info(f"Received {n} detection{'s' if n > 1 else ''}:")
        for i, det in enumerate(msg.detections, start=1):
            center = det.bbox3d.center.position
            self.get_logger().info(
                f"  #{i:02d} → x={center.x:.3f}, y={center.y:.3f}, z={center.z:.3f}"
            )


# ----------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
