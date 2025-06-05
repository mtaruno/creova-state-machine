#!/usr/bin/env python3
"""
nav_ready_trigger.py
--------------------
Listens to /nav2_status (c8nav/msg/Nav2Status).
When `ready` becomes True, calls /trigger_task (std_srvs/srv/Trigger) once.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from c8nav.msg import Nav2Status   # adjust if package name differs


class NavReadyTrigger(Node):
    def __init__(self):
        super().__init__('nav_ready_trigger')

        # Subscribe to the Nav2 status topic
        self.create_subscription(
            Nav2Status,
            '/nav2_status',
            self.status_callback,
            10
        )

        # Service client for /trigger_task
        self.trigger_cli = self.create_client(Trigger, '/trigger_task')

        self._prev_ready = False  # track last ready state
        self.get_logger().info('NavReadyTrigger up: waiting for /nav2_status …')

    # ------------------------------------------------------------------
    #   Topic callback
    # ------------------------------------------------------------------
    def status_callback(self, msg: Nav2Status):
        ready_now = msg.ready

        # Rising edge: False -> True
        if ready_now and not self._prev_ready:
            self.call_trigger_service()

        self._prev_ready = ready_now

    # ------------------------------------------------------------------
    #   Service call helpers
    # ------------------------------------------------------------------
    def call_trigger_service(self):
        if not self.trigger_cli.wait_for_service(timeout_sec=0.0):
            self.get_logger().warning('/trigger_task service not available yet')
            return

        req = Trigger.Request()
        future = self.trigger_cli.call_async(req)
        future.add_done_callback(self.trigger_response_cb)

    def trigger_response_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f'Triggered task: {resp.message}')
            else:
                self.get_logger().warning(f'Trigger service returned failure: {resp.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = NavReadyTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
