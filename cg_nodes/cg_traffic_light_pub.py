#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TrafficLightPub(Node):
    def __init__(self):
        super().__init__('cg_traffic_light_pub')

        self.pub = self.create_publisher(String, '/cg/traffic_light', 10)

        # seconds
        self.green_s = 45
        self.yellow_s = 10
        self.red_s = 22

        self.phase = 'GREEN'
        self.remaining = self.green_s

        self.timer = self.create_timer(1.0, self.tick)
        self.publish_now()

        self.get_logger().info("Traffic light publisher started on /cg/traffic_light")
        self.get_logger().info(f"Cycle: GREEN({self.green_s}) -> YELLOW({self.yellow_s}) -> RED({self.red_s})")

    def publish_now(self):
        msg = String()
        msg.data = self.phase
        self.pub.publish(msg)
        self.get_logger().info(f"Traffic Light: {self.phase} ({self.remaining}s)")

    def tick(self):
        self.remaining -= 1
        if self.remaining <= 0:
            if self.phase == 'GREEN':
                self.phase = 'YELLOW'
                self.remaining = self.yellow_s
            elif self.phase == 'YELLOW':
                self.phase = 'RED'
                self.remaining = self.red_s
            else:
                self.phase = 'GREEN'
                self.remaining = self.green_s

        self.publish_now()

def main():
    rclpy.init()
    node = TrafficLightPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
