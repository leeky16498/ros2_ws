#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus
## 커스텀 메세지를 모듈로 호출
 
class HardwarePulisherNode(Node): # OOP name
    def __init__(self):
        super().__init__("hardware_status_publisher") ## node name
        self.hw_status_pub_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer_ = self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info("Hardware status publisher has been started!")

    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 43.5
        msg.are_motors_ready = True
        msg.debug_message = "Nothing special!"
        self.hw_status_pub_.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = HardwarePulisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()