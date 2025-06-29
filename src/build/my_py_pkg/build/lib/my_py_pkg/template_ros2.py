#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# rcl : ros2 client library
 
 
class MyCustomNode(Node):
    def __init__(self):
        super().__init__("node_name") ## 노드 이름
 
def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()