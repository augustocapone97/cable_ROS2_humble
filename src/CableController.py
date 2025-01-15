#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class MoveEndSphere(Node):
    def __init__(self):
        super().__init__('move_end_sphere')
        self.publisher_ = self.create_publisher(Point, 'cable_target_position', 10)
        self.target_position = Point()

    def move(self):
        offsets = [
            (0.2, 0.6, 2.0),
            (0.2, 0.3, 2.0),
            (-0.2, 0.3, 2.0),
            (-0.2, 0.6, 2.0)
        ]

        for x, y, z in offsets:
            self.target_position.x = x
            self.target_position.y = y
            self.target_position.z = z
            self.publisher_.publish(self.target_position)
            self.get_logger().info(f'Moved to position: x={x}, y={y}, z={z}')
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    move_end_sphere = MoveEndSphere()
    
    try:
        move_end_sphere.move()
    except KeyboardInterrupt:
        pass

    move_end_sphere.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
