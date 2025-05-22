import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math

class RangeNode(Node):
    def __init__(self):
        super().__init__('range_node')
        self.publisher = self.create_publisher(Range, '/range', 10)
        self.timer = self.create_timer(0.2, self.publish_range)  # 5 Hz
        self.start_time = self.get_clock().now()
        self.get_logger().info('RangeNode: publishing /range')

    def publish_range(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9
        distance = 2.0 + 0.5 * math.sin(t)  # oscillate around 2m
        msg = Range()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'range_sensor'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.1
        msg.min_range = 0.2
        msg.max_range = 10.0
        msg.range = float(abs(distance))
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RangeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

