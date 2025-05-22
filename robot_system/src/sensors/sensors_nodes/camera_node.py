import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(0.5, self.publish_image)  # 2 Hz
        self.get_logger().info('CameraNode: publishing /camera/image_raw')

    def publish_image(self):
        # Generate a dummy gray image (480x640 RGB)
        height = 480
        width = 640
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[:] = 128  # medium gray
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = height
        msg.width = width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = 3 * width
        msg.data = image.tobytes()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

