import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose

class FiducialDetector(Node):
    def __init__(self):
        super().__init__('fiducial_detector')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(PoseArray, '/fiducials', 10)
        self.get_logger().info('FiducialDetector: listening on /camera/image_raw')

    def image_callback(self, msg):
        # Simulate detection of a fiducial at a fixed world location
        pose = Pose()
        pose.position.x = 2.0
        pose.position.y = 3.0
        pose.position.z = 0.0
        pose.orientation.w = 1.0
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = 'map'
        pose_array.poses.append(pose)
        self.publisher.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = FiducialDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

