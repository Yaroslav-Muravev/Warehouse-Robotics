import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path

class LocalTrajectory(Node):
    def __init__(self):
        super().__init__('local_trajectory')
        self.subscription = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.publisher = self.create_publisher(Path, '/local_traj', 10)
        self.get_logger().info('LocalTrajectory: ready for /global_path')

    def path_callback(self, msg):
        N = 5
        local_path = Path()
        local_path.header = msg.header
        for i, pose in enumerate(msg.poses):
            if i >= N:
                break
            local_path.poses.append(pose)
        self.publisher.publish(local_path)

def main(args=None):
    rclpy.init(args=args)
    node = LocalTrajectory()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

