import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
import math

class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.publisher = self.create_publisher(Odometry, '/odom_raw', 10)
        # Subscribe to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.update_odometry)  # 10 Hz
        # Robot state (x, y, yaw) and velocities
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()
        self.get_logger().info('OdomNode: publishing /odom_raw')

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_odometry(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        # Integrate motion
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt
        # Create Odometry message
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.w
        self.publisher.publish(msg)
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

