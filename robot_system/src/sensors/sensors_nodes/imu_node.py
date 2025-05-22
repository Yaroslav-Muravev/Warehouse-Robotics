import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Twist
import math

class IMUNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.publish_imu)  # 10 Hz
        self.yaw = 0.0
        self.prev_v = 0.0
        self.v = 0.0
        self.w_rate = 0.0
        self.last_time = self.get_clock().now()
        self.get_logger().info('IMUNode: publishing /imu/data')

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w_rate = msg.angular.z

    def publish_imu(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        # Integrate yaw
        self.yaw += self.w_rate * dt
        # Compute acceleration (simple finite difference)
        accel = 0.0
        if dt > 0:
            accel = (self.v - self.prev_v) / dt
        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'base_link'
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        msg.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
        msg.angular_velocity.z = self.w_rate
        msg.linear_acceleration.x = accel
        self.publisher.publish(msg)
        self.prev_v = self.v
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

