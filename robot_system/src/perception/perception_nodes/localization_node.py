import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseArray
import math

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization_node')
        self.create_subscription(Odometry, '/odom_raw', self.odom_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.create_subscription(PoseArray, '/fiducials', self.fiducial_callback, 10)
        self.publisher = self.create_publisher(Odometry, '/pose', 10)
        # State [x,y] and orientation yaw
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        # Uncertainties
        self.Px = 1.0
        self.Py = 1.0
        self.last_time = None
        self.get_logger().info('LocalizationNode: EKF fusion')

    def odom_callback(self, msg):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        # Prediction: integrate odometry velocities
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += w * dt
        # Increase uncertainty
        self.Px += 0.1
        self.Py += 0.1
        self.last_time = now
        self.publish_pose(msg.header.stamp)

    def imu_callback(self, msg):
        # Optional: use IMU to refine yaw (assume pure yaw rotation)
        qz = msg.orientation.z
        qw = msg.orientation.w
        self.yaw = math.atan2(2.0*(qw*qz), 1.0 - 2.0*(qz*qz))

    def fiducial_callback(self, msg):
        if not msg.poses:
            return
        # Use first fiducial pose as a position measurement (zx, zy)
        z = msg.poses[0]
        zx = z.position.x
        zy = z.position.y
        # Kalman gain
        Kx = self.Px / (self.Px + 0.5)
        Ky = self.Py / (self.Py + 0.5)
        # Update estimate
        self.x += Kx * (zx - self.x)
        self.y += Ky * (zy - self.y)
        # Update uncertainties
        self.Px *= (1 - Kx)
        self.Py *= (1 - Ky)
        # Publish corrected pose
        self.publish_pose(self.get_clock().now().to_msg())

    def publish_pose(self, stamp):
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

