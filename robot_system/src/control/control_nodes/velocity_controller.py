import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Range
import math

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.create_subscription(Path, '/local_traj', self.path_callback, 10)
        self.create_subscription(Odometry, '/pose', self.pose_callback, 10)
        self.create_subscription(Range, '/range', self.range_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.local_path = None
        self.current_pose = None
        self.obstacle_distance = float('inf')
        # PID coefficients
        self.kp = 0.5
        self.ki = 0.1
        self.kd = 0.05
        self.kp_ang = 1.0
        self.prev_error = 0.0
        self.error_sum = 0.0
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.get_logger().info('VelocityController: PID control loop started')

    def path_callback(self, msg):
        self.local_path = msg

    def pose_callback(self, msg):
        self.current_pose = msg

    def range_callback(self, msg):
        self.obstacle_distance = msg.range

    def control_loop(self):
        if not self.local_path or not self.current_pose:
            return
        if not self.local_path.poses:
            return
        # Next waypoint
        target = self.local_path.poses[0].pose.position
        x = self.current_pose.pose.pose.position.x
        y = self.current_pose.pose.pose.position.y
        dx = target.x - x
        dy = target.y - y
        distance = math.hypot(dx, dy)
        # Compute heading error
        target_yaw = math.atan2(dy, dx)
        q = self.current_pose.pose.pose.orientation
        siny = 2.0*(q.w*q.z)
        cosy = 1.0 - 2.0*(q.z*q.z)
        current_yaw = math.atan2(siny, cosy)
        yaw_error = self.normalize_angle(target_yaw - current_yaw)
        # PID for linear velocity
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-6
        error = distance
        self.error_sum += error * dt
        d_error = (error - self.prev_error) / dt
        linear_speed = self.kp*error + self.ki*self.error_sum + self.kd*d_error
        angular_speed = self.kp_ang * yaw_error
        # Obstacle check: stop if too close
        if self.obstacle_distance < 0.5:
            linear_speed = 0.0
            angular_speed = 0.0
        # Publish Twist
        cmd = Twist()
        cmd.linear.x = max(min(linear_speed, 1.0), -1.0)
        cmd.angular.z = max(min(angular_speed, 1.0), -1.0)
        self.publisher.publish(cmd)
        self.prev_error = error
        self.last_time = now

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

