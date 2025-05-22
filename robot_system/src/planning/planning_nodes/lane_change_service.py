import rclpy
from rclpy.node import Node
from planning.srv import LaneChange
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class LaneChangeService(Node):
    def __init__(self):
        super().__init__('lane_change_service')
        self.srv = self.create_service(LaneChange, 'lane_change', self.handle_request)
        self.get_logger().info('LaneChangeService: ready')

    def handle_request(self, request, response):
        direction = request.direction
        self.get_logger().info(f'Received lane change request: direction={direction}')
        # Create a dummy updated path (straight line offset in y by 'direction')
        new_path = Path()
        new_path.header.stamp = self.get_clock().now().to_msg()
        new_path.header.frame_id = 'map'
        for i in range(10):
            pose = PoseStamped()
            pose.header = new_path.header
            pose.pose.position.x = float(i)
            pose.pose.position.y = float(direction)  # offset path
            pose.pose.orientation.w = 1.0
            new_path.poses.append(pose)
        response.updated_path = new_path
        return response

def main(args=None):
    rclpy.init(args=args)
    node = LaneChangeService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

