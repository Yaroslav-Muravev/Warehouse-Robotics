import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from planning.srv import LaneChange
import math
import heapq

class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.subscription = self.create_subscription(Odometry, '/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Path, '/global_path', 10)
        # Service client for lane changes
        self.lane_change_client = self.create_client(LaneChange, 'lane_change')
        self.grid = [
            [0,0,0,0,0,0,0,0,0,0],
            [0,1,1,0,0,0,1,1,1,0],
            [0,1,0,0,1,0,0,0,1,0],
            [0,0,0,0,1,0,0,0,0,0],
            [0,0,0,0,0,0,0,1,0,0],
            [0,0,1,1,0,0,0,0,0,0],
            [0,0,0,0,0,0,1,0,1,0],
            [0,1,0,0,0,0,1,0,1,0],
            [0,0,0,1,1,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0]
        ]
        self.goal = (9, 9)
        self.get_logger().info('GlobalPlanner: ready (A* planning)')

    def pose_callback(self, msg):
        # Convert current pose to grid coordinates (round)
        x = int(round(msg.pose.pose.position.x))
        y = int(round(msg.pose.pose.position.y))
        if x < 0 or y < 0 or y >= len(self.grid) or x >= len(self.grid[0]):
            self.get_logger().warn(f'Pose ({x},{y}) out of grid')
            return
        self.get_logger().info(f'Planning from ({x},{y}) to {self.goal}')
        path = self.astar((x, y), self.goal)
        if path is None:
            self.get_logger().warn('A*: no path found')
            return
        # Convert to nav_msgs/Path
        nav_path = Path()
        nav_path.header.stamp = self.get_clock().now().to_msg()
        nav_path.header.frame_id = 'map'
        for (px, py) in path:
            pose = PoseStamped()
            pose.header = nav_path.header
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            pose.pose.orientation.w = 1.0
            nav_path.poses.append(pose)
        # Request lane change (example: shift lane)
        self.request_lane_change(nav_path)
        # Publish the global path
        self.publisher.publish(nav_path)

    def astar(self, start, goal):
        # A* search on a 2D grid (0 = free, 1 = obstacle)
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start, [start]))
        visited = set()
        while open_set:
            est_total, cost, current, path = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)
            if current == goal:
                return path
            for dx, dy in [(1,0),(-1,0),(0,1),(0,-1)]:
                nx, ny = current[0]+dx, current[1]+dy
                if 0 <= ny < len(self.grid) and 0 <= nx < len(self.grid[0]) and self.grid[ny][nx] == 0:
                    new_cost = cost + 1
                    est = new_cost + self.heuristic((nx,ny), goal)
                    heapq.heappush(open_set, (est, new_cost, (nx,ny), path + [(nx,ny)]))
        return None

    def heuristic(self, a, b):
        # Euclidean distance
        return math.hypot(b[0]-a[0], b[1]-a[1])

    def request_lane_change(self, path_msg):
        if not self.lane_change_client.service_is_ready():
            self.get_logger().warn('LaneChange service not available')
            return
        req = LaneChange.Request()
        req.direction = 1  # e.g., shift right
        future = self.lane_change_client.call_async(req)
        future.add_done_callback(lambda fut: self.lane_change_callback(fut))

    def lane_change_callback(self, future):
        try:
            resp = future.result()
            if resp and resp.updated_path.poses:
                self.get_logger().info('Received updated path from LaneChangeService')
                # Publish the updated path instead
                self.publisher.publish(resp.updated_path)
        except Exception as e:
            self.get_logger().error(f'LaneChange call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

