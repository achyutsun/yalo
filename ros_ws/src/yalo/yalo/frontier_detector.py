from collections import deque
import math
from typing import List, Tuple

from yalo.frontier_utils import (
    find_frontiers,
    FREE_THRESHOLD,
    frontier_centroid,
    get_neighbors,
    grid_to_world,
    world_to_grid,
)
from geometry_msgs.msg import Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import ColorRGBA, Float32MultiArray, Int32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class FrontierDetector(Node):
    """Detect frontier clusters from /map and publish RViz markers."""

    def __init__(self):
        super().__init__('frontier_detector')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('timer_period', 1.0)
        self.declare_parameter('frontier_markers_topic', '/frontier_markers')
        self.declare_parameter('frontier_centroids_topic', '/frontier_centroids')
        self.declare_parameter('frontier_entropy_topic', '/frontier_entropy_scores')
        self.declare_parameter(
            'frontier_cluster_sizes_topic',
            '/frontier_cluster_sizes',
        )
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('goal_match_tolerance_m', 0.40)
        self.declare_parameter('max_published_frontiers', 8)
        self.declare_parameter('min_published_cluster_size', 20)
        self.declare_parameter('min_centroid_separation_m', 0.60)

        self.map_topic = self.get_parameter('map_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.timer_period = float(self.get_parameter('timer_period').value)
        self.frontier_markers_topic = (
            self.get_parameter('frontier_markers_topic').value
        )
        self.frontier_centroids_topic = (
            self.get_parameter('frontier_centroids_topic').value
        )
        self.frontier_entropy_topic = (
            self.get_parameter('frontier_entropy_topic').value
        )
        self.frontier_cluster_sizes_topic = (
            self.get_parameter('frontier_cluster_sizes_topic').value
        )
        self.goal_topic = self.get_parameter('goal_topic').value
        self.goal_match_tolerance_m = float(
            self.get_parameter('goal_match_tolerance_m').value
        )
        self.max_published_frontiers = int(
            self.get_parameter('max_published_frontiers').value
        )
        self.min_published_cluster_size = int(
            self.get_parameter('min_published_cluster_size').value
        )
        self.min_centroid_separation_m = float(
            self.get_parameter('min_centroid_separation_m').value
        )

        self.latest_map = None
        self.latest_goal_world = None

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            map_qos,
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            self.frontier_markers_topic,
            10,
        )
        self.centroid_pub = self.create_publisher(
            PoseArray,
            self.frontier_centroids_topic,
            10,
        )
        self.entropy_pub = self.create_publisher(
            Float32MultiArray,
            self.frontier_entropy_topic,
            10,
        )
        self.cluster_size_pub = self.create_publisher(
            Int32MultiArray,
            self.frontier_cluster_sizes_topic,
            10,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            'Frontier detector ready. Waiting for map and TF.'
        )

    def map_callback(self, msg):
        self.latest_map = msg

    def goal_callback(self, msg: PoseStamped):
        self.latest_goal_world = (
            msg.pose.position.x,
            msg.pose.position.y,
        )

    def timer_callback(self):
        if self.latest_map is None:
            return

        robot_cell = self.get_robot_cell(self.latest_map)
        if robot_cell is None:
            return

        frontiers = find_frontiers(self.latest_map, robot_cell)
        filtered_frontiers = self.filter_frontiers(frontiers, self.latest_map)
        self.publish_frontiers(filtered_frontiers, self.latest_map)

    def get_robot_cell(self, map_msg):
        """Look up the robot pose in the map frame and convert to grid."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().debug(
                f'Waiting for TF {self.global_frame}->{self.robot_frame}: {exc}'
            )
            return None

        translation = transform.transform.translation
        robot_cell = world_to_grid(map_msg, translation.x, translation.y)
        if robot_cell is None:
            self.get_logger().warn(
                'Robot pose is outside the map bounds; skipping frontier '
                'detection cycle.'
            )
            self.publish_frontiers([], map_msg)
            return None

        nearest_free_cell = self.find_nearest_free_cell(map_msg, robot_cell)
        if nearest_free_cell is None:
            self.get_logger().warn(
                'Could not find a free start cell near the robot pose.'
            )
            self.publish_frontiers([], map_msg)
            return None

        return nearest_free_cell

    def find_nearest_free_cell(self, map_msg, start_cell):
        """Snap the BFS start point to the closest free cell if needed."""
        width = map_msg.info.width
        height = map_msg.info.height
        grid = np.array(map_msg.data).reshape((height, width))

        queue = deque([start_cell])
        visited = {start_cell}

        while queue:
            x, y = queue.popleft()

            if 0 <= grid[y, x] <= FREE_THRESHOLD:
                return x, y

            for nx, ny in get_neighbors(x, y, width, height):
                if (nx, ny) in visited:
                    continue

                visited.add((nx, ny))
                queue.append((nx, ny))

        return None

    def publish_frontiers(self, frontiers, map_msg):
        """Publish RViz markers and simple ROS messages for later scoring."""
        frame_id = map_msg.header.frame_id or self.global_frame
        stamp = map_msg.header.stamp

        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self.get_clock().now().to_msg()

        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = frame_id
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        centroid_msg = PoseArray()
        centroid_msg.header.frame_id = frame_id
        centroid_msg.header.stamp = stamp

        entropy_msg = Float32MultiArray()
        cluster_size_msg = Int32MultiArray()

        centroid_points = []
        centroid_colors = []

        for index, frontier in enumerate(frontiers):
            frontier_marker = Marker()
            frontier_marker.header.frame_id = frame_id
            frontier_marker.header.stamp = stamp
            frontier_marker.ns = 'frontier_clusters'
            frontier_marker.id = index
            frontier_marker.type = Marker.CUBE_LIST
            frontier_marker.action = Marker.ADD
            frontier_marker.pose.orientation.w = 1.0
            frontier_marker.scale.x = map_msg.info.resolution
            frontier_marker.scale.y = map_msg.info.resolution
            frontier_marker.scale.z = 0.03
            frontier_marker.color.r = 1.0
            frontier_marker.color.g = 0.0
            frontier_marker.color.b = 0.0
            frontier_marker.color.a = 0.95

            for mx, my in frontier:
                wx, wy = grid_to_world(map_msg, mx, my)
                frontier_marker.points.append(
                    Point(x=wx, y=wy, z=0.0)
                )

            marker_array.markers.append(frontier_marker)

            cx, cy = frontier_centroid(frontier)
            wx, wy = grid_to_world(map_msg, cx, cy)

            centroid_pose = Pose()
            centroid_pose.position.x = wx
            centroid_pose.position.y = wy
            centroid_pose.position.z = 0.12
            centroid_pose.orientation.w = 1.0
            centroid_msg.poses.append(centroid_pose)

            frontier_entropy = self.estimate_frontier_entropy(map_msg, frontier)
            entropy_msg.data.append(frontier_entropy)
            cluster_size_msg.data.append(len(frontier))

            centroid_points.append(Point(x=wx, y=wy, z=0.12))
            if self.is_goal_centroid(wx, wy):
                centroid_colors.append(ColorRGBA(r=0.0, g=0.35, b=1.0, a=1.0))
            else:
                centroid_colors.append(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
            self.get_logger().info(
                f'Centroid {index}: world=({wx:.2f}, {wy:.2f}), '
                f'entropy={frontier_entropy:.3f}, '
                f'cluster_size={len(frontier)}'
            )

        centroid_marker = Marker()
        centroid_marker.header.frame_id = frame_id
        centroid_marker.header.stamp = stamp
        centroid_marker.ns = 'frontier_centroids'
        centroid_marker.id = len(frontiers)
        centroid_marker.type = Marker.SPHERE_LIST
        centroid_marker.action = Marker.ADD
        centroid_marker.pose.orientation.w = 1.0
        centroid_marker.scale.x = 0.28
        centroid_marker.scale.y = 0.28
        centroid_marker.scale.z = 0.28
        centroid_marker.color.r = 1.0
        centroid_marker.color.g = 1.0
        centroid_marker.color.b = 1.0
        centroid_marker.color.a = 1.0
        centroid_marker.points = centroid_points
        centroid_marker.colors = centroid_colors
        marker_array.markers.append(centroid_marker)

        self.get_logger().info(
            f'Published {len(frontiers)} frontier centroids this cycle.'
        )

        self.marker_pub.publish(marker_array)
        self.centroid_pub.publish(centroid_msg)
        self.entropy_pub.publish(entropy_msg)
        self.cluster_size_pub.publish(cluster_size_msg)

    def filter_frontiers(self, frontiers, map_msg) -> List[List[Tuple[int, int]]]:
        """Keep only larger and well-separated frontiers for stable visual output."""
        if not frontiers:
            return []

        sorted_frontiers = sorted(frontiers, key=len, reverse=True)
        selected: List[List[Tuple[int, int]]] = []
        selected_centroids: List[Tuple[float, float]] = []

        for frontier in sorted_frontiers:
            if len(frontier) < self.min_published_cluster_size:
                continue

            cx, cy = frontier_centroid(frontier)
            wx, wy = grid_to_world(map_msg, cx, cy)

            too_close = False
            for sx, sy in selected_centroids:
                if math.hypot(wx - sx, wy - sy) < self.min_centroid_separation_m:
                    too_close = True
                    break

            if too_close:
                continue

            selected.append(frontier)
            selected_centroids.append((wx, wy))

            if len(selected) >= self.max_published_frontiers:
                break

        if not selected and sorted_frontiers:
            selected = sorted_frontiers[:1]

        return selected

    def is_goal_centroid(self, centroid_x: float, centroid_y: float) -> bool:
        """Return True if centroid is close to the latest navigation goal."""
        if self.latest_goal_world is None:
            return False

        gx, gy = self.latest_goal_world
        return (
            math.hypot(centroid_x - gx, centroid_y - gy)
            <= self.goal_match_tolerance_m
        )

    def estimate_frontier_entropy(self, map_msg, frontier):
        """Estimate a normalized entropy score around the frontier centroid."""
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        grid = np.array(map_msg.data, dtype=np.float32).reshape((height, width))

        cx, cy = frontier_centroid(frontier)
        radius_cells = max(1, int(math.ceil(0.5 / max(resolution, 1e-6))))

        x0 = max(0, cx - radius_cells)
        x1 = min(width, cx + radius_cells + 1)
        y0 = max(0, cy - radius_cells)
        y1 = min(height, cy + radius_cells + 1)

        patch = grid[y0:y1, x0:x1]
        unknown_mask = patch == -1

        probabilities = np.where(
            unknown_mask,
            0.5,
            np.clip(0.05 + (patch / 100.0) * 0.90, 1e-9, 1.0 - 1e-9),
        )
        entropy = -(
            probabilities * np.log(probabilities)
            + (1.0 - probabilities) * np.log(1.0 - probabilities)
        ) / math.log(2.0)
        entropy = np.where(unknown_mask, 1.0, entropy)

        return float(np.mean(entropy))


def main(args=None):
    rclpy.init(args=args)
    node = FrontierDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
