from collections import deque

from yalo.frontier_utils import (
    find_frontiers,
    FREE_THRESHOLD,
    frontier_centroid,
    get_neighbors,
    grid_to_world,
    world_to_grid,
)
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Int32MultiArray
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
        self.declare_parameter(
            'frontier_cluster_sizes_topic',
            '/frontier_cluster_sizes',
        )

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
        self.frontier_cluster_sizes_topic = (
            self.get_parameter('frontier_cluster_sizes_topic').value
        )

        self.latest_map = None

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

    def timer_callback(self):
        if self.latest_map is None:
            return

        robot_cell = self.get_robot_cell(self.latest_map)
        if robot_cell is None:
            return

        frontiers = find_frontiers(self.latest_map, robot_cell)
        self.publish_frontiers(frontiers, self.latest_map)

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

        cluster_size_msg = Int32MultiArray()

        centroid_points = []

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
            cluster_size_msg.data.append(len(frontier))

            centroid_points.append(Point(x=wx, y=wy, z=0.12))
            self.get_logger().info(
                f'Centroid {index}: world=({wx:.2f}, {wy:.2f}), '
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
        centroid_marker.color.r = 0.0
        centroid_marker.color.g = 1.0
        centroid_marker.color.b = 0.0
        centroid_marker.color.a = 1.0
        centroid_marker.points = centroid_points
        marker_array.markers.append(centroid_marker)

        self.get_logger().info(
            f'Published {len(frontiers)} frontier centroids this cycle.'
        )

        self.marker_pub.publish(marker_array)
        self.centroid_pub.publish(centroid_msg)
        self.cluster_size_pub.publish(cluster_size_msg)


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
