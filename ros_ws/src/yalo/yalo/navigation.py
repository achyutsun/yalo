#!/usr/bin/env python3

import math
import random
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException, TransformListener


class FrontierNavigator(Node):

    def __init__(self):
        super().__init__('frontier_navigator')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('selection_mode', 'first')  # 'first' or 'random'

        self.global_frame = self.get_parameter('global_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.selection_mode = self.get_parameter('selection_mode').value

        # Nav2 client
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribers
        self.centroid_sub = self.create_subscription(
            PoseArray,
            '/frontier_centroids',
            self.centroid_callback,
            10
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Visualization publishers
        self.centroid_marker_pub = self.create_publisher(Marker, '/chosen_centroid_marker', 10)
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        self.path_marker_pub = self.create_publisher(Marker, '/planned_path_marker', 10)

        self.latest_path = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_map = None
        self.goal_in_progress = False

        # Wait for Nav2
        self.get_logger().info("Waiting for Nav2 action server...")
        while not self._client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("Still waiting for Nav2...")
        self.get_logger().info("Nav2 ready.")

    # ==============================
    # Map handling
    # ==============================
    def map_callback(self, msg):
        self.latest_map = msg

    def world_to_grid(self, map_msg, x, y):
        res = map_msg.info.resolution
        origin = map_msg.info.origin.position

        gx = int((x - origin.x) / res)
        gy = int((y - origin.y) / res)

        if gx < 0 or gy < 0 or gx >= map_msg.info.width or gy >= map_msg.info.height:
            return None

        return gx, gy

    def grid_to_world(self, map_msg, gx, gy):
        res = map_msg.info.resolution
        origin = map_msg.info.origin.position

        x = origin.x + (gx + 0.5) * res
        y = origin.y + (gy + 0.5) * res

        return x, y

    def find_nearest_free(self, map_msg, x, y):
        grid = np.array(map_msg.data).reshape(
            (map_msg.info.height, map_msg.info.width)
        )

        start = self.world_to_grid(map_msg, x, y)
        if start is None:
            return None

        queue = deque([start])
        visited = {start}

        while queue:
            gx, gy = queue.popleft()

            if grid[gy, gx] == 0:  # FREE cell
                return self.grid_to_world(map_msg, gx, gy)

            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = gx + dx, gy + dy

                if (nx, ny) in visited:
                    continue

                if 0 <= nx < map_msg.info.width and 0 <= ny < map_msg.info.height:
                    visited.add((nx, ny))
                    queue.append((nx, ny))

        return None

    # ==============================
    # TF helpers
    # ==============================
    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time()
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
            return None

        t = transform.transform.translation
        return t.x, t.y

    @staticmethod
    def yaw_to_quaternion(yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return 0.0, 0.0, qz, qw

    # ==============================
    # Core logic
    # ==============================
    def centroid_callback(self, msg: PoseArray):
        if self.goal_in_progress or self.latest_map is None:
            return

        if not msg.poses:
            self.get_logger().info("No centroids received.")
            return

        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return

        rx, ry = robot_pos

        # need robot yaw for filtering direction
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_frame,
                Time()
            )
            q = tf.transform.rotation

            # yaw extraction
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

        except Exception:
            self.get_logger().warn("TF yaw lookup failed")
            return

        # ==============================
        # 1. FILTER FRONT CENTROIDS ONLY
        # ==============================
        front_centroids = []

        for p in msg.poses:
            cx = p.position.x
            cy = p.position.y
            

            if self.is_in_front(rx, ry, yaw, cx, cy):
                front_centroids.append(p)

        if not front_centroids:
            self.get_logger().warn("No front-facing centroids available.")
            return

        # ==============================
        # 2. SELECT CENTROID
        # ==============================

        if self.selection_mode == "random":
            target = random.choice(front_centroids)
        else:
            # "first" BUT already filtered
            # OR choose best (largest cluster if encoded elsewhere)
            target = front_centroids[0]

        cx = target.position.x
        cy = target.position.y

        self.publish_centroid_marker(cx, cy)

        snapped = self.find_nearest_free(self.latest_map, cx, cy)

        if snapped is None:
            self.get_logger().warn("No reachable free cell near centroid.")
            return

        gx, gy = snapped

        yaw_goal = math.atan2(gy - ry, gx - rx)

        self.send_goal(gx, gy, yaw_goal)

    def is_in_front(self, rx, ry, yaw, cx, cy):
        """
        Returns True if centroid is in front of robot.
        """

        dx = cx - rx
        dy = cy - ry

        # robot forward direction
        fx = math.cos(yaw)
        fy = math.sin(yaw)

        # dot product
        dot = dx * fx + dy * fy

        return dot > 0  # front half-plane only

    def send_goal(self, x, y, yaw):
        self.goal_in_progress = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = self.global_frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.x = qx
        goal_msg.pose.pose.orientation.y = qy
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Sending goal: ({x:.2f}, {y:.2f}) | yaw={yaw:.2f} rad"
        )

        send_goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    # ==============================
    # Callbacks
    # ==============================
    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send goal: {exc}")
            self.goal_in_progress = False
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2.")
            self.goal_in_progress = False
            return

        self.get_logger().info("Goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        if hasattr(feedback, 'distance_remaining'):
            self.get_logger().info(
                f"Distance remaining: {feedback.distance_remaining:.2f}"
            )

    def result_callback(self, future):
        try:
            _ = future.result().result
            self.get_logger().info("Navigation finished.")
        except Exception as exc:
            self.get_logger().error(f"Navigation result failed: {exc}")
        finally:
            self.goal_in_progress = False

    def publish_centroid_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "chosen_centroid"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.1

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # 🔵 BLUE color
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.lifetime.sec = 2  # disappears after 2 sec

        self.centroid_marker_pub.publish(marker)

    def path_callback(self, msg: Path):
        self.latest_path = msg
        self.publish_path_marker(msg)

    def publish_path_marker(self, path_msg: Path):
        marker = Marker()
        marker.header.frame_id = self.global_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "global_path"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.03  # line thickness

        # 🟢 green path
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []

        for pose in path_msg.poses:
            p = pose.pose.position
            marker.points.append(p)

        self.path_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()