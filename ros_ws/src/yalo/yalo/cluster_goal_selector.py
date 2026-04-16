import math
from typing import List, Optional, Sequence, Tuple

from geometry_msgs.msg import PoseArray, PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32, Int32MultiArray


class ClusterGoalSelector(Node):
    """Pick the next cluster goal from entropy and cluster metadata."""

    def __init__(self) -> None:
        super().__init__('cluster_goal_selector')

        self.declare_parameter('centroids_topic', '/frontier_centroids')
        self.declare_parameter('cluster_sizes_topic', '/frontier_cluster_sizes')
        self.declare_parameter('cluster_entropy_topic', '/cluster_entropy')
        self.declare_parameter('selected_goal_topic', '/selected_cluster_goal')
        self.declare_parameter('selected_index_topic', '/selected_cluster_index')
        self.declare_parameter('timer_period', 0.5)

        # Entropy must dominate by default. Increase this above 0.0 only if you
        # want to keep the robot on its current region when entropy is close.
        self.declare_parameter('entropy_window', 0.0)

        # Distance is used to break ties / near-ties among candidates.
        self.declare_parameter('distance_weight', 1.0)

        self.centroids_topic = self.get_parameter('centroids_topic').value
        self.cluster_sizes_topic = (
            self.get_parameter('cluster_sizes_topic').value
        )
        self.cluster_entropy_topic = (
            self.get_parameter('cluster_entropy_topic').value
        )
        self.selected_goal_topic = self.get_parameter('selected_goal_topic').value
        self.selected_index_topic = (
            self.get_parameter('selected_index_topic').value
        )
        self.timer_period = float(self.get_parameter('timer_period').value)
        self.entropy_window = float(self.get_parameter('entropy_window').value)
        self.distance_weight = float(self.get_parameter('distance_weight').value)

        self.latest_centroids_msg: Optional[PoseArray] = None
        self.latest_cluster_sizes: Optional[List[int]] = None
        self.latest_entropy: Optional[List[float]] = None

        self.prev_goal_xy: Optional[Tuple[float, float]] = None
        self.prev_selected_index: Optional[int] = None

        self.create_subscription(
            PoseArray,
            self.centroids_topic,
            self.centroids_callback,
            10,
        )
        self.create_subscription(
            Int32MultiArray,
            self.cluster_sizes_topic,
            self.cluster_sizes_callback,
            10,
        )
        self.create_subscription(
            Float32MultiArray,
            self.cluster_entropy_topic,
            self.entropy_callback,
            10,
        )

        self.goal_pub = self.create_publisher(PoseStamped, self.selected_goal_topic, 10)
        self.index_pub = self.create_publisher(Int32, self.selected_index_topic, 10)

        self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info(
            'Cluster goal selector ready. Waiting for centroids, sizes, and entropy.'
        )

    def centroids_callback(self, msg: PoseArray) -> None:
        self.latest_centroids_msg = msg

    def cluster_sizes_callback(self, msg: Int32MultiArray) -> None:
        self.latest_cluster_sizes = list(msg.data)

    def entropy_callback(self, msg: Float32MultiArray) -> None:
        self.latest_entropy = [float(v) for v in msg.data]

    def timer_callback(self) -> None:
        if self.latest_centroids_msg is None:
            return
        if self.latest_cluster_sizes is None or self.latest_entropy is None:
            return

        centroids = self.latest_centroids_msg.poses
        sizes = self.latest_cluster_sizes
        entropies = self.latest_entropy

        if not centroids:
            return

        n = len(centroids)
        if len(sizes) != n or len(entropies) != n:
            self.get_logger().warn(
                'Length mismatch: centroids=%d sizes=%d entropy=%d',
                n,
                len(sizes),
                len(entropies),
            )
            return

        selected_index = self.select_index(centroids, sizes, entropies)
        self.publish_selection(selected_index)

    def select_index(
        self,
        centroids: Sequence,
        sizes: Sequence[int],
        entropies: Sequence[float],
    ) -> int:
        max_entropy = max(entropies)

        # Candidates are the top entropy clusters (or near-top if entropy_window > 0).
        candidate_indices = [
            idx
            for idx, entropy in enumerate(entropies)
            if entropy >= (max_entropy - self.entropy_window)
        ]

        if not candidate_indices:
            return 0

        def distance_to_prev(idx: int) -> float:
            if self.prev_goal_xy is None:
                return 0.0
            point = centroids[idx].position
            dx = point.x - self.prev_goal_xy[0]
            dy = point.y - self.prev_goal_xy[1]
            return math.hypot(dx, dy)

        # First sort by entropy descending, then cluster size descending,
        # then prefer closer to previous goal to avoid front-back oscillation.
        candidate_indices.sort(
            key=lambda idx: (
                -entropies[idx],
                -sizes[idx],
                self.distance_weight * distance_to_prev(idx),
                idx,
            )
        )
        return candidate_indices[0]

    def publish_selection(self, selected_index: int) -> None:
        centroids_msg = self.latest_centroids_msg
        if centroids_msg is None:
            return

        selected_pose = centroids_msg.poses[selected_index]

        goal_msg = PoseStamped()
        goal_msg.header = centroids_msg.header
        goal_msg.pose = selected_pose

        index_msg = Int32()
        index_msg.data = selected_index

        self.goal_pub.publish(goal_msg)
        self.index_pub.publish(index_msg)

        self.prev_goal_xy = (
            selected_pose.position.x,
            selected_pose.position.y,
        )

        if self.prev_selected_index != selected_index:
            self.get_logger().info(
                'Selected cluster %d (goal x=%.2f y=%.2f)',
                selected_index,
                selected_pose.position.x,
                selected_pose.position.y,
            )
        self.prev_selected_index = selected_index


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ClusterGoalSelector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()