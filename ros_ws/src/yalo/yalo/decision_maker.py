#!/usr/bin/env python3
"""Weighted frontier decision node.

This node chooses the next frontier cluster goal using:
1. Entropy score (prefer higher)
2. Distance from robot to cluster (prefer lower)
3. Distance from previous goal to cluster (prefer lower for consistency)

If entropy values are not available yet, it falls back to cluster sizes.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray


@dataclass
class FrontierCandidate:
	"""Container for one frontier candidate."""

	index: int
	x: float
	y: float
	entropy_norm: float
	score: float = 0.0


class DecisionMaker(Node):
	"""Pick and publish the best frontier goal."""

	def __init__(self) -> None:
		super().__init__('decision_maker')

		self.declare_parameter('frontier_centroids_topic', '/frontier_centroids')
		self.declare_parameter('frontier_entropy_topic', '/frontier_entropy_scores')
		self.declare_parameter('frontier_cluster_sizes_topic', '/frontier_cluster_sizes')
		self.declare_parameter('odom_topic', '/odom')
		self.declare_parameter('goal_topic', '/goal_pose')
		self.declare_parameter('goal_frame', 'map')
		self.declare_parameter('decision_period', 1.0)

		self.declare_parameter('entropy_weight', 1.0)
		self.declare_parameter('distance_weight', 0.25)
		self.declare_parameter('consistency_weight', 0.45)
		self.declare_parameter('switch_margin', 0.15)
		self.declare_parameter('same_goal_tolerance', 0.20)

		self.frontier_centroids_topic = self.get_parameter(
			'frontier_centroids_topic'
		).value
		self.frontier_entropy_topic = self.get_parameter(
			'frontier_entropy_topic'
		).value
		self.frontier_cluster_sizes_topic = self.get_parameter(
			'frontier_cluster_sizes_topic'
		).value
		self.odom_topic = self.get_parameter('odom_topic').value
		self.goal_topic = self.get_parameter('goal_topic').value
		self.goal_frame = self.get_parameter('goal_frame').value
		self.decision_period = float(self.get_parameter('decision_period').value)

		self.entropy_weight = float(self.get_parameter('entropy_weight').value)
		self.distance_weight = float(self.get_parameter('distance_weight').value)
		self.consistency_weight = float(
			self.get_parameter('consistency_weight').value
		)
		self.switch_margin = float(self.get_parameter('switch_margin').value)
		self.same_goal_tolerance = float(
			self.get_parameter('same_goal_tolerance').value
		)

		self.centroids: List[Tuple[float, float]] = []
		self.entropy_values: List[float] = []
		self.cluster_sizes: List[int] = []

		self.robot_x = 0.0
		self.robot_y = 0.0
		self.have_odom = False

		self.last_goal: Optional[Tuple[float, float]] = None
		self.last_goal_score: Optional[float] = None

		self.create_subscription(
			PoseArray,
			self.frontier_centroids_topic,
			self._centroids_cb,
			10,
		)
		self.create_subscription(
			Float32MultiArray,
			self.frontier_entropy_topic,
			self._entropy_cb,
			10,
		)
		self.create_subscription(
			Int32MultiArray,
			self.frontier_cluster_sizes_topic,
			self._sizes_cb,
			10,
		)
		self.create_subscription(
			Odometry,
			self.odom_topic,
			self._odom_cb,
			10,
		)

		self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
		self.timer = self.create_timer(self.decision_period, self._timer_cb)

		self.get_logger().info('decision_maker started')

	def _centroids_cb(self, msg: PoseArray) -> None:
		self.centroids = [(p.position.x, p.position.y) for p in msg.poses]

	def _entropy_cb(self, msg: Float32MultiArray) -> None:
		self.entropy_values = [float(v) for v in msg.data]

	def _sizes_cb(self, msg: Int32MultiArray) -> None:
		self.cluster_sizes = [int(v) for v in msg.data]

	def _odom_cb(self, msg: Odometry) -> None:
		pos = msg.pose.pose.position
		self.robot_x = pos.x
		self.robot_y = pos.y
		self.have_odom = True

	def _timer_cb(self) -> None:
		if not self.have_odom:
			return
		if not self.centroids:
			return

		raw_values = self._select_raw_values()
		if not raw_values:
			return

		values = self._align_values(raw_values)
		if not values:
			return

		candidates = self._build_candidates(values)
		if not candidates:
			return

		for c in candidates:
			c.score = self._score(c)

		best = max(candidates, key=lambda c: c.score)
		chosen = self._apply_hysteresis(candidates, best)

		# Avoid republishing almost the same goal point.
		if self.last_goal is not None:
			delta = math.hypot(
				chosen.x - self.last_goal[0],
				chosen.y - self.last_goal[1],
			)
			if delta < self.same_goal_tolerance:
				return

		self._publish_goal(chosen.x, chosen.y)
		self.last_goal = (chosen.x, chosen.y)
		self.last_goal_score = chosen.score

		self.get_logger().info(
			f'chosen cluster={chosen.index} '
			f'goal=({chosen.x:.2f}, {chosen.y:.2f}) '
			f'entropy_norm={chosen.entropy_norm:.3f} '
			f'score={chosen.score:.3f}'
		)

	def _select_raw_values(self) -> List[float]:
		if self.entropy_values:
			return [float(v) for v in self.entropy_values]
		if self.cluster_sizes:
			return [float(v) for v in self.cluster_sizes]
		return []

	def _align_values(self, raw_values: Sequence[float]) -> List[float]:
		n = min(len(self.centroids), len(raw_values))
		if n == 0:
			return []
		if len(self.centroids) != len(raw_values):
			self.get_logger().warn(
				'centroids and scores have different sizes; using overlap only'
			)
		return [float(raw_values[i]) for i in range(n)]

	def _build_candidates(self, values: Sequence[float]) -> List[FrontierCandidate]:
		max_value = max(values)
		if max_value <= 0.0:
			max_value = 1.0

		candidates: List[FrontierCandidate] = []
		for i, (x, y) in enumerate(self.centroids[: len(values)]):
			norm = values[i] / max_value
			candidates.append(
				FrontierCandidate(index=i, x=x, y=y, entropy_norm=norm)
			)
		return candidates

	def _score(self, c: FrontierCandidate) -> float:
		dist_robot = math.hypot(c.x - self.robot_x, c.y - self.robot_y)

		dist_prev = 0.0
		if self.last_goal is not None:
			dist_prev = math.hypot(c.x - self.last_goal[0], c.y - self.last_goal[1])

		return (
			self.entropy_weight * c.entropy_norm
			- self.distance_weight * dist_robot
			- self.consistency_weight * dist_prev
		)

	def _apply_hysteresis(
		self,
		candidates: Sequence[FrontierCandidate],
		best: FrontierCandidate,
	) -> FrontierCandidate:
		if self.last_goal is None:
			return best

		previous = self._closest_to_last_goal(candidates)
		if previous is None:
			return best

		improvement = best.score - previous.score
		if improvement <= self.switch_margin:
			return previous
		return best

	def _closest_to_last_goal(
		self,
		candidates: Sequence[FrontierCandidate],
	) -> Optional[FrontierCandidate]:
		if self.last_goal is None:
			return None

		nearest: Optional[FrontierCandidate] = None
		nearest_dist = float('inf')
		for c in candidates:
			d = math.hypot(c.x - self.last_goal[0], c.y - self.last_goal[1])
			if d < nearest_dist:
				nearest = c
				nearest_dist = d
		return nearest

	def _publish_goal(self, x: float, y: float) -> None:
		msg = PoseStamped()
		msg.header.stamp = self.get_clock().now().to_msg()
		msg.header.frame_id = self.goal_frame
		msg.pose.position = Point(x=x, y=y, z=0.0)
		msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
		self.goal_pub.publish(msg)


def main(args: Optional[Sequence[str]] = None) -> None:
	rclpy.init(args=args)
	node = DecisionMaker()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
