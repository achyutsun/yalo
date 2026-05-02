#!/usr/bin/env python3
"""Decision maker that directly reuses entropy_explorer frontier logic."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from yalo.entropy_explorer import OccupancyGridManager, detect_frontiers, score_frontiers


@dataclass
class FrontierCandidate:
    """Scored frontier candidate derived from entropy_explorer outputs."""

    x: float
    y: float
    entropy_norm: float
    motion_cost_norm: float
    dist_robot_norm: float
    dist_prev_norm: float
    score: float


class DecisionMaker(Node):
    """Choose a goal frontier using entropy and consistency weighting."""

    def __init__(self) -> None:
        super().__init__('decision_maker')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('decision_period', 1.0)

        self.declare_parameter('entropy_weight', 1.0)
        self.declare_parameter('motion_cost_weight', 0.55)
        self.declare_parameter('consistency_weight', 0.45)
        self.declare_parameter('switch_margin', 0.15)
        self.declare_parameter('same_goal_tolerance', 0.20)
        self.declare_parameter('base_drain', 0.05)
        self.declare_parameter('linear_cost_weight', 1.0)
        self.declare_parameter('angular_cost_weight', 0.6)
        self.declare_parameter('startup_tax', 0.35)
        self.declare_parameter('start_velocity_threshold', 0.05)
        self.declare_parameter('stop_distance_threshold', 0.25)

        self.map_topic = self.get_parameter('map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.decision_period = float(self.get_parameter('decision_period').value)

        self.entropy_weight = float(self.get_parameter('entropy_weight').value)
        self.motion_cost_weight = float(self.get_parameter('motion_cost_weight').value)
        self.consistency_weight = float(
            self.get_parameter('consistency_weight').value
        )
        self.switch_margin = float(self.get_parameter('switch_margin').value)
        self.same_goal_tolerance = float(
            self.get_parameter('same_goal_tolerance').value
        )
        self.base_drain = float(self.get_parameter('base_drain').value)
        self.linear_cost_weight = float(self.get_parameter('linear_cost_weight').value)
        self.angular_cost_weight = float(self.get_parameter('angular_cost_weight').value)
        self.startup_tax = float(self.get_parameter('startup_tax').value)
        self.start_velocity_threshold = float(
            self.get_parameter('start_velocity_threshold').value
        )
        self.stop_distance_threshold = float(
            self.get_parameter('stop_distance_threshold').value
        )

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self._map_cb,
            map_qos,
        )
        self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_cb,
            10,
        )

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.timer = self.create_timer(self.decision_period, self._timer_cb)

        self.ogm: Optional[OccupancyGridManager] = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_linear_speed = 0.0
        self.robot_angular_speed = 0.0
        self.have_odom = False

        self.last_goal: Optional[Tuple[float, float]] = None
        self.last_goal_score: Optional[float] = None

        self.get_logger().info('decision_maker started (entropy_explorer-based)')

    def _map_cb(self, msg: OccupancyGrid) -> None:
        self.ogm = OccupancyGridManager(msg)

    def _odom_cb(self, msg: Odometry) -> None:
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.robot_linear_speed = math.hypot(
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
        )
        self.robot_angular_speed = abs(msg.twist.twist.angular.z)
        self.have_odom = True

    def _timer_cb(self) -> None:
        if self.ogm is None or not self.have_odom:
            return

        frontiers = detect_frontiers(self.ogm)
        if not frontiers:
            return

        frontiers = score_frontiers(frontiers, self.ogm, self.robot_x, self.robot_y)

        candidates = self._score_candidates(frontiers)
        if not candidates:
            return

        best = max(candidates, key=lambda c: c.score)
        chosen = self._apply_hysteresis(candidates, best)

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
            'goal=('
            f'{chosen.x:.2f}, {chosen.y:.2f}) '
            f'entropy_norm={chosen.entropy_norm:.3f} '
            f'motion_norm={chosen.motion_cost_norm:.3f} '
            f'dr={chosen.dist_robot_norm:.3f} '
            f'dp={chosen.dist_prev_norm:.3f} '
            f'score={chosen.score:.3f}'
        )

    def _score_candidates(self, frontiers) -> List[FrontierCandidate]:
        raw = []
        for fr in frontiers:
            entropy_score = float(fr.utility)
            dist_robot = math.hypot(
                fr.centroid_x - self.robot_x,
                fr.centroid_y - self.robot_y,
            )
            heading_to_goal = math.atan2(
                fr.centroid_y - self.robot_y,
                fr.centroid_x - self.robot_x,
            )
            heading_error = abs(self._wrap_angle(heading_to_goal - self.robot_yaw))
            motion_cost = (
                self.base_drain
                + self.linear_cost_weight * dist_robot
                + self.angular_cost_weight * heading_error
            )
            if (
                self.robot_linear_speed < self.start_velocity_threshold
                and dist_robot > self.stop_distance_threshold
            ):
                motion_cost += self.startup_tax
            dist_prev = 0.0
            if self.last_goal is not None:
                dist_prev = math.hypot(
                    fr.centroid_x - self.last_goal[0],
                    fr.centroid_y - self.last_goal[1],
                )
            raw.append(
                (
                    fr.centroid_x,
                    fr.centroid_y,
                    entropy_score,
                    motion_cost,
                    dist_robot,
                    dist_prev,
                )
            )

        max_entropy = max(item[2] for item in raw) or 1.0
        max_motion = max(item[3] for item in raw) or 1.0
        max_dr = max(item[4] for item in raw) or 1.0
        max_dp = max(item[5] for item in raw) or 1.0

        candidates: List[FrontierCandidate] = []
        for x, y, ent, motion, dr, dp in raw:
            ent_n = ent / max_entropy
            motion_n = motion / max_motion
            dr_n = dr / max_dr
            dp_n = dp / max_dp if self.last_goal is not None else 0.0
            score = (
                self.entropy_weight * ent_n
                - self.motion_cost_weight * motion_n
                - self.consistency_weight * dp_n
            )
            candidates.append(
                FrontierCandidate(
                    x=x,
                    y=y,
                    entropy_norm=ent_n,
                    motion_cost_norm=motion_n,
                    dist_robot_norm=dr_n,
                    dist_prev_norm=dp_n,
                    score=score,
                )
            )

        return candidates

    def _apply_hysteresis(
        self,
        candidates: Sequence[FrontierCandidate],
        best: FrontierCandidate,
    ) -> FrontierCandidate:
        if self.last_goal is None:
            return best

        previous = min(
            candidates,
            key=lambda c: math.hypot(c.x - self.last_goal[0], c.y - self.last_goal[1]),
        )
        if best.score - previous.score <= self.switch_margin:
            return previous
        return best

    def _publish_goal(self, x: float, y: float) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.goal_frame
        msg.pose.position = Point(x=x, y=y, z=0.0)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.goal_pub.publish(msg)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


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
