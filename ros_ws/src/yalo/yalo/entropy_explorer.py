#!/usr/bin/env python3
"""
=============================================================================
  TurtleBot4 Entropy-Based Frontier Exploration — ROS 2 Node
=============================================================================
  Algorithm:  Entropy-maximising Frontier Exploration
  Platform:   TurtleBot 4 (Raspberry Pi 4B, RPLIDAR-A1, iRobot Create3)
  ROS 2:      Humble / Iron / Jazzy

  Mathematical Foundation
  -----------------------
  Shannon entropy of a single occupancy-grid cell (probability p ∈ [0,1]):

      H(cell) = -p·log₂(p) - (1-p)·log₂(1-p)

  H is maximised at p = 0.5 (unknown) and zero at p = 0 or p = 1.

  Information Gain for frontier f with candidate viewpoint v:
  ───────────────────────────────────────────────────────────
  We approximate the expected information gain by summing cell entropy
  within a sensor-range disc Ω(v) centred on viewpoint v:

      IG(f) = Σ_{c ∈ Ω(v)}  H(c)           (raw entropy integration)

  To prefer frontiers that are reachable and close, we apply a
  distance-weighted penalty (Elfes-style utility):

      U(f) = IG(f) / (1 + λ · d(robot, f))

  where λ is a tunable decay factor and d is Euclidean path cost.

  Frontier Detection  (WFD — Wavefront Frontier Detector)
  ────────────────────────────────────────────────────────
  A cell is a *frontier cell* iff:
      1. It is FREE  (log-odds < FREE_THRESH)
      2. At least one 8-connected neighbour is UNKNOWN (log-odds ≈ 0.5)

  Connected frontier cells are grouped into *frontier regions* via BFS.
  Small clusters (< MIN_FRONTIER_SIZE cells) are discarded.

  Occupancy Grid Encoding (ROS nav_msgs/OccupancyGrid)
  ─────────────────────────────────────────────────────
      -1   → Unknown   → p ≈ 0.50  → H ≈ 1.00 bit  (max entropy)
       0   → Free      → p ≈ 0.05  → H ≈ 0.29 bit
     100   → Occupied  → p ≈ 0.95  → H ≈ 0.29 bit

  ROS 2 Interface
  ───────────────
  Subscriptions:
      /map                     nav_msgs/OccupancyGrid
      /odom                    nav_msgs/Odometry
  Publications / Actions:
      /goal_pose               geometry_msgs/PoseStamped  (Nav2 SimpleCommander)
  Services used:
      /is_path_valid            (optional costmap check)

  Dependencies
  ────────────
      sudo apt install ros-$ROS_DISTRO-nav2-msgs \
                       ros-$ROS_DISTRO-nav2-simple-commander \
                       python3-scipy python3-numpy python3-sklearn

=============================================================================
"""

import math
import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (
    QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
)

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

# Nav2 simple commander (comes with nav2_simple_commander package)
try:
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    HAS_NAV2_SIMPLE = True
except ImportError:
    HAS_NAV2_SIMPLE = False

# ─── Occupancy value thresholds ───────────────────────────────────────────────
UNKNOWN_VAL     = -1
FREE_MAX        =  25    # OccupancyGrid values  0 .. FREE_MAX  → free
OCC_MIN         =  65    # OccupancyGrid values  OCC_MIN..100   → occupied
# cells between FREE_MAX and OCC_MIN are treated as "inflated / uncertain"

# ─── Entropy helper constants ─────────────────────────────────────────────────
LOG2 = math.log(2.0)
EPS  = 1e-9          # avoid log(0)

# ─── RPLIDAR-A1 effective mapping radius on TurtleBot4 ────────────────────────
SENSOR_RANGE_M  = 6.0   # metres  (RPLIDAR-A1 spec: up to 12 m; conservative)

# ─── Exploration tuning parameters ───────────────────────────────────────────
MIN_FRONTIER_SIZE   = 5      # cells — discard tiny frontier clusters
LAMBDA_DECAY        = 0.35   # distance penalty weight in utility function
GOAL_REACHED_DIST   = 0.35   # metres — consider goal reached within this radius
REPLANNING_PERIOD   = 4.0    # seconds — re-evaluate frontiers even if moving
STUCK_TIMEOUT       = 25.0   # seconds — abort goal if robot hasn't moved
STUCK_DIST_THRESH   = 0.10   # metres  — movement threshold for stuck detection


# ══════════════════════════════════════════════════════════════════════════════
#   ENTROPY MATHEMATICS MODULE
# ══════════════════════════════════════════════════════════════════════════════

def occupancy_to_probability(occ_val: int) -> float:
    """
    Convert ROS OccupancyGrid integer value to probability of occupancy.

    ROS convention:
        -1  → unknown  → treat as 0.5 (maximum uncertainty / entropy)
         0  → free     → map to low probability  ≈ 0.05
        100 → occupied → map to high probability ≈ 0.95
    """
    if occ_val == UNKNOWN_VAL:
        return 0.50
    # Linear mapping [0, 100] → [0.05, 0.95]  (clipped)
    p = 0.05 + (occ_val / 100.0) * 0.90
    return float(np.clip(p, EPS, 1.0 - EPS))


def binary_entropy(p: float) -> float:
    """
    Shannon binary entropy in bits.

        H(p) = -p·log₂(p) - (1-p)·log₂(1-p)

    Returns value in [0, 1] (bits).
    """
    p = float(np.clip(p, EPS, 1.0 - EPS))
    return -(p * math.log(p) + (1.0 - p) * math.log(1.0 - p)) / LOG2


def build_entropy_map(grid_data: np.ndarray) -> np.ndarray:
    """
    Vectorised conversion of an entire OccupancyGrid data array → entropy map.

    Parameters
    ----------
    grid_data : np.ndarray, dtype int8, shape (height, width)
        Raw occupancy values from nav_msgs/OccupancyGrid.

    Returns
    -------
    entropy_map : np.ndarray, dtype float32, shape (height, width)
        Per-cell Shannon entropy H(cell) in bits ∈ [0, 1].
    """
    data = grid_data.astype(np.float32)

    # Unknown cells (-1) → probability 0.5 → entropy 1.0
    unknown_mask = (grid_data == UNKNOWN_VAL)

    # Map known cells [0, 100] → probability [0.05, 0.95]
    p = np.where(unknown_mask, 0.5, np.clip(0.05 + data / 100.0 * 0.90, EPS, 1.0 - EPS))

    # H(p) = -(p log p + (1-p) log(1-p)) / log(2)
    entropy_map = -(p * np.log(p + EPS) + (1.0 - p) * np.log(1.0 - p + EPS)) / LOG2
    entropy_map = np.where(unknown_mask, 1.0, entropy_map)

    return entropy_map.astype(np.float32)


# ══════════════════════════════════════════════════════════════════════════════
#   OCCUPANCY GRID MANAGEMENT
# ══════════════════════════════════════════════════════════════════════════════

class OccupancyGridManager:
    """
    Wraps a ROS nav_msgs/OccupancyGrid message and provides spatial queries.

    Coordinate Systems
    ──────────────────
    • *Map frame*  : ROS world coordinates (metres), origin at map.info.origin
    • *Grid frame* : Integer (col, row) cell indices

    The conversion between frames:
        col = int((x_world - origin_x) / resolution)
        row = int((y_world - origin_y) / resolution)
    """

    def __init__(self, map_msg: OccupancyGrid):
        info = map_msg.info
        self.resolution : float = info.resolution          # metres/cell
        self.width      : int   = info.width               # columns
        self.height     : int   = info.height              # rows
        self.origin_x   : float = info.origin.position.x  # world x of (0,0) cell
        self.origin_y   : float = info.origin.position.y  # world y of (0,0) cell
        self.frame_id   : str   = map_msg.header.frame_id

        # 2-D grid array: shape (height, width), dtype int8
        raw = np.array(map_msg.data, dtype=np.int8)
        self.grid : np.ndarray = raw.reshape((self.height, self.width))

        # Pre-build entropy map (reused by information-gain queries)
        self.entropy_map : np.ndarray = build_entropy_map(self.grid)

        # Sensor footprint radius in cells (used for IG integration disc)
        self.sensor_cells : int = max(
            1, int(math.ceil(SENSOR_RANGE_M / self.resolution))
        )

    # ── Coordinate transforms ──────────────────────────────────────────────

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return col, row

    def grid_to_world(self, col: int, row: int) -> Tuple[float, float]:
        x = self.origin_x + (col + 0.5) * self.resolution
        y = self.origin_y + (row + 0.5) * self.resolution
        return x, y

    def in_bounds(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    def get_cell(self, col: int, row: int) -> int:
        return int(self.grid[row, col])

    # ── Cell classification ────────────────────────────────────────────────

    def is_free(self, col: int, row: int) -> bool:
        return 0 <= self.get_cell(col, row) <= FREE_MAX

    def is_unknown(self, col: int, row: int) -> bool:
        return self.get_cell(col, row) == UNKNOWN_VAL

    def is_occupied(self, col: int, row: int) -> bool:
        return self.get_cell(col, row) >= OCC_MIN

    # ── Information-gain integration ──────────────────────────────────────

    def information_gain_at(self, col: int, row: int) -> float:
        """
        Approximate information gain visible from cell (col, row).

        Integrates entropy over a disc of radius SENSOR_RANGE_M around
        the viewpoint, excluding cells that are already known (entropy ≈ 0).

            IG(v) = Σ_{c ∈ Disc(v, R)}  H(c)

        Uses NumPy slice + circular mask for efficiency — O(R²) not O(W·H).
        """
        r = self.sensor_cells
        r0, r1 = max(0, row - r), min(self.height, row + r + 1)
        c0, c1 = max(0, col - r), min(self.width,  col + r + 1)

        patch = self.entropy_map[r0:r1, c0:c1]

        # Build circular mask relative to (col, row)
        rows_idx = np.arange(r0, r1) - row
        cols_idx = np.arange(c0, c1) - col
        CC, RR   = np.meshgrid(cols_idx, rows_idx)
        disc     = (RR ** 2 + CC ** 2) <= (r ** 2)

        return float(np.sum(patch[disc]))


# ══════════════════════════════════════════════════════════════════════════════
#   FRONTIER DETECTION  (Wavefront Frontier Detector — WFD)
# ══════════════════════════════════════════════════════════════════════════════

class Frontier:
    """Represents a connected region of frontier cells with aggregated metrics."""

    __slots__ = ['cells', 'centroid_col', 'centroid_row',
                 'centroid_x', 'centroid_y',
                 'information_gain', 'utility', 'size']

    def __init__(self):
        self.cells         : List[Tuple[int, int]] = []
        self.centroid_col  : int   = 0
        self.centroid_row  : int   = 0
        self.centroid_x    : float = 0.0
        self.centroid_y    : float = 0.0
        self.information_gain : float = 0.0
        self.utility       : float = 0.0
        self.size          : int   = 0


def detect_frontiers(ogm: OccupancyGridManager) -> List[Frontier]:
    """
    Wavefront Frontier Detector (WFD).

    Step 1 — Identify frontier cells:
        A cell is frontier if it is FREE and has ≥1 UNKNOWN 8-neighbour.

    Step 2 — Cluster adjacent frontier cells via BFS into frontier regions.

    Step 3 — Discard regions smaller than MIN_FRONTIER_SIZE.

    Parameters
    ----------
    ogm : OccupancyGridManager

    Returns
    -------
    frontiers : List[Frontier]
        Unsorted list of detected frontier regions.
    """
    grid   = ogm.grid
    height = ogm.height
    width  = ogm.width

    # ── Step 1: fast vectorised frontier-cell mask ─────────────────────────
    # A cell is free?
    free_mask    = (grid >= 0) & (grid <= FREE_MAX)
    unknown_mask = (grid == UNKNOWN_VAL)

    # 8-connected shift: detect free cells adjacent to unknown cells.
    # We shift the unknown mask in all 8 directions and OR the results.
    def shift(arr, dr, dc):
        out = np.zeros_like(arr)
        rs = slice(max(0, dr), height + min(0, dr))
        cs = slice(max(0, dc), width  + min(0, dc))
        rs2 = slice(max(0, -dr), height + min(0, -dr))
        cs2 = slice(max(0, -dc), width  + min(0, -dc))
        out[rs2, cs2] = arr[rs, cs]
        return out

    adj_unknown = np.zeros((height, width), dtype=bool)
    for dr in [-1, 0, 1]:
        for dc in [-1, 0, 1]:
            if dr == 0 and dc == 0:
                continue
            adj_unknown |= shift(unknown_mask, dr, dc)

    frontier_mask = free_mask & adj_unknown

    # ── Step 2: BFS clustering ─────────────────────────────────────────────
    visited  = np.zeros((height, width), dtype=bool)
    frontiers: List[Frontier] = []

    frontier_coords = list(zip(*np.where(frontier_mask)))  # (row, col) pairs

    for (row0, col0) in frontier_coords:
        if visited[row0, col0]:
            continue

        # BFS
        region: List[Tuple[int, int]] = []
        queue  = [(col0, row0)]
        visited[row0, col0] = True

        while queue:
            col, row = queue.pop()
            if frontier_mask[row, col]:
                region.append((col, row))

            for dr in [-1, 0, 1]:
                for dc in [-1, 0, 1]:
                    if dr == 0 and dc == 0:
                        continue
                    nr, nc = row + dr, col + dc
                    if (0 <= nr < height and 0 <= nc < width
                            and not visited[nr, nc]
                            and frontier_mask[nr, nc]):
                        visited[nr, nc] = True
                        queue.append((nc, nr))

        # ── Step 3: size filter ────────────────────────────────────────────
        if len(region) < MIN_FRONTIER_SIZE:
            continue

        f = Frontier()
        f.cells = region
        f.size  = len(region)

        cols = [c for c, _ in region]
        rows = [r for _, r in region]
        f.centroid_col = int(np.mean(cols))
        f.centroid_row = int(np.mean(rows))
        f.centroid_x, f.centroid_y = ogm.grid_to_world(
            f.centroid_col, f.centroid_row
        )
        frontiers.append(f)

    return frontiers


# ══════════════════════════════════════════════════════════════════════════════
#   ENTROPY EXPLORATION SELECTOR
# ══════════════════════════════════════════════════════════════════════════════

def score_frontiers(
    frontiers : List[Frontier],
    ogm       : OccupancyGridManager,
    robot_x   : float,
    robot_y   : float,
) -> List[Frontier]:
    """
    Compute information gain and distance-weighted utility for each frontier.

    Information Gain
    ────────────────
        IG(f) = Σ_{c ∈ Disc(centroid(f), R_sensor)} H(c)

    This is the total entropy visible from the frontier's centroid,
    approximating the knowledge gained by visiting that location.

    Utility (Entropy / Distance Trade-off)
    ───────────────────────────────────────
        U(f) = IG(f) / (1 + λ · d(robot, centroid(f)))

    The denominator penalises far-away frontiers, balancing exploration
    breadth (high IG) against travel cost (low d).

    Parameters
    ----------
    frontiers  : list of Frontier objects (modified in-place)
    ogm        : OccupancyGridManager
    robot_x, robot_y : current robot position in world frame

    Returns
    -------
    frontiers  : same list, sorted by descending utility
    """
    for f in frontiers:
        # Information gain at frontier centroid viewpoint
        f.information_gain = ogm.information_gain_at(
            f.centroid_col, f.centroid_row
        )

        # Euclidean distance robot → frontier centroid
        dx = f.centroid_x - robot_x
        dy = f.centroid_y - robot_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Distance-weighted utility
        f.utility = f.information_gain / (1.0 + LAMBDA_DECAY * dist)

    # Descending utility order → best frontier first
    frontiers.sort(key=lambda fr: fr.utility, reverse=True)
    return frontiers


# ══════════════════════════════════════════════════════════════════════════════
#   ROS 2 ENTROPY EXPLORER NODE
# ══════════════════════════════════════════════════════════════════════════════

class EntropyExplorerNode(Node):
    """
    ROS 2 node implementing entropy-based frontier exploration for TurtleBot4.

    State machine
    ─────────────
    IDLE → PLANNING → NAVIGATING → PLANNING → ...
                  ↓ (no frontiers)
               COMPLETE

    Topics / Interfaces
    ───────────────────
    Sub:  /map          nav_msgs/OccupancyGrid  (SLAM map, latched)
    Sub:  /odom         nav_msgs/Odometry       (robot pose estimate)
    Pub:  /goal_pose    geometry_msgs/PoseStamped  (Nav2 goal)
    Pub:  /entropy_frontiers   visualization_msgs/MarkerArray  (RViz)
    """

    # ── Node lifecycle ─────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('entropy_explorer')

        # ── Parameters (can be overridden via ros2 run ... --ros-args -p k:=v)
        self.declare_parameter('min_frontier_size',  MIN_FRONTIER_SIZE)
        self.declare_parameter('sensor_range_m',     SENSOR_RANGE_M)
        self.declare_parameter('lambda_decay',       LAMBDA_DECAY)
        self.declare_parameter('goal_reached_dist',  GOAL_REACHED_DIST)
        self.declare_parameter('replanning_period',  REPLANNING_PERIOD)
        self.declare_parameter('stuck_timeout',      STUCK_TIMEOUT)
        self.declare_parameter('use_nav2_simple',    HAS_NAV2_SIMPLE)

        # ── QoS for map topic (latched — TRANSIENT_LOCAL)
        map_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 1
        )

        # ── Subscriptions
        self.map_sub  = self.create_subscription(
            OccupancyGrid, '/map', self._map_callback, map_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback, 10
        )

        # ── Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, '/entropy_frontiers', 10
        )

        # ── State
        self.ogm           : Optional[OccupancyGridManager] = None
        self.robot_x       : float = 0.0
        self.robot_y       : float = 0.0
        self.robot_yaw     : float = 0.0

        self.current_goal  : Optional[Tuple[float, float]] = None
        self.goal_sent_at  : float = 0.0
        self.last_move_time: float = time.time()
        self.last_robot_x  : float = 0.0
        self.last_robot_y  : float = 0.0

        self.state         : str   = 'IDLE'
        self.map_updated   : bool  = False
        self.odom_received : bool  = False

        # ── Nav2 SimpleCommander (optional but recommended)
        self.navigator : Optional[object] = None
        if self.get_parameter('use_nav2_simple').value and HAS_NAV2_SIMPLE:
            self.navigator = BasicNavigator()
            self.get_logger().info('Nav2 BasicNavigator initialised.')

        # ── Main planning timer
        period = self.get_parameter('replanning_period').value
        self.timer = self.create_timer(period, self._exploration_loop)

        self.get_logger().info(
            '╔══════════════════════════════════════════════╗\n'
            '║   TurtleBot4 Entropy Explorer — STARTED      ║\n'
            '╚══════════════════════════════════════════════╝'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _map_callback(self, msg: OccupancyGrid):
        self.ogm         = OccupancyGridManager(msg)
        self.map_updated = True
        self.get_logger().debug(
            f'Map received: {self.ogm.width}×{self.ogm.height} cells, '
            f'res={self.ogm.resolution:.3f} m/cell'
        )

    def _odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_x = p.x
        self.robot_y = p.y
        # Extract yaw from quaternion
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)
        self.odom_received = True

    # ── Main exploration loop ─────────────────────────────────────────────

    def _exploration_loop(self):
        """Timer callback — runs the full entropy exploration cycle."""

        if not self.map_updated or not self.odom_received:
            self.get_logger().info('Waiting for /map and /odom …', once=True)
            return

        # ── Check if stuck ────────────────────────────────────────────────
        if self.state == 'NAVIGATING':
            self._check_stuck()

        # ── Check if goal reached ─────────────────────────────────────────
        if self.current_goal is not None and self.state == 'NAVIGATING':
            gx, gy = self.current_goal
            dist_to_goal = math.hypot(
                self.robot_x - gx, self.robot_y - gy
            )
            reached_thr = self.get_parameter('goal_reached_dist').value
            if dist_to_goal < reached_thr:
                self.get_logger().info(
                    f'✓ Goal reached ({dist_to_goal:.2f} m). Re-planning …'
                )
                self.current_goal = None
                self.state = 'PLANNING'

        # ── Planning step ─────────────────────────────────────────────────
        if self.state in ('IDLE', 'PLANNING'):
            self._plan_next_goal()

    # ── Planning ──────────────────────────────────────────────────────────

    def _plan_next_goal(self):
        """
        Core entropy planning step:
        1. Detect frontiers (WFD)
        2. Score by information gain + distance utility
        3. Send best frontier to Nav2
        """
        self.state = 'PLANNING'
        ogm = self.ogm

        # ── 1. Frontier detection ─────────────────────────────────────────
        t0 = time.time()
        frontiers = detect_frontiers(ogm)
        t_detect  = time.time() - t0

        if not frontiers:
            self.get_logger().info(
                '✓ Exploration COMPLETE — no frontiers remaining.'
            )
            self.state = 'COMPLETE'
            return

        # ── 2. Score frontiers by entropy ─────────────────────────────────
        t1 = time.time()
        frontiers = score_frontiers(
            frontiers, ogm, self.robot_x, self.robot_y
        )
        t_score = time.time() - t1

        best = frontiers[0]

        self.get_logger().info(
            f'Frontiers detected: {len(frontiers)}  '
            f'(detect={t_detect*1e3:.1f}ms, score={t_score*1e3:.1f}ms)\n'
            f'  Best frontier → ({best.centroid_x:.2f}, {best.centroid_y:.2f})  '
            f'IG={best.information_gain:.2f} bits  '
            f'U={best.utility:.4f}  '
            f'size={best.size} cells'
        )

        # Log entropy stats
        total_entropy = float(np.sum(ogm.entropy_map))
        mean_entropy  = float(np.mean(ogm.entropy_map))
        self.get_logger().info(
            f'  Map entropy — total={total_entropy:.1f} bits  '
            f'mean={mean_entropy:.4f} bits/cell'
        )

        # ── 3. Visualise in RViz ──────────────────────────────────────────
        self._publish_frontier_markers(frontiers)

        # ── 4. Send navigation goal ───────────────────────────────────────
        self._send_goal(best.centroid_x, best.centroid_y)
        self.current_goal = (best.centroid_x, best.centroid_y)
        self.state = 'NAVIGATING'

    # ── Navigation ────────────────────────────────────────────────────────

    def _send_goal(self, x: float, y: float):
        """Publish goal to Nav2 via /goal_pose or BasicNavigator."""
        yaw_to_goal = math.atan2(y - self.robot_y, x - self.robot_x)

        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position   = Point(x=x, y=y, z=0.0)
        # Quaternion from yaw
        pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw_to_goal / 2.0),
            w=math.cos(yaw_to_goal / 2.0)
        )

        if self.navigator is not None:
            self.navigator.goToPose(pose)
            self.get_logger().info(
                f'→ Nav2 goal sent: ({x:.2f}, {y:.2f})'
            )
        else:
            self.goal_pub.publish(pose)
            self.get_logger().info(
                f'→ /goal_pose published: ({x:.2f}, {y:.2f})'
            )

        self.goal_sent_at   = time.time()
        self.last_move_time = time.time()
        self.last_robot_x   = self.robot_x
        self.last_robot_y   = self.robot_y

    def _check_stuck(self):
        """Abort and re-plan if robot hasn't moved within STUCK_TIMEOUT."""
        moved = math.hypot(
            self.robot_x - self.last_robot_x,
            self.robot_y - self.last_robot_y
        )
        stuck_thresh = self.get_parameter('stuck_timeout').value
        if moved > STUCK_DIST_THRESH:
            self.last_move_time = time.time()
            self.last_robot_x   = self.robot_x
            self.last_robot_y   = self.robot_y
        elif time.time() - self.last_move_time > stuck_thresh:
            self.get_logger().warn(
                '⚠ Robot stuck! Cancelling goal and re-planning …'
            )
            if self.navigator is not None:
                self.navigator.cancelTask()
            self.current_goal = None
            self.state = 'PLANNING'

    # ── RViz visualisation ────────────────────────────────────────────────

    def _publish_frontier_markers(self, frontiers: List[Frontier]):
        """
        Publish MarkerArray for RViz:
        • SPHERE markers at each frontier centroid, coloured by utility
        • Best frontier is bright green, others are cyan → blue gradient
        """
        arr = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Clear old markers
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        arr.markers.append(del_marker)

        max_utility = frontiers[0].utility if frontiers else 1.0

        for idx, f in enumerate(frontiers):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp    = now
            m.ns              = 'entropy_frontiers'
            m.id              = idx
            m.type            = Marker.SPHERE
            m.action          = Marker.ADD

            m.pose.position.x = f.centroid_x
            m.pose.position.y = f.centroid_y
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0

            # Scale proportional to frontier size
            s = max(0.10, min(0.5, f.size * self.ogm.resolution * 0.5))
            m.scale.x = m.scale.y = m.scale.z = s

            # Colour: best → green, others → blue gradient
            norm = f.utility / (max_utility + 1e-9)
            if idx == 0:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 0.9
            else:
                m.color.r = 0.0
                m.color.g = norm * 0.5
                m.color.b = 1.0
                m.color.a = 0.7

            arr.markers.append(m)

            # Text label: utility value
            t = Marker()
            t.header    = m.header
            t.ns        = 'entropy_labels'
            t.id        = 1000 + idx
            t.type      = Marker.TEXT_VIEW_FACING
            t.action    = Marker.ADD
            t.pose.position.x = f.centroid_x
            t.pose.position.y = f.centroid_y
            t.pose.position.z = 0.4
            t.pose.orientation.w = 1.0
            t.scale.z   = 0.12
            t.color.r = t.color.g = t.color.b = t.color.a = 1.0
            t.text      = f'U={f.utility:.3f}\nIG={f.information_gain:.1f}'
            arr.markers.append(t)

        self.marker_pub.publish(arr)


# ══════════════════════════════════════════════════════════════════════════════
#   ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = EntropyExplorerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Entropy Explorer …')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
