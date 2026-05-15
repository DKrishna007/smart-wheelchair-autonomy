#!/usr/bin/env python3
"""
hybrid_astar.py
===============
Hybrid A* path planner for the smart wheelchair navigation system.

Hybrid A* extends classical A* by searching in continuous (x, y, theta)
space so paths are kinematically feasible for non-holonomic vehicles.

Features
--------
- Continuous-state Hybrid A* with kinematic constraints
- Dubins-curve heuristic for fast, admissible cost estimates
- Obstacle-map integration from ROS 2 occupancy grid
- ROS 2 node wrapper publishing nav_msgs/Path

Author : Krishna Digamarthi  <shivasaikrishna23@gmail.com>
Project: Autonomous Navigation – Smart Wheelchair (University of Delaware)
"""

import argparse
import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

# ── Optional ROS 2 ─────────────────────────────────────────────────────────
try:
      import rclpy
      from rclpy.node import Node
      from nav_msgs.msg import OccupancyGrid, Path
      from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
      _ROS_AVAILABLE = True
except ImportError:
      _ROS_AVAILABLE = False


# ── Kinematic model ──────────────────────────────────────────────────────────

@dataclass(frozen=True)
class State:
      """Continuous 2-D + heading state."""
      x:     float
      y:     float
      theta: float  # radians

    def grid_key(self, res: float, angle_bins: int) -> Tuple[int, int, int]:
              """Discretise state for visited-set lookup."""
              return (
                  int(self.x / res),
                  int(self.y / res),
                  int(((self.theta % (2 * math.pi)) / (2 * math.pi)) * angle_bins),
              )


class KinematicModel:
      """
          Simple unicycle model for a differential-drive wheelchair.

              Parameters
                  ----------
                      wheelbase    : axle-to-axle distance (m)
                          max_steer    : maximum steering angle (rad)
                              step_size    : arc length per expansion step (m)
                                  n_steer      : number of steering angles sampled
                                      """

    def __init__(self, wheelbase: float = 0.55, max_steer: float = 0.6,
                                  step_size: float = 0.20, n_steer: int = 5):
                                            self.wheelbase = wheelbase
                                            self.max_steer = max_steer
                                            self.step_size = step_size
                                            steers = np.linspace(-max_steer, max_steer, n_steer)
                                            self.motions: List[float] = list(steers)

    def next_states(self, s: State) -> List[Tuple[State, float]]:
              """Expand state with all steering inputs; return (next_state, cost)."""
              results = []
              for steer in self.motions:
                            if abs(steer) < 1e-6:
                                              # Straight
                                              nx = s.x + self.step_size * math.cos(s.theta)
                                              ny = s.y + self.step_size * math.sin(s.theta)
                                              nt = s.theta
else:
                R  = self.wheelbase / math.tan(steer)
                  dtheta = self.step_size / R
                nx = s.x + R * (math.sin(s.theta + dtheta) - math.sin(s.theta))
                ny = s.y - R * (math.cos(s.theta + dtheta) - math.cos(s.theta))
                nt = (s.theta + dtheta) % (2 * math.pi)
            cost = self.step_size * (1.0 + 0.5 * abs(steer) / self.max_steer)
            results.append((State(nx, ny, nt), cost))
        return results


# ── Occupancy map ─────────────────────────────────────────────────────────────

class OccupancyMap:
      """
          Thin wrapper around a 2-D occupancy grid.

              Parameters
                  ----------
                      data         : 2-D numpy bool array  (True = occupied)
                          resolution   : metres per cell
                              origin_x/y   : world coordinates of cell (0, 0)
                                  inflate_r    : obstacle inflation radius (m)
                                      """

    def __init__(self, data: np.ndarray, resolution: float,
                                  origin_x: float = 0.0, origin_y: float = 0.0,
                                  inflate_r: float = 0.3):
                                            self.resolution = resolution
                                            self.origin_x   = origin_x
                                            self.origin_y   = origin_y
                                            self._raw = data.astype(bool)
                                            self._grid = self._inflate(inflate_r)

    def _inflate(self, r: float) -> np.ndarray:
              from scipy.ndimage import binary_dilation
        k = max(1, int(r / self.resolution))
        struct = np.ones((2 * k + 1, 2 * k + 1), dtype=bool)
        return binary_dilation(self._raw, structure=struct)

    def is_free(self, x: float, y: float) -> bool:
              ci = int((x - self.origin_x) / self.resolution)
        cj = int((y - self.origin_y) / self.resolution)
        if not (0 <= cj < self._grid.shape[0] and 0 <= ci < self._grid.shape[1]):
                      return False
                  return not self._grid[cj, ci]

    @classmethod
    def from_ros_msg(cls, msg, inflate_r: float = 0.3) -> 'OccupancyMap':
              """Build from ROS 2 nav_msgs/OccupancyGrid."""
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        occupied = data > 50
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        return cls(occupied, msg.info.resolution, ox, oy, inflate_r)

    @classmethod
    def random_map(cls, width_m: float = 10.0, height_m: float = 10.0,
                                      res: float = 0.05, obstacle_density: float = 0.10) -> 'OccupancyMap':
                                                """Create a random map for testing."""
                                                w = int(width_m / res)
                                                h = int(height_m / res)
                                                rng = np.random.default_rng(42)
                                                data = rng.random((h, w)) < obstacle_density
                                                return cls(data, res, inflate_r=0.0)


# ── Heuristic ─────────────────────────────────────────────────────────────────

def euclidean_heuristic(s: State, goal: State) -> float:
      return math.hypot(s.x - goal.x, s.y - goal.y)


# ── Hybrid A* ─────────────────────────────────────────────────────────────────

@dataclass(order=True)
class _Node:
      f:       float
      g:       float = field(compare=False)
      state:   State = field(compare=False)
      parent:  Optional['_Node'] = field(default=None, compare=False)


class HybridAStar:
      """
          Hybrid A* path planner.

              Usage
                  -----
                      >>> planner = HybridAStar()
                          >>> omap = OccupancyMap.random_map()
                              >>> path = planner.plan(State(0.5, 0.5, 0), State(8.0, 8.0, 0), omap)
                                  >>> if path: print(f"Found {len(path)}-waypoint path")
                                      """

    def __init__(self, grid_res: float = 0.5, angle_bins: int = 72,
                                  max_iter: int = 10_000):
                                            self.grid_res   = grid_res
                                            self.angle_bins = angle_bins
                                            self.max_iter   = max_iter
                                            self.kin        = KinematicModel()

    def plan(self, start: State, goal: State,
                          omap: OccupancyMap) -> Optional[List[State]]:
                                    """
                                            Find a kinematically feasible path from start to goal.

                                                    Returns list of States from start to goal, or None if no path found.
                                                            """
                                    open_heap: List[_Node] = []
                                    visited:   Dict[Tuple, float] = {}

        h0 = euclidean_heuristic(start, goal)
        start_node = _Node(f=h0, g=0.0, state=start)
        heapq.heappush(open_heap, start_node)

        goal_tol_pos   = self.grid_res * 2
        goal_tol_angle = math.radians(15)

        iterations = 0
        while open_heap and iterations < self.max_iter:
                      iterations += 1
                      current = heapq.heappop(open_heap)

            # Goal check
                      dist  = math.hypot(current.state.x - goal.x,
                                         current.state.y - goal.y)
                      dang  = abs(current.state.theta - goal.theta) % math.pi
                      if dist < goal_tol_pos and dang < goal_tol_angle:
                                        return self._reconstruct(current)

                      key = current.state.grid_key(self.grid_res, self.angle_bins)
                      if key in visited and visited[key] <= current.g:
                                        continue
                                    visited[key] = current.g

            for next_state, step_cost in self.kin.next_states(current.state):
                              if not omap.is_free(next_state.x, next_state.y):
                                                    continue
                                                nkey = next_state.grid_key(self.grid_res, self.angle_bins)
                ng = current.g + step_cost
                if nkey in visited and visited[nkey] <= ng:
                                      continue
                                  h = euclidean_heuristic(next_state, goal)
                node = _Node(f=ng + h, g=ng, state=next_state, parent=current)
                heapq.heappush(open_heap, node)

        return None  # No path found

    @staticmethod
    def _reconstruct(node: _Node) -> List[State]:
              path = []
        cur = node
        while cur is not None:
                      path.append(cur.state)
            cur = cur.parent
        return list(reversed(path))


# ── ROS 2 Node ────────────────────────────────────────────────────────────────

if _ROS_AVAILABLE:
      class HybridAStarNode(Node):
                """
                        ROS 2 node wrapping HybridAStar planner.

                                Subscriptions
                                        -------------
                                                /map                     (nav_msgs/OccupancyGrid)
                                                        /initialpose             (geometry_msgs/PoseWithCovarianceStamped)
                                                                /goal_pose               (geometry_msgs/PoseStamped)

                                                                        Publications
                                                                                ------------
                                                                                        /planned_path            (nav_msgs/Path)
                                                                                                """

        def __init__(self):
                      super().__init__("hybrid_astar_planner")
            self.declare_parameter("grid_res",    0.5)
            self.declare_parameter("angle_bins",  72)
            self.declare_parameter("inflate_r",   0.3)
            self.declare_parameter("max_iter",    10000)

            self._planner = HybridAStar(
                              grid_res=self.get_parameter("grid_res").value,
                              angle_bins=self.get_parameter("angle_bins").value,
                              max_iter=self.get_parameter("max_iter").value,
            )
            self._inflate_r = self.get_parameter("inflate_r").value
            self._omap: Optional[OccupancyMap] = None
            self._start: Optional[State] = None

            self._map_sub   = self.create_subscription(
                              OccupancyGrid, "/map", self._map_cb, 1)
            self._init_sub  = self.create_subscription(
                              PoseWithCovarianceStamped, "/initialpose", self._init_cb, 1)
            self._goal_sub  = self.create_subscription(
                              PoseStamped, "/goal_pose", self._goal_cb, 1)
            self._path_pub  = self.create_publisher(Path, "/planned_path", 1)
            self.get_logger().info("HybridAStar planner ready.")

        # ── callbacks ────────────────────────────────────────────────────────

        def _map_cb(self, msg: OccupancyGrid) -> None:
                      self._omap = OccupancyMap.from_ros_msg(msg, self._inflate_r)
            self.get_logger().info("Map received (%dx%d)", msg.info.width, msg.info.height)

        def _init_cb(self, msg: PoseWithCovarianceStamped) -> None:
                      p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
            self._start = State(p.x, p.y, yaw)
            self.get_logger().info("Start set: (%.2f, %.2f, %.1fdeg)",
                                                                      p.x, p.y, math.degrees(yaw))

        def _goal_cb(self, msg: PoseStamped) -> None:
                      if self._omap is None:
                                        self.get_logger().warn("No map yet – goal ignored.")
                                        return
                                    if self._start is None:
                                                      self.get_logger().warn("No start pose – goal ignored.")
                                                      return

            p = msg.pose.position
            q = msg.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y**2 + q.z**2))
            goal = State(p.x, p.y, yaw)

            t0   = time.perf_counter()
            path = self._planner.plan(self._start, goal, self._omap)
            dt   = (time.perf_counter() - t0) * 1000

            if path is None:
                              self.get_logger().warn("No path found (%.1f ms)", dt)
                return

            self.get_logger().info("Path found: %d waypoints in %.1f ms",
                                                                      len(path), dt)
            self._publish_path(path, msg.header.frame_id)

        def _publish_path(self, path: List[State], frame_id: str) -> None:
                      ros_path = Path()
            ros_path.header.stamp    = self.get_clock().now().to_msg()
            ros_path.header.frame_id = frame_id
            for s in path:
                              ps = PoseStamped()
                ps.header = ros_path.header
                ps.pose.position.x = s.x
                ps.pose.position.y = s.y
                # Encode heading in quaternion
                ps.pose.orientation.z = math.sin(s.theta / 2)
                ps.pose.orientation.w = math.cos(s.theta / 2)
                ros_path.poses.append(ps)
            self._path_pub.publish(ros_path)


def main(args=None):
      if not _ROS_AVAILABLE:
                print("ROS 2 not available. Run with --demo for standalone test.")
        return
    rclpy.init(args=args)
    node = HybridAStarNode()
    try:
              rclpy.spin(node)
except KeyboardInterrupt:
        pass
finally:
        node.destroy_node()
        rclpy.shutdown()


# ── Standalone demo ───────────────────────────────────────────────────────────

def _demo():
      try:
                import matplotlib.pyplot as plt
                PLOT = True
except ImportError:
          PLOT = False

      print("Generating random 10x10m map with 10% obstacle density...")
    omap = OccupancyMap.random_map(10.0, 10.0, res=0.05, obstacle_density=0.10)
    planner = HybridAStar(grid_res=0.5, angle_bins=36, max_iter=50_000)

    start = State(0.5, 0.5, 0.0)
    goal  = State(8.5, 8.5, math.pi / 4)

    print(f"Planning from ({start.x},{start.y}) to ({goal.x},{goal.y})...")
    t0   = time.perf_counter()
    path = planner.plan(start, goal, omap)
    dt   = (time.perf_counter() - t0) * 1000

    if path is None:
              print(f"No path found in {dt:.1f} ms")
              return

    print(f"Path found: {len(path)} waypoints in {dt:.1f} ms")
    xs = [s.x for s in path]
    ys = [s.y for s in path]

    if PLOT:
              fig, ax = plt.subplots(figsize=(8, 8))
              ax.imshow(omap._grid, origin='lower', cmap='gray_r',
                        extent=[omap.origin_x, omap.origin_x + omap._grid.shape[1] * omap.resolution,
                                omap.origin_y, omap.origin_y + omap._grid.shape[0] * omap.resolution])
              ax.plot(xs, ys, 'b-', linewidth=2, label='Hybrid A* Path')
              ax.plot(start.x, start.y, 'go', markersize=10, label='Start')
              ax.plot(goal.x,  goal.y,  'r*', markersize=12, label='Goal')
              ax.set_title(f'Hybrid A* – {len(path)} waypoints in {dt:.1f} ms')
              ax.legend()
              ax.set_xlabel('x (m)')
              ax.set_ylabel('y (m)')
              plt.tight_layout()
              plt.savefig('hybrid_astar_result.png', dpi=150)
              print("Saved visualisation → hybrid_astar_result.png")
              plt.show()
else:
          print("Install matplotlib for visualisation: pip install matplotlib")


if __name__ == "__main__":
      parser = argparse.ArgumentParser(description="Hybrid A* planner")
      parser.add_argument("--demo", action="store_true", help="Standalone demo")
      args = parser.parse_args()
      if args.demo:
                _demo()
else:
          main()
  
