# Usage examples:
#   1) Known start pose:
#      python3 auto_localize.py --pose 1.25 -0.84 1.57 --wait-amcl
#
#   2) No prior guess (global localization):
#      python3 auto_localize.py --global --wait-amcl
#
# Notes:
#  - Expects frames: map, odom, base_link (or base_footprint).
#  - Make sure your ROS 2 environment is sourced before running.

# ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{
#   header: {frame_id: map},
#   pose: {
#     pose: {
#       position: {x: 21603859457015825, y: -0.21251868540862787, z: 0.0},
#       orientation: {z: 0.0, w: 0.9561487978753969}  # yaw=1.57 → qz=sin(yaw/2), qw=cos(yaw/2)
#     },
#     covariance: [0.0625, 0, 0, 0, 0, 0,
#                  0, 0.0625, 0, 0, 0, 0,
#                  0, 0, 0.01, 0, 0, 0,
#                  0, 0, 0, 0.01, 0, 0,
#                  0, 0, 0, 0, 0.01, 0,
#                  0, 0, 0, 0, 0, 0.0685389]  # (σx,σy=0.25 m)^2; σyaw=(15°)^2 in rad^2
#   }
# }"


# ros2 topic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "{
#   header: {
#     stamp: {sec: 0, nanosec: 0},
#     frame_id: map
#   },
#   pose: {
#     pose: {
#       position: {x: 0.3312491774559021, y: -0.21562612056732178, z: 0.0},
#       orientation: {x: 0.0, y: 0.0, z: -0.3446507811474065, w: 0.9387309726724066}
#     },
#     covariance: [
#       0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
#       0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
#       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#       0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#       0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
#     ]
#   }
# }"

# sudo date -s "$(ssh unitree@192.168.123.18 "date +%Y-%m-%d\ %H:%M:%S")"
# 0.21603859457015825 -0.21251868540862787 0.9561487978753969

import argparse
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped as AmclPoseMsg

def yaw_to_quat(yaw: float):
    """Return (x,y,z,w) quaternion for yaw (Z rotation)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))

class AutoLocalize(Node):
    def __init__(self, args):
        super().__init__('auto_localize')

        self.args = args
        self._map_seen = False
        self._amcl_ready = False

        # QoS: /map is latched (transient local) by map_server
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

        # Pub for /initialpose
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)

        # AMCL global localization service
        self.glob_cli = self.create_client(Empty, '/amcl/global_localization')

        # Optional: monitor /amcl_pose to detect convergence
        if self.args.wait_amcl:
            self.create_subscription(AmclPoseMsg, '/amcl_pose', self._on_amcl_pose, 10)

        # Kick main loop
        self.timer = self.create_timer(0.2, self._tick)

        self._called = False
        self._converged = False

        self.get_logger().info('auto_localize: waiting for /map (transient)...')

    # ---- Callbacks ----
    def _on_map(self, _msg):
        if not self._map_seen:
            self._map_seen = True
            self.get_logger().info('/map received.')

    def _on_amcl_pose(self, msg: AmclPoseMsg):
        # Covariance 6x6 (row-major), we check x (0), y (7), yaw (35)
        cov = msg.pose.covariance
        var_x = cov[0]
        var_y = cov[7]
        var_yaw = cov[35]
        std_x = math.sqrt(max(var_x, 0.0))
        std_y = math.sqrt(max(var_y, 0.0))
        std_yaw_deg = math.degrees(math.sqrt(max(var_yaw, 0.0)))

        if (std_x <= self.args.sigma_xy and
            std_y <= self.args.sigma_xy and
            std_yaw_deg <= self.args.sigma_yaw_deg):
            if not self._converged:
                self._converged = True
                self.get_logger().info(
                    f'AMCL converged: σx≈{std_x:.3f} m, σy≈{std_y:.3f} m, σyaw≈{std_yaw_deg:.1f}°'
                )

    # ---- Main loop ----
    def _tick(self):
        # Wait until /map is available
        if not self._map_seen:
            return

        # Only call once
        if not self._called:
            self._called = True
            if self.args.mode == 'pose':
                self._publish_initialpose()
            else:
                self._call_global_localization()

        # If the user doesn’t want to wait, exit once done
        if not self.args.wait_amcl:
            self.get_logger().info('Done (not waiting for AMCL).')
            rclpy.shutdown()
            return

        # If waiting: exit after convergence
        if self._converged:
            self.get_logger().info('Done (AMCL converged).')
            rclpy.shutdown()

    # ---- Actions ----
    def _publish_initialpose(self):
        x, y, yaw = self.args.pose
        qx, qy, qz, qw = yaw_to_quat(yaw)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = Quaternion(
            x=float(qx), y=float(qy), z=float(qz), w=float(qw)
        )

        # Covariance: use user-provided sigmas
        var_xy = self.args.cov_xy ** 2
        var_yaw = math.radians(self.args.cov_yaw_deg) ** 2
        msg.pose.covariance = [
            var_xy, 0, 0, 0, 0, 0,
            0, var_xy, 0, 0, 0, 0,
            0, 0, 0.01, 0, 0, 0,
            0, 0, 0, 0.01, 0, 0,
            0, 0, 0, 0, 0.01, 0,
            0, 0, 0, 0, 0, var_yaw
        ]
        self.init_pub.publish(msg)
        self.get_logger().info(
            f'Published /initialpose at (x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad), '
            f'σxy={self.args.cov_xy:.2f} m, σyaw={self.args.cov_yaw_deg:.1f}°'
        )

    def _call_global_localization(self):
        if not self.glob_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/amcl/global_localization not available yet.')
            return
        req = Empty.Request()
        future = self.glob_cli.call_async(req)
        future.add_done_callback(lambda _: self.get_logger().info('Called /amcl/global_localization.'))

# ---- CLI ----
def parse_args():
    p = argparse.ArgumentParser(description='Auto-initialize AMCL without RViz.')
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument('--pose', nargs=3, type=float, metavar=('X','Y','YAW'),
                      help='Initialize with known pose (radians for YAW).')
    mode.add_argument('--global', dest='global_loc', action='store_true',
                      help='Use AMCL global localization (no prior guess).')

    p.add_argument('--wait-amcl', action='store_true',
                   help='Wait until AMCL covariance is below thresholds before exiting.')
    p.add_argument('--sigma-xy', type=float, default=0.05,
                   help='Convergence threshold for σx and σy in meters (default: 0.05).')
    p.add_argument('--sigma-yaw-deg', type=float, default=5.0,
                   help='Convergence threshold for σyaw in degrees (default: 5.0).')

    p.add_argument('--cov-xy', type=float, default=0.25,
                   help='Initial pose covariance σ for x,y in meters (default: 0.25).')
    p.add_argument('--cov-yaw-deg', type=float, default=15.0,
                   help='Initial pose covariance σ for yaw in degrees (default: 15).')

    args = p.parse_args()
    args.mode = 'pose' if args.pose is not None else 'global'
    return args

def main():
    args = parse_args()
    rclpy.init()
    node = AutoLocalize(args)
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()