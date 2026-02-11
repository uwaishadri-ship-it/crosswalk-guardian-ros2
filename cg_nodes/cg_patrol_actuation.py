#!/usr/bin/env python3
import math
import subprocess
import shutil
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class CGPatrolActuation(Node):
    def __init__(self):
        super().__init__('cg_patrol_actuation')

        # ----------------------------
        # Waypoints (curb-to-curb)
        # ----------------------------
        self.Ax, self.Ay = 0.0, -2.3
        self.Bx, self.By = 0.0,  2.3

        # Start by going to B
        self.goal_x, self.goal_y = self.Bx, self.By
        self.current_goal_name = 'B'

        # ----------------------------
        # Control params
        # ----------------------------
        self.goal_tol = 0.18        # stop distance at curb
        self.yaw_tol = 0.08         # rad tolerance for final heading in turn
        self.k_lin = 0.18           # linear P gain
        self.k_ang = 1.2            # angular P gain
        self.max_lin = 0.12         # m/s
        self.max_ang = 0.8          # rad/s

        # ----------------------------
        # State
        # ----------------------------
        self.light = 'RED'
        self.prev_light = None
        self.warn_sent_this_yellow = False

        self.x = None
        self.y = None
        self.yaw = None

        self.turning = False
        self.yaw_target = None

        # ----------------------------
        # Warning sound (continuous during YELLOW)
        # ----------------------------
        self.sound_player_proc = None
        self.can_paplay = shutil.which("paplay") is not None
        self.sound_file = "/usr/share/sounds/freedesktop/stereo/message-new-instant.oga"

        # ----------------------------
        # ROS interfaces
        # ----------------------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.warn_pub = self.create_publisher(String, '/cg/warn', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.light_sub = self.create_subscription(String, '/cg/traffic_light', self.light_cb, 10)

        # control loop
        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

        self.get_logger().info("cg_patrol_actuation started.")
        self.get_logger().info(f"Waypoints: A=({self.Ax}, {self.Ay}), B=({self.Bx}, {self.By})")
        self.get_logger().info("GREEN: move to curb. YELLOW: rotate+warn at curb only + continuous sound. RED: stop.")

    # ----------------------------
    # Helpers
    # ----------------------------
    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        # yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def light_cb(self, msg: String):
        val = (msg.data or "").strip().upper()
        if val in ['GREEN', 'YELLOW', 'RED']:
            self.light = val

    def stop(self):
        self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def wrap_to_pi(self, a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    # ----------------------------
    # Continuous warning sound control
    # ----------------------------
    def start_yellow_sound(self):
        """Start continuous warning sound loop (non-blocking)."""
        if self.sound_player_proc is not None:
            return  # already running

        if self.can_paplay:
            # Run a small shell loop to replay the sound until killed
            cmd = f'while true; do paplay "{self.sound_file}" >/dev/null 2>&1; sleep 0.15; done'
            self.sound_player_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
        else:
            # Fallback: terminal bell every ~0.7s, looped in background
            cmd = 'while true; do printf "\\a"; sleep 0.7; done'
            self.sound_player_proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

    def stop_yellow_sound(self):
        """Stop continuous warning sound loop."""
        if self.sound_player_proc is None:
            return
        try:
            self.sound_player_proc.terminate()
            # give it a moment; if stubborn, kill
            time.sleep(0.05)
            if self.sound_player_proc.poll() is None:
                self.sound_player_proc.kill()
        except Exception:
            pass
        self.sound_player_proc = None

    def dist_to_goal(self):
        if self.x is None or self.y is None:
            return 999.0
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        return math.sqrt(dx * dx + dy * dy)

    def heading_to_goal(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        return math.atan2(dy, dx)

    def start_turn_180(self):
        # turn to face opposite direction
        self.turning = True
        self.yaw_target = self.wrap_to_pi(self.yaw + math.pi)
        self.get_logger().info(f"Starting 180° turn at curb. yaw_target={self.yaw_target:.3f}")

    def finish_turn_and_swap_goal(self):
        self.turning = False
        self.yaw_target = None

        # swap goal A <-> B
        if self.current_goal_name == 'A':
            self.goal_x, self.goal_y = self.Bx, self.By
            self.current_goal_name = 'B'
        else:
            self.goal_x, self.goal_y = self.Ax, self.Ay
            self.current_goal_name = 'A'

        self.get_logger().info(f"Turn complete. Swapped goal -> {self.current_goal_name} ({self.goal_x}, {self.goal_y})")

    # ----------------------------
    # Main loop
    # ----------------------------
    def loop(self):
        # wait odom
        if self.x is None or self.y is None or self.yaw is None:
            return

        # reset per-phase flags on light change
        if self.prev_light != self.light:
            # if leaving YELLOW -> stop sound immediately
            if self.prev_light == 'YELLOW' and self.light != 'YELLOW':
                self.stop_yellow_sound()

            if self.light != 'YELLOW':
                self.warn_sent_this_yellow = False

            # if light changes away from yellow, cancel turning (safety)
            if self.light != 'YELLOW' and self.turning:
                self.turning = False
                self.yaw_target = None

            self.prev_light = self.light

        # ---------- RED ----------
        if self.light == 'RED':
            self.stop_yellow_sound()
            self.stop()
            return

        # ---------- YELLOW ----------
        if self.light == 'YELLOW':
            # Start continuous sound during YELLOW
            self.start_yellow_sound()

            # warn once per yellow phase
            if not self.warn_sent_this_yellow:
                w = String()
                w.data = "WARNING: Crossing ending. Please stop / clear crosswalk."
                self.warn_pub.publish(w)
                self.warn_sent_this_yellow = True
                self.get_logger().info("Published /cg/warn (YELLOW)")

            # ONLY rotate if already at curb (goal)
            d = self.dist_to_goal()
            if d > self.goal_tol:
                # not at curb yet -> do NOT rotate mid-crosswalk
                self.stop()
                return

            # at curb -> rotate 180 degrees
            if not self.turning:
                self.start_turn_180()

            err = self.wrap_to_pi(self.yaw_target - self.yaw)
            if abs(err) < self.yaw_tol:
                self.stop()
                self.finish_turn_and_swap_goal()
            else:
                wz = clamp(self.k_ang * err, -self.max_ang, self.max_ang)
                self.publish_cmd(0.0, wz)
            return

        # ---------- GREEN ----------
        if self.light == 'GREEN':
            # Ensure yellow sound is off in GREEN
            self.stop_yellow_sound()

            # if we were turning before, cancel it (just in case)
            if self.turning:
                self.turning = False
                self.yaw_target = None

            d = self.dist_to_goal()
            if d <= self.goal_tol:
                # already at curb -> stop and wait for yellow to rotate
                self.stop()
                return

            # drive toward goal (simple heading controller)
            desired = self.heading_to_goal()
            ang_err = self.wrap_to_pi(desired - self.yaw)

            # angular correction
            w = clamp(self.k_ang * ang_err, -self.max_ang, self.max_ang)

            # linear velocity slows down when heading error is big
            v = self.k_lin * d
            v = min(v, self.max_lin)
            if abs(ang_err) > 0.7:
                v = 0.0  # don't drive while facing wrong direction

            self.publish_cmd(v, w)
            return


def main():
    rclpy.init()
    node = CGPatrolActuation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_yellow_sound()
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

