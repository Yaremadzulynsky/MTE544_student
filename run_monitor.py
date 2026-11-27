"""Read-only EKF logging and robot trail visualization; never publishes cmd_vel."""
import csv
import math
import json
from collections import deque
from datetime import datetime
from pathlib import Path as FilePath

import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from kalman_filter import kalman_filter
from utilities import euler_from_quaternion


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class RunMonitor(Node):
    def __init__(self):
        super().__init__('run_monitor')
        self.declare_parameter('log_directory', 'logs')
        directory = FilePath(self.get_parameter('log_directory').value)
        directory.mkdir(parents=True, exist_ok=True)
        self.filename = directory / f"ekf_pose_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
        self.file = self.filename.open('x', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            'stamp_ns', 'imu_stamp_ns', 'odom_x', 'odom_y', 'odom_yaw',
            'ekf_x', 'ekf_y', 'ekf_yaw', 'ekf_v', 'ekf_w', 'ekf_ax',
            'ekf_ay', 'imu_ax', 'imu_ay', 'frame_id', 'segment', 'reset_reason'])
        self.file.flush()
        self.path_file = self.filename.with_suffix(".paths.jsonl").open("x")
        self.last_plan = None
        self.segment = 0
        self.previous_measurement = None
        self.previous_frame = None
        self.kf = None
        self.last_stamp = None
        self.samples = 0
        self.trail = deque(maxlen=6000)
        self.last_trail_stamp = None
        self.frame = None
        self.pose_pub = self.create_publisher(PoseStamped, '/robotPose', 10)
        self.path_pub = self.create_publisher(Path, '/robotTrail', 10)
        self.ekf_pub = self.create_publisher(PoseStamped, '/ekfPose', 10)
        self.odom_sub = message_filters.Subscriber(
            self, Odometry, '/odom', qos_profile=qos_profile_sensor_data)
        self.imu_sub = message_filters.Subscriber(
            self, Imu, '/imu', qos_profile=qos_profile_sensor_data)
        self.odom_sub.registerCallback(self.observe_pose)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.imu_sub], queue_size=30, slop=0.1)
        self.sync.registerCallback(self.fusion)
        self.plan_sub = self.create_subscription(Path, "/designedPath", self.record_plan, 10)
        self.create_timer(5.0, self.check_input)
        self.get_logger().info(f'EKF CSV: {self.filename.resolve()}')

    def record_plan(self, msg):
        points = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]
        signature = (msg.header.frame_id, points)
        if signature == self.last_plan:
            return
        self.last_plan = signature
        self.path_file.write(json.dumps({
            'received_ns': self.get_clock().now().nanoseconds,
            'stamp_ns': stamp_ns(msg.header.stamp),
            'frame_id': msg.header.frame_id, 'segment': self.segment,
            'points': points}) + '\n')
        self.path_file.flush()
        self.get_logger().info(f'Recorded planned path: {len(points)} points')

    def check_input(self):
        if not self.samples:
            self.get_logger().warning('Waiting for synchronized /odom and /imu; no EKF rows yet.')

    def observe_pose(self, odom):
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose = odom.pose.pose
        self.pose_pub.publish(pose)
        stamp = stamp_ns(odom.header.stamp)
        if self.frame != odom.header.frame_id or (
                self.last_trail_stamp is not None and stamp < self.last_trail_stamp):
            self.trail.clear()
            self.last_trail_stamp = None
        self.frame = odom.header.frame_id
        if self.last_trail_stamp is None or stamp - self.last_trail_stamp >= 100_000_000:
            self.trail.append(pose)
            self.last_trail_stamp = stamp
            path = Path()
            path.header = odom.header
            path.poses = list(self.trail)
            self.path_pub.publish(path)

    def fusion(self, odom, imu):
        stamp = stamp_ns(odom.header.stamp)
        yaw = euler_from_quaternion(odom.pose.pose.orientation)
        measured = [odom.pose.pose.position.x, odom.pose.pose.position.y, yaw]
        z = np.array([odom.twist.twist.linear.x, odom.twist.twist.angular.z,
                      imu.linear_acceleration.x, imu.linear_acceleration.y])
        if not np.isfinite(measured + list(z)).all():
            self.get_logger().warning('Skipping nonfinite sensor sample')
            return
        reason = 'initial' if self.last_stamp is None else ''
        if self.last_stamp is not None:
            dt = (stamp - self.last_stamp) / 1e9
            if dt == 0:
                return
            if dt < 0:
                reason = 'time_reversed'
            elif dt > 2.0:
                reason = 'sensor_gap'
            elif odom.header.frame_id != self.previous_frame:
                reason = 'frame_changed'
            elif self.previous_measurement is not None:
                old = self.previous_measurement
                distance = math.hypot(measured[0]-old[0], measured[1]-old[1])
                angle = abs(math.atan2(math.sin(yaw-old[2]), math.cos(yaw-old[2])))
                # Conservative bounds for this mobile robot, with noise allowance.
                if distance > 0.3 + dt or angle > 0.5 + 3.0*dt:
                    reason = 'odom_discontinuity'
            if reason:
                self.segment += 1
                self.kf = None
                self.trail.clear()
                self.last_trail_stamp = None
                self.last_plan = None
                self.get_logger().warning(f'EKF reset: {reason}; segment {self.segment}')
        self.previous_measurement = measured
        self.previous_frame = odom.header.frame_id
        if self.kf is None:
            # Same state ordering and covariance values as localization.py.
            self.kf = kalman_filter(np.eye(6), np.eye(6), 0.5*np.eye(4),
                                    np.array(measured + [0., 0., 0.]))
            dt = 0.0
        else:
            dt = (stamp - self.last_stamp) / 1e9
        self.last_stamp = stamp
        self.kf.predict(dt)
        self.kf.update(z)
        x = self.kf.get_states()
        if not np.isfinite(x).all():
            self.get_logger().error('EKF became nonfinite; resetting estimator')
            self.kf = None
            return
        theta = math.atan2(math.sin(x[2]), math.cos(x[2]))
        self.writer.writerow([stamp, stamp_ns(imu.header.stamp), *measured,
                              x[0], x[1], theta, x[4], x[3], x[5], x[4]*x[3],
                              z[2], z[3], odom.header.frame_id, self.segment, reason])
        self.file.flush()
        self.samples += 1
        pose = PoseStamped()
        pose.header = odom.header
        pose.pose.position.x = float(x[0])
        pose.pose.position.y = float(x[1])
        pose.pose.orientation.z = math.sin(theta/2)
        pose.pose.orientation.w = math.cos(theta/2)
        self.ekf_pub.publish(pose)

    def destroy_node(self):
        self.file.close()
        self.path_file.close()
        return super().destroy_node()


if __name__ == '__main__':
    rclpy.init()
    monitor = RunMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
