import sys
from datetime import datetime
from zoneinfo import ZoneInfo

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

class localization(Node):

    def __init__(self, type, dt, loggerName=None, loggerHeaders=[
    "imu_ax", "imu_ay", "kf_ax", "kf_ay",
    "kf_vx", "kf_w", "kf_x", "kf_y",
    "odom_x", "odom_y", "odom_vx", "odom_w", "stamp",
], motion="unknown"):

        super().__init__("localizer")

        self.pose=None

        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return

        if loggerName is None:
            timestamp = datetime.now(ZoneInfo("America/Toronto")).strftime("%Y%m%d_%H%M%S_%f")
            if type == kalmanFilter:
                # Read the actual configured covariance scales, not duplicated constants.
                q_value = f"{self.kf.Q[0, 0]:g}".replace(".", "_")
                r_value = f"{self.kf.R[0, 0]:g}".replace(".", "_")
                loggerName = f"Q{q_value}R{r_value}_{motion}_{timestamp}.csv"
            else:
                loggerName = f"raw_{motion}_{timestamp}.csv"
        self.loc_logger = Logger(loggerName, loggerHeaders)
        self.get_logger().info(f"Logging to {loggerName}")

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)

    def initKalmanfilter(self, dt):

        # TODONE Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)

        # State: [x, y, heading, angular velocity, velocity, acceleration]
        x = np.zeros(6)

        # Process noise: uncertainty added during each prediction.
        Q = 1 * np.eye(6)

        # Measurement noise for [v, w, ax, ay].
        R = 0.5 * np.eye(4)

        # Initial uncertainty in the six state estimates.
        P = np.eye(6)

        self.kf = kalman_filter(P, Q, R, x, dt)
        # Publish the estimate separately from raw odometry for live comparison.
        self.ekf_publisher = self.create_publisher(odom, "/ekf/odom", 10)
        self.last_stamp_ns = None
        # TODONE Part 3: Use the odometry and IMU data for the EKF
        self.odom_sub = message_filters.Subscriber(
            self, odom, "/odom", qos_profile=odom_qos
        )

        self.imu_sub = message_filters.Subscriber(
            self, Imu, "/imu", qos_profile=odom_qos
        )

        self.time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        self.time_syncher.registerCallback(self.fusion_callback)

    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):

        # TODONE Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        z = np.array([
            odom_msg.twist.twist.linear.x,   # Forward velocity
            odom_msg.twist.twist.angular.z,  # Angular velocity
            imu_msg.linear_acceleration.x,  # Forward acceleration
            imu_msg.linear_acceleration.y,  # Sideways acceleration
        ])

        # Integrate elapsed sensor time, not the controller's nominal timer period.
        stamp_ns = Time.from_msg(odom_msg.header.stamp).nanoseconds
        if self.last_stamp_ns is None:
            # Initialize the estimate in the odometry frame once.
            # Subsequent pose estimates are integrated; odom pose is NOT fused.
            self.kf.x[:3] = [
                odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y,
                euler_from_quaternion(odom_msg.pose.pose.orientation),
            ]
        if self.last_stamp_ns is not None:
            elapsed_ns = stamp_ns - self.last_stamp_ns
            if elapsed_ns <= 0:
                # Ignore duplicate/out-of-order samples; re-anchor after clock reset.
                if elapsed_ns < 0:
                    self.last_stamp_ns = stamp_ns
                return
            self.kf.dt = elapsed_ns / 1e9
            self.kf.predict()
        # The first sample corrects the initial state without inventing elapsed time.
        self.last_stamp_ns = stamp_ns
        self.kf.update(z)

        # Get the estimate
        xhat=self.kf.get_states()
        # Update the pose estimate to be returned by getPose
        self.pose=[
            xhat[0],
            xhat[1],
            xhat[2],
            odom_msg.header.stamp
        ]

        # Keep the sensor timestamp and frames so both paths share coordinates.
        estimate = odom()
        estimate.header = odom_msg.header
        estimate.child_frame_id = odom_msg.child_frame_id
        estimate.pose.pose.position.x = float(xhat[0])
        estimate.pose.pose.position.y = float(xhat[1])
        estimate.pose.pose.orientation.z = float(np.sin(xhat[2] / 2))
        estimate.pose.pose.orientation.w = float(np.cos(xhat[2] / 2))
        estimate.twist.twist.linear.x = float(xhat[4])
        estimate.twist.twist.angular.z = float(xhat[3])
        # ROS pose covariance order is [x, y, z, roll, pitch, yaw].
        for ros_i, state_i in ((0, 0), (1, 1), (5, 2)):
            for ros_j, state_j in ((0, 0), (1, 1), (5, 2)):
                estimate.pose.covariance[6 * ros_i + ros_j] = float(self.kf.P[state_i, state_j])
        for ros_i, state_i in ((0, 4), (5, 3)):
            for ros_j, state_j in ((0, 4), (5, 3)):
                estimate.twist.covariance[6 * ros_i + ros_j] = float(self.kf.P[state_i, state_j])
        self.ekf_publisher.publish(estimate)

        # TODONE Part 4: log your data
        stamp = (
            odom_msg.header.stamp.sec
            + odom_msg.header.stamp.nanosec * 1e-9
        )

        log_values = [
            z[2],                              # imu_ax
            z[3],                              # imu_ay
            xhat[5],                           # kf_ax
            xhat[4] * xhat[3],                  # kf_ay
            xhat[4],                           # kf_vx
            xhat[3],                           # kf_w
            xhat[0],                           # kf_x
            xhat[1],                           # kf_y
            odom_msg.pose.pose.position.x,      # odom_x
            odom_msg.pose.pose.position.y,      # odom_y
            odom_msg.twist.twist.linear.x,       # odom_vx
            odom_msg.twist.twist.angular.z,      # odom_w
            stamp,
        ]
        self.loc_logger.log_values(log_values)

    def odom_callback(self, pose_msg):

        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":

    init()

    LOCALIZER=localization(kalmanFilter, 0.1)

    spin(LOCALIZER)
