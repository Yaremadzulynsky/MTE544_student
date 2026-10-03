# Imports
from math import sqrt
from re import S

import rclpy

from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.duration import Duration

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# TODONE Part 3: Import message types needed:
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time



CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.5  # Initial spiral radius in metres
        self.motion_start_time=None
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODONE Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_min", "angle_increment", "stamp"])

        # TODONE Part 3: Create the QoS profile by setting the proper parameters in (...)
        # BEST_EFFORT subscriptions accept both the simulator's RELIABLE publishers
        # and BEST_EFFORT sensor publishers on hardware; keep only recent readings.
        qos=QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # TODONE Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        
        self.imu_subscription=self.create_subscription(Imu, '/imu', self.imu_callback, qos)
        
        # ENCODER subscription

        self.odom_subscription=self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        
        # LaserScan subscription 
        
        self.laser_subscription=self.create_subscription(LaserScan, '/scan', self.laser_callback, qos)
        
        self.create_timer(0.1, self.timer_callback)


    # TODONE Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    def imu_callback(self, imu_msg: Imu):
        # log imu msgs
        self.imu_logger.log_values([
            imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y,
            imu_msg.angular_velocity.z, Time.from_msg(imu_msg.header.stamp).nanoseconds,
        ])
        self.imu_initialized=True
        
    def odom_callback(self, odom_msg: Odometry):
        
        # log odom msgs
        pose=odom_msg.pose.pose
        q=pose.orientation
        self.odom_logger.log_values([
            pose.position.x, pose.position.y,
            euler_from_quaternion([q.x, q.y, q.z, q.w]),
            Time.from_msg(odom_msg.header.stamp).nanoseconds,
        ])
        self.odom_initialized=True
                
    def laser_callback(self, laser_msg: LaserScan):
        
        # log laser msgs with position msg at that time
        # Retain the scan timestamp for later alignment with odometry; these
        # callbacks do not assume independently received readings are simultaneous.
        self.laser_logger.log_values([
            list(laser_msg.ranges), laser_msg.angle_min, laser_msg.angle_increment,
            Time.from_msg(laser_msg.header.stamp).nanoseconds,
        ])
        self.laser_initialized=True
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODONE Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot

    def motion_elapsed_seconds(self):
        # Start timing on the first motion command, after sensors initialize.
        now=self.get_clock().now()
        if self.motion_start_time is None or now < self.motion_start_time:
            self.motion_start_time=now
        return (now-self.motion_start_time).nanoseconds / 1e9

    def make_straight_line(self):
        msg = Twist()
        # fill up the twist msg for straight line motion
        msg.linear.x = 0.1   # Forward speed in metres per second
        msg.angular.z = 0.0  # No turning
        return msg

    def make_circular_twist(self):
        msg = Twist()
        # fill up the twist msg for circular motion
        radius = 0.5         # metres
        msg.linear.x = 0.1   # metres per second
        msg.angular.z = msg.linear.x / radius  # 0.2 radians per second
        return msg
    def make_spiral_twist(self):
        msg=Twist()
        # fill up the twist msg for spiral motion
        elapsed=self.motion_elapsed_seconds()
        # Grow the turning radius by 0.01 m/s from a 0.5 m starting radius.
        self.radius_=0.5 + 0.01 * elapsed
        msg.linear.x = 0.1   # Forward speed in metres per second
        msg.angular.z = msg.linear.x / self.radius_  # Starts at 0.2 rad/s
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        # fill up the twist msg for line motion
        elapsed=self.motion_elapsed_seconds()
        # Accelerate at 0.02 m/s^2, reaching the 0.2 m/s speed limit after 10 s.
        msg.linear.x = min(0.02 * elapsed, 0.2)
        msg.angular.z = 0.0  # No turning
        return msg

import argparse

if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    # Let Python handle Ctrl+C so ROS remains usable for the final stop command.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    args = argParser.parse_args()

    if args.motion.lower() == "circle":
        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)
    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        try:
            if rclpy.ok():
                stop_msg=Twist()  # Zero all linear and angular velocities.
                ME.vel_publisher.publish(stop_msg)
                # Allow reliable subscribers to acknowledge before destroying ROS.
                ME.vel_publisher.wait_for_all_acked(Duration(seconds=1.0))
        finally:
            ME.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
