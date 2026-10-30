# Imports


import sys

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from pid import PID_ctrl

from rclpy import init, spin, spin_once
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry as odom

from localization import localization, rawSensor

from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController

# You may add any other imports you may need/want to use below
# import ...


class decision_maker(Node):
    
    def __init__(self, publisher_msg, publishing_topic, qos_publisher, goalPoint, rate=10, motion_type=POINT_PLANNER, trajectory="parabola"):

        super().__init__("decision_maker")

        self.publisher = self.create_publisher(publisher_msg, publishing_topic, qos_publisher)

        publishing_period=1/rate
        
        # Instantiate the controller
        # TODO Part 5: Tune your parameters here (to be done in lab section)
    
        if motion_type == POINT_PLANNER:
            self.controller=controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(POINT_PLANNER)    
    
    
        elif motion_type==TRAJECTORY_PLANNER:
            self.controller=trajectoryController(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner=planner(TRAJECTORY_PLANNER, trajectory=trajectory)

        else:
            print("Error! you don't have this planner", file=sys.stderr)


        # Instantiate the localization, use rawSensor for now  
        self.localizer=localization(rawSensor)

        # Instantiate the planner
        # NOTE: goalPoint is used only for the pointPlanner
        self.goal=self.planner.plan(goalPoint)

        self.create_timer(publishing_period, self.timerCallback)


    def timerCallback(self):
        
        spin_once(self.localizer, timeout_sec=0.0)

        if self.localizer.getPose()  is  None:
            print("waiting for odom msgs ....")
            return

        vel_msg=Twist()
        
        # # TODO Part 3: Check if you reached the goal
        # if type(self.goal) == list:
        #     reached_goal=True
        # else: 
        #     reached_goal=False
        pose = self.localizer.getPose()

# Trajectories end at the last point; point goals are already (x, y).
        if self.planner.type == TRAJECTORY_PLANNER:
            final_goal = self.goal[-1]
        else:
            final_goal = self.goal

        distance_to_goal = calculate_linear_error(pose, final_goal)
        reached_goal = distance_to_goal <= 0.05
        

        if reached_goal:
            print("reached goal")
            self.publisher.publish(vel_msg)
            
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            
            raise SystemExit # raise SystemExit to stop the node when the goal is reached
        
        velocity, yaw_rate = self.controller.vel_request(self.localizer.getPose(), self.goal, True)

        vel_msg.linear.x = float(velocity)
        vel_msg.angular.z = float(yaw_rate)
        self.publisher.publish(vel_msg) # publishes velocity commands

import argparse


def main(args=None):
    
    init()

    # TODO Part 3: You migh need to change the QoS profile based on whether you're using the real robot or in simulation.
    # Remember to define your QoS profile based on the information available in "ros2 topic info /odom --verbose" as explained in Tutorial 3
    
    cmd_vel_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    

    if args.motion.lower() == "point":
        DM=decision_maker(Twist, "/cmd_vel", cmd_vel_qos, [-1.0, -1.0], motion_type=POINT_PLANNER)
    elif args.motion.lower() == "trajectory":
        DM=decision_maker(Twist, "/cmd_vel", cmd_vel_qos, [-1.0, -1.0], motion_type=TRAJECTORY_PLANNER, trajectory=args.trajectory) # selects motion profile
    else:
        print("invalid motion type", file=sys.stderr)        
    
    
    
    try:
        spin(DM)
    except SystemExit:
        print(f"reached there successfully {DM.localizer.pose}")


if __name__=="__main__":

    argParser=argparse.ArgumentParser(description="point or trajectory") 
    argParser.add_argument("--motion", type=str, default="point")
    argParser.add_argument("--trajectory", type=str, choices=["parabola", "sigmoid"], default="parabola",
                           help="trajectory shape used with --motion trajectory (default: parabola)") # selects the trajectory shape used with --motion trajectory
    args = argParser.parse_args()

    main(args)
