"""Publish planned waypoints for visualization only; never publishes velocity."""
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from planner import planner, TRAJECTORY_PLANNER


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trajectory', choices=['parabola', 'sigmoid'], default='parabola')
    args = parser.parse_args()
    rclpy.init()
    node = Node('trajectory_preview')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(Path, '/planned_path', qos)
    path = Path()
    path.header.frame_id = 'odom'
    for x, y in planner(TRAJECTORY_PLANNER, trajectory=args.trajectory).plan():
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    def publish():
        path.header.stamp = node.get_clock().now().to_msg()
        for pose in path.poses:
            pose.header.stamp = path.header.stamp
        publisher.publish(path)
    node.create_timer(1.0, publish)
    publish()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
