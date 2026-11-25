import sys,time,json,csv,math,functools,argparse
from pathlib import Path
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry,Path as RosPath,OccupancyGrid
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import planner
from decisions import decision_maker
from utilities import euler_from_quaternion
OUT=Path.cwd()
parser=argparse.ArgumentParser()
parser.add_argument('--heuristic',choices=['euclidean','manhattan'],default='euclidean')
args=parser.parse_args()
original_search=planner.search
rclpy.init()
dm=decision_maker(Twist,'/cmd_vel',10,motion_type=planner.TRAJECTORY_PLANNER)
observer=Node('navigation_test_observer')
pub=observer.create_publisher(PoseStamped,'/goal_pose',10)
stop=observer.create_publisher(Twist,'/cmd_vel',10)
ex=SingleThreadedExecutor();ex.add_node(dm);ex.add_node(observer)
state={'odom':None,'map':0,'path':0,'commands':0,'run':'warmup'};samples=[];paths={};results=[]
def on_odom(m):
    xy=[m.pose.pose.position.x,m.pose.pose.position.y]
    state['odom']=xy
    samples.append([time.monotonic(),state['run'],*xy,euler_from_quaternion(m.pose.pose.orientation)])
def on_map(m):state['map']+=1
def on_path(m):
    state['path']+=1
    paths.setdefault(state['run'],[[p.pose.position.x,p.pose.position.y] for p in m.poses])
def on_cmd(m):state['commands']+=1
observer.create_subscription(Odometry,'/odom',on_odom,qos_profile_sensor_data)
observer.create_subscription(OccupancyGrid,'/customMap',on_map,10)
observer.create_subscription(RosPath,'/designedPath',on_path,10)
observer.create_subscription(Twist,'/cmd_vel',on_cmd,10)
def spin_for(s):
    end=time.monotonic()+s
    while time.monotonic()<end:ex.spin_once(timeout_sec=.05)
try:
    spin_for(3)
    assert state['odom'] is not None,'no odometry'
    assert state['map']>0,'no map publication'
    for heuristic in [args.heuristic]:
        # Select the existing A* function argument; production planner currently hardcodes default.
        planner.search=functools.partial(original_search,heuristic=heuristic)
        for idx,goal in enumerate([(2.,0.),(-1.,0.)],1):
            run=f'{heuristic}_{idx}';state['run']=run
            msg=PoseStamped();msg.header.frame_id='map';msg.pose.position.x=goal[0];msg.pose.position.y=goal[1];msg.pose.orientation.w=1.
            before=state['path'];start=time.monotonic();pub.publish(msg)
            active=False;success=False
            while time.monotonic()-start<110:
                ex.spin_once(timeout_sec=.05)
                active=active or dm.goal is not None
                if active and dm.goal is None:
                    success=True;break
            spin_for(.2)
            error=math.dist(state['odom'],goal)
            assert run in paths, 'No planned path received for '+run
            endpoint=paths[run][-1]
            endpoint_error=math.dist(state['odom'],endpoint)
            result={'run':run,'goal':goal,'success':success,'final_error_m':error,'planned_endpoint':endpoint,'endpoint_error_m':endpoint_error,'wall_seconds':time.monotonic()-start,'path_messages':state['path']-before}
            results.append(result);print(json.dumps(result),flush=True)
            (OUT/'results.json').write_text(json.dumps(results,indent=2))
            if not success:raise AssertionError('Navigation timeout: '+run)
            assert endpoint_error<.11, result
            assert error < .1 + math.sqrt(2)*.05 + .01, result
            spin_for(1)
finally:
    stop.publish(Twist())
    (OUT/'results.json').write_text(json.dumps(results,indent=2))
    (OUT/'paths.json').write_text(json.dumps(paths))
    with (OUT/'samples.csv').open('w') as f:
        w=csv.writer(f);w.writerow(['wall_time','run','x','y','yaw']);w.writerows(samples)
    print('ROS counters:',state,flush=True)
    dm.localizer.destroy_node();dm.destroy_node();observer.destroy_node();rclpy.shutdown()
