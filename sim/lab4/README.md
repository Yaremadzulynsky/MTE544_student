# Lab 4 simulation demo

The repository-root `room.pgm` and `room.yaml` are a **SLAM-acquired map of
`sim/lab4/arena.world`**, a small custom arena with a central obstacle. They are
not a map of the TurtleBot house or the physical lab. Use this world with these
files. Before physical-robot work, replace both files with the map you acquire
in the lab, preserving the map/odometry alignment described in the course README.

## Prerequisites

Use the course ROS 2 Humble / Gazebo Classic setup with `gazebo_ros`,
`turtlebot3_gazebo` (Burger), `robot_state_publisher`, and RViz installed. Source
ROS and your simulator workspace in the terminal first. This launcher discovers
ROS package locations; it does not depend on the original author's home directory.
Do not use Gazebo Sim (`gz sim`) in place of Gazebo Classic (`gzserver`).

Python needs the ROS Python packages, NumPy, Matplotlib, PyYAML, and scikit-learn.
If scikit-learn is missing, use an isolated environment:

```bash
python3 -m venv --system-site-packages .venv-lab4
source .venv-lab4/bin/activate
python -m pip install -r sim/lab4/requirements.txt
```

Keep this virtual environment local; do not commit it. Alternatively, use the
lab's installed `python3-sklearn` package. `MTE544_PYTHON` can select a particular
Python executable without activating a virtual environment.

## Interactive demonstration

From the repository root in a graphical terminal:

```bash
./sim/lab4/launch_demo.sh euclidean
```

The launcher starts the matching world, spawns the robot at `(0, 0)`, publishes
its transforms and the saved map, runs the existing navigation code, and opens
RViz. Choose **2D Goal Pose** near `(2, 0)`, wait for the robot to stop, then choose
`(-1, 0)`. The route must go around the central obstacle. Watch the planned path
and robot in RViz; `decisions.log` reports `reached goal`.

Press Ctrl+C to stop everything, then repeat with:

```bash
./sim/lab4/launch_demo.sh manhattan
```

`run_decisions.py` supplies the existing A* heuristic argument without modifying
the planner/controller. The production `decisions.py` defaults to Euclidean and
raw odometry. The RViz configuration reads `/robot_description`, avoiding a
machine-specific URDF filename.

The launcher is simulation-only: it uses localhost ROS domain 155 and Gazebo
port 11355 by default (`MTE544_SIM_DOMAIN` and `MTE544_GAZEBO_PORT` can
override both for a separate instance). It refuses to reuse an occupied Gazebo port. Outputs go into a fresh,
ignored `sim/lab4/runs/` directory; the tracked map is never overwritten.

## Automated real-simulator checks

No graphical desktop is needed:

```bash
./sim/lab4/launch_demo.sh euclidean --test
./sim/lab4/launch_demo.sh manhattan --test
```

Run these sequentially. Each starts a fresh simulator and sends two goals using
the same `/goal_pose` messages as RViz. It uses the real ROS messages, subscriptions,
A* planner, controller, and Gazebo physics, with no ROS mocks. Each goal has a
110-second deadline once the callback loop is running. An external `timeout 240`
can additionally bound the whole launch if ROS communication itself stalls.

Success requires the decision node to finish the path and the measured pose to
be within 11 cm of its planned endpoint (10 cm configured tolerance plus sampling
allowance). The test also bounds requested-goal error by that tolerance plus the
5 cm grid's quantization. Results, actual poses, planned paths, and logs are saved
in the printed run directory. Goal snapping means requested-goal error can be
larger than endpoint error; both are reported.

## Acquiring another map

The included map was created with `slam_toolbox` from the simulated laser while
driving around the obstacle, then saved with `nav2_map_server map_saver_cli`.
The required packages for acquisition are `slam_toolbox` and `nav2_map_server`.
Follow Part 3 of the course README for the real robot. Stop SLAM before using the
saved-map publisher: both otherwise attempt to publish the map-to-odom transform.

## Recorded evidence and limits

`evidence/acquired-map-results.json` and its path plot record two successful
Euclidean routes on the included scanned map. `two-heuristic-results.json` and
its plot record four successful routes on a supplied geometric map of the same
arena. These were collected on 2026-11-25 using application commit `0eb4921`,
ROS 2 Humble, Gazebo Classic 11.15.1, and TurtleBot3 Burger. The earlier recorded
runs began from the previous route's endpoint; the packaged launcher starts each
new session at `(0, 0)`. Re-run the commands above for new-checkout evidence.

This demonstrates navigation in this arena, not arbitrary maps or physical
TurtleBot4 clearance. The larger real robot needs its own acquired map and an
on-site obstacle-clearance check. The map includes some unknown cells; the current
planner does not treat all unknown space as blocked. Choose goals in mapped free
space. The test does not validate global A* optimality or automatically detect
physical contact.

The TA requirements remain the course README/rubric: show a map, demonstrate two
separated goals with at least one heuristic, and test both heuristics for the full
assignment. Be ready to explain the simulation within the first 20 minutes.
