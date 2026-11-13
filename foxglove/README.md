# Lab 3 EKF viewer

Import `lab3-ekf.json` using Foxglove's Layouts → Import from file.

## Connect

Start Gazebo using the existing Jetson launcher. In a Jetson terminal:

```bash
source ~/mte544-setup/sim-env.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=127.0.0.1 port:=8765 use_sim_time:=true
```

In a Mac terminal, keep this SSH tunnel running:

```bash
ssh -N -L 8765:127.0.0.1:8765 ydzulynsky@ydzulynsky-desktop
```

In Foxglove, choose Open connection → Foxglove WebSocket, URL `ws://localhost:8765`.

Run your controller (`decisions.py`) to produce EKF topics. For viewing sensors without commanding motion, run only the localizer in a separate Jetson terminal:

```bash
source ~/mte544-setup/sim-env.bash
mkdir -p ~/mte544-setup/lab3-viewer-logs
cd ~/mte544-setup/lab3-viewer-logs
python3 ~/MTE544_student/localization.py
```

Run either the standalone localizer or decisions.py, not both: decisions.py creates its own localizer. The CSV logger overwrites its named file when a new localizer starts; save runs under different names/directories.

## Read the layout

- **Motion + sensor fusion:** orange odometry trail and green EKF trail; four measurement plots compare raw readings, pre-update predictions, and post-update estimates.
- **Why the estimate changes:** innovation is measured minus predicted; correction is the state change from K times innovation. These are not ground-truth errors.
- **Uncertainty:** diagonal P entries before and after measurement correction. These are variances, not standard deviations. Position variance may continue growing without an absolute position measurement.

Odometry is one of the filter inputs, not independent ground truth. Initial pose is aligned to the first odometry message once; subsequent odometry poses are only displayed, not fused. No competing odom-to-base TF is published. The 3D fixed frame is `odom`; change it if your robot uses a different frame.

The EKF assumes planar motion and aligned IMU axes. Visualizing the estimate does not establish that sensor frames, noise tuning, or controller tuning are correct.

## Topics

- `/ekf/odom`: nav_msgs/Odometry; pose and velocity with covariance mapped to ROS conventions. Unmodelled dimensions have large variance.
- `/ekf/path`, `/ekf/odom_path`: nav_msgs/Path, up to 1,000 points, approximately 5 Hz in sensor time.
- `/ekf/measurements`, `/ekf/predicted_measurements`, `/ekf/estimated_measurements`, `/ekf/innovation`: Float64MultiArray in order `[v, omega, ax, ay]`.
- `/ekf/prior_variance`, `/ekf/posterior_variance`, `/ekf/correction`: Float64MultiArray in state order `[x, y, theta, omega, v, vdot]`.

Diagnostic arrays have no header; layout plots use receive time. Use recorded timestamps in the original CSV for precise time analysis. Trails show the most recent approximately 200 seconds, depending on actual sampling.

The JSON structure and topic references were checked programmatically. Import/rendering in your installed Foxglove version has not been visually verified.
