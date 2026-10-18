"""Create compact Lab 1 report figures directly from the recorded CSVs."""
import argparse
from pathlib import Path
from math import cos, sin, isfinite
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from utilities import FileReader


def read_log(path):
    headers, rows = FileReader(path).read_file()
    if not rows:
        raise ValueError(f'No samples in {path}')
    return {name: [row[i] for row in rows] for i, name in enumerate(headers)}


def elapsed(data):
    return [(stamp-data['stamp'][0])/1e9 for stamp in data['stamp']]


def make_composite(source, output, profile, scan_index=0):
    odom = read_log(source / f'odom_content_{profile}.csv')
    imu = read_log(source / f'imu_content_{profile}.csv')
    laser = read_log(source / f'laser_content_{profile}.csv')
    if not 0 <= scan_index < len(laser['stamp']):
        raise ValueError(f'Invalid scan index for {profile}')
    fig = plt.figure(figsize=(7.2, 4.5))
    # Explicit rectangles keep nested time plots separate on older Matplotlib.
    path = fig.add_axes([.10, .60, .34, .29])
    position = fig.add_axes([.61, .76, .36, .13])
    yaw = fig.add_axes([.61, .58, .36, .13], sharex=position)
    acceleration = fig.add_axes([.10, .30, .34, .13])
    rotation = fig.add_axes([.10, .12, .34, .13], sharex=acceleration)
    scan = fig.add_axes([.61, .12, .36, .31])

    path.plot(odom['x'], odom['y'], label='Path')
    path.scatter(odom['x'][0], odom['y'][0], marker='o', s=15, color='green', label='Start')
    path.scatter(odom['x'][-1], odom['y'][-1], marker='x', s=20, color='red', label='End')
    path.set(title='Trajectory', xlabel='X (m)', ylabel='Y (m)')
    path.set_aspect('equal', adjustable='box')
    t = elapsed(odom)
    position.plot(t, odom['x'], label='X')
    position.plot(t, odom['y'], '--', label='Y')
    position.set(title='Odometry', ylabel='Position (m)')
    yaw.plot(t, odom['th'], color='tab:green', label='Yaw')
    yaw.set(xlabel='Elapsed time (s)', ylabel='Yaw (rad)')
    position.tick_params(labelbottom=False)

    t = elapsed(imu)
    acceleration.plot(t, imu['acc_x'], label='X')
    acceleration.plot(t, imu['acc_y'], '--', label='Y')
    acceleration.set(title='IMU', ylabel='Accel. (m/s²)')
    rotation.plot(t, imu['angular_z'], color='tab:green', label='Z')
    rotation.set(xlabel='Elapsed time (s)', ylabel='Rate (rad/s)')
    acceleration.tick_params(labelbottom=False)

    points = []
    for i, distance in enumerate(laser['ranges'][scan_index]):
        if isfinite(distance) and distance > 0:
            angle = laser['angle_min'][scan_index] + i * laser['angle_increment'][scan_index]
            points.append((distance*cos(angle), distance*sin(angle)))
    if points:
        x, y = zip(*points)
        scan.scatter(x, y, s=2, label='Returns')
    else:
        scan.text(.5, .5, 'No valid returns', ha='center', transform=scan.transAxes)
    scan.scatter(0, 0, marker='x', s=20, color='red', label='Sensor')
    scan.set(title=f'Laser scan (row {scan_index})', xlabel='Laser X (m)', ylabel='Laser Y (m)')
    scan.set_aspect('equal', adjustable='box')
    for ax in fig.axes:
        ax.grid(alpha=.3)
        ax.legend(fontsize=6, loc='best', framealpha=.8, ncol=2)
    fig.suptitle(f'{profile.capitalize()} — October 9 in-lab data', fontsize=11, fontweight='bold')
    output.mkdir(parents=True, exist_ok=True)
    for extension in ('png', 'pdf'):
        target = output / f'{profile}_composite.{extension}'
        fig.savefig(target, dpi=300)
        print(target)
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-dir', type=Path, default=Path('oct-9-in-lab-logs'))
    parser.add_argument('--output-dir', type=Path, default=Path('runs/oct-9-in-lab-plots/composites'))
    parser.add_argument('--scan-index', type=int, default=0)
    args = parser.parse_args()
    plt.rcParams.update({'font.size': 8, 'axes.titlesize': 9, 'axes.labelsize': 8,
                         'xtick.labelsize': 7, 'ytick.labelsize': 7, 'lines.linewidth': 1})
    for profile in ('circle', 'spiral', 'line'):
        make_composite(args.source_dir, args.output_dir, profile, args.scan_index)
