"""Generate the Lab 1 odometry, IMU, and Cartesian laser plots."""
import argparse
from math import cos, sin, isfinite
from pathlib import Path
from utilities import FileReader


def plot_odometry(data, times, title):
    """Create trajectory and position/heading figures."""
    import matplotlib.pyplot as plt
    figures = []
    x, y = data['x'], data['y']
    fig, ax = plt.subplots()
    ax.plot(x, y, label='Odometry path')
    ax.scatter(x[0], y[0], color='green', marker='o', label='Start')
    ax.scatter(x[-1], y[-1], color='red', marker='x', label='End')
    ax.set(title=f'{title}: trajectory', xlabel='X (m)', ylabel='Y (m)')
    ax.set_aspect('equal', adjustable='box')
    figures.append(('trajectory', fig))

    fig, axes = plt.subplots(2, 1, sharex=True)
    axes[0].plot(times, x, label='X', color='tab:blue')
    axes[0].plot(times, y, label='Y', color='tab:orange', linestyle='--')
    axes[0].set_ylabel('Position (m)')
    axes[1].plot(times, data['th'], label='Yaw', color='tab:green')
    axes[1].set(xlabel='Elapsed time (s)', ylabel='Yaw (rad)')
    fig.suptitle(f'{title}: position and heading')
    figures.append(('position_heading', fig))
    return figures


def plot_imu(data, times, title):
    """Create acceleration and angular velocity figures."""
    import matplotlib.pyplot as plt
    figures = []
    fig, axes = plt.subplots(2, 1, sharex=True)
    axes[0].plot(times, data['acc_x'], label='Acceleration X', color='tab:blue')
    axes[0].plot(times, data['acc_y'], label='Acceleration Y',
                 color='tab:orange', linestyle='--')
    axes[0].set_ylabel('Acceleration (m/s²)')
    axes[1].plot(times, data['angular_z'], label='Angular velocity Z', color='tab:green')
    axes[1].set(xlabel='Elapsed time (s)', ylabel='Angular velocity (rad/s)')
    fig.suptitle(f'{title}: IMU readings')
    figures.append(('imu', fig))
    return figures


def plot_laser_scan(data, times, title, scan_index=0):
    """Convert one laser scan to Cartesian points and plot it."""
    import matplotlib.pyplot as plt
    figures = []
    if not 0 <= scan_index < len(data['stamp']):
        raise ValueError(f"scan index must be between 0 and {len(data['stamp']) - 1}")
    row = {name: samples[scan_index] for name, samples in data.items()}
    points = []
    # Preserve beam indices when filtering so the angles remain correct.
    for index, distance in enumerate(row['ranges']):
        if not isfinite(distance) or distance <= 0:
            continue
        angle = row['angle_min'] + index * row['angle_increment']
        points.append((distance * cos(angle), distance * sin(angle)))
    fig, ax = plt.subplots()
    if points:
        x, y = zip(*points)
        ax.scatter(x, y, s=8, label='Laser returns')
    else:
        ax.text(.5, .5, 'No finite positive returns in this scan',
                ha='center', va='center', transform=ax.transAxes)
    ax.scatter(0, 0, color='red', marker='x', label='Laser origin')
    ax.set(title=f'{title}: scan {scan_index} at {times[scan_index]:.2f} s',
           xlabel='X in laser frame (m)', ylabel='Y in laser frame (m)')
    ax.set_aspect('equal', adjustable='box')
    figures.append((f'scan_{scan_index}', fig))
    return figures


def plot_errors(filename, scan_index=0):
    """Return (figure name, figure) pairs for one sensor CSV."""
    headers, values = FileReader(filename).read_file()
    if not values:
        raise ValueError('CSV has no sensor samples')
    if 'stamp' not in headers or any(len(row) != len(headers) for row in values):
        raise ValueError('Missing stamp column or incomplete CSV row')
    columns = {name: index for index, name in enumerate(headers)}

    def column(name):
        return [row[columns[name]] for row in values]

    stamps = column('stamp')
    times = [(stamp - stamps[0]) / 1e9 for stamp in stamps]
    title = Path(filename).stem.replace('_', ' ')
    data = {name: column(name) for name in headers}
    if {'x', 'y', 'th'} <= data.keys():
        figures = plot_odometry(data, times, title)
    elif {'acc_x', 'acc_y', 'angular_z'} <= data.keys():
        figures = plot_imu(data, times, title)
    elif {'ranges', 'angle_min', 'angle_increment'} <= data.keys():
        figures = plot_laser_scan(data, times, title, scan_index)
    else:
        raise ValueError(f'Unrecognized sensor columns: {headers}')
    for _, fig in figures:
        for ax in fig.axes:
            ax.grid(True)
            ax.legend()
        fig.tight_layout()
    return figures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--files', nargs='+', required=True, help='Sensor CSV files')
    parser.add_argument('--scan-index', type=int, default=0, help='Zero-based laser scan row')
    parser.add_argument('--output-dir', type=Path, help='Save figures as PNG files')
    parser.add_argument('--no-show', action='store_true', help='Do not open plot windows')
    args = parser.parse_args()
    if args.no_show:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    if args.output_dir:
        args.output_dir.mkdir(parents=True, exist_ok=True)
    failed = False
    for filename in args.files:
        try:
            figures = plot_errors(filename, args.scan_index)
        except (OSError, ValueError, IndexError, TypeError) as error:
            print(f'Cannot plot {filename}: {error}')
            failed = True
            continue
        for kind, fig in figures:
            if args.output_dir:
                source = Path(filename)
                target = args.output_dir / f'{source.parent.name}_{source.stem}_{kind}.png'
                fig.savefig(target, dpi=200)
                print(f'Saved {target}')
            if args.no_show:
                plt.close(fig)
    if not args.no_show:
        plt.show()
    return 1 if failed else 0


if __name__ == '__main__':
    raise SystemExit(main())
