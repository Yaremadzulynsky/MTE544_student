import matplotlib.pyplot as plt
import csv
from pathlib import Path
import argparse


def plot_errors(filename, show=True):
    with open(filename, newline="") as stream:
        reader = csv.reader(stream, skipinitialspace=True)
        headers = [name.strip() for name in next(reader, []) if name.strip()]
        values = [[float(value) for value in row[:len(headers)]]
                  for row in reader if any(value.strip() for value in row)]
    if not values:
        print(f"No samples in {filename}; skipping plot.")
        return
    indices = {name.strip(): i for i, name in enumerate(headers)}
    first_stamp = values[0][indices["stamp"]]
    time_list = [val[indices["stamp"]] - first_stamp for val in values]

    fig, axes = plt.subplots(2, 1, figsize=(14, 6))

    axes[0].plot(
        [row[indices["kf_x"]] for row in values],
        [row[indices["kf_y"]] for row in values],
        label="EKF",
    )
    if "odom_x" in indices and "odom_y" in indices:
        axes[0].plot(
            [row[indices["odom_x"]] for row in values],
            [row[indices["odom_y"]] for row in values],
            linestyle="--", label="Raw odometry",
        )
    axes[0].set_title("state space")
    axes[0].legend()
    axes[0].grid()

    axes[1].set_title("Acceleration: IMU vs EKF")
    for name in ("imu_ax", "imu_ay", "kf_ax", "kf_ay"):
        axes[1].plot(time_list, [row[indices[name]] for row in values], label=name)
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Acceleration (m/s²)")
    axes[1].legend()
    axes[1].grid()

    Path("images").mkdir(exist_ok=True)
    fig.savefig(f"images/{Path(filename).stem}.png", bbox_inches="tight")
    print(f"Saved images/{Path(filename).stem}.png")
    if show:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    parser.add_argument("--no-show", action="store_true", help="Save without opening a window")
    args = parser.parse_args()
    print("plotting the files", args.files)
    for filename in args.files:
        plot_errors(filename, show=not args.no_show)
