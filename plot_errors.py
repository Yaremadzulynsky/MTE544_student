import matplotlib.pyplot as plt
import csv
from pathlib import Path
import argparse
import re


def plot_errors(filename, show=True):
    case_name = Path(filename).stem
    # Filenames encode decimal points as underscores, e.g. Q0_5R2_5_point.
    tuning = re.search(r"Q(\d+(?:[_.]\d+)?)R(\d+(?:[_.]\d+)?)", case_name)
    if tuning:
        q_value, r_value = (float(value.replace("_", ".")) for value in tuning.groups())
        title = f"{case_name} — Q = {q_value:g}, R = {r_value:g}"
    else:
        title = f"{case_name} — Q and R unspecified"
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
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(f"images/{Path(filename).stem}.png", bbox_inches="tight")
    print(f"Saved images/{Path(filename).stem}.png")

    # Keep the simplified figure and also provide the original all-state view.
    full_fig, full_axes = plt.subplots(2, 1, figsize=(14, 8))
    for line in axes[0].get_lines():
        full_axes[0].plot(line.get_xdata(), line.get_ydata(),
                          label=line.get_label(), linestyle=line.get_linestyle())
    full_axes[0].set_title("Estimated path")
    full_axes[0].set_xlabel("x (m)")
    full_axes[0].set_ylabel("y (m)")
    full_axes[0].legend()
    full_axes[0].grid()

    for name in headers:
        if name != "stamp":
            full_axes[1].plot(time_list, [row[indices[name]] for row in values],
                              label=name)
    full_axes[1].set_title("All logged measurements and states")
    full_axes[1].set_xlabel("Time (s)")
    full_axes[1].set_ylabel("Value (mixed units: m, m/s, m/s², rad, rad/s)")
    full_axes[1].legend(ncol=4)
    full_axes[1].grid()
    full_fig.suptitle(title)
    full_fig.tight_layout()
    full_path = Path("images") / f"{Path(filename).stem}_all_states.png"
    full_fig.savefig(full_path, bbox_inches="tight")
    print(f"Saved {full_path}")
    if show:
        plt.show()
    plt.close(fig)
    plt.close(full_fig)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    parser.add_argument("--no-show", action="store_true", help="Save without opening a window")
    args = parser.parse_args()
    print("plotting the files", args.files)
    for filename in args.files:
        plot_errors(filename, show=not args.no_show)
