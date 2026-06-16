from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

repo_root = Path(__file__).resolve().parents[1]
log_path = repo_root / "data" / "simulation_log.csv"
waypoint_path = repo_root / "data" / "waypoints.csv"


SIMULATION_COLUMNS = [
    "time",
    "x",
    "y",
    "speed",
    "heading",
    "acceleration",
    "steering_angle_actual",
    "steering_angle_request",
    "target_x",
    "target_y",
    "pursuit_controller_alpha",
    "current_waypoint_index",
]

def read_waypoints() -> pd.DataFrame:
    """
    Reads data/waypoints.csv.

    Expected format:
        x,y

    Works both with and without a header.
    """
    df = pd.read_csv(waypoint_path)

    if not {"x", "y"}.issubset(df.columns):
        df = pd.read_csv(
            waypoint_path,
            header=None,
            names=["x", "y"],
        )

    return df


def plot_simulation_with_lookahead(n_lookahead_samples: int = 50) -> None:
    sim_log = pd.read_csv(log_path)
    waypoints = read_waypoints()

    fig, ax = plt.subplots()

    # Reference path from waypoints
    ax.plot(
        waypoints["y"],
        waypoints["x"],
        "--",
        label="Waypoint path",
    )

    # Driven vehicle trajectory
    ax.plot(
        sim_log["y"],
        sim_log["x"],
        label="Vehicle trajectory",
        marker="s",
    )

    #Start and end markers
    ax.scatter(
        sim_log["y"].iloc[0],
        sim_log["x"].iloc[0],
        marker="o",
        s=60,
        label="Start",
    )

    ax.scatter(
        sim_log["y"].iloc[-1],
        sim_log["x"].iloc[-1],
        marker="s",
        s=60,
        label="End",
    )

    # Select only a limited number of samples to avoid visual clutter
    n_lookahead_samples = min(n_lookahead_samples, len(sim_log))

    sample_indices = np.linspace(
        0,
        len(sim_log) - 1,
        n_lookahead_samples,
        dtype=int,
    )

    sample_indices = np.unique(sample_indices)
    sampled = sim_log.iloc[sample_indices]
    print(sampled.keys())
    # Lookahead points
    ax.scatter(
        sim_log[" target_y"],
        sim_log[" target_x"],
        marker="x",
        s=35,
        label="Lookahead points",
    )

    # Lines from vehicle position to lookahead point
    for _, row in sim_log.iterrows():
        ax.plot(
            [row["y"], row[" target_y"]],
            [row["x"], row[" target_x"]],
            linewidth=2,
            alpha=0.35,
        )

    ax.set_title("Vehicle trajectory and Pure Pursuit lookahead points")
    ax.set_xlabel("x position")
    ax.set_ylabel("y position")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True)
    ax.legend()
    ax.set_xlim(-20, 20)

    plt.show()


def main() -> None:
    plot_simulation_with_lookahead(n_lookahead_samples=100)


if __name__ == "__main__":
    main()