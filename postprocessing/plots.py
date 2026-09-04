import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from metrics import current_path_error
import yaml


def plot_trajectory(folder: Path):
    log_path = folder / "log.csv"
    waypoint_path = folder / "path.csv"

    manifest_path = folder / "scenario_info.yaml"
    with open(manifest_path, "r") as file:
        scenario_data = yaml.safe_load(file)

    data = pd.read_csv(log_path)
    waypoints = pd.read_csv(waypoint_path)

    plt.figure()
    plt.plot(data["y"], data["x"], marker="o")
    plt.plot(waypoints["y"], waypoints["x"], marker="x", linestyle="--", label="waypoints")
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    plt.title(scenario_data["Scenario name"])
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

def plot_trajectory_with_projected_points(folder: Path):
    log_path = folder / "log.csv"
    waypoint_path = folder / "path.csv"

    manifest_path = folder / "scenario_info.yaml"
    with open(manifest_path, "r") as file:
        scenario_data = yaml.safe_load(file)

    data = pd.read_csv(log_path)
    waypoints = pd.read_csv(waypoint_path)
    projected_points = []
    for index, row in data.iloc[::5].iterrows():
        error, point = current_path_error(waypoints, row[["x", "y"]].to_numpy())
        projected_points.append(point)
    projected_points = pd.DataFrame(projected_points, columns=["x", "y"])
    #create table of points projected on the path
    plt.figure()
    plt.plot(data["y"], data["x"], marker="o")
    plt.plot(waypoints["y"], waypoints["x"], marker="x", linestyle="--", label="waypoints")
    plt.plot(projected_points["y"], projected_points["x"], marker="o", linestyle="--", label="projected points")
    #plt.plot(waypoints["y"], waypoints["x"], marker="x", linestyle="--", label="waypoints")
    plt.xlabel("y [m]")
    plt.ylabel("x [m]")
    plt.title(scenario_data["Scenario name"])
    plt.axis("equal")
    plt.grid(True)
    plt.legend()

def create_scenario_dashboard(folder: Path):
    fig = plt.figure(figsize=(12, 8))

    gs = fig.add_gridspec(
        3, 2,
        height_ratios=[1, 1, 0.5]
    )

    ax_path = fig.add_subplot(gs[0:2, 0])
    ax_cte = fig.add_subplot(gs[0, 1])
    ax_steering = fig.add_subplot(gs[1, 1])
    ax_table = fig.add_subplot(gs[2, :])

    log_path = folder / "log.csv"
    waypoint_path = folder / "path.csv"
    manifest_path = folder / "scenario_info.yaml"

    log_data = pd.read_csv(log_path)
    waypoint_data = pd.read_csv(waypoint_path)
    with open(manifest_path, "r") as file:
            scenario_data = yaml.safe_load(file)

    plot_path(ax_path, log_data, waypoint_data, scenario_data)
    plot_steering_vs_time(ax_steering, log_data)

def plot_path(ax, log, waypoints, scenario):
    projected_points = []
    for index, row in log.iloc[::5].iterrows():
        error, point = current_path_error(waypoints, row[["x", "y"]].to_numpy())
        projected_points.append(point)
    projected_points = pd.DataFrame(projected_points, columns=["x", "y"])
    #create table of points projected on the path
    ax.plot(log["y"], log["x"], marker="o")
    ax.plot(waypoints["y"], waypoints["x"], marker="x", linestyle="--", label="waypoints")
    ax.plot(projected_points["y"], projected_points["x"], marker="o", linestyle="--", label="projected points")
    #plt.plot(waypoints["y"], waypoints["x"], marker="x", linestyle="--", label="waypoints")
    ax.set_xlabel("y [m]")
    ax.set_ylabel("x [m]")
    ax.set_title(scenario["Scenario name"])
    ax.axis("equal")
    ax.grid(True)
    ax.legend()    

def plot_steering_vs_time(ax, log):
    ax.plot(log["time"], log["steering_angle_actual_deg"], marker="o")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("Steering angle [deg]")
    ax.grid(True)
    ax.legend()  