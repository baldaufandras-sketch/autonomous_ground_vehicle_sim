import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
from metrics import actual_path_error
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
        error, point = actual_path_error(waypoints, row[["x", "y"]].to_numpy())
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
    