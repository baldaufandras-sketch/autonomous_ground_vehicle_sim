import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

repo_root = Path(__file__).resolve().parents[1]
log_path = repo_root / "data" / "simulation_log.csv"
waypoint_path = repo_root / "data" / "waypoints.csv"

data = pd.read_csv(log_path)
waypoints = pd.read_csv(waypoint_path)

plt.figure()
plt.plot(data["y"], data["x"], marker="o")
plt.plot(waypoints["y"], waypoints["x"], marker="x", linestyle="--", label="waypoints")
plt.xlabel("y [m]")
plt.ylabel("x [m]")
plt.title("Vehicle trajectory with waypoints")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.show()