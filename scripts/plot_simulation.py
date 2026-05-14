import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

repo_root = Path(__file__).resolve().parents[1]
log_path = repo_root / "data" / "simulation_log.csv"

data = pd.read_csv(log_path)

plt.figure()
plt.plot(data["x"], data["y"], marker="o")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Vehicle trajectory")
plt.axis("equal")
plt.grid(True)
plt.show()