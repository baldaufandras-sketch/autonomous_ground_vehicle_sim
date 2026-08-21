import argparse
from pathlib import Path
from plots import plot_trajectory, plot_trajectory_with_projected_points
import matplotlib.pyplot as plt

def postprocess(run_folder):
    print(f"Processing: {run_folder}")
    run_folder = Path("data") / "results" / run_folder / "scenarios"
    if not run_folder.is_dir():
        print("Result folder does not exist")
        return
    for item in run_folder.iterdir():
        plot_trajectory_with_projected_points(item)
        
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Postprocess AGV simulation results."
    )

    parser.add_argument(
        "run_folder",
        help="Path to the simulation result folder"
    )

    args = parser.parse_args()

    postprocess(args.run_folder)


if __name__ == "__main__":
    main()