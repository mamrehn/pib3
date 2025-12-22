import sys
from pathlib import Path

# Add parent directory to path so we can import pib3
sys.path.insert(0, str(Path(__file__).parent.parent))

import pib3

# Convert image to trajectory
img_path = Path(__file__).parent.parent.joinpath("examples/Icon_IT_hover.png")
trajectory = pib3.generate_trajectory(img_path)
trajectory.to_json("output.json")

# Visualize in browser
with pib3.Swift() as viz:
    viz.run_trajectory("output.json")

