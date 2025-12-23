"""Simple test without visualization."""
import sys
from pathlib import Path

# Add parent directory to path so we can import pib3
sys.path.insert(0, str(Path(__file__).parent.parent))

import pib3

# Convert image to trajectory
img_path = Path(__file__).parent.parent.joinpath("examples/Icon_IT_hover.png")
print(f"Generating trajectory from: {img_path}")

trajectory = pib3.generate_trajectory(img_path)

print(f"✓ SUCCESS! Generated {len(trajectory.waypoints)} waypoints")
print(f"  Metadata: {trajectory.metadata}")

# Save output
output_path = Path(__file__).parent / "output.json"
trajectory.to_json(str(output_path))
print(f"✓ Saved to {output_path}")
