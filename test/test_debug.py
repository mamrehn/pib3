"""Debug trajectory generation to see what's failing."""
import sys
from pathlib import Path

# Add parent directory to path so we can import pib3
sys.path.insert(0, str(Path(__file__).parent.parent))

import pib3
from pib3.config import TrajectoryConfig, PaperConfig, IKConfig

# Create config with explicit settings
config = TrajectoryConfig()
config.paper = PaperConfig(
    start_x=0.22,
    size=0.10,
    height_z=0.80,
    center_y=-0.15,  # Explicitly set for left arm
    drawing_scale=0.8,
    lift_height=0.03
)
config.ik = IKConfig(
    max_iterations=300,
    tolerance=0.002,
    step_size=0.2,
    damping=0.1,
    arm="left",
    grip_style="index_finger"  # Position-only IK
)

print("Configuration:")
print(f"  Paper: x=[{config.paper.start_x:.3f}, {config.paper.start_x + config.paper.size:.3f}], "
      f"y={config.paper.center_y:.3f}, z={config.paper.height_z:.3f}")
print(f"  IK: grip={config.ik.grip_style}, arm={config.ik.arm}")
print(f"  IK solver: max_iter={config.ik.max_iterations}, tol={config.ik.tolerance*1000}mm, "
      f"step={config.ik.step_size}, damp={config.ik.damping}")

# Try to generate trajectory
print("\nGenerating trajectory...")
try:
    trajectory = pib3.generate_trajectory("../examples/Icon_IT_hover.png", config=config)
    print(f"✓ SUCCESS! Generated {len(trajectory.waypoints)} waypoints")
    print(f"  Metadata: {trajectory.metadata}")

    # Save output
    trajectory.to_json("../output_debug.json")
    print(f"✓ Saved to ../output_debug.json")

except RuntimeError as e:
    print(f"✗ FAILED: {e}")
    print("\nThis suggests the paper position is still unreachable.")
    print("Try these paper positions:")
    print("  1. Closer and higher: start_x=0.25, height_z=0.85")
    print("  2. Further left: center_y=-0.20")
    print("  3. Smaller: size=0.08")
