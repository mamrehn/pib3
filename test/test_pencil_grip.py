"""Test pencil grip with 5-DOF IK masking."""
import sys
from pathlib import Path

# Add parent directory to path so we can import pib3
sys.path.insert(0, str(Path(__file__).parent.parent))

import pib3
from pib3.config import TrajectoryConfig, IKConfig

print("="*60)
print("Testing PENCIL GRIP with 5-DOF IK Masking")
print("="*60)

# Test 1: Index finger (baseline - should work)
print("\n[Test 1] Index Finger Grip (3-DOF position-only)")
print("-" * 60)

config1 = TrajectoryConfig()
config1.ik.grip_style = "index_finger"

img_path = Path(__file__).parent.parent / "examples" / "Icon_IT_hover.png"

try:
    trajectory1 = pib3.generate_trajectory(str(img_path), config=config1)
    print(f"✓ SUCCESS: {len(trajectory1.waypoints)} waypoints")
    print(f"  Success rate: {trajectory1.metadata.get('success_rate', 0)*100:.1f}%")
except RuntimeError as e:
    print(f"✗ FAILED: {e}")

# Test 2: Pencil grip with 5-DOF masking (NEW - should now work!)
print("\n[Test 2] Pencil Grip (5-DOF: position + Z-axis direction)")
print("-" * 60)

config2 = TrajectoryConfig()
config2.ik.grip_style = "pencil_grip"

try:
    trajectory2 = pib3.generate_trajectory(str(img_path), config=config2)
    print(f"✓ SUCCESS: {len(trajectory2.waypoints)} waypoints")
    success_rate = trajectory2.metadata.get('success_rate', 0)
    print(f"  Success rate: {success_rate*100:.1f}%")
    print(f"  Success count: {trajectory2.metadata.get('success_count', 0)}")
    print(f"  Fail count: {trajectory2.metadata.get('fail_count', 0)}")

    if success_rate >= 0.8:
        print("\n✓ 5-DOF MASKING IS WORKING!")
        print("  Pencil grip now achieves good success rate by only")
        print("  constraining the pen's pointing direction, not full rotation.")
    elif success_rate > 0:
        print("\n⚠ PARTIAL SUCCESS")
        print("  5-DOF masking helps, but some points still fail.")
        print("  May need further tuning of orientation_weight or step_size.")
    else:
        print("\n✗ STILL FAILING")
        print("  5-DOF masking alone is not enough.")
        print("  Paper position or initial pose may still be problematic.")

    # Save for inspection
    output_path = Path(__file__).parent / "output_pencil.json"
    trajectory2.to_json(str(output_path))
    print(f"\n  Saved to: {output_path}")

except RuntimeError as e:
    print(f"✗ FAILED: {e}")
    print("\nPossible causes:")
    print("  - Paper still outside workspace for pencil grip")
    print("  - Orientation constraint still too strict")
    print("  - Initial pose incompatible with pencil grip")

print("\n" + "="*60)
print("SUMMARY")
print("="*60)
print("\n5-DOF Masking Implementation:")
print("  - Only constrains Z-axis direction (pen pointing down)")
print("  - Ignores rotation around pen's own axis")
print("  - Effectively 3 position + 2 orientation DOF")
print("  - Gives arm 1 redundant DOF → easier to find solutions")
print("\nKey improvement:")
print("  - Old: 6 constraints / 6 DOF = 0 redundancy → fails often")
print("  - New: 5 constraints / 6 DOF = 1 redundancy → more robust")
