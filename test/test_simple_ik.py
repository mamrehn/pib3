"""Simple test to isolate IK issues."""
import sys
from pathlib import Path

# Add parent directory to path so we can import pib3
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
from pib3.trajectory import _load_robot, _get_joint_limits, _solve_ik_point, _set_initial_arm_pose, LEFT_ARM_JOINTS
from pib3.config import IKConfig, PaperConfig
import spatialmath as sm

# Load robot
robot = _load_robot()
joint_limits = _get_joint_limits(robot)

# Setup
arm = "left"
arm_joints = LEFT_ARM_JOINTS
arm_indices = list(arm_joints.values())
tcp_link = "urdf_palm_left"
tool_offset = sm.SE3(0.04, -0.06, 0)

# Initial pose
q_init = np.zeros(robot.n)
_set_initial_arm_pose(q_init, arm)

print("Initial joint configuration (arm only):")
for name, idx in arm_joints.items():
    print(f"  {name:30s} (idx {idx:2d}): {q_init[idx]:7.4f} rad = {np.degrees(q_init[idx]):7.2f}°")

# Get initial TCP position
T_tcp = robot.fkine(q_init, end=tcp_link)
if isinstance(T_tcp, np.ndarray):
    T_tcp = sm.SE3(T_tcp)
T_tcp_tool = T_tcp * tool_offset
tcp_pos = T_tcp_tool.t
tcp_rot = T_tcp_tool.R

print(f"\nInitial TCP position: [{tcp_pos[0]:.4f}, {tcp_pos[1]:.4f}, {tcp_pos[2]:.4f}]")
print(f"Initial TCP orientation (rotation matrix):")
print(tcp_rot)

# Paper center
paper = PaperConfig()
paper_center = np.array([
    paper.start_x + paper.size / 2,
    tcp_pos[1],  # Use TCP Y
    paper.height_z
])
print(f"\nPaper center: [{paper_center[0]:.4f}, {paper_center[1]:.4f}, {paper_center[2]:.4f}]")
print(f"Distance: {np.linalg.norm(tcp_pos - paper_center):.4f} m")

# IK config with improved parameters
ik_config = IKConfig(arm=arm)
print(f"\nIK Config:")
print(f"  max_iterations: {ik_config.max_iterations}")
print(f"  tolerance: {ik_config.tolerance} m = {ik_config.tolerance*1000} mm")
print(f"  step_size: {ik_config.step_size}")
print(f"  damping: {ik_config.damping}")

# Test 1: Position-only IK (no orientation)
print("\n" + "="*60)
print("TEST 1: Position-only IK (no orientation constraint)")
print("="*60)

q_sol, success = _solve_ik_point(
    robot, paper_center, q_init, tcp_link, tool_offset,
    arm_indices, joint_limits, arm_indices, ik_config,
    target_orientation=None, orientation_weight=0.0
)

if success:
    T_final = robot.fkine(q_sol, end=tcp_link)
    if isinstance(T_final, np.ndarray):
        T_final = sm.SE3(T_final)
    T_final_tool = T_final * tool_offset
    error = np.linalg.norm(T_final_tool.t - paper_center)
    print(f"✓ SUCCESS! Error: {error*1000:.2f} mm")
    print(f"  Final joint angles:")
    for name, idx in arm_joints.items():
        print(f"    {name:30s}: {q_sol[idx]:7.4f} rad = {np.degrees(q_sol[idx]):7.2f}°")
else:
    print(f"✗ FAILED - Cannot reach position even without orientation constraint")

# Test 2: Position + orientation IK (pencil grip)
print("\n" + "="*60)
print("TEST 2: Position + Orientation IK (pencil grip, weight=0.3)")
print("="*60)

target_orientation = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0]
])
print("Target orientation (180° rotation about X):")
print(target_orientation)

q_sol2, success2 = _solve_ik_point(
    robot, paper_center, q_init, tcp_link, tool_offset,
    arm_indices, joint_limits, arm_indices, ik_config,
    target_orientation=target_orientation, orientation_weight=0.3
)

if success2:
    T_final2 = robot.fkine(q_sol2, end=tcp_link)
    if isinstance(T_final2, np.ndarray):
        T_final2 = sm.SE3(T_final2)
    T_final2_tool = T_final2 * tool_offset
    pos_error = np.linalg.norm(T_final2_tool.t - paper_center)

    # Check orientation error
    R_error = target_orientation @ T_final2_tool.R.T
    angle = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))

    print(f"✓ SUCCESS!")
    print(f"  Position error: {pos_error*1000:.2f} mm")
    print(f"  Orientation error: {np.degrees(angle):.2f}°")
    print(f"  Final joint angles:")
    for name, idx in arm_joints.items():
        print(f"    {name:30s}: {q_sol2[idx]:7.4f} rad = {np.degrees(q_sol2[idx]):7.2f}°")
else:
    print(f"✗ FAILED - Orientation constraint makes solution infeasible")

# Test 3: Try with lower orientation weight
print("\n" + "="*60)
print("TEST 3: Position + Orientation IK (weight=0.1 - very soft)")
print("="*60)

q_sol3, success3 = _solve_ik_point(
    robot, paper_center, q_init, tcp_link, tool_offset,
    arm_indices, joint_limits, arm_indices, ik_config,
    target_orientation=target_orientation, orientation_weight=0.1
)

if success3:
    T_final3 = robot.fkine(q_sol3, end=tcp_link)
    if isinstance(T_final3, np.ndarray):
        T_final3 = sm.SE3(T_final3)
    T_final3_tool = T_final3 * tool_offset
    pos_error = np.linalg.norm(T_final3_tool.t - paper_center)

    R_error = target_orientation @ T_final3_tool.R.T
    angle = np.arccos(np.clip((np.trace(R_error) - 1) / 2, -1, 1))

    print(f"✓ SUCCESS!")
    print(f"  Position error: {pos_error*1000:.2f} mm")
    print(f"  Orientation error: {np.degrees(angle):.2f}°")
else:
    print(f"✗ FAILED")

print("\n" + "="*60)
print("DIAGNOSIS SUMMARY")
print("="*60)

if not success:
    print("⚠ CRITICAL: Robot cannot reach paper position at all!")
    print("   → Paper must be repositioned to reachable workspace")
elif not success2 and not success3:
    print("⚠ Position is reachable, but orientation constraint is too strict")
    print("   → Consider using 'index_finger' grip style (no orientation constraint)")
    print("   → Or further relax orientation weight")
elif not success2 and success3:
    print("✓ Position reachable with very soft orientation constraint")
    print("   → Reduce orientation_weight in config")
else:
    print("✓ Paper position is reachable with current orientation constraint")
    print("   → Issue must be elsewhere in the trajectory generation")
