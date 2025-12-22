"""Diagnostic script to check IK setup and workspace reachability."""
import sys
from pathlib import Path

# Add parent directory to path so we can import pib3
sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np
import pib3
from pib3.trajectory import _load_robot, _get_joint_limits, LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS
from pib3.config import PaperConfig, IKConfig
import spatialmath as sm


def diagnose_joint_limits():
    """Print joint limits from URDF."""
    print("=" * 60)
    print("JOINT LIMITS FROM URDF")
    print("=" * 60)

    robot = _load_robot()
    joint_limits = _get_joint_limits(robot)

    print(f"Robot has {robot.n} joints total")
    print(f"Joint limits extracted: {len(joint_limits)}\n")

    for i, link in enumerate(robot.links):
        if link.isjoint and link.jindex is not None:
            limits = joint_limits[i] if i < len(joint_limits) else None
            print(f"Joint {i:2d}: {link.name:30s} "
                  f"qlim={limits if limits else 'N/A'}")

    print("\nLeft arm joints:", LEFT_ARM_JOINTS)
    print("Right arm joints:", RIGHT_ARM_JOINTS)

    return robot, joint_limits


def check_initial_pose_reach(robot, arm="left"):
    """Check if robot can reach paper center from initial pose."""
    print("\n" + "=" * 60)
    print(f"INITIAL POSE REACHABILITY CHECK ({arm.upper()} ARM)")
    print("=" * 60)

    from pib3.trajectory import _set_initial_arm_pose

    # Get initial pose
    q_init = np.zeros(robot.n)
    _set_initial_arm_pose(q_init, arm)

    # Paper default config
    paper = PaperConfig()
    ik_config = IKConfig(arm=arm)

    # Determine TCP link
    if ik_config.grip_style == "pencil_grip":
        tcp_link = "urdf_palm_left" if arm == "left" else "urdf_palm_right"
        tool_offset = sm.SE3(0.04, -0.06, 0)
    else:
        tcp_link = f"urdf_finger_distal_{2 if arm == 'left' else 7}"
        tool_offset = sm.SE3(0, 0.027, 0)

    # Compute TCP position at initial pose
    T_tcp = robot.fkine(q_init, end=tcp_link)
    if isinstance(T_tcp, np.ndarray):
        T_tcp = sm.SE3(T_tcp)
    T_tcp_tool = T_tcp * tool_offset
    tcp_pos = T_tcp_tool.t

    print(f"\nGrip style: {ik_config.grip_style}")
    print(f"TCP link: {tcp_link}")
    print(f"Tool offset: {tool_offset.t}")
    print(f"\nInitial TCP position: [{tcp_pos[0]:.4f}, {tcp_pos[1]:.4f}, {tcp_pos[2]:.4f}]")

    # Paper center position (using TCP Y if center_y is None)
    paper_center_y = paper.center_y if paper.center_y is not None else tcp_pos[1]
    paper_center = np.array([
        paper.start_x + paper.size / 2,
        paper_center_y,
        paper.height_z
    ])

    print(f"\nPaper configuration:")
    print(f"  start_x: {paper.start_x:.4f} m")
    print(f"  size: {paper.size:.4f} m")
    print(f"  height_z: {paper.height_z:.4f} m")
    print(f"  center_y: {paper_center_y:.4f} m {'(auto from TCP)' if paper.center_y is None else ''}")
    print(f"  drawing_scale: {paper.drawing_scale}")
    print(f"\nPaper center: [{paper_center[0]:.4f}, {paper_center[1]:.4f}, {paper_center[2]:.4f}]")

    # Distance from initial TCP to paper center
    distance = np.linalg.norm(tcp_pos - paper_center)
    print(f"\nDistance from initial TCP to paper center: {distance:.4f} m")

    # Try solving IK for paper center
    print("\nAttempting IK for paper center...")
    from pib3.trajectory import _solve_ik_point

    arm_joints = LEFT_ARM_JOINTS if arm == "left" else RIGHT_ARM_JOINTS
    arm_indices = list(arm_joints.values())
    joint_limits = _get_joint_limits(robot)

    # Get path joints (same as arm_joint_indices for pencil grip)
    path_joints = arm_indices

    # Target orientation for pencil grip
    target_orientation = None
    orientation_weight = 0.0
    if ik_config.grip_style == "pencil_grip":
        target_orientation = sm.SE3.Rx(np.pi).R
        orientation_weight = 0.3

    q_solution, success = _solve_ik_point(
        robot,
        paper_center,
        q_init,
        tcp_link,
        tool_offset,
        arm_indices,
        joint_limits,
        path_joints,
        ik_config,
        target_orientation=target_orientation,
        orientation_weight=orientation_weight,
    )

    if success:
        # Verify solution
        T_final = robot.fkine(q_solution, end=tcp_link)
        if isinstance(T_final, np.ndarray):
            T_final = sm.SE3(T_final)
        T_final_tool = T_final * tool_offset
        final_pos = T_final_tool.t
        error = np.linalg.norm(final_pos - paper_center)

        print(f"✓ IK SUCCESS!")
        print(f"  Final position: [{final_pos[0]:.4f}, {final_pos[1]:.4f}, {final_pos[2]:.4f}]")
        print(f"  Position error: {error*1000:.2f} mm")
        print(f"  Joint configuration: {np.round(q_solution[arm_indices], 3)}")
    else:
        print(f"✗ IK FAILED - Paper center is unreachable!")
        print(f"\nSuggestions:")
        print(f"  1. Try moving paper closer: start_x = {paper.start_x - 0.02:.3f}")
        print(f"  2. Try raising paper: height_z = {paper.height_z + 0.02:.3f}")
        print(f"  3. Try using the other arm: arm = '{'right' if arm == 'left' else 'left'}'")
        print(f"  4. Try different grip style: grip_style = '{'index_finger' if ik_config.grip_style == 'pencil_grip' else 'pencil_grip'}'")


def analyze_image_trajectory():
    """Analyze the specific test image trajectory."""
    print("\n" + "=" * 60)
    print("IMAGE TRAJECTORY ANALYSIS")
    print("=" * 60)

    import cv2
    from PIL import Image

    img_path = "examples/Icon_IT_hover.png"
    img = Image.open(img_path)
    print(f"\nImage: {img_path}")
    print(f"  Size: {img.size}")
    print(f"  Mode: {img.mode}")

    # Convert to grayscale if needed
    img_gray = img.convert('L')
    img_array = np.array(img_gray)

    # Threshold
    threshold = 128
    _, binary = cv2.threshold(img_array, threshold, 255, cv2.THRESH_BINARY)

    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    print(f"  Contours found: {len(contours)}")

    if contours:
        # Compute bounding box of all contours
        all_points = np.vstack(contours)
        x, y, w, h = cv2.boundingRect(all_points)
        print(f"  Bounding box: x={x}, y={y}, w={w}, h={h}")
        print(f"  Aspect ratio: {w/h:.2f}")


if __name__ == "__main__":
    print("\nPIB3 IK DIAGNOSTIC TOOL")
    print("This tool helps diagnose IK failures\n")

    try:
        # 1. Check joint limits
        robot, joint_limits = diagnose_joint_limits()

        # 2. Check reachability for left arm
        check_initial_pose_reach(robot, arm="left")

        # 3. Analyze test image
        analyze_image_trajectory()

        print("\n" + "=" * 60)
        print("DIAGNOSTIC COMPLETE")
        print("=" * 60)

    except Exception as e:
        print(f"\n✗ Error during diagnosis: {e}")
        import traceback
        traceback.print_exc()
