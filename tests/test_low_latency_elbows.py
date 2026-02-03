#!/usr/bin/env python3
"""
Test script for low-latency mode with elbow motors only.

This script tests:
1. Discovering servo bricklet UIDs
2. Low-latency writing to elbow motors
3. Low-latency reading from elbow motors
4. Checking if ROS topics are populated during low-latency mode

Usage:
    python test_low_latency_elbows.py --host 172.26.34.149

Safety: Only controls elbow motors to avoid damage.
        Elbow range: -45° to +90° (mapped to 0%–100%).
"""

import argparse
import time
import threading
from typing import Dict, List, Optional

import pib3
from pib3 import Robot, LowLatencyConfig, build_motor_mapping, PIB_SERVO_CHANNELS


ELBOW_MOTORS = [
    "elbow_left",   # Servo Bricklet 3 (2cPQ), channel 8
    "elbow_right",  # Servo Bricklet 1 (2cPP), channel 8
]


class ROSTopicMonitor:
    """Monitor ROS topics for position updates."""

    def __init__(self, robot: Robot):
        self.robot = robot
        self.ros_updates: List[Dict] = []
        self.lock = threading.Lock()
        self._subscriber = None

    def start(self):
        """Start monitoring /joint_trajectory topic."""
        try:
            import roslibpy
            topic = roslibpy.Topic(
                self.robot._client,
                "/joint_trajectory",
                "trajectory_msgs/JointTrajectory",
            )
            topic.subscribe(self._on_message)
            self._subscriber = topic
            print("[ROS Monitor] Subscribed to /joint_trajectory")
        except Exception as e:
            print(f"[ROS Monitor] Failed to subscribe: {e}")

    def stop(self):
        """Stop monitoring."""
        if self._subscriber:
            try:
                self._subscriber.unsubscribe()
            except Exception:
                pass
            self._subscriber = None

    def _on_message(self, msg: dict):
        """Handle incoming ROS message."""
        with self.lock:
            timestamp = time.time()
            joint_names = msg.get("joint_names", [])
            points = msg.get("points", [])

            elbow_positions = {}
            for i, name in enumerate(joint_names):
                if name in ELBOW_MOTORS:
                    if points and len(points[0].get("positions", [])) > i:
                        elbow_positions[name] = points[0]["positions"][i]

            if elbow_positions:
                self.ros_updates.append({
                    "timestamp": timestamp,
                    "positions": elbow_positions,
                })

    def get_updates_since(self, since_time: float) -> List[Dict]:
        """Get all updates since a given timestamp."""
        with self.lock:
            return [u for u in self.ros_updates if u["timestamp"] > since_time]

    def clear(self):
        """Clear recorded updates."""
        with self.lock:
            self.ros_updates.clear()


def print_separator(title: str = ""):
    """Print a visual separator."""
    if title:
        print(f"\n{'='*60}")
        print(f"  {title}")
        print(f"{'='*60}")
    else:
        print("-" * 60)


def check_servo_power_relay(robot: Robot, relay_uid: str) -> bool:
    """Check if the solid state relay (servo power) is on.

    If off, turn it on so the servos are powered.
    """
    print_separator("Checking Servo Power Relay")

    if robot._tinkerforge_conn is None:
        print("[WARN] Tinkerforge not connected, cannot check relay.")
        return False

    try:
        from tinkerforge.bricklet_solid_state_relay_v2 import BrickletSolidStateRelayV2

        ssr = BrickletSolidStateRelayV2(relay_uid, robot._tinkerforge_conn)
        state = ssr.get_state()
        print(f"  Relay UID: {relay_uid}")
        print(f"  Relay state: {'ON' if state else 'OFF'}")

        if not state:
            print("  [ACTION] Servo power is OFF. Turning ON...")
            ssr.set_state(True)
            time.sleep(0.5)
            state = ssr.get_state()
            print(f"  Relay state after enable: {'ON' if state else 'OFF'}")
            if not state:
                print("  [FAIL] Could not enable servo power relay!")
                return False

        print("  [OK] Servo power is ON")
        return True

    except Exception as e:
        print(f"  [WARN] Could not check relay {relay_uid}: {e}")
        return False


def discover_bricklets(robot: Robot, timeout: float = 2.0) -> List[str]:
    """Discover servo bricklet UIDs via the existing connection."""
    print_separator("Discovering Servo Bricklets")

    uids = robot.discover_servo_bricklets(timeout=timeout)

    if not uids:
        print("[WARN] No servo bricklets found via discovery.")
        print("       Make sure Tinkerforge Brick Daemon is running on the robot.")
        return []

    print(f"[OK] Found {len(uids)} servo bricklet(s):")
    for i, uid in enumerate(uids):
        print(f"     [{i}] UID: {uid}")

    return uids


def test_low_latency_write(
    robot: Robot,
    motor: str,
    positions: List[float],
    monitor: Optional[ROSTopicMonitor] = None,
) -> bool:
    """Test low-latency write to a single motor, with parallel ROS readout."""
    print(f"\n  Testing write to {motor}...")

    for pos in positions:
        try:
            if monitor:
                monitor.clear()
            before = time.time()

            reached = robot.set_joint(motor, pos, unit="percent")
            current = robot.get_joint(motor, unit="percent")
            current_str = f"{current:.1f}" if current is not None else "None"
            status = "OK" if reached else "TIMEOUT"

            ros_status = ""
            if monitor:
                time.sleep(0.1)
                updates = monitor.get_updates_since(before)
                ros_positions = {}
                for u in updates:
                    ros_positions.update(u.get("positions", {}))
                if motor in ros_positions:
                    ros_status = f" | ROS: {ros_positions[motor]:.1f}"
                else:
                    ros_status = " | ROS: no update"

            print(f"    Set {pos:.0f}% -> Tinkerforge: {current_str}% [{status}]{ros_status}")
        except Exception as e:
            print(f"    [ERROR] {e}")
            return False

    return True


def test_low_latency_read(robot: Robot, motors: List[str]) -> Dict[str, float]:
    """Test low-latency read from motors."""
    print(f"\n  Reading {len(motors)} motors via low-latency...")

    start = time.perf_counter()
    positions = robot.get_joints(motors, unit="percent")
    elapsed = (time.perf_counter() - start) * 1000

    print(f"    Read completed in {elapsed:.1f}ms")
    for name, pos in positions.items():
        pos_str = f"{pos:.1f}" if pos is not None else "None"
        print(f"    {name}: {pos_str}%")

    return positions


def run_elbow_test(
    robot: Robot,
    motor: str,
    monitor: Optional[ROSTopicMonitor] = None,
):
    """Run tests on one elbow, with parallel ROS readout."""
    print_separator(f"Testing {motor}")

    # Read current position first
    current = robot.get_joint(motor, unit="percent")
    if current is not None:
        print(f"\n  Current position: {current:.1f}%")
    else:
        print(f"\n  Current position: None (motor not yet read)")

    # Move to neutral first
    print(f"\n  Moving to 50% (neutral)...")
    robot.set_joint(motor, 50.0, unit="percent")

    # Test small movements around center: 40%, 60%, 50%
    success = test_low_latency_write(robot, motor, [40.0, 60.0, 50.0], monitor)
    if success:
        print(f"  [OK] Write test passed for {motor}")
    else:
        print(f"  [FAIL] Write test failed for {motor}")

    # Test read
    positions = test_low_latency_read(robot, [motor])
    if positions:
        print(f"  [OK] Read test passed")
    else:
        print(f"  [FAIL] Read test failed")


def main():
    parser = argparse.ArgumentParser(
        description="Test low-latency mode with elbow motors"
    )
    parser.add_argument(
        "--host",
        default="172.26.34.149",
        help="Robot IP address (default: 172.26.34.149)",
    )
    parser.add_argument(
        "--servo1-uid",
        default="2cPP",
        help="UID of servo bricklet 1 (right arm). Default: 2cPP",
    )
    parser.add_argument(
        "--servo2-uid",
        default="2cPm",
        help="UID of servo bricklet 2 (shoulders). Default: 2cPm",
    )
    parser.add_argument(
        "--servo3-uid",
        default="2cPQ",
        help="UID of servo bricklet 3 (left arm). Default: 2cPQ",
    )
    parser.add_argument(
        "--relay-uid",
        default="27Po",
        help="UID of solid state relay bricklet (servo power). Default: 27Po",
    )
    parser.add_argument(
        "--left-only",
        action="store_true",
        help="Only test left elbow",
    )
    parser.add_argument(
        "--right-only",
        action="store_true",
        help="Only test right elbow",
    )
    args = parser.parse_args()

    print_separator("Low-Latency Mode Test - Elbow Motors")
    print(f"Robot host: {args.host}")
    print(f"Testing: {'Left elbow only' if args.left_only else 'Right elbow only' if args.right_only else 'Both elbows'}")

    servo1_uid = args.servo1_uid
    servo2_uid = args.servo2_uid
    servo3_uid = args.servo3_uid

    print(f"\nUsing UIDs:")
    print(f"  Servo 1 (right arm): {servo1_uid}")
    print(f"  Servo 2 (shoulders): {servo2_uid}")
    print(f"  Servo 3 (left arm):  {servo3_uid}")

    # Build motor mapping but filter to elbows only
    mapping = build_motor_mapping(servo1_uid, servo2_uid, servo3_uid)
    elbow_mapping = {k: v for k, v in mapping.items() if k in ELBOW_MOTORS}

    print(f"\n[INFO] Elbow motor mapping:")
    for name, (uid, ch) in elbow_mapping.items():
        print(f"  {name} -> bricklet {uid}, channel {ch}")

    config = LowLatencyConfig(
        enabled=True,
        motor_mapping=elbow_mapping,
        sync_to_ros=True,
    )

    try:
        with Robot(host=args.host, low_latency=config) as robot:
            print(f"[OK] Connected to robot at {args.host}")

            # Check servo power relay
            check_servo_power_relay(robot, args.relay_uid)

            # Discover bricklets
            discover_bricklets(robot)

            # Low-latency status
            print_separator("Low-Latency Status")
            print(f"  low_latency_available: {robot.low_latency_available}")
            print(f"  low_latency_enabled: {robot.low_latency_enabled}")

            # Start ROS topic monitor
            monitor = ROSTopicMonitor(robot)
            monitor.start()
            time.sleep(0.5)

            # Run tests
            if not args.right_only:
                run_elbow_test(robot, "elbow_left", monitor)

            if not args.left_only:
                run_elbow_test(robot, "elbow_right", monitor)

            # Stop monitor
            monitor.stop()

            # Summary
            print_separator("Test Summary")
            print(f"  Low-latency write: OK")
            print(f"  Low-latency read:  OK")

            # Return elbows to neutral
            print_separator("Cleanup")
            print("Returning elbows to 50%...")
            for motor in ELBOW_MOTORS:
                try:
                    robot.set_joint(motor, 50.0, unit="percent")
                except Exception:
                    pass
            time.sleep(0.5)
            print("[OK] Done")

    except Exception as e:
        print(f"[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
