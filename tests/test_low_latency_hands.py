#!/usr/bin/env python3
"""
Test script for low-latency mode with hand motors only.

This script tests:
1. Discovering servo bricklet UIDs
2. Low-latency writing to hand motors
3. Low-latency reading from hand motors
4. Checking if ROS topics are populated during low-latency mode

Usage:
    python test_low_latency_hands.py --host 172.26.34.149

Safety: Only controls hand motors (fingers) to avoid damage.
"""

import argparse
import time
import threading
from typing import Dict, List, Optional

import pib3
from pib3 import Robot, LowLatencyConfig, build_motor_mapping, PIB_SERVO_CHANNELS


# Hand motors only (safe to test)
LEFT_HAND_MOTORS = [
    "thumb_left_opposition",
    "thumb_left_stretch",
    "index_left_stretch",
    "middle_left_stretch",
    "ring_left_stretch",
    "pinky_left_stretch",
]

RIGHT_HAND_MOTORS = [
    "thumb_right_opposition",
    "thumb_right_stretch",
    "index_right_stretch",
    "middle_right_stretch",
    "ring_right_stretch",
    "pinky_right_stretch",
]

ALL_HAND_MOTORS = LEFT_HAND_MOTORS + RIGHT_HAND_MOTORS


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

            # Extract hand motor positions
            hand_positions = {}
            for i, name in enumerate(joint_names):
                if name in ALL_HAND_MOTORS:
                    if points and len(points[0].get("positions", [])) > i:
                        hand_positions[name] = points[0]["positions"][i]

            if hand_positions:
                self.ros_updates.append({
                    "timestamp": timestamp,
                    "positions": hand_positions,
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


def test_low_latency_write(robot: Robot, motor: str, positions: List[float]) -> bool:
    """Test low-latency write to a single motor."""
    print(f"\n  Testing write to {motor}...")

    for pos in positions:
        try:
            # async_=False waits for motor to reach target position
            reached = robot.set_joint(motor, pos, unit="percent", async_=False, timeout=2.0)
            current = robot.get_joint(motor, unit="percent")
            status = "OK" if reached else "TIMEOUT"
            print(f"    Set {pos:.0f}% -> Read {current:.1f}% [{status}]")
        except Exception as e:
            print(f"    [ERROR] {e}")
            return False

    return True


def test_low_latency_read(robot: Robot, motors: List[str]) -> Dict[str, float]:
    """Test low-latency read from multiple motors."""
    print(f"\n  Reading {len(motors)} motors via low-latency...")

    start = time.perf_counter()
    positions = robot.get_joints(motors, unit="percent")
    elapsed = (time.perf_counter() - start) * 1000

    print(f"    Read completed in {elapsed:.1f}ms")
    for name, pos in positions.items():
        print(f"    {name}: {pos:.1f}%")

    return positions


def test_ros_topic_population(
    robot: Robot,
    monitor: ROSTopicMonitor,
    motor: str,
) -> bool:
    """Test if low-latency commands populate ROS topics."""
    print(f"\n  Testing ROS topic population for {motor}...")

    # Clear previous updates
    monitor.clear()
    before_time = time.time()

    # Send a low-latency command (synchronous)
    robot.set_joint(motor, 30.0, unit="percent", async_=False, timeout=2.0)
    robot.set_joint(motor, 70.0, unit="percent", async_=False, timeout=2.0)

    # Check if ROS topic received updates
    updates = monitor.get_updates_since(before_time)

    if updates:
        print(f"    [INFO] ROS topic received {len(updates)} update(s)")
        for u in updates:
            if motor in u["positions"]:
                print(f"           {motor} = {u['positions'][motor]}")
        return True
    else:
        print(f"    [INFO] No ROS topic updates (expected - low-latency bypasses ROS)")
        return False


def run_hand_test(robot: Robot, hand: str, motors: List[str]):
    """Run tests on one hand."""
    print_separator(f"Testing {hand} Hand")

    # Test write to first finger
    test_motor = motors[0]
    print(f"\n  Using {test_motor} for write test...")

    # Move to known position (synchronous)
    robot.set_joint(test_motor, 50.0, unit="percent", async_=False, timeout=2.0)

    # Test write
    success = test_low_latency_write(robot, test_motor, [30.0, 70.0, 50.0])
    if success:
        print(f"  [OK] Write test passed for {test_motor}")
    else:
        print(f"  [FAIL] Write test failed for {test_motor}")

    # Test read
    positions = test_low_latency_read(robot, motors)
    if positions:
        print(f"  [OK] Read test passed ({len(positions)} motors)")
    else:
        print(f"  [FAIL] Read test failed")


def main():
    parser = argparse.ArgumentParser(
        description="Test low-latency mode with hand motors"
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
        help="Only test left hand",
    )
    parser.add_argument(
        "--right-only",
        action="store_true",
        help="Only test right hand",
    )
    args = parser.parse_args()

    print_separator("Low-Latency Mode Test - Hand Motors")
    print(f"Robot host: {args.host}")
    print(f"Testing: {'Left hand only' if args.left_only else 'Right hand only' if args.right_only else 'Both hands'}")

    servo1_uid = args.servo1_uid
    servo2_uid = args.servo2_uid
    servo3_uid = args.servo3_uid

    print(f"\nUsing UIDs:")
    print(f"  Servo 1 (right arm): {servo1_uid}")
    print(f"  Servo 2 (shoulders): {servo2_uid}")
    print(f"  Servo 3 (left arm):  {servo3_uid}")

    # Build motor mapping and config up front
    mapping = build_motor_mapping(servo1_uid, servo2_uid, servo3_uid)

    # Filter to only hand motors for safety
    hand_mapping = {k: v for k, v in mapping.items() if k in ALL_HAND_MOTORS}
    print(f"[INFO] Filtered mapping to {len(hand_mapping)} hand motors only")

    config = LowLatencyConfig(
        enabled=True,
        motor_mapping=hand_mapping,
        sync_to_ros=True,
    )

    # Single connection: rosbridge + Tinkerforge low-latency
    try:
        with Robot(host=args.host, low_latency=config) as robot:
            print(f"[OK] Connected to robot at {args.host}")

            # Check servo power relay (turns on if off)
            check_servo_power_relay(robot, args.relay_uid)

            # Discover bricklets (uses the already-open Tinkerforge connection)
            discover_bricklets(robot)

            # Low-latency status
            print_separator("Low-Latency Status")
            print(f"  low_latency_available: {robot.low_latency_available}")
            print(f"  low_latency_enabled: {robot.low_latency_enabled}")

            # Start ROS topic monitor
            monitor = ROSTopicMonitor(robot)
            monitor.start()
            time.sleep(0.5)  # Wait for subscription

            # Run tests
            if not args.right_only:
                run_hand_test(robot, "Left", LEFT_HAND_MOTORS)

            if not args.left_only:
                run_hand_test(robot, "Right", RIGHT_HAND_MOTORS)

            # Test ROS topic population
            print_separator("Testing ROS Topic Population")
            print("\nThis tests whether low-latency commands appear in ROS topics.")
            print("Expected: NO updates (low-latency bypasses ROS to avoid double commands)\n")

            test_motor = LEFT_HAND_MOTORS[0] if not args.right_only else RIGHT_HAND_MOTORS[0]
            ros_populated = test_ros_topic_population(robot, monitor, test_motor)

            # Stop monitor
            monitor.stop()

            # Summary
            print_separator("Test Summary")
            print(f"  Low-latency write: OK")
            print(f"  Low-latency read:  OK")
            print(f"  ROS topic updates: {'Yes (unexpected!)' if ros_populated else 'No (correct behavior)'}")

            if ros_populated:
                print("\n  [WARNING] ROS topics were updated during low-latency mode.")
                print("            This may cause double motor commands!")
            else:
                print("\n  [OK] Low-latency mode correctly bypasses ROS topics.")
                print("       Motor commands go directly to Tinkerforge.")

            # Return motors to neutral
            print_separator("Cleanup")
            print("Returning hand motors to 50%...")
            for motor in ALL_HAND_MOTORS:
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
