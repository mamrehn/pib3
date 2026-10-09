#!/usr/bin/env python3
"""
AI Model Demo

Runs AI models on the OAK-D Lite of the robot (or on the simulated camera in
Webots):
1. Listing the models the robot offers
2. Object detection (YOLO26n)
3. Body pose (YOLO26n-pose)
4. Hand landmarks
5. Two models at once
6. Timing a model switch

A model runs on the robot while any client holds it. ``robot.ai.set_model``
asks the robot to run one model and waits until it reports it running; every
start or stop rebuilds the camera pipeline, which takes a few seconds.

Requirements:
    pip install "pib3 @ git+https://github.com/mamrehn/pib3.git"

Usage:
    python model_switching_demo.py --host 172.26.34.149
    python model_switching_demo.py --demo list
    python model_switching_demo.py --demo detection
    python model_switching_demo.py --demo together

Note:
    The robot's model store must contain the models (the YOLO26 ones are not
    in the stock store yet); `--demo list` shows what it offers.
"""

import argparse
import time

try:
    from pib3 import AIModel, Robot
    HAS_PIB3 = True
except ImportError:
    HAS_PIB3 = False


def demo_list_models(robot):
    """List the models in the robot's model store."""
    print("\n=== Models of the robot ===")

    models = robot.ai.available_models()
    if not models:
        print("No answer from /list_models")
        return

    print(f"{'model':36s} {'task':22s} {'cores':>5s}  state")
    for info in models:
        state = "running" if info.active else ("ready" if info.available else "not installed")
        print(f"{info.name:36s} {info.task:22s} {info.shaves:5d}  {state}")


def describe_detection(det):
    cx, cy = det.bbox.center
    return f"{det.label} ({det.confidence:.2f}), box center ({cx:.2f}, {cy:.2f})"


def describe_pose(pose):
    return (f"person, nose at ({pose.nose.x:.2f}, {pose.nose.y:.2f}), "
            f"left shoulder at ({pose.left_shoulder.x:.2f}, {pose.left_shoulder.y:.2f})")


def describe_hand(hand):
    return (f"{hand.handedness.value} hand, index {hand.finger_angles.index:.0f}°, "
            f"middle {hand.finger_angles.middle:.0f}°")


def demo_model(robot, model, read, describe, title, duration):
    """Run ``model``, then print what ``read`` finds in the newest frame twice a second."""
    print(f"\n=== {title} ===")
    started = time.time()
    if not robot.ai.set_model(model):
        print("  The robot did not start the model (see the warning above).")
        return
    print(f"  Running after {time.time() - started:.1f} s")

    polls = 0
    end = time.time() + duration
    while time.time() < end:
        items = read(timeout=1.0, latest_only=True)   # the newest frame only
        if items:
            polls += 1
            for item in items:
                print(f"  {describe(item)}")
        time.sleep(0.5)
    print(f"  {polls} polls with results, {robot.ai.fps:.1f} results/s")


def demo_together(robot, duration):
    """Run two models at once; the camera has 16 cores to share."""
    print("\n=== YOLO26n and pose together ===")
    robot.ai.set_model(AIModel.YOLO26S)
    if not robot.ai.start_model(AIModel.POSE_YOLO):      # keeps YOLO26n running
        print("  The robot did not start the second model.")
        return
    time.sleep(duration)
    for model in (AIModel.YOLO26S, AIModel.POSE_YOLO):
        dets = robot.ai.get_detections(timeout=1.0, latest_only=True, model=model)
        print(f"  {model.value}: {len(dets)} detection(s) in the newest frame")
    robot.ai.stop_model(AIModel.POSE_YOLO)


def demo_switch_timing(robot):
    """Time a switch. Each one is a stop and a start of the camera pipeline."""
    print("\n=== Switch timing ===")
    for model in (AIModel.YOLO26S, AIModel.POSE_YOLO, AIModel.HAND, AIModel.YOLO26S):
        started = time.time()
        ok = robot.ai.set_model(model)
        print(f"  {model.value:30s} {'ok' if ok else 'FAILED':6s} {time.time() - started:5.1f} s")


def main():
    parser = argparse.ArgumentParser(
        description="AI Model Demo",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python model_switching_demo.py --host 172.26.34.149
  python model_switching_demo.py --demo list
  python model_switching_demo.py --demo detection --duration 10
        """
    )
    parser.add_argument("--host", default="172.26.34.149",
                        help="Robot IP address (default: 172.26.34.149)")
    parser.add_argument("--port", type=int, default=9090,
                        help="Rosbridge port (default: 9090)")
    parser.add_argument("--demo",
                        choices=["list", "detection", "pose", "hand", "together", "switch", "all"],
                        default="all", help="Which demo to run (default: all)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Duration for each model test in seconds (default: 5)")
    args = parser.parse_args()

    if not HAS_PIB3:
        print("Error: pib3 not installed.")
        print("Install with: pip install 'pib3 @ git+https://github.com/mamrehn/pib3.git'")
        return

    print(f"Connecting to robot at {args.host}:{args.port}...")
    with Robot(host=args.host, port=args.port) as robot:
        print("Connected.")
        try:
            if args.demo in ("list", "all"):
                demo_list_models(robot)
            if args.demo in ("detection", "all"):
                demo_model(robot, AIModel.YOLO26S, robot.ai.get_detections,
                           describe_detection, "Object detection", args.duration)
            if args.demo in ("pose", "all"):
                demo_model(robot, AIModel.POSE_YOLO, robot.ai.get_poses,
                           describe_pose, "Body pose", args.duration)
            if args.demo in ("hand", "all"):
                demo_model(robot, AIModel.HAND, robot.ai.get_hand_landmarks,
                           describe_hand, "Hand landmarks", args.duration)
            if args.demo in ("together", "all"):
                demo_together(robot, args.duration)
            if args.demo in ("switch", "all"):
                demo_switch_timing(robot)
        finally:
            robot.ai.stop()          # release every model this script started
    print("\nDone.")


if __name__ == "__main__":
    main()
