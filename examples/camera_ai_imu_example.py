#!/usr/bin/env python3
"""
Example: Camera, AI Detection, and IMU usage with pib3.

This script demonstrates:
1. Camera streaming
2. AI object detection with model switching (on-demand)
3. IMU sensor data access
4. Vision-based head tracking

Requirements:
    pip install "pib3 @ git+https://github.com/mamrehn/pib3.git"
    pip install opencv-python numpy

Usage:
    python camera_ai_imu_example.py --host 172.26.34.149

    # Run specific demos
    python camera_ai_imu_example.py --demo camera
    python camera_ai_imu_example.py --demo ai
    python camera_ai_imu_example.py --demo imu
    python camera_ai_imu_example.py --demo tracking

Note:
    This example requires the physical robot with OAK-D Lite camera.
    Camera/AI/IMU features are not available in Webots simulation yet.
"""

import argparse
import time
from typing import Optional

import numpy as np

import pib3

# Optional: OpenCV for display
try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False
    print("Note: OpenCV not installed. Display features disabled.")
    print("Install with: pip install opencv-python")

# Import pib3 - will fail gracefully if not installed
try:
    from pib3 import AIModel, Robot, Joint
    HAS_PIB3 = True
except ImportError:
    HAS_PIB3 = False


def demo_camera_streaming(robot, duration: float = 10.0):
    """Demonstrate camera streaming."""
    print("\n=== Camera Streaming Demo ===")
    print("Streaming camera for", duration, "seconds...")

    frame_count = 0
    start_time = time.time()

    def on_frame(jpeg_bytes):
        nonlocal frame_count
        frame_count += 1

        if HAS_CV2:
            # Decode JPEG to numpy array
            img_array = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)

            if frame is not None:
                # Add frame counter overlay
                cv2.putText(
                    frame,
                    f"Frame: {frame_count}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow("pib3 Camera", frame)
                cv2.waitKey(1)
        else:
            if frame_count % 30 == 0:
                print(f"  Received frame {frame_count}, size: {len(jpeg_bytes)} bytes")

    # Subscribe to camera (streaming starts automatically)
    sub = robot.subscribe_camera_image(on_frame)

    try:
        time.sleep(duration)
    finally:
        # Unsubscribe (streaming stops automatically)
        sub.unsubscribe()
        if HAS_CV2:
            cv2.destroyAllWindows()

    elapsed = time.time() - start_time
    fps = frame_count / elapsed
    print(f"Received {frame_count} frames in {elapsed:.1f}s ({fps:.1f} FPS)")


def demo_ai_detection(robot, duration: float = 15.0):
    """Demonstrate AI object detection."""
    print("\n=== AI Detection Demo ===")
    print("Running AI detection for", duration, "seconds...")

    # First, check which models the robot offers
    print("\nQuerying available models...")
    models = robot.ai.available_models()
    if models:
        print("Available models:")
        for info in models:
            state = "running" if info.active else ("ready" if info.available else "not installed")
            print(f"  - {info.name}: {info.task} ({state})")
    else:
        print("  (No answer from /list_models)")

    # Start YOLO26n; this returns when the robot reports it running (a few s).
    if not robot.ai.set_model(AIModel.YOLO26S):
        print("The robot did not start the model; see the warning above.")
        return

    seen = 0
    end = time.time() + duration
    while time.time() < end:
        # latest_only: what the camera sees now, not every frame since the last call
        detections = robot.ai.get_detections(timeout=1.0, latest_only=True)
        if detections:
            seen += 1
            print(f"\nFPS {robot.ai.fps:.1f}, {len(detections)} object(s):")
            for det in detections:
                x1, y1, x2, y2 = det.bbox.to_pixels(1280, 720)
                print(f"  → {det.label} ({det.confidence:.2f}) at ({x1},{y1})-({x2},{y2})")
        time.sleep(0.5)

    robot.ai.stop()   # releases the model; depth comes back
    print(f"\nSaw objects in {seen} polls")


def demo_imu_data(robot, duration: float = 5.0):
    """Demonstrate IMU sensor data access."""
    print("\n=== IMU Data Demo ===")
    print("Reading IMU data for", duration, "seconds...")

    # The camera publishes the IMU at a fixed 100 Hz; there is nothing to set.
    sample_count = 0

    def on_imu(data):
        nonlocal sample_count
        sample_count += 1

        # Full IMU data includes both accelerometer and gyroscope
        accel = data.get('linear_acceleration', {})
        gyro = data.get('angular_velocity', {})

        if sample_count % 50 == 0:  # Print every 0.5 seconds at 100Hz
            print(f"Sample {sample_count}:")
            print(f"  Accel: x={accel.get('x', 0):7.3f}, "
                  f"y={accel.get('y', 0):7.3f}, "
                  f"z={accel.get('z', 0):7.3f} m/s²")
            print(f"  Gyro:  x={gyro.get('x', 0):7.4f}, "
                  f"y={gyro.get('y', 0):7.4f}, "
                  f"z={gyro.get('z', 0):7.4f} rad/s")

    # Subscribe to full IMU data
    sub = robot.subscribe_imu(on_imu, data_type="full")

    try:
        time.sleep(duration)
    finally:
        sub.unsubscribe()

    print(f"\nReceived {sample_count} IMU samples")


def demo_person_tracking(robot, duration: float = 30.0):
    """Demonstrate vision-based person tracking with head movement."""
    print("\n=== Person Tracking Demo ===")
    print("Tracking persons for", duration, "seconds...")
    print("The robot will turn its head to follow detected persons.")

    # YOLO26n detects the 80 COCO classes, "person" among them
    print("Starting the YOLO26n model...")
    if not robot.ai.set_model(AIModel.YOLO26S):
        print("The robot did not start the model; see the warning above.")
        return

    try:
        # Center head initially
        robot.set_joint(Joint.TURN_HEAD, 50)
        end = time.time() + duration
        while time.time() < end:
            # Only the newest frame: an older one would point the head at
            # where the person was.
            people = [d for d in robot.ai.get_detections(timeout=1.0, latest_only=True)
                      if d.label == "person" and d.confidence > 0.5]
            if people:
                center_x = max(people, key=lambda d: d.confidence).bbox.center[0]

                # Map center_x (0-1) to head position (0-100%)
                # Invert: person on left → head turns left (higher %)
                head_pos = (1.0 - center_x) * 40 + 30  # Range: 30-70%
                robot.set_joint(Joint.TURN_HEAD, head_pos)
                print(f"Person at x={center_x:.2f} → head at {head_pos:.0f}%")
            time.sleep(0.1)
    finally:
        robot.ai.stop()
        # Return head to center
        robot.set_joint(Joint.TURN_HEAD, 50)

    print("Tracking complete.")


def main():
    parser = argparse.ArgumentParser(
        description="pib3 Camera, AI, and IMU Example",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python camera_ai_imu_example.py --host 172.26.34.149
  python camera_ai_imu_example.py --demo camera --duration 20
  python camera_ai_imu_example.py --demo ai
  python camera_ai_imu_example.py --demo tracking
        """
    )

    print(f'piB3 version: {pib3.__version__}')

    host_michael = '172.26.34.222', 'Michael'
    host_wolfgang = '172.26.46.47', 'Wolfgang'
    host_richard = '172.26.30.35', 'Richard'
    host_martin = '172.26.34.149', 'Martin'  # robot with arms

    host_ip, host_name = host_wolfgang

    print(f'I choose you "{host_name}"!')

    parser.add_argument(
        "--host",
        default=host_ip,
        help=f"Robot IP address (default: {host_ip})"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9090,
        help="Rosbridge port (default: 9090)"
    )
    parser.add_argument(
        "--demo",
        choices=["camera", "ai", "imu", "tracking", "all"],
        default="all",
        help="Which demo to run (default: all)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=10.0,
        help="Duration for each demo in seconds (default: 10)"
    )
    args = parser.parse_args()

    # Check pib3 is installed
    if not HAS_PIB3:
        print("Error: pib3 not installed.")
        print("Install with: pip install 'pib3 @ git+https://github.com/mamrehn/pib3.git'")
        return

    print(f"Connecting to robot at {args.host}:{args.port}...")

    try:
        with Robot(host=args.host, port=args.port) as robot:
            print(f"Connected: {robot.is_connected}")

            if args.demo in ("camera", "all"):
                demo_camera_streaming(robot, args.duration)

            if args.demo in ("ai", "all"):
                demo_ai_detection(robot, args.duration)

            if args.demo in ("imu", "all"):
                demo_imu_data(robot, min(args.duration, 5.0))

            if args.demo in ("tracking", "all"):
                demo_person_tracking(robot, args.duration)

            print("\n=== All demos complete ===")

    except ConnectionError as e:
        print(f"Connection failed: {e}")
        print("\nTroubleshooting:")
        print("1. Check robot is powered on and connected to network")
        print("2. Verify rosbridge_server is running on the robot")
        print(f"3. Confirm IP address is correct: {args.host}")
    except KeyboardInterrupt:
        print("\nInterrupted by user")


if __name__ == "__main__":
    main()
