# Tests & Diagnostic Tools

This directory contains test scripts and diagnostic tools for **pib3 framework developers**.

These scripts are used to verify framework functionality and troubleshoot issues. They are not intended as user examples to build upon.

## Directory Structure

```
tests/
├── README.md                    # This file
├── diagnose_connection.py       # Connection troubleshooting tool
├── test_low_latency_hands.py    # Tests low-latency Tinkerforge mode
└── ...                          # Future test scripts
```

## Running Tests & Tools

### Connection Diagnostic

Troubleshoots connection issues to the robot:

```bash
python tests/diagnose_connection.py --host 172.26.34.149
python tests/diagnose_connection.py --host 172.26.34.149 --port 9090
```

Checks:
1. Network ping
2. TCP port connectivity
3. Rosbridge WebSocket protocol

### Low-Latency Mode Test

Tests direct Tinkerforge motor control (bypassing ROS):

```bash
# Auto-discover servo bricklet UIDs
python tests/test_low_latency_hands.py --host 172.26.34.149

# Specify UIDs manually
python tests/test_low_latency_hands.py --host 172.26.34.149 \
    --servo1-uid 29Fy --servo2-uid 29F5 --servo3-uid 29F3

# Test only one hand (safer)
python tests/test_low_latency_hands.py --host 172.26.34.149 --left-only
```

## For Users

If you're looking for **example code** to learn from and build upon, see the `examples/` directory instead:

```
examples/
├── basic_joint_control.py       # Joint control basics
├── camera_ai_imu_example.py     # Camera and sensor usage
├── audio_play_speak_and_record.py  # Audio features
├── trajectory_playback.py       # Trajectory generation & playback
├── scan_and_draw.py             # Full sense-plan-act demo
└── ...
```
