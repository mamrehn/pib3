# Robot ROS Interface

What `RealRobotBackend` exchanges with the robot, for anyone who debugs pib3 or
talks to the robot without it. Everything goes through rosbridge, the robot's
WebSocket bridge to ROS 2: `ws://<robot-ip>:9090`. The names and types below are
those of pib-backend's b3 fork (`mamrehn/pib-backend`, branch `develop`).

Service types work in the short form (`datatypes/ApplyJointTrajectory`) and the
long form (`datatypes/srv/ApplyJointTrajectory`); rosbridge resolves both.

## Joints

| Name | Kind | Type | pib3 uses it for |
|------|------|------|------------------|
| `/apply_joint_trajectory` | service | `datatypes/srv/ApplyJointTrajectory` | moving joints in `motor_mode="ros"` |
| `/joint_trajectory` | topic, robot → client | `trajectory_msgs/msg/JointTrajectory` | joint positions (`get_joint`) |
| `/get_joint_position` | service | `datatypes/srv/GetJointPosition` | not used |
| `/motor_settings` | topic, robot → client | `datatypes/msg/MotorSettings` | the settings cache |
| `/apply_motor_settings` | service | `datatypes/srv/ApplyMotorSettings` | changing motor settings |
| `/motor_current` | topic, robot → client | `diagnostic_msgs/msg/DiagnosticStatus` | not used |

### Units

Positions are **centidegrees** (1/100 degree): `9000` is 90°, `-4500` is −45°.
pib3 converts from radians or percent before it sends anything; see
[Unit Conventions](units.md).

### Moving joints

`/apply_joint_trajectory` takes a `trajectory_msgs/JointTrajectory` and answers
`bool successful`. The robot reads the message in one of two layouts:

1. **One point per joint** (what pib3 sends): `joint_names[i]` goes to
   `points[i].positions[0]`, and every `time_from_start` is zero. Each motor
   moves to its target at its own configured speed (`/motor_settings`); the
   message's `velocities` are ignored.
2. **Timed trajectory**: any point with a non-zero `time_from_start`, or several
   waypoints that each carry a position per joint. The robot interpolates every
   joint smoothly through the waypoints at 100 Hz.

!!! warning "One point with several positions moves one joint"
    A single point that carries all positions, with `time_from_start` zero, is
    read as layout 1: only the first joint moves, and the others are dropped
    without an error. `joint_trajectory_message()` in `pib3/backends/robot.py`
    builds layout 1 for this reason.

```python
from pib3.backends.robot import joint_trajectory_message

# Head turn to 45°, right elbow to 90°, in one request (layout 1)
request = joint_trajectory_message(
    ["turn_head_motor", "elbow_right"], [4500, 9000]
)
```

### Reading positions

The robot publishes one `/joint_trajectory` message per motor after each
command it applied: `joint_names` holds the motor's name and
`points[0].positions[0]` its position in centidegrees. Publishing to
`/joint_trajectory` yourself moves nothing; only `/apply_joint_trajectory`
moves motors. `/get_joint_position` (`string joint_name` → `bool successful`,
`int32 position`, `string message`) asks for one motor on demand.

### Motor current

`/motor_current` carries one `DiagnosticStatus` per motor and reading: `name` is
the motor, `values[0].value` its current in mA as a string, and `level` turns
`WARN` from 1500 mA. The robot publishes it once per second and whenever the
servo bricklet reports a change.

### Motor settings

`/apply_motor_settings` takes a `MotorSettings` and answers
`bool settings_applied` and `bool settings_persisted`. The fields:
`motor_name`, `turned_on`, `pulse_width_min`, `pulse_width_max`,
`rotation_range_min`, `rotation_range_max` (centidegrees), `velocity`,
`acceleration`, `deceleration` (centidegrees per second, per second²),
`period`, `visible` and `invert`. The robot publishes the result on
`/motor_settings`.

### Motor names

The 26 motors match `pib3.Joint`; the two `all_fingers_*` names move every
finger motor of one hand at once.

| Group | Motor names |
|-------|-------------|
| Head | `turn_head_motor`, `tilt_forward_motor` |
| Left arm | `shoulder_vertical_left`, `shoulder_horizontal_left`, `upper_arm_left_rotation`, `elbow_left`, `lower_arm_left_rotation`, `wrist_left` |
| Left hand | `thumb_left_opposition`, `thumb_left_stretch`, `index_left_stretch`, `middle_left_stretch`, `ring_left_stretch`, `pinky_left_stretch`, `all_fingers_left` |
| Right arm | `shoulder_vertical_right`, `shoulder_horizontal_right`, `upper_arm_right_rotation`, `elbow_right`, `lower_arm_right_rotation`, `wrist_right` |
| Right hand | `thumb_right_opposition`, `thumb_right_stretch`, `index_right_stretch`, `middle_right_stretch`, `ring_right_stretch`, `pinky_right_stretch`, `all_fingers_right` |

## Audio

Audio topics and services are described in the
[Audio System](../api/audio.md#ros-topics-and-services) reference.

## Camera and AI models

The model services (`/list_models`, `/start_model`, `/stop_model`,
`/get_detections`) and the `detections/<model_id>` topics are described in
[AI & Camera Subsystems](../api/ai-camera-subsystems.md).

!!! note "Service calls that take longer than 5 s"
    rosbridge abandons a service call after 5 s unless the `call_service`
    message names a `timeout`. roslibpy never sends one, so pib3 builds the
    message itself for `/start_model` and `/stop_model`, where the camera
    answers only after rebuilding its pipeline.
