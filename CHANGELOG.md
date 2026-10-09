# Changelog

## Unreleased

### pib-backend's b3 fork: YOLO26s by default, keypoint confidence

pib3 now targets `mamrehn/pib-backend`, branch `b3-develop`: pib-rocks'
`develop` plus the YOLO26 models, keypoint confidences, a latency fix for the
YOLO detectors and the `/audio_playback` topic. Its model release carries the
blobs.

- **YOLO26s is the default.** `AIModel.YOLO26S` (detection) and
  `AIModel.POSE_YOLO` (pose, also `POSE_YOLO26S`) are the small models:
  COCO mAP 48.6 / 63.0 against 40.9 / 57.2 for nano, at about 12 results/s
  on the camera. `AIModel.YOLO26N` and the new `AIModel.POSE_YOLO26N` are the
  twice-as-fast nano fallbacks. **`POSE_YOLO` changed model** (nano to
  small). Old nano names (`"yolov6n"`, `"yolov8n"`, ...) map to `YOLO26N`, the
  others to `YOLO26S`.
- **`Keypoint.confidence` is real.** The robot sends `keypoint_score` per
  keypoint and the simulation passes ultralytics' values on; until now every
  keypoint read 1.0, so a check like "is the wrist visible?" always passed.
  A robot whose backend predates the field still reads 1.0.
- **Withdrawn upstream:** `AIModel.FACE_MESH` and `AIModel.FACE_LANDMARKS`
  are gone; pib-backend took both models off its camera list (PR-1957).
- **A crashed script's model is released.** `set_model()` also releases this
  owner's hold on any other running model, and `stop_model(model)` asks the
  robot even for a model this run did not start. The robot keeps a hold until
  its owner releases it.
- **The default owner names the machine:** `pib3-<user>@<computer>-<hash of
  the network card>`, so virtual machines cloned from one image no longer
  share (and stop) each other's models.
- **A failed restart keeps a held model.** A second `start_model()` of a
  running model that fails (for example, the connection drops) no longer
  forgets the model while the robot keeps running it.
- **Simulation matches the robot's threshold:** detections below 0.5
  confidence are dropped, as in the robot's YOLO archives (was 0.25).
- Polling `/list_models` after a slow start no longer logs a warning per poll.

### Robot AI, camera and IMU follow pib-backend `develop` (breaking)

The camera node of pib-rocks' `pib-backend` `develop` has its own model
interface (a model store, `/start_model` with an owner, one `DetectionArray`
topic per model). pib3 used the interface of an older backend branch
(`ai_cam_topics`); it now speaks the upstream one. Nothing here is released.

- **`robot.ai` runs models by id.** `AIModel` values are the model ids of the
  store: `YOLO26S`, `YOLO26N`, `POSE_YOLO`, `POSE_YOLO26N`, `HAND`, `FACE`,
  `EMOTION`, `HEAD_POSE`, `QR_CODE`; any other id works as a string.
  `set_model()` stops this client's other models and starts the new one;
  `start_model()` / `stop_model()` run several at once; `models`,
  `available_models()` and a `model=` argument on the getters are new. Each
  start or stop rebuilds the camera pipeline (seconds).
- **Models are shared by owner.** A model runs while any client holds it, so
  scripts, cerebra and the web programs no longer switch each other's models.
  The owner is `pib3-<user>@<computer>-<machine>` (`Robot(ai_owner=...)`),
  stable on purpose so a crashed script's model is released by its next run.
  `disconnect()` releases every model.
- **A start that outlasts rosbridge's service timeout** (about 5 s, a rebuild
  takes longer) is no longer reported as failed: pib3 asks `/list_models`.
- **`Detection` carries keypoints and scalars** (`det.keypoints[i].name`,
  `det.scalars["yaw"]`), so face, emotion, head-pose and QR models come back as
  detections. Coordinates are normalized from the pixels the robot sends.
- **Finger angles are measured in pixel space.** On a 16:9 frame a 90° bend
  read 121° from normalized coordinates.
- **Simulation parity.** `sim.ai` has the same methods and runs models
  together. A failed `set_model` no longer leaves the old model selected with
  nothing behind it.
- **Removed**, with the replacement: `switch_ai_model()` →
  `start_ai_model()`; `set_ai_config()` (confidence and segmentation mode are
  baked into the model archive; the robot has no segmentation);
  `subscribe_current_ai_model()` → `subscribe_ai_status()`;
  `subscribe_camera_legacy()` / `subscribe_camera_errors()` (the topics are
  gone); `parse_ai_result()` → `parse_detection_message()`; `AiModelType`;
  `set_imu_frequency()` raises (the IMU streams at a fixed 100 Hz).
  `subscribe_ai_detections(callback)` is now
  `subscribe_ai_detections(model, callback)` and delivers `DetectionArray`
  messages. Old model names (`"yolov6n"`, `"yolo26n"`, `"pose"`, `"hand"`, ...)
  are remapped with a `DeprecationWarning`.
- **Camera and IMU topics** are the node's own: `/camera_topic`, `/imu`,
  `/quality_factor_topic`, `/timer_period_topic`, `/size_topic`. `"full"` IMU
  data is the whole `sensor_msgs/Imu` message. The frame is 1280×720; keep any
  resolution 16:9.
- **Depth** now exists only while no model runs; the docs say so.

What this needs and what is not verified:

- The robot must run pib-backend `develop`. The YOLO26 models are not in its
  model store; a branch that adds them is prepared, and a robot without them
  answers `set_model(AIModel.YOLO26N)` with `False` and a list of what it has.
- Nothing here was run against a robot's camera node. Tested: the parsers with
  messages built by the backend's own code, the service and topic calls with a
  fake rosbridge, the models on an OAK-D Lite on a laptop. The hand chain
  `hand_tracking_mp` (`AIModel.HAND`) has no published speed; its older
  sibling `hand_tracking` measured 1.0 result/s on a robot.

## 0.2.0 (2026-10-05)

### Emergency stop that works on every laptop

- **The old Numpad-0 stop never fired.** `KeyCode.from_vk(96)` is a Windows
  key code, and pynput also compares the scan code (Windows) or X11 symbol
  (Linux), so it never matched a real key press. On macOS it matched **F5**
  instead. Keys are now matched per platform (`pib3.safety.key_names`).
- **Default keys: Space, Esc, Numpad-0, Pause.** Every laptop has Space and
  Esc.
- **`stop()` now stops.** It used to set a flag that only trajectories and
  sequences checked. A blocking `set_joint()`, `go_home()` or any
  fire-and-forget command kept moving, and later commands still went out.
  Now `stop()` freezes every servo where it is (deceleration briefly 0, so the
  bricklet does not brake through a 75° overshoot) and **latches**: further
  motion commands raise `pib3.EmergencyStopError` until `resume()`. Running
  waits, trajectories and sequences end at once.
- **No toggle.** Pressing the key again no longer resumes. A panicked double
  press used to restart the robot.
- **Ctrl+C freezes the robot** before the usual `KeyboardInterrupt`. Works
  everywhere, no permissions needed.
- **On-screen STOP window**, its own process, always on top. It opens when
  the stop arms and is the visible sign that it is armed; it lists the
  triggers that work on this computer (click, Space, Esc, Ctrl+C; in Webots
  "Space in the 3D view"), turns grey after a stop, and follows the system
  language (German/English, `PIB3_LANG`). It takes no focus and ignores
  Enter. `stop_button="auto"` shows it only where the keys cannot work.
- **Teacher's remote stop:** `pib3-estop --host <robot> [--host ...]
  [--window] [--relax]`. It freezes the servos via the robot's Tinkerforge
  daemon and latches the stop in every pib3 program connected to that robot.
- **The stop arms itself with a program's first motion command.** A program
  that only reads the camera (a camera station, or a perception script next to
  a robot another group drives) never grabs keys, and its Ctrl+C or crash does
  not freeze someone else's arm.
- **One stop halts the whole robot.** Every pib3 program connected to a robot
  latches when any of them stops (two groups drive the two arms of one pib).
  Programs that never moved latch without freezing again.
- **Webots has the stop too, for practice:** Space while the 3D view has focus,
  read from Webots' own keyboard device, so typing in the editor while the
  simulation runs never stops it and no permission is needed. Webots does not
  pass Esc to controllers (measured), so Space is the key in both worlds.
- A crash or Ctrl+C inside `with Robot(...)` freezes the motors, if this
  program moved them. Before, the bricklets finished the last move after the
  program had died.
- Ctrl+C keeps its meaning: the handler freezes the motors and then passes
  the signal on, so `KeyboardInterrupt` cancels the program as before (tested
  with a real SIGINT). Programs that never moved the robot are untouched, and
  the key hook never listens for Ctrl+C.

### Real robot

- **ROS motor mode moved only one joint per trajectory waypoint.** The
  robot's `motor_control` node zips `joint_names` with one point per joint;
  pib3 sent one point carrying all positions. Fixed
  (`joint_trajectory_message`). `set_joints()` in ROS mode now sends one
  request instead of one per joint (`go_home()`: 1 instead of 26 round trips).
- **Direct mode follows the robot's own motor table** (pib-api
  `GET :5000/motor`): exact bricklet UID and pin per motor (no stack-position
  heuristic, no hard-coded UIDs of one robot), the `invert` flags and the
  rotation ranges, applied exactly like the robot's ROS node does. Before,
  an inverted motor moved the other way than in Cerebra, and direct mode
  ignored the robot's ranges (e.g. head tilt ±45°). Falls back to
  auto-discovery when pib-api is unreachable.
- **Speed no longer depends on history.** Every command sets its velocity
  (`speed=` or `robot.default_speed`, 150 deg/s). Before, the slow
  `go_home()` (10 deg/s) stuck to every later move, including trajectories.
- `speed=0` (to the bricklet: full speed) and negative speeds raise.
- Docs no longer claim `get_joint()` reads a potentiometer: it reads the
  bricklet's commanded ramp position.

### Simulation (Webots)

- **The twin moves like the robot:** 150 deg/s, ramped with 150 deg/s², and
  `speed=` is honoured. The proto's motors turned at 20 rad/s (≈ 1150 deg/s)
  with no ramp, about eight times faster than the robot, and `speed` was
  ignored. `WebotsBackend(realistic_motion=False)` restores the old motors.
- `get_joint(..., timeout=0)` reads at once without stepping, like the robot.
  Control loops must start from this measured position: with ramped motion,
  adding `K * error` to the last *command* every step winds up and loses the
  target (measured in the course world for K = 10 to 45).
- A stop from another thread (key, button) is applied on the controller's
  thread; Webots' API is not thread-safe.
- `step_ms` was never used; it is documented as such.

### Motion API (both backends)

- Unknown joint names raise with a suggestion (`'elbow_lft'` → "Did you mean
  'elbow_left'?"); enum names like `"TURN_HEAD"` or `"index_left"` work. A
  typo used to be reported as "not calibrated", or failed silently on ROS.
- Unknown units raise; `"degree"`, `"degrees"`, `"°"`, `"%"`, `"radians"` are
  accepted. `unit="degree"` used to be treated as percent without a word.
- NaN, infinity and non-numbers raise.
- Targets outside a joint's range are clamped to the limit (one hint per
  joint) and a blocking call returns `False`. The bricklet allows ±90° on
  every channel, so e.g. the elbow (−45°) was not protected.
- Blocking calls derive their timeout from speed and range. A fixed 2 s used
  to report slow moves (e.g. 90° at 30 deg/s) as failed.
- `run_trajectory()` first moves to the start pose at 30 deg/s and waits
  (`approach_speed=`), clamps out-of-range waypoints, and streams on a fixed
  schedule (`set_joints_sequence()` too). Before, the start of a drawing was
  smeared along the way to it, and command time stretched the playback.
- `get_joints(Joint.X)` with a single joint works.

### Perception, drawing, IK

- `get_detections()` returns segmentation results. The robot publishes them
  as `"instance-segmentation"`, which was filtered out.
- `image_to_sketch()` keeps the aspect ratio. Non-square images were squeezed
  into a square.
- Closed contours are drawn closed (a circle had a gap), and corner points are
  no longer duplicated.
- IK rejects solutions outside the motors' range. Its fallback attempts run
  without joint limits and could return e.g. an elbow past −45°.
- `MOTOR_GROUPS` / `DEFAULT_MOTOR_SETTINGS` live in `pib3.types`;
  `set_motor_settings()` no longer imports roboticstoolbox.

### Tests

163 tests, all without hardware (`tests/test_safety.py`,
`tests/test_backend_api.py` are new). `tests/test_low_latency.py` is a
hardware script and no longer collected. The STOP window was checked with a
real click under Xvfb, and the course self-test passes 6/6 in headless
Webots R2025a with realistic motion.

### Course material

Updated in the curriculum repository (see its `REDAKTION.md`, Durchgang 10):
the Notaus wording everywhere (page, poster, H5P, kurslib, plans), a
"practise the stop" check in the Webots self-test, week 3 reworked to
`kopf = ist + K * fehler` with a re-measured gain table, and the book chapters
and exam items that claimed `speed=` stays set on the servo.

### Known issues, not changed

- Head tilt is ±45° in both limit files now (was −45..+70°), matching the
  range the pib backend configures. The Webots proto still allows +70°.
- The calibration tool writes into the installed package, so a reinstall
  loses the calibration, and robots calibrated from one laptop share one
  file. Documented in the calibration guide; not changed.
- On connect, direct mode sets every servo channel to 150 deg/s and
  150 deg/s². That also changes Cerebra's speeds (fingers 1000 deg/s, upper
  arm rotation 100 deg/s) until the robot restarts.
- `get_detections()` returns every buffered frame unless `latest_only=True`.
- `mediapipe >= 1.0` removed `mp.solutions`, so the simulated hand model fails.
- Old python-build-standalone builds (uv's CPython 3.13.4/3.13.5 for Linux)
  link Tk 8.6 statically and abort on the first widget, so the STOP button
  cannot open there. Fixed upstream in later builds (3.13.15 ships Tk 9 and
  works); pib3 names the fix (`uv python upgrade`, recreate the venv).
- `joint_limits.yaml` and `robot_limits.yaml` are unused and contradict the
  files in use.
- The default host `172.26.34.149` is one particular robot.
- `speak()` defaults to German, `play_audio_from_speech()` to English.
