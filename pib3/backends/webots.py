"""Webots simulator backend for pib3 package."""

import logging
import math
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Sequence, Union

import numpy as np

logger = logging.getLogger(__name__)

from .base import RobotBackend
from .hints import hint
from ..safety import describe_keys


# Mapping from trajectory joint names to Webots motor device names
JOINT_TO_WEBOTS_MOTOR = {
    "turn_head_motor": "head_horizontal",
    "tilt_forward_motor": "head_vertical",
    "shoulder_vertical_left": "shoulder_vertical_left",
    "shoulder_horizontal_left": "shoulder_horizontal_left",
    "upper_arm_left_rotation": "upper_arm_left",
    "elbow_left": "elbow_left",
    "lower_arm_left_rotation": "forearm_left",
    "wrist_left": "wrist_left",
    "thumb_left_opposition": "thumb_left_opposition",
    "thumb_left_stretch": "thumb_left_distal",
    "index_left_stretch": "index_left_distal",
    "middle_left_stretch": "middle_left_distal",
    "ring_left_stretch": "ring_left_distal",
    "pinky_left_stretch": "pinky_left_distal",
    "shoulder_vertical_right": "shoulder_vertical_right",
    "shoulder_horizontal_right": "shoulder_horizontal_right",
    "upper_arm_right_rotation": "upper_arm_right",
    "elbow_right": "elbow_right",
    "lower_arm_right_rotation": "forearm_right",
    "wrist_right": "wrist_right",
    "thumb_right_opposition": "thumb_right_opposition",
    "thumb_right_stretch": "thumb_right_distal",
    "index_right_stretch": "index_right_distal",
    "middle_right_stretch": "middle_right_distal",
    "ring_right_stretch": "ring_right_distal",
    "pinky_right_stretch": "pinky_right_distal",
}

# Finger joints that have both proximal and distal motors in Webots.
# The real robot has a single motor per finger with a construction train
# (mechanical linkage) that bends both proximal and distal joints equally.
# To act as a faithful digital twin, Webots always commands both joints to
# the same position on write and reads only the distal sensor (since both
# joints are always identical).  Do NOT decouple these joints.
# Webots finger joints: 0° = open, 90° = closed (range 0 to π/2 radians)
FINGER_PROXIMAL_MOTORS = {
    "thumb_left_stretch": "thumb_left_proximal",
    "index_left_stretch": "index_left_proximal",
    "middle_left_stretch": "middle_left_proximal",
    "ring_left_stretch": "ring_left_proximal",
    "pinky_left_stretch": "pinky_left_proximal",
    "thumb_right_stretch": "thumb_right_proximal",
    "index_right_stretch": "index_right_proximal",
    "middle_right_stretch": "middle_right_proximal",
    "ring_right_stretch": "ring_right_proximal",
    "pinky_right_stretch": "pinky_right_proximal",
}


class WebotsBackend(RobotBackend):
    """
    Execute trajectories in Webots simulator.

    Must be instantiated from within a Webots controller script.

    Webots interprets motor positions as relative to the base position at
    simulation time zero. On connect, the backend reads each joint's initial
    position and stores it as a per-joint offset. All position commands are
    then converted from absolute radians to Webots-relative positions using
    these offsets. After ``_reset_to_zero()`` the offsets become 0.0 (since
    all joints are driven to absolute zero), so commanding 0 rad targets
    the proto-defined zero.

    When reading joint positions, the backend waits for motor readings to
    stabilize (same value twice) to ensure accurate readings when motors
    are in motion. Use the `timeout` parameter in get_joints() to control
    how long to wait (default: 5.0 seconds).

    By default the simulated motors move like the real ones: at most
    150 deg/s, ramped with 150 deg/s^2 (the servo bricklet settings pib3 uses
    on the robot), and ``speed=`` is honoured. The proto's own motors would
    otherwise turn at 20 rad/s (about 1150 deg/s) with no ramp, so code
    tuned in the simulator ran about eight times faster than on the robot.
    Pass ``realistic_motion=False`` for the old, instant behaviour.

    Example:
        # In your Webots controller file:
        from pib3.backends import WebotsBackend

        with WebotsBackend() as backend:
            backend.run_trajectory("trajectory.json")

            # Read joint positions (waits up to 5s for stabilization)
            joints = backend.get_joints()

            # Read with custom timeout
            joints = backend.get_joints(timeout=2.0)
    """

    #: Same motion limits as the real robot's servo bricklets
    #: (RealRobotBackend.DEFAULT_MOTION_VELOCITY / _ACCELERATION).
    DEFAULT_SPEED = 150.0          # deg/s
    DEFAULT_ACCELERATION = 150.0   # deg/s^2

    #: The emergency stop works in the simulator too, so it can be practised
    #: before the real arm. Keys come from Webots' own keyboard device, which
    #: only sees key presses while the 3D view has focus: typing in the editor
    #: while the simulation runs does not stop it, and no permission is needed.
    ESTOP_KEYS_DEFAULT = True
    DEFAULT_ESTOP_KEYS = ("space",)
    KEY_FOCUS_HINT = " (click into the 3D view first)"
    #: Webots starts controllers without a terminal.
    STOP_ON_CTRL_C = False
    #: The STOP window shows that the stop is armed, as on the robot.
    ESTOP_BUTTON_DEFAULT = True
    STOP_EFFECT = "sim"

    #: Key codes Webots R2025a delivers to controllers (measured): Space 32,
    #: Enter 4, Numpad-0 as Insert 6 (NumLock off) or '0', letters upper-case.
    #: Esc never reaches a controller, so Space is the stop key here, the
    #: same key that works on the real robot.
    _WEBOTS_KEY_CODES = {"space": (32,), "enter": (4,), "kp_0": (6, 48),
                         "insert": (6,)}

    def __init__(
        self,
        step_ms: int = 50,
        realistic_motion: bool = True,
        estop_keys: Union[bool, str, Sequence[str]] = True,
        stop_button: Union[bool, str] = True,
    ):
        """
        Initialize Webots backend.

        Args:
            step_ms: Unused; kept so old code keeps running. The waypoint
                rate is ``run_trajectory(rate_hz=...)``.
            realistic_motion: Move like the real robot (150 deg/s, ramped,
                ``speed=`` honoured). False restores the proto's instant
                motors (about 1150 deg/s, no ramp).
            estop_keys: Emergency-stop keys, armed by the first motion
                command: True (default) = Space and Esc in the 3D view; a key
                name or list for others; False for none (e.g. if your
                controller reads the keyboard itself).
            stop_button: Show the on-screen STOP button while the stop is
                armed (default True, as on the robot); False never.
        """
        super().__init__()
        self.step_ms = step_ms
        self.realistic_motion = realistic_motion
        if not realistic_motion:
            self._default_speed = None
            self.DEFAULT_ACCELERATION = None
        self._estop_keys_setting = estop_keys
        self._estop_button_setting = stop_button
        # Webots' controller API is not thread-safe: a stop from the key
        # listener's thread only marks the freeze, the main thread does it.
        self._pending_halt = False
        self._velocity_set: Dict[int, float] = {}
        self._keyboard = None
        self._estop_codes: Dict[int, str] = {}
        self._robot = None
        self._timestep = None
        self._motors: Dict[str, Any] = {}
        self._proximal_motors: Dict[str, Any] = {}
        # Perception subsystems (lazy, mirroring RealRobotBackend.camera/.ai)
        self._camera_subsystem = None
        self._ai_subsystem = None
        # How many blocking motor commands were issued (see diagnose()).
        self._blocking_calls = 0
        # Per-joint offsets: the initial position of each joint at simulation
        # start (base position at timepoint zero).  Webots setPosition() is
        # relative to this base, so we subtract the offset when commanding
        # and add it back when reading.
        self._home_offsets: Dict[str, float] = {}

    # ==================== SUBSYSTEM PROPERTIES ====================

    @property
    def camera(self):
        """
        RGB camera of the simulated robot — same contract as ``robot.camera``.

        The camera is mounted on the head in ``pib.proto``, so its view follows
        ``turn_head_motor`` and ``tilt_forward_motor``. That is what makes
        visual servoing in simulation a genuinely closed loop.

        Example:
            >>> frame = sim.camera.get_frame()
            >>> img = frame.to_numpy()      # BGR, ready for OpenCV
        """
        if self._camera_subsystem is None:
            from .webots_camera import WebotsCameraSubsystem
            self._camera_subsystem = WebotsCameraSubsystem(self)
        return self._camera_subsystem

    @property
    def ai(self):
        """
        AI perception for the simulated robot — same contract as ``robot.ai``.

        Defaults to Webots ground-truth ``Recognition`` (perfect, instant, no
        model). Call ``sim.ai.set_model(AIModel.YOLO26N)`` to run a real network on
        the simulated frames instead; ``AIModel.HAND`` and ``AIModel.POSE_YOLO`` work the same
        way (see :mod:`pib3.backends.sim_ai`).

        Example:
            >>> for det in sim.ai.get_detections():
            ...     print(det.label, det.bbox.center)
        """
        if self._ai_subsystem is None:
            from .webots_camera import WebotsAISubsystem
            self._ai_subsystem = WebotsAISubsystem(self)
        return self._ai_subsystem

    def set_joints(self, positions, unit="percent", async_=False,
                   timeout=None, tolerance=None, speed=None) -> bool:
        """Same as :meth:`RobotBackend.set_joints`, plus a control-loop check.

        ``async_=False`` steps the simulator until the joint arrives. That is
        right for a scripted choreography and wrong inside a perception loop:
        each command burns many steps, so the loop crawls and the camera looks
        frozen. The symptom reads as "the simulation hangs", which sends people
        hunting in entirely the wrong place — so count them and say so once.
        """
        if not async_:
            self._blocking_calls += 1
            if self._blocking_calls == self.BLOCKING_CALL_LIMIT:
                hint(
                    "blocking-in-loop",
                    f"{self.BLOCKING_CALL_LIMIT} motor commands so far have "
                    "waited for the joint to arrive (async_=False, the "
                    "default).\n"
                    "  Inside a loop that stalls the simulation: every command "
                    "burns many steps, so the camera seems frozen.\n"
                    "  Fix, in loops:\n"
                    "      sim.set_joint(Joint.TURN_HEAD, value, async_=True)\n"
                    "  Keep the blocking form for scripted sequences, where "
                    "waiting is exactly what you want.",
                )
        return super().set_joints(
            positions, unit=unit, async_=async_, timeout=timeout,
            tolerance=tolerance, speed=speed,
        )

    def _to_backend_format(self, radians: np.ndarray) -> np.ndarray:
        """Convert absolute radians to Webots-relative positions.

        After _reset_to_zero() all offsets are 0.0, so this is an identity.
        """
        # For trajectory playback, offsets are per-column (per joint).
        # After reset they are all 0 so this is effectively a no-op.
        return radians

    # Excursions smaller than this are floating-point residue, not intent:
    # clamp them silently. 1e-6 rad is 6e-5 degrees.
    POSITION_EPSILON_RAD = 1e-6

    @classmethod
    def _set_motor_position(cls, motor, value: float, name: str = "") -> None:
        """``motor.setPosition``, clamped to the motor's declared limits.

        Webots prints a console warning for any target outside
        ``[minPosition, maxPosition]``, however slightly. The offset
        arithmetic here (``target - home_offset``) routinely lands a few
        1e-10 past a limit — numerically zero, but enough to emit one warning
        per finger on every ``go_home()``, which buries real messages.

        Residue below ``POSITION_EPSILON_RAD`` is clamped silently; anything
        larger is clamped *and* logged, so a genuine out-of-range command
        still surfaces instead of being quietly swallowed.

        A motor with ``minPosition == maxPosition`` is unlimited in Webots
        and is left alone.
        """
        lo, hi = motor.getMinPosition(), motor.getMaxPosition()
        if lo != hi:
            clamped = min(max(value, lo), hi)
            overshoot = abs(clamped - value)
            if overshoot > cls.POSITION_EPSILON_RAD:
                logger.warning(
                    "Target %.4f rad for %s is outside [%.4f, %.4f]; "
                    "clamped to %.4f.",
                    value, name or "motor", lo, hi, clamped,
                )
            value = clamped
        motor.setPosition(value)

    def _from_backend_format(self, values: np.ndarray) -> np.ndarray:
        """Convert Webots-relative positions to absolute radians."""
        return values

    def connect(self) -> None:
        """Initialize Webots robot and motors."""
        try:
            from controller import Robot
        except ImportError:
            raise ImportError(
                "Webots controller module not found. "
                "This backend must be used from within a Webots controller script."
            )

        self._robot = Robot()
        self._timestep = int(self._robot.getBasicTimeStep())

        # Initialize all motors (only once)
        if len(self._motors) == 0:
            self._motors = {}
            
            for joint_name, motor_name in JOINT_TO_WEBOTS_MOTOR.items():
                motor = self._robot.getDevice(motor_name)
                if motor is not None:
                    self._motors[joint_name] = motor

                    # Enable once, forever. Readout will be available at each timestep.
                    sensor = motor.getPositionSensor()
                    if sensor is not None:
                        sensor.enable(self._timestep)
        
        # Initialize proximal finger motors (only once)
        if len(self._proximal_motors) == 0:
            self._proximal_motors = {}
            for joint_name, proximal_motor_name in FINGER_PROXIMAL_MOTORS.items():
                motor = self._robot.getDevice(proximal_motor_name)
                if motor is not None:
                    self._proximal_motors[joint_name] = motor
                    logger.debug(f"Initialized proximal motor: {joint_name} -> {proximal_motor_name}")
                    sensor = motor.getPositionSensor()
                    if sensor is not None:
                        sensor.enable(self._timestep)
                else:
                    logger.warning(f"Proximal motor not found: {proximal_motor_name}")

        # Read initial joint positions (base position at timepoint zero).
        # Webots setPosition() targets are relative to this base, so we
        # record the offsets before resetting to absolute zero.
        self._read_home_offsets()

        # Drive all motors to absolute zero, then clear offsets (since
        # the joints are now at 0 rad and positions become absolute).
        self._reset_to_zero()

        if self.realistic_motion:
            accel = math.radians(self.DEFAULT_ACCELERATION)
            for motor in self._all_motors():
                try:
                    motor.setAcceleration(accel)
                except Exception as exc:  # very old Webots builds
                    logger.debug("setAcceleration unavailable: %s", exc)
                    break

        self._activate_safety()

    def _all_motors(self):
        yield from self._motors.values()
        yield from self._proximal_motors.values()

    def _set_motor_velocity(self, motor, speed_deg_s: Optional[float]) -> None:
        """Motor speed for position control; None = the proto's maximum."""
        vmax = motor.getMaxVelocity()
        value = vmax if speed_deg_s is None else math.radians(speed_deg_s)
        if vmax and vmax > 0:
            value = min(value, vmax)
        if self._velocity_set.get(id(motor)) != value:
            motor.setVelocity(value)
            self._velocity_set[id(motor)] = value

    def _halt_motion(self) -> None:
        """Freeze every motor at its current sensor position."""
        if self._robot is None:
            return
        if threading.current_thread() is not threading.main_thread():
            self._pending_halt = True
            return
        self._pending_halt = False
        for name, motor in self._motors.items():
            sensor = motor.getPositionSensor()
            if sensor is None:
                continue
            here = sensor.getValue()
            self._set_motor_position(motor, here, name)
            if name in self._proximal_motors:
                self._set_motor_position(self._proximal_motors[name], here, name)

    def _apply_pending_halt(self) -> None:
        if self._pending_halt:
            self._halt_motion()

    # --- emergency-stop keys via Webots' keyboard ----------------------

    def _stop_button_title(self) -> str:
        return "Webots"

    def _stop_triggers(self):
        triggers = ["click"]
        if self._estop_keys and self._estop_keys_ok and "space" in self._estop_keys:
            triggers.append("space3d")
        return triggers

    def _start_estop_keys(self, names) -> bool:
        """Read the stop keys from Webots' keyboard device (3D view focus)."""
        if self._robot is None:
            return False
        codes: Dict[int, str] = {}
        for name in names:
            if name in self._WEBOTS_KEY_CODES:
                for code in self._WEBOTS_KEY_CODES[name]:
                    codes[code] = name
            elif len(name) == 1:
                codes[ord(name.upper())] = name
            else:
                logger.warning(
                    "Webots does not pass the key %r to controllers; use "
                    "\"space\" (default), \"enter\" or a letter.", name)
        if not codes:
            return False
        keyboard = self._robot.getKeyboard()
        keyboard.enable(int(self._timestep))
        self._keyboard = keyboard
        self._estop_codes = codes
        self._estop_keys = tuple(names)
        return True

    def disable_estop_key(self) -> None:
        """Stop reading the emergency-stop keys."""
        if self._keyboard is not None:
            try:
                self._keyboard.disable()
            except Exception:
                pass
        self._keyboard = None
        self._estop_codes = {}
        self._estop_keys = ()

    def _poll_estop_keys(self) -> None:
        """Check the keyboard for a stop key; runs on the controller's thread."""
        keyboard = self._keyboard
        if keyboard is None or self._stopped:
            return
        while True:
            key = keyboard.getKey()
            if key is None or key < 0:
                return
            name = self._estop_codes.get(key & 0xFFFF)
            if name is not None:
                self.stop(reason=f"{describe_keys([name])} key")
                return

    def _read_home_offsets(self) -> None:
        """Read and store each joint's initial position as its home offset.

        Must be called after sensors are enabled and before _reset_to_zero().
        The offsets represent the base position at simulation timepoint zero.
        """
        # Step once so sensors produce valid readings
        self._robot.step(self._timestep)

        self._home_offsets = {}
        for name, motor in self._motors.items():
            sensor = motor.getPositionSensor()
            if sensor is not None:
                pos = sensor.getValue()
                self._home_offsets[name] = pos
                if abs(pos) > 0.01:
                    logger.info(
                        f"Joint {name} initial offset: {pos:.4f} rad "
                        f"({pos * 180 / np.pi:.1f} deg)"
                    )
            else:
                self._home_offsets[name] = 0.0

    def _reset_to_zero(self) -> None:
        """Reset all motors to absolute zero and wait for convergence.

        Uses the stored home offsets to command the correct Webots-relative
        position that drives each joint to absolute 0 rad.  After convergence
        the offsets are cleared (set to 0.0) so that subsequent commands are
        in absolute coordinates.
        """
        # Log non-zero starting positions
        for name, offset in self._home_offsets.items():
            if abs(offset) > 0.01:
                logger.warning(
                    f"Joint {name} starts at {offset:.4f} rad "
                    f"({offset * 180 / np.pi:.1f} deg) — resetting to zero"
                )

        # Command all motors to absolute zero.
        # Webots target = absolute_target - home_offset = 0.0 - offset = -offset
        for name, motor in self._motors.items():
            offset = self._home_offsets.get(name, 0.0)
            self._set_motor_position(motor, -offset, name)
        for name, motor in self._proximal_motors.items():
            self._set_motor_position(motor, 0.0, name)

        # Step simulation until all motors converge (or timeout)
        tolerance = 0.01  # radians
        max_steps = int(5000 / self._timestep)  # 5 seconds max

        for _ in range(max_steps):
            self._robot.step(self._timestep)

            all_at_zero = True
            for name, motor in self._motors.items():
                sensor = motor.getPositionSensor()
                if sensor is not None:
                    # The sensor reads Webots-relative position;
                    # absolute = reading + home_offset.
                    offset = self._home_offsets.get(name, 0.0)
                    absolute_pos = sensor.getValue() + offset
                    if abs(absolute_pos) > tolerance:
                        all_at_zero = False
                        break

            if all_at_zero:
                logger.info("All motors reached zero position.")
                # Offsets are now consumed — joints are at absolute 0,
                # so future commands need no offset adjustment.
                self._home_offsets = {name: 0.0 for name in self._home_offsets}
                return

        logger.warning("Timeout waiting for motors to reach zero position.")
        # Clear offsets even on timeout so the system remains usable
        self._home_offsets = {name: 0.0 for name in self._home_offsets}

    def disconnect(self) -> None:
        """Release the emergency-stop hooks; the simulator owns the robot."""
        self._deactivate_safety()

    @property
    def is_connected(self) -> bool:
        """Check if robot is initialized."""
        return self._robot is not None

    #: Blocking motor commands before we point out that a control loop wants
    #: async_=True. A scripted choreography legitimately makes dozens of
    #: blocking moves, so this is set well above that.
    BLOCKING_CALL_LIMIT = 60

    def diagnose(self) -> str:
        """Print a snapshot of everything that usually goes wrong, and return it.

        Built for a classroom: when something "just doesn't work", drop
        ``sim.diagnose()`` into the controller and read eight lines instead of
        guessing. It never raises and never changes the simulation.

        Example:
            >>> with pib3.Webots() as sim:
            ...     sim.diagnose()
        """
        lines = ["", "=" * 70, "pib3 diagnose — simulated robot", "=" * 70]

        def row(label, value, fix=""):
            lines.append(f"  {label:<26} {value}")
            if fix:
                lines.append(f"  {'':<26} -> {fix}")

        row("connected", self.is_connected)
        row("time step", f"{self._timestep} ms" if self._timestep else "unknown")
        row("simulation time",
            f"{self._robot.getTime():.2f} s" if self.is_connected else "n/a",
            "0.00 s means you never called sim.step() — nothing can change"
            if self.is_connected and self._robot.getTime() <= 0 else "")

        cam = self._camera_subsystem
        if cam is None:
            row("camera", "never used",
                "sim.camera.get_frame() enables it on first use")
        else:
            row("camera device", "found" if cam.available else "MISSING",
                "" if cam.available else
                "no Camera node in the proto, or the world uses an old copy")
            if cam.available:
                row("resolution", f"{cam.width} x {cam.height}")
                row("frames served", cam.frame_count,
                    "0 means the camera never rendered — step the simulation"
                    if cam.frame_count == 0 else "")
                row("live view panel", "attached" if cam.display else "not attached",
                    "" if cam.display else
                    "sim.camera.show_on_display() shows it in the 3D view")

        ai = self._ai_subsystem
        if ai is None:
            row("ai", "never used", "sim.ai.set_model('recognition') to start")
        else:
            row("ai model", ai.model)
            try:
                n = len(ai.get_detections(latest_only=True))
            except Exception as exc:                       # never break diagnose
                n = f"error: {exc}"
            row("detections right now", n,
                "0 objects: is anything in view, and does that Solid set "
                "recognitionColors?" if n == 0 else "")

        row("blocking motor calls", self._blocking_calls,
            "high counts in a loop stall the simulation — use async_=True"
            if self._blocking_calls >= self.BLOCKING_CALL_LIMIT else "")
        lines += ["=" * 70, ""]

        text = "\n".join(lines)
        print(text)
        return text

    def step(self, duration_ms: Optional[int] = None) -> bool:
        """
        Advance the simulation by one time step.

        Motion calls (``set_joint``, ``run_trajectory``, …) step the simulator
        themselves, so most code never needs this. A **perception loop does**:
        the camera only renders a new image when simulated time moves forward.
        Without a ``step()`` the same frame is returned forever and the loop
        spins on stale data.

        Args:
            duration_ms: Milliseconds to advance. Defaults to the world's
                basic time step.

        Returns:
            True to keep going, False when Webots has asked the controller to
            terminate (window closed, simulation reset) — use it as the loop
            condition.

        Example:
            >>> with pib3.Webots() as sim:
            ...     while sim.step():
            ...         for det in sim.ai.get_detections():
            ...             sim.set_joint(Joint.TURN_HEAD, ..., async_=True)
        """
        if not self.is_connected:
            return False
        self._apply_pending_halt()
        ms = int(duration_ms if duration_ms is not None else self._timestep)
        alive = self._robot.step(ms) != -1
        self._poll_estop_keys()
        return alive

    # Default timeout for waiting for motor stabilization (seconds)
    DEFAULT_GET_JOINTS_TIMEOUT = 5.0

    def _get_joint_radians(
        self,
        motor_name: str,
        timeout: Optional[float] = None,
    ) -> Optional[float]:
        """
        Get current position of a single joint in absolute radians.

        Reads the Webots-relative sensor value and adds the home offset
        to return the absolute position.

        For finger joints, only the distal motor sensor is read.  The real
        robot uses a single motor with a construction train that bends both
        proximal and distal joints equally, so they always have the same
        angle.  Webots mirrors this by always commanding both joints to the
        same value (see ``_set_joints_impl``), making the distal reading
        sufficient.

        Args:
            motor_name: Name of motor (e.g., "elbow_left").
            timeout: Max time to wait for motor to stabilize (seconds).
                    If None, uses DEFAULT_GET_JOINTS_TIMEOUT (5.0s).
                    ``0`` reads the sensor at once without stepping, the way
                    the real robot answers: use it inside control loops.

        Returns:
            Current position in absolute radians, or None if unavailable.
        """
        if not self.is_connected:
            return None

        if motor_name in self._motors:
            motor = self._motors[motor_name]
            sensor = motor.getPositionSensor()
            if sensor is not None:
                if timeout is None:
                    timeout = self.DEFAULT_GET_JOINTS_TIMEOUT
                offset = self._home_offsets.get(motor_name, 0.0)
                if timeout <= 0:
                    return sensor.getValue() + offset
                start = time.time()
                webots_pos_old = sensor.getValue()
                while (time.time() - start) < timeout:
                    self._robot.step(self._timestep)
                    self._poll_estop_keys()
                    webots_pos = sensor.getValue()
                    # Check if motor has stabilized (same reading twice)
                    if abs(webots_pos - webots_pos_old) < 0.0001:
                        return webots_pos + offset
                    else:
                        webots_pos_old = webots_pos
                # Timed out without two identical consecutive reads. In
                # simulation the sensor is always readable and motion is
                # expected to complete, so return the latest reading rather
                # than None (matches _get_joints_radians on timeout).
                return webots_pos_old + offset
            return None

        return None

    def _get_joints_radians(
        self,
        motor_names: Optional[List[str]] = None,
        timeout: Optional[float] = None,
    ) -> Dict[str, float]:
        """
        Get current positions of multiple joints in radians.

        Reads all requested joints simultaneously after waiting for the
        simulation to stabilize. This avoids the inconsistency of reading
        joints sequentially (where each read advances the simulation,
        potentially moving joints that were already read).

        Args:
            motor_names: List of motor names to query. If None, returns all
                        available joints.
            timeout: Max time to wait for all motors to stabilize (seconds).
                    If None, uses DEFAULT_GET_JOINTS_TIMEOUT (5.0s).

        Returns:
            Dict mapping motor names to positions in radians.
        """
        if not self.is_connected:
            return {}

        if timeout is None:
            timeout = self.DEFAULT_GET_JOINTS_TIMEOUT

        names_to_query = motor_names if motor_names is not None else list(self._motors.keys())

        # Filter to names that have valid motors and sensors
        valid_names = []
        for name in names_to_query:
            if name in self._motors:
                sensor = self._motors[name].getPositionSensor()
                if sensor is not None:
                    valid_names.append(name)

        if not valid_names:
            return {}

        if timeout <= 0:
            # Instant read, as on the real robot (no stepping, no waiting).
            return {
                name: self._motors[name].getPositionSensor().getValue()
                + self._home_offsets.get(name, 0.0)
                for name in valid_names
            }

        # Require several consecutive stable reads before accepting the
        # value — a single stable step can be a velocity zero-crossing mid
        # motion and give a false positive.
        required_stable = self.STABILITY_CONSECUTIVE_READS
        stable_threshold = self.STABILITY_THRESHOLD_RAD

        prev_readings: Dict[str, float] = {}
        for name in valid_names:
            sensor = self._motors[name].getPositionSensor()
            prev_readings[name] = sensor.getValue()

        stable_count = 0
        start = time.time()
        while (time.time() - start) < timeout:
            self._robot.step(self._timestep)
            self._poll_estop_keys()

            all_stable = True
            current_readings: Dict[str, float] = {}
            for name in valid_names:
                sensor = self._motors[name].getPositionSensor()
                val = sensor.getValue()
                current_readings[name] = val
                if abs(val - prev_readings.get(name, float('inf'))) >= stable_threshold:
                    all_stable = False

            if all_stable:
                stable_count += 1
                if stable_count >= required_stable:
                    result = {}
                    for name in valid_names:
                        offset = self._home_offsets.get(name, 0.0)
                        result[name] = current_readings[name] + offset
                    return result
            else:
                stable_count = 0

            prev_readings = current_readings

        # Timeout — return best readings we have
        result = {}
        for name in valid_names:
            offset = self._home_offsets.get(name, 0.0)
            result[name] = prev_readings[name] + offset
        return result

    def _set_joints_impl(
        self,
        positions_radians: Dict[str, float],
        velocity_centideg: Optional[int] = None,
    ) -> bool:
        """Set joint positions in absolute radians.

        Converts absolute radians to Webots-relative positions by subtracting
        each joint's home offset.  For finger joints, both proximal and distal
        motors are set to the same position for coupled movement.

        Args:
            positions_radians: Dict mapping motor names to positions in radians.
            velocity_centideg: Speed in centidegrees/second, applied with
                ``Motor.setVelocity`` (capped at the proto's maxVelocity).
                None = the proto's maximum.
        """
        if not self.is_connected:
            return False
        self._apply_pending_halt()
        speed = velocity_centideg / 100.0 if velocity_centideg is not None else None

        for joint_name, position in positions_radians.items():
            if joint_name in self._motors:
                offset = self._home_offsets.get(joint_name, 0.0)
                webots_pos = position - offset
                logger.debug(f"Setting {joint_name} to {position:.4f} rad absolute "
                             f"(webots={webots_pos:.4f}, offset={offset:.4f})")
                motor = self._motors[joint_name]
                self._set_motor_velocity(motor, speed)
                self._set_motor_position(motor, webots_pos, joint_name)

                if joint_name in self._proximal_motors:
                    proximal = self._proximal_motors[joint_name]
                    self._set_motor_velocity(proximal, speed)
                    self._set_motor_position(proximal, webots_pos, joint_name)

        # Step simulation once to initiate movement
        self._robot.step(self._timestep)
        self._poll_estop_keys()
        return True

    def _verify_positions(
        self,
        target_positions: Dict[str, float],
        unit: str,
        timeout: float,
        tolerance: float,
    ) -> bool:
        """
        Verify joints reached target positions by stepping the simulation.

        Overrides base class to continuously step Webots simulation until
        motors reach their targets, providing blocking behavior that matches
        the real robot API.

        Args:
            target_positions: Dict of target positions (in specified unit).
            unit: Unit of the target positions ("percent", "deg", or "rad").
            timeout: Max time to wait (seconds).
            tolerance: Acceptable error (in same unit).

        Returns:
            True if all joints are within tolerance.
        """
        import math

        # Convert targets and per-joint tolerance to radians for comparison.
        # In percent mode, 1% means different absolute angles for each joint
        # (joints have different calibrated ranges), so tolerance must be
        # computed per joint using the same linear map as _percent_to_radians.
        targets_rad: Dict[str, float] = {}
        tolerances_rad: Dict[str, float] = {}

        if unit == "percent":
            for name, pos in target_positions.items():
                targets_rad[name] = self._percent_to_radians(name, pos)
                # 1% of the joint's calibrated range, applied symmetrically.
                span_rad = abs(
                    self._percent_to_radians(name, 100.0)
                    - self._percent_to_radians(name, 0.0)
                )
                tolerances_rad[name] = tolerance * 0.01 * span_rad
        elif unit == "deg":
            tol_rad = math.radians(tolerance)
            for name, pos in target_positions.items():
                targets_rad[name] = math.radians(pos)
                tolerances_rad[name] = tol_rad
        else:  # rad
            for name, pos in target_positions.items():
                targets_rad[name] = pos
                tolerances_rad[name] = tolerance

        start_time = time.time()
        max_steps = int(timeout * 1000 / self._timestep)  # Convert timeout to max simulation steps
        required_stable = self.VERIFY_CONSECUTIVE_READS
        stable_count = 0

        # Always step at least once before accepting a result — otherwise a
        # request that is already within tolerance of the current (pre-move)
        # position would return True before the motor had a chance to act.
        stepped = False

        for _ in range(max_steps):
            if self._stopped:
                self._apply_pending_halt()
                return False
            # Step simulation to let motors move (and register the new target)
            if self._robot.step(self._timestep) == -1:
                return False
            stepped = True
            self._poll_estop_keys()

            # Check if all joints are within per-joint tolerance
            all_within_tolerance = True
            for joint_name, target_rad in targets_rad.items():
                if joint_name not in self._motors:
                    continue

                motor = self._motors[joint_name]
                sensor = motor.getPositionSensor()
                if sensor is None:
                    continue

                offset = self._home_offsets.get(joint_name, 0.0)
                current_pos = sensor.getValue() + offset
                error = abs(current_pos - target_rad)

                if error > tolerances_rad[joint_name]:
                    all_within_tolerance = False
                    break

            if all_within_tolerance:
                stable_count += 1
                if stable_count >= required_stable:
                    return True
            else:
                stable_count = 0

            # Also check wall-clock timeout
            if (time.time() - start_time) >= timeout:
                return False

        return stepped and stable_count >= required_stable

    def _execute_waypoints(
        self,
        joint_names: List[str],
        waypoints: np.ndarray,
        rate_hz: float,
        progress_callback: Optional[Callable[[int, int], None]],
    ) -> bool:
        """Execute waypoints in Webots.

        Waypoints are in absolute radians.  Each position is converted to
        Webots-relative by subtracting the joint's home offset.
        """
        if not self.is_connected:
            return False

        # Build index mapping
        joint_indices = {}
        for i, name in enumerate(joint_names):
            if name in self._motors:
                joint_indices[name] = i

        # Whole multiples of the basic time step: the simulation advances in those anyway.
        step_ms = max(1, round(1000.0 / rate_hz / self._timestep)) * self._timestep
        total = len(waypoints)
        speed = self.default_speed
        for name in joint_indices:
            self._set_motor_velocity(self._motors[name], speed)
            if name in self._proximal_motors:
                self._set_motor_velocity(self._proximal_motors[name], speed)

        for i, point in enumerate(waypoints):
            if self._stopped:
                self._apply_pending_halt()
                logger.warning(f"Trajectory aborted at waypoint {i}/{total} (emergency stop)")
                return False

            for name, idx in joint_indices.items():
                position = point[idx]
                offset = self._home_offsets.get(name, 0.0)
                webots_pos = position - offset
                self._set_motor_position(self._motors[name], webots_pos, name)

                if name in self._proximal_motors:
                    self._set_motor_position(self._proximal_motors[name], webots_pos, name)

            if self._robot.step(step_ms) == -1:
                return False
            self._poll_estop_keys()

            if progress_callback:
                progress_callback(i + 1, total)

        return True

    # ==================== UNIFIED AUDIO OVERRIDES ====================

    def _is_webots(self) -> bool:
        """
        Webots backend returns True.

        This causes ROBOT and LOCAL_AND_ROBOT to resolve to LOCAL only,
        avoiding duplicate playback in simulation.
        """
        return True
