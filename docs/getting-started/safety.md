# Safety and Emergency Stop

pib's arms are light, but servos at full speed can pinch fingers, sweep a
laptop off the table or wreck a gearbox against a mechanical stop. pib3
therefore ships a **software emergency stop** that works on every laptop,
plus defaults that keep a first run slow and inside the joint limits.

!!! danger "A software stop is not a certified safety function"
    Everything on this page runs in software, on a laptop, over Wi-Fi. It is
    the *first* reaction, not a guarantee. Keep everyone out of the arm's
    reach while a program runs, and know where the robot's power switch is.
    Switching the power off makes the arms **drop**, so it is the last
    resort, not the first.

## How to stop the robot

| Trigger | Works on | Needs |
|---|---|---|
| **Space**, **Esc**, Numpad-0 or Pause, anywhere on the desktop | Windows; macOS after granting permission; Linux on X11 | nothing on Windows, see [below](#if-the-keys-do-not-work) otherwise |
| **Ctrl+C** in the terminal that runs the program | everywhere | nothing |
| **STOP button** on screen (click, or Space/Esc while it has focus) | everywhere with a desktop | tkinter (included in the python.org and uv builds) |
| **Teacher's remote stop** `pib3-estop --host <robot>` | any laptop on the robot's network | nothing on the student laptop |

`Robot(...)` arms the keys and Ctrl+C when it connects. If the keys cannot
work on that laptop, it opens the STOP button by itself and says why. The
connect message lists what works:

```text
Emergency stop: Space, Esc, Numpad-0 or Pause / Ctrl+C in this terminal.
```

## What a stop does

1. **Every servo freezes where it is** and holds its position. The arms do
   not fall.
2. **The stop latches.** Every later motion command raises
   `pib3.EmergencyStopError`, so the program ends with a clear message
   instead of moving on. A running `run_trajectory()`, `set_joints_sequence()`
   or blocking `set_joint()` returns `False` at once.
3. **Pressing the key again does nothing.** Only `robot.resume()` in the
   program releases the stop. That is deliberate: a panicked double press
   must not start the robot again.

```python
import pib3
from pib3 import Joint

with pib3.Robot(host="192.168.0.11") as robot:
    try:
        robot.set_joint(Joint.ELBOW_LEFT, 80.0)
    except pib3.EmergencyStopError:
        print("Stopped. Check the robot, then run the program again.")
```

A program that crashes or is interrupted inside `with Robot(...)` also
freezes the motors. Without that, the servo bricklets would finish the last
move on their own, after the program had already ended.

## If the keys do not work

=== "macOS"

    macOS blocks global key presses until you allow them:
    **System Settings → Privacy & Security → Input Monitoring** (and
    **Accessibility**) → enable your terminal or editor (Terminal, iTerm,
    Visual Studio Code). Restart that app afterwards. Until then pib3 opens
    the STOP button.

=== "Linux"

    Under **Wayland** (default on Ubuntu and Fedora) programs cannot see key
    presses of other windows. The keys then work only while an X11 window
    has focus. Use the STOP button or Ctrl+C, or log in with an
    "Ubuntu on Xorg" session. A system Python may lack tkinter:
    `sudo apt install python3-tk`.

=== "Windows"

    Works without setup. Some managed devices block global keyboard hooks;
    pib3 then opens the STOP button.

## The teacher's remote stop

From any laptop on the robot's network:

```bash
pib3-estop --host 192.168.0.11                    # stop now
pib3-estop --host pib-01 --host pib-02            # several robots at once
pib3-estop --host pib-01 --window                 # a STOP button for these robots
pib3-estop --host pib-01 --relax                  # servos off: arms DROP
```

(`python -m pib3.tools.estop ...` does the same.) It freezes every servo
directly through the robot's Tinkerforge daemon, so it works even when the
student's program hangs. It also latches the stop in every pib3 program
connected to that robot, so their next command does not start the robot
again. Students continue with `robot.resume()` or by restarting.

## Defaults that prevent accidents

- **Speed.** Every motion uses `robot.default_speed` (150 deg/s) unless it
  passes `speed=`. For a first run on the real robot, slow the whole program
  down with one line: `robot.default_speed = 45`. `speed=0` is rejected; to
  the servo bricklet it would mean "no limit".
- **Homing.** `go_home()` moves at 10 deg/s, because the arms hang loose at
  power-on and nobody knows the start pose.
- **Joint limits.** Targets outside a joint's range are clamped to the limit
  (with a one-time hint) instead of driving the servo into its mechanical
  stop. The robot's own motor settings from Cerebra (invert flags and
  rotation ranges) apply in pib3's direct mode as well.
- **Trajectories** first move to their start pose at 30 deg/s and wait there.
- **Same motion in the simulator.** Webots moves the joints with the same
  speed and ramps as the real robot, so timing you tune in the simulation
  carries over.

## Limits of the stop

- In `motor_mode="ros"` pib3 has no direct access to the servos. A stop
  then prevents further commands but cannot freeze a move that is already
  running. The default `motor_mode="direct"` can.
- pib's hobby servos report no position. pib3 reads the position the servo
  bricklet *commands*. A blocked joint therefore still reads as "arrived".
