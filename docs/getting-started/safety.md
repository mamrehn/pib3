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
| **Space** (also Esc, Numpad-0, Pause), anywhere on the desktop | Windows; macOS after granting permission; Linux on X11 | nothing on Windows, see [below](#if-the-keys-do-not-work) otherwise |
| **Ctrl+C** in the terminal that runs the program | everywhere | nothing |
| **STOP button** on screen (click, or Space/Esc while it has focus) | everywhere with a desktop | tkinter (included in the python.org and current uv builds) |
| **Teacher's remote stop** `pib3-estop --host <robot>` | any laptop on the robot's network | nothing on the student laptop |
| **Space in Webots** (3D view focused) | the simulation, for practice | nothing |

The stop **arms itself with the first motion command** of a program and
stays armed until the program ends, including the pauses between moves. At
that moment the **STOP window** appears (top right, always on top). It is the
visible sign that the stop is armed, and it lists every way to stop that works
on this computer:

```text
NOT-AUS SCHARF · pib-01            (English systems: E-STOP ARMED)
STOP
Klick · Leertaste · Esc · Strg+C
hält den ganzen Roboter an
```

After a stop it turns grey ("GESTOPPT" / "STOPPED") and names the cause. It
follows the system language; `PIB3_LANG=de` or `PIB3_LANG=en` forces one. If
the keys cannot work on that laptop, the window lists only click and Ctrl+C,
and the console says why. The console also shows:

```text
Emergency stop armed: Space, Esc, Numpad-0 or Pause / Ctrl+C in this terminal / the STOP button.
```

The window takes no keyboard focus and ignores Enter, so it cannot turn the
next Return of a typing hand into a stop. `Robot(stop_button="auto")` shows it
only where the keys cannot work, `stop_button=False` never.

A program that never moves a motor never arms it. A camera station
(`KameraStation`, no servos) or a perception script next to a robot that
another group drives therefore grabs no keys, and its Ctrl+C or crash does
not freeze someone else's arm.

### Ctrl+C keeps its meaning

Ctrl+C still cancels the program, exactly as before: pib3 does not take the
key over. Its signal handler freezes the motors and then passes the signal on,
so `KeyboardInterrupt` arrives as usual. A program that never moved the robot
is not touched at all. The global key hook does not listen for Ctrl+C.

- Copying in the editor, browser or chat never stops anything.
- In a terminal, copy is Ctrl+Shift+C on Linux and Cmd+C on macOS. Windows
  Terminal and the VS Code terminal copy with Ctrl+C only when text is
  selected; without a selection Ctrl+C interrupts.
- Ctrl+C in that terminal ended the program before pib3 0.2 as well. The
  difference is what the arm does: it used to finish its last move after the
  program had died; now it stops where it is.

The keys that *can* fire unintentionally are Space and Esc: the hook sees
them in every window while a program drives the robot. A false stop only
ends the program, and the stop is armed only in programs that move a motor.
Pass `estop_keys=["esc"]` (or `"f12"`, ...) to `Robot(...)` if Space gets in
the way.

## What a stop does

1. **Every servo of the robot freezes where it is** and holds its position.
   The arms do not fall.
2. **The whole robot stops, not just one program.** Two groups drive the two
   arms of one pib at the same time. A stop from either group latches every
   pib3 program connected to that robot; both groups restart afterwards.
3. **The stop latches.** Every later motion command raises
   `pib3.EmergencyStopError`, so the program ends with a clear message
   instead of moving on. A running `run_trajectory()`, `set_joints_sequence()`
   or blocking `set_joint()` returns `False` at once.
4. **Pressing the key again does nothing.** Only `robot.resume()` in the
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

## Practise it in Webots

`pib3.Webots()` has the same stop and the same STOP window ("Klick ·
Leertaste im 3D-Fenster"), so it can be practised before anyone stands next to
a real arm. Click into the 3D view, press **Space** (or click the STOP
window): the joints freeze, and the next motion command ends the controller
with `EmergencyStopError`. Reset the simulation to continue.

Webots only passes key presses to the controller while its 3D view has
focus. Typing in the editor while the simulation runs therefore never stops
it, and no permission is needed. Webots does not pass Esc to controllers, so
Space is the key here, the same key that works on the robot.

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
    `sudo apt install python3-tk`. Older uv-managed Python builds (e.g.
    3.13.4/3.13.5) ship a Tk that crashes on Linux; pib3 names the fix:
    `uv python upgrade`, then recreate the venv.

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
  carries over. Control loops must therefore start from the measured joint
  position (`sim.get_joint(..., timeout=0)`), as on the robot.

## Limits of the stop

- In `motor_mode="ros"` pib3 has no direct access to the servos. A stop
  then prevents further commands but cannot freeze a move that is already
  running. The default `motor_mode="direct"` can.
- pib's hobby servos report no position. pib3 reads the position the servo
  bricklet *commands*. A blocked joint therefore still reads as "arrived".
