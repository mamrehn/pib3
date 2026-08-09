<h1 align="center">
<a href="https://ghloc.vercel.app/mamrehn/pib3?branch=main"><img src="https://raw.githubusercontent.com/mamrehn/pib3/main/examples/pib3_logo.png" width="440"></a>
</h1><br>

Repository statistics:

| Type | Lines of Code |
| ---- | ------------- |
| 🐍 Python Code | ![Python LOC](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/mamrehn/pib3/badge?filter=.py$&format=human) |
| 📜 Documentation | ![Documentation LOC](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/mamrehn/pib3/badge?filter=.md$&format=human) |
| 🤖 Robot Definition | ![Robot Config LOC](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/mamrehn/pib3/badge?filter=.proto$,.urdf$,.yaml$,.stl$&format=human) |

**piB3** provides motor control, inverse kinematics, and seamless code sharing between digital twin and the [pib](https://github.com/pib-rocks) printable humanoid robot.

**Key Features:**

| | |
|---|---|
| 🔄 **Same Code Everywhere** | Test in Webots simulation, then run on real robot |
| 🎮 **Joint Control** | Read/write motors with IDE autocomplete via `Joint` enum |
| 📐 **Multiple Units** | Work in percent (0-100%), degrees, or radians |
| ✍️ **Trajectory Generation** | Convert 2-D images to 3-D robot arm drawing trajectories |
| 🖐️ **Hand Poses** | Pre-defined gestures or custom poses saved as JSON |
| 📷 **Camera Streaming** | Real-time camera image subscription |
| 🤖 **AI Vision** | On-device detection/segmentation with model switching |
| 📊 **IMU Sensors** | Accelerometer and gyroscope data streaming |

## Installation

**Windows** (PowerShell) — read [Windows prerequisites](#windows-prerequisites) first:

```powershell
py -3.13 -m venv venv
.\venv\Scripts\Activate.ps1
python -m pip install --upgrade pip setuptools wheel
pip install -U "pib3 @ git+https://github.com/mamrehn/pib3.git"
```

**Linux / macOS:**

```bash
python3 -m venv venv
source ./venv/bin/activate
pip install -U "pib3 @ git+https://github.com/mamrehn/pib3.git"
```

### Windows prerequisites

Two things must be in place before `pip install` succeeds. Getting either wrong is what
produces the usual wall of red text.

#### 1. Python 3.10–3.13, 64-bit — not the newest release

This is the most common cause of a failed install. `roboticstoolbox-python` publishes
pre-built wheels only for **CPython 3.10, 3.11, 3.12 and 3.13 on `win_amd64`**. On any
other interpreter — including the **Python 3.14** that
[python.org](https://www.python.org/downloads/) currently offers as its default download
button — there is no matching wheel, so pip would fall back to compiling a C extension
from source and fail with `error: Microsoft Visual C++ 14.0 or greater is required`.
Installing a redistributable does not fix that: the redistributable is a *runtime*, not a
compiler.

pib3 therefore declares `requires-python = ">=3.10,<3.14"`, so pip stops early with a
readable message rather than a wall of compiler output:

```text
ERROR: Package 'pib3' requires a different Python: 3.14.7 not in '<3.14,>=3.10'
```

If you see that, you are on the wrong interpreter — not missing a build tool.

Install the latest **3.13.x** *Windows installer (64-bit)* from the
[Windows downloads page](https://www.python.org/downloads/windows/)
(e.g. [Python 3.13.15](https://www.python.org/downloads/release/python-31315/)) and tick
**"Add python.exe to PATH"**. Then check what you actually have:

```powershell
py --list   # lists every installed interpreter
py -3.13 -c "import sys, platform; print(sys.version, platform.machine())"
# expected: 3.13.x ...  AMD64
```

`AMD64` matters — a 32-bit (`x86`) Python has no wheels either. Always create the
environment with that interpreter (`py -3.13 -m venv venv`), never with a bare `python`
that may point at 3.14.

#### 2. Git for Windows

The install command fetches from a Git URL, so `git` must be on `PATH` or pip aborts with
`Cannot find command 'git' - do you have 'git' installed and in your PATH?`.

Install [Git for Windows](https://git-scm.com/download/win), then verify in a **newly
opened** terminal (`PATH` changes do not reach already-running shells):

```powershell
git --version
```

#### Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `error: Microsoft Visual C++ 14.0 or greater is required` | No wheel matches your interpreter, so pip is building from source. Fix the Python version (step 1) — that resolves it in almost every case. Only if you truly must build from source, install the [Visual C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/) and select the **"Desktop development with C++"** workload; the redistributable alone will not do. |
| `Cannot find command 'git'` | Step 2, then reopen the terminal. |
| `Package 'pib3' requires a different Python` | Your interpreter is outside 3.10–3.13. Step 1 — do not try to force it with `--ignore-requires-python`. |
| `No matching distribution found ...`, or pip resolves a years-old `roboticstoolbox-python` and then fails to build it | A 32-bit install, or a stale checkout whose pins let pip backtrack. pip does not warn about this — it quietly picks a prehistoric release and compiles that instead. Step 1. |
| `ImportError: DLL load failed while importing cv2` / `onnxruntime` at *runtime* (install succeeded) | Missing C++ runtime. Install the Visual C++ Redistributable: [x64](https://aka.ms/vs/17/release/vc_redist.x64.exe). |
| `Activate.ps1 cannot be loaded because running scripts is disabled` | `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned`, confirm, reopen PowerShell. Or use `venv\Scripts\activate.bat` from `cmd.exe`. |
| `Could not install packages ... path too long` | Enable [long paths](https://learn.microsoft.com/windows/win32/fileio/maximum-file-path-limitation) or move the project close to the drive root, e.g. `C:\pib`. |

#### Windows on ARM (Snapdragon X, Surface Pro 11)

Not supported natively: neither `roboticstoolbox-python` nor `opencv-python-headless`
publishes `win_arm64` wheels, so both would have to be compiled from source — and building
OpenCV that way is impractical. Install the **x64** build of Python and let Windows run it
under emulation, or use an x64 machine or WSL2.

## Quick Start

### Digital Twin - Same Code Everywhere

```python
import pib3

# Same API works on all backends

with pib3.Webots() as sim:             # Webots simulation
    sim.run_trajectory("output.json")

with pib3.Robot(host="...") as robot:  # Real robot
    robot.run_trajectory("output.json")
```

### Joint Control

```python
from pib3 import Robot, Joint

with Robot(host="172.26.34.149") as robot:
    # Set joints with IDE autocomplete
    robot.set_joint(Joint.TURN_HEAD, 50.0)    # 50% of range
    robot.set_joint(Joint.ELBOW_LEFT, 75.0)

    # Read current position
    pos = robot.get_joint(Joint.ELBOW_LEFT)
    print(f"Left elbow at {pos:.1f}%")

    # Use degrees or radians
    robot.set_joint(Joint.TURN_HEAD, -30.0, unit="deg")

    # Wait for movement to complete
    robot.set_joint(Joint.ELBOW_LEFT, 50.0, async_=False)

    # Save and restore poses
    saved_pose = robot.get_joints()
    robot.set_joints(saved_pose)
```

### Hand Poses

```python
from pib3 import Robot, HandPose, LEFT_HAND_JOINTS

with Robot(host="172.26.34.149") as robot:
    robot.set_joints_pose(HandPose.LEFT_OPEN)              # Open hand
    robot.set_joints_pose(HandPose.LEFT_CLOSED)            # Close hand
    robot.set_joints({j: 50.0 for j in LEFT_HAND_JOINTS})  # 50% grip
```

### Image to Drawing Trajectory

```python
import pib3

trajectory = pib3.generate_trajectory("drawing.png")
trajectory.to_json("output.json")


```

### Camera & AI Vision

```python
from pib3 import Robot, AIModel

with Robot(host="172.26.34.149") as robot:
    # Stream camera frames
    sub = robot.subscribe_camera_image(lambda jpeg: print(f"{len(jpeg)} bytes"))
    sub.unsubscribe()

    # AI object detection — typed results, labels resolved to COCO names
    robot.ai.set_model(AIModel.YOLOV8N)
    for det in robot.ai.get_detections():
        print(f"{det.label}: {det.confidence:.0%} at {det.bbox}")
```

> **Note — `label` on the raw topic is a class *ID*, not a name.** If you
> subscribe with `subscribe_ai_detections()` directly, `det['label']` is an
> integer (`0` = person, `41` = cup, …) and the readable name is in the
> optional `label_name` field. `robot.ai` / `AIDetectionReceiver` wrap this in
> a `Detection` and resolve the name from `COCO_LABELS` for you, so prefer
> them unless you need the raw payload:
>
> ```python
> def on_detection(data):
>     for det in data.get('result', {}).get('detections', []):
>         print(f"class {det['label']}: {det['confidence']:.0%}")  # numeric ID
>
> sub = robot.subscribe_ai_detections(on_detection)
> ```

#### The same code in simulation

The simulated pib has a camera in its head, so `sim.camera` and `sim.ai` offer
the same contract — and because the head moves the camera, visual servoing in
simulation is a genuinely closed loop.

```python
import pib3
from pib3 import Joint

with pib3.Webots() as sim:               # inside a Webots controller
    sim.ai.set_model("recognition")      # simulator ground truth, no model

    while sim.step():                    # step() renders the next frame
        img = sim.camera.get_frame().to_numpy()      # BGR, like the robot
        for det in sim.ai.get_detections():
            x, _ = det.bbox.center                   # 0..1 across the image
            sim.set_joint(Joint.TURN_HEAD, 50 + 60 * (x - 0.5), async_=True)
```

Differences worth knowing:

- **`sim.step()` is required in a perception loop.** Motion calls step the
  simulator themselves, but a loop that only reads does not — without it the
  camera returns the same frame forever.
- **`"recognition"`** is Webots ground truth: exact boxes, `confidence` always
  `1.0`, no model. Pass a model name (`"yolov8n"`, `"pose"`, `"hand"`) to run a
  real network on the simulated frames instead — `pip install "pib3[sim] @ git+https://github.com/mamrehn/pib3.git"`.
  Note a COCO-trained detector sees very little in an untextured world.
- **Objects must opt in.** A Solid is only recognized if it sets
  `recognitionColors`; its `model` field becomes `det.label`.
- **No on-device AI.** Inference runs on the host, so latency is honest but
  different from the OAK-D's accelerator.

Full example: [`examples/webots_camera_view.py`](https://github.com/mamrehn/pib3/blob/main/examples/webots_camera_view.py).
Diagnosing a setup: [`examples/webots_camera_check.py`](https://github.com/mamrehn/pib3/blob/main/examples/webots_camera_check.py).

### IMU Sensors

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    def on_imu(data):
        accel = data['linear_acceleration']
        print(f"Accel: ({accel['x']:.2f}, {accel['y']:.2f}, {accel['z']:.2f}) m/s²")
    
    robot.set_imu_frequency(50)
    sub = robot.subscribe_imu(on_imu, data_type="full")
```

## Documentation

Full documentation: **[mamrehn.github.io/pib3](https://mamrehn.github.io/pib3/)**

- [Installation Guide](https://mamrehn.github.io/pib3/getting-started/installation/) - Setup for Linux/Windows
- [Quick Start](https://mamrehn.github.io/pib3/getting-started/quickstart/) - Basic usage examples
- [Calibration Guide](https://mamrehn.github.io/pib3/getting-started/calibration/) - Configure joint limits
- [API Reference](https://mamrehn.github.io/pib3/api/) - Complete API documentation
- [Tutorials](https://mamrehn.github.io/pib3/tutorials/) - Step-by-step guides

## Acknowledgments

- [PIB Project](https://pib.rocks/) - Open source humanoid robot
- [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
