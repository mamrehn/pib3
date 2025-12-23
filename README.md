<h1 align="center">
<a href="https://ghloc.vercel.app/mamrehn/pib3?branch=main"><img src="examples/pib3_logo.png" width="440"></a>
</h1><br>

Repository statistics: 🐍 ![Python LOC](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/mamrehn/pib3/badge?filter=.py$&format=human), 📜 ![Documentation LOC](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/mamrehn/pib3/badge?filter=.md$&format=human), 🤖 ![Robot Config LOC](https://img.shields.io/endpoint?url=https://ghloc.vercel.app/api/mamrehn/pib3/badge?filter=.proto$,.urdf$,.yaml$,.stl$&format=human)


**piB3** provides motor control, inverse kinematics, and same code for digital twin and the [pib](https://pib.rocks/) printable humanoid robot.

Use cases:
 - Set motors and read joint sensors within your python code.
 - Convert 2-D images to 3-D robot arm drawing trajectories.
 - Test code in Webots or Swift simulations once, then run it on the real world pib.

## Installation

```bash
python -m venv venv
venv\Scripts\activate  # OR on Linux: source ./venv/bin/activate 
pip install -U "pib3[all] @ git+https://github.com/mamrehn/pib3.git"
```

## Quick Start

### Manual Joint Control

```python
from pib3 import Robot

with Robot(host="172.26.34.149") as robot:
    # Move head to center
    robot.set_joint("turn_head_motor", 50.0)

    # Bend elbow to 75%
    robot.set_joint("elbow_left", 75.0)

    # Read current position
    pos = robot.get_joint("elbow_left")
    print(f"Left elbow is at {pos:.1f}%")
```

### Automatic 3-D trajectory planning based on 2-D lines (e.g. for drawing on paper)

```python
import pib3

# Convert image to trajectory
trajectory = pib3.generate_trajectory("drawing.png")
trajectory.to_json("output.json")

# Visualize in browser
with pib3.Swift() as viz:
    viz.run_trajectory("output.json")
```

### Usage of digital twin code on real world pib robot

```python
# Or run on real robot
with pib3.Robot(host="172.26.34.149") as robot:
    robot.run_trajectory("output.json")
```

## Documentation

Full documentation: **[mamrehn.github.io/pib3](https://mamrehn.github.io/pib3/)**

- [Installation Guide](https://mamrehn.github.io/pib3/getting-started/installation/) - Detailed setup for Linux/Windows
- [Quick Start](https://mamrehn.github.io/pib3/getting-started/quickstart/) - Basic usage examples
- [Calibration Guide](https://mamrehn.github.io/pib3/getting-started/calibration/) - Configure joint limits
- [API Reference](https://mamrehn.github.io/pib3/api/) - Complete API documentation
- [Tutorials](https://mamrehn.github.io/pib3/tutorials/) - Step-by-step guides


## Acknowledgments

- [PIB Project](https://pib.rocks/) - Open source humanoid robot
- [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
