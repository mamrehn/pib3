<h1 align="center">
<img src="examples/pib3_logo.png" width="440">
</h1><br>

**piB3**

Inverse kinematics and trajectory generation for the [pib](https://pib.rocks/) printable humanoid robot. Convert images to robot arm drawing trajectories. 

## Installation

```bash
pip install "pib-ik[all] @ git+https://github.com/mamrehn/pib3.git"
```

## Quick Start

```python
import pib3

# Convert image to trajectory
trajectory = pib3.generate_trajectory("drawing.png")
trajectory.to_json("output.json")

# Visualize in browser
with pib3.Swift() as viz:
    viz.run_trajectory("output.json")

# Or run on real robot
with pib3.Robot(host="172.26.34.149") as robot:
    robot.run_trajectory("output.json")
```

### More Manual Joint Control

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
