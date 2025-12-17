# pib-ik

Inverse kinematics and trajectory generation for the [PIB](https://pib.rocks/) humanoid robot. Convert images to robot arm drawing trajectories.

## Installation

```bash
pip install "pib-ik[all] @ git+https://github.com/mamrehn/pib_ik.git"
```

## Quick Start

```python
import pib_ik

# Convert image to trajectory
trajectory = pib_ik.generate_trajectory("drawing.png")
trajectory.to_json("output.json")

# Visualize in browser
with pib_ik.Swift() as viz:
    viz.run_trajectory("output.json")

# Or run on real robot
with pib_ik.Robot(host="172.26.34.149") as robot:
    robot.run_trajectory("output.json")
```

## Documentation

Full documentation: **[mamrehn.github.io/pib_ik](https://mamrehn.github.io/pib_ik/)**

- [Installation Guide](https://mamrehn.github.io/pib_ik/getting-started/installation/) - Detailed setup for Linux/Windows
- [Quick Start](https://mamrehn.github.io/pib_ik/getting-started/quickstart/) - Basic usage examples
- [Calibration Guide](https://mamrehn.github.io/pib_ik/getting-started/calibration/) - Configure joint limits
- [API Reference](https://mamrehn.github.io/pib_ik/api/) - Complete API documentation
- [Tutorials](https://mamrehn.github.io/pib_ik/tutorials/) - Step-by-step guides


## Acknowledgments

- [PIB Project](https://pib.rocks/) - Open source humanoid robot
- [Robotics Toolbox for Python](https://github.com/petercorke/robotics-toolbox-python)
