# pib3

**Inverse kinematics and trajectory generation for the PIB humanoid robot.**

<div class="grid cards" markdown>

-   :material-robot:{ .lg .middle } **What is pib3?**

    ---

    A Python library that enables the [PIB robot](https://pib.rocks/) to draw images using its arm. Convert any image to robot arm trajectories.

-   :material-lightning-bolt:{ .lg .middle } **Quick & Easy**

    ---

    Just three lines of code to convert an image to a trajectory and run it on your robot.

-   :material-devices:{ .lg .middle } **Multiple Backends**

    ---

    Works with real robots, Webots simulation, and Swift 3D visualization in the browser.

-   :material-percent:{ .lg .middle } **Intuitive Control**

    ---

    Control joints using simple percentages (0-100%) instead of radians.

</div>

## Features

- **Image to Trajectory**: Convert any image to robot drawing trajectories
- **Percentage-based Control**: Use 0-100% values that work across all backends
- **Real Robot Support**: Connect to PIB robot via rosbridge websocket
- **Swift Visualization**: Browser-based 3D visualization with interactive controls
- **Webots Integration**: Full Webots simulator support
- **Calibration Tool**: Interactive calibration for your specific robot hardware
- **Save/Restore Poses**: Easily save and restore robot poses

## Quick Example

```python
import pib3

# Convert image to trajectory
trajectory = pib3.generate_trajectory("drawing.png")

# Execute on real robot
with pib3.Robot(host="172.26.34.149") as robot:
    robot.run_trajectory(trajectory)
```

## Installation

```bash
pip install "pib3[all] @ git+https://github.com/mamrehn/pib3.git"
```

See the [Installation Guide](getting-started/installation.md) for detailed instructions.

## Getting Started

<div class="grid cards" markdown>

-   :material-download:{ .lg .middle } **[Installation](getting-started/installation.md)**

    ---

    Set up pib3 on Linux or Windows with virtual environments.

-   :material-rocket-launch:{ .lg .middle } **[Quick Start](getting-started/quickstart.md)**

    ---

    Get up and running in 5 minutes with simple examples.

-   :material-tune:{ .lg .middle } **[Calibration](getting-started/calibration.md)**

    ---

    Calibrate joint limits for accurate percentage-based control.

</div>

## Tutorials

Learn how to use pib3 effectively:

- [Image to Trajectory](tutorials/image-to-trajectory.md) - Convert images to robot movements
- [Controlling the Robot](tutorials/controlling-robot.md) - Master the joint control API
- [Swift Visualization](tutorials/swift-visualization.md) - Use browser-based 3D visualization
- [Working with Sketches](tutorials/working-with-sketches.md) - Manipulate sketches programmatically
- [Custom Configurations](tutorials/custom-configurations.md) - Fine-tune all parameters

## API Reference

Full documentation for all classes and functions:

- [Core Types](api/types.md) - Stroke, Sketch, Trajectory
- [Configuration](api/config.md) - All configuration options
- [Backends](api/backends/base.md) - Robot, Swift, Webots

## Project Links

- [GitHub Repository](https://github.com/mamrehn/pib3)
- [PIB Project](https://pib.rocks/)
- [Report Issues](https://github.com/mamrehn/pib3/issues)

## License

MIT License - see [LICENSE](https://github.com/mamrehn/pib3/blob/main/LICENSE) for details.
