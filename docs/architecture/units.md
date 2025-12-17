# Unit Conventions

Joint angle units and conversions across different components.

## Unit Summary

| Context | Unit | Range (typical) |
|---------|------|-----------------|
| Internal/Trajectory | radians | 0 to ~3.14 |
| User API (default) | percentage | 0% to 100% |
| Webots motors | radians + 1.0 | 1.0 to ~4.14 |
| Real robot (ROS) | centidegrees | 0 to ~18000 |

## Percentage System

The percentage system provides an intuitive interface for joint control:

- **0%**: Joint at minimum limit (fully closed/back)
- **50%**: Joint at midpoint
- **100%**: Joint at maximum limit (fully open/forward)

```python
from pib_ik import Robot

with Robot() as robot:
    # Move to middle position
    robot.set_joint("elbow_left", 50.0)

    # Fully extend
    robot.set_joint("elbow_left", 100.0)

    # Fully retract
    robot.set_joint("elbow_left", 0.0)
```

!!! note "Calibration Required"
    The percentage system requires calibrated joint limits. See the [Calibration Guide](../getting-started/calibration.md).

### Conversion Formula

```python
def percent_to_radians(percent, min_rad, max_rad):
    """Convert percentage to radians."""
    return min_rad + (percent / 100.0) * (max_rad - min_rad)

def radians_to_percent(radians, min_rad, max_rad):
    """Convert radians to percentage."""
    return ((radians - min_rad) / (max_rad - min_rad)) * 100.0
```

## Radians

Direct angular control in radians:

```python
from pib_ik import Robot
import math

with Robot() as robot:
    # Set exact angle
    robot.set_joint("elbow_left", math.pi / 4, unit="rad")  # 45 degrees

    # Read in radians
    pos = robot.get_joint("elbow_left", unit="rad")
    print(f"Elbow at {math.degrees(pos):.1f} degrees")
```

## Webots Offset

Webots motors have a **+1.0 radian offset** from URDF angles:

```
Webots_motor = URDF_radians + 1.0
URDF_radians = Webots_motor - 1.0
```

### Why the Offset?

The PIB robot's Webots proto file uses motor positions where 1.0 represents the "neutral" position. The URDF convention uses 0.0 as neutral. The offset aligns these conventions.

### Handled Automatically

The `WebotsBackend` applies this conversion internally:

```python
class WebotsBackend(RobotBackend):
    WEBOTS_OFFSET = 1.0

    def _to_backend_format(self, radians):
        return radians + self.WEBOTS_OFFSET

    def _from_backend_format(self, values):
        return values - self.WEBOTS_OFFSET
```

You don't need to think about this offset when using the library.

## Real Robot (ROS) Format

The real robot expects positions in **centidegrees** (hundredths of a degree):

```
centidegrees = degrees(radians) * 100
centidegrees = radians * (180 / π) * 100
```

### Example Conversions

| Radians | Degrees | Centidegrees |
|---------|---------|--------------|
| 0.0 | 0° | 0 |
| π/4 | 45° | 4500 |
| π/2 | 90° | 9000 |
| π | 180° | 18000 |

### Handled Automatically

The `RealRobotBackend` converts automatically:

```python
class RealRobotBackend(RobotBackend):
    def _to_backend_format(self, radians):
        return np.round(np.degrees(radians) * 100).astype(int)

    def _from_backend_format(self, centidegrees):
        return np.radians(np.array(centidegrees) / 100.0)
```

## Trajectory Format

Trajectories are always stored in **radians** (URDF convention):

```json
{
  "format_version": "1.0",
  "unit": "radians",
  "coordinate_frame": "urdf",
  "waypoints": [
    [0.5, 1.2, 0.8, ...],
    [0.6, 1.3, 0.9, ...]
  ]
}
```

Backends convert to their native format when executing.

## Comparison Table

| Operation | Swift | Webots | Real Robot |
|-----------|-------|--------|------------|
| Internal storage | radians | radians | radians |
| Backend conversion | none | +1.0 rad | to centideg |
| Position sensor | radians | radians - 1.0 | from centideg |
| API (percentage) | ✓ | ✓ | ✓ |
| API (radians) | ✓ | ✓ | ✓ |

## Code Examples

### Working with Different Units

```python
from pib_ik import Robot, Swift
import math

# Both backends support the same API
def demonstrate_units(backend):
    # Percentage (recommended for most uses)
    backend.set_joint("elbow_left", 50.0)
    pos_pct = backend.get_joint("elbow_left")
    print(f"Percentage: {pos_pct:.1f}%")

    # Radians (for precise control)
    backend.set_joint("elbow_left", math.pi / 3, unit="rad")
    pos_rad = backend.get_joint("elbow_left", unit="rad")
    print(f"Radians: {pos_rad:.3f}")

# Works the same with both backends
with Swift() as viz:
    demonstrate_units(viz)

with Robot(host="172.26.34.149") as robot:
    demonstrate_units(robot)
```

### Manual Conversion

```python
from pib_ik.backends.base import RobotBackend
import math

# If you need manual conversion (rarely needed)
def manual_convert():
    # Get calibration data
    limits = RobotBackend.JOINT_LIMITS  # Dict of {name: (min, max)}

    joint = "elbow_left"
    min_rad, max_rad = limits[joint]

    # Percent to radians
    percent = 75.0
    radians = min_rad + (percent / 100.0) * (max_rad - min_rad)

    # Radians to degrees
    degrees = math.degrees(radians)

    # To centidegrees (for ROS)
    centideg = int(degrees * 100)

    print(f"{percent}% = {radians:.3f} rad = {degrees:.1f}° = {centideg} centideg")
```
