# Core Types

Data structures for representing 2D drawings.

## Overview

pib-ik uses two main types to represent drawings:

- **Stroke**: A single continuous line (pen-down motion)
- **Sketch**: A collection of strokes forming a complete drawing

## Stroke

A stroke represents a single continuous drawing motion - a sequence of points connected without lifting the pen.

::: pib_ik.types.Stroke
    options:
      show_root_heading: true
      show_source: true
      members:
        - __init__
        - __len__
        - length
        - reverse
        - start
        - end

### Creating Strokes

```python
import numpy as np
from pib_ik import Stroke

# From a list of points
stroke = Stroke(
    points=[[0.1, 0.1], [0.5, 0.2], [0.9, 0.1]],
    closed=False
)

# From numpy array
points = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])
closed_stroke = Stroke(points=points, closed=True)

# Access properties
print(f"Length: {stroke.length()}")
print(f"Start: {stroke.start()}")
print(f"End: {stroke.end()}")
```

### Coordinate System

Stroke coordinates are **normalized** to the range [0, 1]:

- `(0, 0)` = top-left corner
- `(1, 1)` = bottom-right corner
- `(0.5, 0.5)` = center

---

## Sketch

A sketch is a collection of strokes representing a complete drawing.

::: pib_ik.types.Sketch
    options:
      show_root_heading: true
      show_source: true
      members:
        - __init__
        - __len__
        - __iter__
        - __getitem__
        - total_points
        - total_length
        - bounds
        - add_stroke
        - to_dict
        - from_dict

### Creating Sketches

```python
from pib_ik import Sketch, Stroke

# Empty sketch
sketch = Sketch()

# Add strokes
sketch.add_stroke(Stroke(points=[[0.1, 0.1], [0.9, 0.1]]))
sketch.add_stroke(Stroke(points=[[0.1, 0.9], [0.9, 0.9]]))

# With strokes and source info
sketch = Sketch(
    strokes=[stroke1, stroke2],
    source_size=(800, 600)  # Original image dimensions
)
```

### Iterating and Accessing

```python
# Iterate over strokes
for stroke in sketch:
    print(f"Stroke with {len(stroke)} points")

# Access by index
first = sketch[0]
last = sketch[-1]

# Get statistics
print(f"Strokes: {len(sketch)}")
print(f"Total points: {sketch.total_points()}")
print(f"Total length: {sketch.total_length()}")
print(f"Bounds: {sketch.bounds()}")
```

### Serialization

```python
import json
from pib_ik import Sketch

# To dictionary (JSON-serializable)
data = sketch.to_dict()

# Save to file
with open("sketch.json", "w") as f:
    json.dump(data, f)

# Load from file
with open("sketch.json") as f:
    data = json.load(f)
sketch = Sketch.from_dict(data)
```
