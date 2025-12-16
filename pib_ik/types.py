"""Core data types for pib_ik package."""

from dataclasses import dataclass, field
from typing import List, Tuple
import numpy as np


@dataclass
class Stroke:
    """A single continuous drawing stroke (pen-down motion).

    Represents a sequence of 2D points that form a continuous line
    drawn without lifting the pen.

    Attributes:
        points: Array of shape (N, 2) with normalized [0,1] coordinates.
                (0,0) = top-left of drawing area, (1,1) = bottom-right.
        closed: If True, the stroke forms a closed loop (first and last
                points should be connected).
    """
    points: np.ndarray
    closed: bool = False

    def __post_init__(self):
        """Ensure points is a numpy array with correct shape."""
        self.points = np.asarray(self.points, dtype=np.float64)
        if self.points.ndim == 1:
            self.points = self.points.reshape(-1, 2)
        if self.points.ndim != 2 or self.points.shape[1] != 2:
            raise ValueError(
                f"Points must have shape (N, 2), got {self.points.shape}"
            )

    def __len__(self) -> int:
        """Return number of points in the stroke."""
        return len(self.points)

    def length(self) -> float:
        """Calculate total arc length of the stroke."""
        if len(self.points) < 2:
            return 0.0
        diffs = np.diff(self.points, axis=0)
        distances = np.linalg.norm(diffs, axis=1)
        return float(np.sum(distances))

    def reverse(self) -> "Stroke":
        """Return a new stroke with reversed point order."""
        return Stroke(points=self.points[::-1].copy(), closed=self.closed)

    def start(self) -> np.ndarray:
        """Return the starting point of the stroke."""
        return self.points[0]

    def end(self) -> np.ndarray:
        """Return the ending point of the stroke."""
        return self.points[-1]


@dataclass
class Sketch:
    """A collection of strokes extracted from an image.

    Represents a complete drawing as a list of individual strokes,
    typically in optimized drawing order to minimize pen-up travel.

    Attributes:
        strokes: List of Stroke objects in drawing order.
        source_size: Original image dimensions (width, height) in pixels.
                     None if not created from an image.
    """
    strokes: List[Stroke] = field(default_factory=list)
    source_size: Tuple[int, int] = None

    def __len__(self) -> int:
        """Return number of strokes in the sketch."""
        return len(self.strokes)

    def __iter__(self):
        """Iterate over strokes."""
        return iter(self.strokes)

    def __getitem__(self, idx) -> Stroke:
        """Get stroke by index."""
        return self.strokes[idx]

    def total_points(self) -> int:
        """Return total number of points across all strokes."""
        return sum(len(s) for s in self.strokes)

    def total_length(self) -> float:
        """Return total arc length of all strokes."""
        return sum(s.length() for s in self.strokes)

    def bounds(self) -> Tuple[float, float, float, float]:
        """Return bounding box (min_u, min_v, max_u, max_v) of all strokes."""
        if not self.strokes:
            return (0.0, 0.0, 1.0, 1.0)
        all_points = np.vstack([s.points for s in self.strokes])
        min_coords = all_points.min(axis=0)
        max_coords = all_points.max(axis=0)
        return (min_coords[0], min_coords[1], max_coords[0], max_coords[1])

    def add_stroke(self, stroke: Stroke) -> None:
        """Add a stroke to the sketch."""
        self.strokes.append(stroke)

    def to_dict(self) -> dict:
        """Convert sketch to a JSON-serializable dictionary."""
        return {
            "strokes": [
                {
                    "points": s.points.tolist(),
                    "closed": s.closed,
                }
                for s in self.strokes
            ],
            "source_size": self.source_size,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Sketch":
        """Create a Sketch from a dictionary (e.g., loaded from JSON)."""
        strokes = [
            Stroke(
                points=np.array(s["points"]),
                closed=s.get("closed", False),
            )
            for s in data.get("strokes", [])
        ]
        return cls(
            strokes=strokes,
            source_size=tuple(data["source_size"]) if data.get("source_size") else None,
        )
