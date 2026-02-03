"""Image to sketch conversion for pib3 package."""

from pathlib import Path
from typing import List, Optional, Tuple, Union
import numpy as np

from .config import ImageConfig
from .types import Sketch, Stroke

# Check for optional dependencies
try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False


def _check_dependencies():
    """Check that required dependencies are available."""
    if not HAS_PIL:
        raise ImportError(
            "Pillow is required for image processing. "
            "Install with: pip install Pillow"
        )
    if not HAS_CV2:
        raise ImportError(
            "opencv-python is required for contour detection. "
            "Install with: pip install opencv-python-headless"
        )


def _load_image_to_array(
    image: Union[str, Path, np.ndarray, "Image.Image"]
) -> Tuple[np.ndarray, Tuple[int, int]]:
    """
    Load image from various sources and convert to grayscale numpy array.

    Returns:
        Tuple of (grayscale_array, (width, height))
    """
    if isinstance(image, np.ndarray):
        arr = image
        if arr.ndim == 3:
            # Convert to grayscale
            if arr.shape[2] == 4:  # RGBA
                # Use luminosity method, respecting alpha
                alpha = arr[:, :, 3] / 255.0
                gray = np.mean(arr[:, :, :3], axis=2) * alpha + 255 * (1 - alpha)
                arr = gray.astype(np.uint8)
            else:  # RGB
                arr = np.mean(arr, axis=2).astype(np.uint8)
        height, width = arr.shape[:2]
        return arr, (width, height)

    if HAS_PIL:
        if isinstance(image, (str, Path)):
            img = Image.open(image)
        else:
            img = image  # Assume PIL Image

        width, height = img.size

        # Handle different image modes
        if img.mode == 'RGBA':
            img_array = np.array(img)
            alpha = img_array[:, :, 3]
            gray = np.mean(img_array[:, :, :3], axis=2)
            # Transparent pixels become white (background)
            arr = np.where(alpha > 128, gray, 255).astype(np.uint8)
        elif img.mode == 'LA':
            img_array = np.array(img)
            gray = img_array[:, :, 0]
            alpha = img_array[:, :, 1]
            arr = np.where(alpha > 128, gray, 255).astype(np.uint8)
        elif img.mode == 'P':
            img_rgba = img.convert('RGBA')
            img_array = np.array(img_rgba)
            alpha = img_array[:, :, 3]
            gray = np.mean(img_array[:, :, :3], axis=2)
            arr = np.where(alpha > 128, gray, 255).astype(np.uint8)
        else:
            if img.mode != 'L':
                img = img.convert('L')
            arr = np.array(img)

        return arr, (width, height)

    raise ValueError("Cannot load image: unsupported type and PIL not available")


def _threshold_image(
    gray_array: np.ndarray,
    threshold: int = 128,
    auto_foreground: bool = True,
) -> np.ndarray:
    """
    Convert grayscale image to binary.

    Args:
        gray_array: Grayscale image (0-255)
        threshold: Pixel value threshold
        auto_foreground: Auto-detect foreground as minority pixels

    Returns:
        Binary image where True = foreground (to draw)
    """
    # Apply threshold: pixels below threshold are foreground
    binary = gray_array < threshold

    if auto_foreground:
        fg_count = np.sum(binary)
        total_pixels = binary.size

        if fg_count > total_pixels / 2:
            # More foreground than background - invert
            binary = ~binary

    return binary


def _vectorize_contours_cv2(
    binary_image: np.ndarray,
    simplify_tolerance: float = 1.0,
) -> List[Tuple[np.ndarray, bool]]:
    """Vectorize contours using OpenCV."""
    img_uint8 = (binary_image.astype(np.uint8)) * 255

    contours_cv, _ = cv2.findContours(
        img_uint8,
        cv2.RETR_LIST,
        cv2.CHAIN_APPROX_NONE
    )

    contours = []
    for contour_cv in contours_cv:
        if len(contour_cv) < 3:
            continue

        # Simplify using approxPolyDP
        simplified = cv2.approxPolyDP(contour_cv, simplify_tolerance, closed=True)

        # Convert to numpy array of (x, y)
        points = np.array([[pt[0][0], pt[0][1]] for pt in simplified], dtype=np.float64)

        if len(points) >= 2:
            contours.append((points, True))  # OpenCV contours are typically closed

    return contours


def _vectorize_contours(
    binary_image: np.ndarray,
    simplify_tolerance: float = 1.0,
) -> List[Tuple[np.ndarray, bool]]:
    """Vectorize binary image to contours."""
    if HAS_CV2:
        return _vectorize_contours_cv2(binary_image, simplify_tolerance)
    else:
        raise ImportError("opencv-python not available")


def _normalize_coordinates(
    contours: List[Tuple[np.ndarray, bool]],
    image_width: int,
    image_height: int,
    margin: float = 0.05,
) -> List[Tuple[np.ndarray, bool]]:
    """Convert pixel coordinates to normalized [0, 1] range."""
    scale = 1.0 - 2 * margin
    normalized = []

    for points, closed in contours:
        norm_points = np.empty_like(points)
        norm_points[:, 0] = margin + (points[:, 0] / image_width) * scale
        norm_points[:, 1] = margin + (points[:, 1] / image_height) * scale
        normalized.append((norm_points, closed))

    return normalized


def _filter_small_contours(
    contours: List[Tuple[np.ndarray, bool]],
    min_length: float = 10.0,
    min_points: int = 3,
) -> List[Tuple[np.ndarray, bool]]:
    """Remove contours that are too small."""
    filtered = []
    for points, closed in contours:
        if len(points) < min_points:
            continue

        # Calculate total arc length
        diffs = np.diff(points, axis=0)
        length = np.sum(np.linalg.norm(diffs, axis=1))

        if length >= min_length:
            filtered.append((points, closed))

    return filtered


def _optimize_path_order(
    contours: List[Tuple[np.ndarray, bool]],
) -> List[Tuple[np.ndarray, bool]]:
    """Optimize contour order to minimize travel distance using nearest-neighbor."""
    if len(contours) <= 1:
        return contours

    remaining = list(range(len(contours)))
    ordered = []

    # Start with first contour
    current_idx = remaining.pop(0)
    points, closed = contours[current_idx]
    ordered.append((points, closed))
    current_end = points[-1]

    while remaining:
        min_dist = float('inf')
        min_idx = 0
        reverse = False

        for i, idx in enumerate(remaining):
            points, closed = contours[idx]
            if len(points) == 0:
                continue

            # Distance to start of contour
            dist_start = np.sum((points[0] - current_end) ** 2)

            # Distance to end of contour (would need to reverse)
            dist_end = np.sum((points[-1] - current_end) ** 2)

            if dist_start < min_dist:
                min_dist = dist_start
                min_idx = i
                reverse = False
            if dist_end < min_dist:
                min_dist = dist_end
                min_idx = i
                reverse = True

        # Add the nearest contour (possibly reversed)
        next_idx = remaining.pop(min_idx)
        points, closed = contours[next_idx]
        if reverse:
            points = points[::-1].copy()
        ordered.append((points, closed))
        current_end = points[-1]

    return ordered


def image_to_sketch(
    image: Union[str, Path, np.ndarray, "Image.Image"],
    config: Optional[ImageConfig] = None,
) -> Sketch:
    """
    Convert an image to a Sketch (collection of Strokes).

    Extracts contours from the image, simplifies them, normalizes coordinates
    to [0,1] range, and optimizes drawing order.

    Args:
        image: Input image as file path, numpy array (H,W or H,W,C), or PIL Image.
        config: Image processing configuration. Uses defaults if None.

    Returns:
        Sketch object containing extracted strokes with normalized coordinates.
        (0,0) = top-left, (1,1) = bottom-right.

    Raises:
        ImportError: If required dependencies (Pillow, scikit-image/opencv) are missing.
        FileNotFoundError: If image path doesn't exist.
        ValueError: If image format is unsupported.

    Example:
        >>> sketch = image_to_sketch("drawing.png")
        >>> print(f"Extracted {len(sketch)} strokes")
    """
    _check_dependencies()

    if config is None:
        config = ImageConfig()

    # Load image
    gray_array, (width, height) = _load_image_to_array(image)

    # Threshold to binary
    binary = _threshold_image(
        gray_array,
        threshold=config.threshold,
        auto_foreground=config.auto_foreground,
    )

    # Vectorize contours
    contours = _vectorize_contours(binary, config.simplify_tolerance)

    # Filter small contours (in pixel space)
    contours = _filter_small_contours(
        contours,
        min_length=config.min_contour_length,
        min_points=config.min_contour_points,
    )

    # Normalize to [0, 1]
    contours = _normalize_coordinates(contours, width, height, config.margin)

    # Optimize path order
    if config.optimize_path_order:
        contours = _optimize_path_order(contours)

    # Convert to Sketch
    strokes = [Stroke(points=points, closed=closed) for points, closed in contours]
    return Sketch(strokes=strokes, source_size=(width, height))
