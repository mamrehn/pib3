#!/usr/bin/env python3
"""
Webots camera & AI self-check — run this first after touching pib.proto.

Verifies the simulated perception path end to end and tells you exactly which
line to fix when something is wrong. Every check is independent: a failure is
reported and the script keeps going, so one run gives you the full picture.

What it checks
    1.  pib3 imports inside Webots' interpreter
    2.  the backend connects and the camera device exists
    3.  frames arrive with the expected shape and dtype
    4.  the camera is ON THE HEAD — turning the head must change the image
    5.  Recognition is enabled and returns objects
    6.  detection boxes are normalized to 0..1 and land inside the image
    7.  THE CLOSED LOOP: turning the head moves a detected object across the
        image in the direction a controller would expect
    8.  (optional) an ultralytics model runs on simulated frames

Usage — this is a WEBOTS CONTROLLER, not a standalone script:
    1. Copy this file into your world's  controllers/webots_camera_check/
       directory (Webots wants controllers/<name>/<name>.py), or point an
       existing controller at it.
    2. Set the pib robot's `controller` field to "webots_camera_check".
    3. Run the simulation and read the console.

Headless / CI:
    webots --batch --mode=fast --stdout --stderr your_world.wbt

For checks 5-7 the world needs at least one recognizable object in front of
the robot. A Solid is only recognized if it sets `recognitionColors`, and its
`model` field becomes the detection label:

    Solid {
      translation 0.6 0 1.0          # ~60 cm in front, roughly head height
      children [ Shape {
        appearance PBRAppearance { baseColor 1 0 0 roughness 1 metalness 0 }
        geometry Sphere { radius 0.06 }
      } ]
      name "testball"
      model "ball"                   # -> det.label
      recognitionColors [ 1 0 0 ]    # WITHOUT THIS IT IS NEVER RECOGNIZED
    }

Images are written next to the controller so you can eyeball orientation:
    check_center.png / check_left.png / check_right.png
"""

import sys
import traceback

# --- results bookkeeping --------------------------------------------------

_results = []


def record(name, ok, detail="", fatal=False):
    """Record one check and print it immediately."""
    _results.append((name, ok, detail, fatal))
    mark = "PASS" if ok else ("FAIL" if fatal else "WARN")
    print(f"[{mark}] {name}")
    if detail:
        for line in str(detail).splitlines():
            print(f"       {line}")
    return ok


def summary():
    print("\n" + "=" * 70)
    failed = [r for r in _results if not r[1] and r[3]]
    warned = [r for r in _results if not r[1] and not r[3]]
    passed = [r for r in _results if r[1]]
    print(f"SUMMARY: {len(passed)} passed, {len(warned)} warnings, {len(failed)} failures")
    for name, _, detail, _ in failed:
        print(f"  FAIL  {name}")
    for name, _, detail, _ in warned:
        print(f"  WARN  {name}")
    print("=" * 70)
    return 1 if failed else 0


# --- 1. import ------------------------------------------------------------

try:
    import numpy as np
    import pib3
    from pib3 import Joint
    record("1. pib3 imports inside Webots", True, f"pib3 {pib3.__version__}")
except Exception as exc:
    print("[FAIL] 1. pib3 imports inside Webots")
    print(f"       {exc}")
    print("       Webots is not using the Python interpreter that has pib3.")
    print("       Fix: Webots -> Preferences -> General -> 'Python command',")
    print("       point it at your venv's python.")
    sys.exit(1)


def settle(sim, steps=5):
    """Advance a few steps so sensors and the camera produce valid data."""
    for _ in range(steps):
        if not sim.step():
            return False
    return True


def save(img, filename):
    """Best-effort PNG dump for eyeballing; never fails the run."""
    try:
        import cv2
        cv2.imwrite(filename, img)
        return filename
    except Exception as exc:
        return f"(not saved: {exc})"


def brightest_x(img):
    """Column of the image's centre of mass, normalized 0..1.

    A crude, dependency-free way to ask "where is the stuff in this image?",
    used to detect that the view actually shifts when the head turns.
    """
    grey = img.mean(axis=2)
    weights = grey.sum(axis=0)
    total = weights.sum()
    if total <= 0:
        return None
    xs = np.arange(len(weights))
    return float((xs * weights).sum() / total / len(weights))


def main():
    with pib3.Webots() as sim:
        record("2a. backend connected", sim.is_connected)

        # --- 2b. step() -----------------------------------------------
        ok = sim.step()
        record("2b. step() advances the simulation", bool(ok),
               "" if ok else "step() returned False immediately — simulation not running?",
               fatal=True)
        if not ok:
            return summary()

        # --- 2c. camera device ----------------------------------------
        cam = sim.camera
        has_cam = cam._ensure_enabled()
        record(
            "2c. camera device found", has_cam,
            "" if has_cam else
            "No Webots device named 'camera'.\n"
            "The Camera node is missing from the robot. Check pib.proto, and\n"
            "note that an existing world keeps its OLD proto copy — re-import\n"
            "or delete the cached PROTO in your world's protos/ folder.",
            fatal=True,
        )
        if not has_cam:
            return summary()

        record("2d. camera resolution", cam.width > 0 and cam.height > 0,
               f"{cam.width} x {cam.height}")

        # --- 3. frames -------------------------------------------------
        settle(sim)
        frame = cam.get_frame()
        if not record("3a. frame received", frame is not None,
                      "" if frame is not None else
                      "get_frame() returned None after several steps.", fatal=True):
            return summary()

        img = frame.to_numpy()
        shape_ok = (
            img is not None
            and img.ndim == 3
            and img.shape[2] == 3
            and img.shape[0] == cam.height
            and img.shape[1] == cam.width
            and img.dtype == np.uint8
        )
        record("3b. frame shape/dtype", shape_ok,
               f"{img.shape} {img.dtype}" if img is not None else "to_numpy() -> None")

        record("3c. frame is not uniformly black", bool(img.max() > 5),
               f"max pixel value = {int(img.max())}\n"
               "All black usually means the camera looks into the robot's own\n"
               "head, or the world has no lighting.")

        # jpeg round-trip (raw frames carry no jpeg_bytes)
        jpg = frame.to_jpeg()
        record("3d. to_jpeg() encodes on demand",
               bool(jpg) and jpg[:2] == b"\xff\xd8",
               f"{len(jpg)} bytes" if jpg else "encode failed (cv2 missing?)")

        # --- 4. camera moves WITH the head ------------------------------
        sim.go_home()
        settle(sim, 10)
        centre = cam.get_frame().to_numpy().copy()
        f_centre = save(centre, "check_center.png")

        sim.set_joint(Joint.TURN_HEAD, 15.0, async_=True)
        settle(sim, 30)
        left = cam.get_frame().to_numpy().copy()
        f_left = save(left, "check_left.png")

        sim.set_joint(Joint.TURN_HEAD, 85.0, async_=True)
        settle(sim, 30)
        right = cam.get_frame().to_numpy().copy()
        f_right = save(right, "check_right.png")

        diff = float(np.abs(left.astype(np.int16) - right.astype(np.int16)).mean())
        record(
            "4. camera is mounted on the head", diff > 1.0,
            f"mean pixel difference between head-left and head-right: {diff:.2f}\n"
            f"images: {f_centre}, {f_left}, {f_right}\n" +
            ("" if diff > 1.0 else
             "The image does NOT change when the head turns. The Camera node is\n"
             "not a child of the head — it must sit inside urdf_camera_link.\n"
             "Without this there is no closed loop and servoing cannot work."),
            fatal=True,
        )

        # Orientation is a human judgement: point at the saved images.
        record("4b. orientation (manual check)", True,
               f"Open {f_centre} and confirm the robot looks OUT at the scene.\n"
               "Inside of the head / ceiling / floor => fix the `rotation` line\n"
               "of the Camera node in pib.proto (cameras look along +x).")

        # --- 5. recognition ---------------------------------------------
        sim.go_home()
        settle(sim, 10)
        ok_model = sim.ai.set_model("recognition")
        record("5a. Recognition enabled", ok_model,
               "" if ok_model else
               "Camera has no Recognition child, or recognitionEnable() failed.")

        # recognitionEnable() obeys the same rule as every Webots sensor: the
        # simulator reports nothing until the next step. Reading immediately
        # after set_model() returns an empty list — and, because results are
        # cached per frame, that emptiness sticks for the current frame too.
        # latest_only=True everywhere below: without it the receiver hands back
        # every buffered frame (up to 100), oldest first — so a reader that
        # takes the first match keeps seeing a detection from before the robot
        # moved. That is the single easiest mistake to make with this API.
        dets = []
        for _ in range(5):
            settle(sim, 5)
            dets = sim.ai.get_detections(latest_only=True)
            if dets:
                break
        record(
            "5b. objects recognized", len(dets) > 0,
            f"{len(dets)} object(s): " + ", ".join(d.label or '<no model>' for d in dets)
            if dets else
            "No objects recognized. Either nothing is in view, or your Solids\n"
            "lack `recognitionColors` (without it they are never reported).\n"
            "See the world snippet in this file's docstring.",
        )

        # --- 6. bbox sanity ----------------------------------------------
        if dets:
            bad = [
                d for d in dets
                if not (0.0 <= d.bbox.xmin <= d.bbox.xmax <= 1.0
                        and 0.0 <= d.bbox.ymin <= d.bbox.ymax <= 1.0)
            ]
            record("6a. boxes normalized to 0..1", not bad,
                   "\n".join(f"{d.label}: {d.bbox}" for d in bad) if bad else
                   "\n".join(f"{d.label}: center={d.bbox.center} area={d.bbox.area:.3f}"
                             for d in dets))
            record("6b. ground truth has confidence 1.0",
                   all(abs(d.confidence - 1.0) < 1e-6 for d in dets))
            record("6c. labels are names, not numbers",
                   all(isinstance(d.label, str) for d in dets),
                   "det.label should be the Solid's `model` field")

        # --- 7. the closed loop -------------------------------------------
        if dets:
            target = max(dets, key=lambda d: d.bbox.area).label

            def x_of(label):
                """Where `label` sits horizontally in the CURRENT frame, 0..1."""
                for _ in range(3):
                    settle(sim, 5)
                    for d in sim.ai.get_detections(latest_only=True):
                        if d.label == label:
                            return d.bbox.center[0]
                return None

            sim.set_joint(Joint.TURN_HEAD, 40.0, async_=True)
            settle(sim, 30)
            x_a = x_of(target)

            sim.set_joint(Joint.TURN_HEAD, 60.0, async_=True)
            settle(sim, 30)
            x_b = x_of(target)

            if x_a is None or x_b is None:
                record("7. closed loop: head turn moves the target", False,
                       f"'{target}' left the field of view during the sweep.\n"
                       "Place the test object further away or reduce the sweep.")
            else:
                shift = x_b - x_a
                moved = abs(shift) > 0.02
                if moved:
                    # Sensitivity of image position to joint command, measured
                    # over the 40 % -> 60 % sweep above.
                    sens = shift / 20.0
                    op = "+=" if sens < 0 else "-="
                    deadbeat = 1.0 / abs(sens)
                    detail = (
                        f"target '{target}': x went {x_a:.3f} -> {x_b:.3f} "
                        f"(shift {shift:+.3f})\n"
                        f"sensitivity dx/djoint = {sens:+.4f} per % of joint range\n"
                        f"\n"
                        f"So the controller for this robot is:\n"
                        f"    target {op} K * (x - 0.5)\n"
                        f"    K = {deadbeat:.0f}  reaches the centre in one step\n"
                        f"    K < {2 * deadbeat:.0f}  stays stable; above that it oscillates\n"
                        f"    K ~ {1.6 * deadbeat:.0f}  rings visibly — the demo worth showing"
                    )
                else:
                    detail = (
                        "The target does not move in the image when the head turns.\n"
                        "If check 4 passed, the camera IS on the head, so suspect a\n"
                        "stale read: use get_detections(latest_only=True), otherwise\n"
                        "you get every buffered frame and the oldest one never moves."
                    )
                record("7. closed loop: head turn moves the target", moved, detail)

        # --- 8. optional: real model on simulated frames -------------------
        try:
            import ultralytics  # noqa: F401
        except ImportError:
            record("8. ultralytics path (optional)", True,
                   "ultralytics not installed — skipped.")
        else:
            if sim.ai.set_model("yolov8n"):
                settle(sim, 5)
                yolo_dets = sim.ai.get_detections(latest_only=True)
                record("8. ultralytics runs on simulated frames", True,
                       f"{len(yolo_dets)} detection(s)"
                       + ("" if yolo_dets else
                          "\nZero detections is EXPECTED in an untextured world:\n"
                          "COCO models see almost nothing in synthetic scenes.\n"
                          "Use 'recognition' for teaching, or texture the objects."))
            else:
                record("8. ultralytics model load", False,
                       "set_model('yolov8n') failed — weights not available offline?")

    return summary()


if __name__ == "__main__":
    try:
        code = main()
    except Exception:
        traceback.print_exc()
        print("\n[FAIL] unhandled exception — see traceback above")
        code = 1
    print(f"\nexit code {code}")
    sys.exit(code)
