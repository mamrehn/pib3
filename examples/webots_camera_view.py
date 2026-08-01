#!/usr/bin/env python3
"""Get the camera image in Webots — the smallest useful example.

The simulated pib has a camera in its head, so it sees its own world. Because
the head moves the camera, "turn the head towards what you see" is a genuine
closed loop: the image changes as a result of the motion.

Run it as a WEBOTS CONTROLLER (Webots starts the file; `python this.py` will
not work):
    1. copy to  controllers/webots_camera_view/webots_camera_view.py
    2. set the pib robot's `controller` field to "webots_camera_view"
    3. press Play

For the detection part the world needs a Solid that opts into recognition —
`recognitionColors` is what makes it visible to the simulator, and `model`
becomes the label:

    Solid {
      translation 0 -0.6 1.0
      children [ Shape {
        appearance PBRAppearance { baseColor 1 0 0 roughness 1 metalness 0 }
        geometry Sphere { radius 0.06 }
      } ]
      name "testball"
      model "ball"
      recognitionColors [ 1 0 0 ]
    }

Something not working? Run examples/webots_camera_check.py — it diagnoses the
camera, the mounting and the recognition setup one step at a time.
"""

import cv2

import pib3
from pib3 import Joint

# Head turn, in percent of the joint range. 50 % is straight ahead.
CENTRE = 50.0

# Raising TURN_HEAD moves the image content LEFT, i.e. dx/djoint is negative
# (measured at -0.027 per % with examples/webots_camera_check.py). So the
# correction is ADDED: a target right of centre (x > 0.5) needs a larger joint
# value. K = 37 would centre in a single step; 20 leaves comfortable margin.
# The loop goes unstable above K = 75.
GAIN = 20.0
DEAD_ZONE = 0.02     # in image units: ignore sub-2 % errors or the head jitters


def main():
    with pib3.Webots() as sim:
        # --- 1. a single image ---------------------------------------
        # Nothing renders before the simulation has stepped at least once.
        sim.step()

        frame = sim.camera.get_frame()
        img = frame.to_numpy()                   # HxWx3, BGR — OpenCV order
        print(f"frame {frame.frame_id}: {img.shape[1]}x{img.shape[0]}")
        cv2.imwrite("view.png", img)             # imshow needs a GUI build

        # --- 2. what the robot sees -----------------------------------
        # "recognition" is the simulator's ground truth: exact boxes, no
        # model, confidence always 1.0. Swap in "yolov8n" to run a real
        # network on the same frames (pip install "pib3[sim]").
        sim.ai.set_model("recognition")

        # --- 3. show it in the 3D window ------------------------------
        # Mirrors the camera onto the Display panel declared in pib.proto.
        # Call it once: Webots then keeps the panel filled by itself, so
        # anyone watching the simulation sees what the robot sees.
        sim.camera.show_on_display()

        # --- 4. close the loop ----------------------------------------
        # step() returns False when Webots shuts the controller down, so it
        # doubles as the loop condition. It is also what produces the next
        # image: without it you would re-read the same frame forever.
        target = CENTRE
        while sim.step():
            # latest_only=True matters: the default returns EVERY buffered
            # frame, so a loop reading the first entry would keep acting on a
            # detection from before the head moved.
            objects = sim.ai.get_detections(latest_only=True)
            if not objects:
                continue

            # Paint the boxes onto the live panel. Once per step: attaching
            # the camera repaints it with a fresh image every step, which
            # erases last step's drawing.
            sim.camera.draw_detections(objects)

            biggest = max(objects, key=lambda d: d.bbox.area)
            x, _ = biggest.bbox.center            # 0..1 across the image
            error = x - 0.5                       # >0 means "target is right of centre"

            if abs(error) > DEAD_ZONE:
                # Incremental, so it works from any head position — unlike
                # "joint = 50 + K*error", which silently assumes the head
                # started centred.
                target = min(max(target + GAIN * error, 0.0), 100.0)
                # async_=True is essential in a control loop: the blocking
                # form waits for the joint to arrive and stalls the
                # simulation, so no new frame would ever appear.
                sim.set_joint(Joint.TURN_HEAD, target, async_=True)

            print(f"{biggest.label:>10}  x={x:.2f}  head={target:5.1f} %", end="\r")


if __name__ == "__main__":
    main()
