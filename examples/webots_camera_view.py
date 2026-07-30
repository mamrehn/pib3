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
GAIN = 60.0          # how hard the head chases the target
DEAD_ZONE = 3.0      # ignore small errors, or the head jitters forever
SMOOTHING = 0.7      # 0 = no smoothing, 0.9 = very sluggish


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

        # --- 3. close the loop ----------------------------------------
        # step() returns False when Webots shuts the controller down, so it
        # doubles as the loop condition. It is also what produces the next
        # image: without it you would re-read the same frame forever.
        target = CENTRE
        while sim.step():
            objects = sim.ai.get_detections()
            if not objects:
                continue

            biggest = max(objects, key=lambda d: d.bbox.area)
            x, _ = biggest.bbox.center            # 0..1 across the image

            # Image position -> joint value. Centre of image -> 50 %.
            wanted = CENTRE + GAIN * (x - CENTRE / 100.0)
            wanted = SMOOTHING * target + (1 - SMOOTHING) * wanted

            if abs(wanted - target) > DEAD_ZONE:
                target = wanted
                # async_=True is essential in a control loop: the blocking
                # form would wait for the joint to arrive and stall the
                # simulation, so no new frame would ever appear.
                sim.set_joint(Joint.TURN_HEAD, target, async_=True)

            print(f"{biggest.label:>10}  x={x:.2f}  head={target:5.1f} %", end="\r")


if __name__ == "__main__":
    main()
