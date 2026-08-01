#!/usr/bin/env python3
"""Template: your own Webots controller, with the live camera panel.

Nothing in pib3 requires you to use the provided controllers. They are plain
scripts; this is the skeleton to copy when you want your own — live view,
detections, and whatever commands you like, side by side.

Install as any other controller (directory name must equal the file name):
    controllers/my_controller/my_controller.py
then set the pib robot's `controller` field to "my_controller".

If pib3 is not on the path, drop a runtime.ini next to this file:

    [environment variables with paths]
    PYTHONPATH = /path/to/pib3/repo:/path/to/venv/lib/python3.X/site-packages
"""

import pib3
from pib3 import Joint

STATION_MODEL = "recognition"     # or "yolov8n" with pib3[sim] installed


def main():
    with pib3.Webots() as sim:
        # ------------------------------------------------------------------
        # Setup — runs once.
        # ------------------------------------------------------------------
        sim.ai.set_model(STATION_MODEL)
        sim.camera.show_on_display()      # live panel in the 3D view
        sim.go_home()

        # The camera is enabled by the first get_frame(), and Webots needs one
        # step after enabling before an image exists — so the first call
        # returns None. Wait for a real frame rather than assuming one.
        while sim.camera.get_frame() is None and sim.step():
            pass

        frame_no = 0

        # ------------------------------------------------------------------
        # Main loop. sim.step() advances time AND produces the next image;
        # it returns False when Webots shuts the controller down.
        # ------------------------------------------------------------------
        while sim.step():
            frame_no += 1

            # --- perception -----------------------------------------------
            # latest_only=True: without it you get every buffered frame and
            # end up acting on a detection from several steps ago.
            objects = sim.ai.get_detections(latest_only=True)
            sim.camera.draw_detections(objects)     # boxes onto the panel

            # The raw pixels, if you want to do your own OpenCV work:
            #   img = sim.camera.get_frame().to_numpy()      # HxWx3 BGR

            # --- your commands go here ------------------------------------
            # async_=True in a loop: the blocking form waits for the joint to
            # arrive and stalls the simulation, so no new frame would appear.
            if objects:
                biggest = max(objects, key=lambda d: d.bbox.area)
                print(f"[{frame_no}] {biggest.label} at {biggest.bbox.center}", end="\r")

            # Anything from the pib3 API works here, exactly as on the real
            # robot — set_joint, set_joints, set_joints_pose, run_trajectory:
            #
            #   sim.set_joint(Joint.ELBOW_LEFT, 60.0, async_=True)
            #   sim.set_joints({Joint.TURN_HEAD: 50.0}, async_=True)
            #   sim.set_joints_pose(HandPose.LEFT_OPEN, async_=True)
            #
            # Read positions back with sim.get_joint(Joint.ELBOW_LEFT).


if __name__ == "__main__":
    main()
