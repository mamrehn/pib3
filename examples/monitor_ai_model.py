#!/usr/bin/env python3
"""
Example: Monitor the AI models of the robot

Prints a line whenever a model of the OAK-D Lite camera changes state
(``idle``, ``starting``, ``running``, ``failed``), with its results per second.
Start or stop models from another terminal, script or cerebra to see updates.
A model that does not come up shows ``failed`` with the reason.

Usage:
    python monitor_ai_model.py --host <robot_ip>
"""

import argparse
import time

from pib3 import Robot


def main():
    parser = argparse.ArgumentParser(description="Monitor the AI models of the robot")
    parser.add_argument("--host", default="172.26.34.149", help="Robot IP address")
    parser.add_argument("--seconds", type=float, default=30.0, help="How long to watch")
    args = parser.parse_args()

    print(f"Connecting to robot at {args.host}...")

    with Robot(host=args.host) as robot:
        print("Connected.")
        last = {}

        def on_status(message):
            # /models_status lists every model, about once a second
            for model in message.get("models", []):
                state = (model["state"], model["message"])
                if last.get(model["model_id"]) == state:
                    continue
                last[model["model_id"]] = state
                if model["state"] == "idle" and not model["active"]:
                    continue
                extra = f" - {model['message']}" if model["message"] else ""
                print(f"  {model['model_id']:36s} {model['state']:9s} "
                      f"{model['fps']:6.1f} results/s{extra}")

        sub = robot.subscribe_ai_status(on_status)
        print(f"\nWatching for {args.seconds:.0f} seconds. Press Ctrl+C to exit early.")
        try:
            time.sleep(args.seconds)
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            sub.unsubscribe()


if __name__ == "__main__":
    main()
