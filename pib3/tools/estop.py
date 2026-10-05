"""Teacher's emergency stop: freeze one or more pib robots from any laptop.

It does two things for each robot:

1. It freezes every servo directly through the robot's Tinkerforge daemon
   (port 4223). That works even if the student's program hangs, or if the
   program is not a pib3 program at all.
2. It tells every pib3 program connected to the robot to latch its emergency
   stop (rosbridge, port 9090). Without this step, the next command from the
   student's program would start the robot again.

Usage::

    python -m pib3.tools.estop --host 192.168.0.11            # stop now
    python -m pib3.tools.estop --host pib-01 --host pib-02     # several robots
    python -m pib3.tools.estop --host pib-01 --window          # STOP button
    python -m pib3.tools.estop --host pib-01 --relax           # PWM off: arms drop!

The robot itself does not latch anything, so a program that does not use
pib3 can start it again. Students continue in their own program with
``robot.resume()``, or they restart it.
"""

import argparse
import sys
import threading
from typing import List

from ..safety import StopButton, broadcast_stop, freeze_robot_servos


def stop_robots(hosts: List[str], relax: bool = False, source: str = "teacher") -> bool:
    """Freeze and latch every robot in ``hosts``. Returns True if all worked."""
    results = {}

    def one(host: str) -> None:
        frozen, latched = 0, False
        try:
            frozen = freeze_robot_servos(host, relax=relax)
        except Exception as exc:
            print(f"[{host}] servos: FAILED ({exc})")
        try:
            latched = broadcast_stop(host, source=source)
        except Exception as exc:
            print(f"[{host}] pib3 programs: not reached ({exc})")
        what = "switched off (limp)" if relax else "frozen"
        print(f"[{host}] {frozen} servo channels {what}; "
              f"pib3 programs {'latched' if latched else 'NOT latched'}")
        results[host] = frozen > 0

    threads = [threading.Thread(target=one, args=(h,)) for h in hosts]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return all(results.get(h) for h in hosts)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Emergency stop for pib robots (teacher tool).")
    parser.add_argument("--host", action="append", required=True,
                        help="robot IP or hostname; repeat for several robots")
    parser.add_argument("--window", action="store_true",
                        help="show a STOP button instead of stopping right away")
    parser.add_argument("--relax", action="store_true",
                        help="switch the servos off instead of holding them "
                             "(arms fall down - last resort)")
    args = parser.parse_args(argv)

    if not args.window:
        return 0 if stop_robots(args.host, relax=args.relax) else 1

    done = threading.Event()
    button = StopButton(
        on_stop=lambda: threading.Thread(
            target=stop_robots, args=(args.host, args.relax), daemon=True).start(),
        title=", ".join(args.host),
    )
    if not button.start():
        print(f"Cannot show the STOP window: {button.failure}")
        return 2
    print("STOP window open. Press Ctrl+C here to close it.")
    try:
        # Short waits: on Windows an untimed wait ignores Ctrl+C.
        while button.running and not done.wait(0.5):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        button.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
