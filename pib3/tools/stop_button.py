"""Big red on-screen STOP button for the pib3 emergency stop.

Started by :class:`pib3.safety.StopButton` as a child process; you do not
normally run it yourself. Protocol on stdin/stdout, one word per line:

* child -> parent: ``READY`` once the window is up, ``STOP`` on every press,
  ``ERROR <text>`` if the window cannot open (e.g. no tkinter).
* parent -> child: ``STOPPED <reason>`` / ``RESUMED`` to update the look.
  End of input (the program ended) closes the window.

Run by hand to try it out::

    python -m pib3.tools.stop_button --title "pib-01"
"""

import argparse
import queue
import signal
import sys
import threading

RED = "#c62828"
RED_ACTIVE = "#e53935"
GREY = "#424242"
YELLOW = "#ffd54f"


def _say(word: str) -> None:
    try:
        sys.stdout.write(word + "\n")
        sys.stdout.flush()
    except (OSError, ValueError):
        pass


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--title", default="pib3")
    args = parser.parse_args(argv)

    # Ctrl+C in the terminal belongs to the user's program, which freezes
    # the robot and tells us. Dying here would only remove the button.
    try:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
    except (ValueError, OSError):
        pass

    try:
        import tkinter as tk
    except Exception as exc:  # pragma: no cover - depends on the Python build
        _say(f"ERROR tkinter is not available ({exc}). On Debian/Ubuntu: "
             f"sudo apt install python3-tk")
        return 3

    try:
        root = tk.Tk()
    except Exception as exc:  # pragma: no cover - no display
        _say(f"ERROR cannot open a window ({exc})")
        return 3

    root.title(f"STOP - {args.title}")
    root.configure(bg=RED)
    root.attributes("-topmost", True)
    width, height = 300, 236
    x = max(0, root.winfo_screenwidth() - width - 40)
    root.geometry(f"{width}x{height}+{x}+60")
    root.minsize(220, 160)

    big = tk.Label(root, text="STOP", bg=RED, fg="white",
                   font=("Helvetica", 56, "bold"), cursor="hand2")
    big.pack(expand=True, fill="both")
    small = tk.Label(root, text=f"{args.title}\nclick · Space · Esc",
                     bg=RED, fg="white", font=("Helvetica", 12))
    small.pack(fill="x", pady=(0, 10))

    state = {"stopped": False}

    def press(_event=None):
        _say("STOP")
        show_stopped("STOP button")
        return "break"

    def show_stopped(reason: str) -> None:
        state["stopped"] = True
        for w in (root, big, small):
            w.configure(bg=GREY)
        big.configure(text="STOPPED", fg=YELLOW, font=("Helvetica", 34, "bold"))
        small.configure(text=f"{reason}\nMotors hold position.\nresume() or restart the program",
                        fg="white")

    def show_ready() -> None:
        state["stopped"] = False
        for w in (root, big, small):
            w.configure(bg=RED)
        big.configure(text="STOP", fg="white", font=("Helvetica", 56, "bold"))
        small.configure(text=f"{args.title}\nclick · Space · Esc")

    def hover(on: bool) -> None:
        if not state["stopped"]:
            for w in (root, big, small):
                w.configure(bg=RED_ACTIVE if on else RED)

    for widget in (root, big, small):
        widget.bind("<Button-1>", press)
    big.bind("<Enter>", lambda _e: hover(True))
    big.bind("<Leave>", lambda _e: hover(False))
    for key in ("<space>", "<Escape>", "<Return>", "<KP_0>", "<KP_Insert>", "<KP_Enter>"):
        root.bind(key, press)

    def on_close():
        # The button disappears together with the program. Closing it by
        # accident must not remove the emergency stop.
        small.configure(text="This window closes\nwhen your program ends.")

    root.protocol("WM_DELETE_WINDOW", on_close)

    # Tk may only be touched from this thread, so the reader thread hands
    # lines over through a queue that the Tk loop polls.
    inbox = queue.Queue()

    def read_parent():
        for line in sys.stdin:
            inbox.put(line)
        inbox.put(None)                   # parent gone -> close

    def poll():
        try:
            while True:
                line = inbox.get_nowait()
                if line is None:
                    root.destroy()
                    return
                word, _, rest = line.strip().partition(" ")
                if word == "STOPPED":
                    show_stopped(rest or "Emergency stop")
                elif word == "RESUMED":
                    show_ready()
        except queue.Empty:
            pass
        root.after(50, poll)

    threading.Thread(target=read_parent, daemon=True).start()
    root.after(50, poll)
    root.after(100, lambda: _say("READY"))
    root.after(150, root.focus_force)
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
