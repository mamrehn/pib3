"""Big red on-screen STOP button for the pib3 emergency stop.

The window is the visible sign that the emergency stop is armed: it opens when
a program first moves the robot, lists every way to stop it, and closes when
the program ends.

Started by :class:`pib3.safety.StopButton` as a child process; you do not
normally run it yourself. Protocol on stdin/stdout, one word per line:

* child -> parent: ``READY`` once the window is up, ``STOP`` on every press,
  ``ERROR <text>`` if the window cannot open (e.g. no tkinter).
* parent -> child: ``STOPPED <reason>`` / ``RESUMED`` to update the look.
  End of input (the program ended) closes the window.

Run by hand to try it out::

    python pib3/tools/stop_button.py --title pib-01 --ausloeser click,space,esc,ctrl_c
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

#: Every text the window shows, per language. Trigger names are the tokens the
#: parent passes with --ausloeser.
TEXTE = {
    "de": {
        "scharf": "NOT-AUS SCHARF", "gestoppt": "GESTOPPT",
        "click": "Klick", "space": "Leertaste", "space3d": "Leertaste im 3D-Fenster",
        "esc": "Esc", "kp_0": "Num 0", "pause": "Pause", "ctrl_c": "Strg+C",
        "robot": "hält den ganzen Roboter an", "sim": "hält die Simulation an",
        "halten": "Motoren halten die Stellung.", "neustart": "Programm neu starten.",
        "zu": "Schließt sich, wenn das Programm endet.",
    },
    "en": {
        "scharf": "E-STOP ARMED", "gestoppt": "STOPPED",
        "click": "click", "space": "Space", "space3d": "Space in the 3D view",
        "esc": "Esc", "kp_0": "Num 0", "pause": "Pause", "ctrl_c": "Ctrl+C",
        "robot": "stops the whole robot", "sim": "stops the simulation",
        "halten": "Motors hold position.", "neustart": "Restart the program.",
        "zu": "Closes when the program ends.",
    },
}


#: The parent names the stop reason in English; the German window translates
#: the ones pib3 produces.
GRUENDE_DE = {
    "Space key": "Leertaste", "Esc key": "Esc", "Numpad-0 key": "Num 0",
    "Pause key": "Pause", "STOP button": "STOP-Knopf", "Ctrl+C": "Strg+C",
}


def _grund(text: str, sprache: str) -> str:
    if sprache != "de":
        return text
    if text in GRUENDE_DE:
        return GRUENDE_DE[text]
    if text.startswith("stop from "):
        quelle, _, grund = text[len("stop from "):].partition(": ")
        return f"Stopp von {quelle}: {_grund(grund, sprache)}" if grund else f"Stopp von {quelle}"
    if text.startswith("remote stop ("):
        return "Fern-Stopp " + text[len("remote stop "):]
    return text


def _say(word: str) -> None:
    try:
        sys.stdout.write(word + "\n")
        sys.stdout.flush()
    except (OSError, ValueError):
        pass


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--title", default="pib3")
    parser.add_argument("--ausloeser", default="click",
                        help="comma-separated triggers: click,space,space3d,esc,ctrl_c,...")
    parser.add_argument("--wirkung", default="robot", choices=("robot", "sim"))
    parser.add_argument("--sprache", default="en", choices=sorted(TEXTE))
    args = parser.parse_args(argv)
    t = TEXTE[args.sprache]
    ausloeser = " · ".join(t.get(a, a) for a in args.ausloeser.split(",") if a)

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

    root.title(f"{t['scharf']} - {args.title}")
    root.configure(bg=RED)
    root.attributes("-topmost", True)
    width, height = 300, 236
    x = max(0, root.winfo_screenwidth() - width - 40)
    root.geometry(f"{width}x{height}+{x}+60")
    root.minsize(220, 180)

    kopf = tk.Label(root, bg=RED, fg="white", font=("Helvetica", 11, "bold"))
    kopf.pack(fill="x", pady=(8, 0))
    gross = tk.Label(root, bg=RED, fg="white", cursor="hand2")
    gross.pack(expand=True, fill="both")
    klein = tk.Label(root, bg=RED, fg="white", font=("Helvetica", 11), justify="center")
    klein.pack(fill="x", pady=(0, 10))
    widgets = (root, kopf, gross, klein)

    state = {"stopped": False}

    def show_armed() -> None:
        state["stopped"] = False
        for w in widgets:
            w.configure(bg=RED)
        kopf.configure(text=f"{t['scharf']} · {args.title}", fg="white")
        gross.configure(text="STOP", fg="white", font=("Helvetica", 56, "bold"))
        klein.configure(text=f"{ausloeser}\n{t[args.wirkung]}")

    def show_stopped(reason: str) -> None:
        state["stopped"] = True
        for w in widgets:
            w.configure(bg=GREY)
        kopf.configure(text=f"{t['gestoppt']} · {args.title}", fg=YELLOW)
        gross.configure(text=t["gestoppt"], fg=YELLOW, font=("Helvetica", 34, "bold"))
        klein.configure(text=f"{reason}\n{t['halten']}\n{t['neustart']}")

    def press(_event=None):
        _say("STOP")
        show_stopped(_grund("STOP button", args.sprache))   # the parent's STOPPED line follows
        return "break"

    def hover(on: bool) -> None:
        if not state["stopped"]:
            for w in widgets:
                w.configure(bg=RED_ACTIVE if on else RED)

    for widget in widgets:
        widget.bind("<Button-1>", press)
    gross.bind("<Enter>", lambda _e: hover(True))
    gross.bind("<Leave>", lambda _e: hover(False))
    # Stop keys while the window has focus (where a global key hook is not
    # allowed). No Enter: a window that pops up under a typing hand must not
    # turn the next Return into a stop.
    for key in ("<space>", "<Escape>", "<KP_0>", "<KP_Insert>"):
        root.bind(key, press)

    def on_close():
        # The button disappears together with the program. Closing it by
        # accident must not remove the emergency stop.
        klein.configure(text=t["zu"])

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
                    show_stopped(_grund(rest, args.sprache) or t["gestoppt"])
                elif word == "RESUMED":
                    show_armed()
        except queue.Empty:
            pass
        root.after(50, poll)

    show_armed()
    threading.Thread(target=read_parent, daemon=True).start()
    root.after(50, poll)
    root.after(100, lambda: _say("READY"))
    root.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
