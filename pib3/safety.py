"""Emergency stop ("Notaus") for pib3.

A classroom needs a stop that works on *every* laptop, not only on the ones
with a numeric keypad. pib3 therefore offers several independent triggers.
All of them do the same thing — they call ``robot.stop()``:

1. **Keys, anywhere on the desktop:** Space, Esc, Pause or Numpad-0. Space and
   Esc exist on every laptop keyboard, and Space is the key you hit blindly
   in a hurry. This uses a global keyboard hook (``pynput``), which some
   systems block — see :func:`keyboard_hook_problem`.
2. **Ctrl+C in the terminal** that runs the program. This works on every
   system and needs no permission.
3. **An on-screen STOP button** (:class:`StopButton`). You can click it with
   a touchpad, mouse or touchscreen. pib3 opens it by itself when the keys
   cannot work.
4. **The teacher's remote stop:** ``python -m pib3.tools.estop --host <ip>``.
   It freezes the servos directly on the robot and tells every pib3 program
   connected to that robot to latch its stop.

``stop()`` *latches*, as an emergency stop must (ISO 13850). The motors hold
their current position, and every later motion command raises
:class:`EmergencyStopError` until the program calls ``robot.resume()`` on
purpose. Pressing the key a second time does **not** resume, so a panicked
double press cannot restart the motion.
"""

from __future__ import annotations

import json
import locale
import logging
import os
import signal
import subprocess
import sys
import threading
import time
from typing import Callable, Dict, FrozenSet, Iterable, List, Optional, Sequence, Set, Tuple, Union

logger = logging.getLogger(__name__)


class EmergencyStopError(RuntimeError):
    """Raised by motion commands while the emergency stop is latched.

    Call ``robot.resume()`` to continue on purpose. A program that does not
    catch this error ends, and that is usually what you want after an
    emergency stop.
    """


# ==================== KEYS ====================

#: Keys that trigger the stop by default. Space and Esc exist on every
#: laptop. Numpad-0 and Pause are kept for desktop keyboards and older course
#: material.
DEFAULT_STOP_KEYS: Tuple[str, ...] = ("space", "esc", "kp_0", "pause")

_KEY_ALIASES = {
    "space": "space", "spacebar": "space", "leertaste": "space", " ": "space",
    "esc": "esc", "escape": "esc",
    "kp_0": "kp_0", "kp0": "kp_0", "numpad0": "kp_0", "numpad_0": "kp_0",
    "num0": "kp_0", "kp_insert": "kp_0",
    "pause": "pause", "break": "pause",
    "enter": "enter", "return": "enter",
    "insert": "insert", "delete": "delete", "backspace": "backspace",
    "tab": "tab", "home": "home", "end": "end",
}

# Virtual key codes of Numpad-0, per platform. pynput's own KeyCode equality
# also compares platform-private fields (the Windows scan code, the X11 symbol
# name), so a key built with ``KeyCode.from_vk(96)`` never equals a real key
# press. We match the numbers ourselves instead.
_WIN_VK_NUMPAD0 = 96          # VK_NUMPAD0
_MAC_VK_KEYPAD0 = 82          # kVK_ANSI_Keypad0
_X11_KEYSYMS_KP0 = (0xFFB0, 0xFF9E)   # XK_KP_0, XK_KP_Insert


def normalize_key_name(name: str) -> str:
    """Turn a user-facing key name into pib3's canonical name.

    Accepts ``"space"``, ``"Esc"``, ``"escape"``, ``"KP_0"``, ``"numpad0"``,
    ``"pause"``, ``"F1"`` ... ``"F24"`` and any single character.

    Raises:
        ValueError: for names pib3 cannot match reliably on every system.
    """
    if not isinstance(name, str) or not name:
        raise ValueError(f"Key name must be a non-empty string, got {name!r}")
    if len(name) == 1:
        return _KEY_ALIASES.get(name, name.lower())
    key = name.strip().lower().replace("-", "_")
    if key in _KEY_ALIASES:
        return _KEY_ALIASES[key]
    if key.startswith("f") and key[1:].isdigit() and 1 <= int(key[1:]) <= 24:
        return key
    raise ValueError(
        f"Unknown stop key {name!r}. Use one of: space, esc, pause, kp_0, "
        f"enter, F1-F24, or a single character."
    )


def key_names(key, platform: str = sys.platform) -> Set[str]:
    """All canonical names a pynput key event answers to.

    Works on the objects pynput delivers (``Key`` members and ``KeyCode``
    instances), but only reads plain attributes, so tests can pass simple
    stand-ins.
    """
    names: Set[str] = set()
    name = getattr(key, "name", None)            # Key.space -> "space"
    if isinstance(name, str) and name:
        names.add(name.lower())
        if name.lower() == "insert":
            # Numpad-0 with NumLock off arrives as Insert on Windows and X11.
            names.add("kp_0")
    vk = getattr(key, "vk", None)
    char = getattr(key, "char", None)
    if isinstance(char, str) and char:
        names.add(" " if char == " " else char.lower())
        if char == " ":
            names.add("space")
    if _is_numpad_zero(vk, char, platform):
        names.add("kp_0")
    return names


def _is_numpad_zero(vk, char, platform: str) -> bool:
    if vk is None:
        # pynput's X11 backend maps KP_0 to KeyCode.from_char("0") and drops
        # the keysym; the top-row 0 keeps vk=48. That difference is all we get.
        return platform.startswith("linux") and char == "0"
    if platform == "win32":
        return vk == _WIN_VK_NUMPAD0
    if platform == "darwin":
        return vk == _MAC_VK_KEYPAD0
    return vk in _X11_KEYSYMS_KP0


def describe_keys(keys: Iterable[str]) -> str:
    """Human-readable list, e.g. ``"Space, Esc, Numpad-0 or Pause"``."""
    pretty = {"space": "Space", "esc": "Esc", "kp_0": "Numpad-0",
              "pause": "Pause", "enter": "Enter"}
    words = [pretty.get(k, k.upper() if k.startswith("f") and k[1:].isdigit() else k)
             for k in keys]
    if len(words) <= 1:
        return "".join(words)
    return ", ".join(words[:-1]) + " or " + words[-1]


def keyboard_hook_problem(platform: str = sys.platform,
                          environ: Optional[Dict[str, str]] = None) -> Optional[str]:
    """Why a global keyboard hook will not see key presses here, or None.

    Checked before the hook starts, so the caller can fall back to the
    on-screen button right away. macOS permission problems only show once the
    hook runs; :class:`KeyboardHook` reports those.
    """
    env = os.environ if environ is None else environ
    if platform.startswith("linux"):
        session = env.get("XDG_SESSION_TYPE", "").lower()
        if session == "wayland" or (env.get("WAYLAND_DISPLAY") and session != "x11"):
            return ("this desktop runs Wayland, which hides key presses from other "
                    "programs; the keys only work while an X11 window has focus")
        if not env.get("DISPLAY"):
            return "there is no graphical desktop (no DISPLAY), so no keys can be read"
    return None


class KeyboardHook:
    """One process-wide pynput listener, shared by every robot object.

    Creating two ``Robot()`` objects must not install two competing global
    hooks, so subscribers register here with the keys they care about.
    """

    _lock = threading.Lock()
    _listener = None
    _subscribers: Dict[int, Tuple[Callable[[str], None], FrozenSet[str]]] = {}
    #: Problem found when the listener started (None = looks fine).
    problem: Optional[str] = None

    @classmethod
    def subscribe(cls, token: int, callback: Callable[[str], None],
                  keys: Iterable[str]) -> Optional[str]:
        """Start (or join) the listener. Returns a problem description or None.

        ``callback`` receives the canonical name of the key that was pressed.
        It runs on the listener thread and must return quickly.
        """
        try:
            from pynput import keyboard
        except Exception as exc:  # ImportError, or no X display on Linux
            return f"the keyboard library pynput cannot run here ({exc})"

        with cls._lock:
            cls._subscribers[token] = (callback, frozenset(keys))
            if cls._listener is not None:
                return cls.problem

            def on_press(pressed):
                try:
                    names = key_names(pressed)
                except Exception:
                    return
                with cls._lock:
                    subs = list(cls._subscribers.values())
                for cb, wanted in subs:
                    hit = names & wanted
                    if hit:
                        try:
                            cb(sorted(hit)[0])
                        except Exception:
                            logger.exception("Emergency-stop key callback failed")

            try:
                listener = keyboard.Listener(on_press=on_press)
                listener.daemon = True
                listener.start()
            except Exception as exc:
                cls._subscribers.pop(token, None)
                return f"the keyboard hook could not start ({exc})"
            cls._listener = listener

        # Wait (briefly) until the hook runs. pynput's own wait() has no
        # timeout and hangs forever if the backend dies during start-up.
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            if getattr(listener, "_ready", True) or not listener.is_alive():
                break
            time.sleep(0.01)

        problem = None
        if not listener.is_alive():
            problem = "the keyboard hook stopped right after starting"
        elif getattr(listener, "IS_TRUSTED", True) is False:
            problem = ("macOS blocks it: allow your terminal or editor under "
                       "System Settings > Privacy & Security > Input Monitoring "
                       "(and Accessibility), then restart it")
        with cls._lock:
            cls.problem = problem
        return problem

    @classmethod
    def unsubscribe(cls, token: int) -> None:
        """Leave the listener; the last subscriber stops it."""
        with cls._lock:
            cls._subscribers.pop(token, None)
            listener = cls._listener if not cls._subscribers else None
            if listener is not None:
                cls._listener = None
                cls.problem = None
        if listener is not None:
            try:
                listener.stop()
            except Exception:
                pass

    @classmethod
    def is_subscribed(cls, token: int) -> bool:
        with cls._lock:
            return token in cls._subscribers


# ==================== CTRL+C ====================


class SigintGuard:
    """Turn Ctrl+C into an emergency stop *before* the program unwinds.

    Without this, Ctrl+C only ends the Python program: a servo that is still
    travelling to its last target keeps going, because the bricklet runs the
    motion on its own. The guard freezes the motors first, then lets the
    usual ``KeyboardInterrupt`` happen.

    The freeze runs on a helper thread: the main thread may be inside a
    Tinkerforge call that holds the connection lock. The thread is not a
    daemon, so the interpreter waits for the freeze before it exits.
    """

    _lock = threading.Lock()
    _previous = None
    _installed = False
    _callbacks: Dict[int, Callable[[], None]] = {}

    @classmethod
    def add(cls, token: int, callback: Callable[[], None]) -> bool:
        """Register ``callback``. Returns False if no handler could be installed."""
        with cls._lock:
            cls._callbacks[token] = callback
            if cls._installed:
                return True
            if threading.current_thread() is not threading.main_thread():
                return False
            try:
                cls._previous = signal.getsignal(signal.SIGINT)
                signal.signal(signal.SIGINT, cls._handler)
            except (ValueError, OSError, AttributeError):
                return False
            cls._installed = True
            return True

    @classmethod
    def remove(cls, token: int) -> None:
        with cls._lock:
            cls._callbacks.pop(token, None)
            if cls._callbacks or not cls._installed:
                return
            if threading.current_thread() is not threading.main_thread():
                return
            try:
                # == not is: every access to a classmethod makes a new bound method.
                if signal.getsignal(signal.SIGINT) == cls._handler:
                    signal.signal(signal.SIGINT, cls._previous or signal.default_int_handler)
            except (ValueError, OSError):
                pass
            cls._installed = False
            cls._previous = None

    @classmethod
    def _handler(cls, signum, frame):
        with cls._lock:
            callbacks = list(cls._callbacks.values())
            previous = cls._previous
        if callbacks:
            def run():
                for cb in callbacks:
                    try:
                        cb()
                    except Exception:
                        logger.exception("Ctrl+C emergency stop failed")
            threading.Thread(target=run, name="pib3-estop-ctrl-c").start()
        if callable(previous):
            previous(signum, frame)
        elif previous == signal.SIG_IGN:
            return
        else:
            raise KeyboardInterrupt


# ==================== ON-SCREEN BUTTON ====================


def ui_language(environ: Optional[Dict[str, str]] = None,
                platform: str = sys.platform) -> str:
    """``"de"`` or ``"en"`` for the STOP window.

    ``PIB3_LANG`` wins; otherwise the system language (German if it starts
    with ``de`` or names German, as Windows locales do).
    """
    env = os.environ if environ is None else environ
    wanted = env.get("PIB3_LANG", "")
    if wanted:
        return "de" if wanted.lower().startswith("de") else "en"
    names = [env.get(k, "") for k in ("LC_ALL", "LC_MESSAGES", "LANG", "LANGUAGE")]
    try:
        names.append(locale.getlocale()[0] or "")
    except (ValueError, TypeError):
        pass
    if platform == "win32" and environ is None:
        try:
            import ctypes
            # Primary language id 0x07 = German (de-DE, de-AT, de-CH, ...).
            names.append("de" if (ctypes.windll.kernel32.GetUserDefaultUILanguage() & 0x3FF) == 0x07 else "")
        except Exception:
            pass
    for name in names:
        low = (name or "").lower()
        if low.startswith("de") or "german" in low or "deutsch" in low:
            return "de"
    return "en"


class StopButton:
    """A big red STOP button in its own small window.

    It runs in a separate process because Tk must own the main thread (on
    macOS strictly so), and the main thread belongs to the user's program.
    The window stays on top, can be clicked with a touchpad, and also reacts
    to Space, Esc and Enter while it has focus. It closes when the program
    ends.
    """

    def __init__(self, on_stop: Callable[[], None], title: str = "pib3",
                 script: Optional[str] = None,
                 triggers: Sequence[str] = ("click",), effect: str = "robot",
                 lang: Optional[str] = None) -> None:
        """
        Args:
            on_stop: Called (on a reader thread) for every press.
            title: Robot name shown in the window.
            script: The window script; tests pass a stand-in.
            triggers: Ways to stop that the window lists, as tokens:
                ``click``, ``space``, ``space3d`` (Webots), ``esc``, ``kp_0``,
                ``pause``, ``ctrl_c``.
            effect: ``"robot"`` (stops the whole robot) or ``"sim"``.
            lang: ``"de"`` or ``"en"``; None = :func:`ui_language`.
        """
        self._on_stop = on_stop
        self._title = title
        self._triggers = list(triggers)
        self._effect = effect
        self._lang = lang or ui_language()
        # Run the file directly, not ``-m pib3.tools...``: that would import
        # the whole package (numpy, OpenCV, ...) and delay the window by
        # seconds. The script itself imports nothing from pib3.
        self._script = script or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "tools", "stop_button.py")
        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._ready = threading.Event()
        self._failed: Optional[str] = None

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, timeout: float = 8.0, wait: bool = True) -> bool:
        """Open the window.

        Args:
            timeout: How long to wait for the window to appear.
            wait: False returns at once (True if the process started) and
                reports a failure later in the log. Used when the stop keys
                already work, so the first motion is not delayed.

        Returns:
            True once the window is on screen (``wait=True``).
        """
        if self.running:
            return True
        cmd = [sys.executable, self._script, "--title", self._title,
               "--ausloeser", ",".join(self._triggers),
               "--wirkung", self._effect, "--sprache", self._lang]
        kwargs = {}
        if sys.platform == "win32":
            # Keep Ctrl+C in the console for the user's program only.
            kwargs["creationflags"] = getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
        try:
            self._proc = subprocess.Popen(
                cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, text=True, bufsize=1, **kwargs,
            )
        except OSError as exc:
            self._failed = str(exc)
            return False
        threading.Thread(target=self._read_loop, name="pib3-stop-button",
                         daemon=True).start()
        if not wait:
            def report():
                if not self._ready.wait(timeout) or self._failed:
                    logger.warning("Could not open the on-screen STOP button: %s",
                                   self._failed or "the window did not open in time")
            threading.Thread(target=report, name="pib3-stop-button-check",
                             daemon=True).start()
            return True
        if not self._ready.wait(timeout):
            self._failed = self._failed or "the window did not open in time"
            self.close()
            return False
        return self._failed is None

    @property
    def failure(self) -> Optional[str]:
        """Why the window could not be shown, if it could not."""
        return self._failed

    def _read_loop(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return
        for line in proc.stdout:
            word = line.strip()
            if word == "READY":
                self._ready.set()
            elif word == "STOP":
                try:
                    self._on_stop()
                except Exception:
                    logger.exception("STOP button callback failed")
            elif word.startswith("ERROR"):
                self._failed = word[5:].strip() or "the window could not open"
                self._ready.set()
        if not self._ready.is_set():
            err = ""
            try:
                err = proc.stderr.read() or ""
            except Exception:
                pass
            last = err.strip().splitlines()[-1:] or [""]
            reason = last[0] or "the window process ended"
            if "xcb" in err and "sequence number" in err:
                # Old python-build-standalone builds (e.g. uv's CPython
                # 3.13.4/3.13.5) link Tk 8.6 statically in a way that aborts
                # on the first widget. Fixed in later builds (Tk 9).
                reason = ("this Python build's Tk crashes on Linux (old uv/"
                          "python-build-standalone build). Fix: `uv python "
                          "upgrade` (or install a current Python), then "
                          "recreate the venv")
            self._failed = self._failed or reason
            self._ready.set()

    def _send(self, line: str) -> None:
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None or proc.stdin is None:
                return
            try:
                proc.stdin.write(line + "\n")
                proc.stdin.flush()
            except (OSError, ValueError):
                pass

    def notify_stopped(self, reason: str = "") -> None:
        self._send(f"STOPPED {reason}".strip())

    def notify_resumed(self) -> None:
        self._send("RESUMED")

    def close(self) -> None:
        with self._lock:
            proc, self._proc = self._proc, None
        if proc is None:
            return
        try:
            if proc.stdin is not None:
                proc.stdin.close()      # the window quits on EOF
            proc.wait(timeout=2.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


# ==================== REMOTE STOP (teacher) ====================

#: rosbridge topic every pib3 robot connection listens on. A message with
#: ``{"action": "stop"}`` latches the emergency stop in that program.
ESTOP_TOPIC = "/pib3/emergency_stop"
ESTOP_TOPIC_TYPE = "std_msgs/msg/String"

#: Tinkerforge device identifier of the Servo Bricklet 2.0.
SERVO_V2_DEVICE_ID = 2157
SERVO_V2_CHANNELS = 10


def parse_estop_message(message: dict) -> Optional[str]:
    """Return the source of a stop request, or None if it is not one."""
    raw = message.get("data") if isinstance(message, dict) else None
    try:
        data = json.loads(raw) if isinstance(raw, str) and raw else {}
    except ValueError:
        data = {}
    if not isinstance(data, dict):
        data = {}
    if str(data.get("action", "stop")).lower() != "stop":
        return None
    return str(data.get("source") or "remote")


def freeze_servo(servo, channel: int, relax: bool = False) -> bool:
    """Stop one Servo Bricklet 2.0 channel where it is.

    The bricklet ramps every move itself. Setting the target to the current
    ramp position ends the move. Deceleration goes to 0 first, otherwise the
    bricklet would brake along its normal ramp: at 150 deg/s and
    150 deg/s^2 that means 75 deg of overshoot and a move back. With
    ``relax=True`` the PWM is switched off instead and the joint goes limp.
    Arms then fall, so this is the last resort.

    Returns:
        True if the channel was stopped, False if it was disabled anyway.
    """
    if relax:
        servo.set_enable(channel, False)
        return True
    try:
        if not servo.get_enabled(channel):
            return False
    except Exception:
        pass
    try:
        cfg = servo.get_motion_configuration(channel)
        velocity = getattr(cfg, "velocity", None)
        if velocity is None and isinstance(cfg, (tuple, list)):
            velocity = cfg[0]
        acceleration = getattr(cfg, "acceleration", None)
        if acceleration is None and isinstance(cfg, (tuple, list)):
            acceleration = cfg[1]
        servo.set_motion_configuration(channel, int(velocity or 0), int(acceleration or 0), 0)
    except Exception:
        pass
    servo.set_position(channel, servo.get_current_position(channel))
    return True


def freeze_robot_servos(host: str, port: int = 4223, relax: bool = False,
                        discovery_time: float = 1.0) -> int:
    """Freeze every servo of the robot at ``host``, from any computer.

    Talks to the robot's Tinkerforge daemon directly and needs neither ROS
    nor the student's program.

    Returns:
        Number of channels stopped.
    """
    from tinkerforge.ip_connection import IPConnection
    from tinkerforge.bricklet_servo_v2 import BrickletServoV2

    ipcon = IPConnection()
    ipcon.connect(host, port)
    uids: List[str] = []

    def on_enumerate(uid, connected_uid, position, hw, fw, device_identifier, enumeration_type):
        if device_identifier == SERVO_V2_DEVICE_ID and uid not in uids:
            uids.append(uid)

    try:
        ipcon.register_callback(IPConnection.CALLBACK_ENUMERATE, on_enumerate)
        ipcon.enumerate()
        time.sleep(discovery_time)
        stopped = 0
        for uid in list(uids):
            servo = BrickletServoV2(uid, ipcon)
            for channel in range(SERVO_V2_CHANNELS):
                try:
                    if freeze_servo(servo, channel, relax=relax):
                        stopped += 1
                except Exception as exc:
                    logger.debug("Could not stop %s channel %d: %s", uid, channel, exc)
        return stopped
    finally:
        try:
            ipcon.disconnect()
        except Exception:
            pass


def broadcast_stop(host: str, port: int = 9090, source: str = "teacher",
                   timeout: float = 3.0) -> bool:
    """Latch the stop in every pib3 program connected to the robot at ``host``."""
    import roslibpy

    client = roslibpy.Ros(host=host, port=port)
    try:
        client.run(timeout=timeout)
        topic = roslibpy.Topic(client, ESTOP_TOPIC, ESTOP_TOPIC_TYPE)
        topic.advertise()
        payload = json.dumps({"action": "stop", "source": source, "time": time.time()})
        # A few repeats: a subscriber that connected a moment ago may not have
        # its subscription registered at rosbridge yet.
        for _ in range(3):
            topic.publish(roslibpy.Message({"data": payload}))
            time.sleep(0.15)
        topic.unadvertise()
        return True
    finally:
        try:
            client.terminate()
        except Exception:
            pass


def coerce_keys(keys: Union[None, bool, str, Sequence[str]]) -> Tuple[str, ...]:
    """Normalize the ``keys`` argument of ``enable_estop_key``."""
    if keys is None or keys is True:
        return DEFAULT_STOP_KEYS
    if keys is False:
        return ()
    if isinstance(keys, str):
        keys = [keys]
    out: List[str] = []
    for k in keys:
        n = normalize_key_name(k)
        if n not in out:
            out.append(n)
    return tuple(out)
