"""Actionable runtime hints for mistakes that otherwise fail silently.

Every problem covered here has the same shape: the program runs, nothing
raises, and the robot simply does the wrong thing. A novice cannot tell such a
bug from "my logic is wrong", so they debug the wrong half of the program —
and in a classroom that turns into one teacher answering the same question
thirty times.

The rule for anything added here:

1. **Detect the symptom, not the intent.** Never guess what the user meant;
   only report a state that is objectively wrong (e.g. fifty reads without a
   single simulation step).
2. **Name the fix, not the problem.** "Add ``latest_only=True``" beats
   "results may be stale".
3. **Fire once.** A hint repeated every frame is noise, and noise is what
   hides the next real message.
4. **Never change behaviour.** Hints only print. Code that ignores them keeps
   working exactly as before.
"""

import logging

logger = logging.getLogger(__name__)

_seen = set()

#: Width of the separator lines. Wide enough to stand out in the Webots
#: console, which interleaves messages from every robot in the world.
_RULE = "-" * 70


def hint(key: str, message: str) -> None:
    """Print an actionable hint once per process.

    Args:
        key: Stable identifier for this hint; repeats are suppressed.
        message: What is wrong and — more importantly — what to change.
            Written for someone who has never seen this API before.
    """
    if key in _seen:
        return
    _seen.add(key)
    logger.warning("\n%s\npib3 hint: %s\n%s", _RULE, message, _RULE)


def reset_hints() -> None:
    """Forget which hints have fired. Mainly for tests."""
    _seen.clear()


def already_hinted(key: str) -> bool:
    """Whether ``key`` has fired already. Mainly for tests."""
    return key in _seen
