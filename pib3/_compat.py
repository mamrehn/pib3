"""Compatibility shims for running pib3 on numpy >= 2.0.

roboticstoolbox-python / spatialmath-python (older releases) reference a
handful of numpy attributes that numpy 2.0 removed. We restore the ones they
use *before* importing those libraries so pib3 works on numpy 1.x and 2.x
alike. Modern rtb/spatialmath releases already support numpy 2; this shim is
defensive cover for environments pinned to older versions.

Call :func:`ensure_numpy2_compat` once before importing roboticstoolbox or
spatialmath (it is idempotent and a no-op on numpy 1.x for already-present
attributes).
"""

import numpy as np

# numpy 2.0 removed these scalar-type aliases; map each to its replacement.
_REMOVED_ALIASES = (
    ("float_", "float64"),
    ("unicode_", "str_"),
    ("string_", "bytes_"),
    ("NaN", "nan"),
    ("Inf", "inf"),
)


def ensure_numpy2_compat() -> None:
    """Restore numpy attributes removed in 2.0 that rtb/spatialmath still use.

    Idempotent and safe on numpy 1.x — only fills in attributes that are
    actually missing.
    """
    if not hasattr(np, "disp"):
        # numpy < 2 exposed np.disp; roboticstoolbox calls it in a few paths.
        np.disp = lambda x: print(x)

    for removed, replacement in _REMOVED_ALIASES:
        if not hasattr(np, removed) and hasattr(np, replacement):
            setattr(np, removed, getattr(np, replacement))
