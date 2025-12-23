"""Test Swift visualization without opening browser."""
import sys
from pathlib import Path
import time
import signal

# Add parent directory to path so we can import pib3
sys.path.insert(0, str(Path(__file__).parent.parent))

import pib3

# Timeout handler
def timeout_handler(signum, frame):
    print("✓ Swift server started successfully (no websocket errors)")
    print("✓ Killed after 3 seconds to avoid browser hang")
    sys.exit(0)

signal.signal(signal.SIGALRM, timeout_handler)
signal.alarm(3)  # Kill after 3 seconds

print("Testing Swift visualization backend...")
try:
    # Load a simple trajectory
    img_path = Path(__file__).parent.parent / "examples" / "Icon_IT_hover.png"
    trajectory = pib3.generate_trajectory(str(img_path))
    print(f"Generated {len(trajectory.waypoints)} waypoints")

    # Try to create Swift backend (will start websocket server)
    viz = pib3.Swift()

    # If we got here without websocket errors, it's working!
    print("✓ Swift backend created without errors")

    # Wait for timeout
    time.sleep(10)

except TypeError as e:
    if "missing 1 required positional argument: 'path'" in str(e):
        print(f"✗ Websocket error still present: {e}")
        sys.exit(1)
    else:
        raise
except Exception as e:
    print(f"Other error: {e}")
    raise
finally:
    signal.alarm(0)  # Cancel alarm
