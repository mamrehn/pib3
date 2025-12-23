"""Test Swift server to diagnose frontend issues."""
import sys
from pathlib import Path
import time
import threading

# Add parent directory to path so we can import pib3
sys.path.insert(0, str(Path(__file__).parent.parent))

import pib3

# Load a simple trajectory
img_path = Path(__file__).parent.parent / "examples" / "Icon_IT_hover.png"
print(f"Generating trajectory from: {img_path}")
trajectory = pib3.generate_trajectory(str(img_path))
print(f"✓ Generated {len(trajectory.waypoints)} waypoints")

# Create Swift backend but don't run trajectory yet
print("\n" + "="*60)
print("Testing Swift visualization server...")
print("="*60)

viz = pib3.Swift()

# Save trajectory for manual testing
output_path = Path(__file__).parent / "output.json"
trajectory.to_json(str(output_path))
print(f"\n✓ Trajectory saved to: {output_path}")

print("\nSwift server should be starting...")
print("Check the browser window that opens.")
print("Press Ctrl+C to stop.")
print("\nIf you see 'Application error', check the browser console (F12)")
print("Look for:")
print("  - WebSocket connection errors")
print("  - 404 errors for JavaScript files")
print("  - CORS errors")

try:
    # This will open the browser
    viz.connect()

    # Keep server alive
    print("\n✓ Swift server started successfully!")
    print("Browse to the URL shown above to test the frontend.")
    print("\nServer is running. Press Ctrl+C to stop...")

    time.sleep(300)  # Keep alive for 5 minutes

except KeyboardInterrupt:
    print("\n\nShutting down...")
    viz.disconnect()
    print("✓ Swift server stopped")
except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
