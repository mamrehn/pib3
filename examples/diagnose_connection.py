
import sys
import time
import socket
import logging

try:
    import roslibpy
except ImportError:
    print("Error: roslibpy is not installed. Please install it with 'pip install roslibpy'")
    sys.exit(1)

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

HOST = "172.26.34.149"
PORT = 9090

def check_ping(host):
    import subprocess
    print(f"Checking ping to {host}...")
    try:
        output = subprocess.check_output(
            ["ping", "-c", "1", "-W", "2", host],
            stderr=subprocess.STDOUT,
            text=True
        )
        print(f"  [OK] Ping successful:\n{output}")
        return True
    except subprocess.CalledProcessError as e:
        print(f"  [FAIL] Ping failed:\n{e.output}")
        return False

def check_port(host, port):
    print(f"Checking TCP connection to {host}:{port}...")
    try:
        sock = socket.create_connection((host, port), timeout=2)
        sock.close()
        print(f"  [OK] TCP connection successful")
        return True
    except Exception as e:
        print(f"  [FAIL] TCP connection failed: {e}")
        return False

def check_rosbridge(host, port):
    print(f"Checking rosbridge connection to {host}:{port}...")
    
    ros = roslibpy.Ros(host=host, port=port)
    try:
        ros.run(timeout=5)
        if ros.is_connected:
            print("  [OK] Rosbridge connected successfully")
            ros.close()
            return True
        else:
            print("  [FAIL] Rosbridge run() returned but not connected")
            return False
    except Exception as e:
        print(f"  [FAIL] Rosbridge connection error: {e}")
        return False

def main():
    print("=== Robot Connection Diagnostic Tool ===\n")
    
    ping_ok = check_ping(HOST)
    port_ok = check_port(HOST, PORT)
    ros_ok = False
    
    if port_ok:
        ros_ok = check_rosbridge(HOST, PORT)
    
    print("\n=== Summary ===")
    print(f"Network (Ping):   {'✅ PASS' if ping_ok else '❌ FAIL'}")
    print(f"Port 9090 (TCP):  {'✅ PASS' if port_ok else '❌ FAIL'}")
    print(f"Rosbridge Proto:  {'✅ PASS' if ros_ok else '❌ FAIL'}")
    
    if ping_ok and port_ok and ros_ok:
        print("\nSUCCESS: Connection to robot looks good!")
    else:
        print("\nFAILURE: Some checks failed. See details above.")

if __name__ == "__main__":
    main()
