#!/usr/bin/env python3
"""Interactive test script for 4-DOF robot arm servos on Arduino + PCA9685.

Usage:
    python3 scripts/test_arm_servos.py [--port /dev/ttyACM0] [--baud 115200]
"""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("Error: pyserial is required. Run: python3 -m pip install pyserial")
    sys.exit(1)


def send_cmd(ser: serial.Serial, cmd: str, timeout: float = 3.0) -> str:
    """Send one ASCII command and wait for response."""
    frame = (cmd.strip() + "\n").encode("ascii")
    ser.write(frame)
    ser.flush()
    print(f"  -> Sent: {cmd.strip()}")

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line:
            print(f"  <- Arduino: {line}")
            return line
    print("  !! Timeout waiting for reply")
    return ""


def find_serial_port() -> str:
    """Auto-detect connected Arduino Uno serial port."""
    import glob
    by_id = glob.glob("/dev/serial/by-id/*Arduino*")
    if by_id:
        return by_id[0]
    acm = sorted(glob.glob("/dev/ttyACM*"))
    if acm:
        return acm[-1]
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    if usb:
        return usb[-1]
    return "/dev/ttyACM0"


def main() -> None:
    parser = argparse.ArgumentParser(description="Test 4-DOF servos via PCA9685.")
    parser.add_argument("--port", default="", help="Serial port (auto-detected if omitted)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    port = args.port or find_serial_port()
    print(f"Connecting to Arduino on {port} at {args.baud} baud...")
    try:
        ser = serial.Serial(port, args.baud, timeout=1.0)
    except Exception as exc:
        print(f"\nFailed to open {port}: {exc}")
        print(f"\n>>> ACTION REQUIRED: Run this command in terminal to unlock port:")
        print(f"    sudo chmod 666 {port}")
        return

    # Wait for Arduino reset and READY signal
    print("Waiting for Arduino boot...")
    time.sleep(2.0)
    # Drain any startup messages
    while ser.in_waiting:
        line = ser.readline().decode("ascii", errors="replace").strip()
        print(f"  [boot] {line}")

    print("\n" + "=" * 50)
    print(" 4-DOF ARM SERVO TESTER (PCA9685)")
    print("=" * 50)
    print("Commands:")
    print("  h  - Home all servos (90, 90, 90)")
    print("  o  - Open gripper (Ch 3)")
    print("  c  - Close gripper (Ch 3)")
    print("  0  - Test Base servo (Ch 0) sweep")
    print("  1  - Test Left Shoulder (Ch 1) sweep")
    print("  2  - Test Right Elbow (Ch 2) sweep")
    print("  j  - Custom joint angles: b,s,e (e.g., 90,100,80)")
    print("  a  - Automatic test sequence (tests all 4 servos)")
    print("  q  - Quit")
    print("=" * 50)

    while True:
        try:
            choice = input("\nEnter command [h/o/c/0/1/2/j/a/q]: ").strip().lower()
        except (KeyboardInterrupt, EOFError):
            break

        if choice == "q":
            break
        elif choice == "h":
            send_cmd(ser, "H")
        elif choice == "o":
            send_cmd(ser, "O")
        elif choice == "c":
            send_cmd(ser, "C")
        elif choice == "0":
            print("Sweeping Base (Channel 0): 70 -> 110 -> 90...")
            send_cmd(ser, "J,70,90,90")
            time.sleep(0.5)
            send_cmd(ser, "J,110,90,90")
            time.sleep(0.5)
            send_cmd(ser, "J,90,90,90")
        elif choice == "1":
            print("Sweeping Left Shoulder (Channel 1): 80 -> 100 -> 90...")
            send_cmd(ser, "J,90,80,90")
            time.sleep(0.5)
            send_cmd(ser, "J,90,100,90")
            time.sleep(0.5)
            send_cmd(ser, "J,90,90,90")
        elif choice == "2":
            print("Sweeping Right Elbow (Channel 2): 80 -> 100 -> 90...")
            send_cmd(ser, "J,90,90,80")
            time.sleep(0.5)
            send_cmd(ser, "J,90,90,100")
            time.sleep(0.5)
            send_cmd(ser, "J,90,90,90")
        elif choice == "j":
            angles = input("Enter base,shoulder,elbow (deg, 0-180): ").strip()
            send_cmd(ser, f"J,{angles}")
        elif choice == "a":
            print("\n--- Running Full Automated Test Sequence ---")
            print("1. Homing...")
            send_cmd(ser, "H")
            time.sleep(1.0)
            print("2. Testing Gripper Open/Close...")
            send_cmd(ser, "O")
            time.sleep(0.8)
            send_cmd(ser, "C")
            time.sleep(0.8)
            send_cmd(ser, "O")
            time.sleep(0.8)
            print("3. Testing Base Rotation...")
            send_cmd(ser, "J,70,90,90")
            time.sleep(0.6)
            send_cmd(ser, "J,110,90,90")
            time.sleep(0.6)
            send_cmd(ser, "J,90,90,90")
            time.sleep(0.6)
            print("4. Testing Shoulder Reach...")
            send_cmd(ser, "J,90,75,90")
            time.sleep(0.6)
            send_cmd(ser, "J,90,105,90")
            time.sleep(0.6)
            send_cmd(ser, "J,90,90,90")
            time.sleep(0.6)
            print("5. Testing Elbow...")
            send_cmd(ser, "J,90,90,75")
            time.sleep(0.6)
            send_cmd(ser, "J,90,90,105")
            time.sleep(0.6)
            print("6. Returning Home...")
            send_cmd(ser, "H")
            print("--- Automated Test Complete ---")
        else:
            print("Unknown command.")

    ser.close()
    print("Closed connection.")


if __name__ == "__main__":
    main()
