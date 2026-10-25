#!/usr/bin/env python3
"""
Airmouse Host Receiver for macOS.
Receives 5-byte motion/button packets from the STM32F103 over HC-05 Bluetooth SPP.

Note for macOS users:
  1. Serial Port: Use the '/dev/cu.*' device (e.g., /dev/cu.HC-05 or /dev/cu.HC-05-DevB).
     The 'cu' (Calling Unit) interface opens immediately without waiting for DCD
     carrier detect, which Bluetooth SPP does not assert.
  2. Accessibility Permissions:
     The Terminal, iTerm2, or IDE (e.g. VS Code) running this script MUST be granted
     Accessibility permission in:
     System Settings -> Privacy & Security -> Accessibility
     Otherwise, PyAutoGUI will raise an error or fail to move the cursor / click buttons.
"""

import argparse
import glob
import sys
import time

try:
    import serial
except ImportError:
    print("Error: 'pyserial' is not installed. Run: pip install -r host/requirements.txt", file=sys.stderr)
    sys.exit(1)

try:
    import pyautogui
except ImportError:
    print("Error: 'pyautogui' is not installed. Run: pip install -r host/requirements.txt", file=sys.stderr)
    sys.exit(1)

# Configure PyAutoGUI for zero-latency real-time mouse control
pyautogui.PAUSE = 0
pyautogui.MINIMUM_DURATION = 0
pyautogui.FAILSAFE = True  # Slam cursor into any screen corner to emergency-abort


def find_default_port() -> str | None:
    """Auto-detect the first matching /dev/cu.HC-05* device."""
    patterns = [
        "/dev/cu.HC-05*",
        "/dev/cu.*HC-05*",
        "/dev/cu.*hc-05*",
        "/dev/cu.*HC_05*",
    ]
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if matches:
            return matches[0]
    return None


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Airmouse Bluetooth Host Receiver for macOS (HC-05)"
    )
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Serial port (default: auto-find first /dev/cu.HC-05*)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=9600,
        help="Serial baud rate (default: 9600)",
    )
    parser.add_argument(
        "--scale",
        type=float,
        default=1.0,
        help="Mouse movement sensitivity scale factor (default: 1.0)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print decoded packets without moving cursor or clicking buttons",
    )
    return parser.parse_args()


def to_int8(val: int) -> int:
    """Convert unsigned 8-bit integer to signed int8 (-128 to 127)."""
    return val - 256 if val > 127 else val


def main() -> None:
    args = parse_arguments()

    port = args.port
    if not port:
        port = find_default_port()

    if not port:
        print(
            "Error: No HC-05 device found matching '/dev/cu.HC-05*'.\n"
            "Please ensure the HC-05 is paired in macOS System Settings -> Bluetooth,\n"
            "or specify the port explicitly with: --port /dev/cu.<device-name>\n"
            "\nAvailable /dev/cu.* serial ports:",
            file=sys.stderr,
        )
        for p in sorted(glob.glob("/dev/cu.*")):
            print(f"  {p}", file=sys.stderr)
        sys.exit(1)

    print(f"Connecting to HC-05 on {port} at {args.baud} baud...")
    if args.dry_run:
        print("[DRY-RUN MODE] Packets will be decoded and printed, mouse will NOT move.")
    else:
        print(f"[LIVE MODE] Controlling mouse cursor (scale={args.scale:.2f}).")
        print("Note: FailSafe is ON. Move mouse to any screen corner to abort.")

    try:
        ser = serial.Serial(port=port, baudrate=args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(
            f"\nError: Could not open serial port '{port}'.\n"
            f"Details: {e}\n\n"
            "Troubleshooting tips:\n"
            "  1. Ensure the HC-05 module is powered on and paired with macOS.\n"
            "  2. Verify that no other application or terminal is using this port.\n"
            "  3. On macOS, ensure you are using the '/dev/cu.*' device rather than '/dev/tty.*'.\n",
            file=sys.stderr,
        )
        sys.exit(1)

    # State tracking
    left_down = False
    right_down = False
    last_packet_time = time.time()
    timeout_warned = False
    buffer = bytearray()

    print("Connected. Listening for airmouse packets (Ctrl+C to quit)...")

    try:
        while True:
            # Read incoming bytes with short non-blocking timeout
            available = ser.in_waiting
            chunk = ser.read(available if available > 0 else 1)
            if chunk:
                buffer.extend(chunk)

            now = time.time()

            # Timeout check: If no valid packet arrives for 1 second, release buttons and warn once
            if (now - last_packet_time) >= 1.0:
                if left_down or right_down:
                    if not args.dry_run:
                        if left_down:
                            pyautogui.mouseUp(button="left")
                        if right_down:
                            pyautogui.mouseUp(button="right")
                    left_down = False
                    right_down = False
                if not timeout_warned:
                    print("Warning: No valid packet received for 1 second. Released buttons.")
                    timeout_warned = True

            # Resync & parse 5-byte packets:
            # byte 0: 0xAA (header sync)
            # byte 1: dx (int8)
            # byte 2: dy (int8)
            # byte 3: buttons (bit0=left, bit1=right)
            # byte 4: XOR of bytes 0..3
            while len(buffer) >= 5:
                # Seek sync byte 0xAA
                if buffer[0] != 0xAA:
                    buffer.pop(0)
                    continue

                # Verify XOR checksum over bytes 0..3
                expected_csum = buffer[0] ^ buffer[1] ^ buffer[2] ^ buffer[3]
                actual_csum = buffer[4]

                if actual_csum == expected_csum:
                    # Valid packet found! Consume all 5 bytes
                    raw_dx = buffer[1]
                    raw_dy = buffer[2]
                    buttons = buffer[3]
                    del buffer[:5]

                    dx = to_int8(raw_dx)
                    dy = to_int8(raw_dy)

                    last_packet_time = time.time()
                    timeout_warned = False

                    left_pressed = bool(buttons & 0x01)
                    right_pressed = bool(buttons & 0x02)

                    if args.dry_run:
                        print(
                            f"[PACKET] dx: {dx:+4d} | dy: {dy:+4d} | "
                            f"Left: {'DOWN' if left_pressed else ' UP '} | "
                            f"Right: {'DOWN' if right_pressed else ' UP '}"
                        )
                    else:
                        # Move cursor only when dx or dy is non-zero
                        if dx != 0 or dy != 0:
                            pyautogui.moveRel(dx * args.scale, dy * args.scale)

                        # Left button edge detection
                        if left_pressed and not left_down:
                            pyautogui.mouseDown(button="left")
                            left_down = True
                        elif not left_pressed and left_down:
                            pyautogui.mouseUp(button="left")
                            left_down = False

                        # Right button edge detection
                        if right_pressed and not right_down:
                            pyautogui.mouseDown(button="right")
                            right_down = True
                        elif not right_pressed and right_down:
                            pyautogui.mouseUp(button="right")
                            right_down = False
                else:
                    # Checksum mismatch: the 0xAA might legitimately be data (e.g., dx = 0xAA)
                    # Discard this byte and resume search from next byte
                    buffer.pop(0)

    except pyautogui.FailSafeException:
        print("\nPyAutoGUI FailSafe triggered (cursor moved to screen corner). Aborting.")
    except KeyboardInterrupt:
        print("\nStopping airmouse host receiver...")
    finally:
        # Guarantee buttons are released upon exit
        if left_down:
            try:
                pyautogui.mouseUp(button="left")
            except Exception:
                pass
        if right_down:
            try:
                pyautogui.mouseUp(button="right")
            except Exception:
                pass
        try:
            ser.close()
        except Exception:
            pass
        print("Clean shutdown complete.")


if __name__ == "__main__":
    main()
