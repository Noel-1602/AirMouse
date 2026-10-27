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
        "--smooth",
        type=float,
        default=0.0,
        help="Exponential moving average smoothing factor (0.0 to 0.9, default: 0.0 = off)",
    )
    parser.add_argument(
        "--scroll-scale",
        type=float,
        default=1.0,
        help="Scroll wheel sensitivity scale factor (default: 1.0)",
    )
    parser.add_argument(
        "--scroll-invert",
        action="store_true",
        help="Invert scroll wheel direction",
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

    if not (0.0 <= args.smooth <= 0.9):
        print(f"Error: --smooth must be between 0.0 and 0.9 (got {args.smooth})", file=sys.stderr)
        sys.exit(1)

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
    scroll_info = f", scroll_scale={args.scroll_scale:.2f}" + (", scroll_invert=True" if args.scroll_invert else "")
    if args.dry_run:
        smooth_info = f", smooth={args.smooth:.2f}" if args.smooth > 0 else ""
        print(f"[DRY-RUN MODE] Packets will be decoded and printed (scale={args.scale:.2f}{smooth_info}{scroll_info}), mouse will NOT move.")
    else:
        smooth_info = f", smooth={args.smooth:.2f}" if args.smooth > 0 else " (smoothing off)"
        print(f"[LIVE MODE] Controlling mouse cursor (scale={args.scale:.2f}{smooth_info}{scroll_info}).")
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

    # Host smoothing & fractional remainder state
    s_prev_dx = 0.0
    s_prev_dy = 0.0
    rem_x = 0.0
    rem_y = 0.0
    zero_packet_count = 0

    print("Connected. Listening for airmouse packets (Ctrl+C to quit)...")

    try:
        while True:
            # Read incoming bytes with short non-blocking timeout
            available = ser.in_waiting
            chunk = ser.read(available if available > 0 else 1)
            if chunk:
                buffer.extend(chunk)

            now = time.time()

            # Timeout check: If no valid packet arrives for 1 second, release buttons, reset smoothing state, and warn once
            if (now - last_packet_time) >= 1.0:
                s_prev_dx = 0.0
                s_prev_dy = 0.0
                rem_x = 0.0
                rem_y = 0.0
                zero_packet_count = 0

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
            # byte 3: buttons (bit0=left, bit1=right, bit2=scroll)
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
                    is_scroll = bool(buttons & 0x04)

                    if is_scroll:
                        # Reset cursor smoothing state when scroll is active so cursor doesn't jump afterwards
                        s_prev_dx = 0.0
                        s_prev_dy = 0.0
                        rem_x = 0.0
                        rem_y = 0.0
                        zero_packet_count = 0

                        scroll_factor = 1.0 if args.scroll_invert else -1.0
                        scroll_ticks = dy
                        scroll_amount = int(round(scroll_ticks * args.scroll_scale * scroll_factor))

                        if args.dry_run:
                            print(
                                f"[SCROLL] scroll: {scroll_ticks:+4d} ticks | "
                                f"Left: {'DOWN' if left_pressed else ' UP '} | "
                                f"Right: {'DOWN' if right_pressed else ' UP '}"
                            )
                        else:
                            if scroll_amount != 0:
                                pyautogui.scroll(scroll_amount)
                    else:
                        # Reset smoothing state when dx = dy = 0 for 5 packets in a row
                        if dx == 0 and dy == 0:
                            zero_packet_count += 1
                            if zero_packet_count >= 5:
                                s_prev_dx = 0.0
                                s_prev_dy = 0.0
                                rem_x = 0.0
                                rem_y = 0.0
                        else:
                            zero_packet_count = 0

                        # Compute smoothed dx and dy
                        if args.smooth > 0:
                            s_dx = args.smooth * s_prev_dx + (1.0 - args.smooth) * float(dx)
                            s_dy = args.smooth * s_prev_dy + (1.0 - args.smooth) * float(dy)
                            s_prev_dx = s_dx
                            s_prev_dy = s_dy
                        else:
                            s_dx = float(dx)
                            s_dy = float(dy)
                            s_prev_dx = s_dx
                            s_prev_dy = s_dy

                        if args.dry_run:
                            print(
                                f"[PACKET] dx: {dx:+4d} | dy: {dy:+4d} | "
                                f"s_dx: {s_dx:+6.2f} | s_dy: {s_dy:+6.2f} | "
                                f"Left: {'DOWN' if left_pressed else ' UP '} | "
                                f"Right: {'DOWN' if right_pressed else ' UP '}"
                            )
                        else:
                            if args.smooth > 0:
                                # Apply scale and accumulate fractional remainders
                                total_x = s_dx * args.scale + rem_x
                                total_y = s_dy * args.scale + rem_y

                                step_x = int(total_x)
                                step_y = int(total_y)

                                rem_x = total_x - float(step_x)
                                rem_y = total_y - float(step_y)

                                if step_x != 0 or step_y != 0:
                                    pyautogui.moveRel(step_x, step_y)
                            else:
                                # With --smooth 0, behaviour is identical to original firmware
                                if dx != 0 or dy != 0:
                                    pyautogui.moveRel(dx * args.scale, dy * args.scale)

                    # Button handling works in both normal and scroll modes
                    if not args.dry_run:
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
