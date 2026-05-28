"""
StepperLightController — Python interface for the Stepper+NeoPixel Controller.

Usage:
    ctrl = StepperLightController("/dev/ttyACM0")
    ctrl.connect(home=True)    # Triggers homing, waits for completion
    ctrl.position = 128        # 0-255 mapped position
    ctrl.velocity = 0          # 0=position mode, ±1..127=velocity mode
    ctrl.leds = [127]*24       # White channel values
    time.sleep(5)
    ctrl.disconnect()

Features:
    - Automatic reconnection if the MCU resets or USB disconnects
    - Re-homes automatically after reconnection
    - Thread-safe property access for position, velocity, LEDs
    - ~50Hz command send rate and feedback display
"""

import serial
import serial.tools.list_ports
import struct
import threading
import time
import sys
import os

NUM_LEDS = 24
CMD_START = 0xFF
FB_START = 0xFE
CMD_PACKET_SIZE = 29  # 1 start + 1 pos + 1 vel + 1 flags + 24 LEDs + 1 checksum
FB_PACKET_SIZE = 6
CMD_SEND_HZ = 50

# Command flags
FLAG_HOMING = 0x01

# Reconnection settings
RECONNECT_POLL_INTERVAL = 1.0   # Seconds between port availability checks
RECONNECT_SETTLE_TIME = 3.0     # Seconds to wait after port reappears
MAX_RECONNECT_ATTEMPTS = 0      # 0 = unlimited


class StepperLightController:
    def __init__(self, port: str, baud: int = 115200, auto_reconnect: bool = True):
        self.port = port
        self.baud = baud
        self.auto_reconnect = auto_reconnect
        self.ser: serial.Serial | None = None

        # Command state (set these from your application)
        self._lock = threading.Lock()
        self._position: int = 0
        self._velocity: int = 0
        self._leds: list[int] = [0] * NUM_LEDS

        # Feedback state (read-only from outside)
        self._fb_lock = threading.Lock()
        self._fb_pos_mapped: int = 0
        self._fb_pos_raw: int = 0
        self._fb_velocity: int = 0
        self._total_steps: int = 0

        # Connection state
        self._running = False
        self._connected = threading.Event()
        self._reconnecting = threading.Lock()  # Prevents concurrent reconnect attempts
        self._shutdown = False  # True only on explicit disconnect()

        # Threads
        self._cmd_thread: threading.Thread | None = None
        self._read_thread: threading.Thread | None = None
        self._reconnect_thread: threading.Thread | None = None

    # ─── Properties ──────────────────────────────────────────────────────

    @property
    def position(self) -> int:
        with self._lock:
            return self._position

    @position.setter
    def position(self, val: int):
        with self._lock:
            self._position = max(0, min(255, int(val)))

    @property
    def velocity(self) -> int:
        with self._lock:
            return self._velocity

    @velocity.setter
    def velocity(self, val: int):
        with self._lock:
            self._velocity = max(-128, min(127, int(val)))

    @property
    def leds(self) -> list[int]:
        with self._lock:
            return list(self._leds)

    @leds.setter
    def leds(self, vals: list[int]):
        assert len(
            vals) == NUM_LEDS, f"Expected {NUM_LEDS} LED values, got {len(vals)}"
        with self._lock:
            self._leds = [max(0, min(255, int(v))) for v in vals]

    @property
    def feedback_position(self) -> int:
        with self._fb_lock:
            return self._fb_pos_mapped

    @property
    def feedback_position_raw(self) -> int:
        with self._fb_lock:
            return self._fb_pos_raw

    @property
    def feedback_velocity(self) -> int:
        with self._fb_lock:
            return self._fb_velocity

    @property
    def total_steps(self) -> int:
        return self._total_steps

    @property
    def is_connected(self) -> bool:
        return self._connected.is_set()

    # ─── Connection Lifecycle ────────────────────────────────────────────

    def connect(self, home: bool = True, homing_timeout: float = 60.0):
        """
        Open serial and start command/feedback threads.

        Args:
            home: If True, send homing command and wait for completion.
            homing_timeout: Max seconds to wait for homing to complete.
        """
        self._shutdown = False
        self._open_serial()

        if home:
            self._run_homing(homing_timeout)
        else:
            print("Skipping homing — assuming MCU is already homed.")
            time.sleep(0.1)
            self.ser.reset_input_buffer()

        self._start_threads()
        print("Controller active. Command & feedback threads running.\n")

    def disconnect(self):
        """Stop all threads and close serial. Disables auto-reconnect."""
        print("\nDisconnecting...")
        self._shutdown = True
        self._running = False
        self._connected.clear()

        self._join_threads()

        # Send stop command
        self._safe_write(self._build_command(0, 0, [0] * NUM_LEDS))
        time.sleep(0.1)
        self._close_serial()

        print("Disconnected.")

    # ─── Internal: Serial Port Management ────────────────────────────────

    def _open_serial(self):
        """Open serial port with retries."""
        print(f"Opening {self.port} @ {self.baud}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
        time.sleep(2)  # USB settle
        self.ser.reset_input_buffer()
        self._connected.set()

    def _close_serial(self):
        """Safely close serial port."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _safe_write(self, data: bytes) -> bool:
        """Write to serial, return False on failure."""
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(data)
                return True
        except (serial.SerialException, OSError):
            pass
        return False

    def _safe_read(self, size: int = 1) -> bytes:
        """Read from serial, return empty bytes on failure."""
        try:
            if self.ser and self.ser.is_open:
                return self.ser.read(size)
        except (serial.SerialException, OSError):
            pass
        return b''

    def _safe_in_waiting(self) -> int:
        """Check bytes available, return 0 on failure."""
        try:
            if self.ser and self.ser.is_open:
                return self.ser.in_waiting
        except (serial.SerialException, OSError):
            pass
        return -1  # Negative signals error

    @staticmethod
    def _port_exists(port: str) -> bool:
        """Check if the serial port currently exists in the system."""
        # Check device file directly (Linux)
        if os.path.exists(port):
            return True
        # Fallback: scan listed ports
        available = [p.device for p in serial.tools.list_ports.comports()]
        return port in available

    # ─── Internal: Thread Management ─────────────────────────────────────

    def _start_threads(self):
        """Start command and feedback threads."""
        self._running = True
        self._connected.set()

        self._cmd_thread = threading.Thread(
            target=self._command_loop, name="cmd_loop", daemon=True
        )
        self._read_thread = threading.Thread(
            target=self._feedback_loop, name="fb_loop", daemon=True
        )
        self._cmd_thread.start()
        self._read_thread.start()

    def _join_threads(self):
        """Wait for threads to stop."""
        for t in [self._cmd_thread, self._read_thread, self._reconnect_thread]:
            if t and t.is_alive():
                t.join(timeout=3)
        self._cmd_thread = None
        self._read_thread = None
        self._reconnect_thread = None

    # ─── Internal: Reconnection ──────────────────────────────────────────

    def _trigger_reconnect(self):
        """
        Called by command or feedback thread when a serial error is detected.
        Spawns a reconnect thread if one isn't already running.
        """
        if self._shutdown:
            return

        # Only one reconnect at a time
        if not self._reconnecting.acquire(blocking=False):
            return

        self._connected.clear()
        print("\n\n!!! Connection lost. Starting reconnection...\n")

        self._reconnect_thread = threading.Thread(
            target=self._reconnect_worker, name="reconnect", daemon=True
        )
        self._reconnect_thread.start()

    def _reconnect_worker(self):
        """Background worker that attempts to reconnect and re-home."""
        try:
            # Stop the running threads (they will exit on _running=False or errors)
            self._running = False
            time.sleep(0.5)

            # Close broken serial
            self._close_serial()

            # Wait for port to reappear
            attempts = 0
            print(f"Waiting for {self.port} to reappear...")
            while not self._shutdown:
                if self._port_exists(self.port):
                    print(f"Port {self.port} detected. Waiting for settle...")
                    time.sleep(RECONNECT_SETTLE_TIME)

                    # Verify it's still there after settle
                    if self._port_exists(self.port):
                        break

                attempts += 1
                if MAX_RECONNECT_ATTEMPTS > 0 and attempts >= MAX_RECONNECT_ATTEMPTS:
                    print(
                        f"Max reconnect attempts ({MAX_RECONNECT_ATTEMPTS}) reached. Giving up.")
                    return

                sys.stdout.write(
                    f"\r  Waiting for port... (attempt {attempts})"
                )
                sys.stdout.flush()
                time.sleep(RECONNECT_POLL_INTERVAL)

            if self._shutdown:
                return

            # Try to open and re-home
            print(f"\nReconnecting to {self.port}...")
            try:
                self._open_serial()
                self._run_homing(timeout=60.0)
                self._start_threads()
                print("Reconnection successful. Threads restarted.\n")
            except Exception as e:
                print(f"Reconnection failed: {e}")
                print("Will retry...")
                self._close_serial()
                # Release lock and retry
                self._reconnecting.release()
                time.sleep(2)
                self._trigger_reconnect()
                return

        finally:
            # Release reconnect lock (if we still hold it)
            try:
                self._reconnecting.release()
            except RuntimeError:
                pass

    # ─── Internal: Homing ────────────────────────────────────────────────

    def _run_homing(self, timeout: float):
        """Send homing command and read text until HOMING_COMPLETE."""
        print("Sending homing command...")

        pkt = self._build_command(0, 0, [0] * NUM_LEDS, flags=FLAG_HOMING)
        if not self._safe_write(pkt):
            raise ConnectionError("Failed to send homing command")

        print("--- Homing ---")
        start = time.time()
        while time.time() - start < timeout:
            line = self._read_text_line(timeout=1.0)
            if line is None:
                continue
            print(f"  {line}")
            if line.startswith("HOMING_COMPLETE:"):
                try:
                    self._total_steps = int(line.split(":")[1])
                except (IndexError, ValueError):
                    self._total_steps = 0
                print(f"Homing complete. Total steps: {self._total_steps}")
                break
        else:
            raise TimeoutError(f"Homing did not complete within {timeout}s")

        time.sleep(0.1)
        try:
            if self.ser and self.ser.is_open:
                self.ser.reset_input_buffer()
        except (serial.SerialException, OSError):
            pass

    # ─── Internal: Serial Protocol ───────────────────────────────────────

    def _read_text_line(self, timeout: float = 0.5) -> str | None:
        """Read a newline-terminated ASCII line, ignoring binary data."""
        buf = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline:
            waiting = self._safe_in_waiting()
            if waiting < 0:
                return None  # Serial error
            if waiting > 0:
                b = self._safe_read(1)
                if not b:
                    return None
                byte = b[0]
                if byte == 0x0A:
                    try:
                        return buf.decode('ascii').strip()
                    except UnicodeDecodeError:
                        buf.clear()
                        continue
                elif byte == 0x0D:
                    continue
                elif 0x20 <= byte <= 0x7E or byte == 0x09:
                    buf.append(byte)
                else:
                    buf.clear()
            else:
                time.sleep(0.01)
        return None

    @staticmethod
    def _build_command(position: int, velocity: int, led_values: list[int],
                       flags: int = 0) -> bytes:
        """Build a 29-byte command packet."""
        vel_byte = velocity & 0xFF
        flags_byte = flags & 0xFF
        payload = bytes([position, vel_byte, flags_byte] + led_values)
        checksum = 0
        for b in payload:
            checksum ^= b
        return bytes([CMD_START]) + payload + bytes([checksum])

    @staticmethod
    def _parse_feedback(buf: bytes) -> dict | None:
        """Parse a 6-byte feedback packet."""
        if len(buf) < FB_PACKET_SIZE or buf[0] != FB_START:
            return None
        pos8 = buf[1]
        pos_raw = (buf[2] << 8) | buf[3]
        vel = struct.unpack('b', bytes([buf[4]]))[0]
        checksum = buf[1] ^ buf[2] ^ buf[3] ^ buf[4]
        if checksum != buf[5]:
            return None
        return {"pos_mapped": pos8, "pos_raw": pos_raw, "velocity": vel}

    # ─── Internal: Background Threads ────────────────────────────────────

    def _command_loop(self):
        """Send commands at fixed rate."""
        interval = 1.0 / CMD_SEND_HZ
        while self._running and not self._shutdown:
            start = time.time()

            with self._lock:
                pkt = self._build_command(
                    self._position,
                    self._velocity,
                    list(self._leds),
                    flags=0
                )

            if not self._safe_write(pkt):
                # Serial write failed — trigger reconnect
                self._trigger_reconnect()
                return

            elapsed = time.time() - start
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _feedback_loop(self):
        """Read feedback packets and print to terminal."""
        buf = bytearray()
        while self._running and not self._shutdown:
            waiting = self._safe_in_waiting()
            if waiting < 0:
                # Serial error — trigger reconnect
                self._trigger_reconnect()
                return
            if waiting > 0:
                data = self._safe_read(waiting)
                if not data:
                    self._trigger_reconnect()
                    return
                buf.extend(data)
            else:
                time.sleep(0.002)
                continue

            # Scan for feedback packets
            while len(buf) >= FB_PACKET_SIZE:
                idx = -1
                for i in range(len(buf)):
                    if buf[i] == FB_START:
                        idx = i
                        break
                if idx == -1:
                    buf.clear()
                    break
                if idx > 0:
                    buf = buf[idx:]
                if len(buf) < FB_PACKET_SIZE:
                    break

                fb = self._parse_feedback(bytes(buf[:FB_PACKET_SIZE]))
                if fb:
                    with self._fb_lock:
                        self._fb_pos_mapped = fb["pos_mapped"]
                        self._fb_pos_raw = fb["pos_raw"]
                        self._fb_velocity = fb["velocity"]

                    sys.stdout.write(
                        f"\r  pos={fb['pos_mapped']:3d}/255  "
                        f"raw={fb['pos_raw']:5d}  "
                        f"vel={fb['velocity']:+4d}   "
                    )
                    sys.stdout.flush()
                    buf = buf[FB_PACKET_SIZE:]
                else:
                    buf = buf[1:]

    # ─── Convenience Methods ─────────────────────────────────────────────

    def request_homing(self, timeout: float = 60.0):
        """Request re-homing while connected."""
        print("\nRequesting re-homing...")

        self._running = False
        self._join_threads()

        try:
            if self.ser and self.ser.is_open:
                self.ser.reset_input_buffer()
        except (serial.SerialException, OSError):
            pass

        self._run_homing(timeout)
        self._start_threads()
        print("Re-homing complete. Threads restarted.\n")

    def wait_while_running(self, duration: float):
        """
        Sleep for the given duration, but remain interruptible.
        Continues even if a reconnect happens mid-sleep.
        """
        deadline = time.time() + duration
        while time.time() < deadline and not self._shutdown:
            time.sleep(0.1)


# ─── Demo Main ───────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Stepper+NeoPixel Controller")
    parser.add_argument("--port", default="/dev/ttyACM0", help="Serial port")
    parser.add_argument("--no-home", action="store_true",
                        help="Skip homing on connect")
    args = parser.parse_args()

    ctrl = StepperLightController(args.port)

    try:
        ctrl.connect(home=not args.no_home)

        print("\n--- Test 1: Position=128, LEDs=127 ---")
        ctrl.position = 128
        ctrl.velocity = 0
        ctrl.leds = [127] * NUM_LEDS
        ctrl.wait_while_running(4)

        print("\n--- Test 2: Position=192, gradient LEDs ---")
        ctrl.position = 192
        ctrl.velocity = 0
        ctrl.leds = [int(255 * i / (NUM_LEDS - 1)) for i in range(NUM_LEDS)]
        ctrl.wait_while_running(4)

        print("\n--- Test 3: Velocity=+30 ---")
        ctrl.velocity = 30
        ctrl.leds = [50] * NUM_LEDS
        ctrl.wait_while_running(4)

        print("\n--- Test 4: Velocity=-100 ---")
        ctrl.velocity = -100
        ctrl.leds = [200] * NUM_LEDS
        ctrl.wait_while_running(4)

        print("\n--- Test 5: Stop, return to 0, LEDs off ---")
        ctrl.position = 0
        ctrl.velocity = 0
        ctrl.leds = [0] * NUM_LEDS
        ctrl.wait_while_running(4)

        print("\n\nTests complete.")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user.")
    finally:
        ctrl.disconnect()


if __name__ == "__main__":
    main()
