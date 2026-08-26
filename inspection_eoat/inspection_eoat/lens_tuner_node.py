"""
Bring-up and StallGuard-tuning aid for the macro-ps focus lens.

Serves a small web UI (no X11, so it works on a headless Pi over the network)
that can jog the lens open-loop and record StallGuard against travel. The point
is to replace the guessed constants in firmware config.h -- STALL_THRESHOLD,
HOMING_VELOCITY, TMC_RMS_CURRENT_MA -- with numbers measured on the real
mechanism, before the homing FSM is ever allowed to drive into a hard stop.

Two safety properties matter here, because this node is used on a lens with no
calibrated range:

  * The firmware applies a VEL_WATCHDOG_MS (150 ms) deadman to velocity mode, so
    a jog must be re-published continuously. That repeat lives here on a ROS
    timer rather than in the browser -- if the page closes, the network drops or
    this node dies, the lens coasts to a stop within 150 ms on its own.

  * Every jog is bounded by a step budget. The node watches `steps` coming back
    in /lens/state and cancels the jog on arrival, so a stuck button or a lost
    page cannot walk the lens into a mechanical stop.
"""

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from ps_interfaces.msg import LensCommand, LensState

STATUS_NAMES = {
    LensState.STATUS_UNCALIBRATED: 'UNCALIBRATED',
    LensState.STATUS_IDLE: 'IDLE',
    LensState.STATUS_MOVING: 'MOVING',
    LensState.STATUS_STALLED: 'STALLED',
    LensState.STATUS_HOMING: 'HOMING',
}

HTML_PATH = os.path.join(os.path.dirname(__file__), 'lens_tuner.html')


class LensTuner(Node):
    def __init__(self):
        super().__init__('lens_tuner')

        self.declare_parameter('state_topic', '/camera_head/lens/state')
        self.declare_parameter('command_topic', '/camera_head/lens/command')
        self.declare_parameter('port', 8080)
        # Must be comfortably faster than the firmware's VEL_WATCHDOG_MS (150 ms)
        # or every jog will stutter as the deadman repeatedly expires.
        self.declare_parameter('command_rate_hz', 20.0)
        # Hard ceiling applied to whatever the UI asks for. The mechanism's real
        # travel is unknown until homing has run at least once.
        self.declare_parameter('max_step_budget', 4000)
        # StallGuard4 needs real shaft speed: at 16 microsteps, 200 steps/s is
        # only 3.75 RPM. Headroom here is what makes the threshold tunable.
        self.declare_parameter('max_velocity', 3000.0)
        self.declare_parameter('log_capacity', 20000)

        state_topic = self.get_parameter('state_topic').value
        cmd_topic = self.get_parameter('command_topic').value
        self.port = int(self.get_parameter('port').value)
        self.max_budget = int(self.get_parameter('max_step_budget').value)
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.log_capacity = int(self.get_parameter('log_capacity').value)

        # /lens/state is published best-effort by the firmware; a reliable
        # subscription would simply never match it.
        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        # The command subscription on the MCU is reliable. Dropping a stop is
        # the one message we cannot afford to lose.
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)

        self.pub = self.create_publisher(LensCommand, cmd_topic, cmd_qos)
        self.create_subscription(LensState, state_topic, self._on_state, state_qos)

        self.lock = threading.Lock()
        self.state = None            # last LensState as a plain dict
        self.state_stamp = 0.0       # wall clock of last state message
        self.jog_velocity = 0.0      # 0 => idle; the deadman does the rest
        self.jog_stop_steps = None   # step count at which to cancel
        self.jog_started_steps = None
        self.last_event = 'idle'
        self.log = []                # (t, steps, velocity, sg_result, status)
        self.log_t0 = None
        self.recording = True

        period = 1.0 / float(self.get_parameter('command_rate_hz').value)
        self.create_timer(period, self._publish_command)

        self.httpd = ThreadingHTTPServer(('0.0.0.0', self.port), _make_handler(self))
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'lens tuner UI on http://<this-host>:{self.port}/')

    # -- ROS ---------------------------------------------------------------
    def _on_state(self, msg: LensState):
        now = time.time()
        with self.lock:
            self.state = {
                'position_norm': None if math.isnan(msg.position_norm) else round(msg.position_norm, 5),
                'velocity': msg.velocity,
                'status': int(msg.status),
                'status_name': STATUS_NAMES.get(int(msg.status), f'?{msg.status}'),
                'steps': int(msg.steps),
                'sg_result': int(msg.sg_result),
                'driver_ok': bool(msg.driver_ok),
                'driver_enabled': bool(msg.driver_enabled),
            }
            self.state_stamp = now

            if self.log_t0 is None:
                self.log_t0 = now
            if self.recording and len(self.log) < self.log_capacity:
                self.log.append((round(now - self.log_t0, 4), int(msg.steps),
                                 float(msg.velocity), int(msg.sg_result), int(msg.status)))

            # Step-budget cancel. Compared by absolute travel so it works in
            # either direction and survives the count passing through zero.
            if self.jog_velocity != 0.0 and self.jog_stop_steps is not None:
                if abs(int(msg.steps) - self.jog_started_steps) >= self.jog_stop_steps:
                    self.jog_velocity = 0.0
                    self.jog_stop_steps = None
                    self.last_event = 'step budget reached'

    def _publish_command(self):
        """Re-publish the active jog to hold off the firmware deadman.

        Also emits an explicit zero once the jog ends rather than just going
        quiet: relying on the deadman alone would leave the lens running for up
        to VEL_WATCHDOG_MS after every stop.
        """
        with self.lock:
            v = self.jog_velocity
        msg = LensCommand()
        msg.mode = LensCommand.MODE_VELOCITY
        msg.value = float(v)
        self.pub.publish(msg)

    # -- called from the HTTP thread ---------------------------------------
    def start_jog(self, velocity, budget):
        velocity = max(-self.max_velocity, min(self.max_velocity, float(velocity)))
        budget = max(1, min(self.max_budget, int(budget)))
        with self.lock:
            if self.state is None:
                return False, 'no /lens/state yet — is the agent up?'
            if self.state['status'] == LensState.STATUS_HOMING:
                return False, 'homing owns the lens'
            self.jog_started_steps = self.state['steps']
            self.jog_stop_steps = budget
            self.jog_velocity = velocity
            self.last_event = f'jog {velocity:+.0f} steps/s, budget {budget}'
        return True, self.last_event

    def stop(self):
        with self.lock:
            self.jog_velocity = 0.0
            self.jog_stop_steps = None
            self.last_event = 'stopped'

    def snapshot(self):
        with self.lock:
            stale = (time.time() - self.state_stamp) > 1.0 if self.state else True
            return {
                'state': self.state,
                'stale': stale,
                'jog_velocity': self.jog_velocity,
                'event': self.last_event,
                'recording': self.recording,
                'log_len': len(self.log),
                'log_capacity': self.log_capacity,
                'max_velocity': self.max_velocity,
                'max_budget': self.max_budget,
                # Recent tail for the plot; the browser keeps no history of its own
                # so a page reload still shows the run.
                'tail': self.log[-600:],
            }

    def csv(self):
        with self.lock:
            rows = list(self.log)
        out = ['t_s,steps,velocity_steps_s,sg_result,status']
        out += [f'{t},{s},{v},{sg},{st}' for (t, s, v, sg, st) in rows]
        return '\n'.join(out) + '\n'

    def clear_log(self):
        with self.lock:
            self.log.clear()
            self.log_t0 = None

    def set_recording(self, on):
        with self.lock:
            self.recording = bool(on)


def _make_handler(node: 'LensTuner'):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass   # keep the ROS console readable

        def _send(self, code, body, ctype='application/json', extra=None):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                try:
                    with open(HTML_PATH, 'rb') as f:
                        self._send(200, f.read(), 'text/html; charset=utf-8')
                except OSError:
                    self._send(500, 'UI file missing', 'text/plain')
            elif self.path == '/api/state':
                self._send(200, json.dumps(node.snapshot()))
            elif self.path == '/api/log.csv':
                self._send(200, node.csv(), 'text/csv',
                           {'Content-Disposition': 'attachment; filename="lens_sg_log.csv"'})
            else:
                self._send(404, 'not found', 'text/plain')

        def do_POST(self):
            n = int(self.headers.get('Content-Length') or 0)
            try:
                payload = json.loads(self.rfile.read(n) or b'{}')
            except ValueError:
                payload = {}

            if self.path == '/api/jog':
                ok, msg = node.start_jog(payload.get('velocity', 0),
                                         payload.get('budget', 100))
                self._send(200 if ok else 409, json.dumps({'ok': ok, 'message': msg}))
            elif self.path == '/api/stop':
                node.stop()
                self._send(200, json.dumps({'ok': True}))
            elif self.path == '/api/clear':
                node.clear_log()
                self._send(200, json.dumps({'ok': True}))
            elif self.path == '/api/record':
                node.set_recording(payload.get('on', True))
                self._send(200, json.dumps({'ok': True}))
            else:
                self._send(404, 'not found', 'text/plain')
    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = LensTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node._publish_command()      # explicit zero on the way out
        node.httpd.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
