#!/usr/bin/env python3
"""igus CRI + Position Interface transport. No ROS or CloudXR dependencies.

Angles are radians in Python and degrees on the wire. Only the six robot joints
are sent: external axes, tools and digital outputs are never commanded.
Protocol references and live verification status are in ../AGENTS.md.
"""
import argparse
from dataclasses import asdict, dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import select
import socket
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config/real_arms.json"


# Jog control (CRI ALIVEJOG velocity jog, as the igus ROS driver Igus_rebel does it): jog [%] = 2.0 * velocity [deg/s]
# (empirical scale of that driver), sent every 10 ms; velocity = 0.4 * feed-forward + 1.0 * position error, the
# gains of its rebel_arm_trajectory_controller (tuned against a fit of the real arm at Override 100).
JOG_PERCENT_PER_DEG_S = 2.0
JOG_PERIOD = 0.01
JOG_KP = 1.0
JOG_FF = 0.4


class RobotError(RuntimeError):
    pass


class Frames:
    """TCP is a byte stream: frames may be split or coalesced arbitrarily."""
    def __init__(self, start="CRISTART", end="CRIEND"):
        self.start, self.end, self.buffer = start, end, ""

    def feed(self, data):
        # latin-1 maps every byte, so a non-ASCII program / frame name (or a split multi-byte
        # character) cannot fault the link; all fields we parse are ASCII.
        self.buffer += data.decode("latin-1")
        if len(self.buffer) > 262144:
            raise RobotError("oversized / unterminated robot message")
        out = []
        while self.end in self.buffer:
            raw, self.buffer = self.buffer.split(self.end, 1)
            if self.start not in raw:
                raise RobotError("invalid robot frame")
            out.append(raw.rsplit(self.start, 1)[1].split())
        return out


def finite_floats(values, count):
    q = tuple(float(x) for x in values[:count])
    if len(q) != count or not all(math.isfinite(x) for x in q):
        raise RobotError("invalid joint positions")
    return q


KINSTATE_TEXT = {0: "no error", 13: "joint limit min", 14: "joint limit max", 21: "centre singularity",
                 22: "out of reach", 23: "wrist singularity", 24: "wrist singularity", 50: "NaN in joint position",
                 99: "motion not allowed (motors off / hardware)",
                 **{k: "virtual box limit" for k in range(30, 36)}}


@dataclass(frozen=True)
class Status:
    joints: tuple
    estop: int
    kinstate: int
    error: str
    joint_errors: tuple
    received: float
    opmode: int

    def problem(self):
        if self.estop != 3:
            return f"ESTOP={self.estop} (expected 3)"
        if self.kinstate != 0 or any(self.joint_errors) or self.error.lower().replace("_", "") != "noerror":
            kin = KINSTATE_TEXT.get(self.kinstate, "undocumented - likely a velocity / jump check of the stream"
                                    if self.kinstate == 51 else "see the igus error table")
            return f"ERROR {self.error}, joint_errors={self.joint_errors}, KINSTATE={self.kinstate} ({kin})"
        if self.opmode == 2:
            return "OPMODE=2 (motion forbidden)"
        return None


def parse_status(tokens):
    def after(key, count):
        start = tokens.index(key) + 1
        values = tokens[start:start + count]
        if len(values) != count:
            raise RobotError(f"incomplete STATUS {key}")
        return values
    try:
        return Status(tuple(math.radians(x) for x in finite_floats(after("POSJOINTCURRENT", 6), 6)),
                      int(after("ESTOP", 1)[0]), int(after("KINSTATE", 1)[0]),
                      after("ERROR", 1)[0], tuple(int(x) for x in after("ERROR", 7)[1:]),
                      time.monotonic(), int(after("OPMODE", 1)[0]) if "OPMODE" in tokens else -1)
    except (ValueError, IndexError) as exc:
        raise RobotError(f"invalid CRI STATUS: {exc}") from exc


def load_endpoints(config=DEFAULT_CONFIG, left_ip=None, right_ip=None):
    data = json.loads(Path(config).read_text())
    for side, override in (("left", left_ip), ("right", right_ip)):
        host = override or data[side]["host"]
        socket.inet_pton(socket.AF_INET, host)
        port = data[side]["cri_port"]
        if type(port) is not int or not 1 <= port <= 65535:
            raise ValueError(f"invalid {side} CRI port")
        data[side] = {"host": host, "cri_port": port}
    if data["left"]["host"] == data["right"]["host"]:
        raise ValueError("left and right must have different IP addresses")
    return data


class EthernetArm:
    def __init__(self, side, host, cri_port=3920, state_timeout=0.5, command_timeout=0.25,
                 max_joint_vel=0.35, max_track_error=math.radians(15), rate=30,
                 engage_tolerance=math.radians(0.5), warmup_ticks=4, control="position"):
        """control: "jog" = CRI ALIVEJOG velocity jog (like the igus ROS driver, proven on our arms) or
        "position" = Position Interface streaming (RobotControl V15 stopped it with KINSTATE 51 on our arm)."""
        if control not in ("jog", "position"):
            raise ValueError(f"unknown control mode {control}")
        self.jogging = control == "jog"
        self.jog = [0.0] * 6
        self.side, self.host, self.cri_port = side, host, cri_port
        self.state_timeout, self.command_timeout = state_timeout, command_timeout
        self.max_joint_vel, self.max_track_error, self.rate = max_joint_vel, max_track_error, rate
        # First target of a stream is checked against the live pose, which may have moved a little (encoder
        # noise, settling after release) since the caller's snapshot: allow this much extra, once.
        self.engage_tolerance = engage_tolerance
        self.warmup_ticks = warmup_ticks
        self.fresh = False
        self.cri = self.position = None
        self.status = None
        self.active = None
        self.interface = None
        self.program_state = None
        self.position_joints, self.position_time = None, 0.0
        self.unknown_position_messages = []
        self.version = None
        self.fault = None
        self.moving = self.owns_position = self.enabled_interface = False
        self.last_target = None
        self.last_command = 0.0
        self._counter = 0
        self._acks = {}
        self._lock = threading.RLock()
        self._changed = threading.Condition(self._lock)
        self._closed = threading.Event()
        self._thread = None

    def _send(self, category, body=""):
        with self._lock:
            if self.cri is None:
                raise RobotError(f"{self.side}: CRI is closed")
            self._counter = self._counter % 9999 + 1
            counter = self._counter
            self._acks.pop(counter, None)
            self.cri.sendall(f"CRISTART {counter} {category} {body} CRIEND\n".encode("ascii"))
            return counter

    def _wait(self, predicate, timeout=3):
        end = time.monotonic() + timeout
        with self._changed:
            while not predicate():
                if self.fault:
                    raise RobotError(f"{self.side}: {self.fault}")
                left = end - time.monotonic()
                if left <= 0 or self._closed.is_set():
                    raise RobotError(f"{self.side}: robot response timed out")
                self._changed.wait(min(left, .05))

    def command(self, body):
        counter = self._send("CMD", body)
        self._wait(lambda: counter in self._acks)
        with self._lock:
            error = self._acks.pop(counter)
        if error:
            raise RobotError(f"{self.side}: {body}: {error}")

    def connect(self):
        self.cri = socket.create_connection((self.host, self.cri_port), timeout=2)
        self.cri.settimeout(.1)
        self.cri.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._thread = threading.Thread(target=self._io, name=f"cri-{self.side}", daemon=True)
        self._thread.start()
        self._send("CMD", "GetActive")
        self._send("CMD", "GetVersion")
        self._send("CMD", "GetPositionInterface")
        self._wait(lambda: self.status is not None and self.interface is not None and self.active is not None
                   and self.program_state is not None)

    def _on_cri(self, tokens):
        if len(tokens) < 2:
            raise RobotError("incomplete CRI message")
        kind, body = tokens[1], tokens[2:]
        with self._changed:
            if kind == "STATUS":
                self.status = parse_status(body)
            elif kind == "CMD" and body[:1] == ["Active"]:
                self.active = body[1].lower() == "true"
            elif kind == "CMD" and body[:1] == ["PositionInterface"]:
                port = int(body[1])
                if not 1 <= port <= 65535:
                    raise RobotError("invalid Position Interface port")
                self.interface = (port, body[2].lower() == "true", body[3].lower() == "true")
            elif kind in ("CMDACK", "CMDERROR"):
                self._acks[int(body[0])] = " ".join(body[1:]) if kind == "CMDERROR" else None
            elif kind == "RUNSTATE" and body[:1] == ["MAIN"]:
                self.program_state = int(body[-2])  # <run state> <replay mode> are last; names may contain spaces
            elif kind == "INFO" and body[:1] == ["Version"]:
                self.version = " ".join(body[1:])
            self._changed.notify_all()

    def _io(self):
        cri_frames, pos_frames = Frames(), Frames("MSGSTART", "MSGEND")
        last_alive = 0.0
        try:
            while not self._closed.is_set():
                now = time.monotonic()
                if self.jogging and now - last_alive >= JOG_PERIOD:
                    with self._lock:
                        st = self.status
                        live = (self.moving and now - self.last_command < 0.1 and st is not None
                                and now - st.received < 0.2)  # stale command or STATUS: zero jog at once
                        jog = self.jog if live else [0.0] * 6
                    self._send("ALIVEJOG", " ".join(f"{v:.4f}" for v in jog) + " 0 0 0")
                    last_alive = now
                elif not self.jogging and now - last_alive >= .2:
                    self._send("ALIVEJOG", "0 0 0 0 0 0 0 0 0")
                    last_alive = now
                with self._lock:
                    sockets = [s for s in (self.cri, self.position) if s is not None]
                readable, _, _ = select.select(sockets, [], [], .004 if self.jogging else .02)
                for sock in readable:
                    data = sock.recv(65536)
                    if not data:
                        raise RobotError("robot closed TCP connection")
                    if sock is self.cri:
                        for tokens in cri_frames.feed(data):
                            self._on_cri(tokens)
                    else:
                        for tokens in pos_frames.feed(data):
                            if tokens[:1] == ["ERROR"]:  # MSGSTART ERROR UNKNOWN|FORMAT MSGEND: target rejected
                                raise RobotError(f"Position Interface rejected a message: {' '.join(tokens)}")
                            if tokens[:2] != ["Pos", "J"]:
                                # e.g. "MSGSTART OK MSGEND", the acknowledgement RobotControl V15 sends for each
                                # target (not in the V1 document); anything else unknown is ignored but kept.
                                if tokens[:1] != ["OK"]:
                                    self.unknown_position_messages.append(tokens[:8])
                                continue
                            q = tuple(math.radians(x) for x in finite_floats(tokens[2:], 6))
                            with self._changed:
                                self.position_joints, self.position_time = q, time.monotonic()
                                self._changed.notify_all()
                if self.moving:
                    self.require_ready(position=True)
                    if time.monotonic() - self.last_command > self.command_timeout:
                        raise RobotError("command watchdog expired (Quest / control loop stalled)")
        except (OSError, ValueError, IndexError, RobotError) as exc:
            if not self._closed.is_set():
                self.fault = str(exc)
                self.abort()

    def require_ready(self, position=False):
        with self._lock:
            if self.fault:
                raise RobotError(f"{self.side}: {self.fault}")
            if self.status is None or time.monotonic() - self.status.received > self.state_timeout:
                raise RobotError(f"{self.side}: stale CRI joint state")
            problem = self.status.problem()
            if problem:
                raise RobotError(f"{self.side}: {problem}; prepare / enable this arm with the robot's own controls")
            if self.active is not True:
                raise RobotError(f"{self.side}: CRI is passive; close the other driver / control client first")
            if self.program_state != 0:
                raise RobotError(f"{self.side}: robot program is running / paused or RUNSTATE is unavailable")
            if position and not self.jogging and (self.position is None
                                                  or time.monotonic() - self.position_time > self.state_timeout):
                raise RobotError(f"{self.side}: stale Position Interface state")

    def enable_motors(self, timeout=6.0):
        """Operator-requested: Reset + Enable (the iRC "Reset" / "Enable" buttons) when the only problem is
        'motor not enabled'. Refuses on e-stop, a passive connection, a running program or any other error.
        The arm holds its pose after enabling. Note: Reset also resets the robot's digital outputs."""
        with self._lock:
            status = self.status
        if status is None:
            raise RobotError(f"{self.side}: no CRI status yet")
        if status.problem() is None:
            return False
        if status.estop != 3:
            raise RobotError(f"{self.side}: ESTOP={status.estop}; release the e-stop first")
        if self.active is not True:
            raise RobotError(f"{self.side}: CRI is passive; close the other driver / control client first")
        if self.program_state != 0:
            raise RobotError(f"{self.side}: a robot program is running; stop it first")
        # MNE (motors off) and a latched kinematic stop (e.g. KINSTATE 50/51 after a rejected target, 13/14 at a
        # joint limit) are cleared by Reset + Enable; drive / encoder / current errors are not touched.
        not_enabled_only = all(e in (0, 4) for e in status.joint_errors) and status.kinstate in (0, 13, 14, 50, 51, 99) \
            and set(t for t in status.error.upper().split("_") if t) <= {"MNE", "NOTREADY", "NOERROR", "NO", "ERROR"}
        if not not_enabled_only:
            raise RobotError(f"{self.side}: {status.problem()} is more than 'motor not enabled'; "
                             "check the robot before enabling")
        self.command("Reset")
        self.command("Enable")
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            with self._lock:
                if self.status is not None and self.status.received > status.received and self.status.problem() is None:
                    return True
            if self.fault:
                raise RobotError(f"{self.side}: {self.fault}")
            time.sleep(.05)
        with self._lock:
            raise RobotError(f"{self.side}: motors still not enabled after Reset + Enable ({self.status.problem()})")

    def prepare(self):
        self.require_ready()
        if self.jogging:
            # same start-up as the igus driver: joint jog mode, velocity override 100 % (jog = commanded speed)
            self.command("MotionTypeJoint")
            self.command("Override 100.0")
            return
        port, running, in_use = self.interface
        if in_use:
            raise RobotError(f"{self.side}: Position Interface already in use by another client")
        if not running:
            # Record ownership before the request so partial failure is also cleaned up.
            self.enabled_interface = True
            self._send("CONFIG", "SetPositionInterface true")
            # The CRI document says the new state is also sent in reply to CONFIG, but RobotControl V15 only
            # logs "Position Interface: Starting": poll GetPositionInterface until it reports running.
            end = time.monotonic() + 4.0
            while not self.interface[1]:
                if time.monotonic() > end:
                    raise RobotError(f"{self.side}: Position Interface did not start")
                self._send("CMD", "GetPositionInterface")
                try:
                    self._wait(lambda: self.interface[1], timeout=0.25)
                except RobotError:
                    if self.fault or self._closed.is_set():
                        raise
        pos = socket.create_connection((self.host, port), timeout=2)
        pos.settimeout(.1)
        pos.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with self._lock:
            self.position = pos
        self._wait(lambda: self.position_joints is not None)
        self.require_ready(position=True)

    def _position(self, q):
        q = finite_floats(q, 6)
        values = " ".join(f"{math.degrees(x):.8f}" for x in q)
        self.position.sendall(f"MSGSTART Pos J {values} MSGEND\n".encode("ascii"))

    def target(self, q):
        self.require_ready(position=True)
        if len(q) != 6:
            raise RobotError(f"{self.side}: exactly six joint targets are required")
        q = finite_floats(q, 6)
        measured = self.measured()
        if max(abs(x-y) for x, y in zip(q, measured)) > self.max_track_error:
            raise RobotError(f"{self.side}: excessive joint following error")
        if not self.moving:
            self.begin_stream()
        previous = self.last_target
        # A delayed tick never grants a larger step. This also rejects jumps on engagement; the first target
        # of a stream may differ a little more (encoder noise between the caller's snapshot and now).
        limit = self.max_joint_vel / self.rate + (self.engage_tolerance if self.fresh else 0.0)
        if max(abs(x-y) for x, y in zip(q, previous)) > limit + 1e-6:
            raise RobotError(f"{self.side}: target exceeds per-tick joint speed limit")
        self.fresh = False
        with self._lock:
            self.require_ready(position=True)
            if self.jogging:
                self.jog = self._jog_toward(q)
            else:
                self._position(q)
            self.last_target, self.last_command = q, time.monotonic()

    def check_first_target(self, q):
        """Validate the first target of a stream against the live pose BEFORE anything is sent."""
        self.require_ready(position=True)
        if len(q) != 6:
            raise RobotError(f"{self.side}: exactly six joint targets are required")
        q = finite_floats(q, 6)
        step = max(abs(x - y) for x, y in zip(q, self.measured()))
        if step > self.max_track_error:
            raise RobotError(f"{self.side}: excessive joint following error")
        if step > self.max_joint_vel / self.rate + self.engage_tolerance + 1e-6:
            raise RobotError(f"{self.side}: target exceeds per-tick joint speed limit")

    def measured(self):
        """Latest measured joints [rad]: Position Interface stream, or CRI STATUS POSJOINTCURRENT in jog mode."""
        with self._lock:
            return self.position_joints if not self.jogging else self.status.joints

    def _jog_toward(self, q):
        """Velocity jog [%] that moves the arm along the target stream: feed-forward + position error."""
        meas = self.status.joints
        vmax = 1.2 * self.max_joint_vel
        out = []
        for qi, pi, mi in zip(q, self.last_target, meas):
            v = JOG_FF * (qi - pi) * self.rate + JOG_KP * (qi - mi)
            v = max(-vmax, min(vmax, v))
            out.append(max(-100.0, min(100.0, JOG_PERCENT_PER_DEG_S * math.degrees(v))))
        return out

    def begin_stream(self):
        """Seed the measured pose and select the Position Interface as the position source (the arm holds)."""
        self.require_ready(position=True)
        if self.jogging:
            with self._lock:
                self.last_target, self.last_command = self.status.joints, time.monotonic()
                self.jog, self.moving, self.fresh = [0.0] * 6, True, True
            return
        measured = self.position_joints
        self.owns_position = True
        with self._lock:
            self._position(measured)
        self.command("UsePositionInterface true")
        with self._lock:
            self.last_target, self.last_command, self.moving, self.fresh = measured, time.monotonic(), True, True

    def resend(self):
        """Repeat the last target (keeps the stream steady and the watchdog fed while another arm warms up)."""
        with self._lock:
            self.require_ready(position=True)
            if self.jogging:
                self.jog = self._jog_toward(self.last_target)
                self.last_command = time.monotonic()
                return
            self._position(self.last_target)
            self.last_command = time.monotonic()

    def hold(self):
        if self.jogging:
            with self._lock:
                self.jog, self.moving, self.last_target = [0.0] * 6, False, None
            return
        if self.owns_position:
            # Close immediately on failed ACK; never leave a live position source behind.
            try:
                self.command("UsePositionInterface false")
            except (OSError, RobotError):
                self.abort()
                raise
            self.moving = self.owns_position = False
            self.last_target = None

    def joints(self):
        with self._lock:
            if self.position_joints is not None and not self.jogging:
                return self.position_joints, self.position_time
            return self.status.joints, self.status.received

    def abort(self):
        """Immediate socket close is the Position Interface's documented stop mechanism."""
        self._closed.set()
        with self._changed:
            self.jog = [0.0] * 6
            if self.jogging and self.cri is not None:
                try:
                    self._send("ALIVEJOG", "0 0 0 0 0 0 0 0 0")  # stop the jog before anything else
                except OSError:
                    pass
            # Closing position TCP takes precedence over waiting on any CRI send.
            if self.position is not None:
                try:
                    self.position.shutdown(socket.SHUT_RDWR)
                except (OSError, RobotError):
                    pass
                self.position.close()
                self.position = None
            if self.owns_position and self.cri is not None:
                try:
                    self._send("CMD", "UsePositionInterface false")
                except (OSError, RobotError):
                    pass
            if self.enabled_interface and self.cri is not None:
                try:
                    self._send("CONFIG", "SetPositionInterface false")
                except OSError:
                    pass
            if self.cri is not None:
                try:
                    self._send("QUIT")
                    # Half-close and read until the robot closes (<= 0.3 s): an immediate close with unread
                    # STATUS data queued becomes a TCP RST, and the robot may then drop the commands above.
                    self.cri.shutdown(socket.SHUT_WR)
                    self.cri.settimeout(.05)
                    end = time.monotonic() + .3
                    while time.monotonic() < end:
                        try:
                            if not self.cri.recv(65536):
                                break
                        except socket.timeout:
                            continue
                except OSError:
                    pass
                self.cri.close()
                self.cri = None
            self.moving = self.owns_position = False
            self._changed.notify_all()

    def close(self):
        self.abort()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1)


def watch_arm(side, host, cri_port=3920, state_timeout=0.5):
    """Read-only CRI link to an arm this program does NOT drive (e.g. someone else is using it): only its
    STATUS joint angles are read, for the inter-arm collision guard. Never configures the Position
    Interface or sends commands; if this connection was made the active one (nobody else connected), it
    hands that back at once with SetActive false so it cannot get in the other operator's way."""
    arm = EthernetArm(side, host, cri_port, state_timeout=state_timeout)
    arm.connect()
    if arm.active:
        arm._send("CMD", "SetActive false")
    return arm


class EthernetArms:
    """All selected arms must pass preflight; any transport fault closes the pair."""
    def __init__(self, endpoints, sides=("left", "right"), dry_run=False, enable_motors=False, control="position",
                 **limits):
        self.arms = {s: EthernetArm(s, **endpoints[s], control=control, **limits) for s in sides}
        self.dry_run = dry_run
        self.enable_motors = enable_motors
        self.control = control
        self.enabled = []  # sides whose motors this session enabled
        self.fault = None
        self._closed = threading.Event()
        self._monitor = None

    def connect(self):
        try:
            for arm in self.arms.values():
                arm.connect()
            if not self.dry_run:
                if self.enable_motors:
                    for side, arm in self.arms.items():
                        if arm.enable_motors():
                            self.enabled.append(side)
                # Check both BEFORE configuring either Position Interface.
                for arm in self.arms.values():
                    arm.require_ready()
                for arm in self.arms.values():
                    arm.prepare()
            self._monitor = threading.Thread(target=self._watch, daemon=True)
            self._monitor.start()
        except BaseException:
            self.close()
            raise

    def _watch(self):
        while not self._closed.wait(.02):
            for side, arm in self.arms.items():
                if arm.fault:
                    self.fault = f"{side}: {arm.fault}"
                    self.close()
                    return

    def send(self, commands):
        try:
            if self.fault:
                raise RobotError(self.fault)
            if self._closed.is_set():
                raise RobotError("Ethernet transport is closed")
            if self.dry_run:
                return
            for arm in self.arms.values():
                arm.require_ready(position=True)
            starting = [a for s, a in self.arms.items() if s in commands and not a.moving]
            for arm in starting:
                arm.check_first_target(commands[arm.side])
            if starting and self.control == "position":
                # The robot interpolates over the AVERAGE interval between received targets. Right after the
                # seed that interval is ~ms, so the first real target looked like a jump and the controller
                # stopped the arm with a kinematic error (KINSTATE 51 seen on the right arm). Select the
                # source, then re-send the held pose at the tick rate - every streaming arm together, so no
                # arm's watchdog starves while another one warms up.
                for arm in starting:
                    arm.begin_stream()
                rate = starting[0].rate
                for _ in range(starting[0].warmup_ticks):
                    time.sleep(1.0 / rate)
                    for arm in self.arms.values():
                        if arm.moving:
                            arm.resend()
            for side, arm in self.arms.items():
                if side in commands:
                    arm.target(commands[side])
                else:
                    arm.hold()
        except KeyboardInterrupt:
            raise  # an exit request, not a fault: keep the links so the caller can still park
        except BaseException:
            self.close()
            raise

    def park(self, target, max_vel, log=print, guard=None, settle=0.5):
        """Stream every selected arm slowly to the joint pose `target` (6 values, rad), then release the
        position source. Straight line in joint space, all arms arrive together, at <= 90 % of the per-tick
        limit; the usual readiness / following-error / watchdog checks apply on every tick.
        guard(q_by_side) may return a reason string to stop early (the arms then hold where they are).
        Returns True when the pose was reached, False on a dry run or when the guard stopped it."""
        if self.dry_run:
            return False
        if self.fault:
            raise RobotError(self.fault)
        target = finite_floats(target, 6)
        arms = list(self.arms.values())
        rate = arms[0].rate
        vel = min(max_vel, 0.9 * min(a.max_joint_vel for a in arms))
        self.send({})          # release any live stream so the motion starts from the measured pose
        time.sleep(.3)
        start = {side: arm.joints()[0] for side, arm in self.arms.items()}
        span = max(abs(t - q) for qs in start.values() for t, q in zip(target, qs))
        steps = max(1, math.ceil(span / vel * rate))
        if self.control == "jog":
            settle = max(settle, 3.0)  # velocity loop: let the position error decay before releasing
        log(f"[park] moving {', '.join(self.arms)} to {[round(math.degrees(t), 1) for t in target]} deg "
            f"({steps / rate:.1f} s at {math.degrees(vel):.0f} deg/s)")
        for k in range(1, steps + 1 + int(settle * rate)):
            t0 = time.monotonic()
            a = min(k, steps) / steps
            cmd = {side: tuple(q + (t - q) * a for t, q in zip(target, qs)) for side, qs in start.items()}
            reason = guard(cmd) if guard else None
            if reason:
                self.send({})
                log(f"[park] stopped: {reason}")
                return False
            self.send(cmd)
            time.sleep(max(0.0, 1.0 / rate - (time.monotonic() - t0)))
        self.send({})
        log("[park] done: " + ", ".join(f"{side} {[round(math.degrees(x), 1) for x in arm.joints()[0]]} deg"
                                       for side, arm in self.arms.items()))
        return True

    def close(self):
        self._closed.set()
        # Abort every link before joining receiver threads.
        for arm in self.arms.values():
            arm.abort()
        for arm in self.arms.values():
            arm.close()

    def report(self):
        return {side: {"host": arm.host, "cri_port": arm.cri_port, "version": arm.version,
                       "active": arm.active, "position_interface": arm.interface,
                       "joints_deg": [round(math.degrees(x), 3) for x in arm.status.joints],
                       "status": asdict(arm.status), "motion_ready": arm.status.problem() is None
                       and arm.active is True and arm.program_state == 0,
                       "program_state": arm.program_state}
                for side, arm in self.arms.items()}


def main():
    p = argparse.ArgumentParser(description="Read-only Ethernet check: no motor enable, configuration or motion")
    p.add_argument("--check-only", action="store_true")
    p.add_argument("--arms", nargs="+", choices=("left", "right"), default=["left", "right"])
    p.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    p.add_argument("--left-ip")
    p.add_argument("--right-ip")
    args = p.parse_args()
    pair = EthernetArms(load_endpoints(args.config, args.left_ip, args.right_ip), args.arms, dry_run=True)
    try:
        pair.connect()
        report = {"observed_at": datetime.now().astimezone().isoformat(), "read_only": True,
                  "arms": pair.report()}
        output = ROOT / "logs/real_connection_check.json"
        output.parent.mkdir(exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        print(f"Saved {output}")
    except (OSError, ValueError, RobotError) as exc:
        p.exit(1, f"Ethernet check failed: {exc}\n")
    finally:
        pair.close()


if __name__ == "__main__":
    main()
