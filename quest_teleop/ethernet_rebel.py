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


class RobotError(RuntimeError):
    pass


class Frames:
    """TCP is a byte stream: frames may be split or coalesced arbitrarily."""
    def __init__(self, start="CRISTART", end="CRIEND"):
        self.start, self.end, self.buffer = start, end, ""

    def feed(self, data):
        self.buffer += data.decode("ascii", errors="strict")
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
            return f"ERROR {self.error}, joint_errors={self.joint_errors}, KINSTATE={self.kinstate}"
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
                 max_joint_vel=0.35, max_track_error=math.radians(15), rate=30):
        self.side, self.host, self.cri_port = side, host, cri_port
        self.state_timeout, self.command_timeout = state_timeout, command_timeout
        self.max_joint_vel, self.max_track_error, self.rate = max_joint_vel, max_track_error, rate
        self.cri = self.position = None
        self.status = None
        self.active = None
        self.interface = None
        self.program_state = None
        self.position_joints, self.position_time = None, 0.0
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
                self.program_state = int(body[5])
            elif kind == "INFO" and body[:1] == ["Version"]:
                self.version = " ".join(body[1:])
            self._changed.notify_all()

    def _io(self):
        cri_frames, pos_frames = Frames(), Frames("MSGSTART", "MSGEND")
        last_alive = 0.0
        try:
            while not self._closed.is_set():
                now = time.monotonic()
                if now - last_alive >= .2:
                    self._send("ALIVEJOG", "0 0 0 0 0 0 0 0 0")
                    last_alive = now
                with self._lock:
                    sockets = [s for s in (self.cri, self.position) if s is not None]
                readable, _, _ = select.select(sockets, [], [], .02)
                for sock in readable:
                    data = sock.recv(65536)
                    if not data:
                        raise RobotError("robot closed TCP connection")
                    if sock is self.cri:
                        for tokens in cri_frames.feed(data):
                            self._on_cri(tokens)
                    else:
                        for tokens in pos_frames.feed(data):
                            if tokens[:2] != ["Pos", "J"]:
                                raise RobotError(f"unexpected Position Interface message: {tokens}")
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
            if position and (self.position is None or time.monotonic() - self.position_time > self.state_timeout):
                raise RobotError(f"{self.side}: stale Position Interface state")

    def prepare(self):
        self.require_ready()
        port, running, in_use = self.interface
        if in_use:
            raise RobotError(f"{self.side}: Position Interface already in use by another client")
        if not running:
            # Record ownership before the request so partial failure is also cleaned up.
            self.enabled_interface = True
            self._send("CONFIG", "SetPositionInterface true")
            self._wait(lambda: self.interface[1])
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
        measured = self.position_joints
        if max(abs(x-y) for x, y in zip(q, measured)) > self.max_track_error:
            raise RobotError(f"{self.side}: excessive joint following error")
        previous = self.last_target if self.moving else measured
        # A delayed tick never grants a larger step. This also rejects jumps on engagement.
        if max(abs(x-y) for x, y in zip(q, previous)) > self.max_joint_vel / self.rate + 1e-6:
            raise RobotError(f"{self.side}: target exceeds per-tick joint speed limit")
        if not self.moving:
            self.owns_position = True
            with self._lock:
                self._position(measured)  # seed the latest physical pose before selecting this source
            self.command("UsePositionInterface true")
            self.last_command = time.monotonic()
            self.moving = True
        with self._lock:
            self.require_ready(position=True)
            self._position(q)
            self.last_target, self.last_command = q, time.monotonic()

    def hold(self):
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
            if self.position_joints is not None:
                return self.position_joints, self.position_time
            return self.status.joints, self.status.received

    def abort(self):
        """Immediate socket close is the Position Interface's documented stop mechanism."""
        self._closed.set()
        with self._changed:
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
                    self.cri.shutdown(socket.SHUT_RDWR)
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


class EthernetArms:
    """All selected arms must pass preflight; any transport fault closes the pair."""
    def __init__(self, endpoints, sides=("left", "right"), dry_run=False, **limits):
        self.arms = {s: EthernetArm(s, **endpoints[s], **limits) for s in sides}
        self.dry_run = dry_run
        self.fault = None
        self._closed = threading.Event()
        self._monitor = None

    def connect(self):
        try:
            for arm in self.arms.values():
                arm.connect()
            if not self.dry_run:
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
            for side, arm in self.arms.items():
                if side in commands:
                    arm.target(commands[side])
                else:
                    arm.hold()
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
