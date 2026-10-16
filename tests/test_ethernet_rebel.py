"""Socket-level tests against two local fake controllers. Never contacts real hardware."""
import math
from pathlib import Path
import socket
import sys
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "quest_teleop"))
from ethernet_rebel import EthernetArms, Frames, RobotError, parse_status


def status_tokens(error=0, kinstate=0, joints=(0, 0, 0, 0, 0, 0)):
    return ("MODE joint POSJOINTCURRENT " + " ".join(map(str, joints)) + " ESTOP 3 ERROR "
            f"{'NoError' if not error else '_MNE'} " + " ".join([str(error)] * 6)
            + f" KINSTATE {kinstate} OPMODE -1").split()


def eventually(predicate, timeout=1):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError("condition not reached")


class FakeRobot:
    def __init__(self):
        self.cri_server, self.pos_server = self.listener(), self.listener()
        self.commands, self.targets = [], []
        self.enabled = self.in_use = False
        self.error = self.kinstate = 0
        self.joints_deg = [0] * 6
        self.active = True
        self.program_state = 0
        self.reject_use = False
        self.mute_status = False
        self.follow = False  # True: the fake arm jumps to every streamed target (ideal tracking)
        self.cri = self.pos = None
        self.closed = threading.Event()
        self.threads = [threading.Thread(target=self.serve_cri, daemon=True),
                        threading.Thread(target=self.serve_position, daemon=True)]
        for thread in self.threads:
            thread.start()

    @staticmethod
    def listener():
        sock = socket.socket()
        sock.bind(("127.0.0.1", 0))
        sock.listen()
        sock.settimeout(.05)
        return sock

    @property
    def endpoint(self):
        return {"host": "127.0.0.1", "cri_port": self.cri_server.getsockname()[1]}

    def accept(self, server):
        while not self.closed.is_set():
            try:
                conn, _ = server.accept()
                conn.settimeout(.01)
                return conn
            except socket.timeout:
                continue
        return None

    def reply(self, category):
        self.cri.sendall(f"CRISTART 7 {category} CRIEND\n".encode())

    def interface(self):
        self.reply(f"CMD PositionInterface {self.pos_server.getsockname()[1]} "
                   f"{str(self.enabled).lower()} {str(self.in_use).lower()}")

    def serve_cri(self):
        try:
            self.cri = self.accept(self.cri_server)
            if not self.cri:
                return
            frames = Frames()
            last = 0
            while not self.closed.is_set():
                if time.monotonic() - last > .02:
                    if not self.mute_status:
                        self.reply("STATUS " + " ".join(status_tokens(self.error, self.kinstate, self.joints_deg)))
                    self.reply(f"RUNSTATE MAIN main.xml main.xml 0 0 {self.program_state} 0")
                    last = time.monotonic()
                try:
                    data = self.cri.recv(65536)
                except socket.timeout:
                    continue
                if not data:
                    return
                for tokens in frames.feed(data):
                    counter, category, *body = tokens
                    self.commands.append((category, *body))
                    cmd = " ".join(body)
                    if cmd == "GetActive":
                        self.reply(f"CMD Active {str(self.active).lower()}")
                    elif cmd == "GetVersion":
                        self.reply("INFO Version FakeRobot 15000")
                    elif cmd == "GetPositionInterface":
                        self.interface()
                    elif category == "CONFIG":
                        self.enabled = body[-1] == "true"
                        self.interface()
                    elif cmd.startswith("UsePositionInterface"):
                        if self.reject_use and body[-1] == "true":
                            self.reply(f"CMDERROR {counter} not_ready")
                        else:
                            self.in_use = body[-1] == "true"
                            self.reply(f"CMDACK {counter}")
                    elif category == "QUIT":
                        self.cri.close()  # like the real server: QUIT closes the connection
                        return
        except (OSError, ValueError):
            pass

    def serve_position(self):
        try:
            self.pos = self.accept(self.pos_server)
            if not self.pos:
                return
            frames = Frames("MSGSTART", "MSGEND")
            while not self.closed.is_set():
                values = " ".join(map(str, self.joints_deg))
                self.pos.sendall(f"MSGSTART Pos J {values} E 7 8 9 C 0 0 0 0 0 0 MSGEND\n".encode())
                try:
                    data = self.pos.recv(65536)
                except socket.timeout:
                    continue
                if not data:
                    self.in_use = False
                    return
                new = frames.feed(data)
                self.targets.extend(new)
                if self.follow and self.in_use and new:
                    self.joints_deg = [float(x) for x in new[-1][2:8]]
        except OSError:
            self.in_use = False

    def disconnect(self):
        if self.cri:
            self.cri.shutdown(socket.SHUT_RDWR)

    def close(self):
        self.closed.set()
        for sock in (self.cri, self.pos, self.cri_server, self.pos_server):
            if sock:
                try:
                    sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                sock.close()
        for thread in self.threads:
            thread.join(timeout=1)


class FramingTests(unittest.TestCase):
    def test_split_and_joined_frames(self):
        frames = Frames()
        self.assertEqual(frames.feed(b"CRISTA"), [])
        self.assertEqual(frames.feed(b"RT 1 CMD Active true CRIEND\nCRISTART 2 CMDACK 9 CRIEND"),
                         [["1", "CMD", "Active", "true"], ["2", "CMDACK", "9"]])

    def test_position_frame_without_newline(self):
        self.assertEqual(Frames("MSGSTART", "MSGEND").feed(b"MSGSTART Pos J 1 2 3 4 5 6 MSGEND")[0][0], "Pos")

    def test_status_rejects_nonfinite_and_missing_errors(self):
        for data in ("POSJOINTCURRENT nan 0 0 0 0 0 ESTOP 3 ERROR NoError 0 0 0 0 0 0 KINSTATE 0",
                     "POSJOINTCURRENT 0 0 0 0 0 0 ESTOP 3 KINSTATE 0"):
            with self.assertRaises(RobotError):
                parse_status(data.split())

    def test_motor_disabled_and_estop(self):
        self.assertIn("_MNE", parse_status(status_tokens(4, 99)).problem())
        tokens = status_tokens()
        tokens[tokens.index("ESTOP") + 1] = "0"
        self.assertIn("ESTOP", parse_status(tokens).problem())


class DualTransportTests(unittest.TestCase):
    def setUp(self):
        self.left, self.right = FakeRobot(), FakeRobot()
        self.pair = None

    def tearDown(self):
        if self.pair:
            self.pair.close()
        self.left.close()
        self.right.close()

    def connect(self, **kwargs):
        self.pair = EthernetArms({"left": self.left.endpoint, "right": self.right.endpoint}, **kwargs)
        self.pair.connect()

    def assert_pair_closed(self):
        eventually(lambda: all(a.cri is None and a.position is None for a in self.pair.arms.values()))

    def test_dry_run_never_configures_or_moves_even_when_motors_disabled(self):
        self.right.error, self.right.kinstate = 4, 99
        self.connect(dry_run=True)
        self.pair.send({"left": [0.001] * 6, "right": [0.002] * 6})
        report = self.pair.report()
        self.assertFalse(report["right"]["motion_ready"])
        self.assertTrue(report["left"]["motion_ready"])
        for robot in (self.left, self.right):
            self.assertFalse(robot.targets)
            self.assertTrue(all(command[0] in ("ALIVEJOG", "QUIT") or command[1].startswith("Get")
                                for command in robot.commands))

    def test_disabled_right_blocks_both_before_configuration(self):
        self.right.error, self.right.kinstate = 4, 99
        with self.assertRaisesRegex(RobotError, "right.*_MNE"):
            self.connect()
        self.assert_pair_closed()
        self.assertFalse(any(cmd[0] == "CONFIG" for cmd in self.left.commands))

    def test_passive_connection_is_not_stolen(self):
        self.right.active = False
        with self.assertRaisesRegex(RobotError, "passive"):
            self.connect()
        self.assert_pair_closed()
        self.assertFalse(any("SetActive" in cmd for cmd in self.right.commands))

    def test_running_program_blocks_startup(self):
        self.right.program_state = 2
        with self.assertRaisesRegex(RobotError, "program"):
            self.connect()
        self.assert_pair_closed()

    def test_distinct_sides_degrees_only_six_axes_and_release(self):
        self.connect()
        self.assertFalse(self.left.targets)
        self.assertFalse(self.right.targets)
        self.pair.send({"left": [0.001] * 6, "right": [-0.002] * 6})
        eventually(lambda: len(self.left.targets) >= 2 and len(self.right.targets) >= 2)
        self.assertAlmostEqual(float(self.left.targets[-1][2]), math.degrees(.001), places=7)
        self.assertAlmostEqual(float(self.right.targets[-1][2]), math.degrees(-.002), places=7)
        self.assertEqual(len(self.left.targets[-1]), 8)  # Pos J + six joints; no E / C / tool
        self.assertEqual(self.left.targets[0][2:], ["0.00000000"] * 6)
        self.pair.send({})
        eventually(lambda: not self.left.in_use and not self.right.in_use)
        self.assertFalse(self.pair.arms["left"].moving)

    def test_nonfinite_target_stops_both(self):
        self.connect()
        with self.assertRaises(RobotError):
            self.pair.send({"left": [float("nan")] * 6})
        self.assert_pair_closed()

    def test_jump_rejected_and_both_closed(self):
        self.connect()
        with self.assertRaisesRegex(RobotError, "speed limit"):
            self.pair.send({"left": [.1] * 6})
        self.assert_pair_closed()
        self.assertFalse(self.left.targets)

    def test_following_error_rejected(self):
        self.connect()
        with self.assertRaisesRegex(RobotError, "following error"):
            self.pair.send({"right": [1] * 6})
        self.assert_pair_closed()

    def test_no_input_watchdog_stops_pair_without_main_loop(self):
        self.connect(command_timeout=.12)
        self.pair.send({"left": [.001] * 6, "right": [.001] * 6})
        self.assert_pair_closed()
        eventually(lambda: self.pair.fault is not None)  # both links may close before the pair monitor records why
        self.assertIn("watchdog", self.pair.fault)

    def test_right_disconnect_stops_left(self):
        self.connect()
        self.pair.send({"left": [.001] * 6, "right": [.001] * 6})
        self.right.disconnect()
        self.assert_pair_closed()
        with self.assertRaises(RobotError):
            self.pair.send({"left": [.002] * 6})

    def test_stale_status_stops_pair(self):
        self.connect(state_timeout=.07, command_timeout=.4)
        self.pair.send({"left": [.001] * 6, "right": [.001] * 6})
        self.right.mute_status = True
        self.assert_pair_closed()
        eventually(lambda: self.pair.fault is not None)  # both links may close before the pair monitor records why
        self.assertIn("stale", self.pair.fault)

    def test_use_command_rejection_closes_both(self):
        self.connect()
        self.right.reject_use = True
        with self.assertRaisesRegex(RobotError, "not_ready"):
            self.pair.send({"left": [.001] * 6, "right": [.001] * 6})
        self.assert_pair_closed()

    def test_existing_position_source_is_not_taken_over(self):
        self.right.enabled = self.right.in_use = True
        with self.assertRaisesRegex(RobotError, "already in use"):
            self.connect()
        self.assert_pair_closed()
        self.assertFalse(any(cmd[:2] == ("CMD", "UsePositionInterface") for cmd in self.right.commands))

    def test_non_ascii_and_spaced_names_do_not_fault(self):
        frames = Frames()
        out = frames.feed("CRISTART 3 RUNSTATE MAIN Prüfung 1.xml sub prog.xml 12 3 0 2 CRIEND".encode("utf-8")[:30])
        out += frames.feed("CRISTART 3 RUNSTATE MAIN Prüfung 1.xml sub prog.xml 12 3 0 2 CRIEND".encode("utf-8")[30:])
        self.assertEqual(out[0][-2:], ["0", "2"])  # run state / replay mode read from the end
        self.connect()
        self.left.cri.sendall("CRISTART 9 RUNSTATE MAIN Prüfung 1.xml a b.xml 1 1 0 0 CRIEND\n".encode("utf-8"))
        time.sleep(.2)
        self.assertIsNone(self.pair.fault)
        self.assertEqual(self.pair.arms["left"].program_state, 0)

    def test_keyboard_interrupt_in_send_keeps_links_for_park(self):
        self.connect()
        arm = self.pair.arms["left"]
        real_target = arm.target

        def interrupted(q):
            raise KeyboardInterrupt
        arm.target = interrupted
        with self.assertRaises(KeyboardInterrupt):
            self.pair.send({"left": [.001] * 6})
        arm.target = real_target
        self.assertFalse(self.pair._closed.is_set())
        self.pair.send({"left": [.001] * 6})  # still usable (e.g. by park)

    def test_first_target_tolerates_small_drift_once(self):
        self.connect()
        self.left.joints_deg = [-0.3] * 6  # live pose moved 0.3 deg AGAINST the step since the snapshot (0)
        eventually(lambda: abs(self.pair.arms["left"].joints()[0][0] + math.radians(.3)) < 1e-9)
        self.pair.send({"left": [0.9 * 0.35 / 30] * 6})  # one full teleop step from the stale snapshot
        with self.assertRaisesRegex(RobotError, "speed limit"):
            self.pair.send({"left": [0.9 * 0.35 / 30 + math.radians(2)] * 6})  # later ticks: no extra tolerance

    def test_cleanup_commands_are_delivered(self):
        for _ in range(8):
            left, right = FakeRobot(), FakeRobot()
            pair = EthernetArms({"left": left.endpoint, "right": right.endpoint})
            pair.connect()
            pair.send({"left": [.001] * 6, "right": [.001] * 6})
            pair.close()
            for robot in (left, right):
                eventually(lambda: ("QUIT",) in robot.commands)
                self.assertIn(("CONFIG", "SetPositionInterface", "false"), robot.commands)
                self.assertIn(("CMD", "UsePositionInterface", "false"), robot.commands)
            left.close()
            right.close()


class ParkTests(unittest.TestCase):
    """Return to the park pose (default all joints 0) before the program exits."""
    tearDown = DualTransportTests.tearDown
    connect = DualTransportTests.connect

    def setUp(self):
        self.left, self.right = FakeRobot(), FakeRobot()
        self.pair = None
        self.left.joints_deg = [10, 20, 30, -10, 40, 5]
        self.right.joints_deg = [-15, 10, 60, 0, 30, -20]
        for robot in (self.left, self.right):
            robot.follow = True

    def test_park_brings_both_arms_to_zero_slowly_and_releases(self):
        self.connect(rate=30, max_joint_vel=0.35)
        self.assertTrue(self.pair.park([0.0] * 6, math.radians(30), log=lambda m: None))
        for robot in (self.left, self.right):
            self.assertTrue(all(abs(x) < 1e-6 for x in robot.joints_deg), robot.joints_deg)
            eventually(lambda: not robot.in_use)
            steps = [[float(x) for x in t[2:8]] for t in robot.targets]
            biggest = max(max(abs(a - b) for a, b in zip(p, q)) for p, q in zip(steps, steps[1:]))
            self.assertLessEqual(biggest, math.degrees(0.9 * 0.35 / 30) + 1e-6)  # 90 % of the per-tick limit
        self.assertIsNone(self.pair.fault)

    def test_guard_stops_park_and_holds(self):
        self.connect(rate=30)
        calls = []

        def guard(cmd):
            calls.append(cmd)
            return "too close" if len(calls) > 5 else None
        self.assertFalse(self.pair.park([0.0] * 6, math.radians(10), log=lambda m: None, guard=guard))
        eventually(lambda: not self.left.in_use and not self.right.in_use)
        self.assertGreater(abs(self.left.joints_deg[2]), 25)  # stopped near the start, not at 0

    def test_dry_run_park_sends_nothing(self):
        self.connect(dry_run=True)
        self.assertFalse(self.pair.park([0.0] * 6, math.radians(10), log=lambda m: None))
        self.assertFalse(self.left.targets or self.right.targets)


if __name__ == "__main__":
    unittest.main()
