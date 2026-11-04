"""Episode triggers for recording CUTTING runs (record_dataset.py --source moveit-real --episode-trigger ...).

Pure logic, standard library only (no rclpy / lerobot), so it is unit-tested without ROS. The recorder feeds it the
messages it receives and drains the events it emits:

  ("start", exec_id)                         an execution begins        -> open an episode
  ("end", exec_id, outcome, reason)          the execution is over      -> save it or discard it
        outcome "finished" = normal end; "stopped" = the coordinator STOPPED (or latched a stop) / the h-cut sequence
        failed or was aborted; "aborted" (dual-cut only) = the coordinator went back to a non-executing state without
        finishing a phase (barrier or goal failure, or an unexpected state).

dual-cut  /dual_cut_coordinator/status (dual_cut_msgs/CoordinatorStatus) + /dual_cut_coordinator/exec_report (JSON)
  start    the first WAITING_BARRIER or EXECUTING status after a status that was not executing. A recorder that starts
           while an execution is already running does NOT join it (it waits for the next one).
  end      STOPPED or stop_latched            -> "stopped"  (reason = stop_reason)
           PLANNED  (finishExecution)         -> "finished" if every phase entered (status phase while executing) has a
                                                 report with completed=true and none has completed=false, else
                                                 "aborted" after END_GRACE_S (e.g. phase B barrier failure after A)
           IDLE / PLANNING                    -> "aborted"
  One episode spans every phase (A, barrier, B) of one ~/execute call.
  observation.cut_progress = [coordinator state code, phase index]: state 0 IDLE 1 PLANNING 2 PLANNED 3 WAITING_BARRIER
  4 EXECUTING 5 STOPPED; phase 0 none, 1 "A", 2 "B".

h-cut     /box_h_cut_executor/t_cut_status (std_msgs/String JSON {step_index, step, status, message, stamp_ns})
  start    {step_index 0, step "sequence", status "running"} (execute_t_cut dispatch)
  end      {step "sequence", status "done"} (all 4 steps)  -> "finished"
           any {status "failed"} (a step failed, was refused, or abort_t_cut)  -> "stopped"
  observation.cut_progress = [step index, status code]: step 0 sequence start, 1 approach_standoff,
  2 side_cut_penetrate, 3 continue_side_cut, 4 continue_top_cut, 5 sequence done; status 0 none, 1 running, 2 done,
  3 failed.

fjt-status  action_msgs/GoalStatusArray of the single-arm FollowJointTrajectory server (every executed trajectory, like the
          passive tools/cut_recorder of igus_rebel_ros2; the h-cut t_cut_status only covers execute_t_cut)
  start    any goal ACCEPTED(1) / EXECUTING(2)
  end      no goal ACCEPTED/EXECUTING for post_roll_s (back-to-back trajectories of one cut stay one episode)
           -> "finished"; a goal of the episode that ended CANCELED(5) / ABORTED(6) -> "stopped"
  observation.cut_progress = [goals active now, status of the newest goal of the episode: 0 none 1 accepted 2 executing
  3 canceling 4 succeeded 5 canceled 6 aborted].

Silence of the status topic (> SILENCE_S) while an execution is open is only logged: the coordinator status is
low-rate and an open execution is ended by a message, never by a timeout.
"""

import json
import threading
import time

END_GRACE_S = 0.5    # PLANNED seen but no completed phase report yet: wait this long for the report in flight
SILENCE_S = 2.0

DUAL_STATE_NAMES = {0: "IDLE", 1: "PLANNING", 2: "PLANNED", 3: "WAITING_BARRIER", 4: "EXECUTING", 5: "STOPPED"}
DUAL_EXECUTING = (3, 4)
DUAL_STOPPED, DUAL_PLANNED = 5, 2
PHASE_INDEX = {"": 0, "A": 1, "B": 2}
HCUT_STATUS_CODE = {"running": 1, "done": 2, "failed": 3}
FJT_ACTIVE = (1, 2)
FJT_STOPPED = {5: "CANCELED", 6: "ABORTED"}
FJT_NAMES = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
FEATURE_NAMES = {"dual-cut": ["coordinator_state", "phase"], "h-cut": ["step_index", "status"],
                 "fjt-status": ["active_goals", "last_goal_status"]}
ROBOT_TYPE = {"dual-cut": "igus_rebel_dual_arm_real_cut", "h-cut": "igus_rebel_single_arm_real_cut",
              "fjt-status": "igus_rebel_single_arm_real_cut"}
DEFAULT_TOPIC = {"dual-cut": "/dual_cut_coordinator/status", "h-cut": "/box_h_cut_executor/t_cut_status",
                 "fjt-status": "/rebel_arm_trajectory_controller/follow_joint_trajectory/_action/status"}
SINGLE_ARM_TRIGGERS = ("h-cut", "fjt-status")
DEFAULT_REPORT_TOPIC = "/dual_cut_coordinator/exec_report"
DEFAULT_TASK = "cut the carton tape"


class _Base:
    def __init__(self, emit, log=print, clock=time.monotonic):
        self._emit, self._log, self._clock = emit, log, clock
        self._lock = threading.RLock()
        self.running = False
        self.exec_id = 0
        self.last_rx = None
        self._silence_logged = False
        self._progress = [0.0, 0.0]
        self.ended_info = {}            # exec_id -> info() at its end (for the sidecar clip.json)

    log_silence = True

    def info(self):
        """What the sidecar clip.json records about the current / just ended execution."""
        return {}

    def progress(self):
        with self._lock:
            return list(self._progress)

    def _rx(self):
        self.last_rx = self._clock()
        self._silence_logged = False

    def _start(self, why):
        self.exec_id += 1
        self.running = True
        self._log(f"[rec] cut trigger: execution {self.exec_id} started ({why})")
        self._emit(("start", self.exec_id))

    def _end(self, outcome, reason):
        self.running = False
        self._log(f"[rec] cut trigger: execution {self.exec_id} over: {outcome}" + (f" ({reason})" if reason else ""))
        self.ended_info[self.exec_id] = self.info()
        self._emit(("end", self.exec_id, outcome, reason))

    def poll(self, now=None):
        """Called from the recorder loop: log (never end) a long silence of the status topic."""
        now = self._clock() if now is None else now
        with self._lock:
            if self.log_silence and self.running and self.last_rx is not None and not self._silence_logged and now - self.last_rx > SILENCE_S:
                self._silence_logged = True
                self._log(f"[rec] cut trigger: no status message for {now - self.last_rx:.1f}s while execution "
                          f"{self.exec_id} is open (status is low-rate); the episode stays open")
            self._poll_locked(now)

    def _poll_locked(self, now):
        pass


class DualCutTrigger(_Base):
    """duck-typed status: .state .phase .stop_latched .stop_reason (dual_cut_msgs/msg/CoordinatorStatus)."""

    def __init__(self, emit, log=print, clock=time.monotonic):
        super().__init__(emit, log, clock)
        self.prev_state = None
        self.ignore_running = False     # recorder started during an execution: wait for a non-executing status
        self.reports = []               # (phase, completed) of this execution
        self.report_docs = []           # the exec_report JSON documents of this execution
        self.phases_entered = set()     # phases the coordinator announced (WAITING_BARRIER / EXECUTING) in this execution
        self.stop_reason_seen = ""
        self._pending_end = None        # deadline: PLANNED seen, waiting for the report of the last phase

    def on_status(self, msg):
        with self._lock:
            self._rx()
            state = int(msg.state)
            self._progress = [float(state), float(PHASE_INDEX.get(msg.phase, 0))]
            if msg.stop_reason:
                self.stop_reason_seen = msg.stop_reason
            executing = state in DUAL_EXECUTING
            if not self.running:
                self.phases_entered = set()
                if executing:
                    if self.prev_state is None:
                        self.ignore_running = True
                        self._log(f"[rec] cut trigger: the coordinator is already {DUAL_STATE_NAMES[state]} - not "
                                  "joining this execution; the next one is recorded")
                    elif not self.ignore_running:
                        self.reports, self.report_docs, self.stop_reason_seen, self._pending_end = [], [], msg.stop_reason, None
                        self._start(f"coordinator {DUAL_STATE_NAMES[state]} phase {msg.phase or '-'}")
                else:
                    self.ignore_running = False
            else:
                if state == DUAL_STOPPED or msg.stop_latched:
                    self._pending_end = None
                    self._end("stopped", msg.stop_reason or self.stop_reason_seen or "unknown")
                elif state == DUAL_PLANNED:
                    if self._pending_end is None:
                        self._pending_end = self._clock() + END_GRACE_S
                    self._resolve_planned()
                elif not executing:
                    self._pending_end = None
                    self._end("aborted", f"coordinator went to {DUAL_STATE_NAMES.get(state, state)} without finishing")
            if self.running and executing and msg.phase:
                self.phases_entered.add(msg.phase)
            self.prev_state = state

    def on_report(self, data):
        """exec_report JSON (one per phase, transient_local: ignored while no execution is open)."""
        try:
            d = json.loads(data)
        except (ValueError, TypeError):
            return
        with self._lock:
            if not self.running or not isinstance(d, dict):
                return
            self.reports.append((str(d.get("phase", "")), bool(d.get("completed", False))))
            self.report_docs.append(d)
            if d.get("stop_reason"):
                self.stop_reason_seen = str(d["stop_reason"])
            self._log(f"[rec] cut trigger: phase {d.get('phase')} report completed={d.get('completed')} "
                      f"wall_s={d.get('wall_s')}")
            if self._pending_end is not None:
                self._resolve_planned()

    def info(self):
        with self._lock:
            return {"exec_reports": list(self.report_docs), "phases_entered": sorted(self.phases_entered),
                    "stop_reason": self.stop_reason_seen}

    def _resolve_planned(self):
        """PLANNED: finished if every phase the coordinator entered has a completed report (a phase-B barrier or goal
        failure after a completed phase A returns to PLANNED without a B report). Waits (END_GRACE_S) for a report
        still in flight."""
        done = {p for p, c in self.reports if c}
        if self.reports and all(c for _, c in self.reports) and self.phases_entered <= done:
            self._pending_end = None
            self._end("finished", "phases " + "+".join(p for p, _ in self.reports))
        elif any(not c for _, c in self.reports):
            self._pending_end = None
            self._end("aborted", "a phase report says completed=false")

    def _poll_locked(self, now):
        if self.running and self._pending_end is not None and now >= self._pending_end:
            self._pending_end = None
            self._end("aborted", "coordinator is PLANNED again but no phase completed (barrier / goal failure)")


class HCutTrigger(_Base):
    """t_cut_status JSON string messages."""

    _last = None

    def info(self):
        with self._lock:
            return {"last_t_cut_status": self._last}

    def on_status(self, data):
        try:
            d = json.loads(data)
            idx, step, status = int(d["step_index"]), str(d["step"]), str(d["status"])
        except (ValueError, TypeError, KeyError):
            self._log(f"[rec] cut trigger: unparsable t_cut_status {str(data)[:120]!r}")
            return
        message = str(d.get("message", ""))
        with self._lock:
            self._last = d
            self._rx()
            self._progress = [float(idx), float(HCUT_STATUS_CODE.get(status, 0))]
            if step == "sequence" and status == "running" and idx == 0:
                if self.running:
                    self._end("stopped", "sequence restarted before the previous one ended")
                self._start("T cut sequence dispatched")
            elif self.running:
                if step == "sequence" and status == "done":
                    self._end("finished", message)
                elif status == "failed":
                    self._end("stopped", f"{step}: {message}"[:200])


class FjtStatusTrigger(_Base):
    """action_msgs/GoalStatusArray (duck-typed: .status_list[].goal_info.goal_id.uuid, .status)."""

    log_silence = False        # an action status topic only publishes on changes

    def __init__(self, emit, log=print, clock=time.monotonic, post_roll_s=5.0):
        super().__init__(emit, log, clock)
        self.post_roll_s = float(post_roll_s)
        self.goals = {}               # goal id (hex) -> last status, goals that were active in this execution
        self.n_active = 0
        self.last_active = 0.0

    def on_status(self, msg):
        every = {bytes(s.goal_info.goal_id.uuid).hex(): int(s.status) for s in msg.status_list}
        active = {g: st for g, st in every.items() if st in FJT_ACTIVE}
        with self._lock:
            self._rx()
            now = self._clock()
            self.n_active = len(active)
            if active:
                self.last_active = now
                if not self.running:
                    self.goals = {}
                    self._start(f"{len(active)} goal(s) active")
            if self.running:
                for g, st in active.items():
                    self.goals[g] = st
                for g in self.goals:      # goals first seen active keep following their status
                    if g in every:
                        self.goals[g] = every[g]
            last = list(self.goals.values())[-1] if self.goals else 0
            self._progress = [float(self.n_active), float(last)]

    def info(self):
        with self._lock:
            return {"goal_last_status": {g[:8]: FJT_NAMES.get(st, str(st)) for g, st in self.goals.items()}}

    def _poll_locked(self, now):
        if self.running and self.n_active == 0 and now - self.last_active >= self.post_roll_s:
            bad = {g: st for g, st in self.goals.items() if st in FJT_STOPPED}
            if bad:
                g, st = next(iter(bad.items()))
                self._end("stopped", f"goal {g[:8]} {FJT_STOPPED[st]}")
            else:
                self._end("finished", f"{len(self.goals)} goal(s)")


def make_trigger(kind, emit, log=print, clock=time.monotonic, post_roll_s=5.0):
    if kind == "fjt-status":
        return FjtStatusTrigger(emit, log, clock, post_roll_s)
    return {"dual-cut": DualCutTrigger, "h-cut": HCutTrigger}[kind](emit, log, clock)


def task_with_suffix(task, outcome, reason):
    """--keep-stopped: the task text of a kept episode that did not finish normally."""
    word = "stopped" if outcome == "stopped" else "aborted"
    return f"{task} [{word}: {reason or 'unknown'}]"
