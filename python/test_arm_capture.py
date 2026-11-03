"""
Tests for python/arm_capture.py session, with no arm and no camera.

  .venv/bin/python python/test_arm_capture.py

A simulation stands in for both. The camera rides on the tool (8 cm ahead of
the TCP, pinhole 780 px, 1280 x 1024) and the 12 x 9 board stands on an easel
1.25 m away. A stand-in for 04_capture --auto (gates 1-3, 5, 6, coverage,
pose bins) writes real status files. The fake arm moves instantly, asserts
every target is inside config/arm.yaml's box, and has a fake IK service. The
probe runs on the simulation first, so the session uses gains measured the
same way as on hardware.

The regression cases replay tests/fixtures/session_arm_33664_20261028_185017.json:
the first arm-driven session on 33664, which ended on ACTION_ABORT
(INVALID_PARAM) at its sixth target; and ..._191025.json, whose bottom-right
corner was never reached; and the two taught sessions of 2026-11-02,
..._150512.json (15:05, 28 accepts) and ..._175213.json (17:52, ended after
10 accepts on "board not back in view after returning from a skipped
target"), replayed through the visit order and the reach-edge rule. Exit 0
only if every check passes.

The aiming and orbit cases also run on a rig-like geometry, use_rig(): optical
axis 12 deg off tool z, board tilted 23 deg, an effective lever fitted to the
recorded centroids of ..._191025 (FINDINGS §20). Without that lever a rotation
about the TCP is nearly a rotation about the camera, and the aim is too easy.
"""

import datetime, json, math, os, re, sys, tempfile, threading, time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm as armlib
import arm_capture as ac
import capture_status as cs

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tests", "fixtures")
FIXTURE = os.path.join(FIXTURES, "session_arm_33664_20261028_185017.json")
CORNER_FIXTURE = os.path.join(FIXTURES, "session_arm_33664_20261028_191025.json")
TAUGHT_1505 = os.path.join(FIXTURES, "session_arm_33664_20261102_150512.json")
TAUGHT_1752 = os.path.join(FIXTURES, "session_arm_33664_20261102_175213.json")

ok = fail = 0


def check(cond, what):
    global ok, fail
    print(("  PASS  " if cond else "  FAIL  ") + what, flush=True)
    ok += bool(cond)
    fail += not cond


# ---------------------------------------------------------------- simulation

STATUS_TEMPLATE = json.load(open(cs.FIXTURE))
W, H, F = 1280, 1024, 780.0
CAM_OFFSET = np.array([0, 0, 0.08])                  # camera 8 cm ahead of the TCP, tool z
CAM_R = np.eye(3)                                    # camera axes in the tool frame
BOARD_C = np.array([1.78, 0.0, 0.45])                # easel; faces -x
BX, BY = np.array([0, -1.0, 0]), np.array([0, 0, -1.0])
CORNERS = np.array([BOARD_C + (c - 5) * 0.06 * BX + (r - 3.5) * 0.06 * BY
                    for r in range(8) for c in range(11)])
BOARD_N = np.array([-1.0, 0, 0])
R0 = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], float)   # tool x = -y, y = -z, z = +x
P0 = (0.45, 0.0, 0.45) + ac.R_to_euler(R0)
EASEL = (CAM_OFFSET, CAM_R, CORNERS, BOARD_N)

# Rig-like: start pose, camera axes and board pose as measured in session
# 191025 (PnP on frame_0001, rotations of 13 frames against the arm's).
RIG_P0 = (0.54952, -0.0301, 0.48042, 84.21268, -1.56856, 95.85246)
RIG_CAM_R = np.array([[-1.0, 0.0082, -0.0002], [-0.008, -0.9779, -0.209], [-0.0019, -0.209, 0.9779]])
RIG_LEVER = np.array([-0.3064, -0.3987, -0.1271])    # effective, not a mount offset
RIG_R_CB = np.array([[-0.0516, -0.9198, 0.389], [0.9963, -0.0209, 0.0828], [-0.068, 0.3918, 0.9175]])
RIG_T_CB = np.array([0.2622, -0.3939, 1.3771])


def use_geometry(geometry):
    global CAM_OFFSET, CAM_R, CORNERS, BOARD_N
    CAM_OFFSET, CAM_R, CORNERS, BOARD_N = geometry


def use_rig():
    R = ac.euler_to_R(*RIG_P0[3:])
    Rc, cam = R @ RIG_CAM_R, np.array(RIG_P0[:3]) + R @ RIG_LEVER
    board = np.array([[(c + 1) * 0.06, (r + 1) * 0.06, 0] for r in range(8) for c in range(11)])
    use_geometry((RIG_LEVER, RIG_CAM_R, cam + (Rc @ (RIG_T_CB + board @ RIG_R_CB.T).T).T,
                  -Rc @ RIG_R_CB[:, 2]))


def view(pose):
    """Projected corners, which are inside the image, tilt (x, y, total),
    apparent square size and roll, as lib/autocapture computes them."""
    R = ac.euler_to_R(*pose[3:])
    cam = np.array(pose[:3]) + R @ CAM_OFFSET
    R = R @ CAM_R
    q = (CORNERS - cam) @ R
    uv = np.c_[W / 2 + F * q[:, 0] / q[:, 2], H / 2 + F * q[:, 1] / q[:, 2]]
    inside = (q[:, 2] > 0) & (uv[:, 0] > 2) & (uv[:, 0] < W - 2) & (uv[:, 1] > 2) & (uv[:, 1] < H - 2)
    n = R.T @ BOARD_N
    n = -n if n[2] > 0 else n
    tx, ty = math.degrees(math.atan2(n[1], -n[2])), math.degrees(math.atan2(n[0], -n[2]))
    tilt = math.degrees(math.acos(min(1, -n[2])))
    gr = uv.reshape(8, 11, 2)
    h = np.median((gr[:, 1:] - gr[:, :-1]).reshape(-1, 2), 0)
    v = np.median((gr[1:] - gr[:-1]).reshape(-1, 2), 0)
    t = h @ h + v @ v
    det = h[0] * v[1] - v[0] * h[1]
    sq = math.sqrt((t + math.sqrt(max(0, t * t - 4 * det * det))) / 2)
    return uv, inside, (tx, ty, tilt), sq, math.degrees(math.atan2(h[1], h[0]))


def fake_joints(pose):
    """A smooth stand-in for the arm's joint angles at a pose."""
    return [pose[3], pose[4], pose[5] / 2, pose[0] * 100, 60 + pose[1] * 100, pose[2] * 100]


def fake_fk(q):
    """fake_joints() inverted."""
    return (q[3] / 100, (q[4] - 60) / 100, q[5] / 100, armlib.wrap180(q[0]), armlib.wrap180(q[1]),
            armlib.wrap180(q[2]) * 2)


class FakeArm:
    """abort_at: these 1-based move numbers return ACTION_ABORT INVALID_PARAM.
    ik: optional function(pose) -> joints, or raises armlib.IKFailed;
    seeded_ik: the same, as function(pose, guess)."""

    def __init__(self, pose, abort_at=(), ik=None, seeded_ik=None):
        self.pose = list(pose)
        self.calls, self.ik_calls, self.joint_calls, self.joint_speeds = [], 0, [], []
        self.stops, self.joint_abort_at = 0, set()
        self.last_abort = self.last_move_s = None
        self.joints_override = None
        self.branch = 0.0         # joint 3's offset from fake_joints(): 0 or 180 (another branch)
        self.unsettled = 0
        self.hidden = False       # Capture shows 87/88 until the next joint move
        self.abort_at, self.ik, self.seeded_ik = set(abort_at), ik, seeded_ik

    def measured_pose(self):
        return tuple(self.pose)

    def measured_joints(self):
        if self.joints_override:
            return list(enumerate(self.joints_override))
        q = fake_joints(self.pose)
        q[2] += self.branch       # a Cartesian move keeps the branch, as a real arm does
        return list(enumerate(q))

    def forward_kinematics(self, joints):
        return fake_fk(joints)

    def stop(self):
        self.stops += 1

    def reach_joints(self, joints, lim, dps=None):
        self.joint_calls.append(list(joints))
        self.joint_speeds.append(dps)
        assert not armlib.outside_box(lim, fake_fk(joints)), f"OUTSIDE BOX {joints}"
        self.joints_override = None
        if len(self.joint_calls) in self.joint_abort_at:
            return "abort", "INVALID_PARAM"
        self.pose = list(fake_fk(joints))
        self.hidden = False
        self.branch = 180.0 * round(armlib.wrap180(joints[2] - fake_joints(self.pose)[2]) / 180.0) % 360
        self.unsettled = 3
        return "end", None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def servoing_mode(self):
        return "SINGLE_LEVEL_SERVOING"

    def inverse_kinematics(self, pose, guess):
        self.ik_calls += 1
        if self.seeded_ik:
            return self.seeded_ik(pose, guess)
        return (self.ik or fake_joints)(pose)

    def reach_pose(self, target, lim):
        self.calls.append(tuple(target))
        assert not armlib.outside_box(lim, target), f"OUTSIDE BOX {target}"
        if len(self.calls) in self.abort_at:
            # Stops part way, as a real abort would.
            self.pose = list(ac.interpolate(tuple(self.pose), tuple(target), 0.4))
            self.unsettled = 3
            return "abort", "INVALID_PARAM"
        before = dict(self.measured_joints())[2]
        self.pose = list(target)
        # fake_joints() halves theta_z into joint 3, so a wrapped Euler angle
        # reads as a 180 deg jump; a real Cartesian move stays on its branch.
        base = fake_joints(self.pose)[2]
        self.branch = min((0.0, 180.0), key=lambda b: abs(armlib.wrap180(base + b - before)))
        self.unsettled = 3
        return "end", None


class FakeStub:
    """The Kortex BaseClient as Arm sees it: records requests, answers FK, and
    fires ACTION_END from ExecuteAction (ACTION_START then ACTION_ABORT
    METHOD_FAILED, with a trajectory error report, while abort is set)."""

    def __init__(self, pose):
        self.pose, self.executed, self.notify, self.stopped = pose, [], None, 0
        self.abort = False

    def GetTrajectoryErrorReport(self, options=None):
        r = armlib.Base_pb2.TrajectoryErrorReport()
        e = r.trajectory_error_elements.add()
        e.error_type = armlib.Base_pb2.TRAJECTORY_ERROR_TYPE_INVALID_JOINT_SPEED
        e.error_identifier = armlib.Base_pb2.TRAJECTORY_ERROR_IDENTIFIER_VELOCITY
        e.error_value, e.min_value, e.max_value, e.index = 10.0, 0.0, 1.0, 2
        e.message = "fake"
        return r

    def ComputeForwardKinematics(self, q, options=None):
        self.fk_request = q
        x, y, z, tx, ty, tz = self.pose
        return armlib.Base_pb2.Pose(x=x, y=y, z=z, theta_x=tx, theta_y=ty, theta_z=tz)

    def OnNotificationActionTopic(self, callback, _, options=None):
        self.notify = callback
        return "handle"

    def Unsubscribe(self, handle, options=None):
        pass

    def Stop(self, options=None):
        self.stopped += 1

    def ExecuteAction(self, action, options=None):
        self.executed.append(action)
        B = armlib.Base_pb2
        if self.abort:
            self.notify(B.ActionNotification(action_event=B.ACTION_START))
            n = B.ActionNotification(action_event=B.ACTION_ABORT,
                                     abort_details=armlib.Errors_pb2.METHOD_FAILED)
            t = n.trajectory_info.add()
            t.trajectory_info_type, t.joint_index = B.JOINT_SPEED_LIMIT_REACHED, 2
            self.notify(n)
            return
        self.notify(B.ActionNotification(action_event=B.ACTION_END))


class Capture(threading.Thread):
    """Stand-in for 04_capture --auto: gates 1-3, 5, 6; coverage; bins; status file."""

    def __init__(self, arm, path, dry, session_path=None, interval_ms=50, die_after=None):
        super().__init__(daemon=True)
        self.arm, self.path, self.dry, self.sp = arm, path, dry, session_path
        self.interval, self.die_after = interval_ms / 1000, die_after
        self.frame, self.halt, self.prev_full = 0, False, False
        self.cells = np.zeros(30, int)
        self.bins = {f"{b}/{t}": 0 for b in ("near", "mid", "far")
                     for t in ("flat", "+x+y", "+x-y", "-x+y", "-x-y")}
        self.accepted, self.last_t, self.last_uv = 0, -1e9, None

    def run(self):
        while not self.halt:
            self.frame += 1
            if self.die_after and self.frame > self.die_after:
                return
            uv, inside, (tx, ty, tilt), sq, roll = view(self.arm.pose)
            if getattr(self.arm, "hidden", False):  # one corner out, 87/88, as at 17:52 entry 51
                inside = inside.copy()
                inside[0] = False
            full = bool(inside.all())
            moving = self.arm.unsettled > 0
            if moving:
                self.arm.unsettled -= 1
            still = full and self.prev_full and not moving
            self.prev_full = full and not moving
            tb = "flat" if tilt < 20 else ("+x" if tx >= 0 else "-x") + ("+y" if ty >= 0 else "-y")
            band = "near" if sq >= 40 else "far" if sq < 35 else "mid"
            bname = f"{band}/{tb}"
            ci = (np.clip(uv[inside, 1] * 6 // H, 0, 5) * 5 +
                  np.clip(uv[inside, 0] * 5 // W, 0, 4)).astype(int)
            new = full and any(self.cells[c] == 0 for c in ci)
            short = self.bins[bname] < 2
            now = time.monotonic()
            if not full:
                verdict = "not_full"
            elif not still:
                verdict = "moving"
            elif not (new or short):
                verdict = "redundant"
            elif now - self.last_t < self.interval:
                verdict = "too_soon"
            elif self.last_uv is not None and np.max(np.linalg.norm(uv - self.last_uv, axis=1)) < 20:
                verdict = "same_pose"
            else:
                verdict = "accept"
                self.accepted += 1
                self.last_t, self.last_uv = now, uv.copy()
                for c in ci:
                    self.cells[c] += 1
                self.bins[bname] += 1
            empty = [int(i) for i in np.flatnonzero(self.cells == 0)]
            complete = not empty and all(n >= 2 for n in self.bins.values())
            s = json.loads(json.dumps(STATUS_TEMPLATE))
            s.update(frame=self.frame, dry_run=self.dry, session_path=self.sp,
                     accepted=self.accepted, complete=complete,
                     timestamp_utc=datetime.datetime.now(datetime.timezone.utc)
                     .strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z")
            n_in = int(inside.sum())
            s["corners"] = {"found": n_in, "total": 88, "complete": full}
            s["markers"] = {"found": 54 if full else n_in * 54 // 88, "total": 54}
            s["board"] = None if n_in == 0 else {
                "centroid_px": uv[inside].mean(0).tolist(),
                "bbox_px": [*uv[inside].min(0).tolist(), *uv[inside].max(0).tolist()],
                "square_px": sq, "roll_deg": roll}
            s["motion"]["still"] = still
            s["decision"] = {"verdict": verdict, "reason": verdict, "bin": bname if full else "none"}
            s["coverage"]["empty_cells"] = empty
            s["coverage"]["empty_count"] = len(empty)
            s["pose_bins"]["counts"] = dict(self.bins)
            s["pose_bins"]["short"] = sum(n < 2 for n in self.bins.values())
            with open(self.path + ".tmp", "w") as f:
                f.write(json.dumps(s))
            os.replace(self.path + ".tmp", self.path)
            if complete:
                return                      # capture exits on completion
            time.sleep(0.01)


# This arm's joint limits, Web App Safeties -> Actuators, 2026-11-02.
WEB_APP_LIMITS = {1: "continuous", 2: (-129.0, 138.0), 3: (-148.0, 152.0), 4: "continuous",
                  5: (-120.0, 128.0), 6: "continuous"}
UNCONFIRMED = {a: None for a in range(1, 7)}


def limit_config_checks(lim, d):
    """config/arm.yaml joint limits: parsing, asymmetric refusals, the null (warn) path."""
    print("joint limits: config/arm.yaml, [low, high] refuses, null warns")
    check(lim["joint_limits_deg"] == WEB_APP_LIMITS,
          "config/arm.yaml: the Web App's limits, actuators 1, 4, 6 continuous")
    src = open(armlib.LIMITS).read()

    def load(text):
        path = os.path.join(d, "arm_limits.yaml")
        with open(path, "w") as f:
            f.write(text)
        try:
            return armlib.load_limits(path)["joint_limits_deg"]
        except armlib.ArmError as e:
            return str(e)

    got = load(src.replace('joint_4_limit_deg: "continuous"', "joint_4_limit_deg: null"))
    check(isinstance(got, dict) and got[4] is None and got[2] == (-129.0, 138.0),
          "[low, high], \"continuous\" and null each load")
    check("joint_6_limit_deg is missing" in load(src.replace('joint_6_limit_deg: "continuous"\n', "")),
          "a missing joint limit is refused, by key")
    for bad, what in (("[ 138.0, -129.0 ]", "low above high"), ("[ -129.0, 200.0 ]", "past 180"),
                      ("138.0", "a single number")):
        check("joint_2_limit_deg" in str(load(src.replace("[ -129.0, 138.0 ]", bad))),
              f"a joint limit with {what} is refused")

    q = [169.979, 230.559, 199.779, 185.307, 239.964, 87.952]     # status, 2026-11-02
    refuse, warn = armlib.joint_limit_checks(lim, q)
    check(not warn and [r.split()[1] for r in refuse] == ["2", "3", "5"],
          "the pose status showed, resting on the stops, is outside joints 2, 3 and 5")
    q2 = [0, -132.0, 0, 0, 0, 0]
    check(armlib.joint_limit_checks(lim, q2)[0] and not armlib.joint_limit_checks(lim, [-v for v in q2])[0],
          "limits are asymmetric: joint 2 at -132 refused, +132 accepted")
    refuse, warn = armlib.joint_limit_checks(dict(lim, joint_limits_deg=UNCONFIRMED), q, 10.0)
    check(not refuse and len(warn) == 1 and "3 at -160.2" in warn[0],
          "null limits warn with the angle, never refuse: " + (warn[0] if warn else "no warning"))
    refuse, _ = ac.joint_margin(lim, [0, 0, 0, 0, 120.0, 0])
    check(refuse and "need 10" in refuse[0], "teach margin: joint 5 at +120, 8 deg from +128, refused")


def main():
    lim = armlib.load_limits()
    d = tempfile.mkdtemp(prefix="panoptes_session_")
    st, gains = os.path.join(d, "status.json"), os.path.join(d, "arm_camera.yaml")
    sess = os.path.join(d, "intrinsics_99999")
    os.makedirs(sess)
    quiet = lambda *a: None
    acfg = dict(ac.load_autocapture_numbers(), min_interval_ms=50.0)
    scfg = ac.load_session_config()
    ac.STILL_TIMEOUT_S = 3.0

    def clear():
        for f in os.listdir(sess):
            os.remove(os.path.join(sess, f))

    def session(arm, cap, yes=True, keys=None, log=quiet):
        cap.start()
        time.sleep(0.1)
        try:
            return ac.run_session(arm, lim, st, yes, gains_path=gains, scfg=scfg, acfg=acfg,
                                  keys=keys, log=log, mode="orbit"), None
        except (ac.ProbeError, armlib.ArmError) as e:
            return None, e
        finally:
            cap.halt = True
            cap.join()

    def record():
        return json.load(open(os.path.join(sess, ac.SESSION_RECORD)))

    try:
        limit_config_checks(lim, d)
        # ------------------------------------------------ the recorded abort
        print("regression: tests/fixtures/session_arm_33664_20261028_185017.json")
        rec = json.load(open(FIXTURE))
        aborted = [p for p in rec["poses"] if p["outcome"] is None and "measured" not in p]
        a = aborted[0] if aborted else {}
        check(len(aborted) == 1 and a["i"] == 5 and a["cell"] == 2
              and rec["end"]["end"] == "arm failure: ABORT (INVALID_PARAM)",
              "the aborted target is entry 5 (cell 2); the session ended on ABORT (INVALID_PARAM)")
        cmd = a.get("commanded", [{}])[-1]
        check(cmd.get("abs") == [0.54954, -0.03009, 0.4805, 102.51697, -1.67953, 97.78914]
              and cmd["rel_p0"]["t_m"] == [0.0, 0.0, 0.0],
              "its pose: P0's position, theta 102.517/-1.680/97.789 deg (pure rotation)")
        prev = rec["poses"][4]["commanded"][-1]["abs"]
        dr = ac.relative(tuple(prev), tuple(cmd["abs"]))["rotvec_deg"]
        check(abs(np.linalg.norm(dr) - 16.6) < 0.5,
              f"it rotates {np.linalg.norm(dr):.1f} deg from the previous target, mainly about "
              f"tool y ({dr[1]:+.1f})")
        sent_before = [c for p in rec["poses"][:5] for c in p["commanded"]]
        check(all(abs(c["abs"][4]) < 5 for c in sent_before + [cmd]) and
              rec["config"]["limits"]["max_rotation_dps"] == 10.0,
              "theta_y within +-5 deg on all; same speed constraint as the 7 moves before it")
        t = ac.timing_breakdown(rec["poses"], rec["end"]["total_s"], rec["end"]["moves_commanded"])
        check(t["fractions"]["move_s"] > 0.85 and t["fractions"]["still_s"] < 0.06,
              f"time: moving {t['fractions']['move_s']:.0%}, waiting for still "
              f"{t['fractions']['still_s']:.0%}, {t['per_move_s']:.1f} s per move")
        sq0 = rec["p0"]["board"]["square_px"]
        k_z = (rec["variants"][5]["predicted_square_px"] - sq0) / rec["config"]["session"]["depth_m"]
        g_rec = {"square_per_m": np.array([0.0, 0.0, k_z])}
        vs = [{"name": f"{ac._band_of(v['predicted_square_px'], sq0, 0.08)}(x)/centre",
               "band": ac._band_of(v["predicted_square_px"], sq0, 0.08),
               "depth": v["name"].split("/")[0], "predicted_square_px": v["predicted_square_px"]}
              for v in rec["variants"] if v["name"].endswith("/centre")]
        warns = ac.depth_sanity(vs, sq0, g_rec, dict(scfg, depth_m=0.10))
        check({v["band"] for v in vs} == {"mid"} and len(warns) == 2 and "12.8%" in warns[0],
              f"relative bands: depth 0.10 m keeps all three in mid ({k_z:.1f} px/m on "
              f"{sq0:.1f} px); both sanity warnings fire")

        # ------------------------------------------------ the taught sessions of 2026-11-02
        print("\nregression: tests/fixtures/session_arm_33664_20261102_{150512,175213}.json")
        recs = {"15:05": json.load(open(TAUGHT_1505)), "17:52": json.load(open(TAUGHT_1752))}
        r52 = recs["17:52"]
        last = r52["poses"][51]
        check(r52["end"]["end"] == "board not back in view after returning from a skipped target"
              and r52["end"]["accepted"] == 10 and r52["end"]["unreachable"] == 31
              and last["variant"] == "lr0_hl1_nf0" and last["cell"] == 4
              and last["outcome"] == "skipped: board left the view" and "total_s" not in last["times"]
              and [c["sent"] for c in last["commanded"]] == [True, True, True]
              and r52["poses"][52]["kind"] == "return",
              "17:52: ended after 10 accepts at entry 51 (lr0_hl1_nf0, cell 4): target, halfway "
              "and the way back all sent, board not back, the entry never finished")
        check(r52["end"]["cells_remaining"] == [0, 4, 29] and len(r52["end"]["bins_short"]) == 12,
              "17:52: 3 cells and every mid and far bin still short when it ended")

        def pan_runs(rec):
            """[(viewpoint, [outcome of each cell pan that sent or checked a move], [accepted])]."""
            out = []
            for e in rec["poses"]:
                if e["kind"] == "variant":
                    out.append((e["variant"], [], []))
                elif e["kind"] == "cell" and out and e.get("commanded"):
                    out[-1][1].append(e["outcome"])
                    out[-1][2].append(bool((e.get("camera") or {}).get("accepted")))
            return out

        for label, rec in recs.items():
            runs = pan_runs(rec)
            edge = [(n, o, a) for n, o, a in runs if ac.at_reach_edge(o)]
            first = runs[0]
            check(first[0] == "lr0_hl0_nf0" and first[1] and set(first[1]) == {"unreachable"}
                  and ac.at_reach_edge(first[1]),
                  f"{label} replay: near corner lr0_hl0_nf0, {len(first[1])} pans all IK-unreachable; "
                  f"now stopped after {ac.REACH_EDGE_PANS} ({len(first[1]) - ac.REACH_EDGE_PANS} "
                  "pans not tried)")
            print(f"    {label}: reach edge at " + "; ".join(
                f"{n} ({len(o) - ac.REACH_EDGE_PANS} of {len(o)} pans dropped, "
                f"{sum(a[ac.REACH_EDGE_PANS:])} of them accepted)" for n, o, a in edge))
        edge05 = {n: sum(a[2:]) for n, o, a in pan_runs(recs["15:05"]) if ac.at_reach_edge(o)}
        edge52 = {n: sum(a[2:]) for n, o, a in pan_runs(recs["17:52"]) if ac.at_reach_edge(o)}
        check(edge05 == {"lr0_hl0_nf0": 0, "lr2_hl0_nf0": 3, "lr2_hl1_nf0": 3}
              and edge52 == {"lr0_hl0_nf0": 0, "lr0_hl1_nf0": 0},
              "the rule's cost, replayed: 15:05 would lose 6 of 28 accepts (lr2_hl0_nf0, "
              "lr2_hl1_nf0 filled cells after two failed pans); 17:52 none, and it skips "
              "lr0_hl1_nf0's cell 4, the pan that ended the session")
        for label, rec in recs.items():
            ok_v = [v for v in rec["variants"] if not v["skip"]]
            idx = {v["name"]: i for i, v in enumerate(ok_v)}
            fams_r = [[idx[n] for n in f] for f in rec["families"]["families"]]
            new = ac.visit_order(ok_v, fams_r)
            bands = [v["capture_band"] for v in new]
            rank = [ac.BAND_ORDER.index(b) for b in bands]
            grouped = all([v["family"] for v in new if v["capture_band"] == b] ==
                          sorted(v["family"] for v in new if v["capture_band"] == b)
                          for b in ac.BAND_ORDER)
            kept = all([v["name"] for v in new if v["capture_band"] == b and v["family"] == f] ==
                       [v["name"] for v in ok_v if v["capture_band"] == b and v["family"] == f]
                       for b in ac.BAND_ORDER for f in range(1, len(fams_r) + 1))
            hubs = lambda seq: len(ac.hub_legs([(v["family"], v["joints_deg"]) for v in seq], lim))
            check([v["name"] for v in ok_v] == [ok_v[m]["name"] for f in fams_r for m in f]
                  and sorted(idx) == sorted(v["name"] for v in new) and rank == sorted(rank)
                  and bands[0] == "mid" and grouped and kept,
                  f"{label} replay: recorded order was family by family; reordered "
                  f"{bands.count('mid')} mid, {bands.count('near')} near, {bands.count('far')} far, "
                  f"first {new[0]['name']}, families grouped and serpentine kept within each band; "
                  f"hub legs between viewpoints {hubs(ok_v)} -> {hubs(new)}")

        # ------------------------------------------------ probe on the sim
        print("\nprobe on the simulation")
        a = FakeArm(P0)
        cap = Capture(a, st, dry=True)
        cap.start()
        time.sleep(0.1)
        ac.run_probe(a, lim, st, True, out_path=gains, log=quiet)
        cap.halt = True
        cap.join()
        g = ac.load_gains(gains)
        check(set(g["rot_axes"]) == {"theta_x", "theta_y"} and abs(g["square_per_m"][2]) > 10,
              f"gains: aim {g['rot_axes']}, {g['square_per_m'][2]:+.1f} square px/m along tool z")

        # ------------------------------------------------ plan
        print("\nplan (dry run)")
        a = FakeArm(P0)
        lines = []
        s, e = session(a, Capture(a, st, dry=False, session_path=sess), yes=False, log=lines.append)
        text = "\n".join(lines)
        check(s is None and e is None and not a.calls and not os.path.exists(
            os.path.join(sess, ac.SESSION_RECORD)), "nothing sent, nothing written")
        names = [l.split()[0] for l in lines if l.startswith("  ") and "/" in l.split()[0]]
        check("mid(r0)/centre" in names and any(n.startswith("near(r") for n in names)
              and any(n.startswith("far(r") for n in names),
              "viewpoints named by predicted size relative to P0: " + ", ".join(names[:3]))
        board_line = [l for l in lines if l.startswith("board ")]
        cap_line = [l for l in lines if l.startswith("cap ")]
        check(len(board_line) == 1 and len(cap_line) == 1 and "reachable" in cap_line[0],
              f"before any motion: {board_line[0][11:] if board_line else '?'}; "
              f"{cap_line[0][11:] if cap_line else '?'}")
        check("Move the board" not in text and "far_square_px" in text and "450 cell targets" not in text,
              "no 'move the board' warning; capture's thresholds noted; no 450-target count")
        check(a.ik_calls >= 4 * 14, f"viewpoints IK-checked along the path ({a.ik_calls} IK calls)")

        # ------------------------------------------------ full session
        print("\nfull session")
        clear()
        a = FakeArm(P0)
        lines = []
        cap = Capture(a, st, dry=False, session_path=sess)
        s, e = session(a, cap, log=lines.append)
        text = "\n".join(lines)
        print("    " + "\n    ".join(l for l in lines if l.startswith(
            ("  ended", "  moves", "  per move", "  time spent", "  cells", "  bins"))))
        check(e is None and s["cells_remaining"] == [] and s["returned_to_p0"],
              f"all cells filled, back at P0 ({s and s['end']})")
        check("left]" in text and "targets (" in text, "live remaining counts in the log")
        check("time spent" in text and "per move" in text and set(s["timing"]["fractions"]) ==
              {"ik_s", "move_s", "still_s", "decide_s", "other_s"}, "timing breakdown printed and stored")
        r = record()
        starts = [l for l in lines if ": " in l and " targets (" in l]
        n_first = int(starts[0].split(": ")[1].split()[0]) if starts else 0
        check(r["schema"] == "panoptes.session_arm/6" and n_first <= 30 + 6,
              f"first variant plans {n_first} targets (30 empty cells + seekers), not 450")
        check(all(c.get("ik_deg") for p in r["poses"] for c in p.get("commanded", [])),
              "every commanded pose records the IK solution it was validated with")
        check(r["cap"]["requested"] == 75 and r["board_estimate"]["base_m"] and
              all(e["miss_px"] is not None for p in r["poses"] for e in p.get("aim_error", [])) and
              any(p.get("aim_error") for p in r["poses"]),
              "session_arm.json has the board estimate, the cap, and the aim error of each move")

        # ------------------------------------------------ the old variants
        print("\nlateral variants, behind the switch")
        clear()
        a = FakeArm(P0)
        lines = []
        sc_lateral = dict(scfg, lateral_variants=1)
        cap = Capture(a, st, dry=False, session_path=sess)
        cap.start()
        time.sleep(0.1)
        s = ac.run_session(a, lim, st, True, gains_path=gains, scfg=sc_lateral, acfg=acfg, mode="orbit",
                           log=lines.append)
        cap.halt = True
        cap.join()
        check(any("mid(z0)/centre" in l for l in lines) and record()["cap"] is None
              and s["cells_remaining"] == [], "lateral_variants 1: the old plan, all cells filled")
        check(scfg["lateral_variants"] == 0, "config/arm_session.yaml: off by default")

        # ------------------------------------------------ one abort: continue
        print("\none ACTION_ABORT on the sixth move (as on 2026-10-28)")
        clear()
        a = FakeArm(P0, abort_at={6})
        lines = []
        s, e = session(a, Capture(a, st, dry=False, session_path=sess), log=lines.append)
        r = record()
        ab = [p for p in r["poses"] if (p["outcome"] or "").startswith("aborted")]
        i = ab[0]["i"] if ab else -1
        before = [p for p in r["poses"][:i] if p["outcome"] == "reached"]
        back = r["poses"][i + 1] if 0 <= i < len(r["poses"]) - 1 else {}
        check(len(ab) == 1 and "INVALID_PARAM" in ab[0]["outcome"]
              and "joints_after_abort_deg" in ab[0], "logged: outcome, reason, joints after the abort")
        check(back.get("kind") == "back" and back["commanded"][-1]["abs"] ==
              before[-1]["commanded"][-1]["abs"], "next move: back to the last reached pose")
        check(e is None and s["aborted"] == 1 and len(r["poses"]) > i + 3 and s["returned_to_p0"],
              f"session went on ({len(r['poses']) - i - 2} more entries) and ended at P0 "
              f"({s and s['end']})")

        # ------------------------------------------------ three aborts: stop
        print("\nthree consecutive aborts")
        clear()
        a = FakeArm(P0, abort_at=set(range(6, 100)))
        s, e = session(a, Capture(a, st, dry=False, session_path=sess))
        r = record()
        check(e and "3 consecutive aborts" in str(e) and len(a.calls) == 8,
              f"ends after the third (target, back, back): {len(a.calls)} moves sent, none after")
        check(r["end"]["aborted"] == 3 and not r["end"]["returned_to_p0"],
              "recorded; not moved to P0 after a fatal end")
        a = FakeArm(P0, abort_at={6, 9, 12, 15})
        s, e = session(a, Capture(a, st, dry=False, session_path=sess))
        check(e is None and s["aborted"] == 4, f"aborts separated by reached targets don't add up "
              f"({s and s['aborted']} aborts, session ended: {s and s['end']})")

        # ------------------------------------------------ IK validation
        print("\nIK validation")

        def ik_fails_far_right(pose):
            if ac.relative(P0, pose)["rotvec_deg"][1] < -15:
                raise armlib.IKFailed("no IK solution (fake)")
            return fake_joints(pose)

        clear()
        a = FakeArm(P0, ik=ik_fails_far_right)
        s, e = session(a, Capture(a, st, dry=False, session_path=sess))
        r = record()
        un = [p for p in r["poses"] if p["outcome"] == "unreachable"]
        sent = set(a.calls)
        check(e is None and un and all("IK failed" in p["note"] for p in un)
              and all(tuple(p["commanded"][-1]["abs"]) not in sent and not p["commanded"][-1]["sent"]
                      for p in un), f"IK failure: {len(un)} targets logged unreachable, none sent")

        def joint5_limit(pose):
            q = fake_joints(pose)
            if ac.relative(P0, pose)["rotvec_deg"][0] > 15:
                q[4] = 125.0                 # inside +128 but within the 5 deg margin
            return q

        clear()
        a = FakeArm(P0, ik=joint5_limit)
        s, e = session(a, Capture(a, st, dry=False, session_path=sess))
        un = [p for p in record()["poses"] if p["outcome"] == "unreachable"]
        check(e is None and un and all("joint 5" in p["note"] for p in un),
              f"joint 5 at 125 deg (limit +128, margin 5): {len(un)} unreachable")

        def branch_flip(pose):
            q = fake_joints(pose)
            if ac.relative(P0, pose)["rotvec_deg"][1] > 15:
                q[3] += 170.0
            return q

        clear()
        a = FakeArm(P0, ik=branch_flip)
        s, e = session(a, Capture(a, st, dry=False, session_path=sess))
        un = [p for p in record()["poses"] if p["outcome"] == "unreachable"]
        check(e is None and un and all("jumps" in p["note"] for p in un),
              f"IK branch change along the path: {len(un)} unreachable")
        q = [350.0, 10, 0, 0, 0, 0]
        check(armlib.wrap180(q[0]) == -10.0, "joint angles reported 0..360 are wrapped before the "
              "limit check")

        # ------------------------------------------------ endings
        print("\nendings")

        class QAfter:
            def __init__(self, n):
                self.k, self.n = 0, n

            def quit_requested(self):
                self.k += 1
                return self.k > self.n

        clear()
        a = FakeArm(P0)
        s, e = session(a, Capture(a, st, dry=False, session_path=sess), keys=QAfter(60))
        check(s and s["end"] == "operator pressed q" and a.calls[-1] == tuple(P0), "q: back to P0")
        a = FakeArm(P0)
        s, e = session(a, Capture(a, st, dry=False, session_path=sess, die_after=150))
        check(s and s["end"].startswith("capture died") and a.calls[-1] == tuple(P0),
              "capture dies: back to P0")

        class CtrlC(FakeArm):
            def reach_pose(self, target, lim):
                if len(self.calls) == 5:
                    self.calls.append(tuple(target))
                    raise armlib.ArmInterrupted("interrupted (Ctrl-C): Base.Stop sent")
                return super().reach_pose(target, lim)

        a = CtrlC(P0)
        s, e = session(a, Capture(a, st, dry=False, session_path=sess))
        check(s and s["end"].startswith("operator Ctrl-C") and a.calls[-1] == tuple(P0),
              "Ctrl-C during a move: back to P0")

        # ------------------------------------------------ aiming
        print("\naiming: the recorded corner")
        rec = json.load(open(CORNER_FIXTURE))
        firsts = [p for p in rec["poses"] if p["cell"] == 29 and p.get("commanded")]
        check(rec["end"]["cells_remaining"] == [29] and len(firsts) == 7 and
              all(len(p["commanded"]) == 4 for p in firsts),
              "recorded: cell 29 tried 7 times, 4 moves each, never filled")
        im = {"width": 1280, "height": 1024}
        g = ac.load_gains(gains)
        rot = ac.rotation_for_aim(g, (640 + 300, 512), (640, 512), im)
        want = math.degrees(math.atan(300 / g["f_px"][0]))
        check(abs(np.linalg.norm(list(rot.values())) - want) < 0.05,
              f"from the centre the rotation is atan(dpx / f): {want:.2f} deg")
        half = ac.rotation_for_aim(g, (900, 700), (640, 512), im, 0.6)
        full = ac.rotation_for_aim(g, (900, 700), (640, 512), im)
        check(np.allclose(list(half.values()), 0.6 * np.array(list(full.values())), atol=1e-9),
              "first move: 60% of the rotation")
        c = ac.compose(P0, rot)
        check(np.allclose(np.array(view(c)[0]).mean(0)[1], np.array(view(P0)[0]).mean(0)[1], atol=25),
              "a purely horizontal aim stays on its row")

        def aim_at(geometry, name, p0, cells):
            """Probe here, then aim at each cell from a settled P0, as a session does."""
            use_geometry(geometry)
            ga = os.path.join(d, f"gains_{name}.yaml")
            arm = FakeArm(p0)
            cp = Capture(arm, st, dry=True)
            cp.start()
            time.sleep(0.1)
            ac.run_probe(arm, lim, st, True, out_path=ga, log=quiet)
            gn = ac.load_gains(ga)
            out = []
            for cell in cells:
                arm.pose = list(p0)
                arm.unsettled = 3
                time.sleep(0.1)
                ses = ac._Session(arm, lim, st, gn, scfg, acfg, None, quiet)
                ses.p0 = p0
                s0 = ses.live()
                b = s0["board"]
                aim = ac.aim_for_cell(cell, s0["coverage"]["grid"], s0["image"], b["bbox_px"],
                                      b["centroid_px"], scfg["edge_margin_px"], gn)
                sv, _ = ses.visit("cell", "test", None, p0, cell=cell, aim=aim,
                                  base=(p0, b["centroid_px"]))
                out.append((cell, aim, b["centroid_px"], sv, ses.entries[0]))
            cp.halt = True
            cp.join()
            return gn, out

        for label, geometry, p0 in (("easel", EASEL, P0), ("rig", None, RIG_P0)):
            if geometry is None:
                use_rig()
                geometry = (CAM_OFFSET, CAM_R, CORNERS, BOARD_N)
            gn, res = aim_at(geometry, label, p0, [0, 4, 25, 29])
            for cell, aim, c0, sv, e in res:
                last = e.get("aim_error", [{}])[-1]
                aim = e.get("aim_refit_px", aim)
                span = np.subtract(aim, c0)
                over = float(np.dot(last.get("error_px", [1e9, 1e9]), span) / np.linalg.norm(span))
                legs = ", ".join(f"{x['leg']} {x['miss_px']:.0f}" for x in e.get("aim_error", []))
                check(sv is not None and e["outcome"] == "reached" and over <= 20
                      and last["miss_px"] <= scfg["centre_tol_px"],
                      f"{label} corner cell {cell}: aimed {np.linalg.norm(span):.0f} px, "
                      f"past the aim by {over:+.1f} px, miss px: {legs}")
            two = [e for _, _, _, _, e in res if len(e.get("aim_error", [])) >= 2]
            check(two and all(x["aim_error"][0]["leg"] == "first" for x in two),
                  f"{label}: {len(two)}/4 corners over {scfg['two_move_px']:.0f} px took two moves")
        use_geometry(EASEL)

        # ------------------------------------------------ orbit
        print("\norbit")
        for label, p0 in (("easel", P0), ("rig", RIG_P0)):
            use_rig() if label == "rig" else use_geometry(EASEL)
            ga = os.path.join(d, f"gains_{label}.yaml")
            gn = ac.load_gains(ga)
            arm = FakeArm(p0)
            cp = Capture(arm, st, dry=False, session_path=sess)
            cp.start()
            time.sleep(0.1)
            s0 = ac.read_status(st)
            cp.halt = True
            cp.join()
            b = ac.estimate_board(p0, s0, gn, 0.06)
            vs = ac.build_orbit(p0, s0, gn, scfg, acfg, b)
            errs = [ac.pointing_error_deg(v["pose"], b["base_m"], gn) for v in vs]
            check(len(vs) == 75 and max(errs) < 0.5,
                  f"{label}: {len(vs)} viewpoints, optical axis on the board centre within "
                  f"{max(errs):.4f} deg (limit 0.5)")
            rs = [math.dist(v["pose"][:3], b["base_m"]) for v in vs]
            check(sorted(set(round(r - b["distance_m"], 3) for r in rs)) == [-0.2, 0.0, 0.2],
                  f"{label}: radii P0's {b['distance_m']:.3f} m and +-0.20 m")
            rolled = [v for v in vs if v["orbit"]["roll_deg"]]
            check(0 < len(rolled) < len(vs) and {v["orbit"]["roll_deg"] for v in rolled} == {15.0, -15.0},
                  f"{label}: roll +-15 on {len(rolled)} of {len(vs)}")
            if label == "rig":
                use_rig()
                vu = [view(v["pose"]) for v in vs]
                offs = [float(np.hypot(*(x[0].mean(0) - (640, 512)))) for x in vu]
                rig_full = sum(bool(x[1].all()) for x in vu)
                print(f"    rig: true board lands {np.median(offs):.0f} px (median), {max(offs):.0f} px "
                      f"(worst) from centre; 88/88 in view at {rig_full}/{len(vs)} viewpoints")
                check(b["distance_m"] > 0.9, f"rig: distance estimate {b['distance_m']:.2f} m "
                      f"(PnP 1.46), size gains {b['size_gain_distance_m']:.2f} m")
                check(rig_full == len(vs), f"rig: 88/88 at all {len(vs)} viewpoints")
        use_geometry(EASEL)

        clear()
        tight = dict(lim, y_min=-0.05, y_max=0.05)
        a = FakeArm(P0)
        lines = []
        cap = Capture(a, st, dry=False, session_path=sess)
        cap.start()
        time.sleep(0.1)
        ac.run_session(a, tight, st, False, gains_path=gains, scfg=scfg, acfg=acfg, log=lines.append,
                       mode="orbit")
        cap.halt = True
        cap.join()
        cl = [l for l in lines if l.startswith("cap ")][0]
        n = int(cl.split()[1].split("/")[0])
        check(0 < n < 75 and "outside the box" in cl and not a.calls,
              f"clipped to the box, nothing sent: {cl[11:]}")
        print("\nIK clips the cap too")

        def ik_no_high(pose):
            if pose[2] > 0.60:
                raise armlib.IKFailed("no IK solution (fake)")
            return fake_joints(pose)

        a = FakeArm(P0, ik=ik_no_high)
        lines = []
        cap = Capture(a, st, dry=False, session_path=sess)
        cap.start()
        time.sleep(0.1)
        ac.run_session(a, lim, st, False, gains_path=gains, scfg=scfg, acfg=acfg, log=lines.append,
                       mode="orbit")
        cap.halt = True
        cap.join()
        cl = [l for l in lines if l.startswith("cap ")][0]
        check(" 0 refused by IK" not in cl and not a.calls, f"{cl[11:]}")

        # ------------------------------------------------ pose algebra
        print("\npose algebra")
        p = ac.compose(P0, {"theta_x": 10, "theta_y": -5}, [0.01, 0.02, 0.03])
        rel = ac.relative(P0, p)
        check(np.allclose(rel["t_m"], [0.01, 0.02, 0.03], atol=1e-5), "relative() inverts compose()")
        m = ac.interpolate(P0, p, 0.25)
        check(np.allclose(ac.relative(P0, m)["t_m"], [0.0025, 0.005, 0.0075], atol=1e-5) and
              np.allclose(np.array(ac.relative(P0, m)["rotvec_deg"]) * 4, rel["rotvec_deg"], atol=1e-3),
              "interpolate(): a quarter of the translation and of the rotation")

        # ------------------------------------------------ joint-space moves
        print("\njoint moves")
        Q = [10.0, 340.0, 75.0, 20.0, 300.0, 90.0]
        stub = FakeStub(P0)
        real = armlib.Arm()
        real._base_stub, real._base = stub, armlib._ReadOnly(stub, armlib.BASE_READS)
        check(np.allclose(real.forward_kinematics(Q), P0, atol=1e-4) and
              [(a.joint_identifier, a.value) for a in stub.fk_request.joint_angles] == list(enumerate(Q)),
              "forward_kinematics(): JointAngles with ids 0..5, Pose back")
        try:
            real._base.ExecuteAction(None)
            check(False, "ExecuteAction is not reachable through the reads list")
        except armlib.ArmError:
            check(True, "ExecuteAction is not reachable through the reads list")
        check(real.reach_joints(Q, lim) == ("end", None) and real.last_move_s is not None,
              "reach_joints() runs to ACTION_END and records its duration")
        act = stub.executed[-1]
        c = act.reach_joint_angles
        check(act.WhichOneof("action_parameters") == "reach_joint_angles" and
              [(a.joint_identifier, a.value) for a in c.joint_angles.joint_angles] == list(enumerate(Q))
              and not c.HasField("constraint"),
              "the action sent by default: reach_joint_angles, six joints, no constraint (Kinova's form)")
        check(real.reach_joints(Q, lim, 7.5) == ("end", None) and
              stub.executed[-1].reach_joint_angles.constraint.type
              == armlib.Base_pb2.JOINT_CONSTRAINT_SPEED and
              stub.executed[-1].reach_joint_angles.constraint.value == 7.5,
              "reach_joints(dps=7.5): JOINT_CONSTRAINT_SPEED 7.5")
        sent = len(stub.executed)
        try:
            real.reach_joints(Q, lim, armlib.ROTATION_CEILING_DPS + 1)
            check(False, "reach_joints() refuses a constraint above the ceiling")
        except armlib.ArmError:
            check(len(stub.executed) == sent, "reach_joints() refuses a constraint above the ceiling")
        stub.abort = True
        result, reason = real.reach_joints(Q, lim, 10.0)
        stub.abort = False
        check(result == "abort" and reason.startswith("METHOD_FAILED;")
              and "ACTION_START, ACTION_ABORT(METHOD_FAILED) [JOINT_SPEED_LIMIT_REACHED joint_index 2"
              in reason and "TRAJECTORY_ERROR_TYPE_INVALID_JOINT_SPEED" in reason
              and "max 1 'fake'" in reason,
              f"abort reason: sub-code, every notification, trajectory error report ({reason})")
        check(real.last_abort and "notification 2 of 2" in real.last_abort
              and "abort_details: METHOD_FAILED" in real.last_abort
              and "JOINT_CONSTRAINT_SPEED" in real.last_abort,
              "Arm.last_abort: the action sent and each notification in full")
        real.reach_joints(Q, lim)
        check(real.last_abort is None, "last_abort is cleared by the next action")
        check(real.reach_pose(tuple(P0), lim) == ("end", None) and
              stub.executed[-1].WhichOneof("action_parameters") == "reach_pose",
              "reach_pose() still runs through the same path")
        sent = len(stub.executed)
        stub.pose = (P0[0] + 2.0,) + tuple(P0[1:])
        try:
            real.reach_joints(Q, lim)
            check(False, "reach_joints() refuses an FK outside the box")
        except armlib.ArmError as e:
            check("outside the box" in str(e) and len(stub.executed) == sent,
                  "reach_joints() refuses an FK outside the box, nothing sent")
        wrong = armlib.Base_pb2.Action()
        wrong.apply_emergency_stop.SetInParent()
        try:
            real._execute(wrong, "apply_emergency_stop", lim)
            check(False, "_execute() refuses any other action")
        except armlib.ArmError:
            check(len(stub.executed) == sent, "_execute() refuses any other action")

        now = fake_joints(P0)

        def joint_refused(pose_now, target, limits=lim, **kw):
            arm = FakeArm(pose_now)
            try:
                armlib.check_joint_move(arm, limits, tuple(pose_now), now, target, show=False, **kw)
                return ""
            except armlib.ArmError as e:
                return str(e)

        q2, fk, _ = armlib.check_joint_move(FakeArm(P0), lim, tuple(P0), now, now, show=False)
        cont = armlib.continuous_joints(lim)
        check(np.allclose(fk, P0, atol=1e-9) and cont == {1, 4, 6}
              and all(0 <= v < 360 for i, v in enumerate(q2, 1) if i not in cont)
              and all(q2[i - 1] == now[i - 1] for i in cont),
              "check_joint_move(): FK of the target; limited joints in [0, 360), continuous "
              "ones from the measured angle")
        here = [350.0, 0, 0, 350.0, 60, 10.0]
        fixed = FakeArm(P0)
        fixed.forward_kinematics = lambda q: tuple(P0)       # the fake FK reads joints 4, 6 as x, z
        q3 = armlib.check_joint_move(fixed, lim, tuple(P0), here,
                                     [10.0, 0, 0, 160.0, 60, 200.0], show=False)[0]
        check(q3[0] == 370.0 and q3[3] == 520.0 and q3[5] == -160.0,
              f"continuous joints are sent the short way: 350->10 as {q3[0]:g}, 350->160 as "
              f"{q3[3]:g}, 10->200 as {q3[5]:g}")
        out = list(now)
        out[3] = lim["x_max"] * 100 + 5
        check("target: x" in joint_refused(P0, out), "a joint target whose FK is outside the box is refused")
        check("current pose" in joint_refused((5.0,) + tuple(P0[1:]), now),
              "a joint move from outside the box is refused")
        past = list(now)
        past[1] = 140.0
        check("joint 2" in joint_refused(P0, past), "a joint past its limit is refused, by name")
        warned = []
        armlib.check_joint_move(FakeArm(P0), dict(lim, joint_limits_deg=UNCONFIRMED), tuple(P0),
                                now, past, show=False, warn=warned.append)
        check(len(warned) == 1 and "2 at +140.0" in warned[0],
              "the same target with null limits is passed, with a warning naming the angle")
        def joint5_at_125(pose):
            q = fake_joints(pose)
            q[4] = 125.0
            return q

        nul = dict(lim, joint_limits_deg=UNCONFIRMED)
        why, _ = ac.ik_check(FakeArm(P0, ik=joint5_at_125), nul, P0, P0, joint5_at_125(P0))
        guide, _ = ac.ik_check(FakeArm(P0, ik=joint5_at_125), lim, P0, P0, joint5_at_125(P0))
        check(why is None and "joint 5" in str(guide),
              f"the IK path check refuses joint 5 at 125 only on a confirmed limit: {guide}")
        check("6-joint" in joint_refused(P0, now[:5]), "the wrong number of joints is refused")
        far = list(now)
        far[0] = armlib.wrap180(far[0] + 170)
        check("move_timeout_s" in joint_refused(P0, far, dict(lim, move_timeout_s=5.0), rate_dps=10.0)
              and "move_timeout_s" in joint_refused(P0, far, dict(lim, move_timeout_s=5.0), dps=10.0),
              "a joint move longer than move_timeout_s is refused, at a measured rate or a constraint")
        check(not joint_refused(P0, far, dict(lim, move_timeout_s=5.0)),
              "with neither, no estimate: the same move is not refused on time")
        wrapped = list(now)
        wrapped[0] += 360.0
        check(not joint_refused(P0, wrapped, dict(lim, move_timeout_s=5.0), rate_dps=1.0),
              "the time estimate wraps: +360 deg is no move")

        class Args:
            ip, port, username, password = "x", 0, "u", "p"
            yes = force = False
            joints = now
            speed_constraint = None

        def cli(fn, arm, **kw):
            armlib._connect = lambda a: arm
            args = Args()
            for k, v in kw.items():
                setattr(args, k, v)
            import contextlib, io
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                fn(args)
            return buf.getvalue()

        connect = armlib._connect
        a = FakeArm(P0)
        moved = list(now)
        moved[5] += 2.0
        text = cli(armlib.joints, a, joints=moved)
        check("dry run: nothing sent" in text and not a.joint_calls, "arm.py joints without --yes sends nothing")
        text = cli(armlib.joints, a, joints=moved, yes=True)
        check(len(a.joint_calls) == 1 and "result     END" in text and "joints     largest error" in text,
              "arm.py joints --yes sends one move and reports the error")
        check(a.joint_speeds == [None] and "no speed constraint" in text
              and "duration measured after the move" in text and "sent as    id 0:" in text,
              "arm.py joints sends no constraint by default and says so")
        big = list(now)
        big[0] += 90.0
        text = cli(armlib.joints, a, joints=big, yes=True)
        check(len(a.joint_calls) == 2, "no size limit on an unconstrained move (90 deg sent)")
        text = cli(armlib.joints, a, joints=now, speed_constraint=5.0)
        check("JOINT_CONSTRAINT_SPEED 5 deg/s" in text and "about 18.0 s at 5.0 deg/s" in text,
              f"--speed-constraint 5: the plan uses it for the time ({text.splitlines()[4][:90]})")
        try:
            cli(armlib.joints, a, joints=now, speed_constraint=1.0, yes=True)
            check(False, "--speed-constraint: the timeout check applies")
        except armlib.ArmError as e:
            check("move_timeout_s" in str(e) and len(a.joint_calls) == 2,
                  "--speed-constraint: the timeout check applies, nothing sent")
        try:
            cli(armlib.joints, a, joints=moved, speed_constraint=0.0)
            check(False, "--speed-constraint 0 is refused")
        except armlib.ArmError as e:
            check("outside (0," in str(e), "--speed-constraint 0 is refused")

        # ------------------------------------------------ saved poses
        print("\nsaved poses")
        poses = os.path.join(d, "arm_poses.yaml")
        armlib.POSES = poses
        check(armlib.load_poses() == {}, "no file: no poses")
        a = FakeArm(P0)
        text = cli(armlib.save_pose, a, name="board_a")
        cli(armlib.save_pose, FakeArm(ac.compose(P0, {"theta_x": 3})), name="board_b")
        got = armlib.load_poses()
        check(sorted(got) == ["board_a", "board_b"] and
              np.allclose(got["board_a"]["joints_deg"], fake_joints(P0), atol=1e-3) and
              np.allclose(got["board_a"]["pose"], P0, atol=1e-5) and
              got["board_a"]["saved_utc"].endswith("Z"),
              "save-pose keeps joints, pose and time; a second name adds to the file")
        try:
            cli(armlib.save_pose, FakeArm(ac.compose(P0, {"theta_x": 5})), name="board_a")
            check(False, "save-pose refuses an existing name without --force")
        except armlib.ArmError as e:
            check("--force" in str(e) and abs(armlib.load_poses()["board_a"]["pose"][3] - P0[3]) < 1e-3,
                  "save-pose refuses an existing name without --force, file unchanged")
        text = cli(armlib.save_pose, FakeArm(ac.compose(P0, {"theta_x": 5})), name="board_a", force=True)
        check("replaced" in text and armlib.load_poses()["board_b"]["saved_utc"] and
              abs(armlib.load_poses()["board_a"]["pose"][3] - P0[3]) > 1,
              "saving a name again replaces it, says so, and keeps the others")
        try:
            cli(armlib.save_pose, a, name="../bad")
            check(False, "a bad pose name is refused")
        except armlib.ArmError:
            check(True, "a bad pose name is refused")
        try:
            armlib.saved_pose("nope")
            check(False, "an unknown pose is refused")
        except armlib.ArmError as e:
            check("board_a, board_b" in str(e), "an unknown pose is refused, listing the saved ones")
        cli(armlib.save_pose, a, name="board_a", force=True)
        text = cli(armlib.list_poses, a)
        check("board_a" in text and "board_b" in text and "joints" in text, "list-poses prints every pose")
        a = FakeArm(ac.compose(P0, {"theta_x": 7}))
        text = cli(armlib.goto_pose, a, name="board_a")
        check("dry run" in text and not a.joint_calls, "goto-pose without --yes sends nothing")
        cli(armlib.goto_pose, a, name="board_a", yes=True)
        check(len(a.joint_calls) == 1 and not a.calls and np.allclose(a.pose, P0, atol=1e-3),
              "goto-pose --yes moves in joint space, and not through reach_pose")
        armlib._connect = connect

        # ------------------------------------------------ wrist singularity
        print("\nwrist singularity")
        wrist = lambda a5: armlib.wrist_singular([0, 0, 0, 0, a5, 0])
        check(wrist(9.9) and wrist(-9.9) and wrist(350.5) and wrist(0) and wrist(171) and wrist(190)
              and wrist(10.0), "within 10 deg of 0 or 180, either way round, in 0..360 or +-180")
        check(not (wrist(10.1) or wrist(349.9) or wrist(169) or wrist(90) or wrist(270)),
              "clear of it: 10.1, 349.9, 169, 90, 270")
        check("actuator 5" in wrist(3) and "+3.0" in wrist(3), "the message names the joint and its angle")
        rec = json.load(open(CORNER_FIXTURE))
        check(not armlib.wrist_singular(rec["p0"]["joints_deg"]),
              f"the recorded P0 of ..._191025 passes (actuator 5 "
              f"{armlib.wrap180(rec['p0']['joints_deg'][4]):+.1f} deg)")

        clear()
        a = FakeArm(P0)
        a.joints_override = [0, 0, 0, 0, 3.0, 0]
        res, err = session(a, Capture(a, st, dry=False, session_path=sess), yes=True)
        check(res is None and "actuator 5" in str(err) and not a.calls and "--from-pose" in str(err),
              "a session from a singular wrist refuses, naming the joint and --from-pose")

        away = ac.compose(P0, {"theta_x": 4})

        def from_pose(arm, name, yes):
            cp = Capture(arm, st, dry=False, session_path=sess)
            cp.start()
            time.sleep(0.1)
            try:
                return ac.run_session(arm, lim, st, yes, gains_path=gains, scfg=scfg, acfg=acfg,
                                      keys=QAfter(60), log=quiet, from_pose=name,
                                      mode="orbit"), None
            except (ac.ProbeError, armlib.ArmError) as e:
                return None, e
            finally:
                cp.halt = True
                cp.join()

        clear()
        a = FakeArm(away)
        a.joints_override = [0, 0, 0, 0, 3.0, 0]
        res, err = from_pose(a, "board_a", False)
        check(res is None and err is None and not a.joint_calls and not a.calls,
              "--from-pose dry run from a singular wrist: stops at the plan, nothing sent")
        clear()
        a = FakeArm(away)
        a.joints_override = [0, 0, 0, 0, 3.0, 0]
        res, err = from_pose(a, "board_a", True)
        check(err is None and res is not None and len(a.joint_calls) == 1 and a.calls and
              np.allclose(record()["p0"]["abs"], P0, atol=1e-3),
              "--from-pose --yes: the joint move first, then the session, with P0 at the saved pose")
        clear()
        a = FakeArm(P0)
        res, err = from_pose(a, "board_a", False)
        check(res is None and err is None and not a.joint_calls, "already at the pose: no move")
        q_sing = dict(armlib.load_poses()["board_a"])
        q_sing["joints_deg"] = [0, 0, 0, 0, 5.0, 0]
        far_pose = dict(armlib.load_poses()["board_a"])
        far_pose["pose"] = [P0[0] + 0.2] + list(P0[1:])
        armlib.write_poses({"singular": q_sing, "far": far_pose}, poses)
        a = FakeArm(away)
        res, err = from_pose(a, "singular", True)
        check("actuator 5" in str(err) and not a.joint_calls, "a singular saved pose is refused before moving")
        res, err = from_pose(a, "far", True)
        check("probe's start pose" in str(err) and not a.joint_calls,
              "a saved pose away from the probe's start is refused before moving")
        res, err = from_pose(a, "missing", True)
        check("no saved pose" in str(err) and not a.joint_calls, "an unknown saved pose is refused")

        # ------------------------------------------------ taught viewpoints
        print("\ntaught viewpoints")
        rnd = np.random.default_rng(3)
        corners = {c: rnd.uniform(-100, 100, 6) for c in ac.TAUGHT.values()}
        check(all(np.array_equal(ac.trilinear(corners, *c), v) for c, v in corners.items()),
              "trilinear(): reproduces all eight corners exactly")
        order = ac.serpentine(3, 3, 3)
        check(len(order) == 27 and len(set(order)) == 27, "serpentine order visits all 27 once")
        check(all(sum(abs(p - q) for p, q in zip(a1, b1)) == 1 for a1, b1 in zip(order, order[1:])),
              "each step is one grid step, so no long moves")
        real = armlib.load_poses(os.path.join(FIXTURES, "arm_poses_20261102.yaml"))
        cp = {ac.TAUGHT[n]: tuple(real[n]["pose"]) for n in ac.TAUGHT}
        errs = [max(abs(armlib.wrap180(a - b)) for a, b in
                    zip(ac.trilinear_pose(cp, *c), cp[c])) for c in cp]
        check(max(errs) < 1e-9, f"trilinear_pose(): reproduces the eight taught poses of "
              f"2026-11-02 at the corners (worst {max(errs):.1e})")
        mid = ac.trilinear_pose({c: (0, 0, 0, 0, 0, 170.0 if c[0] else -170.0) for c in cp}, 0.5, 0, 0)
        check(abs(abs(mid[5]) - 180) < 1e-9, "rotation blends the short way: -170 and +170 meet at 180")
        q1 = [0, 0, -45.0, 0, 60.0, 0]
        q2 = [0, 0, 135.0, 0, 60.0, 0]                       # joint 3 half a turn over
        vecs = [list(q1), [5, 0, -40, 0, 60, 0], list(q2), [5, 0, 140, 0, 60, 0], [10, 0, -35, 0, 60, 0]]
        fams = ac.families(vecs)
        seq = [(f, vecs[m]) for f, members in enumerate(fams, 1) for m in members]
        check(fams == [[0, 1, 4], [2, 3]] and len(ac.hub_legs(seq)) == 1,
              f"two synthetic families: {fams}, ordered family by family, exactly one hub transition")
        check(ac.joint_gap([350.0] * 6, [10.0] * 6) == 20.0, "the joint gap wraps across 0/360")
        wrist = [list(q1), [180.0, 0, -45, 170.0, 60, 180.0]]     # joints 1, 4, 6 half a turn over
        check(ac.families(wrist) == [[0], [1]] and ac.families(wrist, lim) == [[0, 1]]
              and ac.joint_gap(*wrist, lim) == 0.0,
              "continuous joints 1, 4, 6 never split a family; 2, 3, 5 do")

        def corner_pose(name):
            lr, hl, nf = ac.TAUGHT[name]
            return ac.compose(P0, None, [0.04 * (lr - 0.5), 0.04 * (hl - 0.5), 0.16 * (0.5 - nf)])

        taught_file = os.path.join(d, "taught.yaml")
        a = FakeArm(P0)
        cap = Capture(a, st, dry=True)
        cap.start()
        time.sleep(0.1)
        try:
            prompts, lines = [], []
            tries = {}

            def jog(prompt):
                prompts.append(prompt)
                n = prompt.split()[2].rstrip(",")
                tries[n] = tries.get(n, 0) + 1
                a.pose = list(corner_pose(n))
                a.joints_override = None
                a.unsettled = 3
                time.sleep(0.1)
                if n == "near_left_high" and tries[n] == 1:
                    a.joints_override = fake_joints(a.pose)[:4] + [120.0, 0.0]
                return ""

            busy = {"n": 0}

            def connect():
                busy["n"] += 1
                if busy["n"] == 2:
                    raise armlib.ArmError("session refused by the arm")
                return a

            done = ac.run_teach(connect, lim, st, ask=jog, log=lines.append, poses_path=taught_file)
            text = "\n".join(lines)
            got = armlib.load_poses(taught_file)
            check(done and sorted(got) == sorted(ac.TAUGHT),
                  "teach saves all eight, re-asking a pose that fails")
            check("REFUSED    near_left_high: joint 5" in text and "need 10" in text,
                  "teach refuses a pose with a joint inside the margin, naming it")
            check("isn't free" in text and a.stops >= 8, "teach stops the arm each time, and a held arm is refused")
            check(prompts[0] == "jog to near_left_high, close the Web App, press Enter (q to quit) ",
                  "the prompt")
            check("far/near   square size ratio" in text and "families   1 among" in text,
                  "teach ends with the ratio and the families: " +
                  next(l for l in text.splitlines() if l.startswith("far/near")))
            try:
                ac.run_teach(connect, lim, st, ask=jog, log=quiet, poses_path=taught_file)
                check(False, "teach refuses over existing poses without --force")
            except ac.ProbeError as e:
                check("--force" in str(e), "teach refuses over existing poses without --force")
            check(ac.run_teach(connect, lim, st, force=True, ask=lambda p: "q", log=quiet,
                               poses_path=taught_file) is False, "q leaves teach, not complete")
        finally:
            cap.halt = True
            cap.join()

        def taught_session(arm, yes, keys=None, path=taught_file, limits=lim, grid=scfg, log=quiet,
                           speed_constraint=None):
            cp = Capture(arm, st, dry=False, session_path=sess)
            cp.start()
            time.sleep(0.1)
            try:
                return ac.run_session(arm, limits, st, yes, gains_path=gains, scfg=grid, acfg=acfg,
                                      keys=keys, log=log, poses_path=path,
                                      speed_constraint=speed_constraint), None
            except (ac.ProbeError, armlib.ArmError) as e:
                return None, e
            finally:
                cp.halt = True
                cp.join()

        clear()
        a = FakeArm(ac.compose(P0, {"theta_x": 4}))
        lines = []
        res, err = taught_session(a, False, log=lines.append)
        vps = [l for l in lines if l.startswith("  lr")]
        check(res is None and err is None and not a.joint_calls and not a.calls and len(vps) == 27,
              "dry run: all 27 viewpoints planned, nothing sent")
        clear()
        a = FakeArm(ac.compose(P0, {"theta_x": 4}))
        res, err = taught_session(a, True, keys=QAfter(400))
        r = record()
        moves = [c for e in r["poses"] if e["kind"] == "variant" for c in e.get("commanded", [])]
        check(err is None and res is not None and r["mode"] == "taught" and
              sorted(r["taught_poses"]) == sorted(ac.TAUGHT) and
              "joints_deg" in r["taught_poses"]["far_left_low"] and
              "pose" in r["taught_poses"]["far_left_low"],
              f"session: ended '{res and res['end']}'; the record has the mode and the taught poses")
        check(moves and all(c["joint_speed_dps"] is None for c in moves) and
              all(v is None for v in a.joint_speeds) and len(a.joint_calls) >= len(moves) and
              any("measured_joint_dps" in c for c in moves),
              f"viewpoint moves are joint moves with no constraint and a measured rate, "
              f"{len(moves)} of them")
        check(res is not None and res["returned_to_p0"], "it returns to its start by a joint move")
        check(r["families"]["ik"]["corners_used"] == 8 and
              sum(1 for v in r["variants"] if v["taught"] and v["ik_seed"] == "taught") == 8 and
              all("ik_failed" in v for v in r["variants"]),
              "the record keeps the IK summary, and each viewpoint's taught corner and failed seeds")
        clear()
        a = FakeArm(ac.compose(P0, {"theta_x": 4}))
        res, err = taught_session(a, True, keys=QAfter(100), speed_constraint=12.0)
        r = record()
        check(err is None and a.joint_calls and all(v == 12.0 for v in a.joint_speeds) and
              r["config"]["joint_speed_constraint_dps"] == 12.0,
              f"session --speed-constraint 12: every joint move carries it ({len(a.joint_calls)}), "
              "and the record says so")
        check(a.calls and res["accepted"] > 0, f"cell pans ran between: {len(a.calls)} Cartesian moves, "
              f"{res and res['accepted']} accepted")
        clear()
        a = FakeArm(P0)
        a.joint_abort_at = {1}
        res, err = taught_session(a, True, keys=QAfter(60))
        check(err is None and res is not None and res["aborted"] == 1,
              "an aborted viewpoint is logged, and the session goes on")

        # Visit order on the simulation: mid, near, far.
        plain = lambda pose, guess: fake_joints(pose)
        vps_sim, _ = ac.build_taught(FakeArm(P0, seeded_ik=plain), ac.taught_poses(taught_file), P0,
                                     scfg, acfg, lim)
        sim_bands = [v["capture_band"] for v in vps_sim if not v["skip"]]
        sim_rank = [ac.BAND_ORDER.index(b) for b in sim_bands]
        check(sim_rank == sorted(sim_rank) and len(set(sim_bands)) >= 2,
              "simulation plan visits " + " -> ".join(
                  f"{b} {sim_bands.count(b)}" for b in ac.BAND_ORDER if b in sim_bands))

        # 17:52 replayed: the board stays out of view (87/88, as at entry 51)
        # after the first cell pan until the next joint move.
        class HideAfter(FakeArm):
            def reach_pose(self, target, lim):
                r = super().reach_pose(target, lim)
                if len(self.calls) == 1:
                    self.hidden = True
                return r

        clear()
        a = HideAfter(P0, seeded_ik=plain)
        lines = []
        res, err = taught_session(a, True, keys=QAfter(1500), log=lines.append)
        r = record()
        ab = [e for e in r["poses"] if e["kind"] == "abandon"]
        i = ab[0]["i"] if ab else -1
        prev_e = r["poses"][i - 1] if i > 0 else {}
        after = r["poses"][i + 1:] if i >= 0 else []
        check(err is None and len(ab) == 1 and ab[0]["outcome"] == "abandoned: board not in view"
              and ab[0]["note"].startswith("board not back in view after returning from a skipped target")
              and prev_e.get("outcome") == "skipped: board left the view"
              and len(prev_e.get("commanded", [])) == 3 and "total_s" in prev_e["times"],
              f"board not back after the return: the pan is finished as skipped (target, halfway, "
              f"back) and the viewpoint abandoned ({ab[0]['note'] if ab else 'no abandon entry'})")
        nxt = after[0] if after else {}
        check(nxt.get("kind") in ("hub", "variant") and "joints_deg" in nxt["commanded"][0]
              and any(e["kind"] == "variant" for e in after)
              and any("viewpoint abandoned" in l for l in lines),
              f"logged, then a joint move to the next viewpoint ({nxt.get('kind')} "
              f"{nxt.get('variant')}); {sum(e['kind'] == 'variant' for e in after)} viewpoints after it")
        check(res is not None and "board not back" not in res["end"] and res["abandoned"] == 1
              and res["returned_to_p0"],
              f"the session doesn't end there: ended '{res and res['end']}', back at P0, "
              f"summary abandoned {res and res['abandoned']}")

        # Reach edge: every pan from the first viewpoint fails the IK path check.
        vp0 = next(v for v in vps_sim if not v["skip"])

        def edge_ik(pose, guess):
            if math.dist(pose[:3], vp0["pose"][:3]) < 1e-3 and \
                    np.linalg.norm(ac.relative(vp0["pose"], pose)["rotvec_deg"]) > 0.5:
                raise armlib.IKFailed("no IK solution (fake, reach edge)")
            return fake_joints(pose)

        clear()
        a = FakeArm(P0, seeded_ik=edge_ik)
        lines = []
        res, err = taught_session(a, True, keys=QAfter(1500), log=lines.append)
        r = record()
        at0 = [e for e in r["poses"] if e["variant"] == vp0["name"]]
        cells0 = [e for e in at0 if e["kind"] == "cell"]
        edge_e = [e for e in at0 if e["kind"] == "abandon"]
        check(err is None and [e["outcome"] for e in cells0] == ["unreachable"] * ac.REACH_EDGE_PANS
              and len(edge_e) == 1 and edge_e[0]["outcome"] == "abandoned: viewpoint at reach edge"
              and any(f"{vp0['name']}: viewpoint at reach edge" in l for l in lines),
              f"reach edge at {vp0['name']}: {len(cells0)} pans tried, then '{edge_e[0]['note'] if edge_e else '?'}'")
        later_v = [e for e in r["poses"] if e["kind"] == "variant" and e["variant"] != vp0["name"]]
        check(res is not None and res["reach_edge"] == 1 and later_v and res["returned_to_p0"]
              and a.calls,
              f"then on to the next viewpoint: {len(later_v)} more visited, ended '{res and res['end']}'")
        tight = dict(lim, x_max=P0[0])
        clear()
        a = FakeArm(P0)
        lines = []
        taught_session(a, False, limits=tight, log=lines.append)
        check(sum("SKIP" in l for l in lines) > 0 and sum(l.startswith("  lr") for l in lines) == 27,
              "viewpoints outside the box are skipped: " + next(l for l in lines if l.startswith("checks")))
        res, err = taught_session(FakeArm(P0), False, path=poses)
        check("taught poses missing" in str(err), "a missing taught pose is refused, by name")

        def near_limit(pose):
            q = fake_joints(pose)
            if pose[1] < P0[1] - 0.01:                       # the right-hand column
                q[4] = 120.0                                 # 8 deg inside joint 5's +128
            return q

        clear()
        lines = []
        taught_session(FakeArm(P0, ik=near_limit), False, log=lines.append)
        un = [l for l in lines if l.startswith("  lr2") and "SKIP: unreachable: joint 5" in l]
        corner_ok = [l for l in lines if re.match(r"  lr2_hl[02]_nf[02] ", l) and "SKIP" not in l]
        check(len(un) == 5 and "need 10" in un[0] and len(corner_ok) == 4,
              f"a viewpoint whose IK lands within 10 deg of a limit is skipped: {len(un)}/5 in the "
              f"right-hand column; its 4 corners keep their taught joints ({len(corner_ok)})")

        # Two families: the right-hand corners taught with joint 3 half a turn
        # over (the fake FK can't tell), and an IK that keeps the seed's side.
        two = os.path.join(d, "two_families.yaml")
        for name, p in armlib.load_poses(taught_file).items():
            q = list(p["joints_deg"])
            if ac.TAUGHT[name][0]:
                q[2] += 180.0
            armlib.store_pose(name, q, p["pose"], True, p["square_px"], two)

        def keep_side(pose, guess):
            q = fake_joints(pose)
            if abs(armlib.wrap180(guess[2] - q[2])) > 90:
                q[2] += 180.0
            return q

        clear()
        lines = []
        res, err = taught_session(FakeArm(P0, seeded_ik=keep_side), False, path=two, log=lines.append)
        rep_line = next((l for l in lines if l.startswith("families")), "")
        check("families   2: 1 18 viewpoints, 2 9 viewpoints" in rep_line and
              "3 between viewpoints" in rep_line and "no 'hub' pose" in str(err),
              f"two families over two bands, no hub: refused after the report: {rep_line}")
        armlib.store_pose("hub", fake_joints(P0), P0, path=two)
        clear()
        a = FakeArm(P0, seeded_ik=keep_side)
        res, err = taught_session(a, True, keys=QAfter(4000), path=two)
        r = record()
        hubs = [e for e in r["poses"] if e["kind"] == "hub"]
        vinfo = {v["name"]: (v["capture_band"], v["family"]) for v in r["variants"]}
        visited = [vinfo[e["variant"]] for e in r["poses"] if e["kind"] == "variant"]
        by_band = {b: [f for vb, f in visited if vb == b] for b in ac.BAND_ORDER}
        check(err is None and hubs and all(e["outcome"] == "reached" for e in hubs)
              and [ac.BAND_ORDER.index(vb) for vb, _ in visited] ==
              sorted(ac.BAND_ORDER.index(vb) for vb, _ in visited)
              and all(f == sorted(f) and len(set(f)) == 2 for f in by_band.values() if f)
              and "family 1 -> 2" in hubs[0]["note"]
              and r["families"]["hub_legs"] == {"between": 3, "start": 0, "return": 0}
              and len(hubs) == 3,
              f"with a hub, starting at it: per band family 1 then 2 (mid, then far), one hub move per "
              f"family change, none to start or "
              f"return ({len(hubs)} hub moves: "
              + ", ".join(e["note"] for e in hubs) + ")")
        # ------------------------------------------------ corners take taught joints
        tp = ac.taught_poses(taught_file)
        taught_q = [list(p["joints_deg"]) for p in tp.values()]
        asked = []

        def picky_ik(pose, guess):
            asked.append(tuple(pose))
            if any(np.allclose(pose, p["pose"], atol=1e-9) for p in tp.values()):
                raise armlib.IKFailed("no IK solution (first convergence of the IK has failed)")
            name = next(v for v in names if np.allclose(v[1], pose, atol=1e-9))[0]
            if name == "lr1_hl0_nf0" or (name == "lr1_hl1_nf1" and
                                         any(np.allclose(guess, q) for q in taught_q)):
                raise armlib.IKFailed(f"no IK solution (fake, {name})")
            return fake_joints(pose)

        n3 = [scfg[k] for k in ac.GRID_KEYS]
        corners3 = {ac.TAUGHT[m]: tuple(p["pose"]) for m, p in tp.items()}
        names = [(f"lr{i}_hl{j}_nf{k}",
                  ac.trilinear_pose(corners3, i / (n3[0] - 1), j / (n3[1] - 1), k / (n3[2] - 1)))
                 for i, j, k in ac.serpentine(*n3)]
        vps, rep = ac.build_taught(FakeArm(P0, seeded_ik=picky_ik), tp, P0, scfg, acfg,
                                   dict(lim, x_min=-9, x_max=9, y_min=-9, y_max=9, z_min=-9, z_max=9))
        by = {v["name"]: v for v in vps}
        corner_vps = [v for v in vps if v["taught"]]
        check(len(corner_vps) == 8 and all(v["ik_seed"] == "taught" and not v["skip"] and
              v["joints_deg"] == list(tp[v["taught"]]["joints_deg"]) and
              tuple(v["pose"]) == tuple(tp[v["taught"]]["pose"]) for v in corner_vps) and
              not any(np.allclose(q, p["pose"], atol=1e-9) for q in asked for p in tp.values()),
              "every grid corner uses its taught joints and measured pose; IK is never asked for one")
        ik = rep["ik"]
        others = len(vps) - 8
        check(ik["corners"] == ik["corners_used"] == 8 and ik["others"] == others and
              ik["failed"] == 1 and ik["solved_previous_seed"] == 1 and
              ik["solved_nearest_seed"] == others - 2 and
              by["lr1_hl0_nf0"]["skip"].startswith("unreachable: seeded ") and
              [f["seed"] for f in by["lr1_hl1_nf1"]["ik_failed"]][0] in ac.TAUGHT and
              by["lr1_hl1_nf1"]["ik_seed"] == "previous" and
              {f["name"] for f in ik["failures"]} == {"lr1_hl0_nf0", "lr1_hl1_nf1"} and
              len(ik["failures"]) == 3,
              f"IK summary: {ik['failed']} failed with every seed, {ik['solved_previous_seed']} by the "
              f"previous seed, {ik['solved_nearest_seed']} by the nearest; each failed attempt named")
        lines = []
        taught_session(FakeArm(P0, seeded_ik=picky_ik), False, log=lines.append)
        ikl = next((l for l in lines if l.startswith("ik ")), "")
        check("8/8 corners use their taught joints" in ikl and "1 failed with every seed" in ikl
              and sum(l.startswith("  IK failed  lr1_hl0_nf0") for l in lines) == 2,
              f"the dry run prints the IK summary and each failure: {ikl}")

        q_far = list(now)
        q_far[3] += 40.0                 # 0.40 m of x by fake_fk
        q_far[0] += 16.0
        far_lim = dict(lim, x_max=1.0, move_timeout_s=10.0)
        armlib.check_joint_move(FakeArm(ac.compose(P0, None, None)), far_lim, tuple(P0),
                                now, q_far, show=False)
        check(True, "unconstrained, no measured rate: no timeout estimate (40 deg in 10 s passes)")
        try:
            armlib.check_joint_move(FakeArm(ac.compose(P0, None, None)), far_lim, tuple(P0),
                                    now, q_far, show=False, rate_dps=2.0)
            check(False, "a measured rate of 2 deg/s refuses 40 deg against a 10 s timeout")
        except armlib.ArmError as e:
            check("move_timeout_s" in str(e),
                  "a measured rate of 2 deg/s refuses 40 deg against a 10 s timeout")
    finally:
        for root, _, files in os.walk(d, topdown=False):
            for f in files:
                os.remove(os.path.join(root, f))
            os.rmdir(root)

    print(f"\ntest_arm_capture: {ok}/{ok + fail} passed")
    return fail == 0


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
