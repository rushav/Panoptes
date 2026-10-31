"""
Arm + camera: the camera rides on the arm, the board stands on an easel.

  python arm_capture.py probe   --status-file PATH [--yes]
  python arm_capture.py session --status-file PATH [--yes]

PROBE: find how tool motions move the board in the image. Run with
04_capture --auto-dry --status-file PATH, the board centred and fully in
view. From the arm's current pose P0 it makes six small moves, each followed
by a return to P0:

  theta_x, theta_y, theta_z   rotate +2 deg about the TOOL axis (the TCP stays put)
  x, y, z                     translate +0.03 m along the TOOL axis

After each move and each return it waits for the status file to show a new,
full (88/88), still board, and reads the board's centroid, apparent roll and
apparent square size. A return that doesn't bring the centroid back within
3 px of where it started stops the probe. From the six responses it writes
config/arm_camera.yaml: which tool axis and sign moves the board right/left,
down/up, clockwise/anticlockwise and closer/farther, and by how much. The
measurement doesn't depend on what is mounted on the arm; it is only valid
for the mounting and the pose it was taken in.

TEACH: the operator jogs the arm to eight corner viewpoints with the Web App;
each is checked and saved to config/arm_poses.yaml. See run_teach().

SESSION: drive the arm through an intrinsics session, closed loop on the
status file of 04_capture --auto --status-file PATH. Viewpoints come from the
taught poses, or from the older orbit planner with --orbit. See run_session()
and ARCHITECTURE §9.

Every move goes through arm.check_move() (box, timeout estimate, servoing
mode) and Arm.reach_pose(), and only with --yes; session --from-pose's first
move goes through arm.check_joint_move() and Arm.reach_joints(). Without --yes both commands
check their targets and print the plan. Neither moves to recover from an
arm failure (abort, timeout): it stops and prints the arm.py goto command
that returns to P0. The session does return to P0 itself on q, Ctrl-C, a
dead capture or a normal end.

Tool-frame moves need the Kortex Euler convention: x-y-z extrinsic (Gen3
user guide, "Cartesian reference frames"), so R = Rz(theta_z) Ry(theta_y)
Rx(theta_x). Adding to theta_x/y/z directly, as arm.py move --dtx does, is
NOT a tool-frame rotation except for theta_x.

The E-stop pendant must be within reach (RUNBOOK §13).
"""

import argparse, datetime, json, math, os, select, sys, time

import cv2
import numpy as np

import arm as armlib
import board as board_config
from capture_status import StatusError, age_s, describe, full_and_still, read_status

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "arm_camera.yaml")

ANGLE_DEG = 2.0           # rotation probe
STEP_M = 0.03             # translation probe
RETURN_TOL_PX = 3.0       # centroid must come back within this
EDGE_MARGIN_PX = 40.0     # board bbox at least this far inside the image to start
STALE_S = 2.0             # status older than this: the capture isn't running
SETTLE_FRAMES = 3         # new frames to skip after a move before reading
READ_FRAMES = 3           # still frames averaged per reading
STILL_TIMEOUT_S = 15.0    # wait this long for a settled reading
POLL_S = 0.05
MAX_TILT_Y_DEG = 80.0     # Euler theta_y near +-90 is singular; refuse

PROBES = ("theta_x", "theta_y", "theta_z", "x", "y", "z")


class ProbeError(RuntimeError):
    pass


# ---------------------------------------------------------------- rotations

def _axis_rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    if axis == 0:
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == 1:
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_to_R(tx, ty, tz):
    """Kortex theta_x/y/z (deg, x-y-z extrinsic) to a rotation matrix."""
    return _axis_rot(2, tz) @ _axis_rot(1, ty) @ _axis_rot(0, tx)


def R_to_euler(R):
    """Inverse of euler_to_R, degrees; theta_y in [-90, 90]."""
    ty = math.degrees(math.asin(max(-1.0, min(1.0, -R[2, 0]))))
    tx = math.degrees(math.atan2(R[2, 1], R[2, 2]))
    tz = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    return tx, ty, tz


def probe_target(p0, name, angle_deg=ANGLE_DEG, step_m=STEP_M):
    """Pose after one probe move from p0 (x, y, z, theta_x, theta_y, theta_z)."""
    R = euler_to_R(*p0[3:])
    pos = np.array(p0[:3], dtype=float)
    if name.startswith("theta_"):
        R = R @ _axis_rot("xyz".index(name[-1]), angle_deg)   # post-multiply: tool axis
    else:
        pos = pos + step_m * R[:, "xyz".index(name)]           # tool axis, in base frame
    return tuple(pos) + R_to_euler(R)


# ---------------------------------------------------------------- camera side

class Reading:
    def __init__(self, statuses):
        b = [s["board"] for s in statuses]
        self.frames = [s["frame"] for s in statuses]
        self.cx = float(np.mean([x["centroid_px"][0] for x in b]))
        self.cy = float(np.mean([x["centroid_px"][1] for x in b]))
        self.roll = float(np.mean([x["roll_deg"] for x in b]))
        self.square = float(np.mean([x["square_px"] for x in b]))

    def __str__(self):
        return (f"centroid ({self.cx:7.2f}, {self.cy:7.2f}) px  roll {self.roll:+7.3f} deg  "
                f"square {self.square:6.3f} px  (frames {self.frames[0]}-{self.frames[-1]})")


def _fresh(path):
    s = read_status(path)
    a = age_s(s)
    if a > STALE_S:
        raise ProbeError(f"status file is {a:.1f} s old: is 04_capture still running?")
    return s


def settled_reading(path, after_frame, timeout_s=STILL_TIMEOUT_S):
    """Average of READ_FRAMES distinct full, still frames, all at least
    SETTLE_FRAMES newer than after_frame. A frame that isn't full and still
    starts the count over."""
    deadline = time.monotonic() + timeout_s
    got, last = [], None
    while time.monotonic() < deadline:
        s = _fresh(path)
        last = s
        if s["frame"] >= after_frame + SETTLE_FRAMES and (not got or s["frame"] != got[-1]["frame"]):
            if full_and_still(s) and s["board"]["square_px"] is not None and \
                    s["board"]["roll_deg"] is not None:
                got.append(s)
                if len(got) == READ_FRAMES:
                    return Reading(got)
            else:
                got = []
        time.sleep(POLL_S)
    raise ProbeError(f"no settled board within {timeout_s:g} s; last status: "
                     f"{describe(last) if last else 'none'}")


def check_ready(s, dry_run=True, board=True):
    """Reasons the capture's latest status is not fit to start; empty if fit.
    dry_run: the probe needs --auto-dry (nothing written); a session needs
    --auto (a session folder to write into). board: the board must already be
    full, still and clear of the image edge."""
    why = []
    if dry_run and not s["dry_run"]:
        why.append("04_capture is not in --auto-dry (it would write a session)")
    if not dry_run and (s["dry_run"] or not s.get("session_path")):
        why.append("04_capture is not in --auto (no session folder to write into)")
    if not board:
        return why
    c = s["corners"]
    if not (c["complete"] and c["found"] == c["total"]):
        why.append(f"corners {c['found']}/{c['total']}, need all")
    if not s["motion"]["still"]:
        why.append(f"board not still ({s['decision']['reason']})")
    b, img = s["board"], s["image"]
    if b is not None:
        x0, y0, x1, y1 = b["bbox_px"]
        margin = min(x0, y0, img["width"] - x1, img["height"] - y1)
        if margin < EDGE_MARGIN_PX:
            why.append(f"board within {margin:.0f} px of the image edge; "
                       f"centre it (need {EDGE_MARGIN_PX:.0f} px)")
    return why


# ---------------------------------------------------------------- probe

def _move(arm, lim, target, what):
    current = arm.measured_pose()
    target = armlib.check_move(arm, lim, current, target, show=False)
    result, reason = arm.reach_pose(target, lim)
    if result != "end":
        raise ProbeError(f"{what}: {result.upper()}" + (f" ({reason})" if reason else "") +
                         ("; Base.Stop sent" if result == "timeout" else ""))
    return target


def _effects(base, after):
    return {"dx_px": after.cx - base.cx, "dy_px": after.cy - base.cy,
            "droll_deg": armlib.wrap180(after.roll - base.roll),
            "dsquare_px": after.square - base.square}


def run_probe(arm, lim, status_path, yes, out_path=OUT, log=print):
    """The whole probe. Returns the result dict (also written to out_path),
    or None for a dry run. Raises ProbeError or arm.ArmError on refusal."""
    s0 = read_status(status_path)
    if age_s(s0) > STALE_S:
        raise ProbeError(f"status file is {age_s(s0):.1f} s old: start 04_capture "
                         "--auto-dry --status-file first")
    why = check_ready(s0)
    if why:
        raise ProbeError("not starting:\n  " + "\n  ".join(why))
    log(f"camera     {describe(s0)}")

    p0 = arm.measured_pose()
    if abs(p0[4]) > MAX_TILT_Y_DEG:
        raise ProbeError(f"theta_y {p0[4]:+.1f} deg is within {90 - MAX_TILT_Y_DEG:g} deg of the "
                         "Euler singularity; tool-frame targets would be unreliable")
    log(f"start P0   {armlib.fmt_pose(p0)}")

    targets = {name: probe_target(p0, name) for name in PROBES}
    refused = []
    log(f"\nplan: six probes, each followed by a return to P0 "
        f"({ANGLE_DEG:g} deg rotations, {STEP_M * 1000:g} mm translations, tool frame)")
    for name in PROBES:
        log(f"  {name:<8} -> {armlib.fmt_pose(targets[name])}")
        try:
            targets[name] = armlib.check_move(arm, lim, p0, targets[name], show=False)
        except armlib.ArmError as e:
            refused.append(f"{name}: {e}")
    if refused:
        raise ProbeError("REFUSED, nothing sent:\n  " + "\n  ".join(refused))
    log("checks     all six targets and P0 inside the box; servoing SINGLE_LEVEL_SERVOING")
    if not yes:
        log("\ndry run: nothing sent. Add --yes to run the probe (about a minute).")
        return None

    goto_p0 = (".venv/bin/python python/arm.py goto " +
               " ".join(f"{v:.4f}" for v in p0) + " --yes")
    effects, returns = {}, {}
    try:
        for name in PROBES:
            base = settled_reading(status_path, read_status(status_path)["frame"])
            _move(arm, lim, targets[name], f"probe {name}")
            after = settled_reading(status_path, read_status(status_path)["frame"])
            _move(arm, lim, p0, f"return from {name}")
            back = settled_reading(status_path, read_status(status_path)["frame"])
            err = math.hypot(back.cx - base.cx, back.cy - base.cy)
            effects[name] = _effects(base, after)
            returns[name] = err
            e = effects[name]
            log(f"  {name:<8} dx {e['dx_px']:+8.2f}  dy {e['dy_px']:+8.2f} px  "
                f"roll {e['droll_deg']:+7.3f} deg  square {e['dsquare_px']:+7.3f} px  "
                f"return {err:.2f} px")
            if err > RETURN_TOL_PX:
                raise ProbeError(f"after {name} the board came back {err:.2f} px from where it "
                                 f"started (limit {RETURN_TOL_PX:g}); the camera moved in its mount, "
                                 "or the arm didn't return")
    except (ProbeError, armlib.ArmError, StatusError) as e:
        raise ProbeError(f"{e}\nprobe stopped; nothing written. The arm was not moved after "
                         f"the failure. To return to P0 (dry run first, without --yes):\n  {goto_p0}")

    result = summarise(effects, returns, p0, s0)
    write_yaml(result, out_path)
    log("\n" + "\n".join(summary_lines(result)))
    log(f"\nwrote {os.path.relpath(out_path)}")
    return result


# ---------------------------------------------------------------- results

def _per_unit(name, e):
    unit = ANGLE_DEG if name.startswith("theta_") else STEP_M
    return {k: v / unit for k, v in e.items()}


def _choose(sens, names, key):
    """Axis among names with the largest |sens[name][key]|, its sign, value,
    and the second-largest magnitude as a fraction of the largest."""
    mags = sorted(((abs(sens[n][key]), n) for n in names), reverse=True)
    best = mags[0][1]
    v = sens[best][key]
    ratio = mags[1][0] / mags[0][0] if mags[0][0] > 0 else 1.0
    return {"axis": best, "sign": 1 if v >= 0 else -1, "per_unit": abs(v),
            "second_best_ratio": ratio}


def summarise(effects, returns, p0, s0):
    sens = {n: _per_unit(n, e) for n, e in effects.items()}
    rots = [n for n in PROBES if n.startswith("theta_")]
    trans = [n for n in PROBES if not n.startswith("theta_")]
    return {
        "created_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "camera_serial": s0["camera_serial"],
        "start_pose": list(p0),
        "start_board": s0["board"],
        "sensitivity": sens,
        "raw_effects": effects,
        "return_error_px": returns,
        "image_x": {"rotation": _choose(sens, rots, "dx_px"),
                    "translation": _choose(sens, trans, "dx_px")},
        "image_y": {"rotation": _choose(sens, rots, "dy_px"),
                    "translation": _choose(sens, trans, "dy_px")},
        "roll": {"rotation": _choose(sens, rots, "droll_deg")},
        "size": {"translation": _choose(sens, trans, "dsquare_px")},
    }


def _axis_words(axis):
    return f"rotate about tool {axis[-1]}" if axis.startswith("theta_") else f"move along tool {axis}"


def summary_lines(r):
    out = ["Per probe (per +1 deg of rotation, per +1 m of translation, tool frame):"]
    for n in PROBES:
        s = r["sensitivity"][n]
        unit = "+1 deg" if n.startswith("theta_") else "+1 m  "
        out.append(f"  tool {n:<7} {unit} moves the board {s['dx_px']:+9.2f} px in image x, "
                   f"{s['dy_px']:+9.2f} px in image y, roll {s['droll_deg']:+8.3f} deg, "
                   f"square {s['dsquare_px']:+8.3f} px")
    out.append("\nTo move the board in the image (the opposite direction is the opposite sign):")
    rows = [("right (+x)", r["image_x"]["rotation"], "px/deg"),
            ("right (+x)", r["image_x"]["translation"], "px/m"),
            ("down (+y)", r["image_y"]["rotation"], "px/deg"),
            ("down (+y)", r["image_y"]["translation"], "px/m"),
            ("clockwise", r["roll"]["rotation"], "deg/deg"),
            ("closer", r["size"]["translation"], "square px/m")]
    for label, c, unit in rows:
        flag = "   CAUTION: coupled, second axis {:.0%} as strong".format(
            c["second_best_ratio"]) if c["second_best_ratio"] > 0.5 else ""
        out.append(f"  {label:<11} {_axis_words(c['axis'])} {'+' if c['sign'] > 0 else '-'}  "
                   f"({c['per_unit']:.2f} {unit}){flag}")
    return out


def _flow(d, places=4):
    return "{ " + ", ".join(f"{k}: {v:.{places}f}" for k, v in d.items()) + " }"


def write_yaml(r, path):
    """config/arm_camera.yaml, readable by cv2.FileStorage (repo convention)."""
    L = ["%YAML:1.0", "---",
         "# Written by python/arm_capture.py probe (RUNBOOK §13). Rerun the probe",
         "# rather than editing: it is valid only for this camera, this camera",
         "# position, this grip of the board, and the arm pose it started from.",
         "#",
         "# Tool frame. Rotations are about the tool axes through the TCP, angles in",
         "# deg; translations along the tool axes, in m. Kortex Euler angles are",
         "# x-y-z extrinsic: R = Rz(theta_z) Ry(theta_y) Rx(theta_x) (Gen3 user guide).",
         "# Image: x right, y down, px; roll positive = clockwise on screen; square_px",
         "# = apparent size of one board square, larger = closer.",
         f'created_utc: "{r["created_utc"]}"',
         f"camera_serial: {r['camera_serial']}",
         "start_pose: [ " + ", ".join(f"{v:.5f}" for v in r["start_pose"]) + " ]   # x y z m, theta_x/y/z deg",
         f"probe_angle_deg: {ANGLE_DEG}",
         f"probe_step_m: {STEP_M}",
         "start_board: { centroid_px: [ %.2f, %.2f ], square_px: %.3f, roll_deg: %.3f }" % (
             *r["start_board"]["centroid_px"], r["start_board"]["square_px"],
             r["start_board"]["roll_deg"]),
         "",
         "# Image change per unit tool motion: rotation rows per deg, translation rows per m.",
         "sensitivity:"]
    for n in PROBES:
        L.append(f"  {n}: {_flow(r['sensitivity'][n])}")
    L += ["# Measured change for the actual probe size.", "raw_effects:"]
    for n in PROBES:
        L.append(f"  {n}: {_flow(r['raw_effects'][n])}")
    L += ["# Centroid distance between before and after each return to the start, px.",
          "return_error_px: " + _flow(r["return_error_px"], 3),
          "",
          "# Which tool axis to use for each image direction. sign: the tool motion's",
          "# sign that moves the board right / down / clockwise / closer; the opposite",
          "# direction is the opposite sign. second_best_ratio: the next strongest axis",
          "# as a fraction of this one (over 0.5: the axes are coupled).",
          "mapping:"]
    entries = [("right", "rotation", r["image_x"]["rotation"], "px_per_deg"),
               ("right", "translation", r["image_x"]["translation"], "px_per_m"),
               ("down", "rotation", r["image_y"]["rotation"], "px_per_deg"),
               ("down", "translation", r["image_y"]["translation"], "px_per_m"),
               ("clockwise", "rotation", r["roll"]["rotation"], "deg_per_deg"),
               ("closer", "translation", r["size"]["translation"], "square_px_per_m")]
    current = None
    for direction, kind, c, unit in entries:
        if direction != current:
            L.append(f"  {direction}:")
            current = direction
        L.append(f'    {kind}: {{ axis: "{c["axis"]}", sign: {c["sign"]}, '
                 f"{unit}: {c['per_unit']:.4f}, second_best_ratio: {c['second_best_ratio']:.3f} }}")
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write("\n".join(L) + "\n")
    os.replace(tmp, path)


# ---------------------------------------------------------------- session config

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config")
SESSION_CFG = os.path.join(CONFIG_DIR, "arm_session.yaml")
AUTOCAPTURE_CFG = os.path.join(CONFIG_DIR, "autocapture.yaml")
SESSION_RECORD = "session_arm.json"
LOST_FRAMES = 3           # consecutive frames under 88/88 after settling: board left the view
ACCEPT_WAIT_FRAMES = 4    # after min_interval, frames to wait for capture's accept
# Extra size allowed when fitting an aimed board into the image: growth()
# is a pinhole model of the centre; a tilted board's near edge grows more
# (keystone, ~10% at 30 deg and 1.3 m) and the aim lands ~20 px long.
FIT_SAFETY = 1.15
DISTANCE_DISAGREE = 0.15  # warn if the two board-distance estimates differ by more


GRID_KEYS = ("n_lr", "n_hl", "n_nf")


def _numbers(path, spec):
    """{key: value} for spec {key: (lo, hi)}; every key required, lo < v <= hi."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ProbeError(f"cannot open {path}")
    out = {}
    for key, (lo, hi) in spec.items():
        n = fs.getNode(key)
        if n.empty() or not (n.isInt() or n.isReal()):
            raise ProbeError(f"{path}: {key} is missing or not a number")
        v = n.real()
        if not lo < v <= hi:
            raise ProbeError(f"{path}: {key} = {v} is outside ({lo}, {hi}]")
        out[key] = v
    fs.release()
    return out


def _number_list(path, key, lo, hi):
    """A non-empty sequence of numbers, each lo <= v <= hi."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ProbeError(f"cannot open {path}")
    n = fs.getNode(key)
    if n.empty() or not n.isSeq() or n.size() == 0:
        fs.release()
        raise ProbeError(f"{path}: {key} is missing or not a list")
    out = [n.at(i).real() for i in range(n.size())]
    fs.release()
    if not all(lo <= v <= hi for v in out):
        raise ProbeError(f"{path}: {key} = {out} is outside [{lo}, {hi}]")
    return out


def load_session_config(path=SESSION_CFG):
    cfg = _numbers(path, {"lateral_variants": (-1, 1), "lateral_m": (0, 0.30), "depth_m": (0, 0.30),
                          "orbit_roll_every": (0, 100),
                          "two_move_px": (0, 2000), "first_move_fraction": (0, 1),
                          "mid_band_fraction": (0, 0.5), "band_spread_min": (0, 1),
                          "still_timeout_s": (0, 60), "max_rotation_deg": (0, 45),
                          "centre_tol_px": (0, 500), "edge_margin_px": (0, 300),
                          "p0_tolerance_m": (0, 0.5), "p0_tolerance_deg": (0, 90)})
    for key, v in _numbers(path, {k: (1, 20) for k in GRID_KEYS}).items():
        if v != int(v):
            raise ProbeError(f"{path}: {key} = {v} is not a whole number")
        cfg[key] = int(v)
    cfg["orbit_angles_deg"] = _number_list(path, "orbit_angles_deg", -45, 45)
    cfg["orbit_radii_m"] = _number_list(path, "orbit_radii_m", -0.5, 0.5)
    cfg["orbit_rolls_deg"] = _number_list(path, "orbit_rolls_deg", -45, 45)
    return cfg


def load_autocapture_numbers(path=AUTOCAPTURE_CFG):
    """The fields the session needs from 04_capture's own config."""
    return _numbers(path, {"min_interval_ms": (-1, 60000), "min_per_bin": (0, 1000),
                           "near_square_px": (0, 10000), "far_square_px": (0, 10000)})


def load_gains(path=OUT):
    """The probe's result, as the matrices the session uses."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ProbeError(f"cannot open {path}: run arm_capture.py probe first")
    sens = {}
    for n in PROBES:
        node = fs.getNode("sensitivity").getNode(n)
        if node.empty():
            raise ProbeError(f"{path}: sensitivity.{n} missing")
        sens[n] = {k: node.getNode(k).real() for k in ("dx_px", "dy_px", "droll_deg", "dsquare_px")}
    m = fs.getNode("mapping")
    axis = lambda d, kind: m.getNode(d).getNode(kind).getNode("axis").string()
    sign = lambda d, kind: int(m.getNode(d).getNode(kind).getNode("sign").real())
    start = fs.getNode("start_pose")
    g = {"source": path,
         "created_utc": fs.getNode("created_utc").string(),
         "camera_serial": int(fs.getNode("camera_serial").real()),
         "start_pose": tuple(start.at(i).real() for i in range(start.size())),
         "rot_axes": (axis("right", "rotation"), axis("down", "rotation")),
         "lateral_axes": (axis("right", "translation"), axis("down", "translation")),
         "closer_axis": axis("closer", "translation"),
         "closer_sign": sign("closer", "translation"),
         "sens": sens}
    fs.release()
    rx, ry = g["rot_axes"]
    if len(g["start_pose"]) != 6 or not rx.startswith("theta_") or not ry.startswith("theta_"):
        raise ProbeError(f"{path}: malformed (start_pose or mapping)")
    if rx == ry:
        raise ProbeError(f"{path}: the same tool axis ({rx}) moves the board right and down; "
                         "the camera can't be aimed in 2-D with these gains. Rerun the probe.")
    g["J_rot"] = np.array([[sens[rx]["dx_px"], sens[ry]["dx_px"]],
                           [sens[rx]["dy_px"], sens[ry]["dy_px"]]])       # px per deg
    g["J_trans"] = np.array([[sens[a]["dx_px"] for a in "xyz"],
                             [sens[a]["dy_px"] for a in "xyz"]])          # px per m
    g["square_per_m"] = np.array([sens[a]["dsquare_px"] for a in "xyz"])
    # Effective focal length along image x and y, px: the probe's px/deg at
    # the image centre is f * pi/180 for a camera rotating about its own
    # centre. The TCP isn't that centre, so these are aiming gains, not the
    # lens's focal length (FINDINGS §20: 653/752 against 768).
    g["f_px"] = np.array([np.hypot(*g["J_rot"][0]), np.hypot(*g["J_rot"][1])]) * 180 / math.pi
    g["cam_axes"] = camera_axes(sens)
    if np.linalg.cond(g["J_rot"]) > 20:
        raise ProbeError(f"{path}: rotation gains for {rx}/{ry} are nearly parallel "
                         f"(condition {np.linalg.cond(g['J_rot']):.0f}); rerun the probe")
    return g


# ---------------------------------------------------------------- pose algebra

def compose(pose, rot_deg=None, trans_m=None):
    """pose moved by trans_m (3, tool frame of pose, m), then rotated by rot_deg
    ({"theta_x": deg, ...}, one rotation vector about the tool axes, about the
    TCP)."""
    R = euler_to_R(*pose[3:])
    p = np.array(pose[:3], dtype=float)
    if trans_m is not None:
        p = p + R @ np.asarray(trans_m, dtype=float)
    if rot_deg:
        rv = np.zeros(3)
        for axis, deg in rot_deg.items():
            rv["xyz".index(axis[-1])] += math.radians(deg)
        R = R @ cv2.Rodrigues(rv)[0]
    return tuple(float(v) for v in p) + R_to_euler(R)


def relative(p0, pose):
    """pose in P0's tool frame: translation (m) and rotation vector (deg)."""
    R0, R = euler_to_R(*p0[3:]), euler_to_R(*pose[3:])
    t = R0.T @ (np.array(pose[:3]) - np.array(p0[:3]))
    rv = cv2.Rodrigues(R0.T @ R)[0].ravel()
    return {"t_m": [round(float(v), 5) for v in t],
            "rotvec_deg": [round(math.degrees(float(v)), 4) for v in rv]}


def camera_axes(sens):
    """The camera's x, y, z axes in the tool frame (columns), from the probe.
    The size gains point at the board, which the probe sees centred, so they
    are the optical axis. A rotation about camera x moves the board +f down
    and one about camera y moves it -f right, so the rotation gains' image-y
    and image-x columns give the other two. Agrees within 1.3 deg with PnP
    against arm rotations (FINDINGS §20)."""
    z = np.array([sens[a]["dsquare_px"] for a in "xyz"])
    if np.linalg.norm(z) < 1.0:
        raise ProbeError("the probe saw almost no size change along any tool axis; "
                         "the optical axis can't be found. Rerun the probe.")
    z = z / np.linalg.norm(z)
    x = np.array([sens[f"theta_{a}"]["dy_px"] for a in "xyz"])
    x = x - (x @ z) * z
    x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    if y @ np.array([sens[f"theta_{a}"]["dx_px"] for a in "xyz"]) > 0:
        raise ProbeError("the probe's size and rotation gains give a left-handed camera; rerun it")
    return np.column_stack([x, y, z])


def rotation_for_shift(g, shift_px, cap_deg=None):
    """Tool rotation predicted to move the board by shift_px (dx, dy), from
    the probe's linear gains; scaled down to cap_deg if larger. Valid near
    the image centre; rotation_for_aim() handles large moves. Only the
    lateral variants use it."""
    d = np.linalg.solve(g["J_rot"], np.asarray(shift_px, dtype=float))
    norm = float(np.hypot(*d))
    capped = cap_deg is not None and norm > cap_deg
    if capped:
        d = d * (cap_deg / norm)
    rx, ry = g["rot_axes"]
    return {rx: float(d[0]), ry: float(d[1])}, capped


def _centre(image):
    return np.array([image["width"] / 2.0, image["height"] / 2.0])


def rotation_for_aim(g, aim_px, now_px, image, scale=1.0):
    """Tool rotation (one rotation vector, {"theta_x": deg, ...}) to move the
    board centroid from now_px to aim_px, or scale of it. Pinhole: the ray to
    the board must end up on the aim's ray, so the camera turns by the angle
    between the two rays, about their common normal. For a centred board that
    angle is atan(dpx / f). Off centre the rays are what's right: atan(dpx / f)
    from there overshoots by 1 + tan^2 of the off-axis angle, which is what
    the second move of a long aim would do. Camera axes to tool axes by
    cam_axes, so the optical axis needn't be tool z."""
    c, f = _centre(image), g["f_px"]
    ray = lambda px: np.array([*((np.asarray(px, dtype=float) - c) / f), 1.0])
    a, b = ray(aim_px), ray(now_px)
    axis = np.cross(a, b)
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return {f"theta_{k}": 0.0 for k in "xyz"}
    angle = scale * math.degrees(math.atan2(n, float(a @ b)))
    rv = g["cam_axes"] @ (axis / n) * angle
    return {f"theta_{k}": float(v) for k, v in zip("xyz", rv)}


def growth(g, at_px, image):
    """Apparent size at image point at_px relative to the image centre, for
    the same board distance: 1 + tan^2(off-axis angle) (pinhole; the lens's
    barrel distortion makes the real growth smaller)."""
    t = (np.asarray(at_px) - _centre(image)) / g["f_px"]
    return 1.0 + float(t @ t)


def _cell_rect(cell, grid):
    col, row = cell % grid["cols"], cell // grid["cols"]
    w, h = grid["cell_width_px"], grid["cell_height_px"]
    return col * w, row * h, (col + 1) * w, (row + 1) * h


def aim_for_cell(cell, grid, image, bbox, centroid, margin, g):
    """Where to put the board centroid so the board covers `cell` and stays
    at least `margin` inside the image. Starts at the cell centre and moves
    toward the image centre until the board fits, its half-size scaled by
    growth() at the aim (it grows off axis). None if the board that fits no
    longer reaches the cell (too big at this distance)."""
    x0, y0, x1, y1 = _cell_rect(cell, grid)
    half0 = np.array([(bbox[2] - bbox[0]) / 2, (bbox[3] - bbox[1]) / 2]) / growth(g, centroid, image)
    c, size = _centre(image), np.array([image["width"], image["height"]], dtype=float)

    def half(at):
        return half0 * growth(g, at, image) * FIT_SAFETY

    def fits(at):
        h = half(at)
        return bool(np.all(at - h >= margin) and np.all(at + h <= size - margin))

    want = np.array([(x0 + x1) / 2, (y0 + y1) / 2])
    if not fits(c):
        return None
    if fits(want):
        aim = want
    else:
        lo, hi = 0.0, 1.0                    # fraction of the way from c to want
        for _ in range(30):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if fits(c + mid * (want - c)) else (lo, mid)
        aim = c + lo * (want - c)
    h = half(aim) / FIT_SAFETY
    if aim[0] - h[0] >= x1 or aim[0] + h[0] <= x0 or aim[1] - h[1] >= y1 or aim[1] + h[1] <= y0:
        return None
    return float(aim[0]), float(aim[1])


def refit_aim(aim, before, after, image, margin):
    """aim, moved in if needed so the board stays margin inside the image,
    from two boards measured on the way there. A tilted board isn't symmetric
    about its centroid and its near side grows fastest (recorded at cell 29:
    249 px below the centroid, 182 above), which aim_for_cell() can't know
    from the base view. Each side's reach from the centroid is extrapolated
    along the move from how it grew so far."""
    c0, c1 = np.array(before["centroid_px"]), np.array(after["centroid_px"])
    reach = lambda b, c: np.array([c - b["bbox_px"][:2], np.array(b["bbox_px"][2:]) - c])
    r0, r1 = reach(before, c0), reach(after, c1)
    on = np.linalg.norm(np.asarray(aim) - c1) / max(np.linalg.norm(c1 - c0), 1.0)
    r = r1 + np.maximum(r1 - r0, 0.0) * on
    lo = margin + r[0]
    hi = np.array([image["width"], image["height"]], dtype=float) - margin - r[1]
    out = np.minimum(np.maximum(aim, lo), hi)
    return float(out[0]), float(out[1])


def corner_cells(grid):
    """Pointing the camera at a frame corner tilts the board compoundly,
    which the four quadrant bins need."""
    c, r = grid["cols"], grid["rows"]
    return [0, c - 1, c * (r - 1), c * r - 1]


def centre_cells(grid):
    """The middle column's two central cells: near-centre views for the flat
    bin, far enough from a centred base (~85 px) for capture's same-pose gate."""
    c, r = grid["cols"], grid["rows"]
    return [(r // 2 - 1) * c + c // 2, (r // 2) * c + c // 2]


def _band_of(square, sq0, frac):
    """Planner's band, relative to P0's square size: mid within +-frac."""
    r = square / sq0
    return "near" if r > 1 + frac else "far" if r < 1 - frac else "mid"


def capture_band(square, acfg):
    """The band capture will put a view of this square size in (fixed
    thresholds, config/autocapture.yaml). Bins and its end condition use it."""
    return ("near" if square >= acfg["near_square_px"] else
            "far" if square < acfg["far_square_px"] else "mid")


def build_variants(p0, s0, g, scfg, acfg):
    """Three depths along TOOL z (P0, and depth_m each way) x {no offset,
    +-lateral_m along the probe's "right" and "down" translation axes}, each
    with the rotation the gains predict keeps the board where it was. Named
    by the planner's band, from the predicted square size relative to P0's
    (mid = within +-mid_band_fraction), and the depth: "near(z+0.10)/+x"."""
    sq0 = s0["board"]["square_px"]
    k_z = float(g["square_per_m"][2])            # square px per m along tool z
    if abs(k_z) < 1.0:
        raise ProbeError(f"the probe saw almost no size change along tool z ({k_z:+.2f} px/m); "
                         "depth variants can't be named. Rerun the probe.")
    lat_r, lat_d = g["lateral_axes"]
    offsets = [("centre", None, 0)] + [(f"{'+' if s > 0 else '-'}{a}", a, s)
                                       for a in (lat_r, lat_d) if a != "z" for s in (1, -1)]
    out = []
    for dz in (0.0, scfg["depth_m"], -scfg["depth_m"]):
        tag = "z0" if dz == 0 else f"z{dz:+.2f}"
        for name, axis, sgn in offsets:
            t = np.zeros(3)
            t[2] = dz
            if axis:
                t["xyz".index(axis)] += sgn * scfg["lateral_m"]
            rot, _ = rotation_for_shift(g, -(g["J_trans"] @ t))
            square = sq0 + float(g["square_per_m"] @ t)
            band = _band_of(square, sq0, scfg["mid_band_fraction"])
            out.append({"name": f"{band}({tag})/{name}", "band": band, "depth": tag,
                        "t_m": t.tolist(), "rot_deg": rot, "pose": compose(p0, rot, t),
                        "predicted_square_px": square,
                        "capture_band": capture_band(square, acfg), "skip": None})
    return out


def estimate_board(p0, s0, g, square_m):
    """The board centre from one status: depth = f * square_m / square_px
    along the ray through the centroid, into the tool frame by cam_axes, into
    the base frame by p0. The camera's offset from the TCP isn't measured,
    so the camera is taken to be at the TCP. The orbit is unaffected to first
    order: it turns the tool about this point, which is rigid with the tool.
    size_gain_distance_m is a second estimate that needs no focal length."""
    b = s0["board"]
    f = float(np.sqrt(np.prod(g["f_px"])))
    depth = f * square_m / b["square_px"]
    ray = np.array([*((np.array(b["centroid_px"]) - _centre(s0["image"])) / g["f_px"]), 1.0])
    tool = g["cam_axes"] @ (ray * depth)
    base = np.array(p0[:3]) + euler_to_R(*p0[3:]) @ tool
    return {"tool_m": [round(float(v), 4) for v in tool], "base_m": [round(float(v), 4) for v in base],
            "distance_m": round(float(np.linalg.norm(tool)), 4), "depth_m": round(depth, 4),
            "focal_px": round(f, 1), "square_px": b["square_px"], "square_m": square_m,
            "size_gain_distance_m": round(b["square_px"] / float(np.linalg.norm(g["square_per_m"])), 4)}


def _rotate_onto(a, b):
    """Smallest rotation matrix taking unit vector a to unit vector b."""
    axis = np.cross(a, b)
    n = float(np.linalg.norm(axis))
    if n < 1e-12:
        return np.eye(3)
    return cv2.Rodrigues(axis / n * math.atan2(n, float(a @ b)))[0]


def orbit_viewpoints(scfg):
    """(radius offset, elevation, azimuth, roll) for the whole cap, as one
    nearest-first walk from the centre so each move is short. Every
    orbit_roll_every-th off-centre viewpoint takes the next non-zero roll."""
    todo = [(el, az) for el in scfg["orbit_angles_deg"] for az in scfg["orbit_angles_deg"]]
    rolls = [r for r in scfg["orbit_rolls_deg"] if r != 0]
    every = int(scfg["orbit_roll_every"])
    out, cur, k = [], (0.0, 0.0), 0
    for dr in scfg["orbit_radii_m"]:
        left = list(todo)
        while left:
            cur = min(left, key=lambda v: math.dist(v, cur))
            left.remove(cur)
            roll = 0.0
            if cur != (0.0, 0.0):
                k += 1
                if rolls and k % every == 0:
                    roll = rolls[(k // every - 1) % len(rolls)]
            out.append((dr, *cur, roll))
    return out


def build_orbit(p0, s0, g, scfg, acfg, board):
    """Viewpoints on a spherical cap around the estimated board centre, as
    variants. Azimuth is about base z through the board, elevation about the
    horizontal axis across the view; both start from P0's line of sight.
    Each pose points the optical axis (cam_axes, not tool z) at the board."""
    centre = np.array(board["base_m"])
    pos0 = np.array(p0[:3], dtype=float)
    v0 = (pos0 - centre) / np.linalg.norm(pos0 - centre)
    r0 = float(np.linalg.norm(pos0 - centre))
    R0 = euler_to_R(*p0[3:])
    aimed = _rotate_onto(R0 @ g["cam_axes"][:, 2], -v0) @ R0
    up = np.array([0.0, 0.0, 1.0])
    across = np.cross(v0, up)
    across = across / np.linalg.norm(across)
    sq0 = s0["board"]["square_px"]
    out = []
    for dr, el, az, roll in orbit_viewpoints(scfg):
        Q = cv2.Rodrigues(up * math.radians(az))[0] @ cv2.Rodrigues(across * math.radians(el))[0]
        v = Q @ v0
        R = cv2.Rodrigues(-v * math.radians(roll))[0] @ Q @ aimed
        pose = tuple(float(x) for x in centre + (r0 + dr) * v) + R_to_euler(R)
        square = sq0 * r0 / (r0 + dr)
        band = _band_of(square, sq0, scfg["mid_band_fraction"])
        tag = "r0" if dr == 0 else f"r{dr:+.2f}"
        name = "centre" if (el, az, roll) == (0, 0, 0) else f"el{el:+.0f}az{az:+.0f}" + (
            f"ro{roll:+.0f}" if roll else "")
        rel = relative(p0, pose)
        out.append({"name": f"{band}({tag})/{name}", "band": band, "depth": tag,
                    "t_m": rel["t_m"], "rot_deg": rel["rotvec_deg"], "pose": pose,
                    "predicted_square_px": square, "capture_band": capture_band(square, acfg),
                    "skip": None, "orbit": {"radius_m": round(r0 + dr, 4), "elevation_deg": el,
                                            "azimuth_deg": az, "roll_deg": roll}})
    return out


def pointing_error_deg(pose, board_base, g):
    """Angle between the optical axis at pose and the line from the TCP to
    the board centre."""
    axis = euler_to_R(*pose[3:]) @ g["cam_axes"][:, 2]
    to = np.array(board_base) - np.array(pose[:3])
    return math.degrees(math.atan2(np.linalg.norm(np.cross(axis, to)), float(axis @ to)))


# ---------------------------------------------------------------- taught poses

# (left-right, high-low, near-far) index of each corner, 0 or 1, in teaching order.
TAUGHT = {"near_left_high": (0, 0, 0), "near_right_high": (1, 0, 0),
          "near_left_low": (0, 1, 0), "near_right_low": (1, 1, 0),
          "far_left_high": (0, 0, 1), "far_right_high": (1, 0, 1),
          "far_left_low": (0, 1, 1), "far_right_low": (1, 1, 1)}
TEACH_MARGIN_DEG = 10.0   # a taught pose stays this far inside each joint limit
FAR_NEAR_RATIO_MAX = 0.6  # far square size / near square size should be under this
MAX_SPREAD_DEG = 170.0    # taught angles nearer half a turn apart have no clear shortest way


def joint_margin(joints):
    """Reasons a joint is inside TEACH_MARGIN_DEG of its limit; empty if clear."""
    why = []
    for actuator, limit in armlib.JOINT_LIMITS_DEG.items():
        a = armlib.wrap180(joints[actuator - 1])
        if limit - abs(a) < TEACH_MARGIN_DEG:
            why.append(f"joint {actuator} at {a:+.1f} deg is {limit - abs(a):.1f} deg from its "
                       f"limit +-{limit:g}; need {TEACH_MARGIN_DEG:g}")
    return why


def unwrapped(vectors):
    """Joint vectors in deg, each joint on the side of the first vector's that
    makes the difference under half a turn, so interpolation takes the short way."""
    base = np.array(vectors[0], dtype=float)
    return [base + [armlib.wrap180(a - b) for a, b in zip(v, base)] for v in vectors]


def joint_spread(vectors):
    """(largest max-min over the joints, deg; its actuator number)."""
    q = np.array(unwrapped(vectors))
    d = q.max(axis=0) - q.min(axis=0)
    return float(d.max()), int(d.argmax()) + 1


def far_near_ratio(squares):
    """Mean far square size over mean near. squares: {taught name: px}."""
    far = [v for k, v in squares.items() if k.startswith("far")]
    near = [v for k, v in squares.items() if k.startswith("near")]
    return float(np.mean(far) / np.mean(near))


def trilinear(corners, u, v, w):
    """corners {(lr, hl, nf): value}, each index 0 or 1; u, v, w in [0, 1]."""
    return sum(c * (u if i else 1 - u) * (v if j else 1 - v) * (w if k else 1 - w)
               for (i, j, k), c in corners.items())


def serpentine(n_lr, n_hl, n_nf):
    """Every (lr, hl, nf) grid index once, each one grid step from the last."""
    out, row = [], 0
    for k in range(n_nf):
        for j in (range(n_hl) if k % 2 == 0 else reversed(range(n_hl))):
            for i in (range(n_lr) if row % 2 == 0 else reversed(range(n_lr))):
                out.append((i, j, k))
            row += 1
    return out


def taught_poses(path=None):
    """The eight corners from config/arm_poses.yaml, in teaching order."""
    poses = armlib.load_poses(path)
    missing = [n for n in TAUGHT if n not in poses or "square_px" not in poses[n]]
    if missing:
        raise ProbeError(f"taught poses missing: {', '.join(missing)}. Run arm_capture.py teach.")
    return {n: poses[n] for n in TAUGHT}


def build_taught(arm, poses, p0, scfg, acfg):
    """The viewpoints: grid over the eight taught joint vectors, serpentine."""
    spread, actuator = joint_spread([p["joints_deg"] for p in poses.values()])
    if spread > MAX_SPREAD_DEG:
        raise ProbeError(f"taught poses span {spread:.0f} deg on joint {actuator}: too near half "
                         "a turn for a short way between them. Teach them closer.")
    q = dict(zip(TAUGHT.values(), unwrapped([p["joints_deg"] for p in poses.values()])))
    sq = {TAUGHT[n]: p["square_px"] for n, p in poses.items()}
    n = [scfg[k] for k in GRID_KEYS]
    out = []
    for i, j, k in serpentine(*n):
        u, v, w = i / (n[0] - 1), j / (n[1] - 1), k / (n[2] - 1)
        joints = [float(a) for a in trilinear(q, u, v, w)]
        pose = arm.forward_kinematics([a % 360.0 for a in joints])
        square = float(trilinear(sq, u, v, w))
        band = capture_band(square, acfg)
        rel = relative(p0, pose)
        out.append({"name": f"lr{i}_hl{j}_nf{k}", "grid": [i, j, k], "joints_deg": joints,
                    "band": band, "depth": f"nf{k}", "t_m": rel["t_m"], "rot_deg": rel["rotvec_deg"],
                    "pose": pose, "predicted_square_px": square, "capture_band": band,
                    "skip": None})
    return out


def run_teach(connect, lim, status_path, force=False, ask=input, log=print, poses_path=None):
    """
    Teach the eight corners. connect() returns an Arm context manager, opened
    once per pose so nothing here is connected while the operator jogs. A pose
    that fails a check is asked for again. Refuses before the first prompt if
    any name exists, unless force. Returns True once all eight are saved.

    Base.Stop or the session failing means the Web App still holds the arm.
    """
    existing = [n for n in TAUGHT if n in armlib.load_poses(poses_path)]
    if existing and not force:
        raise ProbeError(f"already taught: {', '.join(existing)}; nothing changed. Add --force "
                         "to teach them again.")
    _fresh(status_path)
    squares, vectors = {}, []
    for name in TAUGHT:
        while True:
            if ask(f"jog to {name}, close the Web App, press Enter (q to quit) ").strip() == "q":
                log(f"stopped: {len(vectors)}/{len(TAUGHT)} saved in this run")
                return False
            try:
                with connect() as arm:
                    arm.stop()
                    mode = arm.servoing_mode()
                    if mode != "SINGLE_LEVEL_SERVOING":
                        raise armlib.ArmError(f"servoing mode is {mode}")
                    frame = _fresh(status_path)["frame"]
                    r = settled_reading(status_path, frame)
                    q, pose = [v for _, v in arm.measured_joints()], arm.measured_pose()
            except armlib.ArmError as e:
                log(f"REFUSED    {name}: the arm isn't free ({e}). Is the Web App closed?")
                continue
            except ProbeError as e:
                log(f"REFUSED    {name}: {e}")
                continue
            why = joint_margin(q)
            if why:
                log(f"REFUSED    {name}: " + "; ".join(why))
                continue
            for w in armlib.outside_box(lim, pose):
                log(f"WARNING    {name}: {w}; viewpoints out there will be skipped")
            singular = armlib.wrist_singular(q)
            if singular:
                log(f"WARNING    {name}: {singular}; viewpoints near it will be skipped")
            armlib.store_pose(name, q, pose, True, r.square, poses_path)
            squares[name] = r.square
            vectors.append(q)
            log(f"saved      {name}: square {r.square:.1f} px  {armlib.fmt_pose(pose)}")
            break
    ratio = far_near_ratio(squares)
    spread, actuator = joint_spread(vectors)
    log(f"\nfar/near   square size ratio {ratio:.2f} (want < {FAR_NEAR_RATIO_MAX:g})")
    log(f"spread     largest joint-space spread {spread:.1f} deg, joint {actuator}")
    if ratio >= FAR_NEAR_RATIO_MAX:
        log("WARNING    the near and far poses are too alike: teach them further apart")
    if spread > MAX_SPREAD_DEG:
        log(f"WARNING    a session refuses a spread over {MAX_SPREAD_DEG:g} deg")
    return True


def depth_sanity(variants, sq0, g, scfg):
    """Warnings if the depth variants don't separate in size: near vs far
    must differ by more than band_spread_min of P0's square size."""
    by = {v["depth"]: v["predicted_square_px"] for v in variants if v["name"].endswith("/centre")}
    sq = sorted(by.values())
    spread = (sq[-1] - sq[0]) / sq0
    k = abs(float(g["square_per_m"][2]))
    out = []
    if spread <= scfg["band_spread_min"]:
        out.append(f"near and far differ by only {spread:.1%} in square size (want > "
                   f"{scfg['band_spread_min']:.0%}): depth_m {scfg['depth_m']:g} m at "
                   f"{k:.1f} px/m on {sq0:.1f} px. depth_m >= "
                   f"{scfg['band_spread_min'] * sq0 / (2 * k):.3f} m would separate them.")
    named = {v["band"] for v in variants if v["name"].endswith("/centre")}
    if named != {"near", "mid", "far"}:
        out.append(f"the depth variants are all named {'/'.join(sorted(named))}: +-depth_m stays "
                   f"inside the +-{scfg['mid_band_fraction']:.0%} mid band; depth_m >= "
                   f"{scfg['mid_band_fraction'] * sq0 / k:.3f} m leaves it.")
    return out


def interpolate(a, b, f):
    """Pose a fraction f of the way from a to b: position linear, rotation
    along the shortest arc."""
    Ra, Rb = euler_to_R(*a[3:]), euler_to_R(*b[3:])
    rv = cv2.Rodrigues(Ra.T @ Rb)[0]
    R = Ra @ cv2.Rodrigues(rv * f)[0]
    p = np.array(a[:3]) + f * (np.array(b[:3]) - np.array(a[:3]))
    return tuple(float(v) for v in p) + R_to_euler(R)


def halfway(a, b):
    """Midpoint of two poses: position mean, half the relative rotation."""
    return interpolate(a, b, 0.5)


IK_SAMPLES = 4            # IK checked at 1/4, 2/4, 3/4 and the end of each move
IK_JOINT_MARGIN_DEG = 5.0  # stay this far inside the guide's joint limits
IK_JUMP_DEG = 45.0        # a joint moving more than this between samples: branch change


def ik_check(arm, start, target, joints_now):
    """None if the arm's own IK finds a solution at IK_SAMPLES points along
    start -> target (each seeded with the previous solution, the first with
    the measured joints), every one inside JOINT_LIMITS_DEG less the margin,
    with no joint jumping IK_JUMP_DEG between samples. Otherwise the reason.
    Returns (reason, final joint solution or None)."""
    guess = list(joints_now)
    for k in range(1, IK_SAMPLES + 1):
        f = k / IK_SAMPLES
        try:
            q = arm.inverse_kinematics(interpolate(start, target, f), guess)
        except armlib.IKFailed as e:
            return f"IK failed at {f:.0%} of the path ({e})", None
        qw = [armlib.wrap180(v) for v in q]
        for actuator, lim in armlib.JOINT_LIMITS_DEG.items():
            if actuator <= len(qw) and abs(qw[actuator - 1]) > lim - IK_JOINT_MARGIN_DEG:
                return (f"joint {actuator} at {qw[actuator - 1]:+.1f} deg at {f:.0%} of the path "
                        f"(limit +-{lim:g}, margin {IK_JOINT_MARGIN_DEG:g})"), q
        jumps = [abs(armlib.wrap180(a - b)) for a, b in zip(q, guess)]
        if max(jumps) > IK_JUMP_DEG:
            j = int(np.argmax(jumps))
            return (f"joint {j + 1} jumps {jumps[j]:.0f} deg near {f:.0%} of the path "
                    "(IK branch change: near a singularity)"), q
        guess = q
    return None, guess


# ---------------------------------------------------------------- session

class SessionStop(Exception):
    """An orderly end: the session returns to P0."""


class _Skip(Exception):
    """The target is given up and its entry finished; the arm is at `here`."""

    def __init__(self, here):
        super().__init__("skipped")
        self.here = here


class TargetAborted(Exception):
    """The arm aborted (or refused) a validated target. Not fatal on its own."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


MAX_CONSECUTIVE_ABORTS = 3


class _Keys:
    """Non-blocking 'q' from the terminal, if stdin is one."""

    def __enter__(self):
        self.fd = None
        if sys.stdin.isatty():
            import termios, tty
            self.fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        if self.fd is not None:
            import termios
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        return False

    def quit_requested(self):
        while self.fd is not None and select.select([self.fd], [], [], 0)[0]:
            if os.read(self.fd, 1) in (b"q", b"Q"):
                return True
        return False


# Where the time goes, per target entry (seconds): validating (ik_s), the arm
# moving (move_s, ExecuteAction to ACTION_END), waiting for a full still
# board (still_s), waiting for capture's decision (decide_s).
TIMERS = ("ik_s", "move_s", "still_s", "decide_s")


def timing_breakdown(entries, total_s, moves):
    """Totals and fractions of TIMERS over entries, per move and per session.
    Reads records from before 2026-10-29 too (settle_s = still_s)."""
    sums = dict.fromkeys(TIMERS, 0.0)
    for e in entries:
        t = e.get("times", {})
        for k in TIMERS:
            sums[k] += t.get(k, t.get("settle_s", 0.0) if k == "still_s" else 0.0)
    other = max(0.0, total_s - sum(sums.values()))
    out = {k: round(v, 3) for k, v in sums.items()}
    out["other_s"] = round(other, 3)
    out["fractions"] = {k: round(v / total_s, 3) if total_s > 0 else 0.0
                        for k, v in list(sums.items()) + [("other_s", other)]}
    out["per_move_s"] = round(total_s / moves, 3) if moves else None
    out["motion_per_move_s"] = round(sums["move_s"] / moves, 3) if moves else None
    return out


class _Session:
    def __init__(self, arm, lim, status_path, g, scfg, acfg, keys, log):
        self.arm, self.lim, self.path = arm, lim, status_path
        self.g, self.scfg, self.acfg, self.keys, self.log = g, scfg, acfg, keys, log
        self.t0 = time.monotonic()
        self.entries = []
        self.moves = 0
        self.aborts = 0           # consecutive: targets aborted since the last one reached
        self.last = None
        self.record = None
        self.left = ""            # live "targets left" text for the log line

    def _time(self, e, key, since):
        e["times"][key] = round(e["times"].get(key, 0.0) + time.monotonic() - since, 3)

    # ---- camera side

    def live(self):
        """Fresh status. A stale file means capture died, unless its last word
        was that the session is complete (capture exits on completion)."""
        try:
            s = read_status(self.path)
        except StatusError as e:
            raise SessionStop(f"capture died: {e}")
        if age_s(s) > STALE_S:
            if s["complete"]:
                raise SessionStop("complete (capture ended)")
            raise SessionStop(f"capture died: status file {age_s(s):.1f} s old")
        self.last = s
        if self.keys and self.keys.quit_requested():
            raise SessionStop("operator pressed q")
        return s

    def settle(self, after_frame, e):
        """('ok' | 'lost' | 'timeout', status) for frames SETTLE_FRAMES past after_frame."""
        t = time.monotonic()
        deadline = t + self.scfg["still_timeout_s"]
        lost = 0
        s = self.live()
        try:
            while time.monotonic() < deadline:
                s = self.live()
                if s["frame"] >= after_frame + SETTLE_FRAMES:
                    if full_and_still(s):
                        return "ok", s
                    c = s["corners"]
                    lost = lost + 1 if c["found"] < c["total"] else 0
                    if lost >= LOST_FRAMES:
                        return "lost", s
                time.sleep(POLL_S)
            return "timeout", s
        finally:
            self._time(e, "still_s", t)

    def await_decision(self, accepted_before, e):
        """After min_interval, up to ACCEPT_WAIT_FRAMES frames for an accept."""
        t = time.monotonic()
        time.sleep(self.acfg["min_interval_ms"] / 1000.0)
        s = self.live()
        start = s["frame"]
        while s["accepted"] <= accepted_before and s["frame"] < start + ACCEPT_WAIT_FRAMES:
            time.sleep(POLL_S)
            s = self.live()
        self._time(e, "decide_s", t)
        return s, s["accepted"] > accepted_before

    # ---- arm side

    def move(self, target, e, is_target=True):
        """Box check, then IK validation along the path, then the move. False
        (entry marked refused / unreachable) if either check fails. Raises
        TargetAborted on ACTION_ABORT or a refused ExecuteAction; any other
        arm failure is fatal (ArmError). A reached target (is_target) resets
        the consecutive-abort count; a return move doesn't."""
        current = self.arm.measured_pose()
        try:
            target = armlib.check_move(self.arm, self.lim, current, target, show=False)
        except armlib.ArmError as ex:
            e["outcome"] = "refused"
            e["note"] = str(ex).replace("REFUSED, nothing sent:\n  ", "")
            return False
        t = time.monotonic()
        joints = [v for _, v in self.arm.measured_joints()]
        why, q = ik_check(self.arm, current, target, joints)
        self._time(e, "ik_s", t)
        cmd = {"abs": [round(v, 5) for v in target], "rel_p0": relative(self.p0, target),
               "joints_before_deg": [round(v, 2) for v in joints],
               "ik_deg": None if q is None else [round(v, 2) for v in q]}
        e.setdefault("commanded", []).append(cmd)
        if why:
            cmd["sent"] = False
            e["outcome"] = "unreachable"
            e["note"] = why
            return False
        cmd["sent"] = True
        self.moves += 1
        t = time.monotonic()
        try:
            result, reason = self.arm.reach_pose(target, self.lim)
        except armlib.ActionRefused as ex:
            result, reason = "abort", f"ExecuteAction refused: {ex}"
        finally:
            self._time(e, "move_s", t)
        if result == "abort":
            raise TargetAborted(reason or "no reason given")
        if result != "end":
            raise armlib.ArmError(f"{result.upper()}" + (f" ({reason})" if reason else "") +
                                  ("; Base.Stop sent" if result == "timeout" else ""))
        if is_target:
            self.aborts = 0
        return True

    def new_entry(self, kind, variant, cell=None, aim=None):
        e = {"i": len(self.entries), "kind": kind, "variant": variant, "cell": cell,
             "aim_px": None if aim is None else [round(aim[0], 1), round(aim[1], 1)],
             "outcome": None, "note": None,
             "times": {"start_s": round(time.monotonic() - self.t0, 3)}}
        self.entries.append(e)
        return e

    def finish(self, e, s, accepted=None):
        e["measured"] = {"abs": [round(v, 5) for v in self.arm.measured_pose()]}
        e["measured"]["rel_p0"] = relative(self.p0, e["measured"]["abs"])
        if s is not None:
            b = s["board"]
            e["camera"] = {"frame": s["frame"], "corners": s["corners"]["found"],
                           "centroid_px": None if b is None else b["centroid_px"],
                           "verdict": s["decision"]["verdict"], "reason": s["decision"]["reason"],
                           "bin": s["decision"]["bin"], "accepted": accepted}
        e["times"]["total_s"] = round(time.monotonic() - self.t0 - e["times"]["start_s"], 3)
        self.write()
        if s is None:
            cam = ""
        elif accepted:
            cam = f"  ACCEPTED ({s['decision']['bin']})"
        else:
            cam = f"  {s['decision']['verdict']}: {s['decision']['reason']}"
        self.log(f"  [{e['i']:3d}] {e['kind']:<8} {e['variant']:<26} "
                 f"{'' if e['cell'] is None else 'cell %-3d' % e['cell']} {e['outcome']}{cam}"
                 + (f"  ({e['note']})" if e["note"] else "") + self.left)

    def write(self):
        if self.record is None:
            return
        self.record["poses"] = self.entries
        tmp = self.record_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.record, f, indent=1)
        os.replace(tmp, self.record_path)

    # ---- one target

    def move_joints(self, joints, e, is_target=True):
        """A joint move at the viewpoint speeds: check_joint_move(), then send.
        False (entry marked refused) if it fails; otherwise as move()."""
        current = self.arm.measured_pose()
        now = [v for _, v in self.arm.measured_joints()]
        try:
            q, target, dps = armlib.check_joint_move(self.arm, self.lim, current, now, joints,
                                                     show=False, viewpoint=True)
        except armlib.ArmError as ex:
            e["outcome"] = "refused"
            e["note"] = str(ex).replace("REFUSED, nothing sent:\n  ", "")
            return False
        e.setdefault("commanded", []).append(
            {"abs": [round(v, 5) for v in target], "rel_p0": relative(self.p0, target),
             "joints_before_deg": [round(v, 2) for v in now],
             "joints_deg": [round(v, 2) for v in q], "joint_speed_dps": round(dps, 2), "sent": True})
        self.moves += 1
        t = time.monotonic()
        try:
            result, reason = self.arm.reach_joints(q, self.lim, dps)
        except armlib.ActionRefused as ex:
            result, reason = "abort", f"ExecuteAction refused: {ex}"
        finally:
            self._time(e, "move_s", t)
        if result == "abort":
            raise TargetAborted(reason or "no reason given")
        if result != "end":
            raise armlib.ArmError(f"{result.upper()}" + (f" ({reason})" if reason else "") +
                                  ("; Base.Stop sent" if result == "timeout" else ""))
        if is_target:
            self.aborts = 0
        return True

    def visit(self, kind, variant, target, here, cell=None, aim=None, base=None, joints=None):
        """Move to target, or for a cell to aim from base (pose, centroid) on,
        settle, read capture's decision. Returns (status, pose now): status
        None if the target was skipped, unreachable or aborted. After an abort
        the arm goes back to `here`, the last pose reached. joints: reach a
        viewpoint by this joint move instead; the next viewpoint is a joint
        move too, so a failed one is left as it is."""
        e = self.new_entry(kind, variant, cell, aim)
        try:
            return self._visit(e, target, here, aim, base, joints)
        except _Skip as ex:
            return None, ex.here
        except TargetAborted as ex:
            self.aborts += 1
            e["outcome"] = f"aborted: {ex.reason}"
            e["joints_after_abort_deg"] = [round(v, 2) for _, v in self.arm.measured_joints()]
            self.finish(e, None)
            if joints is not None:
                if self.aborts >= MAX_CONSECUTIVE_ABORTS:
                    raise armlib.ArmError(f"{self.aborts} consecutive aborts; last: {ex.reason}")
                return None, self.arm.measured_pose()
            self.recover(here, variant, ex.reason)
            return None, here

    def recover(self, here, variant, reason):
        """After an abort: stop at MAX_CONSECUTIVE_ABORTS, else return to the
        last reached pose (aborts there count too)."""
        while True:
            if self.aborts >= MAX_CONSECUTIVE_ABORTS:
                raise armlib.ArmError(f"{self.aborts} consecutive aborts; last: {reason}")
            e = self.new_entry("back", variant)
            e["note"] = "after an abort, to the last reached pose"
            try:
                if not self.move(here, e, is_target=False):
                    self.finish(e, None)
                    raise armlib.ArmError(f"after an abort, the way back to the last reached "
                                          f"pose failed its checks: {e['note']}")
            except TargetAborted as ex:
                self.aborts += 1
                reason = ex.reason
                e["outcome"] = f"aborted: {ex.reason}"
                self.finish(e, None)
                continue
            res, s = self.settle(self.live()["frame"], e)
            e["outcome"] = "reached" if res == "ok" else f"reached; board {res}"
            self.finish(e, s)
            if res != "ok":
                raise SessionStop("board not in view at the last reached pose after an abort")
            return

    def _visit(self, e, target, here, aim, base, joints=None):
        accepted_before = self.live()["accepted"]
        if joints is not None:
            s, target = self._reach_joints(e, joints, here)
        elif aim is None:
            s, target = self._reach(e, target, here)
        else:
            s, target = self._aim(e, here, aim, base)
        s, accepted = self.await_decision(accepted_before, e)
        e["outcome"] = "reached"
        self.finish(e, s, accepted)
        return s, target

    def _reach_joints(self, e, joints, here):
        """Joint move and settle. Returns (status, pose reached); raises _Skip
        with the entry finished if the viewpoint is given up."""
        if not self.move_joints(joints, e):
            self.finish(e, None)
            raise _Skip(here)
        res, s = self.settle(self.live()["frame"], e)
        reached = self.arm.measured_pose()
        if res != "ok":
            e["outcome"] = f"skipped: board {res} here"
            self.finish(e, s)
            raise _Skip(reached)
        return s, reached

    def _reach(self, e, target, here):
        """Move and settle. Returns (status, pose reached); raises _Skip with
        the entry finished if the target is given up."""
        if not self.move(target, e):
            self.finish(e, None)
            raise _Skip(here)
        res, s = self.settle(self.live()["frame"], e)
        if res == "lost":
            # Step halfway back toward where the board was in view; one retry.
            mid = halfway(here, target)
            e["note"] = "board left the view; halfway back"
            if self.move(mid, e):
                res, s = self.settle(self.live()["frame"], e)
                target = mid
            if res != "ok":
                e["outcome"] = "skipped: board left the view"
                if self.move(here, e, is_target=False):
                    back, _ = self.settle(self.live()["frame"], e)
                    if back != "ok":
                        raise SessionStop("board not back in view after returning from a "
                                          "skipped target")
                self.finish(e, s)
                raise _Skip(here)
        if res == "timeout":
            e["outcome"] = "skipped: not still within %gs" % self.scfg["still_timeout_s"]
            self.finish(e, s)
            full = s["corners"]["found"] == s["corners"]["total"]
            raise _Skip(target if full else here)
        return s, target

    def _aim_error(self, e, leg, aim, s):
        """Record and log where the move left the board. Returns the miss, px."""
        c = s["board"]["centroid_px"]
        err = [round(c[0] - aim[0], 1), round(c[1] - aim[1], 1)]
        norm = round(float(np.hypot(*err)), 1)
        e.setdefault("aim_error", []).append({"leg": leg, "centroid_px": [round(v, 1) for v in c],
                                              "error_px": err, "miss_px": norm})
        self.log(f"        aim {leg:<10} ({c[0]:6.1f}, {c[1]:6.1f}) px, error "
                 f"({err[0]:+6.1f}, {err[1]:+6.1f}), miss {norm:.1f} px")
        return norm

    def _aim_pose(self, here, base_pose, aim, now, image, scale=1.0):
        """Pose that turns the board from `now` toward `aim` from here, with
        the total rotation from the base pose capped at max_rotation_deg."""
        pose = compose(here, rotation_for_aim(self.g, aim, now, image, scale))
        R = euler_to_R(*base_pose[3:]).T @ euler_to_R(*pose[3:])
        rv = np.degrees(cv2.Rodrigues(R)[0].ravel())
        norm, cap = float(np.linalg.norm(rv)), self.scfg["max_rotation_deg"]
        if norm <= cap:
            return pose
        return compose(base_pose, {f"theta_{k}": float(v) * cap / norm for k, v in zip("xyz", rv)})

    def _aim(self, e, here, aim, base):
        """Turn the board onto aim: from the centroid now, in two moves if it
        is further than two_move_px (the aim is refitted to the board seen
        after the first), then one correction if the last move missed by
        more than centre_tol_px."""
        base_pose, base_c = base
        s = self.live()
        if full_and_still(s):
            now = s["board"]["centroid_px"]
        else:
            now = base_c
            here = base_pose
        image = s["image"]
        if math.dist(aim, now) > self.scfg["two_move_px"]:
            first = self._aim_pose(here, base_pose, aim, now, image, self.scfg["first_move_fraction"])
            board0 = s["board"]
            s, here = self._reach(e, first, here)
            self._aim_error(e, "first", aim, s)
            now = s["board"]["centroid_px"]
            aim2 = refit_aim(aim, board0, s["board"], image, self.scfg["edge_margin_px"])
            if aim2 != tuple(aim):
                aim = aim2
                e["aim_refit_px"] = [round(aim[0], 1), round(aim[1], 1)]
        s, target = self._reach(e, self._aim_pose(here, base_pose, aim, now, image), here)
        miss = self._aim_error(e, "final", aim, s)
        if miss <= self.scfg["centre_tol_px"]:
            return s, target
        e["note"] = f"missed aim by {miss:.0f} px; corrected"
        corrected = self._aim_pose(target, base_pose, aim, s["board"]["centroid_px"], image)
        if not self.move(corrected, e):
            return s, target
        res, s2 = self.settle(self.live()["frame"], e)
        if res == "ok":
            self._aim_error(e, "correction", aim, s2)
            return s2, corrected
        e["note"] += "; correction lost the board, back to the last good pose"
        self.move(target, e, is_target=False)
        res, s2 = self.settle(self.live()["frame"], e)
        if res != "ok":
            e["outcome"] = "skipped: board not back after a correction"
            self.finish(e, s2)
            raise _Skip(here)
        return s2, target


def _flat_short(s, band):
    return s["pose_bins"]["counts"][f"{band}/flat"] < s["pose_bins"]["min_per_bin"]


def _quad_short(s, band):
    m = s["pose_bins"]["min_per_bin"]
    return any(n < m for k, n in s["pose_bins"]["counts"].items()
               if k.startswith(band + "/") and not k.endswith("/flat"))


POSE_REACHED_DEG = 0.5    # every joint this close to a saved pose: already there


def _short_bins(s):
    m = s["pose_bins"]["min_per_bin"]
    return [k for k, n in s["pose_bins"]["counts"].items() if n < m]


def _off_p0(pose, g, scfg):
    """Why pose is too far from the probe's start pose, or None."""
    d_m = math.dist(pose[:3], g["start_pose"][:3])
    rv = cv2.Rodrigues(euler_to_R(*g["start_pose"][3:]).T @ euler_to_R(*pose[3:]))[0]
    d_deg = math.degrees(float(np.linalg.norm(rv)))
    if d_m > scfg["p0_tolerance_m"] or d_deg > scfg["p0_tolerance_deg"]:
        return (f"{d_m * 1000:.0f} mm / {d_deg:.1f} deg from the probe's start pose "
                f"(limits {scfg['p0_tolerance_m'] * 1000:.0f} mm / "
                f"{scfg['p0_tolerance_deg']:g} deg); rerun the probe here")
    return None


def go_to_saved_pose(arm, lim, status_path, name, g, scfg, yes, log, poses_path=None):
    """
    session --from-pose: move in joint space to a saved pose. Returns True
    with the arm there and the board settled; False for a dry run that has to
    stop because the arm isn't there yet, as the session plan is built from
    the arm's pose and what the camera sees from it. Raises ProbeError if the
    saved pose can't start a session.
    """
    saved = armlib.saved_pose(name, poses_path)
    why = []
    singular = armlib.wrist_singular(saved["joints_deg"])
    if singular:
        why.append(singular)
    off = _off_p0(saved["pose"], g, scfg)
    if off:
        why.append(f"it is {off}")
    if why:
        raise ProbeError(f"not moving to saved pose {name}:\n  " + "\n  ".join(why))

    now = [v for _, v in arm.measured_joints()]
    if max(abs(v) for v in armlib.joint_error(saved["joints_deg"], now)) <= POSE_REACHED_DEG:
        log(f"from pose  already at {name}")
        return True
    joints, target, _ = armlib.check_joint_move(arm, lim, arm.measured_pose(), now,
                                             saved["joints_deg"], show=False)
    log(f"from pose  {name}: joint-space move to {armlib.fmt_pose(target)}")
    if not yes:
        log("\ndry run: nothing sent, nothing written. The session plan needs the arm there: "
            f"run arm.py goto-pose {name} --yes, then this dry run again.")
        return False
    frame = read_status(status_path)["frame"]
    result, reason = arm.reach_joints(joints, lim)
    if result != "end":
        raise armlib.ArmError(f"move to saved pose {name}: {result.upper()}" +
                              (f" ({reason})" if reason else "") +
                              ("; Base.Stop sent" if result == "timeout" else ""))
    settled_reading(status_path, frame)
    return True


def run_session(arm, lim, status_path, yes, gains_path=OUT, scfg=None, acfg=None,
                keys=None, log=print, from_pose=None, poses_path=None, mode="poses"):
    """
    Closed-loop intrinsics session. Returns the summary dict (None for a dry
    run). Raises ProbeError on a refusal before any move, armlib.ArmError on a
    fatal arm failure (after which the arm is not moved again).

    mode "poses": viewpoints from the taught poses (build_taught), reached by
    joint moves at the viewpoint speeds; the arm may start anywhere, board in
    view or not. P0 is then only where it started, and it returns there by a
    joint move. mode "orbit": the orbit (build_orbit), or the lateral variants
    (build_variants) if lateral_variants is set, starting from the probe's pose.

    For each viewpoint, in order: go to its base pose, read
    where the board is, then visit, nearest first, the cells still empty when
    the variant starts, plus the bin-seeking cells (corners, centres) while
    capture's bins for this variant's band are short. Every move is box
    checked and IK validated from the measured pose; a target that fails is
    logged and skipped. An ACTION_ABORT on a validated target is logged, the
    arm returns to the last reached pose, and the session goes on; the
    MAX_CONSECUTIVE_ABORTS-th abort in a row ends it.

    from_pose (orbit only): the name of a saved pose to move to first
    (go_to_saved_pose). Refuses to start with the wrist singular; a taught
    session makes joint moves from the start, and skips viewpoints near it.
    """
    taught = mode == "poses"
    if taught and from_pose:
        raise ProbeError("--from-pose is for --orbit; a taught session moves to its viewpoints itself")
    scfg = scfg or load_session_config()
    acfg = acfg or load_autocapture_numbers()
    g = load_gains(gains_path)

    s0 = read_status(status_path)
    if age_s(s0) > STALE_S:
        raise ProbeError(f"status file is {age_s(s0):.1f} s old: start 04_capture --auto "
                         "--status-file first")
    if from_pose:
        if s0["dry_run"] or not s0.get("session_path"):
            raise ProbeError("not starting:\n  04_capture is not in --auto (no session folder "
                             "to write into)")
        if not go_to_saved_pose(arm, lim, status_path, from_pose, g, scfg, yes, log, poses_path):
            return None
        s0 = read_status(status_path)
    why = check_ready(s0, dry_run=False, board=not taught)
    if s0["camera_serial"] != g["camera_serial"]:
        why.append(f"capture is camera {s0['camera_serial']}; the probe was camera "
                   f"{g['camera_serial']}")
    if s0["pose_bins"]["min_per_bin"] != acfg["min_per_bin"]:
        why.append(f"capture's min_per_bin {s0['pose_bins']['min_per_bin']} differs from "
                   f"{AUTOCAPTURE_CFG}: was capture given another --auto-config?")
    p0 = arm.measured_pose()
    if not taught:
        off = _off_p0(p0, g, scfg)
        if off:
            why.append(f"arm is {off}")
        singular = armlib.wrist_singular([v for _, v in arm.measured_joints()])
        if singular:
            why.append(f"{singular}. Move in joint space first: --from-pose NAME, or arm.py goto-pose")
        if abs(p0[4]) > MAX_TILT_Y_DEG:
            why.append(f"theta_y {p0[4]:+.1f} deg is near the Euler singularity")
    if why:
        raise ProbeError("not starting:\n  " + "\n  ".join(why))

    log(f"camera     {describe(s0)}")
    log(f"start P0   {armlib.fmt_pose(p0)}")
    log(f"gains      {os.path.relpath(g['source'])} ({g['created_utc']}): aim with "
        f"{g['rot_axes'][0]} (right) and {g['rot_axes'][1]} (down); lateral "
        f"{g['lateral_axes'][0]}/{g['lateral_axes'][1]}; depth along tool z "
        f"({float(g['square_per_m'][2]):+.1f} square px/m)")

    grid, image = s0["coverage"]["grid"], s0["image"]
    board, poses, sq0, orbit = None, None, None, False
    if taught:
        poses = taught_poses(poses_path)
        ratio = far_near_ratio({n: p["square_px"] for n, p in poses.items()})
        log(f"taught     {len(poses)} poses, far/near square size {ratio:.2f}; grid "
            f"{scfg['n_lr']} x {scfg['n_hl']} x {scfg['n_nf']}")
        variants = build_taught(arm, poses, p0, scfg, acfg)
    else:
        sq0 = s0["board"]["square_px"]
        square_m = board_config.load()["square_mm"]["measured"] / 1000.0
        board = estimate_board(p0, s0, g, square_m)
        orbit = not scfg["lateral_variants"] > 0
        variants = (build_orbit(p0, s0, g, scfg, acfg, board) if orbit
                    else build_variants(p0, s0, g, scfg, acfg))
    if board:
        log(f"board      estimated at x {board['base_m'][0]:+.3f}  y {board['base_m'][1]:+.3f}  "
        f"z {board['base_m'][2]:+.3f} m (base), {board['distance_m']:.2f} m from the tool; "
        f"size gains say {board['size_gain_distance_m']:.2f} m")
        if abs(board["size_gain_distance_m"] / board["distance_m"] - 1) > DISTANCE_DISAGREE:
            log(f"WARNING    the two distances differ by more than {DISTANCE_DISAGREE:.0%}: "
                "the gains' focal length or the probe is off")
    if not taught:
        log(f"bands      planner: mid = P0's {sq0:.1f} px/sq +-{scfg['mid_band_fraction']:.0%}. "
            f"capture: far < {acfg['far_square_px']:g}, near >= {acfg['near_square_px']:g} px/sq "
            f"({os.path.relpath(AUTOCAPTURE_CFG)}), so P0 is capture-{capture_band(sq0, acfg)}; "
            f"capture's bins match the planner's with far_square_px "
            f"{sq0 * (1 - scfg['mid_band_fraction']):.1f}, near_square_px "
            f"{sq0 * (1 + scfg['mid_band_fraction']):.1f}")
    for w in ([] if orbit or taught else depth_sanity(variants, sq0, g, scfg)):
        log(f"WARNING    {w}")
    if taught and len({v["capture_band"] for v in variants}) < 3:
        log(f"WARNING    every viewpoint falls in capture "
            f"{'/'.join(sorted({v['capture_band'] for v in variants}))}: capture's other bins can't "
            "fill. Change near_square_px and far_square_px, or teach further apart.")
    if orbit and len({v["capture_band"] for v in variants}) < 3:
        log(f"WARNING    every radius falls in capture "
            f"{'/'.join(sorted({v['capture_band'] for v in variants}))}: capture's other bins can't "
            "fill. Use the bands line's thresholds, or change orbit_radii_m.")

    joints0 = [v for _, v in arm.measured_joints()]
    log(f"\nplan: {len(variants)} {'viewpoints' if orbit or taught else 'variants'}; cells are chosen when each one starts "
        f"({len(s0['coverage']['empty_cells'])} empty now)")
    for v in variants:
        if taught:
            why = armlib.outside_box(lim, v["pose"])
            singular = armlib.wrist_singular(v["joints_deg"])
            if why or singular:
                v["skip"] = "; ".join(why + ([singular] if singular else []))
            log(f"  {v['name']:<26} {'SKIP: ' + v['skip'] if v['skip'] else armlib.fmt_pose(v['pose'])}"
                f"  square {v['predicted_square_px']:.1f} px, capture-{v['capture_band']}")
            continue
        try:
            v["pose"] = armlib.check_move(arm, lim, p0, v["pose"], show=False)
            reason, _ = ik_check(arm, p0, v["pose"], joints0)
            if reason:
                v["skip"] = f"unreachable: {reason}"
        except armlib.ArmError as e:
            v["skip"] = str(e).replace("REFUSED, nothing sent:\n  ", "")
        log(f"  {v['name']:<26} {'SKIP: ' + v['skip'] if v['skip'] else armlib.fmt_pose(v['pose'])}"
            f"  square {v['predicted_square_px']:.1f} px ({v['predicted_square_px'] / sq0 - 1:+.1%}), "
            f"capture-{v['capture_band']}")
    n_ok = sum(1 for v in variants if not v["skip"])
    cap = None
    if orbit:
        off = sum(1 for v in variants if v["skip"] and not v["skip"].startswith("unreachable"))
        cap = {"requested": len(variants), "reachable": n_ok, "outside_box_or_refused": off,
               "ik_refused": len(variants) - n_ok - off,
               "per_radius": {r: [sum(1 for v in variants if v["depth"] == r and not v["skip"]),
                                  sum(1 for v in variants if v["depth"] == r)]
                              for r in dict.fromkeys(v["depth"] for v in variants)}}
        log(f"cap        {n_ok}/{len(variants)} viewpoints reachable ({n_ok / len(variants):.0%}): "
            + ", ".join(f"{r} {a}/{b}" for r, (a, b) in cap["per_radius"].items())
            + f"; {off} outside the box, {cap['ik_refused']} refused by IK")
    if taught:
        log(f"checks     {n_ok}/{len(variants)} viewpoints in the box by the arm's FK and clear of "
            "the wrist; every move is checked again before it is sent")
    else:
        log(f"checks     {n_ok}/{len(variants)} poses in the box and IK-reachable from P0; "
            "every target is checked again (box, IK along the path) before it is sent")
    if not yes:
        log("\ndry run: nothing sent, nothing written. Add --yes to run the session.")
        return None

    ses = _Session(arm, lim, status_path, g, scfg, acfg, keys, log)
    ses.p0 = p0
    ses.record_path = os.path.join(s0["session_path"], SESSION_RECORD)
    ses.record = {
        "schema": "panoptes.session_arm/4",
        "mode": "taught" if taught else "orbit" if orbit else "lateral",
        "taught_poses": poses,
        "created_utc": datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "status_file": status_path, "session_path": s0["session_path"],
        "camera_serial": s0["camera_serial"],
        "p0": {"abs": [round(v, 5) for v in p0], "joints_deg": [round(v, 2) for v in joints0],
               "board": s0["board"],
               "note": "every pose's rel_p0 is in P0's tool frame: t_m (m), rotvec_deg"},
        "board_estimate": board, "cap": cap,
        "gains": {"source": os.path.relpath(g["source"]), "created_utc": g["created_utc"],
                  "rot_axes": g["rot_axes"], "lateral_axes": g["lateral_axes"],
                  "f_px": [round(float(v), 2) for v in g["f_px"]],
                  "cam_axes_in_tool": np.round(g["cam_axes"], 5).tolist(),
                  "square_px_per_m_tool_z": round(float(g["square_per_m"][2]), 3)},
        "config": {"session": scfg, "autocapture": acfg, "limits": lim,
                   "ik": {"samples": IK_SAMPLES, "joint_margin_deg": IK_JOINT_MARGIN_DEG,
                          "jump_deg": IK_JUMP_DEG, "joint_limits_deg": armlib.JOINT_LIMITS_DEG}},
        "variants": [{k: v[k] for k in ("name", "band", "depth", "t_m", "rot_deg", "pose",
                                        "predicted_square_px", "capture_band", "skip")}
                     | ({"orbit": v["orbit"]} if orbit else {})
                     | ({k: v[k] for k in ("grid", "joints_deg")} if taught else {})
                     for v in variants],
        "poses": [], "end": None}
    ses.write()
    log(f"\nrecording to {ses.record_path}\n")

    end, arm_failed = None, None
    corners, centres = corner_cells(grid), centre_cells(grid)
    # A capture band whose corner views land in another band (off-axis views
    # look bigger: see growth()) won't fill its quadrant bins this way.
    futile = set()

    def seekers(st, band):
        """Bin-seeking cells for capture band `band`: corners while its
        quadrant bins are short (and not futile), centres while flat is short."""
        out = set()
        if _quad_short(st, band) and band not in futile:
            out |= set(corners)
        if _flat_short(st, band):
            out |= set(centres)
        return out

    here = p0
    try:
        for vi, v in enumerate(variants):
            if v["skip"]:
                continue
            s = ses.live()
            if s["complete"]:
                raise SessionStop("complete")
            band = v["capture_band"]
            # Fixed when the variant starts: the cells empty now, plus seekers.
            todo = set(s["coverage"]["empty_cells"]) | seekers(s, band)
            later = sum(1 for w in variants[vi + 1:] if not w["skip"])
            # The orbit's viewpoint is itself a new tilt: go while this band
            # has a short bin, even with no cell to chase.
            wanted = todo or ((orbit or taught) and (_flat_short(s, band) or _quad_short(s, band)))
            if not wanted:
                ses.log(f"  {v['name']}: no empty cell, capture-{band} bins filled; skipped "
                        f"({later} variants left)")
                continue
            ses.log(f"  {v['name']}: {len(todo)} targets "
                    f"({len(s['coverage']['empty_cells'])} empty cells), {later} variants after it")
            ses.left = f"  [{len(todo)} left]"
            s, here = ses.visit("variant", v["name"], v["pose"], here,
                                joints=v["joints_deg"] if taught else None)
            if s is None:
                continue
            base_pose, base_c = here, s["board"]["centroid_px"]
            bbox = s["board"]["bbox_px"]
            cur = base_c
            while True:
                s = ses.live()
                if s["complete"]:
                    raise SessionStop("complete")
                empty = set(s["coverage"]["empty_cells"])
                # Drop what was filled since: a cell no longer empty stays only
                # as a bin seeker still wanted.
                todo = {c for c in todo if c in empty or c in seekers(s, band)}
                if not todo:
                    break
                centre = lambda c: ((_cell_rect(c, grid)[0] + _cell_rect(c, grid)[2]) / 2,
                                    (_cell_rect(c, grid)[1] + _cell_rect(c, grid)[3]) / 2)
                cell = min(todo, key=lambda c: (math.dist(centre(c), cur), c))
                todo.discard(cell)
                ses.left = f"  [{len(todo)} left]"
                aim = aim_for_cell(cell, grid, image, bbox, base_c, scfg["edge_margin_px"], g)
                if aim is None:
                    e = ses.new_entry("cell", v["name"], cell)
                    e["outcome"] = "skipped: board can't reach this cell at this distance"
                    ses.finish(e, None)
                    continue
                corner_seek = cell in corners and cell not in empty
                s, here2 = ses.visit("cell", v["name"], None, here,
                                     cell=cell, aim=aim, base=(base_pose, base_c))
                if s is not None:
                    cur = s["board"]["centroid_px"]
                    seen = s["decision"]["bin"].split("/")[0]
                    if corner_seek and seen != band and band not in futile:
                        futile.add(band)
                        ses.log(f"  capture-{band}: a corner view was binned "
                                f"{s['decision']['bin']}; no more corner seeking for it "
                                "(off-axis views look closer)")
                here = here2
        raise SessionStop("target list exhausted")
    except SessionStop as e:
        end = str(e)
    except KeyboardInterrupt:
        end = "operator Ctrl-C"
    except armlib.ArmInterrupted as e:
        end = f"operator Ctrl-C during a move ({e})"
    except armlib.ArmError as e:
        end, arm_failed = f"arm failure: {e}", e
    ses.left = ""

    goto_p0 = (".venv/bin/python python/arm.py " +
               (f"joints {' '.join(f'{v:.3f}' for v in joints0)}" if taught else
                f"goto {' '.join(f'{v:.4f}' for v in p0)}") + " --yes")
    returned = False
    if arm_failed is None:
        log(f"\n{end}: returning to P0")
        try:
            e = ses.new_entry("return", "P0")
            if ses.move_joints(joints0, e, False) if taught else ses.move(p0, e, is_target=False):
                e["outcome"] = "reached"
                returned = True
            ses.finish(e, None)
        except (armlib.ArmError, TargetAborted, KeyboardInterrupt) as ex:
            log(f"return to P0 failed ({ex}); arm left where it stopped. To return "
                f"(dry run first, without --yes):\n  {goto_p0}")
    else:
        log(f"\n{end}\nThe arm was not moved after the failure. To return to P0 "
            f"(dry run first, without --yes):\n  {goto_p0}")

    last = ses.last or s0
    outcomes = [e["outcome"] or "" for e in ses.entries]
    total_s = time.monotonic() - ses.t0
    timing = timing_breakdown(ses.entries, total_s, ses.moves)
    summary = {
        "end": end, "returned_to_p0": returned,
        "moves_commanded": ses.moves,
        "targets": sum(1 for e in ses.entries if e["kind"] in ("variant", "cell")),
        "accepted": last["accepted"] - s0["accepted"],
        "redundant": sum(1 for e in ses.entries if (e.get("camera") or {}).get("verdict") ==
                         "redundant" and not e["camera"]["accepted"]),
        "skipped": sum(1 for o in outcomes if o.startswith("skipped") or o == "refused"),
        "unreachable": sum(1 for o in outcomes if o == "unreachable"),
        "aborted": sum(1 for o in outcomes if o.startswith("aborted")),
        "total_s": round(total_s, 1),
        "timing": timing,
        "capture_accepted_total": last["accepted"],
        "cells_remaining": last["coverage"]["empty_cells"],
        "bins_short": _short_bins(last),
        "complete": last["complete"]}
    ses.record["end"] = summary
    ses.write()
    f = timing["fractions"]
    log("\nsummary")
    log(f"  ended            {end}{'' if returned or arm_failed else ' (NOT back at P0)'}")
    log(f"  moves commanded  {summary['moves_commanded']}  (targets {summary['targets']})")
    log(f"  accepted         {summary['accepted']} this session  (capture's total {summary['capture_accepted_total']})")
    log(f"  redundant        {summary['redundant']}")
    log(f"  skipped          {summary['skipped']}   unreachable {summary['unreachable']}   "
        f"aborted {summary['aborted']}")
    log(f"  total time       {summary['total_s']:.0f} s")
    if timing["per_move_s"] is not None:
        log(f"  per move         {timing['per_move_s']:.1f} s  (arm moving {timing['motion_per_move_s']:.1f} s)")
    log(f"  time spent       moving {f['move_s']:.0%}   waiting for still {f['still_s']:.0%}   "
        f"capture decision {f['decide_s']:.0%}   IK checks {f['ik_s']:.0%}   other {f['other_s']:.0%}")
    log(f"  cells remaining  {len(summary['cells_remaining'])}: {summary['cells_remaining']}")
    log(f"  bins short       {len(summary['bins_short'])}: {', '.join(summary['bins_short'])}")
    log(f"  record           {ses.record_path}")
    if arm_failed is not None:
        raise arm_failed
    return summary


# ---------------------------------------------------------------- CLI

def main():
    parser = argparse.ArgumentParser(description="Arm + camera calibration helpers.")
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("probe", help="measure tool-axis -> image mapping; writes config/arm_camera.yaml")
    p.add_argument("--status-file", required=True,
                   help="the PATH given to 04_capture --auto-dry --status-file")
    p.add_argument("--yes", action="store_true", help="actually move; without it, print the plan")
    p.add_argument("--ip", default=armlib.DEFAULT_IP)
    p.add_argument("--port", type=int, default=armlib.DEFAULT_PORT)
    p.add_argument("--username", default="admin")
    p.add_argument("--password", default="admin")

    p = sub.add_parser("session", help="drive an intrinsics session; capture --auto must be running")
    p.add_argument("--status-file", required=True,
                   help="the PATH given to 04_capture --auto --status-file")
    p.add_argument("--yes", action="store_true",
                   help="actually move (asked once, for the whole session); without it, print the plan")
    which = p.add_mutually_exclusive_group()
    which.add_argument("--poses", action="store_true",
                       help="viewpoints from the taught poses (the default)")
    which.add_argument("--orbit", action="store_true",
                       help="viewpoints from the orbit planner, starting from the probe's pose")
    p.add_argument("--from-pose", metavar="NAME",
                   help="with --orbit: first move in joint space to this pose from "
                        "config/arm_poses.yaml")
    p.add_argument("--ip", default=armlib.DEFAULT_IP)
    p.add_argument("--port", type=int, default=armlib.DEFAULT_PORT)
    p.add_argument("--username", default="admin")
    p.add_argument("--password", default="admin")

    p = sub.add_parser("teach", help="save the eight taught viewpoints; capture must be running")
    p.add_argument("--status-file", required=True,
                   help="the PATH given to 04_capture --status-file")
    p.add_argument("--force", action="store_true", help="teach again over existing poses")
    p.add_argument("--ip", default=armlib.DEFAULT_IP)
    p.add_argument("--port", type=int, default=armlib.DEFAULT_PORT)
    p.add_argument("--username", default="admin")
    p.add_argument("--password", default="admin")
    args = parser.parse_args()

    try:
        lim = armlib.load_limits()
        if args.command == "teach":
            connect = lambda: armlib.Arm(args.ip, args.port, args.username, args.password)
            sys.exit(0 if run_teach(connect, lim, args.status_file, args.force) else 1)
        with armlib.Arm(args.ip, args.port, args.username, args.password) as arm:
            if args.command == "probe":
                run_probe(arm, lim, args.status_file, args.yes)
            else:
                with _Keys() as keys:
                    summary = run_session(arm, lim, args.status_file, args.yes, keys=keys,
                                          from_pose=args.from_pose,
                                          mode="orbit" if args.orbit else "poses")
                if summary is not None and not summary["returned_to_p0"]:
                    sys.exit(1)
    except (ProbeError, armlib.ArmError, StatusError) as e:
        sys.exit(f"arm_capture: {e}")


if __name__ == "__main__":
    main()
