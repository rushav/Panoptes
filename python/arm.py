"""
Kinova Gen3 arm: read state, small Cartesian moves and joint-space moves,
with every end point inside a box.

The E-stop pendant must be within reach whenever this talks to the arm
(RUNBOOK §13).

WHAT CAN MOVE THE ARM. The Kortex BaseClient stub carries reads and motion
commands side by side, so Arm never hands it out. Every call goes through
_ReadOnly, which refuses any method not on the allow-list it was built with:

  BASE_READS      every command (status, and the reads move/goto/stop make)
  MOTION_CALLS    move, goto, joints and goto-pose, only after --yes and the
                  box check
  STOP_CALLS      stop

Within MOTION_CALLS, ExecuteAction accepts any Kortex Action (twist, wrench,
software e-stop, sequences...). reach_pose() and reach_joints() build the
only two sent here, each with a speed constraint, and _execute() refuses to
send anything else.

THE BOX. config/arm.yaml bounds the tool point in the base frame. Both the
current pose and the target must be inside it before anything is sent.
There is no flag to disable it or to point at another config file. A joint
move's target is the arm's own forward kinematics of those angles, and only
the end points are checked: the tool swings through an arc between them.

SAVED POSES. config/arm_poses.yaml holds named joint angles and the Cartesian
pose measured with them, written by save-pose. goto-pose moves in joint space,
because a joint move does not go through the Cartesian planner, which rejects
a path from or across a singularity (FINDINGS §20).

Session timeout. CreateSession asks the arm to drop the session after
session_inactivity_timeout ms without traffic. SessionManager keeps it alive
from a background thread (KeepAlive every 150 ms); if this process dies, the
keep-alives stop and the arm releases the session by itself. Same values as
Kinova's examples/utilities.py.

Requires python/requirements-arm.txt (installed with --no-deps; FINDINGS §20).

Usage:
  python arm.py status
  python arm.py move [--dx M] [--dy M] [--dz M] [--dtx DEG] [--dty DEG] [--dtz DEG] [--yes]
  python arm.py goto X Y Z TX TY TZ [--yes]
  python arm.py joints J1 J2 J3 J4 J5 J6 [--yes]
  python arm.py save-pose NAME [--force]
  python arm.py list-poses
  python arm.py goto-pose NAME [--yes]
  python arm.py stop
  (each also takes --ip, --port, --username, --password)

python/arm_capture.py moves the arm through Arm.reach_pose() too, after
check_move(), with the same --yes rule.
"""

import argparse, datetime, math, os, re, socket, sys, threading

import cv2

from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.Exceptions.KServerException import KServerException
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import Base_pb2, Common_pb2, Errors_pb2, Session_pb2

DEFAULT_IP = "192.168.1.10"
DEFAULT_PORT = 10000                  # Kortex TCP; 10001 is UDP, cyclic control only
LIMITS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "arm.yaml")
POSES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "arm_poses.yaml")

SESSION_INACTIVITY_MS = 10000         # arm drops the session after this without traffic
CONNECTION_INACTIVITY_MS = 2000
CONNECT_TIMEOUT_S = 3.0               # TCPTransport.connect() has no timeout of its own
RPC_TIMEOUT_MS = 3000                 # per request; Kortex default is 10000

# Ceilings on config/arm.yaml, so a typo there can't ask for a fast move.
# The arm's own Cartesian limits are 0.5 m/s and 50 deg/s.
TRANSLATION_CEILING_MPS = 0.20
ROTATION_CEILING_DPS = 30.0
TIMEOUT_CEILING_S = 300.0

# The only stub methods this module may call.
# ComputeForward/InverseKinematics are computations on the arm's controller;
# they command nothing.
BASE_READS = ("GetFirmwareBundleVersions", "GetProductConfiguration",
              "GetMeasuredCartesianPose", "GetMeasuredJointAngles", "GetServoingMode",
              "ComputeForwardKinematics", "ComputeInverseKinematics")
DEVICE_CONFIG_READS = ("GetSerialNumber",)
DEVICE_MANAGER_READS = ("ReadAllDevices",)
MOTION_CALLS = ("OnNotificationActionTopic", "ExecuteAction", "Unsubscribe", "Stop")

# Gen3 6 DoF joint position limits, deg, by actuator 1..6 (user guide, Table
# 45 "Joint position limits"); actuators 1, 4 and 6 turn without limit. Not
# readable through the API (ControlConfig exposes speed and acceleration
# limits only).
JOINT_LIMITS_DEG = {2: 128.9, 3: 147.8, 5: 120.3}
STOP_CALLS = ("Stop",)

# Actuators 4 and 6 line up when actuator 5 is at 0 or 180: the wrist
# singularity. Its guide limit, above, puts 180 out of reach on this arm.
WRIST_ACTUATOR = 5
WRIST_SINGULAR_DEG = 10.0


class ArmError(RuntimeError):
    pass


class ArmInterrupted(ArmError):
    """Ctrl-C during a move; Base.Stop has been sent."""


class ActionRefused(ArmError):
    """The arm refused ExecuteAction outright (KServerException)."""


class IKFailed(ArmError):
    """ComputeInverseKinematics found no solution for a pose."""


class _ReadOnly:
    """Wraps a Kortex client stub; only allow-listed methods are reachable."""

    def __init__(self, stub, allowed):
        self._stub, self._allowed = stub, frozenset(allowed)

    def __getattr__(self, name):
        if name not in self._allowed:
            raise ArmError(f"{type(self._stub).__name__}.{name} is not on this allow-list "
                           f"({', '.join(sorted(self._allowed))})")
        return getattr(self._stub, name)


# ---------------------------------------------------------------- limits

def load_limits(path=LIMITS):
    """config/arm.yaml as a dict. Every field required; refuses bad values."""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ArmError(f"cannot open {path}")

    def number(key, lo, hi):
        n = fs.getNode(key)
        if n.empty() or not (n.isInt() or n.isReal()):
            raise ArmError(f"{path}: {key} is missing or not a number")
        v = n.real()
        if not lo < v <= hi:
            raise ArmError(f"{path}: {key} = {v} is outside ({lo}, {hi}]")
        return v

    lim = {k: number(k, -2.0, 2.0) for k in
           ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max")}
    for axis in "xyz":
        if lim[f"{axis}_min"] >= lim[f"{axis}_max"]:
            raise ArmError(f"{path}: {axis}_min must be below {axis}_max")
    lim["max_translation_mps"] = number("max_translation_mps", 0, TRANSLATION_CEILING_MPS)
    lim["max_rotation_dps"] = number("max_rotation_dps", 0, ROTATION_CEILING_DPS)
    lim["viewpoint_translation_mps"] = number("viewpoint_translation_mps", 0, TRANSLATION_CEILING_MPS)
    lim["viewpoint_rotation_dps"] = number("viewpoint_rotation_dps", 0, ROTATION_CEILING_DPS)
    lim["move_timeout_s"] = number("move_timeout_s", 0, TIMEOUT_CEILING_S)
    fs.release()
    return lim


def outside_box(lim, pose):
    """Reasons the pose's position is outside the box; empty if inside."""
    out = []
    for axis, v in zip("xyz", pose[:3]):
        lo, hi = lim[f"{axis}_min"], lim[f"{axis}_max"]
        if not lo <= v <= hi:
            out.append(f"{axis} {v:+.4f} m is outside [{lo:+.3f}, {hi:+.3f}]")
    return out


def wrap180(a):
    """Angle in degrees, to (-180, 180]."""
    a = math.fmod(a, 360.0)
    if a > 180.0:
        a -= 360.0
    elif a <= -180.0:
        a += 360.0
    return a


def pose_error(target, measured):
    """(dx, dy, dz) m, |d| m, (dtx, dty, dtz) deg, each wrapped. Euler-angle
    differences: near theta_y = +-90 one orientation has several triples."""
    d = [m - t for m, t in zip(measured[:3], target[:3])]
    dt = [wrap180(m - t) for m, t in zip(measured[3:], target[3:])]
    return d, math.sqrt(sum(v * v for v in d)), dt


def fmt_pose(p):
    return (f"x {p[0]:+.4f}  y {p[1]:+.4f}  z {p[2]:+.4f} m   "
            f"theta_x {p[3]:+8.3f}  theta_y {p[4]:+8.3f}  theta_z {p[5]:+8.3f} deg")


def fmt_joints(q):
    return "  ".join(f"{i}: {v:8.3f}" for i, v in enumerate(q, 1)) + "  deg"


def joint_error(target, measured):
    """Wrapped difference of each joint, deg."""
    return [wrap180(m - t) for m, t in zip(measured, target)]


def wrist_singular(joints):
    """A message naming the joint if actuator 5 is within WRIST_SINGULAR_DEG of
    0 or 180; None otherwise. joints: deg, actuator order."""
    a = wrap180(joints[WRIST_ACTUATOR - 1])
    near = 0 if abs(a) < 90 else 180
    if abs(abs(a) - near) > WRIST_SINGULAR_DEG:
        return None
    return (f"actuator {WRIST_ACTUATOR} is at {a:+.1f} deg, within {WRIST_SINGULAR_DEG:g} deg of "
            f"{near} (wrist singular): the Cartesian planner rejects paths from here")


# ---------------------------------------------------------------- saved poses

def load_poses(path=None):
    """config/arm_poses.yaml as {name: {joints_deg, pose, saved_utc}}; {} if
    the file doesn't exist yet."""
    path = path or POSES
    if not os.path.exists(path):
        return {}
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise ArmError(f"cannot open {path}")
    root, poses = fs.getNode("poses"), {}
    for name in root.keys():
        n = root.getNode(name)
        poses[name] = {
            key: [n.getNode(key).at(i).real() for i in range(n.getNode(key).size())]
            for key in ("joints_deg", "pose")}
        if len(poses[name]["pose"]) != 6 or not poses[name]["joints_deg"]:
            raise ArmError(f"{path}: pose {name} needs joints_deg and six pose values")
        poses[name]["saved_utc"] = n.getNode("saved_utc").string()
        if not n.getNode("square_px").empty():
            poses[name]["square_px"] = n.getNode("square_px").real()
    fs.release()
    return poses


def write_poses(poses, path=None):
    path = path or POSES
    lines = ["%YAML:1.0", "---",
             "# Saved arm poses, written by python/arm.py save-pose (RUNBOOK §13). joints_deg",
             "# is what goto-pose moves to; pose is the Cartesian pose measured with it",
             "# (x y z m, theta_x/y/z deg). square_px: the board's apparent square size,",
             "# from arm_capture.py teach. Valid for one camera setup.",
             "poses:"]
    for name, p in poses.items():
        lines += [f"   {name}:",
                  "      joints_deg: [ " + ", ".join(f"{v:.3f}" for v in p["joints_deg"]) + " ]",
                  "      pose: [ " + ", ".join(f"{v:.5f}" for v in p["pose"]) + " ]",
                  f"      saved_utc: \"{p['saved_utc']}\""]
        if "square_px" in p:
            lines.append(f"      square_px: {p['square_px']:.3f}")
    with open(path + ".tmp", "w") as f:
        f.write("\n".join(lines) + "\n")
    os.replace(path + ".tmp", path)


def store_pose(name, joints_deg, pose, force=False, square_px=None, path=None):
    """Add a pose to the file; returns the entry it replaced, or None. An
    existing name is replaced only with force."""
    if not re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name):
        raise ArmError(f"pose name {name!r}: use letters, digits and _, starting with a letter")
    poses = load_poses(path)
    replaced = poses.get(name)
    if replaced and not force:
        raise ArmError(f"pose {name} exists (saved {replaced['saved_utc']}); nothing written. "
                       "Add --force to overwrite it.")
    poses[name] = {"joints_deg": list(joints_deg), "pose": list(pose),
                   "saved_utc": datetime.datetime.now(datetime.timezone.utc)
                   .strftime("%Y-%m-%dT%H:%M:%SZ")}
    if square_px is not None:
        poses[name]["square_px"] = square_px
    write_poses(poses, path)
    return replaced


def saved_pose(name, path=None):
    poses = load_poses(path)
    if name not in poses:
        raise ArmError(f"no saved pose {name!r} in {os.path.relpath(path or POSES)} "
                       f"(saved: {', '.join(poses) or 'none'}). Save one with arm.py save-pose.")
    return poses[name]


# ---------------------------------------------------------------- arm

class Arm:
    """
    with Arm() as arm:
        print(arm.measured_pose())

    __enter__ connects and opens a session; __exit__ closes both, also on an
    exception.
    """

    def __init__(self, ip=DEFAULT_IP, port=DEFAULT_PORT, username="admin", password="admin"):
        self.ip, self.port = ip, port
        self._credentials = (username, password)
        self._transport = None
        self._session = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def connect(self):
        # Fail fast with a plain socket first: an unreachable arm otherwise
        # blocks in TCPTransport.connect() for the OS TCP timeout (~2 min).
        try:
            socket.create_connection((self.ip, self.port), timeout=CONNECT_TIMEOUT_S).close()
        except OSError as e:
            raise ArmError(f"cannot reach {self.ip}:{self.port} ({e}). "
                           "Check the 192.168.1.11/24 address on enp6s0 (RUNBOOK §13).") from e

        self._transport = TCPTransport()
        self._transport.connect(self.ip, self.port)
        router = RouterClient(self._transport, None)      # None: Kortex's basic stderr callback

        info = Session_pb2.CreateSessionInfo()
        info.username, info.password = self._credentials
        info.session_inactivity_timeout = SESSION_INACTIVITY_MS
        info.connection_inactivity_timeout = CONNECTION_INACTIVITY_MS
        session = SessionManager(router)
        try:
            session.CreateSession(info, self._options())
        except Exception as e:
            self._transport.disconnect()
            self._transport = None
            raise ArmError(f"session refused by {self.ip}: {e}") from e
        self._session = session

        # ONE BaseClient per router. Each one registers itself as THE
        # receiver of Base notifications (RouterClient.notificationService
        # is keyed by service id), so a second instance would steal the
        # ACTION_END the motion wrapper waits for. All Base wrappers share it.
        self._base_stub = BaseClient(router)
        self._base = _ReadOnly(self._base_stub, BASE_READS)
        self._device_config = _ReadOnly(DeviceConfigClient(router), DEVICE_CONFIG_READS)
        self._device_manager = _ReadOnly(DeviceManagerClient(router), DEVICE_MANAGER_READS)

    def close(self):
        if self._session is not None:
            options = RouterClientSendOptions()
            options.timeout_ms = 1000
            self._session.CloseSession(options)
            self._session = None
        if self._transport is not None:
            self._transport.disconnect()
            self._transport = None

    @staticmethod
    def _options():
        options = RouterClientSendOptions()
        options.timeout_ms = RPC_TIMEOUT_MS
        return options

    # ---------------------------------------------------------------- reads

    def base_device_id(self):
        """Device id of the base, from the device manager's list."""
        handles = self._device_manager.ReadAllDevices(options=self._options()).device_handle
        bases = [h.device_identifier for h in handles if h.device_type == Common_pb2.BASE]
        if len(bases) != 1:
            raise ArmError(f"expected one BASE device, found {len(bases)}: " +
                           ", ".join(f"{Common_pb2.DeviceTypes.Name(h.device_type)} "
                                     f"id {h.device_identifier}" for h in handles))
        return bases[0]

    def devices(self):
        """[(type name, device id, order)] for every device the base reports."""
        handles = self._device_manager.ReadAllDevices(options=self._options()).device_handle
        return [(Common_pb2.DeviceTypes.Name(h.device_type), h.device_identifier, h.order)
                for h in handles]

    def firmware(self):
        """(main bundle version, {component name: version})."""
        v = self._base.GetFirmwareBundleVersions(options=self._options())
        return v.main_bundle_version, {c.name: c.version for c in v.components_versions}

    def serial_number(self):
        return self._device_config.GetSerialNumber(self.base_device_id(),
                                                   self._options()).serial_number

    def product(self):
        """Degrees of freedom and end effector, as the arm reports its build."""
        p = self._base.GetProductConfiguration(options=self._options())
        return {"dof": p.degree_of_freedom,
                "end_effector": Base_pb2.EndEffectorType.Name(p.end_effector_type)}

    def measured_pose(self):
        """(x, y, z, theta_x, theta_y, theta_z): m and deg, base frame."""
        p = self._base.GetMeasuredCartesianPose(options=self._options())
        return (p.x, p.y, p.z, p.theta_x, p.theta_y, p.theta_z)

    def measured_joints(self):
        """[(joint id, angle deg)]."""
        j = self._base.GetMeasuredJointAngles(options=self._options())
        return [(a.joint_identifier, a.value) for a in j.joint_angles]

    def forward_kinematics(self, joints_deg):
        """The arm's own FK: (x, y, z, theta_x, theta_y, theta_z) for joint
        angles in deg, actuator order."""
        q = Base_pb2.JointAngles()
        for i, v in enumerate(joints_deg):
            a = q.joint_angles.add()
            a.joint_identifier = i
            a.value = v
        try:
            p = self._base.ComputeForwardKinematics(q, options=self._options())
        except KServerException as e:
            raise ArmError(f"forward kinematics refused by the arm: {e}") from e
        return (p.x, p.y, p.z, p.theta_x, p.theta_y, p.theta_z)

    def inverse_kinematics(self, pose, guess_deg):
        """Joint angles (deg, actuator order) the arm's own IK finds for pose,
        starting from guess_deg. Raises IKFailed if it finds none."""
        ik = Base_pb2.IKData()
        p = ik.cartesian_pose
        p.x, p.y, p.z, p.theta_x, p.theta_y, p.theta_z = pose
        for i, v in enumerate(guess_deg):
            a = ik.guess.joint_angles.add()
            a.joint_identifier = i
            a.value = v
        try:
            out = self._base.ComputeInverseKinematics(ik, options=self._options())
        except KServerException as e:
            raise IKFailed(f"no IK solution ({e})") from e
        return [a.value for a in out.joint_angles]

    def servoing_mode(self):
        m = self._base.GetServoingMode(options=self._options())
        return Base_pb2.ServoingMode.Name(m.servoing_mode)

    # ---------------------------------------------------------------- motion

    def stop(self):
        """Base.Stop: ends the current action under control. Not the E-stop."""
        try:
            _ReadOnly(self._base_stub, STOP_CALLS).Stop(options=self._options())
        except KServerException as e:
            raise ArmError(f"Base.Stop refused by the arm: {e}") from e

    def reach_pose(self, target, lim):
        """
        Send one reach_pose action and block until it ends.

        Returns ("end", None), ("abort", reason) or ("timeout", None). On
        timeout or Ctrl-C, sends Base.Stop before returning or raising. The
        caller must already have checked the box; this checks it again.
        """
        problems = outside_box(lim, target)
        if problems:
            raise ArmError("target outside the box: " + "; ".join(problems))

        action = Base_pb2.Action()
        action.name = "panoptes arm.py reach_pose"
        action.application_data = ""
        pose = action.reach_pose.target_pose
        pose.x, pose.y, pose.z, pose.theta_x, pose.theta_y, pose.theta_z = target
        speed = action.reach_pose.constraint.speed
        speed.translation = lim["max_translation_mps"]
        speed.orientation = lim["max_rotation_dps"]
        return self._execute(action, "reach_pose", lim)

    def reach_joints(self, joints_deg, lim, dps=None):
        """
        Send one reach_joint_angles action, at dps per joint (default
        max_rotation_dps), and block until it ends. Returns and stops as
        reach_pose(). The caller
        must already have run check_joint_move(); this checks the box again,
        on the arm's FK of joints_deg.
        """
        problems = outside_box(lim, self.forward_kinematics(joints_deg))
        if problems:
            raise ArmError("target outside the box: " + "; ".join(problems))

        action = Base_pb2.Action()
        action.name = "panoptes arm.py reach_joint_angles"
        action.application_data = ""
        for i, v in enumerate(joints_deg):
            a = action.reach_joint_angles.joint_angles.joint_angles.add()
            a.joint_identifier = i
            a.value = v
        constraint = action.reach_joint_angles.constraint
        constraint.type = Base_pb2.JOINT_CONSTRAINT_SPEED
        constraint.value = dps or lim["max_rotation_dps"]
        return self._execute(action, "reach_joint_angles", lim)

    def _execute(self, action, kind, lim):
        """Run one action to its end; the only path to ExecuteAction."""
        if kind not in ("reach_pose", "reach_joint_angles") or \
                action.WhichOneof("action_parameters") != kind:
            raise ArmError(f"refusing to send an action that is not {kind}")
        motion = _ReadOnly(self._base_stub, MOTION_CALLS)

        done = threading.Event()
        outcome = {}

        def on_action(n):
            if done.is_set():
                return
            if n.action_event in (Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT):
                outcome["event"] = n.action_event
                outcome["abort"] = n.abort_details
                done.set()

        handle = motion.OnNotificationActionTopic(on_action, Base_pb2.NotificationOptions(),
                                                  options=self._options())
        try:
            try:
                motion.ExecuteAction(action, options=self._options())
            except KServerException as e:
                raise ActionRefused(f"ExecuteAction refused by the arm: {e}") from e
            if not done.wait(lim["move_timeout_s"]):
                motion.Stop(options=self._options())
                return "timeout", None
        except KeyboardInterrupt:
            motion.Stop(options=self._options())
            raise ArmInterrupted("interrupted (Ctrl-C): Base.Stop sent")
        finally:
            motion.Unsubscribe(handle, options=self._options())

        if outcome["event"] == Base_pb2.ACTION_END:
            return "end", None
        return "abort", Errors_pb2.SubErrorCodes.Name(outcome["abort"])


# ---------------------------------------------------------------- commands

def _connect(args):
    return Arm(args.ip, args.port, args.username, args.password)


def status(args):
    with _connect(args) as arm:
        print(f"arm {arm.ip}:{arm.port}  session open "
              f"(inactivity timeout {SESSION_INACTIVITY_MS} ms)\n")

        bundle, components = arm.firmware()
        print(f"firmware   {bundle}")
        for name, version in components.items():
            print(f"             {name:<24} {version}")
        print(f"serial     {arm.serial_number()}  (base)")
        product = arm.product()
        print(f"build      {product['dof']} DOF, end effector {product['end_effector']}")
        print("devices    " + ", ".join(f"{t} {i}" for t, i, _ in arm.devices()))

        x, y, z, tx, ty, tz = arm.measured_pose()
        print(f"\npose       x {x:+.4f}  y {y:+.4f}  z {z:+.4f} m")
        print(f"           theta_x {tx:+8.3f}  theta_y {ty:+8.3f}  theta_z {tz:+8.3f} deg")
        print("joints     " + "  ".join(f"{i}: {v:8.3f}" for i, v in arm.measured_joints()) + "  deg")
        print(f"servoing   {arm.servoing_mode()}")
    print("\nsession closed")


def check_move(arm, lim, current, target, show=True):
    """
    Every check made before a move is sent; the only path to reach_pose()
    for move, goto and python/arm_capture.py. Returns the target with its
    angles wrapped. Raises ArmError("REFUSED, nothing sent: ...") if the
    current pose or the target is outside the box, the estimated time
    exceeds move_timeout_s, or the servoing mode is not single-level.
    show=True prints current, target, change, box and the result.
    """
    target = tuple(target[:3]) + tuple(wrap180(a) for a in target[3:])
    d, dist, dt = pose_error(current, target)
    rot = max(abs(a) for a in dt)
    est = max(dist / lim["max_translation_mps"], rot / lim["max_rotation_dps"])
    if show:
        print(f"current    {fmt_pose(current)}")
        print(f"target     {fmt_pose(target)}")
        print(f"change     {dist * 1000:.1f} mm, up to {rot:.2f} deg; at "
              f"{lim['max_translation_mps']:.3f} m/s and {lim['max_rotation_dps']:.1f} deg/s "
              f"about {est:.1f} s (timeout {lim['move_timeout_s']:g} s)")
        print(f"box        x [{lim['x_min']:+.3f}, {lim['x_max']:+.3f}]  "
              f"y [{lim['y_min']:+.3f}, {lim['y_max']:+.3f}]  "
              f"z [{lim['z_min']:+.3f}, {lim['z_max']:+.3f}] m  (config/arm.yaml)")

    refuse = [f"current pose: {p}" for p in outside_box(lim, current)]
    refuse += [f"target: {p}" for p in outside_box(lim, target)]
    if est > lim["move_timeout_s"]:
        refuse.append(f"estimated {est:.1f} s exceeds move_timeout_s; split the move")
    mode = arm.servoing_mode()
    if mode != "SINGLE_LEVEL_SERVOING":
        refuse.append(f"servoing mode is {mode}, not SINGLE_LEVEL_SERVOING")
    if refuse:
        raise ArmError("REFUSED, nothing sent:\n  " + "\n  ".join(refuse))
    if show:
        print("checks     current and target inside the box; servoing SINGLE_LEVEL_SERVOING")
    return target


def check_joint_move(arm, lim, current, joints_now, joints, show=True, viewpoint=False):
    """
    Every check made before a joint move is sent; the only path to
    reach_joints(). current and joints_now are the measured pose and joints;
    joints is the target, deg, actuator order. The box applies to the arm's FK
    of the target; the time estimate is the largest wrapped joint change at
    max_rotation_dps. Otherwise as check_move(). viewpoint: use the viewpoint
    speeds, the joint speed lowered until the tool's straight-line distance
    over the move's time is within viewpoint_translation_mps. Returns (joints
    in [0, 360), FK pose, joint speed deg/s).
    """
    if len(joints) != len(joints_now):
        raise ArmError(f"REFUSED, nothing sent:\n  {len(joints)} joint angles for a "
                       f"{len(joints_now)}-joint arm")
    joints = [v % 360.0 for v in joints]             # as the arm reports them
    refuse = []
    for actuator, limit in JOINT_LIMITS_DEG.items():
        a = wrap180(joints[actuator - 1])
        if abs(a) > limit:
            refuse.append(f"target: joint {actuator} {a:+.1f} deg is past its limit +-{limit:g}")
    if refuse:
        raise ArmError("REFUSED, nothing sent:\n  " + "\n  ".join(refuse))

    target = arm.forward_kinematics(joints)
    rot = max(abs(v) for v in joint_error(joints_now, joints))
    dps = lim["max_rotation_dps"]
    if viewpoint:
        dps = lim["viewpoint_rotation_dps"]
        dist = math.dist(current[:3], target[:3])
        if rot > 0 and dist > 0:
            dps = min(dps, rot * lim["viewpoint_translation_mps"] / dist)
    est = rot / dps
    if show:
        print(f"current    {fmt_joints(joints_now)}")
        print(f"target     {fmt_joints(joints)}")
        print(f"current    {fmt_pose(current)}")
        print(f"target FK  {fmt_pose(target)}")
        print(f"change     up to {rot:.2f} deg on a joint; at {dps:.1f} deg/s "
              f"about {est:.1f} s (timeout {lim['move_timeout_s']:g} s)")
        print(f"box        x [{lim['x_min']:+.3f}, {lim['x_max']:+.3f}]  "
              f"y [{lim['y_min']:+.3f}, {lim['y_max']:+.3f}]  "
              f"z [{lim['z_min']:+.3f}, {lim['z_max']:+.3f}] m  (config/arm.yaml); "
              "end points only, the tool swings through an arc between them")

    refuse = [f"current pose: {p}" for p in outside_box(lim, current)]
    refuse += [f"target: {p}" for p in outside_box(lim, target)]
    if est > lim["move_timeout_s"]:
        refuse.append(f"estimated {est:.1f} s exceeds move_timeout_s; split the move")
    mode = arm.servoing_mode()
    if mode != "SINGLE_LEVEL_SERVOING":
        refuse.append(f"servoing mode is {mode}, not SINGLE_LEVEL_SERVOING")
    if refuse:
        raise ArmError("REFUSED, nothing sent:\n  " + "\n  ".join(refuse))
    if show:
        print("checks     current and target inside the box; servoing SINGLE_LEVEL_SERVOING")
    return joints, target, dps


def _motion(args, make_target):
    """Shared by move and goto: plan, check, then send only with --yes."""
    lim = load_limits()
    with _connect(args) as arm:
        current = arm.measured_pose()
        target = check_move(arm, lim, current, make_target(current))

        if not args.yes:
            print("\ndry run: nothing sent. Add --yes to move.")
            return

        print("\nsending reach_pose ...")
        result, reason = arm.reach_pose(target, lim)
        _report(target, arm.measured_pose(), result, reason, lim)


def _report(target, reached, result, reason, lim):
    d, dist, dt = pose_error(target, reached)
    print(f"result     {result.upper()}" + (f"  ({reason})" if reason else ""))
    print(f"measured   {fmt_pose(reached)}")
    print(f"error      dx {d[0] * 1000:+.2f}  dy {d[1] * 1000:+.2f}  dz {d[2] * 1000:+.2f} mm"
          f"  |d| {dist * 1000:.2f} mm   "
          f"dtheta_x {dt[0]:+.3f}  dtheta_y {dt[1]:+.3f}  dtheta_z {dt[2]:+.3f} deg")
    if result == "timeout":
        raise ArmError(f"no ACTION_END or ACTION_ABORT within {lim['move_timeout_s']:g} s; "
                       "Base.Stop sent")
    if result == "abort":
        raise ArmError(f"ACTION_ABORT: {reason}")


def _joint_motion(args, joints):
    """Shared by joints and goto-pose: plan, check, then send only with --yes."""
    lim = load_limits()
    with _connect(args) as arm:
        current = arm.measured_pose()
        now = [v for _, v in arm.measured_joints()]
        joints, target, _ = check_joint_move(arm, lim, current, now, joints)

        if not args.yes:
            print("\ndry run: nothing sent. Add --yes to move.")
            return

        print("\nsending reach_joint_angles ...")
        result, reason = arm.reach_joints(joints, lim)
        err = joint_error(joints, [v for _, v in arm.measured_joints()])
        _report(target, arm.measured_pose(), result, reason, lim)
        print(f"joints     largest error {max(abs(v) for v in err):.3f} deg")


def move(args):
    def target(c):
        return (c[0] + args.dx, c[1] + args.dy, c[2] + args.dz,
                c[3] + args.dtx, c[4] + args.dty, c[5] + args.dtz)
    _motion(args, target)


def goto(args):
    _motion(args, lambda c: (args.x, args.y, args.z, args.tx, args.ty, args.tz))


def joints(args):
    _joint_motion(args, args.joints)


def save_pose(args):
    with _connect(args) as arm:
        q, pose = [v for _, v in arm.measured_joints()], arm.measured_pose()
    replaced = store_pose(args.name, q, pose, args.force)
    print(f"joints     {fmt_joints(q)}")
    print(f"pose       {fmt_pose(pose)}")
    if replaced:
        print(f"replaced   {args.name} saved {replaced['saved_utc']}: {fmt_joints(replaced['joints_deg'])}")
    why = wrist_singular(q)
    if why:
        print(f"WARNING    {why}; a session will refuse to start from this pose")
    print(f"saved      {args.name} in {os.path.relpath(POSES)}")


def list_poses(args):
    poses = load_poses()
    if not poses:
        print(f"no saved poses in {os.path.relpath(POSES)}")
    for name, p in poses.items():
        print(f"{name}  saved {p['saved_utc']}" +
              (f"  square {p['square_px']:.1f} px" if "square_px" in p else ""))
        print(f"  joints   {fmt_joints(p['joints_deg'])}")
        print(f"  pose     {fmt_pose(p['pose'])}")


def goto_pose(args):
    p = saved_pose(args.name)
    print(f"saved      {args.name}, {p['saved_utc']}: {fmt_pose(p['pose'])}")
    _joint_motion(args, p["joints_deg"])


def stop(args):
    with _connect(args) as arm:
        arm.stop()
        print("Base.Stop sent. Arm stays powered and holds position; this is NOT the E-stop.")


def main():
    parser = argparse.ArgumentParser(description="Kinova Gen3: read state; moves in a box.")
    sub = parser.add_subparsers(dest="command", required=True)

    def connection(p):
        p.add_argument("--ip", default=DEFAULT_IP)
        p.add_argument("--port", type=int, default=DEFAULT_PORT)
        p.add_argument("--username", default="admin")
        p.add_argument("--password", default="admin")

    p = sub.add_parser("status", help="print firmware, serial, pose, joints, servoing mode")
    connection(p)

    p = sub.add_parser("move", help="relative Cartesian move from the measured pose")
    for name, unit in (("dx", "m"), ("dy", "m"), ("dz", "m"),
                       ("dtx", "deg"), ("dty", "deg"), ("dtz", "deg")):
        p.add_argument(f"--{name}", type=float, default=0.0, help=f"{unit}, default 0")
    p.add_argument("--yes", action="store_true", help="actually move; without it, print the plan")
    connection(p)

    p = sub.add_parser("goto", help="absolute Cartesian move, base frame")
    for name, unit in (("x", "m"), ("y", "m"), ("z", "m"),
                       ("tx", "deg"), ("ty", "deg"), ("tz", "deg")):
        p.add_argument(name, type=float, help=unit)
    p.add_argument("--yes", action="store_true", help="actually move; without it, print the plan")
    connection(p)

    p = sub.add_parser("joints", help="joint-space move; the box applies to the arm's FK of the angles")
    p.add_argument("joints", type=float, nargs=6, metavar="J",
                   help="J1 to J6, deg, actuator order")
    p.add_argument("--yes", action="store_true", help="actually move; without it, print the plan")
    connection(p)

    p = sub.add_parser("save-pose", help="store the measured joints and pose in config/arm_poses.yaml")
    p.add_argument("name", metavar="NAME")
    p.add_argument("--force", action="store_true", help="overwrite a pose of that name")
    connection(p)

    p = sub.add_parser("list-poses", help="print config/arm_poses.yaml; does not touch the arm")

    p = sub.add_parser("goto-pose", help="joint-space move to a saved pose")
    p.add_argument("name", metavar="NAME")
    p.add_argument("--yes", action="store_true", help="actually move; without it, print the plan")
    connection(p)

    p = sub.add_parser("stop", help="Base.Stop: end the current action (not the E-stop)")
    connection(p)

    args = parser.parse_args()
    try:
        {"status": status, "move": move, "goto": goto, "joints": joints, "save-pose": save_pose,
         "list-poses": list_poses, "goto-pose": goto_pose, "stop": stop}[args.command](args)
    except ArmError as e:
        sys.exit(f"arm: {e}")


if __name__ == "__main__":
    main()
