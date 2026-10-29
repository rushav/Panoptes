"""
Kinova Gen3 arm, read-only. First contact: connect, read state, close.

NOTHING HERE MOVES THE ARM. The Kortex BaseClient stub carries both reads and
motion commands, so the Arm class never hands it out: every call goes through
_ReadOnly, which refuses any method not on its allow-list. Adding a motion
command means adding it to an allow-list on purpose, in review.

The E-stop pendant must be within reach whenever this talks to the arm
(RUNBOOK §13).

Session timeout. CreateSession asks the arm to drop the session after
session_inactivity_timeout ms without traffic. SessionManager keeps it alive
from a background thread (KeepAlive every 150 ms); if this process dies, the
keep-alives stop and the arm releases the session by itself. Same values as
Kinova's examples/utilities.py.

Requires python/requirements-arm.txt (installed with --no-deps; FINDINGS §20).

Usage: python arm.py status [--ip IP] [--port PORT] [--username U] [--password P]
"""

import argparse, socket, sys

from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
from kortex_api.autogen.messages import Base_pb2, Common_pb2, Session_pb2

DEFAULT_IP = "192.168.1.10"
DEFAULT_PORT = 10000                  # Kortex TCP; 10001 is UDP, cyclic control only

SESSION_INACTIVITY_MS = 10000         # arm drops the session after this without traffic
CONNECTION_INACTIVITY_MS = 2000
CONNECT_TIMEOUT_S = 3.0               # TCPTransport.connect() has no timeout of its own
RPC_TIMEOUT_MS = 3000                 # per request; Kortex default is 10000

# The only stub methods this module may call. Reads only.
BASE_READS = ("GetFirmwareBundleVersions", "GetProductConfiguration",
              "GetMeasuredCartesianPose", "GetMeasuredJointAngles", "GetServoingMode")
DEVICE_CONFIG_READS = ("GetSerialNumber",)
DEVICE_MANAGER_READS = ("ReadAllDevices",)


class ArmError(RuntimeError):
    pass


class _ReadOnly:
    """Wraps a Kortex client stub; only allow-listed methods are reachable."""

    def __init__(self, stub, allowed):
        self._stub, self._allowed = stub, frozenset(allowed)

    def __getattr__(self, name):
        if name not in self._allowed:
            raise ArmError(f"{type(self._stub).__name__}.{name} is not an allowed read "
                           "(python/arm.py is read-only)")
        return getattr(self._stub, name)


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

        self._base = _ReadOnly(BaseClient(router), BASE_READS)
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
        """(x, y, z) in m and (theta_x, theta_y, theta_z) in deg, base frame."""
        p = self._base.GetMeasuredCartesianPose(options=self._options())
        return (p.x, p.y, p.z), (p.theta_x, p.theta_y, p.theta_z)

    def measured_joints(self):
        """[(joint id, angle deg)]."""
        j = self._base.GetMeasuredJointAngles(options=self._options())
        return [(a.joint_identifier, a.value) for a in j.joint_angles]

    def servoing_mode(self):
        m = self._base.GetServoingMode(options=self._options())
        return Base_pb2.ServoingMode.Name(m.servoing_mode)


def status(args):
    with Arm(args.ip, args.port, args.username, args.password) as arm:
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

        (x, y, z), (tx, ty, tz) = arm.measured_pose()
        print(f"\npose       x {x:+.4f}  y {y:+.4f}  z {z:+.4f} m")
        print(f"           theta_x {tx:+8.3f}  theta_y {ty:+8.3f}  theta_z {tz:+8.3f} deg")
        print("joints     " + "  ".join(f"{i}: {v:8.3f}" for i, v in arm.measured_joints()) + "  deg")
        print(f"servoing   {arm.servoing_mode()}")
    print("\nsession closed")


def main():
    parser = argparse.ArgumentParser(description="Kinova Gen3, read-only.")
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("status", help="print firmware, serial, pose, joints, servoing mode")
    p.add_argument("--ip", default=DEFAULT_IP)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--username", default="admin")
    p.add_argument("--password", default="admin")
    args = parser.parse_args()
    try:
        {"status": status}[args.command](args)
    except ArmError as e:
        sys.exit(f"arm: {e}")


if __name__ == "__main__":
    main()
