"""
Rig model for the synthetic studies: layout loading, look-at extrinsics,
Brown-Conrady projection with its analytic Jacobian, undistortion and
visibility.

UNITS. layout.yaml and study.yaml are in inches because that is how the room
was measured. load_layout() and load_study() convert to millimetres, and they
are the only places that do. Everything returned from this module is mm.

Cameras are held as stacked arrays (R: C x 3 x 3, t: C x 3) so every
operation runs over all cameras and all points at once in numpy.
"""

import os
import numpy as np
import cv2

MM_PER_IN = 25.4


# ---------------------------------------------------------------- yaml

def _node(n):
    """cv2.FileNode -> plain python (dict / list / float / str)."""
    if n.isMap():
        return {k: _node(n.getNode(k)) for k in n.keys()}
    if n.isSeq():
        return [_node(n.at(i)) for i in range(n.size())]
    if n.isString():
        s = n.string()
        # FileStorage keeps "value  # comment" whole on unquoted strings;
        # numbers are unaffected. Fail loudly rather than use a mangled path.
        if " #" in s:
            raise ValueError(f"yaml string {s!r} swallowed a comment; quote the value")
        return s
    if n.isInt():
        return int(n.real())
    if n.isReal():
        return n.real()
    return None


def read_yaml(path):
    # OpenCV's reader, same as mask.yaml uses, so no PyYAML dependency.
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(path)
    out = _node(fs.root())
    fs.release()
    return out


# ---------------------------------------------------------------- cameras

def look_at(position, target, up):
    """
    World->camera rotation, OpenCV convention (x right, y down, z forward).
    Rows are the camera axes expressed in world coordinates.
    """
    f = target - position
    f = f / np.linalg.norm(f)
    right = np.cross(f, up)
    if np.linalg.norm(right) < 1e-6:
        # Looking straight along `up`: roll is undefined. Borrow world +y
        # (or +x if that is also parallel) so the frame is still well formed.
        alt = np.array([0.0, 1.0, 0.0])
        if abs(f @ alt) > 0.99:
            alt = np.array([1.0, 0.0, 0.0])
        right = np.cross(f, alt)
    right /= np.linalg.norm(right)
    down = np.cross(f, right)
    return np.stack([right, down, f])


class Rig:
    """C cameras sharing one intrinsic model. All lengths mm."""

    def __init__(self, names, serials, centers, R, model):
        self.names = list(names)
        self.serials = list(serials)
        self.C = np.asarray(centers, float)          # C x 3 camera centres
        self.R = np.asarray(R, float)                # C x 3 x 3
        self.t = -np.einsum("cij,cj->ci", self.R, self.C)
        m = model
        self.fx, self.fy, self.cx, self.cy = m["fx"], m["fy"], m["cx"], m["cy"]
        self.dist = np.asarray(m["dist"], float)
        self.width, self.height = m["resolution"]
        self.model = m
        self.r_max = self._max_radius()

    @property
    def K(self):
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])

    def subset(self, idx):
        return Rig([self.names[i] for i in idx], [self.serials[i] for i in idx],
                   self.C[idx], self.R[idx], self.model)

    # ---- distortion, on normalised coordinates

    def distort(self, xn):
        """Normalised (...,2) -> distorted normalised (...,2), with d/dxn (...,2,2)."""
        k1, k2, p1, p2, k3 = self.dist
        x, y = xn[..., 0], xn[..., 1]
        r2 = x * x + y * y
        rad = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
        drad = k1 + r2 * (2 * k2 + 3 * k3 * r2)       # d rad / d r2
        xd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
        yd = y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        J = np.empty(xn.shape + (2,))
        J[..., 0, 0] = rad + 2 * x * x * drad + 2 * p1 * y + 6 * p2 * x
        J[..., 0, 1] = 2 * x * y * drad + 2 * p1 * x + 2 * p2 * y
        J[..., 1, 0] = 2 * x * y * drad + 2 * p1 * x + 2 * p2 * y
        J[..., 1, 1] = rad + 2 * y * y * drad + 6 * p1 * y + 2 * p2 * x
        return np.stack([xd, yd], -1), J

    def undistort(self, uv, iters=8):
        """
        Pixels (...,2) -> undistorted normalised coordinates (...,2).
        Newton on distort(xn) = xd. Converges quadratically, so 8 steps reach
        machine precision; cv2.undistortPoints' fixed-point default (5 steps)
        stops around 1e-4 px at the frame edge, too loose for the zero-noise test.
        """
        xd = np.stack([(uv[..., 0] - self.cx) / self.fx,
                       (uv[..., 1] - self.cy) / self.fy], -1)
        xn = xd.copy()
        for _ in range(iters):
            d, J = self.distort(xn)
            xn = xn - np.linalg.solve(J, (d - xd)[..., None])[..., 0]
        return xn

    def _max_radius(self):
        """
        Largest undistorted radius any in-image pixel maps to (the corners),
        after checking the distortion is monotonic out to there. Points beyond
        this radius are outside the field of view even if the polynomial folds
        them back onto the sensor: a known artefact of Brown-Conrady far
        outside its fitted range, and one that would otherwise give phantom
        observations of points behind the lens edge.
        """
        corners = np.array([[0, 0], [self.width, 0], [0, self.height],
                            [self.width, self.height]], float)
        r_max = np.linalg.norm(self.undistort(corners), axis=-1).max()
        r = np.linspace(0, r_max * 1.05, 2000)
        d, _ = self.distort(np.stack([r, np.zeros_like(r)], -1))
        if np.any(np.diff(d[:, 0]) <= 0):
            raise ValueError("distortion folds inside the image; coefficients implausible")
        return r_max

    # ---- projection

    def to_camera(self, X):
        """World points (...,3) -> camera-frame points (...,C,3)."""
        return np.einsum("cij,...j->...ci", self.R, X) + self.t

    def project(self, X, jacobian=False):
        """
        World points (...,3) -> pixels (...,C,2), depth (...,C), and if asked
        d(pixel)/d(world point) (...,C,2,3). Depth <= 0 rows are garbage;
        callers mask them with visible().
        """
        Xc = self.to_camera(X)
        z = Xc[..., 2]
        zs = np.where(np.abs(z) < 1e-9, 1e-9, z)       # keep behind-camera rows finite
        xn = Xc[..., :2] / zs[..., None]
        xd, Jd = self.distort(xn)
        uv = np.stack([self.fx * xd[..., 0] + self.cx, self.fy * xd[..., 1] + self.cy], -1)
        if not jacobian:
            return uv, z, xn
        # chain: pixel <- distorted <- normalised <- camera <- world
        Jn = np.zeros(xn.shape[:-1] + (2, 3))
        Jn[..., 0, 0] = 1 / zs
        Jn[..., 1, 1] = 1 / zs
        Jn[..., 0, 2] = -xn[..., 0] / zs
        Jn[..., 1, 2] = -xn[..., 1] / zs
        J = Jd @ Jn @ self.R                            # (...,C,2,3)
        J[..., 0, :] *= self.fx
        J[..., 1, :] *= self.fy
        return uv, z, xn, J

    def visible(self, X):
        """(...,C) bool: in front of the camera, on the sensor, inside the true FOV."""
        uv, z, xn = self.project(X)
        return ((z > 0)
                & (uv[..., 0] >= 0) & (uv[..., 0] <= self.width)
                & (uv[..., 1] >= 0) & (uv[..., 1] <= self.height)
                & (np.linalg.norm(xn, axis=-1) <= self.r_max))


# ---------------------------------------------------------------- loading

def load_layout(path):
    """layout.yaml -> (Rig, info dict). The inch->mm conversion lives here."""
    y = read_yaml(path)
    if y["units"] != "inch":
        raise ValueError(f"layout units {y['units']!r}; expected inch")
    mm = lambda v: np.asarray(v, float) * MM_PER_IN
    aim, up = mm(y["aim"]), np.asarray(y["up"], float)
    cams = y["cameras"]
    centers = np.stack([mm(c["position"]) for c in cams])
    R = np.stack([look_at(c, aim, up) for c in centers])
    rig = Rig([c["name"] for c in cams], [c["serial"] for c in cams],
              centers, R, y["camera_model"])
    info = {"volume_min": mm(y["volume"]["min"]), "volume_max": mm(y["volume"]["max"]),
            "aim": aim, "up": up, "perch": mm(y["perch"]), "raw": y}
    return rig, info


def load_study(path):
    """study.yaml -> dict, every *_in length converted to mm under a *_mm key."""
    y = read_yaml(path)

    def conv(d):
        if isinstance(d, dict):
            out = {}
            for k, v in d.items():
                if k.endswith("_in"):
                    out[k[:-3] + "_mm"] = (np.asarray(v, float) * MM_PER_IN).tolist() \
                        if isinstance(v, list) else v * MM_PER_IN
                else:
                    out[k] = conv(v)
            return out
        return d

    s = conv(y)
    s["raw"] = y
    s["output"]["dir"] = os.path.normpath(os.path.join(os.path.dirname(path), y["output"]["dir"]))
    return s


def perturb(rig, rng, pos_mm, aim_deg, roll_deg):
    """
    A plausible "true" rig given a tape-measured one: each centre moved
    uniformly within +/- pos_mm per axis, each optical axis tilted by up to
    aim_deg in a random direction, then rolled by up to +/- roll_deg.
    """
    n = len(rig.names)
    centers = rig.C + rng.uniform(-pos_mm, pos_mm, (n, 3))
    R = np.empty_like(rig.R)
    for i in range(n):
        phi = rng.uniform(0, 2 * np.pi)
        tilt = np.deg2rad(rng.uniform(0, aim_deg)) * np.array([np.cos(phi), np.sin(phi), 0])
        roll = np.array([0, 0, np.deg2rad(rng.uniform(-roll_deg, roll_deg))])
        # Deltas are in the camera frame, so they compose on the left.
        R[i] = cv2.Rodrigues(roll)[0] @ cv2.Rodrigues(tilt)[0] @ rig.R[i]
    return Rig(rig.names, rig.serials, centers, R, rig.model)
