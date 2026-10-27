"""
ChArUco board config: config/charuco_board.yaml, the single source of truth.

Mirrors loadBoardSpec() in lib/detect.cpp: same file, same checks, and the
measured sizes are required. Nominal is never used in their place.

A session is analysed against the board it was CAPTURED with, not whatever
config/charuco_board.yaml says today:

  - session.json written by the current 04_capture carries the full board
    record (it has legacy_pattern). That record is used.
  - Older session.json files carry only squares/sizes/dictionary. Those
    sessions used the old board, so config/charuco_board_old.yaml is used,
    after checking that it agrees with what the session recorded.

Every loader prints the exact board it resolved.

Usage: python board.py [board.yaml]     # print the board, or the error
"""

import os, sys
import cv2

CONFIG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config")
CURRENT = os.path.join(CONFIG_DIR, "charuco_board.yaml")
OLD = os.path.join(CONFIG_DIR, "charuco_board_old.yaml")

PROVENANCE = ("vendor", "order_date", "material", "pattern_mm", "plate_mm",
              "measured_by", "measured_date")


class BoardConfigError(RuntimeError):
    pass


def _rel(path):
    """Repo-relative path for printing and recording."""
    root = os.path.normpath(os.path.join(CONFIG_DIR, ".."))
    path = os.path.normpath(path)
    return os.path.relpath(path, root) if path.startswith(root + os.sep) else path


def _check(spec, where):
    """Validate a spec dict in the yaml/session.json schema."""
    def fail(msg):
        raise BoardConfigError(f"board config {where}: {msg}")

    def positive(v, key):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0:
            fail(f"{key} is missing, blank or not a positive number")
        return float(v)

    for key in ("rows", "cols"):
        if isinstance(spec.get(key), bool) or not isinstance(spec.get(key), int):
            fail(f"{key} is missing or not an integer")
    if spec["rows"] < 2 or spec["cols"] < 2:
        fail("rows and cols must be >= 2")

    for key in ("square_mm", "marker_mm"):
        sizes = spec.get(key) or {}
        sizes["measured"] = positive(sizes.get("measured"), key + ".measured")
        if sizes.get("nominal") is not None:
            sizes["nominal"] = positive(sizes["nominal"], key + ".nominal")
        spec[key] = sizes
    if spec["marker_mm"]["measured"] >= spec["square_mm"]["measured"]:
        fail("marker_mm must be smaller than square_mm")

    name = spec.get("dictionary")
    if not isinstance(name, str) or not hasattr(cv2.aruco, name):
        fail(f"unknown dictionary {name!r}")
    size = len(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name)).bytesList)
    if spec["rows"] * spec["cols"] // 2 > size:
        fail(f"{name} has too few markers for this board")

    if not isinstance(spec.get("legacy_pattern"), bool):
        fail("legacy_pattern must be true or false")
    return spec


def load(path=CURRENT):
    """Read and validate a board yaml. Raises BoardConfigError."""
    try:
        fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
        opened = fs.isOpened()
    except cv2.error:
        opened = False
    if not opened:
        raise BoardConfigError(f"board config: cannot open {_rel(path)}")

    def value(node):
        if node.empty() or node.isNone():
            return None
        if node.isInt():
            return int(node.real())
        if node.isReal():
            return node.real()
        if node.isString():
            return node.string()
        return None

    # OpenCV 5.x reads `false` as an int, 4.6 as a string; accept both.
    lp = value(fs.getNode("legacy_pattern"))
    legacy = {0: False, 1: True, "false": False, "true": True}.get(lp, lp)

    spec = {
        "source": _rel(path),
        "name": value(fs.getNode("name")) or "",
        "rows": value(fs.getNode("rows")),
        "cols": value(fs.getNode("cols")),
        "square_mm": {k: value(fs.getNode("square_mm").getNode(k))
                      for k in ("nominal", "measured")},
        "marker_mm": {k: value(fs.getNode("marker_mm").getNode(k))
                      for k in ("nominal", "measured")},
        "dictionary": value(fs.getNode("dictionary")),
        "legacy_pattern": legacy,
        "provenance": {k: value(fs.getNode("provenance").getNode(k)) or ""
                       for k in PROVENANCE},
    }
    fs.release()
    return _check(spec, _rel(path))


def for_session(meta, session=""):
    """The board a session was captured with. Raises BoardConfigError."""
    rec = meta.get("board") or {}
    where = os.path.join(session, "session.json")

    if "legacy_pattern" in rec:
        spec = _check({**rec, "square_mm": dict(rec.get("square_mm") or {}),
                       "marker_mm": dict(rec.get("marker_mm") or {})}, where)
        return spec, "recorded in session.json"

    # Pre-config session: its record must match the old board exactly.
    spec = load(OLD)
    recorded = (rec.get("squares_x"), rec.get("squares_y"),
                rec.get("square_mm"), rec.get("marker_mm"), rec.get("dictionary"))
    expected = (spec["cols"], spec["rows"], spec["square_mm"]["measured"],
                spec["marker_mm"]["measured"], spec["dictionary"])
    if recorded != expected:
        raise BoardConfigError(
            f"{where} has no full board record, and its board {recorded} does not "
            f"match {spec['source']} {expected}")
    return spec, f"session.json predates board records; matched to {spec['source']}"


def describe(spec):
    """One line naming the exact board. Same content as BoardSpec::describe()."""
    def g(v):
        return "?" if v is None else f"{v:g}"
    markers = spec["rows"] * spec["cols"] // 2
    corners = (spec["rows"] - 1) * (spec["cols"] - 1)
    p = spec["provenance"]
    return (f"board {spec['name']} from {spec['source']}: {spec['cols']} cols x "
            f"{spec['rows']} rows, square {g(spec['square_mm']['measured'])} mm "
            f"(nominal {g(spec['square_mm'].get('nominal'))}), marker "
            f"{g(spec['marker_mm']['measured'])} mm "
            f"(nominal {g(spec['marker_mm'].get('nominal'))}), {spec['dictionary']}, "
            f"legacy_pattern {str(spec['legacy_pattern']).lower()} -> {markers} markers "
            f"(ids 0-{markers - 1}), {corners} corners; vendor {p.get('vendor') or '?'}, "
            f"measured by {p.get('measured_by') or '?'} on {p.get('measured_date') or '?'}")


def make(spec):
    """(CharucoBoard, CharucoDetector) for a validated spec. OpenCV 5.x."""
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec["dictionary"]))
    board = cv2.aruco.CharucoBoard(
        (spec["cols"], spec["rows"]),
        spec["square_mm"]["measured"], spec["marker_mm"]["measured"], dictionary)
    board.setLegacyPattern(spec["legacy_pattern"])
    return board, cv2.aruco.CharucoDetector(board)


if __name__ == "__main__":
    try:
        print(describe(load(sys.argv[1] if len(sys.argv) > 1 else CURRENT)))
    except BoardConfigError as e:
        sys.exit(str(e))
