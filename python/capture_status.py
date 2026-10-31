"""
Reader for the live status file of 04_capture --status-file PATH.

The writer (lib/capture_status.cpp) replaces the file atomically every
frame, so a reader normally sees a whole file. A torn read is still possible
(e.g. the file on a network filesystem, or edited by hand): a read that
fails to parse is retried once after a short delay, then refused. The
schema is in docs/ARCHITECTURE.md §5.

Usage:
  python capture_status.py PATH          # print the latest status
  python capture_status.py --self-test   # fixture: tests/fixtures/capture_status.json
"""

import datetime, json, os, sys, tempfile, threading, time

SCHEMA = "panoptes.capture_status/1"
FIXTURE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                       "tests", "fixtures", "capture_status.json")
RETRY_DELAY_S = 0.05

REQUIRED = ("schema", "timestamp_utc", "t_ms", "frame", "camera_serial", "dry_run", "session_path",
            "settings", "image", "markers", "corners", "fit_px", "board", "motion",
            "decision", "coverage", "pose_bins", "accepted", "complete")


class StatusError(RuntimeError):
    pass


def _parse(path):
    with open(path) as f:
        text = f.read()
    return json.loads(text)


def read_status(path, retry_delay_s=RETRY_DELAY_S):
    """The latest status as a dict. A read that doesn't parse is retried
    once; a second failure, a missing file, or a wrong schema raises
    StatusError."""
    try:
        try:
            s = _parse(path)
        except (json.JSONDecodeError, UnicodeDecodeError):
            time.sleep(retry_delay_s)
            s = _parse(path)
    except FileNotFoundError as e:
        raise StatusError(f"{path}: no status file (is 04_capture running with "
                          "--status-file?)") from e
    except (json.JSONDecodeError, UnicodeDecodeError) as e:
        raise StatusError(f"{path}: unreadable twice in a row ({e})") from e
    if s.get("schema") != SCHEMA:
        raise StatusError(f"{path}: schema {s.get('schema')!r}, expected {SCHEMA!r}")
    missing = [k for k in REQUIRED if k not in s]
    if missing:
        raise StatusError(f"{path}: missing fields {missing}")
    return s


def age_s(status, now=None):
    """Seconds since the writer stamped this status (wall clock, UTC)."""
    t = datetime.datetime.strptime(status["timestamp_utc"], "%Y-%m-%dT%H:%M:%S.%fZ")
    t = t.replace(tzinfo=datetime.timezone.utc)
    now = now or datetime.datetime.now(datetime.timezone.utc)
    return (now - t).total_seconds()


def full_and_still(status):
    """Every corner found and the still gate passed on this frame."""
    c = status["corners"]
    return bool(c["complete"] and c["found"] == c["total"] and status["motion"]["still"])


def describe(status):
    """One line, for logs."""
    c, b, m = status["corners"], status["board"], status["motion"]
    board = ("no board" if b is None else
             f"centroid ({b['centroid_px'][0]:.1f}, {b['centroid_px'][1]:.1f}) px  "
             f"square {b['square_px'] if b['square_px'] is None else round(b['square_px'], 2)} px  "
             f"roll {b['roll_deg'] if b['roll_deg'] is None else round(b['roll_deg'], 2)} deg")
    med = "-" if m["median_px"] is None else f"{m['median_px']:.3f}"
    return (f"frame {status['frame']}  corners {c['found']}/{c['total']}  {board}  "
            f"motion median {med} px  {'still' if m['still'] else 'NOT still'}  "
            f"{status['decision']['verdict']}")


# ---------------------------------------------------------------- self-test

def _self_test():
    ok = fail = 0

    def check(cond, what):
        nonlocal ok, fail
        print(("  PASS  " if cond else "  FAIL  ") + what)
        ok += bool(cond)
        fail += not cond

    s = read_status(FIXTURE)
    check(s["corners"] == {"found": 88, "total": 88, "complete": True} and
          s["markers"] == {"found": 54, "total": 54}, "fixture: 88/88 corners, 54/54 markers")
    check(s["board"]["centroid_px"] == [640.0, 512.0] and abs(s["board"]["roll_deg"]) < 0.01
          and abs(s["board"]["square_px"] - 39.0) < 0.01, "fixture: centroid, roll, square_px")
    check(s["coverage"]["grid"] == {"cols": 5, "rows": 6, "cell_width_px": 256.0,
                                    "cell_height_px": 170.667, "id": "row * cols + col"}
          and len(s["coverage"]["empty_cells"]) == s["coverage"]["empty_count"],
          "fixture: grid definition, empty-cell list matches its count")
    check(len(s["pose_bins"]["counts"]) == 15 and s["pose_bins"]["counts"]["mid/flat"] == 1
          and s["accepted"] == 1 and s["complete"] is False, "fixture: 15 bins, one accept")
    check(full_and_still(s), "fixture: full_and_still")
    now = datetime.datetime(2026, 10, 28, 18, 0, 2, 500000, tzinfo=datetime.timezone.utc)
    check(abs(age_s(s, now) - 2.5) < 1e-6, "age_s from timestamp_utc")
    print("    " + describe(s))

    with open(FIXTURE) as f:
        whole = f.read()
    d = tempfile.mkdtemp(prefix="panoptes_status_")
    path = os.path.join(d, "status.json")
    try:
        # Torn, then replaced by the writer before the retry.
        with open(path, "w") as f:
            f.write(whole[: len(whole) // 2])

        def replace():
            with open(path + ".tmp", "w") as f:
                f.write(whole)
            os.replace(path + ".tmp", path)
        threading.Timer(0.02, replace).start()
        check(read_status(path, retry_delay_s=0.2)["frame"] == s["frame"],
              "torn read, file replaced before the retry -> read succeeds")

        with open(path, "w") as f:
            f.write(whole[:100])
        try:
            read_status(path, retry_delay_s=0.01)
            check(False, "torn twice -> StatusError")
        except StatusError as e:
            check("unreadable twice" in str(e), "torn twice -> StatusError")

        with open(path, "w") as f:
            f.write(whole.replace(SCHEMA, "panoptes.capture_status/0"))
        try:
            read_status(path)
            check(False, "wrong schema -> StatusError")
        except StatusError as e:
            check("schema" in str(e), "wrong schema -> StatusError")

        not_still = json.loads(whole)
        not_still["motion"]["still"] = False
        with open(path, "w") as f:
            json.dump(not_still, f)
        check(not full_and_still(read_status(path)), "still false -> not full_and_still")

        try:
            read_status(os.path.join(d, "absent.json"))
            check(False, "missing file -> StatusError")
        except StatusError:
            check(True, "missing file -> StatusError")
    finally:
        for name in os.listdir(d):
            os.remove(os.path.join(d, name))
        os.rmdir(d)

    print(f"\ncapture_status self-test: {ok}/{ok + fail} passed")
    return fail == 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    if sys.argv[1] == "--self-test":
        sys.exit(0 if _self_test() else 1)
    try:
        st = read_status(sys.argv[1])
    except StatusError as e:
        sys.exit(f"capture_status: {e}")
    print(f"{describe(st)}  (age {age_s(st):.2f} s)")
