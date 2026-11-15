"""
Analysis for the LED latency and sync check: joins the flashes that
python/led.py logged to the frame groups 08_sync recorded at the same time,
and answers

  Q1 sync:    does every camera first see each flash in the same frame?
  Q2 latency: LED on -> frame group taken from the SDK on the host.

Procedure: docs/RUNBOOK.md §15.

Usage:
  python latency.py blobs   <sync_session>
  python latency.py analyze <sync_session> <led_session> [--radius PX] [--window-ms MS] [--out DIR] [--no-plot]
  python latency.py --self-test

<sync_session> is a sync_<label>_<stamp>/ folder (groups.jsonl, session.json);
<led_session> a led_<label>_<stamp>/ folder (flashes.jsonl, session.json).
"""

import argparse, json, math, os, random, statistics, sys, tempfile

REPO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
OUT_DEFAULT = os.path.join(REPO, "analysis", "latency")
FRAME_PERIOD_NOMINAL_S = 1 / 30


# loading

def load_groups(sync_dir):
    """Per camera: list of (frame_id, host_ns, blobs), ascending frame_id.
    Plus the group receive times, for the frame period."""
    cams, hosts = {}, []
    with open(os.path.join(sync_dir, "groups.jsonl")) as f:
        for line in f:
            g = json.loads(line)
            hosts.append(g["host_mono_ns"])
            for c in g["cams"]:
                if not c.get("present") or c.get("unexpected"):
                    continue
                blobs = c.get("blobs", []) if c.get("object", True) else []
                cams.setdefault(c["serial"], []).append((c["frame_id"], g["host_mono_ns"], blobs))
    for s in cams:
        cams[s].sort(key=lambda r: r[0])
    return cams, hosts


def load_flashes(led_dir):
    recs = []
    with open(os.path.join(led_dir, "flashes.jsonl")) as f:
        for line in f:
            recs.append(json.loads(line))
    return recs


def frame_period_s(hosts):
    d = [b - a for a, b in zip(hosts, hosts[1:]) if b > a]
    return statistics.median(d) / 1e9 if d else FRAME_PERIOD_NOMINAL_S


# statistics

def summary(xs):
    if not xs:
        return None
    s = sorted(xs)
    q = lambda p: s[min(len(s) - 1, int(round(p * (len(s) - 1))))]
    return {"n": len(s), "min": s[0], "p5": q(0.05), "median": statistics.median(s),
            "p95": q(0.95), "max": s[-1]}


def _binom_cdf(k, n, p):
    if p <= 0:
        return 1.0
    if p >= 1:
        return 0.0 if k < n else 1.0
    lp, lq = math.log(p), math.log1p(-p)
    return min(1.0, sum(math.exp(math.lgamma(n + 1) - math.lgamma(i + 1) - math.lgamma(n - i + 1)
                                 + i * lp + (n - i) * lq) for i in range(k + 1)))


def upper95(k, n):
    """Two-sided 95% Clopper-Pearson upper bound on a share, k of n."""
    if n == 0:
        return 1.0
    if k >= n:
        return 1.0
    lo, hi = k / n, 1.0
    for _ in range(60):
        mid = (lo + hi) / 2
        if _binom_cdf(k, n, mid) > 0.025:
            lo = mid
        else:
            hi = mid
    return hi


# blobs

def cmd_blobs(args):
    cams, hosts = load_groups(args.sync_session)
    print(f"{args.sync_session}: {len(hosts)} groups")
    print(f"{'serial':>7} {'frames':>7} {'with blob':>10} {'share':>7} {'max blobs':>10} {'median x, y':>16}")
    for s in sorted(cams):
        rows = cams[s]
        lit = [r for r in rows if r[2]]
        xs = [b[0] for r in lit for b in r[2]]
        ys = [b[1] for r in lit for b in r[2]]
        pos = f"{statistics.median(xs):.1f}, {statistics.median(ys):.1f}" if xs else "-"
        print(f"{s:>7} {len(rows):>7} {len(lit):>10} {len(lit) / max(1, len(rows)):>7.1%} "
              f"{max((len(r[2]) for r in rows), default=0):>10} {pos:>16}")
    print("LED off: every share should be 0.0%. LED held on: 100.0% and max blobs 1.")
    return 0


# analyze

def led_positions(cams, flashes, window_ns):
    """Per camera, the median position of every blob seen inside a flash window:
    the LED, if the volume is otherwise empty."""
    pos = {}
    for s, rows in cams.items():
        xs, ys, i = [], [], 0
        for fl in flashes:
            while i < len(rows) and rows[i][1] < fl["t_send_ns"]:
                i += 1
            j = i
            while j < len(rows) and rows[j][1] <= fl["t_send_ns"] + window_ns:
                for b in rows[j][2]:
                    xs.append(b[0]); ys.append(b[1])
                j += 1
        if xs:
            pos[s] = (statistics.median(xs), statistics.median(ys))
    return pos


def is_lit(blobs, p, radius):
    if not blobs:
        return False
    if radius <= 0 or p is None:
        return True
    return any((b[0] - p[0]) ** 2 + (b[1] - p[1]) ** 2 <= radius * radius for b in blobs)


def analyze(sync_dir, led_dir, radius=20.0, window_ms=400.0):
    cams, hosts = load_groups(sync_dir)
    recs = load_flashes(led_dir)
    serials = sorted(cams)
    period = frame_period_s(hosts)
    window_ns = int(window_ms * 1e6)
    flashes = [r for r in recs if r.get("reply") == "F"]
    rec_lo, rec_hi = (min(hosts), max(hosts)) if hosts else (0, 0)
    inside = [fl for fl in flashes if rec_lo <= fl["t_send_ns"] and fl["t_send_ns"] + window_ns <= rec_hi]
    pos = led_positions(cams, inside, window_ns)

    lit = {s: [is_lit(r[2], pos.get(s), radius) for r in cams[s]] for s in serials}
    trials, cam_stats = [], {s: {"seen": 0, "not_seen": 0, "lit_before": 0, "run_lengths": []}
                             for s in serials}
    # Baseline: frames with a blob outside every flash window (stray reflections).
    stray = {s: 0 for s in serials}
    for s in serials:
        rows, k = cams[s], 0
        for i, r in enumerate(rows):
            if not r[2]:
                continue
            while k < len(inside) and inside[k]["t_send_ns"] + window_ns < r[1]:
                k += 1
            in_win = k < len(inside) and inside[k]["t_send_ns"] - 50_000_000 <= r[1]
            if not in_win:
                stray[s] += 1

    idx = {s: 0 for s in serials}
    for fl in inside:
        t0, t1 = fl["t_send_ns"], fl["t_reply_ns"]
        per = {}
        for s in serials:
            rows, L = cams[s], lit[s]
            i = idx[s]
            while i < len(rows) and rows[i][1] < t0:
                i += 1
            idx[s] = i
            j = i
            while j < len(rows) and rows[j][1] <= t0 + window_ns and not L[j]:
                j += 1
            if j >= len(rows) or rows[j][1] > t0 + window_ns:
                cam_stats[s]["not_seen"] += 1
                continue
            # The frame before the first lit one must exist, be the previous ID, and be dark.
            if j == 0 or rows[j - 1][0] != rows[j][0] - 1 or L[j - 1]:
                cam_stats[s]["lit_before"] += 1
                continue
            run = 0
            while j + run < len(rows) and L[j + run] and rows[j + run][0] == rows[j][0] + run:
                run += 1
            cam_stats[s]["seen"] += 1
            cam_stats[s]["run_lengths"].append(run)
            per[s] = (rows[j][0], rows[j][1])
        trials.append({"i": fl["i"], "t_send_ns": t0, "t_reply_ns": t1, "first": per})

    # A constant per-camera frame ID offset is a labelling question, not timing:
    # find it first (median offset to the trial's majority), then judge timing.
    def majority(ids):
        counts = {}
        for v in ids:
            counts[v] = counts.get(v, 0) + 1
        best = max(counts.values())
        return min(v for v, c in counts.items() if c == best)

    full = [t for t in trials if len(t["first"]) == len(serials)]
    raw_off = {s: [] for s in serials}
    for t in full:
        m = majority([v[0] for v in t["first"].values()])
        for s in serials:
            raw_off[s].append(t["first"][s][0] - m)
    const_off = {s: (int(round(statistics.median(raw_off[s]))) if raw_off[s] else 0) for s in serials}

    disagree_trials, per_cam = 0, {s: {"offsets": {}, "disagree": 0} for s in serials}
    raw_disagree = 0
    for t in full:
        if len({v[0] for v in t["first"].values()}) > 1:
            raw_disagree += 1
        adj = {s: t["first"][s][0] - const_off[s] for s in serials}
        m = majority(list(adj.values()))
        any_d = False
        for s in serials:
            d = adj[s] - m
            per_cam[s]["offsets"][d] = per_cam[s]["offsets"].get(d, 0) + 1
            if d != 0:
                per_cam[s]["disagree"] += 1
                any_d = True
        t["disagree"] = any_d
        disagree_trials += any_d

    # Q2: earliest group in which any camera showed the flash.
    lat = {"lo": [], "mid": [], "hi": []}
    rtts = [(t["t_reply_ns"] - t["t_send_ns"]) / 1e6 for t in full]
    for t in full:
        h = min(v[1] for v in t["first"].values())
        lat["hi"].append((h - t["t_send_ns"]) / 1e6)
        lat["lo"].append((h - t["t_reply_ns"]) / 1e6)
        lat["mid"].append((h - (t["t_send_ns"] + t["t_reply_ns"]) / 2) / 1e6)
        t["latency_mid_ms"] = lat["mid"][-1]

    n = len(full)
    out = {
        "sync_session": os.path.basename(os.path.normpath(sync_dir)),
        "led_session": os.path.basename(os.path.normpath(led_dir)),
        "settings": {"radius_px": radius, "window_ms": window_ms},
        "frame_period_ms": period * 1e3,
        "flashes": {"acknowledged": len(flashes), "inside_recording": len(inside), "seen_by_all": n},
        "round_trip_ms": summary(rtts),
        "led_position_px": {str(s): pos.get(s) for s in serials},
        "stray_frames_outside_flashes": {str(s): stray[s] for s in serials},
        "q1": {
            "trials": n,
            "trials_raw_frame_ids_differ": raw_disagree,
            "constant_frame_id_offset": {str(s): const_off[s] for s in serials},
            "trials_disagree": disagree_trials,
            "share_disagree": disagree_trials / n if n else None,
            "share_upper95": upper95(disagree_trials, n) if n else None,
            "spread_upper95_ms": upper95(disagree_trials, n) * period * 1e3 if n else None,
            "per_camera": {str(s): {
                "seen": cam_stats[s]["seen"], "not_seen": cam_stats[s]["not_seen"],
                "lit_before_send": cam_stats[s]["lit_before"],
                "offset_counts": {str(k): v for k, v in sorted(per_cam[s]["offsets"].items())},
                "disagree": per_cam[s]["disagree"],
                "skew_upper95_ms": upper95(per_cam[s]["disagree"], n) * period * 1e3 if n else None,
                "lit_run_frames": summary(cam_stats[s]["run_lengths"]),
            } for s in serials},
        },
        "q2_ms": {"led_on_midpoint": summary(lat["mid"]), "led_on_reply_bound": summary(lat["lo"]), "led_on_send_bound": summary(lat["hi"])},
        "trials": [{"i": t["i"], "latency_mid_ms": t["latency_mid_ms"], "disagree": t["disagree"], "first_frame_id": {str(s): v[0] for s, v in t["first"].items()}} for t in full],
    }
    return out


def print_report(r):
    f = r["flashes"]
    print(f"=== LED sync/latency: {r['sync_session']} + {r['led_session']} ===")
    print(f"flashes: {f['acknowledged']} acknowledged, {f['inside_recording']} inside the recording, "
          f"{f['seen_by_all']} seen by every camera   frame period {r['frame_period_ms']:.4f} ms")
    rt = r["round_trip_ms"]
    if rt:
        print(f"round trip (error bar on LED-on): min {rt['min']:.3f}  median {rt['median']:.3f}  "
              f"p95 {rt['p95']:.3f}  max {rt['max']:.3f} ms")
    q = r["q1"]
    print(f"\nQ1 sync: {q['trials']} trials")
    print(f"  trials whose raw first-lit frame IDs differ: {q['trials_raw_frame_ids_differ']}")
    offs = {s: v for s, v in q["constant_frame_id_offset"].items() if v}
    print(f"  constant frame ID offsets: {offs if offs else 'none (frame ID N is one pulse on all cameras)'}")
    if q["trials"]:
        print(f"  trials with a timing disagreement (after constant offsets): {q['trials_disagree']} "
              f"({q['share_disagree']:.2%}; 95% upper {q['share_upper95']:.2%} "
              f"-> spread across cameras < {q['spread_upper95_ms']:.3f} ms)")
    print(f"  {'serial':>7} {'seen':>5} {'miss':>5} {'early':>5} {'-1':>4} {'0':>5} {'+1':>4} {'other':>5} "
          f"{'skew<95% ms':>11} {'lit frames':>10} {'stray':>6} {'LED x, y':>14}")
    for s, c in q["per_camera"].items():
        oc = c["offset_counts"]
        other = sum(v for k, v in oc.items() if k not in ("-1", "0", "1"))
        runs = c["lit_run_frames"]
        rl = f"{runs['min']}-{runs['max']}" if runs else "-"
        p = r["led_position_px"].get(s)
        ps = f"{p[0]:.0f}, {p[1]:.0f}" if p else "-"
        sk = f"{c['skew_upper95_ms']:.3f}" if c["skew_upper95_ms"] is not None else "-"
        print(f"  {s:>7} {c['seen']:>5} {c['not_seen']:>5} {c['lit_before_send']:>5} {oc.get('-1', 0):>4} "
              f"{oc.get('0', 0):>5} {oc.get('1', 0):>4} {other:>5} {sk:>11} {rl:>10} "
              f"{r['stray_frames_outside_flashes'][s]:>6} {ps:>14}")
    print("\nQ2 latency, LED on -> group received (ms):")
    for k, label in (("led_on_midpoint", "LED-on = round-trip midpoint"),
                     ("led_on_reply_bound", "LED-on = reply (lower bound)"),
                     ("led_on_send_bound", "LED-on = send (upper bound)")):
        s = r["q2_ms"][k]
        if s:
            print(f"  {label:<30} min {s['min']:8.3f}  p5 {s['p5']:8.3f}  median {s['median']:8.3f}  "
                  f"p95 {s['p95']:8.3f}  max {s['max']:8.3f}")


def plot(r, path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.ticker
    except ImportError:
        print("matplotlib missing: no figure (pip install -r python/requirements-latency.txt)")
        return None
    lat = [t["latency_mid_ms"] for t in r["trials"]]
    if not lat:
        return None
    ink, muted, grid, mark = "#1f1f1e", "#6b6b68", "#e6e6e3", "#2a78d6"
    fig, ax = plt.subplots(figsize=(7, 3.6), dpi=200)
    lo, hi = math.floor(min(lat)), math.ceil(max(lat))
    ax.hist(lat, bins=range(lo, hi + 2), color=mark, edgecolor="white", linewidth=1)
    ax.set_xlabel("LED on -> frame group received on host (ms)", color=ink)
    ax.set_ylabel("trials", color=ink)
    rt = r["round_trip_ms"]
    s = r["q2_ms"]["led_on_midpoint"]
    ax.set_title(f"Flash-to-host latency, n = {len(lat)}  (min {s['min']:.1f}, median {s['median']:.1f} ms; "
                 f"LED-on known to ±{rt['median'] / 2:.2f} ms)", fontsize=9, color=ink, loc="left")
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(muted)
    ax.tick_params(colors=muted)
    ax.yaxis.set_major_locator(matplotlib.ticker.MaxNLocator(integer=True))
    ax.yaxis.grid(True, color=grid)
    ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
    return path


def cmd_analyze(args):
    r = analyze(args.sync_session, args.led_session, args.radius, args.window_ms)
    print_report(r)
    os.makedirs(args.out, exist_ok=True)
    base = os.path.join(args.out, r["sync_session"])
    with open(base + ".json", "w") as f:
        json.dump(r, f, indent=1)
    print(f"\nwrote {base}.json")
    if not args.no_plot and plot(r, base + "_latency.png"):
        print(f"wrote {base}_latency.png")
    return 0


# self-test

def synth(folder, n_trials=400, delay_ms=12.0, skew_ms=None, id_offset=None, stray_cam=None,
          seed=7, rtt_ms=1.4):
    """Write a synthetic sync session and LED session with known answers.
    Camera c exposes at t0 + k*T + skew_ms[c]; frame k reaches the host at
    exposure + delay + 0..1 ms. The LED is on from t_on for 200 ms."""
    rng = random.Random(seed)
    serials = [33275, 33276, 33277, 33659, 33660, 33661, 33662, 33663, 33664, 33665]
    skew_ms, id_offset = skew_ms or {}, id_offset or {}
    T = 1e9 / 30
    t0 = 5_000_000_000
    flashes, t = [], t0 + 2e9
    for i in range(n_trials):
        t += rng.uniform(1.0, 2.0) * 1e9
        rtt = rtt_ms * 1e6 * rng.uniform(0.9, 1.6)
        frac = rng.uniform(0.2, 0.8)            # where in the round trip the LED went on
        flashes.append((i, t - frac * rtt, t - frac * rtt + rtt, t))
    n_frames = int((t + 3e9 - t0) / T)
    sync_dir = os.path.join(folder, "sync_synth")
    led_dir = os.path.join(folder, "led_synth")
    os.makedirs(sync_dir); os.makedirs(led_dir)
    fi = 0
    with open(os.path.join(sync_dir, "groups.jsonl"), "w") as f:
        for k in range(n_frames):
            host = int(t0 + k * T + delay_ms * 1e6 + rng.uniform(0, 1e6))
            cams = []
            for s in serials:
                exp = t0 + k * T + skew_ms.get(s, 0) * 1e6
                while fi < len(flashes) and flashes[fi][3] + 0.2e9 < exp:
                    fi += 1
                on = any(fl[3] <= exp < fl[3] + 0.2e9 for fl in flashes[max(0, fi - 1):fi + 1])
                blobs = [[600.0 + s % 7, 500.0 + s % 5, 12.0, 0.9]] if on else []
                if s == stray_cam and k % 97 == 0:
                    blobs.append([100.0, 100.0, 5.0, 0.5])
                cams.append({"serial": s, "present": True, "frame_id": 7000 + k + id_offset.get(s, 0), "object": True, "blobs": blobs})
            f.write(json.dumps({"g": k, "host_mono_ns": host, "group_frame_id": 7000 + k, "cams": cams}) + "\n")
    with open(os.path.join(led_dir, "flashes.jsonl"), "w") as f:
        for i, ts, tr, ton in flashes:
            f.write(json.dumps({"i": i, "t_send_ns": int(ts), "t_reply_ns": int(tr), "reply": "F"}) + "\n")
    return sync_dir, led_dir


def self_test():
    checks = []

    def check(name, ok, detail=""):
        checks.append(ok)
        print(f"  [{'pass' if ok else 'FAIL'}] {name} {detail}")

    with tempfile.TemporaryDirectory() as d:
        print("clean rig: delay 12 ms, no skew, no offsets, a stray reflection on 33661")
        os.makedirs(os.path.join(d, "a"))
        r = analyze(*synth(os.path.join(d, "a"), stray_cam=33661))
        q, lat = r["q1"], r["q2_ms"]["led_on_midpoint"]
        check("every flash seen by all ten", r["flashes"]["seen_by_all"] == 400, f"({r['flashes']['seen_by_all']})")
        check("zero disagreements", q["trials_disagree"] == 0 and q["trials_raw_frame_ids_differ"] == 0)
        check("no constant offsets", not any(q["constant_frame_id_offset"].values()))
        check("bound 95% upper ~0.92% at k=0, n=400", abs(q["share_upper95"] - 0.00918) < 0.0005,
              f"({q['share_upper95']:.4%})")
        check("min latency ~ pipeline delay (12-13 ms)", 11.5 <= lat["min"] <= 13.5, f"({lat['min']:.3f})")
        check("median latency ~ delay + T/2 (~29 ms)", 26.5 <= lat["median"] <= 31.5, f"({lat['median']:.3f})")
        check("spread ~ one frame", 31 <= lat["max"] - lat["min"] <= 35, f"({lat['max'] - lat['min']:.3f})")
        check("stray blob not counted as the LED", q["per_camera"]["33661"]["seen"] == 400
              and r["stray_frames_outside_flashes"]["33661"] > 0)
        runs = q["per_camera"]["33664"]["lit_run_frames"]
        check("lit run is 6-7 frames for 200 ms", runs["min"] >= 6 and runs["max"] <= 7, f"({runs['min']}-{runs['max']})")

        print("skewed rig: 33276 exposes 3 ms late; 33663 carries frame IDs one ahead")
        os.makedirs(os.path.join(d, "b"))
        r = analyze(*synth(os.path.join(d, "b"), skew_ms={33276: 3.0}, id_offset={33663: 1}, seed=12))
        q = r["q1"]
        check("constant +1 offset found on 33663 only",
              q["constant_frame_id_offset"]["33663"] == 1
              and sum(1 for v in q["constant_frame_id_offset"].values() if v) == 1)
        check("raw IDs differ in every trial", q["trials_raw_frame_ids_differ"] == q["trials"])
        share = q["per_camera"]["33276"]["disagree"] / q["trials"]
        check("33276 disagrees in ~9% of trials (3/33.3 ms)", 0.05 <= share <= 0.14, f"({share:.2%})")
        # A late exposure catches an edge the others just missed: one frame EARLY, never late.
        check("33276 is only ever one frame early (-1)",
              q["per_camera"]["33276"]["offset_counts"].get("1", 0) == 0
              and q["per_camera"]["33276"]["offset_counts"].get("-1", 0) == q["per_camera"]["33276"]["disagree"])
        others = sum(c["disagree"] for s, c in q["per_camera"].items() if s != "33276")
        check("no other camera disagrees", others == 0, f"({others})")
        p = os.path.join(d, "fig.png")
        made = plot(r, p)
        check("figure written (or matplotlib absent)", made is None or os.path.getsize(p) > 1000)
    print(f"self-test: {sum(checks)}/{len(checks)} passed")
    return 0 if all(checks) else 1


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv == ["--self-test"]:
        return self_test()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sp = sub.add_parser("blobs", help="per-camera blob counts: the LED-off and LED-on checks")
    sp.add_argument("sync_session")
    sp = sub.add_parser("analyze", help="Q1 sync and Q2 latency")
    sp.add_argument("sync_session")
    sp.add_argument("led_session")
    sp.add_argument("--radius", type=float, default=20.0,
                    help="px around each camera's LED position that count as the LED; 0 = any blob")
    sp.add_argument("--window-ms", type=float, default=400.0,
                    help="how long after the send to look for the first lit frame")
    sp.add_argument("--out", default=OUT_DEFAULT)
    sp.add_argument("--no-plot", action="store_true")
    args = p.parse_args(argv)
    return {"blobs": cmd_blobs, "analyze": cmd_analyze}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
