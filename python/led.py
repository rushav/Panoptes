"""
Drives the IR LED (firmware/led/led.ino) and logs when each flash happened, on
the same clock as 08_sync's host_mono_ns. Procedure: docs/RUNBOOK.md §15.

Each flash is logged with t_send_ns (just before the command) and t_reply_ns
(just after the board's reply). The LED went on between the two.

Usage:
  python led.py clock
  python led.py ping --port PORT [--count N]
  python led.py hold --port PORT
  python led.py run  --port PORT [--trials N] [--min-gap S] [--max-gap S] [--label TEXT] [--out DIR] [--seed N]
"""

import argparse, datetime, json, os, random, re, statistics, sys, time

SESSION_ROOT = os.path.join(os.path.expanduser("~"), "panoptes-data", "sessions")
REPLY_TIMEOUT_S = 0.5
FLASH_S = 0.2  # FLASH_MS in firmware/led/led.ino


def mono_ns():
    return time.monotonic_ns()


def open_board(port):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial missing: .venv/bin/pip install -r python/requirements-latency.txt")
    ser = serial.Serial()
    ser.port, ser.baudrate, ser.timeout = port, 115200, REPLY_TIMEOUT_S
    ser.dtr, ser.rts = True, False  # opening the port must not reset the board
    ser.open()
    if command(ser, b"L")[0] != "L":
        sys.exit(f"{port}: the LED firmware did not answer")
    return ser


def command(ser, byte):
    """Send one byte; return (reply, t_send_ns, t_reply_ns). reply is None on a timeout."""
    ser.reset_input_buffer()
    t_send = mono_ns()
    ser.write(byte)
    ser.flush()
    line = ser.readline()
    t_reply = mono_ns()
    return (line.decode("ascii", "replace").strip() or None), t_send, t_reply


def rtt_summary(rtts_ns):
    if not rtts_ns:
        return None
    s = sorted(rtts_ns)
    return {"n": len(s), "min_ms": s[0] / 1e6, "median_ms": statistics.median(s) / 1e6,
            "p95_ms": s[min(len(s) - 1, round(0.95 * (len(s) - 1)))] / 1e6, "max_ms": s[-1] / 1e6}


def print_rtt(summ):
    if summ is None:
        print("round trip: none")
        return
    print(f"round trip: n {summ['n']}  min {summ['min_ms']:.3f}  median {summ['median_ms']:.3f}"
          f"  p95 {summ['p95_ms']:.3f}  max {summ['max_ms']:.3f} ms")


def cmd_clock(_args):
    impl = time.get_clock_info("monotonic").implementation
    print(f"time.monotonic: {impl}")
    ok = impl == "clock_gettime(CLOCK_MONOTONIC)"
    print("matches 08_sync host_mono_ns: " + ("yes" if ok else "NO"))
    return 0 if ok else 1


def cmd_ping(args):
    ser = open_board(args.port)
    rtts, timeouts = [], 0
    for _ in range(args.count):
        reply, t0, t1 = command(ser, b"L")
        if reply == "L":
            rtts.append(t1 - t0)
        else:
            timeouts += 1
        time.sleep(random.uniform(0.002, 0.02))
    print_rtt(rtt_summary(rtts))
    print(f"timeouts: {timeouts}")
    return 0


def cmd_hold(args):
    ser = open_board(args.port)
    if command(ser, b"H")[0] != "H":
        sys.exit("the board did not acknowledge 'H'")
    print("LED ON. Press Enter to turn it off.")
    try:
        input()
    except (KeyboardInterrupt, EOFError):
        pass
    command(ser, b"L")
    print("LED off")
    return 0


def cmd_run(args):
    if args.min_gap < 2 * FLASH_S or args.max_gap < args.min_gap:
        sys.exit(f"need {2 * FLASH_S} <= --min-gap <= --max-gap")
    if args.label and not re.fullmatch(r"[A-Za-z0-9_-]+", args.label):
        sys.exit("--label: letters, digits, - and _ only")
    ser = open_board(args.port)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = os.path.join(args.out, f"led_{args.label}_{stamp}" if args.label else f"led_{stamp}")
    os.makedirs(folder)
    rng = random.Random(args.seed)
    session = {
        "tool": "led", "format": 2, "label": args.label,
        "created": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "ended": "running",
        "host": {"unix_time_s": time.time(), "mono_ns": mono_ns(), "clock": time.get_clock_info("monotonic").implementation},
        "port": args.port, "flash_s": FLASH_S,
        "requested": {"trials": args.trials, "min_gap_s": args.min_gap, "max_gap_s": args.max_gap, "seed": args.seed},
        "summary": None,
    }

    def write_session():
        tmp = os.path.join(folder, "session.json.tmp")
        with open(tmp, "w") as f:
            json.dump(session, f, indent=2)
        os.replace(tmp, os.path.join(folder, "session.json"))

    write_session()
    print(f"writing {folder}/flashes.jsonl")
    print(f"{args.trials} flashes, about {args.trials * (args.min_gap + args.max_gap) / 2:.0f} s")

    rtts, timeouts, bad = [], 0, 0
    ended = "done"
    next_t = time.monotonic() + 1.0
    with open(os.path.join(folder, "flashes.jsonl"), "w") as log:
        try:
            for i in range(args.trials):
                time.sleep(max(0.0, next_t - time.monotonic()))
                reply, t0, t1 = command(ser, b"F")
                log.write(json.dumps({"i": i, "t_send_ns": t0, "t_reply_ns": t1, "reply": reply}) + "\n")
                if reply == "F":
                    rtts.append(t1 - t0)
                elif reply is None:
                    timeouts += 1
                else:
                    bad += 1
                # A random gap, so each flash lands at a random point between camera frames.
                next_t += rng.uniform(args.min_gap, args.max_gap)
                if (i + 1) % 50 == 0:
                    log.flush()
                    print(f"  {i + 1}/{args.trials}")
        except KeyboardInterrupt:
            ended = "interrupted"
            print("\ninterrupted")
        finally:
            try:
                command(ser, b"L")
            except Exception:
                pass

    session["ended"] = ended
    session["summary"] = {"flashes_acknowledged": len(rtts), "timeouts": timeouts,
                          "bad_replies": bad, "round_trip": rtt_summary(rtts)}
    write_session()
    print(f"=== led summary ===\nflashes acknowledged: {len(rtts)} of {args.trials}"
          f"  timeouts: {timeouts}  bad replies: {bad}")
    print_rtt(session["summary"]["round_trip"])
    print(f"wrote {folder}/{{flashes.jsonl,session.json}}")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("clock", help="check the clock matches 08_sync's")
    sp = sub.add_parser("ping", help="time the round trip, LED stays off")
    sp.add_argument("--port", required=True)
    sp.add_argument("--count", type=int, default=1000)
    sp = sub.add_parser("hold", help="LED on until Enter")
    sp.add_argument("--port", required=True)
    sp = sub.add_parser("run", help="flash at random gaps and log every flash")
    sp.add_argument("--port", required=True)
    sp.add_argument("--trials", type=int, default=600)
    sp.add_argument("--min-gap", type=float, default=1.0, help="seconds")
    sp.add_argument("--max-gap", type=float, default=2.0, help="seconds")
    sp.add_argument("--label", default="")
    sp.add_argument("--out", default=SESSION_ROOT)
    sp.add_argument("--seed", type=int, default=None)
    args = p.parse_args(argv)
    return {"clock": cmd_clock, "ping": cmd_ping, "hold": cmd_hold, "run": cmd_run}[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
