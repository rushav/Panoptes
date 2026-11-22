# Runbook

Commands first (§1), then one procedure per task. Every flag and key here is
taken from the source's argument parsing. Run everything from the repo root.

- How it works: [ARCHITECTURE.md](ARCHITECTURE.md)
- What was measured, the rules, open items: [FINDINGS.md](FINDINGS.md)
- Camera specs and inventory: [hardware/cameras.md](hardware/cameras.md)

`<serial>` is required; `[serial]` is optional and defaults to **33661**.
`<session>` is `"$HOME"/panoptes-data/sessions/intrinsics_<serial>_<YYYYmmdd_HHMMSS>`.

---

## 1. Quick reference

### 1.1 Environment setup
Once per machine, and again after moving the repo (the venv records its path). Details: §2.
```sh
sudo apt install build-essential cmake libopencv-dev python3.12-venv tcpdump
rm -rf .venv && python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r python/requirements.txt
.venv/bin/python -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco,'CharucoDetector'))"
```
expect: the last line prints a 5.x version and `True`.

### 1.2 Build + setcap
After every pull or code change; relinking strips capabilities. Fresh build: `rm -rf src/0*/build tests/build` first. Details: §3.
```sh
for t in src/0*/; do cmake -S "$t" -B "$t/build" -DCMAKE_BUILD_TYPE=Release && cmake --build "$t/build" -j"$(nproc)" || break; done
for b in src/01_discover/build/discover src/02_stream/build/stream src/03_detect/build/detect src/04_capture/build/capture src/05_identify/build/identify src/06_focus/build/focus src/07_mask/build/mask src/07_objdiag/build/objdiag src/08_sync/build/sync; do sudo setcap cap_net_raw,cap_net_admin+eip "$b"; done
getcap src/*/build/*
cmake -S tests -B tests/build -DCMAKE_BUILD_TYPE=Release && cmake --build tests/build -j"$(nproc)" && tests/build/test_autocapture && tests/build/test_sync
```
expect: `[100%] Built target` for all 9 tools; `getcap` lists all 9 with `cap_net_admin,cap_net_raw=eip`; `test_autocapture: 63/63 passed`; `test_sync: 71/71 passed`.

### 1.3 Network, discover, identify
Start of every session. Details: §4.
```sh
ip -brief link show enp6s0
sudo tcpdump -i enp6s0 -n port 13013
src/01_discover/build/discover
src/05_identify/build/identify [fps] [exposure] [threshold] [gain] [--size WxH]
src/07_objdiag/build/objdiag <serial> [exposure] [threshold] [gain] [fps] [--sweep]
```
expect: `UP ... LOWER_UP`; packets from 169.254.x.x; `settled on 10 camera(s)`, every row `Initialized`; every `identify` tile shows fps > 0.

### 1.4 Stream, detect, focus
One camera in grayscale. Details: §5.
```sh
src/02_stream/build/stream [serial]
src/03_detect/build/detect [serial] [exposure] [gain] [fps] [--board FILE] [--legacy | --no-legacy]
src/06_focus/build/focus <serial>
```
expect: `wrote frame.png`; `detect` at 1–1.5 m reads `markers 54/54   corners 88/88` with `fit` ≈ 1 px (under 3 px near the frame edge); `focus` back at 100% of peak.

### 1.5 Blocking masks
Learn and `--load` need an **empty** volume; `--coverage` needs one marker. Details: §6.
```sh
src/07_mask/build/mask
src/07_mask/build/mask --load
src/07_mask/build/mask --show
src/07_mask/build/mask --coverage
src/07_mask/build/mask --probe
src/07_mask/build/mask --clear
src/07_mask/build/mask --load --config config
```
expect: `residual: none. Volume reads empty on every camera.` and no `LEAK`; `--coverage` prints `PASS` (none recorded yet).

### 1.6 Corner jitter session
Board and camera fixed on tripods, 200 captures. Details: §7.
```sh
src/04_capture/build/capture [serial] [--gain N] [--intensity N] [--exposure US]
.venv/bin/python python/corner_jitter.py <session> [--max-fit PX] [--out DIR]
.venv/bin/python python/corner_jitter.py <png_folder> --board config/charuco_board.yaml
.venv/bin/python python/corner_jitter.py --self-test
```
expect: `frames: 200 total, 200 kept`, radial median ≈ 0.054 px at gain 7, no `WARNING: a corner moved`; self-test `9/9 passed`.

### 1.7 Manual intrinsics session
Focus, check, capture, solve. Details: §8.
```sh
src/06_focus/build/focus <serial>
src/03_detect/build/detect <serial>
src/04_capture/build/capture [serial] [--gain N] [--intensity N] [--exposure US] [--board FILE]
.venv/bin/python python/fit_intrinsics.py <session> [<session> ...]
```
expect: `empty cells 0/30` in green with `tilted` ≥ 30%; fx and fy within 1% of each other.

### 1.8 Auto intrinsics session (UNVALIDATED)
Tune with `--auto-dry` (writes nothing), then run. Not yet validated against manual capture. Details: §9.
```sh
src/04_capture/build/capture [serial] [--gain N] [--intensity N] [--exposure US] --auto-dry [--auto-config FILE] [--board FILE] [--status-file PATH]
src/04_capture/build/capture [serial] [--gain N] [--intensity N] [--exposure US] --auto [--auto-config FILE] [--board FILE] [--status-file PATH]
.venv/bin/python python/fit_intrinsics.py <auto_session> <manual_session>
.venv/bin/python python/capture_status.py PATH
.venv/bin/python python/capture_status.py --self-test
```
expect: `ended: complete: 0 empty cells and every pose bin has >= 2 frames`; validation passes at fx/fy within ~1 px, cx/cy within ~3 px; `capture_status.py PATH` prints the latest frame's line (`STALLED ...` during a stall); self-test `13/13 passed`. A stall prints `no frames from SERIAL for N s (state ...)`, then `restarting camera SERIAL` from 5 s; the session carries on in the same folder (§9).

### 1.9 Python fits and studies
venv interpreter only. Details: §10.
```sh
.venv/bin/python python/fit_intrinsics.py <session> [<session> ...]
.venv/bin/python python/fit_intrinsics.py --write-camera [--force] [--status TEXT] <session> [<session> ...]
.venv/bin/python python/compare_models.py <session>
.venv/bin/python python/bootstrap_intrinsics.py <session> [n_subsets] [subset_size]
.venv/bin/python python/board.py [board.yaml]
.venv/bin/python python/triangulate.py [layout.yaml]
.venv/bin/python python/accuracy_study.py [--allow-unmeasured] [layout.yaml] [study.yaml]
.venv/bin/python python/volume_coverage.py [--self-test] [--allow-unmeasured] [layout.yaml] [coverage.yaml]
.venv/bin/python python/test_arm_capture.py
```
expect: `triangulate.py` prints `self-test: 12/12 passed` (plus `note: layout is unmeasured ...` while `config/layout.yaml` has `measured: 0`); `test_arm_capture.py` prints `186/186 passed`; the study prints `refusing to run: ... layout is UNMEASURED` and exits 1 while `measured: 0`, otherwise `Wrote .../accuracy_study.json`; `volume_coverage.py --self-test` prints `self-test: 9/9 passed`, and a full run refuses the same way while `measured: 0`, otherwise prints `Wrote .../analysis/coverage/volume_coverage.json`.

### 1.10 Git
Changes are left uncommitted for review. Details: §11.
```sh
git status
git diff --stat
git diff
git add <files>
git commit
git log --oneline -10
```
expect: `git status` clean after the commit; no `build/`, `.venv/` or `analysis/*.png` staged.

### 1.11 Arm (Kinova Gen3)
E-stop pendant in reach, nothing inside the box. After power-up the arm rests on its joint stops: run `status`, and if a `limits` margin is negative, jog that joint to mid-range with the Web App before anything else. Motion only with `--yes`; first move is `move --dz 0.02 --yes`. Teach the eight viewpoints once per easel position, not per camera; again if the easel or arm base moves. Per camera: unmount it from the rig, mount it on the arm and tie its cable down at the bracket with slack through the wrist, unfold the arm, teach (once per easel position), probe, session twice, fit with `--status` by the flag rule (marginal if an fx/fy range is over 1.0 px or cx/cy over 3.0 px); capture at `--gain 0 --intensity 15 --exposure 8133`. All ten are done: the next stage is to remount all ten, re-learn masks once, then extrinsics (not built yet). Procedures in §13. Save a `hub` pose (board-centred, every joint mid-range) with `save-pose hub`: a session whose viewpoints span several arm configurations goes through it, and refuses to start without it. The continuous joints' cable turns are counted from the hub's angles and kept within ±`cable_window_deg` (`config/arm.yaml`); start with the cable unwrapped at the hub. Never start an `--orbit` session from Home: `save-pose` a board-centred pose and start from it. Camera on the arm, board on an easel ~1.0 m from the camera, leaned back 10–12°, P0 near x 0.43 m. The probe needs `capture --auto-dry --status-file`, the session `capture --auto --status-file`; board centred and fully visible for both. Details: §13.
```sh
.venv/bin/pip install --no-deps -r python/requirements-arm.txt
ip -brief addr show enp6s0
.venv/bin/python python/arm.py status [--ip IP] [--port PORT] [--username U] [--password P]
.venv/bin/python python/arm.py move [--dx M] [--dy M] [--dz M] [--dtx DEG] [--dty DEG] [--dtz DEG] [--yes]
.venv/bin/python python/arm.py goto X Y Z TX TY TZ [--yes]
.venv/bin/python python/arm.py joints J1 J2 J3 J4 J5 J6 [--speed-constraint DEG_S] [--yes]
.venv/bin/python python/arm.py save-pose NAME [--force]
.venv/bin/python python/arm.py list-poses
.venv/bin/python python/arm.py goto-pose NAME [--speed-constraint DEG_S] [--yes]
.venv/bin/python python/arm.py stop
.venv/bin/python python/arm_capture.py probe --status-file PATH [--yes] [--ip IP] [--port PORT] [--username U] [--password P]
.venv/bin/python python/arm_capture.py teach --status-file PATH [--force] [--ip IP] [--port PORT] [--username U] [--password P]
.venv/bin/python python/arm_capture.py session --status-file PATH [--yes] [--poses | --orbit] [--from-pose NAME] [--speed-constraint DEG_S] [--ip IP] [--port PORT] [--username U] [--password P]
```
expect: `enp6s0` lists `192.168.1.11/24` and a `169.254.x.x` address; `status` prints a `limits` line per actuator (`limit [low, high], margin M deg` or `continuous`) and ends `session closed`; `move`/`goto`/`joints`/`goto-pose` without `--yes` end `dry run: nothing sent`; `joints`/`goto-pose` print a `cable` line (each continuous joint's value to send, `short way` or `long way round: unwinding`, and its turns before and after) and, past 180° on one, a `waypoints` line; with `--yes`, `result     END` and an `error` line; `save-pose` ends `saved      NAME in config/arm_poses.yaml`; `probe --yes` prints six probe lines, the mapping, and `wrote config/arm_camera.yaml`; `teach` ends with `far/near` and `families` lines; `session` prints the `taught` line, all 27 viewpoints, `checks` and `families` (and `hub` if one is needed) and a `cable` line before anything moves (`--orbit`: `board` and `cap`); `session --yes` prints `NAME: viewpoint abandoned (...)` or `NAME: viewpoint at reach edge (...)` when it gives up a viewpoint and moves on, ends every pose line with `turns J1 … J4 … J6 …`, and ends with a `summary` block (its `cable` line gives the net turns, 0 for each joint after a return to P0) and writes `session_arm.json` into the capture's session folder.

### 1.12 Frame sync
All ten cameras, masks learned at `config/mask.yaml`'s settings, from the repo root. Startup takes about a minute. Details: §14.
```sh
src/08_sync/build/sync --label empty
src/08_sync/build/sync --label marker
src/08_sync/build/sync [--seconds N] [--label TEXT] [--delivery complete|favor-timely|timely] [--cameras N] [--warmup S] [--config DIR] [--out DIR]
```
expect: ten rows reading `object` and `yes`; `cModuleSync: 10 cameras, delivery complete (asked complete), incomplete groups allowed`; after 60 s a `=== frame sync summary ===` block with `complete 1800 of 1800 (100.00%)`, `group rate 30.000 Hz`, `0 gaps`, and `wrote .../sync_<label>_<timestamp>/{groups.jsonl,session.json}`.

---

## 2. Environment setup

```sh
sudo apt install build-essential cmake libopencv-dev python3.12-venv tcpdump
rm -rf .venv && python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r python/requirements.txt
.venv/bin/python -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco,'CharucoDetector'))"
```

1. **System packages.**
   - `libopencv-dev` is OpenCV **4.6**, for every C++ tool.
   - `python3.12-venv` is **not installed by default on Ubuntu 24.04**;
     without it `python3 -m venv` fails confusingly.
   - `tcpdump` is for the network check (§4).
2. **OptiTrack Camera SDK** at `~/optitrack/CameraSDK`, outside the repo.
   Each `src/*/CMakeLists.txt` sets `SDK_DIR` to
   `$ENV{HOME}/optitrack/CameraSDK`; if yours is elsewhere, edit it there
   (there is no command-line override). Tools link `CameraLibrary` and
   `jpeg` from `$SDK_DIR/lib`.
3. **Python venv.** Required: Ubuntu 24.04 refuses `pip install` outside one.
   Versions are pinned in `python/requirements.txt`. The `rm -rf .venv` in
   the block makes the same lines work for a first install and for a rebuild.
4. **Verify** with the last line: a 5.x version and `True`.
5. **Keep both OpenCVs.** C++ uses 4.6, Python 5.x; don't unify them
   (FINDINGS §2 rule 13, §18).
6. **To refresh the pins** after deliberately changing a package:
   `.venv/bin/pip freeze > python/requirements.txt`.

**Troubleshooting**

- **Python scripts fail to start after the repo moved or was renamed.** The
  venv records its absolute path. Rerun the block (it deletes and recreates
  `.venv`).

---

## 3. Build and setcap

The command block is §1.2; it appears only there.

1. Each tool under `src/` is its own CMake project; there is no top-level
   build. The first line of §1.2 builds them all; after a change to build
   configuration, delete `src/0*/build` first.
2. **setcap after every build.** The camera network needs raw sockets, and
   **linking strips file capabilities**, so the second line is needed after
   every relink. `getcap` lists the binaries that have them; a missing one
   has none.
3. **Never run the tools under `sudo` instead.** It breaks desktop audio for
   the session and leaves root-owned files.
4. **Unit tests** (last line). `tests/` is its own CMake project, outside the
   `src/0*/` loop, with no camera SDK and no setcap. Run both from the repo
   root; each exits 0 only if every check passes.
   - `test_autocapture` reads `config/charuco_board.yaml` and
     `config/autocapture.yaml`. It covers the `--auto` accept logic on
     synthetic corners and what auto mode writes to `session.json`.
   - `test_sync [config_dir]` (default `config`) reads `config/mask.yaml`
     and `config/masks/`. It covers the `groups.jsonl` line, the frame-sync
     statistics on synthetic groups, and the mask-file reader. Its last
     check fails if any saved mask is unreadable or was learned under
     settings other than `mask.yaml`'s: the same condition under which
     `sync` refuses to run.
   `KEEP_TEST_SESSION=1 tests/build/test_autocapture` leaves the test's
   session folder in the system temp directory, for a `json.load` check.

| Tool | Binary |
|---|---|
| 01_discover | `src/01_discover/build/discover` |
| 02_stream | `src/02_stream/build/stream` |
| 03_detect | `src/03_detect/build/detect` |
| 04_capture | `src/04_capture/build/capture` |
| 05_identify | `src/05_identify/build/identify` |
| 06_focus | `src/06_focus/build/focus` |
| 07_mask | `src/07_mask/build/mask` |
| 07_objdiag | `src/07_objdiag/build/objdiag` |
| 08_sync | `src/08_sync/build/sync` |
| tests | `tests/build/test_autocapture`, `tests/build/test_sync` |

**Troubleshooting**

- **`0.0 fps`, `NO FRAMES`, "no cameras opened", or `camera N not found`
  right after a build.** Run `getcap src/*/build/*`; a binary missing from
  the output needs the setcap line again.
- **`mask` can't find its config after the repo moved.** `07_mask` compiles
  in the absolute path of `config/` at configure time. Delete
  `src/07_mask/build`, rebuild, setcap; or pass `--config config`.

---

## 4. Network, discovery and object mode

```sh
ip -brief link show enp6s0
sudo tcpdump -i enp6s0 -n port 13013
src/01_discover/build/discover
src/05_identify/build/identify [fps] [exposure] [threshold] [gain] [--size WxH]
src/07_objdiag/build/objdiag <serial> [exposure] [threshold] [gain] [fps] [--sweep]
```

1. **Link.** The cameras are on PoE from a switch on host NIC **`enp6s0`**,
   dedicated and link-local only (internet comes over Wi-Fi). Want `UP ...
   LOWER_UP`. If it isn't configured, one way with NetworkManager is
   `nmcli connection add type ethernet ifname enp6s0 con-name cameras ipv4.method link-local ipv6.method disabled`
   (*unverified*: how this host was set up is not recorded).
2. **Broadcasts.** Cameras self-assign 169.254.x.x and broadcast on UDP
   13013. Want packets from 169.254.x.x. Ctrl-C to stop.
3. **`discover`** takes no arguments. It prints the device count as it grows,
   waits until the count has been stable for 5 s (30 s cap), then prints
   `settled on N camera(s)` and one line per unique serial: revision, name,
   `Initialized` or `not ready`.
4. **`identify`** opens every initialised camera in object mode, tiled five
   across on the primary monitor (size from `xrandr`; `--size WxH`
   overrides), each tile labelled with serial, fps and blob count.
   Positional defaults: fps 30, exposure 500, threshold 200, gain 0. At start
   it prints each camera's mode: `object` is normal, `NO FRAMES` means
   nothing arrived, `WRONG MODE` see troubleshooting. Expect about a minute
   before ten cameras stream: discovery repeats once per camera
   (*inferred*, not timed). It does not load, clear or report masks.

   | Key | Action |
   |---|---|
   | `g` | Grayscale mode (exposure 8133, gain 7) |
   | `o` | Object mode (launch exposure, gain 0) |
   | `+` / `=` | Threshold +10, up to the camera's max |
   | `-` | Threshold −10, down to the camera's min |
   | `]` | Exposure +100, capped at 33133 (correct only at 30 Hz) |
   | `[` | Exposure −100, down to 10 |
   | `d` | Dump every blob (x, y, area, roundness) to the console |
   | `q` | Quit |

5. **`objdiag`** is for one camera that shows nothing. Console only.
   Positional defaults: exposure 500, threshold 200, gain 0, fps 30. Give the
   positionals in order before `--sweep`. It prints:
   1. the SDK state after start: mode, every setting with its range, IR ring,
      blocking-mask flag, sync support
   2. what was asked against what the camera reports, flagging a threshold
      outside the camera's range
   3. per received frame: frame ID, gap since the last (`dF`, dropped
      frames), `type`, object count, readback threshold and exposure, empty
      polls between; a summary every 5 s. Ctrl-C to stop.

   `--sweep` instead steps the threshold across the camera's range (13
   steps, 1.5 s each) and prints the peak object count at each.

**Troubleshooting**

- **`NO-CARRIER`.** Nothing plugged in, or the cameras are unpowered.
- **Link up, silence on 13013.** The cameras aren't broadcasting.
  Power-cycle the PoE switch.
- **Fewer cameras than exist.** Enumeration is staggered. `discover`,
  `capture`, `identify`, `focus`, `mask` and `objdiag` wait for 5 s of no
  change; `stream` and `detect` use a 1.5 s window that has undercounted ten
  cameras, so if either says `camera N not found`, run `discover` first and
  retry. A serial the SDK lists twice is collapsed automatically.
- **A tool hangs at start right after another was killed.** Old sockets are
  still bound. Wait a couple of seconds.
- **`WRONG MODE`** (`identify`, `mask`). Probably a false alarm: the flag
  describes the last *poll*, and a poll with no frame waiting reads as "not
  object" (FINDINGS, Open items #2). Check with `objdiag <serial>`: a `type`
  column of `Object` means the camera is fine.
- **`0.0 fps` / `NO FRAMES`.** `getcap` first (§3), then the network
  (steps 1–2). `objdiag` prints `[no frames at all - not a threshold problem]`
  in that case.
- **Frames arrive but 0 blobs.** Lighting, exposure or threshold, not
  acquisition. Run `objdiag <serial> --sweep`, and check the requested
  threshold is inside the camera's range (`objdiag` flags it).
- **A camera is blind in one region.** It may be a blocking mask left from
  `07_mask`; see §6.
- **Exposure won't go higher, or reads back lower than asked.** The ceiling is
  `floor(1e6/fps) − 200` (§12) and changes with the frame rate. `identify`'s
  `]` is hardcoded to 33133. Compare `asked` with the readback `identify` and
  `objdiag` print.

---

## 5. Grayscale stream, detection and focus

```sh
src/02_stream/build/stream [serial]
src/03_detect/build/detect [serial] [exposure] [gain] [fps] [--board FILE] [--legacy | --no-legacy]
src/06_focus/build/focus <serial>
```

1. **`stream`** opens one camera in grayscale with exposure 8133, intensity
   15, 30 Hz (set directly, not from `Settings`), discards stale
   non-grayscale frames, and writes `frame.png` to the **current directory**.
   Exit 1 if no grayscale frame arrives.
2. **`detect`** prints the board it loaded (and any `--legacy` /
   `--no-legacy` override), then shows markers and corners live with
   `markers N/M   corners N/C   ids>K n (max I)   fit F px   mean B`, also
   printed about once a second. Positional defaults: serial 33661, exposure
   8133, gain 7, fps 30; board `config/charuco_board.yaml`, `legacy_pattern`
   from the file.
   - `ids>K n`: markers with an ID beyond the board's range (misreads or
     another target).
   - `fit`: worst homography residual in px; −1 with fewer than 6 corners. A
     wrong layout still gives a full corner count; only `fit` shows it: about
     1 px right, ~200 px wrong (FINDINGS §7).

   | Key | Action |
   |---|---|
   | `s` | Save the raw frame to `./detect_frame.png` |
   | `q` | Quit |

3. **`focus`** runs at the grayscale operating point and shows the variance
   of the Laplacian over the centre half (outlined), its peak, the current
   value as a percentage of peak, and a history strip. Put the board in the
   centre rectangle at working distance, twist the lens past the peak and
   back, lock the ring. Focus with the IR filter in. The number is only for
   finding the peak; it isn't comparable between cameras or scenes.

   | Key | Action |
   |---|---|
   | `r` | Reset the peak |
   | `q` | Quit |

**Troubleshooting**

- **Board looks dark in `detect`.** Raise the exposure (the **second**
  positional, after the serial; up to 33133 at 30 Hz) or the gain (third).
- **`board config ...: square_mm.measured is missing`** (`detect`,
  `capture`). Fill in the caliper measurements in
  `config/charuco_board.yaml`. Nominal is never used in their place.
- **`camera N not found`.** `stream` and `detect` use the short discovery
  window; run `discover` first (§4).
- **Full corner count but `fit` in the hundreds.** Wrong layout or rows/cols
  swapped in the board file (FINDINGS §9).

---

## 6. Blocking masks

```sh
src/07_mask/build/mask
src/07_mask/build/mask --load
src/07_mask/build/mask --show
src/07_mask/build/mask --coverage
src/07_mask/build/mask --probe
src/07_mask/build/mask --clear
src/07_mask/build/mask --load --config config
```

Every mode opens every camera at the `config/mask.yaml` settings (§12).
`--config DIR` works with any mode.

| Mode | Does | Volume |
|---|---|---|
| *(none)* | Learn: clear masks, 5 s countdown, 30 s learn, build, apply, 10 s verify, save to `config/masks/<serial>.json` | **Empty** of markers |
| `--load` | Apply saved masks, then verify for 10 s | Empty |
| `--show` | Apply saved masks; tiled view with masked cells tinted red | Any |
| `--coverage` | Apply saved masks; walk **one** marker through the volume; log to `logs/coverage_<stamp>.csv` | One marker |
| `--probe` | Measure SDK mask behaviour step by step | Empty |
| `--clear` | Remove masks from every camera | Any |

1. **Learn** with the volume empty: launch, step out during the 5 s
   countdown, wait for the save.
2. **Load** at the start of a tracking session; check the verify table reads
   `residual: none` with no `LEAK` line.
3. **Look** with `--show`:

   | Key | Action |
   |---|---|
   | `m` | Mask on/off, to see what it hides |
   | `g` | Grayscale view |
   | `o` | Object view |
   | `q` | Quit |

4. **Accept** with `--coverage`: walk one marker through the whole volume,
   then `q`. It prints PASS or FAIL: FAIL if any moment was seen by fewer
   than 3 cameras. It also lists, per camera, the losses next to its mask.
5. **Masks stay on the cameras after `mask` exits.** They're valid only
   while no camera moves and only at the settings they were learned under.

**Troubleshooting**

- **A camera sees nothing in one region, or loses a marker in one place.**
  Likely a mask: `identify`, `objdiag` and `capture` don't report them, and
  `objdiag`'s "blocking mask" line is the SDK flag, not the camera. Compare
  with `mask --show` and `m`; rule masks out with `mask --clear`.
- **`error: cannot read .../mask.yaml`.** The repo moved since `mask` was
  built. Rebuild it (§3) or pass `--config config`.
- **`REFUSED - saved grid ...`** on `--load`. The saved mask doesn't match the
  camera's grid. Relearn.
- **`WARNING settings differ from learn time`.** Relearn, or run at the
  `config/mask.yaml` settings the masks were learned under.
- **`<-- TOO MUCH`** during learn. The camera stares at other rings. Downtilt
  or re-aim it and relearn rather than masking over 10% of its frame.

---

## 7. Corner jitter session

```sh
src/04_capture/build/capture [serial] [--gain N] [--intensity N] [--exposure US]
.venv/bin/python python/corner_jitter.py <session> [--max-fit PX] [--out DIR]
.venv/bin/python python/corner_jitter.py <png_folder> --board config/charuco_board.yaml
.venv/bin/python python/corner_jitter.py --self-test
```

Measures how far a detected corner moves between frames when nothing moves:
the figure in FINDINGS §6 and the noise the accuracy study assumes. Record it
with `04_capture`, not `02_stream`: `capture` writes every frame to a session
with the board and per-frame settings, and only accepts full boards. Each
capture is a new camera frame (*inferred* from `lib/camera.cpp`), and the
script also rejects byte-identical frames.

1. **Mount both.** Board on a tripod, camera fixed. Nobody touches either.
2. **Place** the board at 1–1.5 m, fully in view, roughly centred.
3. **Tilt about 20°** about the vertical axis. To check: run
   `capture <serial>`, press space once, read `tilt N deg` in the printed
   line, `u` to undo; adjust and repeat.
4. **Operating point.** `capture` starts at gain 7, 8133 µs, intensity 15,
   30 Hz, filter in. For a gain-0 comparison, start a separate session and
   press `[` seven times before the first capture. Don't change settings
   mid-session; the script rejects frames whose settings differ from the
   majority.
5. **Capture 200 frames.** Hold or repeat space until `captured 200` (about
   30 s at 6–8 fps), then `q`.
6. **Analyse** with the second line. `--max-fit` defaults to 3 px; `--out`
   defaults to the session folder, where it writes `corner_jitter.json` and
   `corner_jitter.png`. For a plain PNG folder use the third line.
7. **Read.** It prints the board, frames kept and rejected with reasons,
   corners used, the fit of kept frames, and radial and per-axis std
   (median, p90, max) for all corners, centre and edge cells.
   - Compare like with like: radial is √2 × per-axis for isotropic noise;
     FINDINGS §6 quotes radial medians.
   - The edge rows are empty unless the board reaches the outer ring of
     cells. For an edge figure, record a second session with the board near a
     frame corner.
8. **Record** the result in FINDINGS §6 with the session names.

**Troubleshooting**

- **`WARNING: a corner moved more than 1 px`.** Something moved; it's not a
  jitter measurement. Re-mount and rerun.
- **Every frame rejected for `fit`.** The fit is a homography on the
  distorted image; near the frame edge distortion alone approaches 3 px
  (2.2–2.9 px in the self-test for an off-centre tilted board). Centre the
  board, or raise `--max-fit` and say so in the result.
- **Frames rejected for settings.** Settings changed mid-session. Start a new
  session.

---

## 8. Manual intrinsics session

```sh
src/06_focus/build/focus <serial>
src/03_detect/build/detect <serial>
src/04_capture/build/capture [serial] [--gain N] [--intensity N] [--exposure US] [--board FILE]
.venv/bin/python python/fit_intrinsics.py <session> [<session> ...]
```

`capture` prints the board it loaded and refuses to start if the board file
is missing or a measured size is blank. It starts at the grayscale operating
point (§12) unless given start-up flags, all modes:

| Flag | Default | Range |
|---|---|---|
| `--exposure US` | 8133 | 1 to `floor(1e6/fps) − 200` (33133 at 30 fps), and within the camera's own range once started |
| `--gain N` | 7 | 0–7 |
| `--intensity N` | 15 | IR ring, 0–15 |

A value out of range exits with `need an integer lo..hi` before the camera
starts. At start-up it prints `settings  exposure E us (camera MIN..MAX)  gain
G  intensity I  frame rate 30  IR filter in   [from flags; read back]` (or
`[defaults; read back]`): the camera's readback, not the request. A
`WARNING   read back differs from the request` lists what differs; check it
before capturing. `session.json` records them under `baseline_settings`.
The `+ - [ ]` keys still change exposure and gain from there. It creates `$HOME/panoptes-data/sessions/intrinsics_<serial>_<YYYYmmdd_HHMMSS>/`
(local time) and prints the path. The overlay shows corners, captures, empty
cells, exposure against its ceiling, gain, the tilted share, the coverage grid
(red = empty) and a tilt histogram (amber = under 20°). Use
`--board config/charuco_board_old.yaml` to reproduce an old-board session.

| Key | Action |
|---|---|
| `space` | Capture. Only with every corner found; otherwise prints `skipped` |
| `u` | Undo the last capture, removing the PNG and its coverage |
| `+` / `=` | Exposure +1000, up to the camera's reported ceiling |
| `-` | Exposure −1000, down to 10 |
| `]` | Gain +1, up to 7 |
| `[` | Gain −1, down to 0 |
| `q` | Quit; prints the frame count, empty cells and tilted share |

1. **Network up**, then `discover`; the serial must read `Initialized` (§4).
2. **Focus.** `focus <serial>` with the board in the centre rectangle at
   working distance. Find the peak, lock the ring, `q`.
3. **Optional check.** `detect <serial>` at 1–1.5 m should read
   `markers 54/54   corners 88/88` with `fit` about 1 px.
4. **Capture** with the board at **1–1.5 m**; further away it fills too
   little of the frame and corners cluster mid-image.
   - Work the board into the **corners and edges** until `empty cells` is
     **0/30**; the line turns green.
   - Keep **tilted** at **30% or more**: tilt 20–40° about both axes, plus
     some roll and distance changes.
   - A useful pattern: nine positions (centre, four edges, four corners)
     fronto-parallel, the same nine tilted about the vertical axis, the same
     nine about the horizontal axis, then fill what's left.
   - `u` undoes a bad capture; `q` ends the session and prints its folder.
5. **Solve** with `fit_intrinsics.py`. It writes `<session>/intrinsics.json`
   and prints a table; give it several sessions of the same camera to see
   the spread, which is the real accuracy figure.
6. **Read the result.**
   - RMS is **not** the acceptance criterion; zero empty cells (step 4) is.
   - Look for a bad capture in `per_view_error` in `intrinsics.json`.
   - fx and fy more than ~1% apart means too little tilt.
   - Compare with the previous fits in FINDINGS §16.
7. **Use it.** For the camera's own file, rerun with all its accepted
   sessions and `--write-camera`: one joint fit, written to
   `config/intrinsics/<serial>.yaml` (`--force` to replace). The synthetic
   studies' shared model is still `camera_model` in `config/layout.yaml`,
   edited by hand. Record the fit in FINDINGS §16.

**Troubleshooting**

- **`skipped: only N/88 corners`.** The board is partly out of frame or too
  far. Move it closer than 1.5 m.
- **Board config errors at start.** See §5.
- **Python finds 0 corners on every frame.** See §10.

---

## 9. Auto intrinsics session

```sh
src/04_capture/build/capture [serial] [--gain N] [--intensity N] [--exposure US] --auto-dry [--auto-config FILE] [--board FILE] [--status-file PATH]
src/04_capture/build/capture [serial] [--gain N] [--intensity N] [--exposure US] --auto [--auto-config FILE] [--board FILE] [--status-file PATH]
.venv/bin/python python/fit_intrinsics.py <auto_session> <manual_session>
.venv/bin/python python/capture_status.py PATH
.venv/bin/python python/capture_status.py --self-test
```

> **Auto mode is UNVALIDATED.** Its intrinsics have not yet been compared
> with a manual session's on real data (step 6). Until that check has passed
> and is recorded in FINDINGS, use manual sessions for any intrinsics that
> matter and treat auto sessions as experiments.

The operator moves the board slowly; the tool keeps the frames it judges
useful. Rules and thresholds are in `config/autocapture.yaml`
(`--auto-config` overrides) and are copied into `session.json`. How the gates
work: [ARCHITECTURE §5](ARCHITECTURE.md#5-automatic-frame-selection).

**Rejection reasons**, in the order checked:

| # | Reason | Passes when |
|--:|---|---|
| 1 | `not_full`, `bad_fit` | all 88 corners found, fit under `max_fit_px` |
| 2 | `no_reference`, `moving` | since the previous frame (which itself passed 1), the median corner moved less than `still_px` **and** the largest move is under `still_max_px` |
| 3 | `redundant` | it reaches an empty coverage cell, **or** its pose bin has fewer than `min_per_bin` frames |
| 4 | `settings_changed` | exposure, gain, intensity, rate and filter match the first frame |
| 5 | `too_soon` | at least `min_interval_ms` since the last accept |
| 6 | `same_pose` | some corner moved at least `min_move_px` since the last accept |

**Pose bins:** 5 tilt directions × 3 distance bands = 15.
`flat` is within `flat_deg` of fronto-parallel; otherwise `+x+y`, `+x-y`,
`-x+y` or `-x-y`, where `+x` = board faces image-down (bottom edge further
away) and `+y` = faces image-right. Each quadrant needs a **compound** tilt; a
single-axis tilt sits on a boundary and lands by the small other angle.
Bands `near` / `mid` / `far` come from `size` (px per square) against
`near_square_px` and `far_square_px`.

**Live view**, on top of the manual overlay:

- every frame's verdict (`ACCEPT` and reason, or `reject:` and reason), and
  `ACCEPT #N` for 1.5 s after each accept
- a large **HOLD STILL** when gate 2 fails; **SETTINGS CHANGED** while gate 4
  holds
- `motion median` and `max` against `still_px` / `still_max_px`, and
  `size`, and the bin table bottom right (red under
  `min_per_bin`, green at or above, current bin outlined)
- a terminal bell and one printed line per accept

| Key | Action |
|---|---|
| `+` `=` `-` `]` `[` | As in manual mode (§8); any change pauses accepting until restored |
| `q` | Quit |
| `space`, `u` | Disabled |

1. **Do steps 1–3 of §8** (network, focus, optional check).
2. **Tune with `--auto-dry`.** Same loop, every frame's verdict printed with
   `motion median`, `max`, `size` and `bin`; nothing written, no session
   folder.
   1. Hold the board still: `motion median` should sit well under
      `still_px` (around 0.08 px if FINDINGS §6 holds hand-held) and `max`
      well under `still_max_px`. Move it at your working pace: it should
      read `moving median`. If a hand-held board never gets under 0.3 px,
      record that rather than quietly raising `still_px` past the blur it
      exists to stop. `still_max_px` is an outlier bound, not the stillness
      threshold; don't tighten it to make the gate stricter.
   2. Hold the board at 1.0, 1.25 and 1.5 m and read `size`. Set
      `near_square_px` and `far_square_px` so those land in `near`, `mid`,
      `far`. Tilting must not change the band.
   3. Save any change in `config/autocapture.yaml`; the session copies the
      values it ran with.
3. **Run `--auto`.** Work through the volume as in §8 step 4, slowly, pausing
   in each pose until the bell. Use the bin table to find what's missing. It
   ends by itself at **0 empty cells and every bin at `min_per_bin`**, or on
   `q`, and prints which, with the bin table and every verdict's count.
4. **Solve and read** as in §8 steps 5–6.
5. **What the session holds** (auto only): each frame has an `auto` object
   (accept reason `cells` / `bin`, `motion_median_px`, `motion_max_px`,
   fit, size, tilt);
   `autocapture` holds the config, bin counts, verdict counts and reference
   settings; `autocapture_end` says why it ended. Rejected frames are never
   written.
   **Status file** (`--status-file PATH`, auto modes only): after every
   frame `PATH` is replaced with a JSON of the live state (corners, board
   centroid and size, motion, verdict, empty cells, bins, accepts). Another
   process reads it; nothing here depends on it. Schema:
   [ARCHITECTURE §5](ARCHITECTURE.md#5-automatic-frame-selection). Use a
   path outside the repo, e.g. `"$HOME"/panoptes-data/capture_status.json`.
   `capture_status.py PATH` prints the latest one; `--self-test` checks the
   reader against `tests/fixtures/capture_status.json`. Without `--auto` or
   `--auto-dry` the flag is refused (`--status-file needs --auto or
   --auto-dry`). If a write fails, the error prints once and the file goes
   stale; readers treat a file older than 2 s as dead. During a stall the
   file keeps being rewritten with `stalled` set (below).
   **Stalls** (auto modes): no frame for 1 s prints
   `no frames from SERIAL for N s  (state ...  running ...  data rate ...)`,
   again every 5 s. From 5 s, and every 5 s until frames return, it prints
   `restarting camera SERIAL (attempt N, ...)`, stops and restarts the
   camera with the start-up settings (any `+ - [ ]` change is dropped), and
   on the first frames back checks the readback: `restart verified:
   readback matches the baseline settings`, or a `WARNING` with the
   differences (gate 4 then refuses every frame). `frames from SERIAL again
   after N s` marks the end. The session carries on in the same folder;
   each stall is in `session.json` `stalls`. `q` still quits during a
   stall. `main loop blocked for N s in STEP` (stderr) is different: a call
   that hasn't returned, which the restart can't fix. Quit, wait a couple of
   seconds, restart capture. Copy any of these lines into FINDINGS.
6. **Validation rule: do this before trusting auto mode.**
   1. On one camera, the same day, lens untouched: one `--auto` session and
      one manual session.
   2. Fit both together (third line of the block).
   3. **Pass** if **fx and fy agree within ~1 px and cx and cy within
      ~3 px**. Source: the same-day 2026-09-25 manual pair in FINDINGS §16
      (fx 0.23, fy 0.63, cx 2.51, cy 1.51 px). The three-session range there
      is wider (fx 2.48, cx 9.40 px), and FINDINGS §16 puts that down to the
      hand-held method, so a cx/cy failure alone may not be auto mode's
      fault: record a second manual session the same day and check the two
      manual ones agree first.
   4. Record the result, pass or fail, in FINDINGS with both session names.
      Only on a pass, remove the UNVALIDATED notes here and in §1.8, and
      close Open items #11.

**Troubleshooting**

- **Never accepts.** Read the reason on screen or in the `--auto-dry`
  output:
  - `moving median` every frame: never still enough (gate 2).
  - `moving max` with a low median: one corner jumps. Look for a corner
    detected badly (glare, the board edge) or the board flexing.
  - `redundant`: what's in view is filled; check the bin table and the red
    coverage cells.
  - `settings_changed`: undo the `+ - [ ]` presses.
  - `same_pose`: move on to a new pose.
- **`autocapture config ...: <field> is missing`.** `config/autocapture.yaml`
  must set every field; there are no defaults.
- **A bin never fills.** Some poses may be unreachable (e.g. far and steeply
  tilted). End with `q`; the printout gives every bin's count and how many
  are short.
- **`no frames from SERIAL` keeps repeating, restarts don't help**
  (`not restarted: serial not in CameraList`, or `state Disconnected`).
  A dropped camera can take ~35 s to reappear (FINDINGS §11); if it hasn't
  after a minute, check the camera's cable and PoE port (on the arm, the
  cable at the wrist, §13 per-camera step 2), then `q` and start capture
  again. The frames already
  accepted stay in the session folder.

---

## 10. Python fits and studies

```sh
.venv/bin/python python/fit_intrinsics.py <session> [<session> ...]
.venv/bin/python python/fit_intrinsics.py --write-camera [--force] [--status TEXT] <session> [<session> ...]
.venv/bin/python python/compare_models.py <session>
.venv/bin/python python/bootstrap_intrinsics.py <session> [n_subsets] [subset_size]
.venv/bin/python python/board.py [board.yaml]
.venv/bin/python python/triangulate.py [layout.yaml]
.venv/bin/python python/accuracy_study.py [--allow-unmeasured] [layout.yaml] [study.yaml]
.venv/bin/python python/volume_coverage.py [--self-test] [--allow-unmeasured] [layout.yaml] [coverage.yaml]
.venv/bin/python python/test_arm_capture.py
```

| Command | Does |
|---|---|
| `fit_intrinsics.py` | Brown-Conrady on all frames; writes `<session>/intrinsics.json`, prints a table, and with several sessions the spread across them. `--write-camera`: also one joint fit over all the sessions' frames, written to `config/intrinsics/<serial>.yaml` with provenance and the cross-session range as `uncertainty_px`; refused if the sessions differ in camera, image size, board or settings, or if the file exists (`--force` replaces it). `--status TEXT` (only with `--write-camera`) is written verbatim as the file's `status`, e.g. `done` or `done, marginal (fx/fy 1.4 px)` (flag rule in §13); omitted, no status line. Prints `joint fit, all sessions: …` and `wrote config/intrinsics/<serial>.yaml` |
| `compare_models.py` | Brown-Conrady vs rational vs fisheye: held-out error and edge straightness printed; writes `<session>/model_comparison.json` (training RMS only) |
| `bootstrap_intrinsics.py` | Refits random subsets (defaults 20 subsets of 50) to see whether pose sampling explains a cross-session difference |
| `board.py` | Prints the board a yaml describes (default `config/charuco_board.yaml`), or the error |
| `triangulate.py` | Triangulation self-test, 12 checks; exit 0 only if all pass. Prints `note: layout is unmeasured ...` when the layout has `measured: 0`; the checks still apply |
| `accuracy_study.py` | Synthetic study (defaults `config/layout.yaml`, `config/study.yaml`); writes `analysis/` PNGs, `accuracy_study.json` and `.npz`; refuses to run if the self-test fails, or if the layout has `measured: 0`. `--allow-unmeasured` runs on an unmeasured layout anyway, warns, and records `layout_measured: false` in the JSON |
| `volume_coverage.py` | Camera count and triangulation angle per voxel over the volume, geometry only (defaults `config/layout.yaml`, `config/coverage.yaml`). Prints the share of the volume seen by 0/1/2/3+ cameras, `seen` (≥ `min_cameras_seen`), `good` (seen and a camera pair ≥ `min_angle_deg`), angle percentiles, a by-height table and the perch; writes `analysis/coverage/volume_coverage.json`, `.npz` grids, one PNG per slice and `slices_overview.png`. `--self-test`: 9 checks only, exit 0 only if all pass; a full run runs them first and refuses to report if any fails. Refuses a layout with `measured: 0` unless `--allow-unmeasured`, which warns, marks the PNG titles `STALE LAYOUT` and records `layout_measured: false` |
| `test_arm_capture.py` | Arm session tests, no arm or camera: a simulated camera on the arm and a stand-in capture; replays the 2026-10-28 abort from `tests/fixtures/session_arm_33664_20261028_185017.json` and reads the corner run in `..._191025.json`; aiming and orbit on a second, rig-like geometry; joint moves, saved poses, the wrist check; teach and taught sessions (pose-space grid, IK, families and the hub); joint limits from `config/arm.yaml`, asymmetric and null; continuous joints the short way; the abort report; joint moves with and without `--speed-constraint`, and the measured joint rate; grid corners from taught joints and the IK summary; replays the 2026-11-02 taught sessions `..._20261102_150512.json` and `..._175213.json` through the mid-near-far visit order and the reach-edge rule; abandoning a viewpoint when the board isn't back (17:52's ending, simulated) and at the reach edge; a capture stall that recovers and one that doesn't; cable winding (`cable_window_deg`, the long way round, long moves as waypoints, unwinding before a pan, net zero turns at the end, against a stand-in arm that always turns the short way); 186 checks, exit 0 only if all pass |

1. Run from the repo root with `.venv/bin/python` only. Each script prints the
   board it resolved: from the session's `session.json`, or
   `config/charuco_board_old.yaml` for sessions recorded before the board
   config existed.
2. After changing anything in `python/` or `config/layout.yaml`, run
   `triangulate.py`; all 12 checks must pass. After changing `arm.py`,
   `arm_capture.py`, `capture_status.py` or the `config/arm*.yaml` files,
   run `test_arm_capture.py`; all 186 must pass.
3. `analysis/*.png` and `.npz` are regenerated and gitignored;
   `analysis/accuracy_study.json` is tracked. Record new numbers in FINDINGS.
   `analysis/coverage/*.png` and `.npz` are gitignored too.
4. **Volume coverage settings** are in `config/coverage.yaml` (inches):
   - `grid.spacing_in`: voxel edge, an upper bound (default 2). Each axis
     is split evenly, so the printed edge can be slightly smaller.
   - `grid.chunk_points`: voxels per batch; memory only, results are
     identical at any value.
   - `min_cameras_seen`: cameras needed for a voxel to count as seen (3).
   - `min_angle_deg`: effective angle, 0–90, a camera pair needs for
     `good` (15).
   - `slices.z_in`: horizontal heat-map heights; `slices.perch_plane`: the
     axis normal to the vertical slice through the perch (`"y"`).
   - `output.dir`: relative to the config file's directory.

   When the rig is re-measured, edit `config/layout.yaml`, set
   `measured: 1` and rerun without `--allow-unmeasured`; re-choose
   `slices.z_in` for the new geometry.
5. `config/layout.yaml` carries `measured: 0` or `1`. Leave it at 0 until
   every position, the volume, the aim and the perch in the file have been
   re-measured or taken from extrinsics; then set 1 and re-run the study.
   While it is 0:
   - don't quote any position, the volume, the aim or the perch from the
     file as measured; only the name/serial mapping is current
   - don't run `accuracy_study.py --allow-unmeasured` against the default
     `study.yaml` unless you mean to overwrite `analysis/accuracy_study.json`
     with numbers for stale geometry. To try it out, point a copy of
     `study.yaml` at another `output.dir`.

**Troubleshooting**

- **0 corners on every frame.** The board's `legacy_pattern` doesn't match
  the physical board (FINDINGS §9). Check the board each script prints.
- **`accuracy_study.py` exits with "refusing to run: ... layout is
  UNMEASURED".** The layout has `measured: 0` (item 5 above). That is the
  intended state until the rig is re-measured. `volume_coverage.py` prints
  the same message for the same reason.
- **`accuracy_study.py` exits with "self-test failed".** Run `triangulate.py`
  alone and read which check failed. The study refuses to report numbers from
  a broken triangulator.

---

## 11. Git

```sh
git status
git diff --stat
git diff
git add <files>
git commit
git log --oneline -10
```

1. Claude Code leaves its changes in the working tree; review with `git diff`
   and commit yourself.
2. Commit the doc updates in the same commit as the change they describe
   (CLAUDE.md, "Keeping the docs current").
3. Don't stage generated files: `build/`, `.venv/`, `analysis/*.png`,
   `*.npz`. `logs/coverage_*.csv` are currently tracked.

---

## 12. Operating points (reference)

**Grayscale, for calibration:** the `Settings` defaults in `lib/camera.h`,
used by `capture` and `focus` (`stream` sets the same exposure, intensity
and rate directly). Expect 54/54 markers and 88/88 corners at 1–1.5 m
(FINDINGS §19) and a live view of 6–8 fps; that rate is normal (FINDINGS §5).

| Setting | Value |
|---|---|
| IR filter | in |
| IR intensity | 15 |
| Gain | 7 |
| Exposure | 8133 µs |
| Frame rate | 30 Hz |

**Object mode, for tracking: not settled** (FINDINGS §13). A mask is valid
only at the settings it was learned under.

| Used by | Exposure | Threshold | Gain | Intensity | Rate |
|---|---:|---:|---:|---:|---:|
| `identify`, `objdiag` defaults | 500 | 200 | 0 | 15 | 30 Hz |
| `mask` (`config/mask.yaml`); saved masks learned here | 10 | 40 | 0 | 15 | 30 Hz |

**Exposure ceiling:** `floor(1e6 / fps) − 200`. Read it back after any rate
change rather than assuming it.

| Rate | Ceiling |
|---:|---:|
| 30 Hz | 33133 |
| 60 Hz | 16466 |
| 120 Hz | 8133 |
| 240 Hz | 3966 |

---

## 13. Arm (Kinova Gen3)

```sh
.venv/bin/pip install --no-deps -r python/requirements-arm.txt
ip -brief addr show enp6s0
.venv/bin/python python/arm.py status [--ip IP] [--port PORT] [--username U] [--password P]
.venv/bin/python python/arm.py move [--dx M] [--dy M] [--dz M] [--dtx DEG] [--dty DEG] [--dtz DEG] [--yes]
.venv/bin/python python/arm.py goto X Y Z TX TY TZ [--yes]
.venv/bin/python python/arm.py joints J1 J2 J3 J4 J5 J6 [--speed-constraint DEG_S] [--yes]
.venv/bin/python python/arm.py save-pose NAME [--force]
.venv/bin/python python/arm.py list-poses
.venv/bin/python python/arm.py goto-pose NAME [--speed-constraint DEG_S] [--yes]
.venv/bin/python python/arm.py stop
.venv/bin/python python/arm_capture.py probe --status-file PATH [--yes] [--ip IP] [--port PORT] [--username U] [--password P]
.venv/bin/python python/arm_capture.py teach --status-file PATH [--force] [--ip IP] [--port PORT] [--username U] [--password P]
.venv/bin/python python/arm_capture.py session --status-file PATH [--yes] [--poses | --orbit] [--from-pose NAME] [--speed-constraint DEG_S] [--ip IP] [--port PORT] [--username U] [--password P]
```

> **Safety: the E-stop pendant must be within reach whenever anything in
> this repo talks to the arm,** including `status`. Before any `--yes`:
> **nothing and nobody inside the box**, and a hand near the E-stop.

| Command | Does |
|---|---|
| `status` | Prints firmware, serial, build, devices, pose, joints, each joint's limit and margin, servoing mode. Never moves. |
| `move` | Relative Cartesian move from the **measured** pose. Each of `--dx --dy --dz` (m) and `--dtx --dty --dtz` (deg) defaults to 0. |
| `goto` | Absolute Cartesian move: `X Y Z` in m, `TX TY TZ` in deg, base frame. |
| `stop` | Sends `Base.Stop`. Not the E-stop: see below. |

| Flag | Commands | Default |
|---|---|---|
| `--yes` | `move`, `goto` | off: print the plan, send nothing, exit 0 |
| `--ip` | all | `192.168.1.10` |
| `--port` | all | `10000` (Kortex TCP) |
| `--username` | all | `admin` |
| `--password` | all | `admin` |

**Limits** are in `config/arm.yaml`, all required:

| Field | Default | Meaning |
|---|---|---|
| `x_min` `x_max` | 0.25, 0.95 m | workspace box, base frame |
| `y_min` `y_max` | −0.45, 0.45 m | |
| `z_min` `z_max` | 0.15, 0.85 m | |
| `max_translation_mps` | 0.05 | speed constraint sent with every Cartesian move; `arm.py` refuses above 0.20 |
| `max_rotation_dps` | 10.0 | refused above 30. Not used by joint moves |
| `move_timeout_s` | 30.0 | wait for the end of a move; refused above 300 |
| `cable_window_deg` | 170.0 | each continuous joint stays within its hub angle ± this, counted in turns; a joint move goes the long way round rather than leave it. Required, (0, 360] |
| `joint_1_limit_deg` … `joint_6_limit_deg` | 1, 4, 6 `"continuous"`; 2 `[-129, 138]`; 3 `[-148, 152]`; 5 `[-120, 128]` | position limit of each actuator, deg, on the angle wrapped to ±180: `[low, high]` with −180 ≤ low < high ≤ 180, `"continuous"`, or null (unconfirmed). All six are required |

**Joint limits** are copied from the Web App: Safeties → Actuators, "Joint
Limit Low/High". A 0/0 pair there means disabled; write `"continuous"`.
Kortex can't read them (FINDINGS §20), so if the Web App's values change,
copy them again. A limit refuses:

- a `joints` / `goto-pose` target outside it;
- a taught pose within 10° of either end;
- a session IK sample within 5° of either end.

A null limit only prints a `WARNING` with the angle.

There is no flag to disable the box or to use another config file. To change
a limit, edit `config/arm.yaml`.

**What `move` and `goto` do,** in order. Every check runs before anything
is sent:

1. Load `config/arm.yaml`; refuse a missing or out-of-range field.
2. Connect, read the measured pose, compute the target. Angles are wrapped
   to (−180, 180]. `--dtx` etc. add to the Euler angles theta_x/y/z; they
   are not rotations about a fixed axis.
3. Print `current`, `target`, `change` (distance, largest angle, estimated
   time at the configured speeds) and `box`.
4. **Refuse, sending nothing,** if the current pose or the target is outside
   the box, if the estimated time exceeds `move_timeout_s`, or if the
   servoing mode is not `SINGLE_LEVEL_SERVOING` (`arm.py` never changes the
   mode). Exit 1, `REFUSED, nothing sent` and each reason.
5. Without `--yes`: print `dry run: nothing sent. Add --yes to move.` and
   exit 0.
6. With `--yes`: send one `reach_pose` with the speed constraint, and block
   until `ACTION_END`, `ACTION_ABORT` or `move_timeout_s`.
7. Print `result`, the `measured` pose and the `error` to target (mm, deg).
   `ABORT` prints the arm's reason and exits 1. A timeout sends
   `Base.Stop` and exits 1. Ctrl-C during the wait sends `Base.Stop`.

The box bounds the **tool point only**. The elbow and wrist can leave it,
so "nothing inside the box" means clear of the whole arm's sweep, not only
the box.

**`stop` versus the E-stop.**

| | `arm.py stop` (`Base.Stop`) | E-stop pendant |
|---|---|---|
| What it is | A software request over the network | A push button in series with the power cable |
| Effect | Ends the current action; the arm stays powered and holds its pose | Cuts power to the robot |
| The arm afterwards | Holding, ready for the next command | **Descends**: no mechanical brakes; the actuators only slow the fall (Gen3 user guide, "Robot power adapter and E-stop"). Cradle it if you can. |
| Depends on | The network, the arm's controller, a working `arm.py` | Nothing |
| Undo | Nothing to undo | Twist the button clockwise until it pops up |

`stop` is a convenience, not a safety function. If anything looks wrong,
or anyone is at risk, use the E-stop.

**Procedure**

1. **Network: already done.** `enp6s0` carries both the cameras and the arm:
   a second static address, `192.168.1.11/24`, alongside the link-local
   address the cameras use, which stays. Check with the second line: both
   addresses listed.
2. **Install the Kortex API, once per venv** (and again after the venv is
   rebuilt, §2). `--no-deps` is required: the wheel pins `protobuf==3.5.1`,
   which cannot import on Python 3.12, and the file pins 3.20.3 instead
   (FINDINGS §20). Afterwards `pip check` reports
   `kortex-api 2.6.0.post3 has requirement protobuf==3.5.1`; that is
   expected.
3. **E-stop pendant in reach.**
4. **Run `status`.** Prints, in order:
   - `firmware`: the main bundle version, then each component's version
   - `serial`: the base's serial number
   - `build`: degrees of freedom and end effector as the arm reports them
     (expect 6 DOF, `END_EFFECTOR_TYPE_ROBOTIQ_2F_85`)
   - `devices`: every device the base lists, by type and id
   - `pose`: measured Cartesian pose, x y z in m, theta_x/y/z in deg
   - `joints`: measured joint angles, deg, by joint id
   - `limits`: one line per actuator (1–6): the angle wrapped to ±180, then
     `limit [low, high], margin M deg` (M to the nearer end, negative
     outside), `continuous`, or `limit unconfirmed`
   - `servoing`: the current servoing mode, e.g. `SINGLE_LEVEL_SERVOING`
   - `session closed` on exit

   **After power-up the arm rests on its joint stops** (it has no brakes),
   at or past the limits of actuators 2, 3 and 5. If any `limits` margin is
   negative or small, jog that joint toward mid-range with the Web App
   first: `arm.py` refuses a target outside a limit, and a session's IK
   check refuses paths near one.

   Check that `pose` is inside the box in `config/arm.yaml`. If it isn't,
   bring the arm into the box with Kinova's own tools first; `arm.py`
   refuses to move from outside the box.
5. **First motion, in this order:**
   1. E-stop pendant in reach, hand near it.
   2. Nothing and nobody inside the box or the arm's sweep: cameras on
      stands, the board, cables.
   3. Dry run: `.venv/bin/python python/arm.py move --dz 0.02`. Read the
      plan: `target` z is 20 mm above `current`, and `checks` passes.
   4. **`.venv/bin/python python/arm.py move --dz 0.02 --yes`.** The arm
      rises 20 mm at 0.05 m/s. Expect `result     END` and an `error`
      line of a few mm or less.
   5. Record the output in FINDINGS (Open items).
6. **Later moves:** always run without `--yes` first and read the plan.

**Joint moves and saved poses.** `move` and `goto` go through the arm's
Cartesian planner, which refuses a path from or across a singularity
(`ACTION_ABORT INVALID_PARAM`). A joint move doesn't.

- `arm.py joints J1 J2 J3 J4 J5 J6`: deg, actuator order, 0–360 or ±180.
  A continuous joint (`config/arm.yaml`) is sent as its measured angle plus
  the change the cable window allows, so the plan's `target` can show it
  outside 0–360 (e.g. `370` for 10 from 350); the others are sent in 0–360.
  The `cable` line gives each continuous joint's value, `short way` or
  `long way round: unwinding`, and its turns from the saved `hub` pose's
  angle before and after. `arm.py` assumes the cable is unwound when it
  starts (each joint within half a turn of the hub). A move that turns a
  continuous joint over 180° is sent as waypoints (`waypoints  N moves`),
  each checked against the box. Same `--yes` rule, servoing check, wait and timeout as `goto`. The box
  applies to the arm's forward kinematics of the angles. The plan prints
  that pose as `target FK`; `dry run` sends nothing. No speed constraint
  is sent: the arm picks the speed, the `change` line says `no speed
  constraint` and has no time estimate, and after the move `duration`
  gives the time and average deg/s. A target outside a
  joint limit is refused, naming the joint; with a null limit the plan
  starts with `WARNING    target joint limit unconfirmed` and the angles.
  The plan ends with `sent as    id 0: … id 5: …`, the `joint_identifier`
  and value of each angle exactly as they go into the action.
- `--speed-constraint DEG_S` (`joints`, `goto-pose`): sends
  `JOINT_CONSTRAINT_SPEED` at `DEG_S` (0 < `DEG_S` ≤ 30). The plan then
  estimates the time at that speed and refuses past `move_timeout_s`. The
  arm aborts a short move with it, `METHOD_FAILED`, when that speed implies
  too high an acceleration; leave it off unless you need it.
- `arm.py save-pose NAME`: stores the measured joints and pose in
  `config/arm_poses.yaml`. `NAME` is letters, digits and `_`, starting with a
  letter. An existing name is refused unless `--force`, which prints what it
  replaced. It doesn't move the arm.
- `arm.py list-poses`: prints the file. It doesn't talk to the arm.
- `arm.py goto-pose NAME`: a joint move to the saved joints, as `joints`.

> **The box checks only the end points of a joint move.** Between them the
> tool swings through an arc, which can leave the box, and the move can be
> large. Clear the whole sweep, and stay on the E-stop. The arm picks the
> speed unless `--speed-constraint` is given.

**Never start an `--orbit` session from Home.** The planner can refuse a
Cartesian path from there. (A taught session's first move is a joint move, so
it can start anywhere.) Once per camera setup, after step 4 of the
probe below, run `arm.py save-pose NAME` with the board centred, and start
the probe and every `--orbit` session from that pose: `arm.py goto-pose NAME --yes`, or
`session --from-pose NAME`.

**Probe: which tool motion moves the board where in the image.**
`arm_capture.py probe` writes `config/arm_camera.yaml`. It is valid only
for this camera, this camera position, this grip of the board and the pose
it started from; rerun it after any of these change.

| Flag | Default |
|---|---|
| `--status-file` | required: the `PATH` given to `capture --status-file` |
| `--yes` | off: check every target, print the plan, send nothing |
| `--ip` `--port` `--username` `--password` | as `arm.py` |

It makes six probes from the start pose P0, each followed by a return to
P0: rotate +2° about the tool x, y and z axes (about the TCP), and move
+30 mm along the tool x, y and z axes. These are true tool-frame motions,
not `arm.py move --dtx/--dty/--dtz`, which add to Euler angles. Every move
goes through the same checks as `move` (box, time, servoing mode). All
twelve are checked before the first is sent. **It needs about a minute.**

1. **Do steps 1–5 above first** (`status` works, the first motion worked).
2. **Camera on the arm, board on an easel**, placed as in the session's
   "Easel and P0" below if a session will follow: the probe is only valid
   where it was run. Check the camera mount is rigid: the probe assumes the camera moves only with the tool. Leave the
   camera cable enough slack for ±35° of wrist rotation and route it so it
   can't snag. The camera body is outside the box's tool point: clear its
   sweep too.
3. **Start the camera** in another terminal, dry run only:
   `src/04_capture/build/capture <serial> --auto-dry --status-file "$HOME"/panoptes-data/capture_status.json`
4. **Centre the board and make it fully visible.** Using Kinova's tools or
   `arm.py move`, bring the board to the image centre with margin on every
   side: `corners 88/88`, no HOLD STILL. The probe refuses to start
   unless the status shows 88/88, still, `--auto-dry`, a file under 2 s
   old, and the board's bounding box at least 40 px inside the image.
   `capture_status.py PATH` shows what it will see.
   Then `arm.py save-pose NAME`, once for this camera setup.
5. **E-stop in reach; nothing and nobody in the sweep.**
6. **Dry run:** `.venv/bin/python python/arm_capture.py probe --status-file "$HOME"/panoptes-data/capture_status.json`.
   Read the six targets; it ends `dry run: nothing sent`.
7. **Run with `--yes`.** Each probe line shows the board's change (`dx`,
   `dy` px, `roll` deg, `square` px) and `return`, how far the centroid
   came back after returning to P0. Then a per-probe summary
   (`tool theta_y +1 deg moves the board ... px in image x, ...`) and the
   mapping: for right, down, clockwise and closer, the tool axis, its sign
   and px/deg or px/m. The opposite direction is the opposite sign.
   `CAUTION: coupled` marks an axis whose runner-up is over half as strong.
8. **If it stops** (abort, timeout, board lost or not still within 15 s,
   or a return more than 3 px off), it writes nothing and doesn't move the
   arm again. It prints the `arm.py goto` command back to P0: run it
   without `--yes` first.

**Teach: eight viewpoints, by hand.** The session's viewpoints are a grid over
the camera poses of eight poses you jog to; the arm's IK finds the joints for
each. **Teach once per easel position, not per camera, and again if the
easel or the arm base moves.** The probe is still needed once per camera
mounting, for the pan gains.

1. **Probe** (above), before or after teaching: the session needs it, teach
   doesn't.
2. **Start capture,** either `--auto-dry` or `--auto`, with `--status-file`.
3. **Run** `.venv/bin/python python/arm_capture.py teach --status-file "$HOME"/panoptes-data/capture_status.json`.
   Existing poses are refused unless `--force`. `q` at a prompt stops; poses
   saved so far stay.
4. **For each prompt:** jog with the Web App, close the Web App, press Enter.
   The tool sends `Base.Stop`, then checks, and saves if all pass:
   - the arm is free: it connects, `Base.Stop` succeeds, and the servoing mode
     is `SINGLE_LEVEL_SERVOING`;
   - the board is 88/88 and still in the status file;
   - every joint with a limit in `config/arm.yaml` is at least 10° inside
     both ends of it.

   A failed check prints `REFUSED` and the same pose is asked again. A pose
   outside the box or with the wrist singular saves with a `WARNING`:
   viewpoints near it are skipped.

The whole board must be in view at every pose. "Near" and "far" are distance
from the board; "left", "right", "high" and "low" are where the camera sits
as you face the board.

| Pose, in order | Camera position |
|---|---|
| `near_left_high` | Close to the board, to its left, above it, looking down and right at it |
| `near_right_high` | Close, to its right, above it, looking down and left |
| `near_left_low` | Close, to its left, below it, looking up and right |
| `near_right_low` | Close, to its right, below it, looking up and left |
| `far_left_high` | Far from the board, to its left, above it, looking down and right |
| `far_right_high` | Far, to its right, above it, looking down and left |
| `far_left_low` | Far, to its left, below it, looking up and right |
| `far_right_low` | Far, to its right, below it, looking up and left |

Go well off to each side: tilted views come from these corners, and aim past
20° from square-on. Keeping a similar arm configuration at all eight means
fewer families and fewer hub moves, but it isn't required.

After the eighth it prints the far/near square-size ratio (want < 0.6, a
`WARNING` otherwise) and `families`: how many arm configurations the eight
span (more than 90° apart on joint 2, 3 or 5; the continuous joints 1, 4
and 6 never split one), and which pose is in which. With more than one, and no `hub` saved yet, it prints a `NOTE`.

**The hub.** A taught session passes through `config/arm_poses.yaml`'s `hub`
pose whenever a viewpoint move changes family or joint 2, 3 or 5 by over
90°. Starting the session at the hub saves the start and return hub moves.
Jog to a pose with the board centred and every limited joint well inside its
limits, then `.venv/bin/python python/arm.py save-pose hub` (`--force` to
replace it). The session refuses to start if it plans a hub move and `hub`
is missing, outside the box, or within 10° of a joint limit.

**Session: an intrinsics session driven by the arm.** `arm_capture.py
session` moves the camera through views while `capture --auto` decides which
frames to keep, and reads its status file after every move. It uses the
probe's `config/arm_camera.yaml`, `config/arm_session.yaml` and
`config/autocapture.yaml`, and `config/arm_poses.yaml`'s taught poses and
`hub`. Cell pans go through the same checks as `move` with `config/arm.yaml`'s
box and speeds; viewpoint moves are joint moves to the arm's IK solution,
through the same checks as `joints`, at the viewpoint speeds.

Planning a taught session:

- **Viewpoints:** a 3 × 3 × 3 grid (`n_lr` × `n_hl` × `n_nf`) blended
  from the eight taught Cartesian poses: position linear, rotation along the
  shortest arc.
- **IK:** each is solved by the arm (`ComputeInverseKinematics`), seeded
  with the joints of the nearest taught pose, then the previous viewpoint's
  solution.
- **Skipped:** outside the box (`SKIP: x … outside`); no solution, or one
  within 10° of a joint limit (`SKIP: unreachable: …`); wrist singular.
- **Order:** mid-band viewpoints first, then near, then far (the predicted
  capture band); within a band, family by family, serpentine within each. A
  family that spans two bands needs a hub move for each.
- **`families` line:** the families' sizes and the hub transitions planned
  (between viewpoints, from the start, back to it; none for start and return
  if the arm starts at the hub). A hub transition is decided again before
  each move, from the measured joints. A null joint limit's warning on a viewpoint move goes
into that entry's `warnings` in `session_arm.json`, not the log. `--yes` is
asked once, for the whole session.

| Flag | Default |
|---|---|
| `--status-file` | required: the `PATH` given to `capture --auto --status-file` |
| `--yes` | off: check everything, print the plan, send nothing, write nothing |
| `--poses` | the default: viewpoints from the taught poses |
| `--orbit` | off: the older orbit planner, from the probe's P0; everything marked "orbit" below, "Easel and P0" included |
| `--speed-constraint DEG_S` | off: joint moves (viewpoint, hub, return, `--from-pose`) carry no speed constraint and the arm picks the speed. With it, every joint move carries `JOINT_CONSTRAINT_SPEED` at `DEG_S` (0 < `DEG_S` ≤ 30), and the record's `config.joint_speed_constraint_dps` says so |
| `--from-pose NAME` | `--orbit` only. Off: first a joint move to this pose from `config/arm_poses.yaml`. Refused before moving if the pose is wrist singular or outside `p0_tolerance_*` of the probe's start pose. Capture must already be running in `--auto`. Without `--yes`, it prints that move and stops, unless the arm is already there (every joint within 0.5°), as the session plan needs the arm in place |
| `--ip` `--port` `--username` `--password` | as `arm.py` |

`config/arm.yaml` speeds: `max_translation_mps` 0.05 and `max_rotation_dps`
10 apply to cell pans (Cartesian). Viewpoint, hub and return moves are joint
moves at the arm's own speed (above). Each joint move's time check uses the
session's measured rate so far (`measured_joint_dps` per move in
`session_arm.json`); the first has none.

`config/arm_session.yaml`, all required:

| Field | Default | Meaning |
|---|---|---|
| `n_lr`, `n_hl`, `n_nf` | 3, 3, 3 | taught grid, left-right × high-low × near-far, corners included; each ≥ 2 |
| `orbit_angles_deg` | 0, 12, −12, 24, −24 | elevation and azimuth offsets of the cap from P0's line of sight to the board |
| `orbit_radii_m` | 0, 0.20, −0.20 | added to P0's distance from the board |
| `orbit_rolls_deg` | 0, 15, −15 | roll about the optical axis; the non-zero values are used in turn |
| `orbit_roll_every` | 3 | every this many off-centre viewpoints takes the next roll |
| `lateral_variants` | 0 | 1: the old lateral variants instead of the orbit |
| `lateral_m` | 0.12 | lateral variants only: offset, each way along the two "lateral" tool axes |
| `depth_m` | 0.15 | lateral variants only: this far along tool z from P0, each way |
| `two_move_px` | 300 | a cell target further than this from the current centroid takes two moves |
| `first_move_fraction` | 0.6 | share of the rotation in the first of the two moves |
| `mid_band_fraction` | 0.08 | planner's mid band: within ±this of P0's square size; near above, far below |
| `band_spread_min` | 0.15 | lateral variants only: warn if the near and far depth variants differ in square size by no more than this fraction of P0's |
| `still_timeout_s` | 10 | wait this long for a full, still board after a move, then skip the target |
| `max_rotation_deg` | 35 | cap on a cell target's aiming rotation from its viewpoint |
| `centre_tol_px` | 40 | last move missed by more: one correcting move |
| `edge_margin_px` | 40 | keep the aimed board this far inside the image |
| `p0_tolerance_m`, `p0_tolerance_deg` | 0.05, 10 | refuse if the arm is further than this from the probe's start pose |

**A taught session,** from wherever the arm is (that pose is its P0):

- **Plan.** Every grid viewpoint is checked by the arm's forward kinematics
  against the box, and against the wrist singularity. The dry run lists each
  with its predicted square size and capture band; `SKIP` marks a failure. A
  `WARNING` means the viewpoints don't span capture's three bands.
- **Order.** A serpentine, each viewpoint one grid step from the last, near
  first.
- **Each viewpoint.** Joint move, wait for a still board, then the cell pans
  for empty cells, below. A viewpoint is visited if a cell is empty or a bin
  of its band is short. If the board isn't full and still there, it moves on.
- **End.** Capture's end condition, or viewpoints exhausted; then a joint move
  back to the start.

**Easel and P0** (`--orbit`). Put the board **about 1.0 m from the camera, leaned back
10–12°**, with **P0 near x 0.43 m**. Why (FINDINGS §20):

- **Distance.** The box limits how far the arm can travel around the board,
  and the same travel is a larger angle closer in. Elevations and azimuths
  of ±24° are out of the box at any useful distance with the default
  `config/arm.yaml`; ~1.0 m is where ±12° is usable and the board still fits
  at the near radius. 0.8 m gives more of the cap, but the board then fills
  most of the frame and has little room to reach the corners.
- **P0's x.** `r-0.20` moves the tool 0.20 m toward the board, so P0 at
  x > 0.50 m puts that whole radius past `x_max` (0.70 m).
- **Tilt.** With ±12° what's reachable, a board square to P0 stays under
  `flat_deg` (20°) at every viewpoint and only pointing at frame corners
  tilts it. Leaned 10–12°, the viewpoints on one side cross 20° while most
  stay flat, so the flat bin still fills. At 22° half the flat views are
  lost.
- **Bands.** At 1.0 m the three radii give about 58, 46 and 38 px/sq, which
  is near, near, mid against the default thresholds. Use the `bands` line's
  values for `near_square_px` and `far_square_px`, or capture's far bins
  can't fill.

What it does with `--orbit`, from P0 (the pose it starts at). The cell pans
apply to both modes:

- **Board estimate.** Depth from the apparent square size and the board's
  measured `square_mm`, direction from the centroid, written into
  `session_arm.json`. The `board` line prints it once, with a second
  distance from the probe's size gains.
- **Viewpoints:** a spherical cap around that point: `orbit_radii_m` ×
  `orbit_angles_deg` of elevation × the same of azimuth, 75 by default. Each
  pose points the camera's optical axis (found by the probe, not tool z) at
  the board. Some are rolled. The walk goes nearest first. A viewpoint is
  named by its predicted square size relative to P0's and its place on the
  cap: `mid(r0)/centre`, `near(r-0.20)/el+12az-12ro+15`. `mid` is within
  ±`mid_band_fraction`. The plan also shows the band **capture** will put it
  in (`capture-far` etc.), from `config/autocapture.yaml`'s fixed thresholds.
  Capture's bins and end condition use those, not the planner's names.
- **Clipping.** A viewpoint outside the box or refused by the IK check is
  skipped. The `cap` line prints the reachable fraction, per radius, before
  anything moves.
- **Lateral variants** (`lateral_variants: 1`): instead of the orbit, 3
  depths (P0, ±`depth_m` along tool z) × {no offset, ±`lateral_m` along each
  lateral axis}, each with a rotation that keeps the board where it was:
  `mid(z0)/centre`, `near(z+0.10)/+x`.
- **Within a viewpoint:** the targets are fixed when it starts: the cells
  empty at that moment, plus bin-seeking cells (the four frame corners while
  capture's tilted bins for its capture band are short, the two centre
  cells while its flat bin is short). The viewpoint itself is visited while
  any bin of its band is short. Cells are visited nearest first, aiming the
  camera so the board covers each cell and stays whole in view. A cell
  filled meanwhile is dropped. The log shows `[N left]`.
  Pointing at a corner tilts the board 25–35° compoundly. After each target
  it waits for a still board, waits `min_interval_ms`, and reads whether
  capture accepted the view.
- **Aiming:** from the current centroid, by the pinhole angle to the target.
  A target over `two_move_px` away takes two moves: `first_move_fraction` of
  the rotation, a re-read of the centroid and of how the board has grown
  (the aim moves in if the board would leave the frame), then the rest. Each
  move logs an `aim <leg>` line with the error. A last move missing by more
  than `centre_tol_px` gets one correction.
- **Before every move**, after the box check: the arm's own IK
  (`ComputeInverseKinematics`) at 1/4, 2/4, 3/4 of the path and at the end,
  each seeded with the previous solution (the first with the measured
  joints). A failed IK, a joint within 5° of either end of its limit in
  `config/arm.yaml`, or a joint jumping over 45°
  between samples (a branch change near a singularity) marks the target
  `unreachable`. It is logged, not sent, and the session moves on.
- **Skips:** a target outside the box or unreachable (logged); a board that
  leaves the view (it steps halfway back, retries once, then returns); a
  board not still within `still_timeout_s`.
- **Abandons a viewpoint**, logs `NAME: viewpoint abandoned (reason)` and
  goes on to the next viewpoint (a joint move, in a taught session) when the
  board isn't in view after a return (from a skipped target, a correction or
  an abort), or that return fails its checks.
- **Leaves a viewpoint at the reach edge**, logging `NAME: viewpoint at
  reach edge (first 2 cell pans IK-unreachable; N cells not tried: [...])`,
  when its first two cell pans are both `unreachable`.
- **An arm abort** (`ACTION_ABORT`, or `ExecuteAction` refused) on a
  validated target is logged as `aborted: <reason>`, with the joints after
  it. The arm then goes back to the last reached pose and the session
  continues. A third abort in a row (aborted returns included; a reached
  target resets the count) ends the session without moving again.
- **Ends** at capture's end condition, when nothing is left to visit, on
  `q`, on Ctrl-C, or when the status file is over 2 s old (capture died).
  A capture reporting a stall is not dead: the session logs `capture
  stalled: no frames from SERIAL ...; arm held, waiting up to 30 s`, holds
  the arm still, and resumes on `capture recovered after N s`; if frames
  don't come back within 30 s it ends with `capture stalled and did not
  recover within 30 s`.
  A board out of view does not end it (above).
  Each of these returns to P0. After three aborts in a row, or a move
  timeout, it does **not** move again and prints the `arm.py goto` command
  back to P0.
- **Cable winding.** The dry run's `cable` line names the reference (the
  saved `hub` pose) and the window. Start with the cable hanging
  unwrapped: the count assumes each continuous joint is within half a
  turn of the hub at the start. Every joint move turns a continuous joint
  the way that keeps it within ± `cable_window_deg` of the hub, printing
  `cable J1 … (short way | long way round: unwinding, turns A -> B)`.
  After a Cartesian pan has taken one outside, an `unwind` line comes
  before the next pan: same pose, the joint turned back. If an unwind is
  refused (the box) or aborts, the session ends `cable unwinding refused`
  or `cable unwinding aborted` and returns to P0. The return to P0 ends
  with net zero turns; the summary's `cable` line shows them.
- **Records** `session_arm.json` in the capture's session folder, rewritten
  after every target: P0, every commanded pose (absolute and relative to P0
  in P0's tool frame), the joints before it and the IK solution it was
  validated with, the measured pose, the cell or variant it was for,
  capture's decision, timings (`ik_s`, `move_s`, `still_s`, `decide_s`),
  the aim error of each move, the board estimate, the cap, the mode, the
  taught poses used, every `abandon` entry, every capture stall
  (`capture_stalls`), the cable state (`cable`, each pose's `cable_turns`,
  every `unwind` entry), and the summary. Schema
  `panoptes.session_arm/8`.

**Limit: capture's bins are fixed, the planner's names are relative.**
Capture bins distance by apparent square size against
`near_square_px`/`far_square_px` in `config/autocapture.yaml`. If those
don't bracket P0, every view lands in one capture band, and capture's other
bins (and `complete`) can't be reached. The dry run's `bands` line gives the
two values that would make capture bin the way the planner names variants
(P0 ±`mid_band_fraction`). Set them before starting capture if you want
that; they are copied into `session.json`. Also, a board at the edge of
the frame looks bigger (FINDINGS §20), so corner views can land a capture
band closer. When the session sees that, it stops corner-seeking for that
band.

**Procedure, one camera's intrinsics on the arm**

Per camera, in this order. Done for all ten cameras; results in FINDINGS
§16. Repeat it only for a camera that extrinsics shows needs
recalibrating (FINDINGS Open items #27).
`PATH` is `"$HOME"/panoptes-data/capture_status.json`.

1. **Unmount** the camera from its rig position. Its mask (and any
   extrinsics) is invalid from here on.
2. **Mount** it on the arm's camera bracket and connect it. **Tie the cable
   down at the bracket and leave slack through the wrist** before any
   session: jog the wrist (joints 5 and 6) through its full range in the
   Web App and check the cable never pulls tight. A pulled cable drops the
   camera off the network mid-session (`no frames from SERIAL`, `state
   Disconnected`, FINDINGS §11). With the arm at the `hub`, the cable must
   hang without a wrap round the wrist: sessions count turns from there.
3. **Unfold the arm.** `arm.py status`; jog every joint with a negative or
   small `limits` margin to mid-range with the Web App (steps 3–4 of
   **Procedure** above). Close the Web App.
4. **Teach, once per easel position**:
   `arm_capture.py teach --status-file PATH` with capture running
   (`--auto-dry --status-file PATH`). Skip if the easel and arm base haven't
   moved since the last teach and `config/arm_poses.yaml` has all eight and
   the `hub`. If a taught corner no longer shows the whole board with this
   camera, teach again with `--force`.
5. **Probe, once per mounting:** `arm.py goto-pose hub --yes`, board
   centred and fully visible (jog if not), capture in `--auto-dry`; then
   `arm_capture.py probe --status-file PATH`, read the plan, and again with
   `--yes`. It writes `config/arm_camera.yaml`.
6. **Session, twice.** Each time start capture fresh:
   `src/04_capture/build/capture <serial> --gain 0 --intensity 15 --exposure 8133 --auto --status-file PATH`.
   Then the taught-session dry run and `--yes` (below). Gain 0, not 7: at
   near poses the IR ring glares off the board at higher gain
   (FINDINGS §16). Quit capture with `q` when the session ends.
7. **Fit:** `.venv/bin/python python/fit_intrinsics.py --write-camera <session_1> <session_2>`.
   Expect RMS under ~0.2 px, then `wrote config/intrinsics/<serial>.yaml`.
   The ten cameras' ranges are in FINDINGS §16: fx/fy 0.4–1.4 px, cx/cy
   0.2–1.8 px. Read the `spread across sessions` lines and set `--status`
   by the flag rule. **Marginal** if the fx or fy range is over 1.0 px, or
   cx or cy over 3.0 px:
   `--status "done, marginal (P N px)"`, where P names the failing
   parameters (`fx`, `fy`, `fx/fy`, `cx/cy`...) and N is the larger range
   to 0.1 px. Otherwise `--status done`. Fitting before knowing the
   status? Fit without `--write-camera` first, or refit with `--force`.
   Record the fit in FINDINGS §16 and its per-camera progress table.
8. **Unmount** it from the arm. Remounting and masks wait for the stage
   below.

**Procedure, after all ten cameras' intrinsics: remount, masks, then extrinsics**

Every camera has been off its rig position, so every mask is wrong
(FINDINGS §15).

1. **Remount** each camera at its rig position (name and serial in
   `config/layout.yaml`, top view in `docs/panoptes_camera_layout.svg`; the
   positions in the file are not measured, see §10 item 5),
   filter in, ring at 15. Check all ten come up (§4):
   `src/01_discover/build/discover` prints `settled on 10 camera(s)`, and
   every `src/05_identify/build/identify` tile shows fps > 0.
2. **Re-learn masks** (§6) once, for all ten. Run `src/07_mask/build/mask`
   with the volume empty; learn runs on every camera, then applies,
   verifies and saves `config/masks/<serial>.json`. Check the verify table
   reads `residual: none` with no `LEAK` line, and no `<-- TOO MUCH`.
3. **Accept** with `mask --coverage` (§6): walk one marker through the
   volume; expect `PASS`.
4. **Extrinsics** with all ten intrinsics held fixed; then test refining
   focal length against the wand-length residual. Not built yet: FINDINGS
   Open items #7. A camera that stands out there gets the per-camera
   procedure above again (Open items #27).

**Procedure, taught session**

1. **Teach** once per easel position, and **probe** once per camera
   mounting (above). The per-camera procedure below gives the order.
2. **Start capture** with `--auto` and `--status-file`. Check
   `capture_status.py PATH` shows `session_path`, not `null`.
3. **Dry run:** `.venv/bin/python python/arm_capture.py session --status-file "$HOME"/panoptes-data/capture_status.json`.
   Read the `taught` line, the `ik` line, the viewpoints (each with its
   `family`), the `checks` count and the `families` line; it ends `dry run:
   nothing sent`. The `ik` line reads `8/8 corners use their taught joints
   (no IK); N others: B outside the box, S solved seeded by the nearest
   taught pose, P by the previous viewpoint after that failed, F failed with
   every seed`; each failed attempt follows as `  IK failed  NAME  seeded
   SEED: error`. Corners never go through IK.
   `no 'hub' pose`: save one (above).
4. **E-stop in reach; nothing and nobody in the sweep.** Viewpoint and hub
   moves are joint moves at the arm's own speed, and the box checks only
   their ends. A hub move swings the whole arm between configurations.
5. **Run with `--yes`.** `q` or Ctrl-C returns to the start and ends cleanly.
   Mid viewpoints come first. `viewpoint at reach edge` on the near corners
   is expected; `viewpoint abandoned` on most viewpoints is not: stop (`q`)
   and check the easel and the taught poses.

**Procedure, `--orbit`**

1. **Place the easel and P0** ("Easel and P0" above), then **probe, at the
   session's P0** (above). Rerun the probe whenever the
   easel, the camera mount or P0 changes; the session refuses a P0 more
   than 50 mm or 10° from the probe's start.
2. **Easel steady, board centred and fully visible** at P0: `corners 88/88`,
   no HOLD STILL, at least 40 px of margin.
3. **Start capture for real,** in another terminal:
   `src/04_capture/build/capture <serial> --auto --status-file "$HOME"/panoptes-data/capture_status.json`
   Check `capture_status.py PATH` shows `session_path`, not `null`.
4. **Dry run:** `.venv/bin/python python/arm_capture.py session --orbit --status-file "$HOME"/panoptes-data/capture_status.json`.
   It refuses unless capture is in `--auto`, the board is 88/88 and still,
   the camera matches the probe's, and the arm is at the probe's P0. Read:
   - `board`: the estimated position in the base frame and the distance.
     If it isn't about where the easel stands, stop. Two distances that
     differ by over 15% print a `WARNING`;
   - `cap`: viewpoints reachable, per radius, and how many the box and the
     IK removed. Under ~30%, the easel is too far or P0 is badly placed
     (see "Easel and P0");
   - `bands`: the planner's mid band, capture's thresholds, and the
     `config/autocapture.yaml` values that would match them. To use them,
     stop capture, edit, restart capture, and rerun the dry run;
   - a `WARNING` if the radii don't reach all three capture bands (with
     `lateral_variants: 1`, if near and far differ by no more than
     `band_spread_min`, or every depth is named mid);
   - each viewpoint's name, pose, predicted square size (and change from
     P0) and capture band, or `SKIP` with the reason (box or IK);
   - `checks`: how many poses passed the box and IK.
5. **E-stop in reach, hand near it; nobody inside the arm's or the camera's
   sweep.** The tool travels the cap: up to about 0.4 m at 1.0 m, plus up
   to ~35° of aiming about the TCP at each viewpoint.
6. **Run with `--yes`.** One line per target: `[i] kind variant cell
   outcome` and capture's verdict, or `ACCEPTED (bin)`. Watch:
   - capture's window: the board should stay whole; HOLD STILL only while
     moving;
   - `skipped: board left the view` more than occasionally: stop (`q`) and
     rerun the probe;
   - `unreachable` lines: the arm's IK refused the target (the note says
     why); many in a row mean P0 is near a joint limit or singularity;
   - `aborted:` lines: logged and recovered; three in a row end the session;
   - `viewpoint abandoned` lines: the board wasn't in view after a return;
     the session moves on to the next viewpoint;
   - a `capture-<band>: a corner view was binned ...` line: that band's
     tilted bins won't fill this way (the limit above).
7. **It ends by itself** and returns to P0, or press `q` (after the current
   move) or Ctrl-C (immediately: `Base.Stop`, then back to P0; a second
   Ctrl-C stops the return too). The `summary` gives moves commanded,
   accepted, redundant, skipped / unreachable / aborted, viewpoints
   abandoned and at the reach edge, total time, the
   time per move (and how much of it the arm was moving), the share of the
   session spent moving, waiting for still, waiting for capture's decision
   and in IK checks, cells remaining and bins short.
   If capture reached its end condition it has exited; otherwise quit
   capture with `q`.
8. **Solve** as for any auto session (§9). `session_arm.json` sits next to
   `session.json`.

**Troubleshooting**

- **`REFUSED    NAME: the arm isn't free`.** The Web App is still connected,
  or the arm isn't idle. Close it and press Enter again.
- **`taught poses missing`.** Run `teach`. Poses from `save-pose` have no
  square size, so they don't count.
- **`no 'hub' pose` / `the 'hub' pose is unusable`.** Save a board-centred,
  mid-range pose with `arm.py save-pose hub` (`--force` to replace it).
- **Many `SKIP: unreachable` or many families.** The taught corners are far
  apart in arm configuration. Teach them again closer to one configuration.
- **`not starting: arm is N mm / D deg from the probe's start pose`.** Move
  the arm back with `arm.py goto-pose NAME`, or rerun the probe here.
- **`not starting: actuator 5 is at A deg, within 10 deg of 0 (wrist
  singular)`.** Use `--from-pose NAME`, or `arm.py goto-pose NAME` first.
- **`ACTION_ABORT: INVALID_PARAM` on `move`, `goto` or a session target.** The
  Cartesian planner rejected the path: a singular start or a singular
  crossing, not a bad parameter. Get off it with `joints` or `goto-pose`.
- **`cap` reachable fraction is low.** `outside the box` dominating: the
  easel is too far, or P0 is too near an edge of the box (`r-0.20` needs
  x ≤ 0.50 m). `refused by IK` dominating: P0 is near a joint limit; pick
  another P0 and rerun the probe there.
- **`aim final` lines miss by over 100 px, or a wrong direction.** The gains
  no longer fit: rerun the probe.
- **`not starting: 04_capture is not in --auto`.** The session writes into
  capture's session folder; a dry-run capture has none.
- **`arm failure: 3 consecutive aborts`.** Read the three `aborted:`
  entries' `joints_after_abort_deg` in `session_arm.json`, and the arm's
  fault in the Kinova Web App. Clear it, return to P0 with the printed
  `arm.py goto` (dry run first), then restart.
- **`capture died`.** The status file went stale; the session returned to
  P0. Check capture's terminal; `main loop blocked ... in STEP` there means
  capture hung in a call.
- **`capture stalled and did not recover within 30 s`.** Capture's camera
  sent no frames and its restarts didn't bring them back; the session
  returned to P0. Read capture's `no frames` / `restarting` lines, then see
  §9 troubleshooting.
- **`arm: cannot reach 192.168.1.10:10000 (timed out)`.** `192.168.1.11/24`
  is missing from `enp6s0`, or the arm is off or still booting. Check
  `ip -brief addr show enp6s0`, then `ping -c 3 192.168.1.10`.
- **`arm: session refused`.** Check the credentials first. If they are
  right, close other clients (e.g. the Kinova Web App) and retry.
- **`REFUSED, nothing sent` / `current pose: ... outside`.** The arm is
  outside the box; see step 4.
- **`servoing mode is LOW_LEVEL_SERVOING`** (or another mode). Something
  else switched it. `arm.py` won't change it; set single-level servoing in
  the Kinova Web App.
- **`ExecuteAction refused by the arm`.** Usually a fault on the arm (e.g.
  after an E-stop). Clear faults in the Kinova Web App, then `status`.
- **`ACTION_ABORT: <reason>`.** The arm rejected or stopped the trajectory
  (e.g. a joint or Cartesian limit). The arm stays where it stopped.
  `<reason>` is `SUBCODE; notifications: …; trajectory errors: …`. Above
  it, `arm.py` prints `what the arm reported:` with the action sent, every
  notification in full (`abort_details`, `trajectory_info`) and
  `GetTrajectoryErrorReport`. Copy that block into FINDINGS.
- **`not starting: board not still`** with the arm at rest. Something is
  shaking the easel or the camera, or the mount flexes; fix that first.
- **`came back N px from where it started`.** The camera moved in its
  mount, the easel moved, or the arm didn't return. Check, return to P0,
  rerun.
- **`status file is N s old`.** `capture` isn't running, isn't in an auto
  mode, or was given a different path.
- **`TypeError: Descriptors cannot be created directly`.** protobuf 4.x or
  later got installed; reinstall with the first line.
- **`AttributeError: module 'collections' has no attribute
  'MutableMapping'`.** protobuf 3.5.1 got installed (a plain `pip install`
  of the wheel pulls it); reinstall with the first line.
- **Reconnect refused right after a crashed script.** The arm drops the
  dead session by itself 10 s after its last keep-alive; wait that long
  and retry.

---

## 14. Frame sync

```sh
src/08_sync/build/sync --label empty
src/08_sync/build/sync --label marker
src/08_sync/build/sync [--seconds N] [--label TEXT] [--delivery complete|favor-timely|timely] [--cameras N] [--warmup S] [--config DIR] [--out DIR]
```

| Flag | Default | Does |
|---|---|---|
| `--seconds N` | 60 | Recording length |
| `--label TEXT` | none | Goes into the folder name and `session.json`; letters, digits, `-` and `_` |
| `--delivery` | `complete` | `cModuleSync` optimisation: `complete`, `favor-timely` or `timely` |
| `--cameras N` | 10 | Refuses to record unless exactly N cameras open |
| `--warmup S` | 2 | Seconds of groups discarded before recording |
| `--config DIR` | `config` | Where `mask.yaml` and `masks/` are; relative to where you run it |
| `--out DIR` | `$HOME/panoptes-data/sessions` | Session root |

`sync` opens every camera in object mode at `config/mask.yaml`'s settings,
applies the saved masks, and records the SDK's frame groups. It has no
window. What the numbers mean is in FINDINGS §21; the file format is in
ARCHITECTURE §10.

1. **Before.** Masks learned today, at `mask.yaml`'s settings, with the
   cameras where they are now (§6). No other camera tool running; wait a
   couple of seconds after stopping one.
2. **Run from the repo root.** Discovery and opening ten cameras take about
   a minute before the table appears.
3. **Read the table.** Every row must say `object` under mode and `yes`
   under object-mode-synchronous, with the readback equal to `mask.yaml`.
   A `READBACK DIFFERS` line names a setting the camera did not take.
4. **Recording.** A progress line every 5 s. Ctrl-C ends early; the session
   is still written, with `"ended": "interrupted"`.
5. **Read the summary.**
   - `complete`: share of groups holding all ten cameras.
   - `cameras present`: how many groups held 10, 9, ... cameras.
   - `group frame ID ... gaps`: groups the SDK never delivered.
   - `frame ID spread`: groups whose cameras carry different frame IDs.
   - Per camera: `missing`, `id-gaps`, `id-skipped`.
   - `hardware stamps`: each camera's stamp interval, and its offset to the
     master camera. `range us` and `drift ppm` are how far that offset
     moved during the run.
6. **Two recordings make a set:** one with the volume empty
   (`--label empty`), one with a single marker carried through it
   (`--label marker`). Record the numbers in FINDINGS §21.
7. **Output** is `sync_<label>_<YYYYMMDD_HHMMSS>/` under the session root:
   `groups.jsonl` (one group per line, about 87 kB per second with ten
   cameras and one blob each) and `session.json`.

**Troubleshooting**

- **`REFUSED: N cameras opened, --cameras expects 10`.** A camera is down
  (§4). `--cameras N` records with fewer, on purpose only.
- **`mask REFUSED - learned under other settings (...)`.** The saved mask
  doesn't match `mask.yaml`. Re-learn (§6); don't edit either file to match.
- **`mask REFUSED - no saved mask`.** Learn masks first (§6).
- **`NO GROUPS during warm-up`.** The cameras are streaming (the table
  showed `object`) but the SDK formed no group. The recording goes ahead so
  the absence is on file; report it.
- **`complete` below 100%.** Read `cameras present` and the per-camera
  `missing` column to see which camera, then `identify` (§4) for that
  camera's fps.
- **Every camera shows one blob in an "empty" volume.** Something
  reflective is in view of all ten. It doesn't affect the sync numbers, but
  the recording isn't an empty-volume one.
