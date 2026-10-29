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
for b in src/01_discover/build/discover src/02_stream/build/stream src/03_detect/build/detect src/04_capture/build/capture src/05_identify/build/identify src/06_focus/build/focus src/07_mask/build/mask src/07_objdiag/build/objdiag; do sudo setcap cap_net_raw,cap_net_admin+eip "$b"; done
getcap src/*/build/*
cmake -S tests -B tests/build -DCMAKE_BUILD_TYPE=Release && cmake --build tests/build -j"$(nproc)" && tests/build/test_autocapture
```
expect: `[100%] Built target` for all 8 tools; `getcap` lists all 8 with `cap_net_admin,cap_net_raw=eip`; `test_autocapture: 53/53 passed`.

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
src/04_capture/build/capture [serial]
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
src/04_capture/build/capture [serial] [--board FILE]
.venv/bin/python python/fit_intrinsics.py <session> [<session> ...]
```
expect: `empty cells 0/30` in green with `tilted` ≥ 30%; fx and fy within 1% of each other.

### 1.8 Auto intrinsics session (UNVALIDATED)
Tune with `--auto-dry` (writes nothing), then run. Not yet validated against manual capture. Details: §9.
```sh
src/04_capture/build/capture [serial] --auto-dry [--auto-config FILE] [--board FILE]
src/04_capture/build/capture [serial] --auto [--auto-config FILE] [--board FILE]
.venv/bin/python python/fit_intrinsics.py <auto_session> <manual_session>
```
expect: `ended: complete: 0 empty cells and every pose bin has >= 2 frames`; validation passes at fx/fy within ~1 px, cx/cy within ~3 px.

### 1.9 Python fits and studies
venv interpreter only. Details: §10.
```sh
.venv/bin/python python/fit_intrinsics.py <session> [<session> ...]
.venv/bin/python python/compare_models.py <session>
.venv/bin/python python/bootstrap_intrinsics.py <session> [n_subsets] [subset_size]
.venv/bin/python python/board.py [board.yaml]
.venv/bin/python python/triangulate.py [layout.yaml]
.venv/bin/python python/accuracy_study.py [layout.yaml] [study.yaml]
```
expect: `triangulate.py` prints `self-test: 12/12 passed`; the study prints `Wrote .../accuracy_study.json`.

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

### 1.11 Arm (Kinova Gen3, read-only)
E-stop pendant in reach first. Install once; `status` reads and never moves. Details: §13.
```sh
.venv/bin/pip install --no-deps -r python/requirements-arm.txt
ip -brief addr show enp6s0
.venv/bin/python python/arm.py status [--ip IP] [--port PORT] [--username U] [--password P]
```
expect: `pip` ends `Successfully installed`; `enp6s0` lists `192.168.1.11/24` and a `169.254.x.x` address; `status` prints firmware, serial, pose, joints and servoing mode, then `session closed`.

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
   `src/0*/` loop, with no camera SDK and no setcap. Run from the repo root;
   it reads `config/charuco_board.yaml` and `config/autocapture.yaml` and
   exits 0 only if every check passes. It covers the `--auto` accept logic
   on synthetic corners and what auto mode writes to `session.json`.
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
| tests | `tests/build/test_autocapture` |

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
src/04_capture/build/capture [serial]
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
src/04_capture/build/capture [serial] [--board FILE]
.venv/bin/python python/fit_intrinsics.py <session> [<session> ...]
```

`capture` prints the board it loaded and refuses to start if the board file
is missing or a measured size is blank. It starts at the grayscale operating
point (§12), creates `$HOME/panoptes-data/sessions/intrinsics_<serial>_<YYYYmmdd_HHMMSS>/`
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
7. **Use it** by editing `camera_model` in `config/layout.yaml` by hand;
   nothing writes it automatically. Record the fit in FINDINGS §16.

**Troubleshooting**

- **`skipped: only N/88 corners`.** The board is partly out of frame or too
  far. Move it closer than 1.5 m.
- **Board config errors at start.** See §5.
- **Python finds 0 corners on every frame.** See §10.

---

## 9. Auto intrinsics session

```sh
src/04_capture/build/capture [serial] --auto-dry [--auto-config FILE] [--board FILE]
src/04_capture/build/capture [serial] --auto [--auto-config FILE] [--board FILE]
.venv/bin/python python/fit_intrinsics.py <auto_session> <manual_session>
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
6. **Validation rule: do this before trusting auto mode.**
   1. On one camera, the same day, lens untouched: one `--auto` session and
      one manual session.
   2. Fit both together (third line of the block).
   3. **Pass** if **fx and fy agree within ~1 px and cx and cy within
      ~3 px**. Source: the same-day 2026-09-25 manual pair in FINDINGS §16
      (fx 0.23, fy 0.63, cx 2.51, cy 1.51 px). The three-session range there
      is wider (fx 2.48, cx 9.40 px; Open items #1), so a cx/cy failure
      alone may not be auto mode's fault: record a second manual session the
      same day and check the two manual ones agree first.
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

---

## 10. Python fits and studies

```sh
.venv/bin/python python/fit_intrinsics.py <session> [<session> ...]
.venv/bin/python python/compare_models.py <session>
.venv/bin/python python/bootstrap_intrinsics.py <session> [n_subsets] [subset_size]
.venv/bin/python python/board.py [board.yaml]
.venv/bin/python python/triangulate.py [layout.yaml]
.venv/bin/python python/accuracy_study.py [layout.yaml] [study.yaml]
```

| Command | Does |
|---|---|
| `fit_intrinsics.py` | Brown-Conrady on all frames; writes `<session>/intrinsics.json`, prints a table, and with several sessions the spread across them |
| `compare_models.py` | Brown-Conrady vs rational vs fisheye: held-out error and edge straightness printed; writes `<session>/model_comparison.json` (training RMS only) |
| `bootstrap_intrinsics.py` | Refits random subsets (defaults 20 subsets of 50) to see whether pose sampling explains a cross-session difference |
| `board.py` | Prints the board a yaml describes (default `config/charuco_board.yaml`), or the error |
| `triangulate.py` | Triangulation self-test, 12 checks; exit 0 only if all pass |
| `accuracy_study.py` | Synthetic study (defaults `config/layout.yaml`, `config/study.yaml`); writes `analysis/` PNGs, `accuracy_study.json` and `.npz`; refuses to run if the self-test fails |

1. Run from the repo root with `.venv/bin/python` only. Each script prints the
   board it resolved: from the session's `session.json`, or
   `config/charuco_board_old.yaml` for sessions recorded before the board
   config existed.
2. After changing anything in `python/` or `config/layout.yaml`, run
   `triangulate.py`; all 12 checks must pass.
3. `analysis/*.png` and `.npz` are regenerated and gitignored;
   `analysis/accuracy_study.json` is tracked. Record new numbers in FINDINGS.

**Troubleshooting**

- **0 corners on every frame.** The board's `legacy_pattern` doesn't match
  the physical board (FINDINGS §9). Check the board each script prints.
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

## 13. Arm (Kinova Gen3, read-only)

```sh
.venv/bin/pip install --no-deps -r python/requirements-arm.txt
ip -brief addr show enp6s0
.venv/bin/python python/arm.py status [--ip IP] [--port PORT] [--username U] [--password P]
```

> **Safety: the E-stop pendant must be within reach whenever anything in
> this repo talks to the arm.** That includes read-only commands like
> `status`.

`arm.py status` connects, opens a Kortex session, prints the arm's state and
closes. It sends **no motion command**; `arm.py` refuses any Kortex call not
on its read-only allow-list (ARCHITECTURE §9).

| Flag | Default |
|---|---|
| `--ip` | `192.168.1.10` |
| `--port` | `10000` (Kortex TCP) |
| `--username` | `admin` |
| `--password` | `admin` |

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
     (expect 6 DOF, `END_EFFECTOR_TYPE_NOT_INSTALLED`)
   - `devices`: every device the base lists, by type and id
   - `pose`: measured Cartesian pose, x y z in m, theta_x/y/z in deg
   - `joints`: measured joint angles, deg, by joint id
   - `servoing`: the current servoing mode, e.g. `SINGLE_LEVEL_SERVOING`
   - `session closed` on exit

**Troubleshooting**

- **`arm: cannot reach 192.168.1.10:10000 (timed out)`.** `192.168.1.11/24`
  is missing from `enp6s0`, or the arm is off or still booting. Check
  `ip -brief addr show enp6s0`, then `ping -c 3 192.168.1.10`.
- **`arm: session refused`.** Check the credentials first. If they are
  right, close other clients (e.g. the Kinova Web App) and retry.
- **`TypeError: Descriptors cannot be created directly`.** protobuf 4.x or
  later got installed; reinstall with the first line.
- **`AttributeError: module 'collections' has no attribute
  'MutableMapping'`.** protobuf 3.5.1 got installed (a plain `pip install`
  of the wheel pulls it); reinstall with the first line.
- **Reconnect refused right after a crashed script.** The arm drops the
  dead session by itself 10 s after its last keep-alive; wait that long
  and retry.
