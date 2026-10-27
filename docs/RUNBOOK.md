> **2026-10-19:** the repo root is now what used to be `mocap/` in GIX-TALON (renamed PANOPTES).

# Runbook

An operator guide for someone standing in front of the rig. Every flag and key
here is taken from the source.

- Why things are the way they are: [ARCHITECTURE.md](ARCHITECTURE.md)
- What was measured: [FINDINGS.md](FINDINGS.md)
- Camera specs: [hardware/cameras.md](hardware/cameras.md)

All paths are relative to the repo root. Run commands from the repo root.

---

## Quick reference

```sh
# 1. Build every tool (section 3)
for t in src/0*/; do cmake -S "$t" -B "$t/build" -DCMAKE_BUILD_TYPE=Release && cmake --build "$t/build" -j"$(nproc)" || break; done

# 2. Grant network capabilities. After EVERY build: relinking strips them (section 3)
for b in src/01_discover/build/discover src/02_stream/build/stream src/03_detect/build/detect \
         src/04_capture/build/capture src/05_identify/build/identify src/06_focus/build/focus \
         src/07_mask/build/mask src/07_objdiag/build/objdiag; do sudo setcap cap_net_raw,cap_net_admin+eip "$b"; done

# 3. List every camera on the network
src/01_discover/build/discover

# 4. Stream one camera in grayscale; saves one frame to ./frame.png
src/02_stream/build/stream [serial]

# 5. Open all cameras in object mode, tiled, labelled by serial
src/05_identify/build/identify [fps] [exposure] [threshold] [gain] [--size WxH]

# 6. Focus assist for one camera
src/06_focus/build/focus <serial>

# 7. Blocking masks: apply saved masks and verify (empty volume)
src/07_mask/build/mask --load

# 8. Intrinsics capture with live coverage grid
src/04_capture/build/capture [serial]

# 9. Solve intrinsics for one or more sessions
.venv/bin/python python/fit_intrinsics.py "$HOME"/panoptes-data/sessions/intrinsics_<serial>_<timestamp> [...]

# 10. Triangulation self-test, then the synthetic accuracy study
.venv/bin/python python/triangulate.py && .venv/bin/python python/accuracy_study.py
```

The default serial is 33661 wherever one is optional (tools 4 and 8).

---

## 1. Environment setup from a clean clone

### 1.1 System packages

```sh
sudo apt install build-essential cmake libopencv-dev python3.12-venv tcpdump
```

- `libopencv-dev` provides OpenCV **4.6**, used by every C++ tool.
- `python3.12-venv` is **not installed by default on Ubuntu 24.04**. Without it,
  `python3 -m venv` fails with a confusing error.
- `tcpdump` is for the network check in section 2.

### 1.2 OptiTrack Camera SDK

The SDK is expected at `~/optitrack/CameraSDK`, outside the repo. Each
`src/*/CMakeLists.txt` sets `SDK_DIR` to `$ENV{HOME}/optitrack/CameraSDK`. If
yours is elsewhere, edit it there; there is no command-line override. The
tools link `CameraLibrary` and `jpeg` from `$SDK_DIR/lib`.

### 1.3 Python venv

Ubuntu 24.04 refuses `pip install` outside a venv (PEP 668), so a venv is
required. Pinned versions are in `python/requirements.txt`:

```sh
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r python/requirements.txt
```

**Recreating the venv.** A venv records its absolute path, so it breaks if
the repo directory is moved or renamed. Delete it and recreate it:

```sh
rm -rf .venv
python3 -m venv .venv
.venv/bin/pip install --upgrade pip
.venv/bin/pip install -r python/requirements.txt
```

To refresh the pins after deliberately changing a package:
`.venv/bin/pip freeze > python/requirements.txt`.

Verify. This must print a 5.x version and `True`:

```sh
.venv/bin/python -c "import cv2; print(cv2.__version__, hasattr(cv2.aruco,'CharucoDetector'))"
```

### 1.4 Two OpenCV versions, on purpose

C++ uses the system OpenCV 4.6. Python uses OpenCV 5.x from the venv. Don't
unify them. The calibration solve needs 5.x's `CharucoDetector` /
`matchImagePoints`, and 5.x can't be loaded into a process already linked to
4.6. The venv also keeps 5.x's numpy 2.x away from apt's numpy-1.x packages.
See [FINDINGS §16](FINDINGS.md#16-build-and-host-environment).

---

## 2. Camera network

The cameras are powered over PoE from a switch connected to the host NIC
**`enp6s0`**. They self-assign link-local (169.254.x.x) addresses and
broadcast on UDP 13013. `enp6s0` is dedicated to the camera network and is
link-local only; internet comes over Wi-Fi.

Check the link, then check that cameras are broadcasting:

```sh
ip -brief link show enp6s0                    # want: UP ... LOWER_UP
sudo tcpdump -i enp6s0 -n port 13013          # want: packets from 169.254.x.x
```

- **`NO-CARRIER`**: nothing is plugged in, or the cameras are unpowered.
- **Link up but silence on 13013**: the cameras aren't broadcasting.
  Power-cycle the PoE switch.

**Unverified:** how `enp6s0` was given link-local addressing on this host is
not recorded. On 2026-10-19 no NetworkManager profile was bound to it, with
the link down. With NetworkManager, one way is
`nmcli connection add type ethernet ifname enp6s0 con-name cameras ipv4.method link-local ipv6.method disabled`.

---

## 3. Build and setcap

Each tool under `src/` is its own CMake project. There is no top-level
build. Build all of them:

```sh
for t in src/0*/; do
  cmake -S "$t" -B "$t/build" -DCMAKE_BUILD_TYPE=Release &&
  cmake --build "$t/build" -j"$(nproc)" || break
done
```

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

**setcap after every build.** The camera network needs raw-socket access.
**Linking strips file capabilities**, so every rebuild that relinks a binary
needs this again:

```sh
for b in src/01_discover/build/discover src/02_stream/build/stream \
         src/03_detect/build/detect src/04_capture/build/capture \
         src/05_identify/build/identify src/06_focus/build/focus \
         src/07_mask/build/mask src/07_objdiag/build/objdiag; do
  sudo setcap cap_net_raw,cap_net_admin+eip "$b"
done
getcap src/*/build/*        # a binary missing from this output has no capabilities
```

Don't run the tools under `sudo` instead. It breaks desktop audio for the
session and leaves root-owned files.

**`mask` has its config path built in.** `07_mask` compiles in the absolute
path of `config/` at configure time. If the repo moves, delete
`src/07_mask/build` and rebuild (then setcap), or pass `--config config`.

---

## 4. Tools

Startup includes discovery, which waits until the camera count has been
stable for 5 s (up to 30 s). Tools that open every camera repeat that wait
once per camera, so expect roughly a minute before ten cameras are streaming.
*That time is inferred from the code, not measured.*

### 01_discover: what is on the network

```sh
src/01_discover/build/discover
```

Takes no arguments. Prints the device count as it grows, then one line per
unique serial: revision, name, and `Initialized` or `not ready`.

### 02_stream: one grayscale frame

```sh
src/02_stream/build/stream [serial]          # default 33661
```

Opens one camera in grayscale (exposure 8133, intensity 15, 30 Hz), discards
stale non-grayscale frames, and writes `frame.png` to the **current
directory**. It exits 1 if no grayscale frame arrives.

### 03_detect: live ChArUco detection for settings sweeps

```sh
src/03_detect/build/detect [serial] [exposure] [gain] [fps] [--board FILE] [--legacy | --no-legacy]
# defaults 33661 8133 7 30, board config/charuco_board.yaml, legacy_pattern from the file
```

It first prints the board it loaded (and any `--legacy` / `--no-legacy`
override), then shows a live view with markers and corners drawn and
`markers N/M corners N/C ids>K n (max I) fit F px mean B` overlaid. It prints
that line about once a second.

- `ids>K n`: markers whose ID is beyond the board's range. These are misreads
  or another target.
- `fit`: worst residual, in px, of a homography from board coordinates to the
  detected corners. It reads −1 with fewer than 6 corners. A wrong layout or
  swapped rows/cols still gives a FULL corner count in OpenCV 4.6, with IDs on
  the wrong corners. Only `fit` shows it: a few px is right, ~200 px is wrong.

| Key | Action |
|---|---|
| `s` | Save the raw frame to `./detect_frame.png` |
| `q` | Quit |

### 04_capture: intrinsics capture

```sh
src/04_capture/build/capture [serial] [--board FILE]   # defaults 33661, config/charuco_board.yaml
```

It prints the board it loaded, and refuses to start if the board file is
missing or a measured size is blank. The full board record goes into
`session.json`. Use `--board config/charuco_board_old.yaml` to reproduce an
old-board session. Starts at the grayscale operating point (section 5). It creates
`$HOME/panoptes-data/sessions/intrinsics_<serial>_<YYYYmmdd_HHMMSS>/`
(local time) and prints the path. The overlay shows corners, captures, empty
cells, exposure against its ceiling, gain, and the tilted share.

| Key | Action |
|---|---|
| `space` | Capture. Accepted only with every corner of the board found; otherwise prints `skipped` |
| `u` | Undo the last capture, removing the PNG and its coverage |
| `+` / `=` | Exposure +1000, up to the camera's reported ceiling |
| `-` | Exposure −1000, down to 10 |
| `]` | Gain +1, up to 7 |
| `[` | Gain −1, down to 0 |
| `q` | Quit; prints the frame count, empty cells and tilted share |

### 05_identify: all cameras, object mode

```sh
src/05_identify/build/identify [fps] [exposure] [threshold] [gain] [--size WxH]
# defaults: 30 500 200 0; --size overrides the detected monitor size
```

Opens every initialised camera in object mode and tiles them five across on
the primary monitor (found via `xrandr`), each labelled with its serial,
fps and blob count. At start it prints each camera's mode, read back from
the camera:

- `object`: normal.
- `NO FRAMES`: nothing has arrived from that camera.
- `WRONG MODE`: see section 7 before believing it.

| Key | Action |
|---|---|
| `g` | Grayscale mode (exposure 8133, gain 7) |
| `o` | Object mode (launch exposure, gain 0) |
| `+` / `=` | Threshold +10, up to the camera's max |
| `-` | Threshold −10, down to the camera's min |
| `]` | Exposure +100, capped at 33133, which is correct only at 30 Hz |
| `[` | Exposure −100, down to 10 |
| `d` | Dump every blob (x, y, area, roundness) to the console |
| `q` | Quit |

`identify` does not load, clear or report blocking masks.

### 06_focus: focus assist

```sh
src/06_focus/build/focus <serial>
```

Grayscale at the default operating point. It shows the variance of the
Laplacian over the centre half (outlined), its peak, the current value as a
percentage of peak, and a history strip. Put the board in the centre
rectangle and twist the lens past the peak, then back to it. Focus with the
IR filter in.

The number is not comparable between cameras or scenes; use it only to find
the peak.

| Key | Action |
|---|---|
| `r` | Reset the peak |
| `q` | Quit |

### 07_mask: blocking masks

```sh
src/07_mask/build/mask [--load|--show|--coverage|--probe|--clear] [--config DIR]
```

All modes open every camera at the `config/mask.yaml` settings (section 5).

| Mode | Does | Volume |
|---|---|---|
| *(none)* | Learn: clear masks, 5 s countdown, 30 s learn, build, apply, 10 s verify, save to `config/masks/<serial>.json` | **Empty** of markers |
| `--load` | Apply saved masks, then verify for 10 s | Empty |
| `--show` | Apply saved masks; tile view with masked cells tinted red | Any |
| `--coverage` | Apply saved masks; walk **one** marker through the volume; logs to `logs/coverage_<stamp>.csv` | One marker |
| `--probe` | Measure SDK mask behaviour step by step | Empty |
| `--clear` | Remove masks from every camera | Any |

`--show` keys:

| Key | Action |
|---|---|
| `m` | Mask on/off, to see what the mask hides |
| `g` | Grayscale view |
| `o` | Object view |
| `q` | Quit |

`--coverage` stops on `q` and prints PASS or FAIL. It fails if any moment of
the walk was seen by fewer than 3 cameras. It also lists, for each camera,
how many losses happened next to its mask.

**Masks stay applied on the cameras after `mask` exits.** They are valid only
while no camera moves and only at the settings they were learned under.
`--load` warns on a settings change and refuses a grid mismatch.

### 07_objdiag: why does a camera show nothing?

```sh
src/07_objdiag/build/objdiag <serial> [exposure] [threshold] [gain] [fps] [--sweep]
# defaults: 500 200 0 30
```

Console only.

1. It prints the SDK state after start: mode, every setting with its range,
   IR ring, blocking-mask flag, sync support.
2. It prints what was asked against what the camera reports, and flags a
   threshold outside the camera's range.
3. Then, for every received frame, it prints the frame ID, the gap since the
   last frame (`dF`, dropped frames), the frame type, the object count, the
   readback threshold and exposure, and the empty polls in between. Every 5 s
   it prints a summary.

`--sweep` instead steps the threshold across the camera's range (13 steps,
1.5 s each) and reports the peak object count at each step. Stop the
per-frame log with Ctrl-C.

### Python

Run from the repo root with the venv interpreter.

| Command | Does |
|---|---|
| `.venv/bin/python python/fit_intrinsics.py <session> [<session> ...]` | Brown-Conrady fit on all frames. Writes `<session>/intrinsics.json`, prints a table, and with several sessions prints the spread across them |
| `.venv/bin/python python/compare_models.py <session>` | Brown-Conrady vs rational vs fisheye: held-out error and edge straightness. Writes `<session>/model_comparison.json` (training RMS only) |
| `.venv/bin/python python/bootstrap_intrinsics.py <session> [n_subsets] [subset_size]` | Refit random subsets (defaults 20 of 50) to see whether pose sampling explains a cross-session difference |
| `.venv/bin/python python/triangulate.py [layout.yaml]` | Triangulation self-test, 12 checks. Exit 0 only if all pass |
| `.venv/bin/python python/accuracy_study.py [layout.yaml] [study.yaml]` | Synthetic study. Writes `analysis/` PNGs, `accuracy_study.json` and `.npz`. Refuses to run if the self-test fails |

---

## 5. Operating points

**Grayscale, for calibration.** These are the `Settings` defaults in
`lib/camera.h`, used by `capture`, `focus` and `stream`:

| Setting | Value |
|---|---|
| IR filter | in |
| IR intensity | 15 |
| Gain | 7 |
| Exposure | 8133 µs |
| Frame rate | 30 Hz |

Expect every marker and corner (27/27 and 40/40 on the old 9 × 6 board; the
12 × 9 board gives 54/54 and 88/88, FINDINGS §17) at
1–1.5 m, and a live view of 6–8 fps; that rate is normal. The reasons are in
[ARCHITECTURE §2](ARCHITECTURE.md#2-two-operating-modes).

**Object mode, for tracking.** This is **not settled**. Two sets are in use
(FINDINGS §11):

| Used by | Exposure | Threshold | Gain | Intensity | Rate |
|---|---:|---:|---:|---:|---:|
| `identify`, `objdiag` defaults | 500 | 200 | 0 | 15 | 30 Hz |
| `mask` (`config/mask.yaml`); the saved masks were learned here | 10 | 40 | 0 | 15 | 30 Hz |

A mask is only valid at the settings it was learned under.

**The exposure ceiling depends on the frame rate:** `floor(1e6 / fps) − 200`.

| Rate | Ceiling |
|---:|---:|
| 30 Hz | 33133 |
| 60 Hz | 16466 |
| 120 Hz | 8133 |
| 240 Hz | 3966 |

After any rate change, read the ceiling back instead of assuming it.

---

## 6. Calibration session walkthrough

For one camera's intrinsics:

1. **Network up** (section 2), then `discover`. Confirm the serial is
   listed as `Initialized`.
2. **Focus.** Run `focus <serial>`, with the board in the centre rectangle at
   working distance. Find the peak, lock the lens ring, then `q`.
3. **Optional check.** Run `detect <serial>`. At 1–1.5 m it should read
   `markers 54/54 corners 88/88` with `fit` at a few px (12 × 9 board). If it
   is dark, raise the exposure (third
   argument, up to 33133 at 30 Hz) or the gain.
4. **Capture.** Run `capture <serial>` with the board at **1–1.5 m**. Further
   than that, the board fills too little of the frame and corners cluster
   mid-image.
   - Work the board into the **corners and edges** of the frame until
     `empty cells` reaches **0/30**. The line turns green.
   - At the same time, keep the **tilted** share at **30% or more**: tilt 20–40°
     about both axes, plus some roll and distance changes. Watch the tilt
     histogram; the amber bars are under 20°.
   - A useful pattern is nine positions (centre, four edges, four corners)
     fronto-parallel, the same nine tilted about the vertical axis, the same
     nine tilted about the horizontal axis, then fill what's left.
   - `u` undoes a bad capture. `q` ends the session and prints its folder.
5. **Solve.** Run `fit_intrinsics.py` on the session folder. Give it several
   sessions of the same camera to see the spread, which is the real accuracy
   figure.
6. **Read the result.**
   - **RMS is not the acceptance criterion.** Zero empty cells, step 4, is.
   - Look at `per_view_error` in `intrinsics.json` for a bad capture.
   - fx and fy more than ~1% apart points at too little tilt.
   - Compare with the previous fits in
     [FINDINGS §14](FINDINGS.md#14-intrinsics-of-camera-33661).
7. **To use the result**, edit `camera_model` in `config/layout.yaml` by
   hand. Nothing writes it automatically.

---

## 7. Troubleshooting

**`discover` or a tool reports fewer cameras than exist.**

- Enumeration is staggered and slow with ten cameras. `discover`, `capture`,
  `identify`, `focus`, `mask` and `objdiag` wait for 5 s of no change (30 s cap).
- `stream` and `detect` still use a 1.5 s window, which has undercounted ten
  cameras. If either says `camera N not found`, run `discover` first and
  retry.
- Check the link and the 13013 broadcasts (section 2).
- If a tool was just killed, wait a couple of seconds before starting the
  next. A new instance can hang while the old sockets are still bound.
- A serial listed twice by the SDK is collapsed automatically.

**`WRONG MODE` on startup** (`identify`, `mask`). This is probably a false
alarm. The flag describes the most recent *poll*, and a poll that found no
frame waiting reads as "not object" (FINDINGS, Open items #2). To be sure,
run `objdiag <serial>`. If its `type` column reads `Object`, the camera is
fine.

**A camera sees nothing in one region, or loses a marker in one place.** This
looks like a detection bug, but it may be a **blocking mask**. Masks stay on
the camera after `mask` exits, and `identify`, `objdiag` and `capture` don't
report them. The "blocking mask" line in `objdiag` is the SDK's flag, not the
camera's state.

- To see the masks: `mask --show`, and press `m` to compare.
- To rule them out: `mask --clear`.

**`0.0 fps` / `NO FRAMES` / "no cameras opened".**

- Check `getcap` first: setcap is missing after every relink (section 3).
- Then check the network (section 2).
- `objdiag` prints `[no frames at all - not a threshold problem]` when that
  is the case.

**Frames arrive (fps > 0) but 0 blobs.** This is a lighting, exposure or
threshold problem, not acquisition.

- Run `objdiag <serial> --sweep` to see the blob count across the camera's
  whole threshold range.
- Check the requested threshold is inside the camera's range; `objdiag`
  flags it.

**The exposure won't go higher, or it reads back lower than you asked.** The
ceiling is `floor(1e6/fps) − 200` (section 5). After a frame-rate change,
the old ceiling is wrong. `capture` reads the ceiling from the camera;
`identify`'s `]` key is hardcoded to 33133, the 30 Hz value. Compare
`asked` with the readback that `identify` and `objdiag` print.

**`capture` prints `skipped: only N/88 corners`.** The board is partly out
of frame or too far away. Move it closer than 1.5 m.

**`mask` prints `error: cannot read .../mask.yaml`.** The repo has moved
since `mask` was built. Rebuild it, or pass `--config config`.

**`mask --load` prints `REFUSED - saved grid ...`.** The saved mask doesn't
match the camera's grid. Relearn.

**`mask --load` prints `WARNING settings differ from learn time`.** Either
relearn, or run at the settings in `config/mask.yaml` that the masks were
learned under.

**`mask` learn reports `<-- TOO MUCH`.** The camera is looking straight at
other rings. Downtilt or re-aim it and relearn, rather than masking more than
10% of its frame.

**Python finds 0 corners on every frame.** The board's `legacy_pattern` is
wrong for the physical board (FINDINGS §7). Python takes it from the
session's `session.json` board record, or from
`config/charuco_board_old.yaml` for sessions recorded before the board
config existed. Each script prints the board it resolved.

**A tool prints `board config ...: square_mm.measured is missing`.** Fill in
the caliper measurements in `config/charuco_board.yaml`. Nominal sizes are
never used in their place.

**`accuracy_study.py` exits with "self-test failed".** Run
`python/triangulate.py` alone and read which check failed. The study refuses
to report numbers from a broken triangulator.
