# 07_mask: per-camera blocking masks

Each camera sees other cameras' IR rings as bright static blobs. A blocking
mask drops those regions **on the camera**, before blob detection.

## A mask is permanent blindness

A marker that passes behind a masked region disappears from that camera.
So:

- Masks are as tight as the glare: the observed extent of each static
  source plus its blob radius, then `dilate_cells` of margin.
- A mask is valid **only while the camera has not moved**. If a camera is
  bumped or re-aimed, relearn its mask. Each saved mask records when it was
  learned and under which settings; `--load` prints its age and warns if the
  settings have changed since.
- A mask covering more than `warn_coverage` of the frame means the camera is
  looking straight at other rings. Downtilt it instead of blinding it.
- `--coverage` is the acceptance test. A mask that silences interference
  but leaves part of the volume seen by fewer than 3 cameras is worse than
  the interference was.

## Use

```
cmake -S . -B build && cmake --build build
sudo setcap cap_net_raw,cap_net_admin+eip ./build/mask    # after every relink

./build/mask --probe      # once, empty volume: measure SDK behaviour
./build/mask              # learn -> apply -> verify -> save (EMPTY volume)
./build/mask --show       # check where the masks sit (m toggles, g grayscale)
./build/mask --coverage   # walk one marker through the volume, q to finish
./build/mask --load       # reapply saved masks without relearning
./build/mask --clear      # remove masks from every camera
```

Settings live in `config/mask.yaml`. Saved masks are written to
`config/masks/<serial>.json`, and coverage logs to `logs/`.

## SDK mask semantics

These were read from the SDK binary (`cBlockControlBase` /
`cBlockControlMask` in libCameraLibrary.so), not from documentation. They
are inferred, not yet confirmed on hardware; `--probe` checks the
behavioural ones.

- Grid: `cols = ceil(1280 / BlockingGrid())`, `rows = ceil(1024 / grid)`.
  `BlockingMaskWidth/Height` return those cell counts. Image pixel (x, y)
  is in cell (x / grid, y / grid).
- `SetBitMaskPixel(X, Y)` takes **cell** coordinates despite its name. A
  pixel coordinate is bounds-checked only against the cell grid, so it
  silently sets the wrong cell or is silently ignored.
- `AddBlockingRectangle` takes **pixel** coordinates and divides by the grid.
- `Get/SetBlockingMask` use one byte per cell, row-major, and the size must
  be exactly cols × rows. `SetBlockingMask` with any other size **clears**
  the staged mask; `GetBlockingMask` does nothing.
- Every edit goes into an SDK-side staging buffer. `UpdateBlockingMask()`
  sends the chunks that differ from what the SDK believes the camera holds.
  That belief starts empty in each new process, so a mask the camera kept
  from an earlier run cannot be read back. `Camera::setMask` therefore
  commits all-blocked and then the target, which forces every chunk to be
  sent.
- `IsBlockingMaskEnabled()` reports an SDK-side flag, which the controller
  constructor appears to set on. It says nothing about the camera.
