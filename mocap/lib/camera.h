// Acquisition from a single OptiTrack camera.
//
// Wraps the SDK behaviors that are easy to get wrong and never optional:
// staggered discovery, asynchronous mode changes, and the frame queue
// returning oldest-first. Callers get validated data or nothing.
//
// Two video modes, and the difference matters:
//
//   Grayscale - the full raw image. Large (~1.3 MB/frame), bandwidth
//   bound, and NOT hardware-synchronized across cameras. Needed for
//   calibration, because a checkerboard is not a set of bright blobs.
//
//   Object - the camera thresholds onboard and returns only blob
//   centroids. Tiny, fast, and hardware-synchronized across cameras.
//   This is what mocap actually runs on.

#pragma once

#include <opencv2/core.hpp>
#include <memory>
#include <string>
#include <vector>

namespace mocap {

struct CameraInfo {
    int serial;
    int revision;
    std::string name;
    bool initialized;
};

// CameraSDK version string, for provenance records.
std::string sdkVersion();

// Find every camera on the network. Blocks until the device count
// stops growing, because the SDK reports "ready" after the FIRST
// device initializes, not all of them.
std::vector<CameraInfo> discover();

enum class Mode { Grayscale, Object };

struct Settings {
    Mode mode     = Mode::Grayscale;
    int exposure  = 8133;
    int gain      = 7;       // 0-7
    int intensity = 15;      // IR ring, 0-15
    int frameRate = 30;
    bool irFilter = true;    // 850 nm bandpass in place
    int threshold = 200;     // object mode only: brightness cutoff

    // Object mode only. With this off the camera sends NOTHING when it
    // sees no markers, so "no frames" and "no blobs" become the same
    // observation and a dark scene is indistinguishable from a dead
    // stream. On, the cadence is continuous and ObjectCount tells the
    // truth. Keep it on for anything diagnostic.
    bool emptyFrames = true;
};

// One detected blob in object mode. Area and roundness are the camera's
// own measurements, useful for rejecting reflections that are not
// markers - a marker is small and round, a glare streak is not.
struct Blob {
    float x, y;              // centroid, sub-pixel
    float area;
    float roundness;
};

class Camera {
public:
    // Opens by serial. Throws std::runtime_error if not found.
    explicit Camera(int serial);
    ~Camera();

    void start(const Settings& s);
    void apply(const Settings& s);     // change settings while streaming

    // Change one setting without re-applying the rest. Re-applying every
    // setting mid-stream can stall frame delivery.
    void setThreshold(int t);
    void setExposure(int e);

    // Newest available grayscale frame, or an empty Mat if none is
    // ready. The Mat is a COPY - safe to hold.
    // Returns empty if the camera is in object mode.
    cv::Mat grab();

    // Newest available blob list, or empty if none is ready.
    // Returns empty if the camera is in grayscale mode.
    std::vector<Blob> grabBlobs();

    // What the last fetch actually saw. Every field describes the frame
    // that was received, not what was requested.
    struct FrameStats {
        bool received = false;  // NextFrame() returned a frame at all
        bool object   = false;  // its FrameType() was ObjectMode
        int  objects  = 0;      // its ObjectCount(); 0 is a legitimate value
        int  frameId  = -1;     // gaps here are dropped frames, which an
                                // fps average smooths away and hides
    };
    FrameStats lastFrame() const;

    // True only if the most recent grab()/grabBlobs() actually received
    // a frame. An empty result means "no frame ready yet" OR "a frame
    // carrying nothing" - callers that cannot tell those apart will
    // overwrite good data and count polls as frames.
    bool gotFrame() const;

    // What mode the last RECEIVED frame actually carried, as opposed to
    // what was requested. Mode changes are asynchronous, so these can
    // disagree - and a silent disagreement looks like a detection bug.
    bool modeIsObject() const;
    bool sawAnyFrame() const;

    int width()  const;
    int height() const;
    int serial() const { return serial_; }
    std::string name() const;

    // What the camera reports, which may differ from what was asked.
    Settings actual() const;

    // Exposure ceiling is frame-period-derived: floor(1e6/fps) - 200.
    // Re-read after any frame rate change; a cached value is a lie.
    int maxExposure() const;

    // The camera's own threshold limits. Ask before assuming 0-255:
    // feeding SetThreshold a value outside this range is silently
    // clamped or ignored, which reads downstream as "no markers".
    int minThreshold() const;
    int maxThreshold() const;

    // Multi-line dump of what the SDK reports about this camera: mode,
    // readback of every setting, threshold/exposure limits, and the two
    // features that can silently suppress every blob - the IR ring and
    // the blocking mask. For diagnostics; do not parse it.
    std::string diagnostics() const;

    // ---- Blocking mask -------------------------------------------------
    // Masked regions are dropped ON THE CAMERA, before blob detection: a
    // marker behind a mask does not exist as far as this camera is
    // concerned. The mask is a coarse grid, not per-pixel. Semantics below
    // were read from the SDK binary (cBlockControlBase/Mask), not docs:
    //
    //   * Image pixel (x, y) falls in cell (x / cellPx, y / cellPx).
    //   * SetBitMaskPixel takes CELL coordinates despite its name. Pixel
    //     coordinates are bounds-checked against the cell grid only, so
    //     small ones silently mask the wrong cell and large ones are
    //     silently ignored. This API is cell-only for that reason.
    //   * Every edit lands in an SDK-side staging copy. Nothing reaches
    //     the camera until UpdateBlockingMask(), which sends only chunks
    //     that differ from what the SDK believes the camera holds.
    //   * That belief starts EMPTY in every new process. So a mask the
    //     camera retained from an earlier run is invisible to readback,
    //     and an incremental update on top of it leaves stale cells in
    //     place. setMask() therefore always clears first.
    struct MaskGrid {
        int cellPx = 0;          // image pixels per cell side
        int cols = 0, rows = 0;  // cells across, down
    };
    MaskGrid maskGrid() const;

    // Replace the camera's mask: clear on camera, stage `cells` (CV_8U,
    // rows x cols, nonzero = blocked), commit, enable. Throws if `cells`
    // does not match maskGrid() - a mismatched mask blinds the wrong place.
    void setMask(const cv::Mat& cells);

    // Clear on camera and disable.
    void clearMask();

    // Primitives, exposed so the commit behaviour can be measured.
    void stageMask(const cv::Mat& cells);  // staging copy only, no I/O
    void commitMask();                     // UpdateBlockingMask()
    void enableMask(bool on);
    bool maskEnabled() const;              // SDK flag, not a camera readback

    // The SDK's STAGED copy. Shows what was asked for this process, not
    // what the camera currently applies.
    cv::Mat stagedMask() const;

private:
    struct Impl;                       // SDK types stay out of this header
    std::unique_ptr<Impl> impl_;
    int serial_;
};

}  // namespace mocap