#include "camera.h"

#include <cameralibrary.h>
#include <chrono>
#include <stdexcept>
#include <thread>

using namespace CameraLibrary;

namespace mocap {

std::string sdkVersion() { return CameraLibrary::VersionString(); }

// ---------------------------------------------------------------------
// Discovery
// ---------------------------------------------------------------------

std::vector<CameraInfo> discover() {
    CameraManager::X().WaitForInitialization();

    // The SDK signals "ready" when the FIRST device initializes, not all
    // of them, so trusting it yields an incomplete list. Poll until the
    // count holds steady instead.
    //
    // The wait scales with camera count. Tuned for 4 cameras (6 stable
    // polls) this reported only 5 of 10 ready: the count plateaus while
    // later cameras are still initializing, and a short window reads
    // that plateau as "done". 30 s cap, 5 s of no change to conclude.
    size_t previous = 0;
    int stable = 0;
    for (int i = 0; i < 120 && stable < 20; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        CameraList list;
        if (list.Count() == previous && list.Count() > 0) ++stable;
        else { stable = 0; previous = list.Count(); }
    }

    // The list can report the same serial more than once. Collapse it so
    // each physical camera appears once.
    std::vector<CameraInfo> found;
    CameraList list;
    for (int i = 0; i < list.Count(); ++i) {
        int serial = list[i].Serial();
        bool seen = false;
        for (auto& c : found) if (c.serial == serial) seen = true;
        if (seen) continue;

        found.push_back({serial,
                         list[i].Revision(),
                         list[i].Name(),
                         list[i].State() == Initialized});
    }
    return found;
}

// ---------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------

struct Camera::Impl {
    std::shared_ptr<CameraLibrary::Camera> cam;
    Settings requested;
    bool applied = false;              // has apply() ever sent a full set
    Camera::FrameStats last;
    bool anyFrame = false;

    // ONE fetch path for both modes. grab() and grabBlobs() previously
    // had separate copies of this, and the copies drifted: grabBlobs()
    // never set the "received" flag, so callers gating on gotFrame()
    // threw away every object-mode frame. Duplicating this logic is how
    // that bug happened, so it lives in exactly one place now.
    std::shared_ptr<const Frame> fetch() {
        last = Camera::FrameStats{};

        // NextFrame() returns the OLDEST queued frame. One call per loop
        // and you fall progressively behind real time, so drain to the
        // newest. Reassignment drops the previous reference, freeing
        // that buffer. (LatestFrame() is the SDK's one-call equivalent;
        // this loop is kept because it is what the working grayscale
        // tools already run on this hardware.)
        std::shared_ptr<const Frame> frame;
        while (auto f = cam->NextFrame()) frame = f;
        if (!frame) return nullptr;

        last.received = true;
        last.object   = (frame->FrameType() == Core::ObjectMode);
        last.objects  = frame->ObjectCount();
        last.frameId  = frame->FrameID();
        anyFrame      = true;
        return frame;
    }
};

Camera::Camera(int serial)
    : impl_(std::make_unique<Impl>()), serial_(serial) {

    discover();   // ensure enumeration has completed

    CameraList list;
    for (int i = 0; i < list.Count(); ++i) {
        if (list[i].Serial() == serial) {
            impl_->cam = CameraManager::X().GetCamera(list[i].UID());
            break;
        }
    }
    if (!impl_->cam)
        throw std::runtime_error("camera " + std::to_string(serial) + " not found");
}

Camera::~Camera() = default;

void Camera::start(const Settings& s) {
    auto type = (s.mode == Mode::Object) ? Core::ObjectMode : Core::GrayscaleMode;

    impl_->cam->SetVideoType(type);
    impl_->cam->Start();

    // Mode and settings both go AFTER Start(): settings applied while
    // stopped are not reliably retained, and SetVideoType is async so
    // it needs reasserting once streaming has begun.
    impl_->cam->SetVideoType(type);

    // Do NOT pre-seed requested here: apply() diffs against it, and a
    // pre-seeded copy makes every setting look unchanged and sends none.
    apply(s);
}

void Camera::apply(const Settings& s) {
    // Send only what actually changed. The vendor's own sample app sets
    // one value per user action and nothing else, and re-sending the
    // full set mid-stream costs a settling period per command - which
    // looks like a stall in frame delivery. The first call after
    // start() sends everything.
    const Settings& was = impl_->requested;
    const bool all = !impl_->applied;

    // Frame rate first: it determines the exposure ceiling, so setting
    // exposure before the rate can leave it silently clamped.
    if (all || s.frameRate != was.frameRate) impl_->cam->SetFrameRate(s.frameRate);

    if (all || s.exposure  != was.exposure)  impl_->cam->SetExposure(s.exposure);
    if (all || s.intensity != was.intensity) impl_->cam->SetIntensity(s.intensity);
    if (all || s.gain      != was.gain)
        impl_->cam->SetImagerGain(static_cast<eImagerGain>(s.gain));

    if (s.mode == Mode::Object && (all || s.threshold != was.threshold))
        impl_->cam->SetThreshold(s.threshold);

    // Without this the camera stays silent whenever it sees no markers,
    // so a scene with nothing bright in it is indistinguishable from a
    // stream that never started. Object mode only; grayscale frames are
    // never empty.
    if (s.mode == Mode::Object && (all || s.emptyFrames != was.emptyFrames))
        impl_->cam->SendEmptyFrames(s.emptyFrames);

    if (impl_->cam->IsFilterSwitchAvailable() &&
        (all || s.irFilter != was.irFilter))
        impl_->cam->SetIRFilter(s.irFilter);

    // Only touch the video type when it actually changes - reasserting
    // it needlessly costs a mode-change settling period.
    if (!all && s.mode != was.mode) {
        impl_->cam->SetVideoType(s.mode == Mode::Object
                                 ? Core::ObjectMode : Core::GrayscaleMode);
    }

    impl_->requested = s;
    impl_->applied   = true;
}

void Camera::setThreshold(int t) {
    impl_->cam->SetThreshold(t);
    impl_->requested.threshold = t;
}

void Camera::setExposure(int e) {
    impl_->cam->SetExposure(e);
    impl_->requested.exposure = e;
}

cv::Mat Camera::grab() {
    auto frame = impl_->fetch();
    if (!frame) return {};

    // Stale frames from before the mode change are still object-mode;
    // GrayscaleData() on one returns null.
    if (impl_->last.object) return {};

    const unsigned char* pixels = frame->GrayscaleData(*impl_->cam);
    if (!pixels) return {};

    int w = width(), h = height();
    if (w != 1280 || h != 1024)
        throw std::runtime_error("unexpected frame dimensions " +
                                 std::to_string(w) + "x" + std::to_string(h));

    // Copy rather than wrap. Wrapping points at SDK memory that dies
    // with the frame, which is a use-after-free waiting to happen.
    // 1.3 MB at 8 fps is free.
    return cv::Mat(h, w, CV_8UC1, (void*)pixels).clone();
}

std::vector<Blob> Camera::grabBlobs() {
    auto frame = impl_->fetch();
    if (!frame) return {};
    if (!impl_->last.object) return {};

    std::vector<Blob> blobs;
    for (int i = 0; i < frame->ObjectCount(); ++i) {
        const cObject* o = frame->Object(i);
        blobs.push_back({float(o->X()), float(o->Y()),
                         float(o->Area()), float(o->Roundness())});
    }
    return blobs;
}

Camera::FrameStats Camera::lastFrame() const { return impl_->last; }

bool Camera::gotFrame()     const { return impl_->last.received; }
bool Camera::modeIsObject() const { return impl_->last.object; }
bool Camera::sawAnyFrame()  const { return impl_->anyFrame; }

// PhysicalPixelWidth/Height are pixel counts. ImagerWidth/Height are
// doubles - sensor size in mm. Width/Height describe an ROI crop.
int Camera::width()  const { return impl_->cam->PhysicalPixelWidth(); }
int Camera::height() const { return impl_->cam->PhysicalPixelHeight(); }

std::string Camera::name() const { return impl_->cam->Name(); }

Settings Camera::actual() const {
    Settings s;
    s.mode      = impl_->requested.mode;
    s.exposure  = impl_->cam->Exposure();
    s.gain      = static_cast<int>(impl_->cam->ImagerGain());
    s.intensity = impl_->cam->Intensity();
    s.frameRate = impl_->cam->FrameRate();
    s.irFilter  = impl_->requested.irFilter;

    // Read this back rather than echoing what we asked for. Reporting
    // the request is how "threshold correctly applied" got printed for
    // a value the camera may never have accepted.
    s.threshold = impl_->cam->Threshold();

    s.emptyFrames = impl_->requested.emptyFrames;
    return s;
}

int Camera::maxExposure() const {
    // Measured on 33661 across four rates: the ceiling is exactly
    // floor(1e6/fps) - 200. The SDK does re-report it, so ask rather
    // than caching - a value read before a rate change is wrong.
    return impl_->cam->MaximumExposureValue();
}

int Camera::minThreshold() const { return impl_->cam->MinimumThreshold(); }
int Camera::maxThreshold() const { return impl_->cam->MaximumThreshold(); }

Camera::MaskGrid Camera::maskGrid() const {
    auto& c = *impl_->cam;
    return {c.BlockingGrid(), c.BlockingMaskWidth(), c.BlockingMaskHeight()};
}

void Camera::stageMask(const cv::Mat& cells) {
    MaskGrid g = maskGrid();
    if (cells.type() != CV_8UC1 || cells.cols != g.cols || cells.rows != g.rows)
        throw std::runtime_error("mask " + std::to_string(cells.cols) + "x" +
                                 std::to_string(cells.rows) + " does not match camera grid " +
                                 std::to_string(g.cols) + "x" + std::to_string(g.rows));
    // One byte per cell, row-major. The size must be exactly cols*rows:
    // the SDK answers any other size by CLEARING the staged mask.
    cv::Mat c = cells.isContinuous() ? cells : cells.clone();
    impl_->cam->SetBlockingMask(c.data, int(c.total()));
}

void Camera::commitMask()          { impl_->cam->UpdateBlockingMask(); }
void Camera::enableMask(bool on)   { impl_->cam->SetEnableBlockingMask(on); }
bool Camera::maskEnabled() const   { return impl_->cam->IsBlockingMaskEnabled(); }

// UpdateBlockingMask only sends chunks where staging differs from what
// the SDK thinks the camera holds - and in a fresh process it thinks the
// camera holds nothing. Whether ClearBlockingMask itself reaches the
// camera is unverified. So force the camera's state from both sides:
// commit all-blocked (every chunk differs from "empty" and is sent), then
// commit the target (every chunk that is not all-blocked differs and is
// sent). Correct whatever the camera held before. Costs one frame or so
// of full blindness, at setup only.
void Camera::setMask(const cv::Mat& cells) {
    stageMask(cells);                 // validate before touching the camera
    impl_->cam->ClearBlockingMask();
    stageMask(cv::Mat(cells.size(), CV_8UC1, cv::Scalar(1)));
    commitMask();
    stageMask(cells);
    commitMask();
    enableMask(true);
}

void Camera::clearMask() {
    MaskGrid g = maskGrid();
    setMask(cv::Mat(g.rows, g.cols, CV_8UC1, cv::Scalar(0)));
    enableMask(false);
}

cv::Mat Camera::stagedMask() const {
    MaskGrid g = maskGrid();
    cv::Mat m(g.rows, g.cols, CV_8UC1, cv::Scalar(0));
    impl_->cam->GetBlockingMask(m.data, int(m.total()));
    return m;
}

std::string Camera::diagnostics() const {
    auto& c = *impl_->cam;

    auto modeName = [](Core::eVideoMode m) -> const char* {
        switch (m) {
            case Core::SegmentMode:   return "Segment";
            case Core::GrayscaleMode: return "Grayscale";
            case Core::ObjectMode:    return "Object";
            case Core::PrecisionMode: return "Precision";
            case Core::MJPEGMode:     return "MJPEG";
            case Core::VideoMode:     return "Video";
            default:                  return "Unknown";
        }
    };
    auto yn = [](bool b) { return b ? "yes" : "no"; };

    std::string r;
    r += "  name            " + std::string(c.Name()) + "\n";
    r += "  running         " + std::string(yn(c.IsCameraRunning())) + "\n";
    r += "  video type      " + std::string(modeName(c.VideoType())) + "\n";
    r += "  object mode ok  " + std::string(yn(c.IsVideoTypeSupported(Core::ObjectMode))) + "\n";
    r += "  object synced   " + std::string(yn(c.IsVideoTypeSynchronous(Core::ObjectMode))) + "\n";
    r += "  frame rate      " + std::to_string(c.FrameRate()) + "\n";
    r += "  exposure        " + std::to_string(c.Exposure()) +
         "  (" + std::to_string(c.MinimumExposureValue()) +
         ".." + std::to_string(c.MaximumExposureValue()) + ")\n";
    r += "  threshold       " + std::to_string(c.Threshold()) +
         "  (" + std::to_string(c.MinimumThreshold()) +
         ".." + std::to_string(c.MaximumThreshold()) + ")\n";
    r += "  intensity       " + std::to_string(c.Intensity()) +
         "  (" + std::to_string(c.MinimumIntensity()) +
         ".." + std::to_string(c.MaximumIntensity()) + ")\n";
    r += "  imager gain     " + std::to_string(static_cast<int>(c.ImagerGain())) + "\n";

    // An IR ring that is absent or off leaves retroreflective markers
    // unlit, and a blocking mask left enabled from an earlier session
    // zeroes out whole regions of the imager. Either one produces
    // exactly zero blobs with every other setting looking correct.
    r += "  IR ring present " + std::string(yn(c.IsIRIlluminationAvailable())) + "\n";
    r += "  continuous IR   " + std::string(c.IsContinuousIRAvailable()
             ? (c.ContinuousIR() ? "available, on" : "available, off")
             : "not available") + "\n";
    r += "  blocking mask   " + std::string(c.IsBlockingMaskEnabled() ? "ENABLED" : "off") +
         "  (" + std::to_string(c.BlockingMaskWidth()) +
         "x" + std::to_string(c.BlockingMaskHeight()) + ")\n";
    r += "  IR filter sw    " + std::string(c.IsFilterSwitchAvailable() ? "present" : "none") + "\n";
    r += "  hw filtered     " + std::string(yn(c.IsHardwareFiltered(Core::ObjectMode))) + "\n";
    r += "  max obj diam    " + std::to_string(c.MaxObjectDiameter()) + "\n";
    r += "  data rate       " + std::to_string(c.DataRate()) + "\n";
    return r;
}

}  // namespace mocap