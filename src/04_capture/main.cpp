// Intrinsics capture: live view with detection, coverage tracking, and
// capture to a session folder.
//
// Acceptance criterion is ZERO EMPTY CELLS, not RMS. See coverage.h.
//
// Manual mode: space captures. --auto: the operator moves the board slowly
// and lib/autocapture decides which frames to keep. --auto-dry: the same
// loop and decisions, printed, with nothing written. --status-file PATH
// (auto modes only) rewrites a JSON status after every frame, for another
// process to read (lib/capture_status.h).
//
// --gain N (0-7), --intensity N (IR ring, 0-15), --exposure US: start-up
// settings, sent with the rest in Camera::start() before the first frame.
// Defaults are mocap::Settings (gain 7, intensity 15, exposure 8133). The
// exposure is refused above floor(1e6/fps) - 200 before the camera starts,
// and again against the camera's own MaximumExposureValue() after the frame
// rate is set (CLAUDE.md rule 7). The requested and read-back values are
// printed and written to session.json as "baseline_settings".
//
// Stalls (auto modes). No frame for 1 s prints "no frames from SERIAL for N
// s" with Camera::stateLine(), again every 5 s. From 5 s, and every 5 s
// until frames return, the camera is stopped and restarted with the
// baseline settings (Camera::restart); the readback of the first frames
// after it is checked against them. The session carries on in the same
// folder; each stall is recorded in session.json "stalls". The status file
// keeps being rewritten during a stall, with "stalled" set. A watchdog
// thread prints if the main loop itself stops turning for 5 s (a call that
// blocks), naming the step it is in.

#include "../../lib/camera.h"
#include "../../lib/detect.h"
#include "../../lib/coverage.h"
#include "../../lib/session.h"
#include "../../lib/autocapture.h"
#include "../../lib/capture_status.h"

#include <opencv2/opencv.hpp>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <atomic>
#include <optional>
#include <thread>
#include <stdexcept>

static int runAuto(mocap::Camera& cam, const mocap::Settings baseline,
                   const mocap::Detector& detector, mocap::Coverage& coverage,
                   mocap::Session* session, const mocap::AutoConfig& cfg,
                   const std::string& statusPath, int serial);

// Whole-string integer, or nullopt.
static std::optional<int> parseInt(const char* s) {
    char* end = nullptr;
    errno = 0;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0' || errno || v < INT32_MIN || v > INT32_MAX) return std::nullopt;
    return int(v);
}

static std::string settingsJson(const mocap::Settings& s) {
    return cv::format("{\"exposure\": %d, \"gain\": %d, \"intensity\": %d, "
                      "\"frame_rate\": %d, \"ir_filter\": %s}",
                      s.exposure, s.gain, s.intensity, s.frameRate,
                      s.irFilter ? "true" : "false");
}

// "  exposure: asked 8133, camera reports 7133\n" per differing setting;
// empty if the readback matches the request.
static std::string settingsMismatch(const mocap::Settings& asked, const mocap::Settings& got) {
    std::string m;
    if (got.exposure != asked.exposure)
        m += cv::format("  exposure: asked %d, camera reports %d\n", asked.exposure, got.exposure);
    if (got.gain != asked.gain)
        m += cv::format("  gain: asked %d, camera reports %d\n", asked.gain, got.gain);
    if (got.intensity != asked.intensity)
        m += cv::format("  intensity: asked %d, camera reports %d\n", asked.intensity, got.intensity);
    if (got.frameRate != asked.frameRate)
        m += cv::format("  frame rate: asked %d, camera reports %d\n", asked.frameRate, got.frameRate);
    return m;
}

static std::string jsonString(const std::string& v) {
    std::string out = "\"";
    for (char c : v) {
        if (c == '"' || c == '\\') out += '\\';
        out += c == '\n' ? ' ' : c;
    }
    return out + "\"";
}

int main(int argc, char** argv) {
    int serial = 33661;
    std::string boardPath = mocap::kBoardConfig;
    std::string autoPath = mocap::kAutoConfig;
    std::string statusPath;
    bool autoMode = false, dryRun = false;
    mocap::Settings settings;
    bool fromFlags = false;
    // --name N with lo <= N <= hi into *out; false (message printed) if not.
    auto intFlag = [&](int& i, const char* name, int lo, int hi, int* out) {
        if (i + 1 >= argc) { fprintf(stderr, "%s needs a value\n", name); return false; }
        auto v = parseInt(argv[++i]);
        if (!v || *v < lo || *v > hi) {
            fprintf(stderr, "%s %s: need an integer %d..%d\n", name, argv[i], lo, hi);
            return false;
        }
        *out = *v;
        fromFlags = true;
        return true;
    };
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--gain")) {
            if (!intFlag(i, "--gain", 0, 7, &settings.gain)) return 1;
        } else if (!strcmp(argv[i], "--intensity")) {
            if (!intFlag(i, "--intensity", 0, 15, &settings.intensity)) return 1;
        } else if (!strcmp(argv[i], "--exposure")) {
            // Ceiling at the start-up frame rate (rule 7); rechecked against
            // the camera once it is streaming.
            int ceiling = 1000000 / settings.frameRate - 200;
            if (!intFlag(i, "--exposure", 1, ceiling, &settings.exposure)) {
                fprintf(stderr, "  (ceiling at %d fps: floor(1e6/fps) - 200 = %d us)\n",
                        settings.frameRate, ceiling);
                return 1;
            }
        }
        else if (!strcmp(argv[i], "--board") && i + 1 < argc) boardPath = argv[++i];
        else if (!strcmp(argv[i], "--auto-config") && i + 1 < argc) autoPath = argv[++i];
        else if (!strcmp(argv[i], "--auto")) autoMode = true;
        else if (!strcmp(argv[i], "--auto-dry")) autoMode = dryRun = true;
        else if (!strcmp(argv[i], "--status-file") && i + 1 < argc) statusPath = argv[++i];
        else serial = std::stoi(argv[i]);
    }

    if (!statusPath.empty() && !autoMode) {
        fprintf(stderr, "--status-file needs --auto or --auto-dry\n");
        return 1;
    }

    mocap::BoardSpec board;
    try {
        board = mocap::loadBoardSpec(boardPath);
    } catch (const std::exception& e) {
        fprintf(stderr, "%s\n", e.what());
        return 1;
    }
    printf("%s\n\n", board.describe().c_str());
    mocap::Detector detector(board);

    mocap::AutoConfig autoCfg;
    if (autoMode) {
        try {
            autoCfg = mocap::loadAutoConfig(autoPath);
        } catch (const std::exception& e) {
            fprintf(stderr, "%s\n", e.what());
            return 1;
        }
        printf("auto%s: %s  max_fit %.2f px  still median < %.2f max < %.2f px  interval %d ms"
               "  move %.1f px\n"
               "  flat < %.0f deg  near >= %.1f px/sq  far < %.1f px/sq  min_per_bin %d\n\n",
               dryRun ? " DRY RUN" : "", autoCfg.source.c_str(), autoCfg.maxFitPx,
               autoCfg.stillPx, autoCfg.stillMaxPx, autoCfg.minIntervalMs, autoCfg.minMovePx,
               autoCfg.flatDeg,
               autoCfg.nearSquarePx, autoCfg.farSquarePx, autoCfg.minPerBin);
    }

    mocap::Camera cam(serial);
    cam.start(settings);

    // The camera's own limits once the frame rate is set, and what it
    // reports back. An exposure outside the camera's range is refused; a
    // readback that differs from the request is printed and recorded (the
    // SDK may not report a new value at once), and autocapture's gate 4
    // takes its reference from the first evaluated frame's readback.
    const mocap::Settings readback = cam.actual();
    const int minExp = cam.minExposure(), maxExp = cam.maxExposure();
    printf("settings  exposure %d us (camera %d..%d)  gain %d  intensity %d  frame rate %d  "
           "IR filter %s   [%s]\n",
           readback.exposure, minExp, maxExp, readback.gain, readback.intensity,
           readback.frameRate, readback.irFilter ? "in" : "out",
           fromFlags ? "from flags; read back" : "defaults; read back");
    if (settings.exposure < minExp || settings.exposure > maxExp) {
        fprintf(stderr, "--exposure %d is outside the camera's %d..%d us; nothing captured\n",
                settings.exposure, minExp, maxExp);
        return 1;
    }
    const std::string mismatch = settingsMismatch(settings, readback);
    if (!mismatch.empty())
        printf("WARNING   read back differs from the request:\n%s", mismatch.c_str());

    cv::Size imageSize(cam.width(), cam.height());
    mocap::Coverage coverage(imageSize);

    // A dry run must write nothing, and the Session constructor creates
    // the folder.
    std::string root = std::string(getenv("HOME")) + "/panoptes-data/sessions";
    std::optional<mocap::Session> session;
    if (!dryRun) {
        session.emplace(root, serial, board);
        session->setExtra("baseline_settings", cv::format(
            "{\"source\": \"%s\", \"requested\": %s, \"readback\": %s, "
            "\"readback_matches\": %s, \"exposure_range\": [%d, %d]}",
            fromFlags ? "flags" : "defaults", settingsJson(settings).c_str(),
            settingsJson(readback).c_str(), mismatch.empty() ? "true" : "false",
            minExp, maxExp));
    }

    printf("%s  %dx%d\n", cam.name().c_str(), cam.width(), cam.height());
    if (session) printf("session: %s\n\n", session->path().c_str());
    else         printf("session: none (dry run, nothing is written)\n\n");

    if (autoMode)
        return runAuto(cam, settings, detector, coverage,
                       session ? &*session : nullptr, autoCfg, statusPath, serial);

    printf("  space  capture      u  undo last\n");
    printf("  + -    exposure     [ ]  gain\n");
    printf("  q      quit\n\n");

    while (true) {
        cv::Mat gray = cam.grab();
        if (gray.empty()) { cv::waitKey(5); continue; }

        mocap::Detection d = detector.detect(gray);
        cv::Mat display = detector.annotate(gray, d);
        coverage.draw(display);

        mocap::Settings actual = cam.actual();

        char line1[200], line2[200];
        snprintf(line1, sizeof(line1),
                 "corners %2zu/%d   captures %d   empty cells %d/%d",
                 d.corners.size(), board.totalCorners(),
                 coverage.captureCount(),
                 coverage.emptyCells(), coverage.totalCells());
        snprintf(line2, sizeof(line2),
                 "exp %d/%d  gain %d  tilted %.0f%%",
                 actual.exposure, cam.maxExposure(), actual.gain,
                 coverage.tiltedFraction() * 100.0);

        // Green once coverage is complete - the actual stopping signal.
        cv::Scalar colour = coverage.emptyCells() == 0
                          ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255);

        cv::putText(display, line1, cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, colour, 2);
        cv::putText(display, line2, cv::Point(20, 75),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 200), 2);

        cv::Mat small;
        cv::resize(display, small, cv::Size(), 0.7, 0.7);
        cv::imshow("intrinsics capture", small);

        int key = cv::waitKey(1);
        if (key == 'q') break;

        if (key == ' ') {
            // Only accept complete boards. A partial view still carries
            // valid corners, but mixing partial and full captures makes
            // per-view error harder to read.
            if (!d.complete(board)) {
                printf("skipped: only %zu/%d corners\n",
                       d.corners.size(), board.totalCorners());
            } else {
                auto pose = coverage.add(d, board);
                session->record(gray, d, actual);
                printf("captured %d  tilt %.0f deg  dist %.2f m  empty cells %d\n",
                       session->count(), pose.tiltDeg, pose.distanceMM / 1000.0,
                       coverage.emptyCells());
            }
        }

        if (key == 'u' && session->count() > 0) {
            coverage.removeLast();
            session->removeLast();
            printf("undid last, %d remain\n", session->count());
        }

        if (key == '+' || key == '=') {
            settings.exposure = std::min(settings.exposure + 1000, cam.maxExposure());
            cam.apply(settings);
        }
        if (key == '-') {
            settings.exposure = std::max(settings.exposure - 1000, 10);
            cam.apply(settings);
        }
        if (key == ']') { settings.gain = std::min(settings.gain + 1, 7); cam.apply(settings); }
        if (key == '[') { settings.gain = std::max(settings.gain - 1, 0); cam.apply(settings); }
    }

    session->flush();
    printf("\n%d frames in %s\n", session->count(), session->path().c_str());
    printf("empty cells: %d/%d   tilted: %.0f%%\n",
           coverage.emptyCells(), coverage.totalCells(),
           coverage.tiltedFraction() * 100.0);
    return 0;
}
// ---------------------------------------------------------------- auto

// Pose-bin counts as a small table, bottom right: rows are distance bands,
// columns tilt directions. Red under min_per_bin, green at or above it; the
// current frame's bin is outlined.
static void drawBins(cv::Mat& display, const mocap::AutoCapture& ac,
                     const mocap::PoseBin& current) {
    const int cw = 70, ch = 34, labelW = 70;
    const int x0 = display.cols - labelW - cw * mocap::kTiltBins - 20;
    const int y0 = display.rows - ch * (mocap::kBands + 1) - 20;
    const auto font = cv::FONT_HERSHEY_SIMPLEX;
    const cv::Scalar grey(200, 200, 200);

    cv::rectangle(display, cv::Rect(x0 - 8, y0 - 8,
                                    labelW + cw * mocap::kTiltBins + 16,
                                    ch * (mocap::kBands + 1) + 16),
                  cv::Scalar(30, 30, 30), cv::FILLED);
    for (int t = 0; t < mocap::kTiltBins; ++t)
        cv::putText(display, mocap::tiltBinName(t),
                    cv::Point(x0 + labelW + t * cw + 8, y0 + ch - 10), font, 0.6, grey, 1);
    for (int b = 0; b < mocap::kBands; ++b) {
        int y = y0 + (b + 1) * ch;
        cv::putText(display, mocap::bandName(b), cv::Point(x0, y + ch - 10), font, 0.6, grey, 1);
        for (int t = 0; t < mocap::kTiltBins; ++t) {
            int n = ac.binCount(t, b);
            bool done = n >= ac.config().minPerBin;
            cv::Rect cell(x0 + labelW + t * cw, y, cw - 4, ch - 4);
            cv::rectangle(display, cell, done ? cv::Scalar(0, 120, 0) : cv::Scalar(0, 0, 140),
                          cv::FILLED);
            if (current.valid && current.tilt == t && current.band == b)
                cv::rectangle(display, cell, cv::Scalar(255, 255, 255), 3);
            cv::putText(display, std::to_string(n), cv::Point(cell.x + 24, cell.y + ch - 12),
                        font, 0.7, cv::Scalar(255, 255, 255), 2);
        }
    }
}

static void bigText(cv::Mat& display, const std::string& text, int y, cv::Scalar colour) {
    int base = 0;
    cv::Size sz = cv::getTextSize(text, cv::FONT_HERSHEY_DUPLEX, 3.0, 6, &base);
    cv::putText(display, text, cv::Point((display.cols - sz.width) / 2, y),
                cv::FONT_HERSHEY_DUPLEX, 3.0, cv::Scalar(0, 0, 0), 14);
    cv::putText(display, text, cv::Point((display.cols - sz.width) / 2, y),
                cv::FONT_HERSHEY_DUPLEX, 3.0, colour, 6);
}

// Prints to stderr if the main loop stops turning: a stall where grab()
// keeps returning nothing is handled in the loop, but a call that never
// returns (SDK, imwrite, imshow) would otherwise be silent.
class LoopWatchdog {
public:
    explicit LoopWatchdog(double afterS) : afterMs_((long long)(afterS * 1000)) {
        beat("start");
        thread_ = std::thread([this] { run(); });
    }
    ~LoopWatchdog() { stop_ = true; thread_.join(); }
    void beat(const char* phase) { phase_ = phase; beatMs_ = nowMs(); }
    void phase(const char* p) { phase_ = p; }
private:
    static long long nowMs() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void run() {
        long long warnedAt = 0;
        while (!stop_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
            long long since = nowMs() - beatMs_;
            if (since >= afterMs_ && nowMs() - warnedAt >= afterMs_) {
                fprintf(stderr, "main loop blocked for %.0f s in %s (%s)\n", since / 1000.0,
                        phase_.load(), mocap::utcNowIso().c_str());
                warnedAt = nowMs();
            } else if (since < afterMs_) {
                warnedAt = 0;
            }
        }
    }
    long long afterMs_;
    std::atomic<long long> beatMs_{0};
    std::atomic<const char*> phase_{"start"};
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

static constexpr double kStallReportS  = 1.0;   // first "no frames" line
static constexpr double kStallRepeatS  = 5.0;   // then every this often
static constexpr double kRestartAfterS = 5.0;   // first restart, then every kRestartAfterS
static constexpr int    kVerifyFrames  = 10;    // frames to wait for a matching readback
static constexpr double kStallStatusS  = 0.25;  // status file rewrite interval in a stall

static int runAuto(mocap::Camera& cam, const mocap::Settings baseline,
                   const mocap::Detector& detector, mocap::Coverage& coverage,
                   mocap::Session* session, const mocap::AutoConfig& cfg,
                   const std::string& statusPath, int serial) {
    using Clock = std::chrono::steady_clock;
    auto secondsSince = [](Clock::time_point t) {
        return std::chrono::duration<double>(Clock::now() - t).count();
    };
    mocap::Settings settings = baseline;   // what the keys change
    const mocap::BoardSpec& board = detector.spec();
    const bool dry = session == nullptr;
    mocap::AutoCapture ac(cfg, detector, cv::Size(cam.width(), cam.height()));
    if (!statusPath.empty()) printf("status file: %s (rewritten every frame)\n", statusPath.c_str());
    bool statusFailed = false;

    printf("  move the board SLOWLY; hold each pose until it is taken\n");
    printf("  + -    exposure     [ ]  gain   (any change pauses accepting)\n");
    printf("  q      quit\n\n");

    auto writeSummary = [&](const std::string& ended) {
        if (!session) return;
        session->setExtra("autocapture", ac.summaryJson("  "));
        if (!ended.empty()) session->setExtra("autocapture_end", "\"" + ended + "\"");
    };
    writeSummary("");

    const auto t0 = Clock::now();
    std::string ended = "operator quit (q)";
    bool settingsOff = false;
    std::string lastAccept;
    double lastAcceptMs = -1e9;

    // The last frame's results: written again, marked stale, during a stall.
    mocap::Detection d;
    mocap::Decision dec;
    mocap::Settings actual = cam.actual();
    cv::Mat lastSmall;

    auto writeStatus = [&](const mocap::StatusMeta& extra) {
        if (statusPath.empty()) return;
        mocap::StatusMeta meta = extra;
        meta.frame = ac.evaluated();
        meta.tMs = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        meta.utc = mocap::utcNowIso();
        meta.serial = serial;
        meta.dryRun = dry;
        if (session) meta.sessionPath = session->path();
        try {
            mocap::writeFileAtomic(statusPath, mocap::captureStatusJson(
                meta, actual, d, board, dec, ac, coverage));
            if (statusFailed) printf("status file: writing again\n");
            statusFailed = false;
        } catch (const std::exception& e) {
            // Once per failure run, not every frame. A reader sees the
            // file go stale and must stop trusting it.
            if (!statusFailed) fprintf(stderr, "%s\n", e.what());
            statusFailed = true;
        }
    };

    // ---- stall state. A stall is >= kStallReportS with no frame.
    auto lastFrameAt = Clock::now();
    std::string lastFrameUtc = mocap::utcNowIso();
    bool stalled = false;
    double nextReportS = kStallReportS;
    int restarts = 0;
    Clock::time_point lastRestartAt;
    Clock::time_point lastStallStatusAt;
    std::string stallState;          // stateLine() at the last check
    std::string stallFirstState;     // stateLine() when the stall was first reported
    std::string attemptsJson;        // this stall's restart attempts
    std::string stallsJson;          // finished stall records, for session.json
    std::string pendingStall;        // recovered after a restart, readback not yet verified
    int verifyLeft = 0, verifyUsed = 0;
    int stallCount = 0;

    auto saveStalls = [&] {
        if (session) session->setExtra("stalls", "[" + stallsJson + "\n  ]");
    };
    auto addStall = [&](const std::string& record) {
        stallsJson += (stallsJson.empty() ? "\n    " : ",\n    ") + record;
        saveStalls();
    };
    // The stall's record up to its end, without the closing brace.
    auto stallRecord = [&](double gapS, bool recovered) {
        return cv::format(
            "{\"last_frame_utc\": \"%s\", \"end_utc\": \"%s\", \"gap_s\": %.2f, "
            "\"recovered\": %s, \"frames_before\": %d, \"accepted_before\": %d, "
            "\"camera_state\": %s, \"restart_attempts\": [%s]",
            lastFrameUtc.c_str(), mocap::utcNowIso().c_str(), gapS,
            recovered ? "true" : "false", ac.evaluated(), coverage.captureCount(),
            jsonString(stallFirstState).c_str(), attemptsJson.c_str());
    };
    auto finishVerify = [&](bool matches, const std::string& note) {
        addStall(pendingStall + cv::format(
            ", \"verify\": {\"asked\": %s, \"readback\": %s, \"readback_matches\": %s, "
            "\"frames\": %d, \"note\": %s}}",
            settingsJson(baseline).c_str(), settingsJson(actual).c_str(),
            matches ? "true" : "false", verifyUsed, jsonString(note).c_str()));
        pendingStall.clear();
        verifyLeft = 0;
    };

    LoopWatchdog watchdog(5.0);

    while (true) {
        watchdog.beat("grab");
        cv::Mat gray = cam.grab();
        if (gray.empty()) {
            const double gapS = secondsSince(lastFrameAt);
            if (gapS >= kStallReportS) {
                if (!stalled) {
                    stalled = true;
                    nextReportS = kStallReportS;
                    restarts = 0;
                    attemptsJson.clear();
                    stallFirstState.clear();
                    ++stallCount;
                    // A restart that never produced its verifying frames
                    // ends in this new stall.
                    if (!pendingStall.empty())
                        finishVerify(false, "stalled again before the readback was verified");
                }
                if (gapS >= nextReportS) {
                    watchdog.phase("stateLine");
                    stallState = cam.stateLine();
                    if (stallFirstState.empty()) stallFirstState = stallState;
                    printf("no frames from %d for %.0f s  (%s)  [%s]\n", serial, gapS,
                           stallState.c_str(), mocap::utcNowIso().c_str());
                    fflush(stdout);
                    nextReportS += kStallRepeatS;
                }
                if (gapS >= kRestartAfterS &&
                    (restarts == 0 || secondsSince(lastRestartAt) >= kRestartAfterS)) {
                    ++restarts;
                    lastRestartAt = Clock::now();
                    const std::string before = cam.stateLine();
                    printf("restarting camera %d (attempt %d, %.0f s without frames)\n",
                           serial, restarts, gapS);
                    fflush(stdout);
                    watchdog.phase("Camera::restart");
                    std::string why;
                    const bool sent = cam.restart(baseline, &why);
                    stallState = cam.stateLine();
                    printf("  %s  (%s)\n",
                           sent ? "stopped, started, every baseline setting sent; waiting for frames"
                                : ("not restarted: " + why).c_str(),
                           stallState.c_str());
                    fflush(stdout);
                    settings = baseline;
                    attemptsJson += cv::format(
                        "%s{\"utc\": \"%s\", \"after_s\": %.1f, \"sent\": %s, \"why\": %s, "
                        "\"state_before\": %s, \"state_after\": %s}",
                        attemptsJson.empty() ? "" : ", ", mocap::utcNowIso().c_str(), gapS,
                        sent ? "true" : "false", jsonString(why).c_str(),
                        jsonString(before).c_str(), jsonString(stallState).c_str());
                }
                if (!statusPath.empty() && secondsSince(lastStallStatusAt) >= kStallStatusS) {
                    watchdog.phase("stateLine");
                    stallState = cam.stateLine();
                    mocap::StatusMeta m;
                    m.stalled = true;
                    m.stalledS = secondsSince(lastFrameAt);
                    m.restarts = restarts;
                    m.cameraState = stallState;
                    watchdog.phase("status (stalled)");
                    writeStatus(m);
                    lastStallStatusAt = Clock::now();
                }
                if (!lastSmall.empty()) {
                    cv::Mat shown = lastSmall.clone();
                    bigText(shown, cv::format("NO FRAMES %.0f s", gapS), shown.rows / 2,
                            cv::Scalar(0, 0, 255));
                    watchdog.phase("display (stalled)");
                    cv::imshow("intrinsics capture", shown);
                }
            }
            watchdog.phase("waitKey");
            if (cv::waitKey(5) == 'q') {
                if (stalled) ended = "operator quit (q) during a stall";
                break;
            }
            continue;
        }
        if (stalled) {
            const double gapS = secondsSince(lastFrameAt);
            printf("frames from %d again after %.1f s (%d restart attempt%s)  [%s]\n", serial, gapS,
                   restarts, restarts == 1 ? "" : "s", mocap::utcNowIso().c_str());
            fflush(stdout);
            // The previous frame is from before the gap; it cannot vouch
            // for this one being still.
            ac.forgetPrevious();
            if (restarts > 0) {
                pendingStall = stallRecord(gapS, true);
                verifyLeft = kVerifyFrames;
                verifyUsed = 0;
            } else {
                addStall(stallRecord(gapS, true) + ", \"verify\": null}");
            }
            stalled = false;
        }
        lastFrameAt = Clock::now();
        lastFrameUtc = mocap::utcNowIso();
        double tMs = std::chrono::duration<double, std::milli>(lastFrameAt - t0).count();

        watchdog.phase("detect");
        d = detector.detect(gray);
        actual = cam.actual();

        // After a restart: the readback must come back to the baseline. The
        // SDK may not report a new value at once, so allow a few frames.
        if (verifyLeft > 0) {
            ++verifyUsed;
            const std::string mm = settingsMismatch(baseline, actual);
            if (mm.empty()) {
                printf("restart verified: readback matches the baseline settings (frame %d after)\n",
                       verifyUsed);
                finishVerify(true, "");
            } else if (--verifyLeft == 0) {
                printf("WARNING   after the restart the readback differs from the baseline "
                       "(%d frames); gate 4 refuses every frame until it matches:\n%s",
                       verifyUsed, mm.c_str());
                finishVerify(false, mm);
            }
            fflush(stdout);
        }

        watchdog.phase("evaluate");
        dec = ac.evaluate(d, actual, tMs, coverage);

        // Say so once on the way out of the reference settings and once on
        // the way back, not every frame.
        bool off = !ac.settingsMatch(actual);
        if (off != settingsOff) {
            if (off) printf("SETTINGS CHANGED (%s): no frame is accepted until they are restored\n",
                            ac.settingsDiff(actual).c_str());
            else     printf("settings restored; accepting again\n");
            settingsOff = off;
        }

        if (dec.accepted()) {
            ac.commit(dec, d, coverage);
            if (session) {
                watchdog.phase("record");
                session->record(gray, d, actual, dec.jsonFields());
                writeSummary("");
            }
            lastAccept = dec.acceptReason();
            lastAcceptMs = tMs;
            printf("\a%s %3d  %-28s motion median %.3f max %.3f px  fit %.2f px  %4.1f px/sq  "
                   "tilt %2.0f (x %+3.0f y %+3.0f)  empty cells %d  bins short %d\n",
                   dry ? "would accept" : "accepted", coverage.captureCount(),
                   lastAccept.c_str(), dec.motionMedPx, dec.motionMaxPx, dec.fitPx,
                   dec.bin.squarePx,
                   dec.bin.tiltDeg, dec.bin.tiltXDeg, dec.bin.tiltYDeg,
                   coverage.emptyCells(), ac.binsShort());
            fflush(stdout);
        } else if (dry) {
            printf("  %8.0f ms  %-50s motion median %s max %s  size %s  bin %s\n", tMs,
                   dec.describe(cfg).c_str(),
                   dec.motionMedPx < 0 ? "  -  " : cv::format("%.3f", dec.motionMedPx).c_str(),
                   dec.motionMaxPx < 0 ? "  -  " : cv::format("%.3f", dec.motionMaxPx).c_str(),
                   dec.bin.valid ? cv::format("%.1f", dec.bin.squarePx).c_str() : "-",
                   dec.bin.name().c_str());
        }

        watchdog.phase("status");
        writeStatus(mocap::StatusMeta{});

        // ---- display: manual mode's overlay, plus the auto status
        watchdog.phase("display");
        cv::Mat display = detector.annotate(gray, d);
        coverage.draw(display);

        char line1[200], line2[200], line3[200], line4[240];
        snprintf(line1, sizeof(line1),
                 "corners %2zu/%d   captures %d   empty cells %d/%d",
                 d.corners.size(), board.totalCorners(), coverage.captureCount(),
                 coverage.emptyCells(), coverage.totalCells());
        snprintf(line2, sizeof(line2),
                 "exp %d/%d  gain %d  tilted %.0f%%   bins short %d/%d%s",
                 actual.exposure, cam.maxExposure(), actual.gain,
                 coverage.tiltedFraction() * 100.0, ac.binsShort(),
                 mocap::kTiltBins * mocap::kBands, dry ? "   DRY RUN: nothing written" : "");
        snprintf(line3, sizeof(line3), "%s %s", dec.accepted() ? "" : "reject:",
                 dec.describe(cfg).c_str());
        snprintf(line4, sizeof(line4),
                 "motion median %s < %.2f  max %s < %.2f px   size %s px/sq   bin %s",
                 dec.motionMedPx < 0 ? "-" : cv::format("%.3f", dec.motionMedPx).c_str(),
                 cfg.stillPx,
                 dec.motionMaxPx < 0 ? "-" : cv::format("%.3f", dec.motionMaxPx).c_str(),
                 cfg.stillMaxPx,
                 dec.bin.valid ? cv::format("%.1f", dec.bin.squarePx).c_str() : "-",
                 dec.bin.name().c_str());

        cv::Scalar colour = coverage.emptyCells() == 0
                          ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255);
        cv::putText(display, line1, cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, colour, 2);
        cv::putText(display, line2, cv::Point(20, 75),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 200), 2);
        cv::putText(display, line3, cv::Point(20, 115), cv::FONT_HERSHEY_SIMPLEX, 0.9,
                    dec.accepted() ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 140, 255), 2);
        cv::putText(display, line4, cv::Point(20, 150),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(200, 200, 200), 2);
        if (tMs - lastAcceptMs < 1500)
            cv::putText(display, "ACCEPT #" + std::to_string(coverage.captureCount()) +
                        "  " + lastAccept, cv::Point(20, 190),
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

        if (dec.verdict == mocap::Verdict::Moving || dec.verdict == mocap::Verdict::NoReference)
            bigText(display, "HOLD STILL", display.rows / 2, cv::Scalar(0, 220, 255));
        if (off)
            bigText(display, "SETTINGS CHANGED", display.rows / 2 + 110, cv::Scalar(0, 0, 255));

        drawBins(display, ac, dec.bin);

        cv::resize(display, lastSmall, cv::Size(), 0.7, 0.7);
        cv::imshow("intrinsics capture", lastSmall);

        if (ac.complete(coverage)) {
            ended = "complete: 0 empty cells and every pose bin has >= " +
                    std::to_string(cfg.minPerBin) + " frames";
            cv::waitKey(1);
            break;
        }

        watchdog.phase("waitKey");
        int key = cv::waitKey(1);
        if (key == 'q') break;
        if (key == ' ' || key == 'u')
            printf("auto mode: space and u are disabled; q to quit\n");

        // Same keys as manual mode. Any change trips gate 4 until undone.
        if (key == '+' || key == '=') {
            settings.exposure = std::min(settings.exposure + 1000, cam.maxExposure());
            cam.apply(settings);
        }
        if (key == '-') {
            settings.exposure = std::max(settings.exposure - 1000, 10);
            cam.apply(settings);
        }
        if (key == ']') { settings.gain = std::min(settings.gain + 1, 7); cam.apply(settings); }
        if (key == '[') { settings.gain = std::max(settings.gain - 1, 0); cam.apply(settings); }
    }

    // A stall or a verification still open at the end is recorded as it stands.
    if (!pendingStall.empty()) finishVerify(false, "capture ended before the readback was verified");
    if (stalled) addStall(stallRecord(secondsSince(lastFrameAt), false) + ", \"verify\": null}");
    writeSummary(ended);

    printf("\nended: %s\n", ended.c_str());
    if (session) printf("%d frames in %s\n", session->count(), session->path().c_str());
    else         printf("%d frames would have been kept (dry run, nothing written)\n",
                        coverage.captureCount());
    printf("empty cells: %d/%d   tilted: %.0f%%   bins short of %d: %d/%d\n",
           coverage.emptyCells(), coverage.totalCells(),
           coverage.tiltedFraction() * 100.0, cfg.minPerBin, ac.binsShort(),
           mocap::kTiltBins * mocap::kBands);
    for (int b = 0; b < mocap::kBands; ++b) {
        printf("  %-5s", mocap::bandName(b));
        for (int t = 0; t < mocap::kTiltBins; ++t)
            printf("  %s %d", mocap::tiltBinName(t), ac.binCount(t, b));
        printf("\n");
    }
    if (stallCount) printf("stalls: %d (each in session.json \"stalls\")\n", stallCount);
    printf("%d frames evaluated:", ac.evaluated());
    for (int v = 0; v < mocap::kVerdictCount; ++v)
        printf("  %s %d", mocap::verdictName((mocap::Verdict)v), ac.tally((mocap::Verdict)v));
    printf("\n");
    return 0;
}
