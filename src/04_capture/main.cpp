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
#include <optional>
#include <stdexcept>

static int runAuto(mocap::Camera& cam, mocap::Settings settings,
                   const mocap::Detector& detector, mocap::Coverage& coverage,
                   mocap::Session* session, const mocap::AutoConfig& cfg,
                   const std::string& statusPath, int serial);

int main(int argc, char** argv) {
    int serial = 33661;
    std::string boardPath = mocap::kBoardConfig;
    std::string autoPath = mocap::kAutoConfig;
    std::string statusPath;
    bool autoMode = false, dryRun = false;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--board") && i + 1 < argc) boardPath = argv[++i];
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
    mocap::Settings settings;
    cam.start(settings);

    cv::Size imageSize(cam.width(), cam.height());
    mocap::Coverage coverage(imageSize);

    // A dry run must write nothing, and the Session constructor creates
    // the folder.
    std::string root = std::string(getenv("HOME")) + "/panoptes-data/sessions";
    std::optional<mocap::Session> session;
    if (!dryRun) session.emplace(root, serial, board);

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

static int runAuto(mocap::Camera& cam, mocap::Settings settings,
                   const mocap::Detector& detector, mocap::Coverage& coverage,
                   mocap::Session* session, const mocap::AutoConfig& cfg,
                   const std::string& statusPath, int serial) {
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

    const auto t0 = std::chrono::steady_clock::now();
    std::string ended = "operator quit (q)";
    bool settingsOff = false;
    std::string lastAccept;
    double lastAcceptMs = -1e9;

    while (true) {
        cv::Mat gray = cam.grab();
        if (gray.empty()) {
            if (cv::waitKey(5) == 'q') break;
            continue;
        }
        double tMs = std::chrono::duration<double, std::milli>(
                         std::chrono::steady_clock::now() - t0).count();

        mocap::Detection d = detector.detect(gray);
        mocap::Settings actual = cam.actual();
        mocap::Decision dec = ac.evaluate(d, actual, tMs, coverage);

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

        if (!statusPath.empty()) {
            mocap::StatusMeta meta;
            meta.frame = ac.evaluated();
            meta.tMs = tMs;
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
        }

        // ---- display: manual mode's overlay, plus the auto status
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

        cv::Mat small;
        cv::resize(display, small, cv::Size(), 0.7, 0.7);
        cv::imshow("intrinsics capture", small);

        if (ac.complete(coverage)) {
            ended = "complete: 0 empty cells and every pose bin has >= " +
                    std::to_string(cfg.minPerBin) + " frames";
            cv::waitKey(1);
            break;
        }

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
    printf("%d frames evaluated:", ac.evaluated());
    for (int v = 0; v < mocap::kVerdictCount; ++v)
        printf("  %s %d", mocap::verdictName((mocap::Verdict)v), ac.tally((mocap::Verdict)v));
    printf("\n");
    return 0;
}
