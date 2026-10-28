// Unit tests for lib/autocapture: the accept gates on synthetic corner sets.
//
// Corners are the real board (config/charuco_board.yaml) projected through
// a pinhole camera at the focal length the pose bins assume, plus small
// seeded noise. No camera, no images.
//
// Run from the repo root:  tests/build/test_autocapture [board.yaml]
// Exit 0 only if every check passes.

#include "../lib/autocapture.h"
#include "../lib/session.h"

#include <opencv2/calib3d.hpp>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unistd.h>

using namespace mocap;
namespace fs = std::filesystem;

static int passed = 0, failed = 0;

static void check(bool ok, const std::string& what) {
    printf("  %s  %s\n", ok ? "PASS" : "FAIL", what.c_str());
    (ok ? passed : failed)++;
}

static const cv::Size kImage(1280, 1024);
static const double kFocal = 780.0;

// The board's interior corners projected from a pose: tilted rxDeg about
// the camera x axis and ryDeg about y, its centre at image point `at` and
// distMM from the camera. Noise is seeded, so every run sees the same set.
static Detection project(const BoardSpec& b, double rxDeg, double ryDeg, double distMM,
                         cv::Point2d at = {640, 512}, double noisePx = 0.0, int seed = 1) {
    const int cx = b.squaresX - 1, cy = b.squaresY - 1;
    std::vector<cv::Point3d> obj;
    for (int id = 0; id < cx * cy; ++id)
        obj.emplace_back((id % cx + 1) * b.squareMM, (id / cx + 1) * b.squareMM, 0);
    cv::Vec3d centre((cx + 1) / 2.0 * b.squareMM, (cy + 1) / 2.0 * b.squareMM, 0);

    double rx = rxDeg * CV_PI / 180, ry = ryDeg * CV_PI / 180;
    cv::Matx33d Rx(1, 0, 0, 0, std::cos(rx), -std::sin(rx), 0, std::sin(rx), std::cos(rx));
    cv::Matx33d Ry(std::cos(ry), 0, std::sin(ry), 0, 1, 0, -std::sin(ry), 0, std::cos(ry));
    cv::Matx33d R = Rx * Ry;
    cv::Vec3d target((at.x - kImage.width / 2.0) / kFocal * distMM,
                     (at.y - kImage.height / 2.0) / kFocal * distMM, distMM);
    cv::Vec3d t = target - R * centre;

    cv::Mat rvec;
    cv::Rodrigues(cv::Mat(R), rvec);
    cv::Matx33d K(kFocal, 0, kImage.width / 2.0, 0, kFocal, kImage.height / 2.0, 0, 0, 1);
    std::vector<cv::Point2d> img;
    cv::projectPoints(obj, rvec, cv::Mat(t), cv::Mat(K), cv::Mat(), img);

    cv::RNG rng(seed);
    Detection d;
    for (int id = 0; id < cx * cy; ++id) {
        d.cornerIds.push_back(id);
        d.corners.emplace_back(float(img[id].x + rng.gaussian(noisePx)),
                               float(img[id].y + rng.gaussian(noisePx)));
    }
    return d;
}

static Detection shifted(Detection d, float dx, float dy) {
    for (auto& p : d.corners) p += cv::Point2f(dx, dy);
    return d;
}

static AutoConfig testConfig() {
    AutoConfig c;
    c.source = "test";
    c.maxFitPx = 5.0;
    c.stillPx = 0.3;
    c.minIntervalMs = 300;
    c.minMovePx = 20.0;
    c.flatDeg = 20.0;
    c.nearSquarePx = 40.0;
    c.farSquarePx = 35.0;
    c.approxFocalPx = kFocal;
    c.minPerBin = 2;
    return c;
}

int main(int argc, char** argv) {
    BoardSpec board = loadBoardSpec(argc > 1 ? argv[1] : kBoardConfig);
    Detector detector(board);
    const Settings S;                       // the operating point
    printf("%s\n\n", board.describe().c_str());

    // ------------------------------------------------------------ config
    printf("config\n");
    {
        AutoConfig c = loadAutoConfig(kAutoConfig);
        check(c.stillPx > 0 && c.minPerBin >= 1 && c.farSquarePx <= c.nearSquarePx,
              "config/autocapture.yaml loads and is self-consistent");

        fs::path dir = fs::temp_directory_path() / ("panoptes_test_" + std::to_string(getpid()));
        fs::create_directories(dir);
        std::ofstream(dir / "bad.yaml") << "%YAML:1.0\n---\nmax_fit_px: 5.0\nstill_px: 0.3\n";
        bool threw = false;
        std::string msg;
        try { loadAutoConfig((dir / "bad.yaml").string()); }
        catch (const std::runtime_error& e) { threw = true; msg = e.what(); }
        check(threw && msg.find("min_interval_ms") != std::string::npos,
              "a missing field is refused and named (" + msg + ")");
        fs::remove_all(dir);
    }

    // ------------------------------------------------------------ gate 1
    printf("\ngate 1: full board\n");
    {
        AutoCapture ac(testConfig(), detector, kImage);
        Coverage cov(kImage);
        Detection full = project(board, 0, 0, 1200, {640, 512}, 0.03);

        Detection partial = full;
        partial.corners.resize(87);
        partial.cornerIds.resize(87);
        Decision dec = ac.evaluate(partial, S, 0, cov);
        check(dec.verdict == Verdict::NotFull && dec.corners == 87, "87/88 corners -> not_full");

        Detection empty;
        check(ac.evaluate(empty, S, 100, cov).verdict == Verdict::NotFull, "no corners -> not_full");

        // Two IDs swapped: the count is full but the labels are wrong.
        Detection swapped = full;
        std::swap(swapped.corners[0], swapped.corners[87]);
        dec = ac.evaluate(swapped, S, 200, cov);
        check(dec.verdict == Verdict::BadFit && dec.fitPx > 100,
              "full count, swapped labels -> bad_fit (fit " + std::to_string(dec.fitPx) + " px)");

        dec = ac.evaluate(full, S, 300, cov);
        check(dec.fitPx >= 0 && dec.fitPx < 0.5, "a clean projection fits under 0.5 px");
    }

    // ------------------------------------------------------------ gate 2
    printf("\ngate 2: still\n");
    {
        AutoCapture ac(testConfig(), detector, kImage);
        Coverage cov(kImage);
        Detection a = project(board, 0, 0, 1200, {640, 512}, 0.03, 1);

        Decision dec = ac.evaluate(a, S, 0, cov);
        check(dec.verdict == Verdict::NoReference && dec.motionPx < 0,
              "first full frame -> no_reference");

        Detection moved = shifted(a, 0.5f, 0);
        dec = ac.evaluate(moved, S, 140, cov);
        check(dec.verdict == Verdict::Moving && std::fabs(dec.motionPx - 0.5) < 1e-3,
              "every corner 0.5 px -> moving, motion 0.500 (" + std::to_string(dec.motionPx) + ")");

        // One corner moving is enough: the gate is on the worst corner.
        Detection oneCorner = moved;
        oneCorner.corners[40].x += 0.35f;
        dec = ac.evaluate(oneCorner, S, 280, cov);
        check(dec.verdict == Verdict::Moving && dec.motionPx > 0.3,
              "one corner 0.35 px, rest still -> moving");

        // The reference is the PREVIOUS frame, so stopping is enough.
        Detection stopped = project(board, 0, 0, 1200, {640, 512}, 0.03, 2);
        stopped = shifted(stopped, 0.5f, 0);
        stopped.corners[40] = oneCorner.corners[40];
        dec = ac.evaluate(stopped, S, 420, cov);
        check(dec.verdict == Verdict::Accept && dec.motionPx < 0.3,
              "same pose again with 0.03 px noise -> accept (motion " +
              std::to_string(dec.motionPx) + ")");

        Detection partial = a;
        partial.corners.pop_back();
        partial.cornerIds.pop_back();
        ac.evaluate(partial, S, 560, cov);
        dec = ac.evaluate(a, S, 700, cov);
        check(dec.verdict == Verdict::NoReference, "full frame after a partial one -> no_reference");

        Detection bad = a;
        std::swap(bad.corners[0], bad.corners[87]);
        ac.evaluate(bad, S, 840, cov);
        dec = ac.evaluate(a, S, 980, cov);
        check(dec.verdict == Verdict::NoReference, "full frame after a bad_fit one -> no_reference");
    }

    // ------------------------------------------------------------ gate 3
    printf("\ngate 3: novel\n");
    {
        AutoConfig cfg = testConfig();
        cfg.minPerBin = 1;
        AutoCapture ac(cfg, detector, kImage);
        Coverage cov(kImage);
        Detection a = project(board, 0, 0, 1200, {640, 512}, 0.03, 1);
        Detection a2 = project(board, 0, 0, 1200, {640, 512}, 0.03, 2);

        ac.evaluate(a, S, 0, cov);
        Decision dec = ac.evaluate(a2, S, 140, cov);
        check(dec.accepted() && !dec.newCells.empty() && dec.binUnfilled,
              "first still frame fills cells and its bin: " + dec.acceptReason());
        ac.commit(dec, a2, cov);

        // Same pose 60 px across: same cells, same bin, already filled.
        Detection b = shifted(a, 60, 0), b2 = shifted(a2, 60, 0);
        ac.evaluate(b, S, 2000, cov);
        dec = ac.evaluate(b2, S, 2140, cov);
        check(dec.verdict == Verdict::Redundant && dec.newCells.empty(),
              "same bin, no new cell -> redundant (" + dec.bin.name() + ")");

        // Same bin, but reaching an empty cell at the frame corner.
        Detection c = project(board, 0, 0, 1200, {380, 330}, 0.03, 3);
        Detection c2 = project(board, 0, 0, 1200, {380, 330}, 0.03, 4);
        ac.evaluate(c, S, 4000, cov);
        dec = ac.evaluate(c2, S, 4140, cov);
        check(dec.accepted() && !dec.newCells.empty() && !dec.binUnfilled,
              "filled bin, new cell -> accept: " + dec.acceptReason());
        ac.commit(dec, c2, cov);

        // Centre again, but tilted: no new cell, new bin.
        Detection t = project(board, 30, 30, 1200, {640, 512}, 0.03, 5);
        Detection t2 = project(board, 30, 30, 1200, {640, 512}, 0.03, 6);
        ac.evaluate(t, S, 6000, cov);
        dec = ac.evaluate(t2, S, 6140, cov);
        check(dec.accepted() && dec.newCells.empty() && dec.binUnfilled,
              "no new cell, empty bin -> accept: " + dec.acceptReason());
    }
    {
        // min_per_bin 2: a bin holding 1 is still unfilled.
        AutoCapture ac(testConfig(), detector, kImage);
        Coverage cov(kImage);
        Detection a = project(board, 0, 0, 1200, {640, 512}, 0.03, 1);
        Detection a2 = project(board, 0, 0, 1200, {640, 512}, 0.03, 2);
        ac.evaluate(a, S, 0, cov);
        Decision dec = ac.evaluate(a2, S, 140, cov);
        ac.commit(dec, a2, cov);
        Detection b = shifted(a, 60, 0), b2 = shifted(a2, 60, 0);
        ac.evaluate(b, S, 2000, cov);
        dec = ac.evaluate(b2, S, 2140, cov);
        check(dec.accepted() && dec.binUnfilled && dec.newCells.empty(),
              "min_per_bin 2: second frame in a bin -> accept: " + dec.acceptReason());
        ac.commit(dec, b2, cov);
        Detection c = shifted(a, -60, 0), c2 = shifted(a2, -60, 0);
        ac.evaluate(c, S, 4000, cov);
        dec = ac.evaluate(c2, S, 4140, cov);
        check(dec.verdict == Verdict::Redundant, "min_per_bin 2: third -> redundant");
    }

    // ------------------------------------------------------------ gate 4
    printf("\ngate 4: settings\n");
    {
        AutoCapture ac(testConfig(), detector, kImage);
        Coverage cov(kImage);
        Detection a = project(board, 0, 0, 1200, {640, 512}, 0.03, 1);
        Detection a2 = project(board, 0, 0, 1200, {640, 512}, 0.03, 2);
        ac.evaluate(a, S, 0, cov);               // takes the reference

        for (int which = 0; which < 5; ++which) {
            Settings changed = S;
            const char* name[] = {"exposure", "gain", "intensity", "frame_rate", "ir_filter"};
            if (which == 0) changed.exposure -= 1000;
            if (which == 1) changed.gain = 0;
            if (which == 2) changed.intensity = 10;
            if (which == 3) changed.frameRate = 60;
            if (which == 4) changed.irFilter = false;
            Decision dec = ac.evaluate(a2, changed, 140 + which * 10, cov);
            check(dec.verdict == Verdict::SettingsChanged && !ac.settingsMatch(changed),
                  std::string(name[which]) + " changed -> settings_changed (" +
                  ac.settingsDiff(changed) + ")");
        }
        Settings thr = S;
        thr.threshold = 40;
        check(ac.settingsMatch(thr), "threshold is not one of the gated settings");

        Decision dec = ac.evaluate(a, S, 400, cov);
        check(dec.accepted(), "settings restored -> accept");

        // In gate order, an earlier failure is what is reported.
        Settings changed = S;
        changed.gain = 3;
        Detection partial = a;
        partial.corners.pop_back();
        partial.cornerIds.pop_back();
        check(ac.evaluate(partial, changed, 500, cov).verdict == Verdict::NotFull,
              "settings changed AND partial -> not_full (gate order)");
    }

    // ------------------------------------------------------------ gates 5, 6
    printf("\ngates 5-6: spacing\n");
    {
        AutoCapture ac(testConfig(), detector, kImage);
        Coverage cov(kImage);
        Detection a = project(board, 0, 0, 1200, {640, 512}, 0.03, 1);
        Detection a2 = project(board, 0, 0, 1200, {640, 512}, 0.03, 2);
        Detection a3 = project(board, 0, 0, 1200, {640, 512}, 0.03, 3);
        ac.evaluate(a, S, 0, cov);
        Decision dec = ac.evaluate(a2, S, 140, cov);
        ac.commit(dec, a2, cov);

        dec = ac.evaluate(a3, S, 280, cov);
        check(dec.verdict == Verdict::TooSoon, "140 ms after an accept -> too_soon");
        dec = ac.evaluate(a, S, 439, cov);
        check(dec.verdict == Verdict::TooSoon, "299 ms after -> too_soon");

        // Held still well past the interval: the same pose is not taken twice.
        dec = ac.evaluate(a3, S, 2000, cov);
        check(dec.verdict == Verdict::SamePose && dec.movePx < 1,
              "same pose 1.86 s later -> same_pose (moved " + std::to_string(dec.movePx) + " px)");

        Detection b = shifted(a, 25, 0), b2 = shifted(a2, 25, 0);
        ac.evaluate(b, S, 2140, cov);
        dec = ac.evaluate(b2, S, 2280, cov);
        check(dec.accepted() && dec.movePx >= 20,
              "moved 25 px, same bin (1 of 2) -> accept");
    }
    {
        AutoCapture ac(testConfig(), detector, kImage);
        Coverage cov(kImage);
        Detection a = project(board, 0, 0, 1200, {640, 512}, 0.03, 1);
        Detection a2 = project(board, 0, 0, 1200, {640, 512}, 0.03, 2);
        ac.evaluate(a, S, 0, cov);
        Decision dec = ac.evaluate(a2, S, 140, cov);
        ac.commit(dec, a2, cov);
        // A still frame of a new pose lands exactly at the interval.
        ac.evaluate(shifted(a2, 25, 0.1f), S, 300, cov);
        dec = ac.evaluate(shifted(a, 25, 0), S, 440, cov);
        check(dec.accepted(), "exactly min_interval_ms after the accept -> accept");
    }

    // ------------------------------------------------------------ pose bins
    printf("\npose bins\n");
    {
        AutoConfig cfg = testConfig();
        PoseBin p = poseBin(project(board, 0, 0, 1200), board, kImage, cfg);
        check(p.valid && p.tilt == Flat && p.tiltDeg < 1, "fronto-parallel -> flat");
        p = poseBin(project(board, 10, -10, 1200), board, kImage, cfg);
        check(p.tilt == Flat, "14 deg compound tilt -> flat (under flat_deg 20)");

        // Physical meaning, independent of the formula: positive tilt about y
        // means the board faces image-right, so its right edge is further
        // away and spans fewer pixels than its left edge.
        auto edgeSpans = [&](const Detection& d, bool vertical) {
            const int cx = board.squaresX - 1, cy = board.squaresY - 1;
            auto at = [&](int col, int row) { return d.corners[row * cx + col]; };
            if (vertical)   // left vs right column
                return std::make_pair(cv::norm(at(0, cy - 1) - at(0, 0)),
                                      cv::norm(at(cx - 1, cy - 1) - at(cx - 1, 0)));
            return std::make_pair(cv::norm(at(cx - 1, 0) - at(0, 0)),        // top vs bottom row
                                  cv::norm(at(cx - 1, cy - 1) - at(0, cy - 1)));
        };
        int quadrantsOk = 0;
        std::vector<int> seen;
        for (double rx : {30.0, -30.0})
            for (double ry : {30.0, -30.0}) {
                Detection d = project(board, rx, ry, 1200);
                PoseBin q = poseBin(d, board, kImage, cfg);
                auto [left, right] = edgeSpans(d, true);
                auto [top, bottom] = edgeSpans(d, false);
                bool facesRight = right < left, facesDown = bottom < top;
                bool ok = q.valid && q.tilt != Flat &&
                          (q.tiltYDeg > 0) == facesRight && (q.tiltXDeg > 0) == facesDown;
                printf("    rx %+3.0f ry %+3.0f -> %-5s tilt x %+5.1f y %+5.1f  (%s, %s)\n",
                       rx, ry, tiltBinName(q.tilt), q.tiltXDeg, q.tiltYDeg,
                       facesDown ? "faces down" : "faces up",
                       facesRight ? "faces right" : "faces left");
                quadrantsOk += ok;
                seen.push_back(q.tilt);
            }
        std::sort(seen.begin(), seen.end());
        check(quadrantsOk == 4 && std::unique(seen.begin(), seen.end()) == seen.end(),
              "four compound tilts -> four distinct quadrants, signs match the image");

        // Distance bands: 60 mm squares at 780 px focal length.
        double near = poseBin(project(board, 0, 0, 1000), board, kImage, cfg).squarePx;
        p = poseBin(project(board, 0, 0, 1000), board, kImage, cfg);
        check(p.band == Near && std::fabs(near - 46.8) < 0.5,
              "1.00 m -> near (" + std::to_string(near) + " px/sq, expect 46.8)");
        p = poseBin(project(board, 0, 0, 1250), board, kImage, cfg);
        check(p.band == Mid, "1.25 m -> mid (" + std::to_string(p.squarePx) + " px/sq)");
        p = poseBin(project(board, 0, 0, 1500), board, kImage, cfg);
        check(p.band == Far, "1.50 m -> far (" + std::to_string(p.squarePx) + " px/sq)");
        p = poseBin(project(board, 0, 35, 1250), board, kImage, cfg);
        check(p.band == Mid, "1.25 m tilted 35 deg about y stays mid (" +
                             std::to_string(p.squarePx) + " px/sq)");
        // Compound tilt shrinks BOTH board axes (by cos 25 each, to 33.9 px
        // here), which a per-axis spacing would read as far.
        for (double rx : {25.0, -25.0})
            for (double ry : {25.0, -25.0}) {
                p = poseBin(project(board, rx, ry, 1250), board, kImage, cfg);
                check(p.band == Mid && std::fabs(p.squarePx - 37.44) < 1.0,
                      cv::format("1.25 m tilted %+.0f/%+.0f stays mid (%.2f px/sq)",
                                 rx, ry, p.squarePx));
            }

        Detection partial = project(board, 0, 0, 1200);
        partial.corners.pop_back();
        partial.cornerIds.pop_back();
        check(!poseBin(partial, board, kImage, cfg).valid, "partial board -> no pose bin");
    }

    // ------------------------------------------------------------ end
    printf("\nsession end\n");
    {
        AutoConfig cfg = testConfig();
        cfg.minPerBin = 1;
        AutoCapture ac(cfg, detector, kImage);
        Coverage cov(kImage, 1, 1);          // one cell, so only bins decide
        check(!ac.complete(cov) && ac.binsShort() == 15, "fresh: 15 bins short, not complete");

        const double dist[] = {1000, 1250, 1500};
        const double tilt[][2] = {{0, 0}, {25, 25}, {25, -25}, {-25, 25}, {-25, -25}};
        double t = 0;
        int accepts = 0;
        for (int b = 0; b < 3; ++b)
            for (int k = 0; k < 5; ++k) {
                Detection d1 = project(board, tilt[k][0], tilt[k][1], dist[b], {640, 512}, 0.03, 10 + k);
                Detection d2 = project(board, tilt[k][0], tilt[k][1], dist[b], {640, 512}, 0.03, 20 + k);
                ac.evaluate(d1, S, t += 1000, cov);
                Decision dec = ac.evaluate(d2, S, t += 140, cov);
                if (dec.accepted()) { ac.commit(dec, d2, cov); ++accepts; }
                else printf("    %s %s: %s\n", bandName(b), tiltBinName(k),
                            dec.describe(cfg).c_str());
            }
        check(accepts == 15 && ac.complete(cov),
              "one still pose per bin -> 15 accepts, complete");
        // The first frame of each new pose is compared with the previous
        // pose: no reference only the very first time, moving after that.
        check(ac.tally(Verdict::Accept) == 15 && ac.tally(Verdict::NoReference) == 1 &&
              ac.tally(Verdict::Moving) == 14 && ac.evaluated() == 30,
              "tally counts every evaluated frame by verdict");

        bool threw = false;
        Decision rejected;
        try { ac.commit(rejected, Detection(), cov); } catch (const std::logic_error&) { threw = true; }
        check(threw, "commit of a rejected frame throws");
    }

    // ------------------------------------------------------------ session.json
    printf("\nsession.json\n");
    {
        fs::path root = fs::temp_directory_path() / ("panoptes_test_" + std::to_string(getpid()));
        AutoCapture ac(testConfig(), detector, kImage);
        Coverage cov(kImage);
        Detection a = project(board, 0, 0, 1200, {640, 512}, 0.03, 1);
        Detection a2 = project(board, 0, 0, 1200, {640, 512}, 0.03, 2);
        ac.evaluate(a, S, 0, cov);
        Decision dec = ac.evaluate(a2, S, 140, cov);
        ac.commit(dec, a2, cov);

        Session s(root.string(), 99999, board);
        cv::Mat gray(kImage, CV_8UC1, cv::Scalar(128));
        s.record(gray, a2, S, dec.jsonFields());
        s.record(gray, a2, S);                       // manual-style entry
        s.setExtra("autocapture", ac.summaryJson("  "));
        s.setExtra("autocapture_end", "\"operator quit (q)\"");
        s.setExtra("autocapture_end", "\"complete\"");   // replaces, not duplicates

        std::ifstream in(s.path() + "/session.json");
        std::stringstream ss;
        ss << in.rdbuf();
        std::string j = ss.str();
        auto count = [&](const std::string& k) {
            size_t n = 0, pos = 0;
            while ((pos = j.find(k, pos)) != std::string::npos) { ++n; pos += k.size(); }
            return n;
        };
        check(count("\"autocapture_end\"") == 1 && count("\"complete\"") == 1,
              "setExtra replaces a key");
        check(count("\"auto\": {") == 1 && count("\"still_px\"") == 1 &&
              count("\"verdicts\"") == 1, "frame reason, config and tally are written");
        check(count("{") == count("}") && count("[") == count("]"), "braces balance");
        printf("    written to %s (python json.load check is in RUNBOOK)\n", s.path().c_str());
        if (getenv("KEEP_TEST_SESSION") == nullptr) fs::remove_all(root);
    }

    printf("\ntest_autocapture: %d/%d passed\n", passed, passed + failed);
    return failed == 0 ? 0 : 1;
}
