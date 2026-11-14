// Frame groups from every camera through the SDK's cModuleSync, recorded
// and measured. Step 1 of extrinsics: before anything is solved from
// several cameras at once, find out what a "frame group" actually is on
// this rig.
//
// Opens every camera in object mode at config/mask.yaml's settings, applies
// the saved masks (config/masks/<serial>.json), and records each group to a
// session folder: groups.jsonl (one group per line) and session.json
// (provenance and the summary). Format: docs/ARCHITECTURE.md.
//
// What the summary can and cannot show: it reports how the SDK grouped
// frames and what clocks the SDK attached to them. It does not observe when
// any camera exposed. Frames sharing a group and a frame ID are frames the
// SDK matched, not frames shown to have been exposed together.
//
// Run from the repo root:
//   sync [--seconds N] [--label TEXT] [--delivery complete|favor-timely|timely]
//        [--cameras N] [--warmup S] [--config DIR] [--out DIR]

#include "../../lib/camera.h"
#include "../../lib/maskfile.h"
#include "../../lib/sync.h"

#include <opencv2/core.hpp>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using clk = std::chrono::steady_clock;

static volatile std::sig_atomic_t gStop = 0;
static void onSignal(int) { gStop = 1; }

static double secs(clk::time_point t0) {
    return std::chrono::duration<double>(clk::now() - t0).count();
}

static int64_t monoNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(clk::now().time_since_epoch()).count();
}

static std::string stamp(const char* fmt) {
    std::time_t t = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), fmt, std::localtime(&t));
    return buf;
}

static std::string quote(const std::string& s) {
    std::string o = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (static_cast<unsigned char>(c) < 0x20) o += ' ';
        else o += c;
    }
    return o + "\"";
}

struct Cam {
    std::unique_ptr<mocap::Camera> cam;
    int serial = 0;
    bool objectSynchronous = false;
    mocap::Settings readback;
    mocap::MaskFile mask;
    std::string maskPath;
    // Mode of the last frame actually RECEIVED while draining. modeIsObject()
    // alone describes the last poll, and a poll that got nothing reads as
    // "not object".
    bool sawFrame = false, lastWasObject = false;
};

struct Options {
    double seconds = 60, warmup = 2;
    std::string label, configDir = "config", outRoot;
    mocap::FrameSync::Delivery delivery = mocap::FrameSync::Delivery::Complete;
    int cameras = 10;
};

static int usage() {
    printf("usage: sync [--seconds N] [--label TEXT] [--delivery complete|favor-timely|timely]\n"
           "            [--cameras N] [--warmup S] [--config DIR] [--out DIR]\n"
           "  --seconds N   recording length, default 60\n"
           "  --label TEXT  goes into the folder name and session.json (letters, digits, - and _)\n"
           "  --delivery    cModuleSync optimisation, default complete\n"
           "  --cameras N   refuse to record unless exactly N cameras open, default 10\n"
           "  --warmup S    groups discarded before recording starts, default 2 s\n"
           "  --config DIR  default config (run from the repo root)\n"
           "  --out DIR     session root, default $HOME/panoptes-data/sessions\n");
    return 1;
}

static void drain(std::vector<Cam>& cams, int ms) {
    auto t0 = clk::now();
    while (secs(t0) * 1000 < ms) {
        for (auto& c : cams) {
            c.cam->grabBlobs();
            if (c.cam->gotFrame()) { c.sawFrame = true; c.lastWasObject = c.cam->modeIsObject(); }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

// session.json, whole, each time: once when recording starts ("running"),
// once at the end. Written to a temporary name and renamed into place.
static void writeSession(const fs::path& dir, const Options& o, const mocap::MaskConfig& cfg,
                         const std::vector<Cam>& cams, const mocap::FrameSync& sync,
                         const std::string& created, double unixAtStart, int64_t monoAtStart,
                         long warmupGroups, const std::string& ended,
                         const std::vector<std::pair<std::string, long>>& health, long healthPolls,
                         const mocap::SyncStats* stats) {
    std::string j = "{\n";
    j += " \"tool\": \"08_sync\",\n \"format\": 1,\n";
    j += " \"created\": " + quote(created) + ",\n";
    j += " \"label\": " + quote(o.label) + ",\n";
    j += " \"ended\": " + quote(ended) + ",\n";
    char buf[512];
    snprintf(buf, sizeof(buf), " \"host\": {\"unix_time_s\": %.6f, \"mono_ns\": %lld},\n",
             unixAtStart, (long long)monoAtStart);
    j += buf;
    j += " \"versions\": {\"camera_sdk\": " + quote(mocap::sdkVersion()) +
         ", \"opencv\": " + quote(CV_VERSION) + "},\n";
    auto settings = [&](const mocap::Settings& s) {
        snprintf(buf, sizeof(buf), "{\"exposure_us\": %d, \"gain\": %d, \"intensity\": %d, "
                 "\"threshold\": %d, \"frame_rate_hz\": %d}",
                 s.exposure, s.gain, s.intensity, s.threshold, s.frameRate);
        return std::string(buf);
    };
    j += " \"requested\": " + settings(cfg.camera) + ",\n";
    snprintf(buf, sizeof(buf), " \"sync\": {\"delivery_requested\": \"%s\", \"delivery_readback\": \"%s\", "
             "\"allow_incomplete_groups\": %s, \"timestamp_mode\": \"%s\", \"sdk_camera_count\": %d, "
             "\"sdk_delivery_rate\": %.3f},\n",
             mocap::FrameSync::deliveryName(o.delivery),
             mocap::FrameSync::deliveryName(sync.delivery()),
             sync.allowIncomplete() ? "true" : "false", sync.timeStampMode(), sync.cameraCount(),
             double(sync.deliveryRate()));
    j += buf;
    snprintf(buf, sizeof(buf), " \"duration_requested_s\": %.3f,\n \"warmup\": {\"seconds\": %.3f, "
             "\"groups_discarded\": %ld},\n", o.seconds, o.warmup, warmupGroups);
    j += buf;
    j += " \"cameras\": [\n";
    for (size_t i = 0; i < cams.size(); ++i) {
        auto& c = cams[i];
        j += "  {\"serial\": " + std::to_string(c.serial) + ", \"name\": " + quote(c.cam->name()) +
             ", \"object_mode_synchronous\": " + (c.objectSynchronous ? "true" : "false") +
             ", \"readback\": " + settings(c.readback) +
             ", \"mask\": {\"file\": " + quote(fs::path(c.maskPath).filename().string()) +
             ", \"learned_at\": " + quote(c.mask.learnedAt) +
             ", \"cells_set\": " + std::to_string(c.mask.cellsSet) + "}}" +
             (i + 1 < cams.size() ? ",\n" : "\n");
    }
    j += " ],\n";
    j += " \"health_polls\": " + std::to_string(healthPolls) + ",\n \"health\": [";
    for (size_t i = 0; i < health.size(); ++i)
        j += std::string(i ? ", " : "") + "{\"indicator\": " + quote(health[i].first) +
             ", \"polls_raised\": " + std::to_string(health[i].second) + "}";
    j += "],\n";
    j += " \"summary\": " + (stats ? stats->json(cfg.camera.frameRate) : std::string("null")) + "\n}\n";

    fs::path tmp = dir / "session.json.tmp";
    { std::ofstream f(tmp); f << j; if (!f) throw std::runtime_error("cannot write " + tmp.string()); }
    fs::rename(tmp, dir / "session.json");
}

int main(int argc, char** argv) {
    Options o;
    const char* home = getenv("HOME");
    o.outRoot = std::string(home ? home : ".") + "/panoptes-data/sessions";
    try {
        for (int i = 1; i < argc; ++i) {
            std::string a = argv[i];
            auto val = [&]() -> std::string {
                if (i + 1 >= argc) throw std::runtime_error(a + " needs a value");
                return argv[++i];
            };
            if (a == "--seconds") o.seconds = std::stod(val());
            else if (a == "--warmup") o.warmup = std::stod(val());
            else if (a == "--cameras") o.cameras = std::stoi(val());
            else if (a == "--label") o.label = val();
            else if (a == "--config") o.configDir = val();
            else if (a == "--out") o.outRoot = val();
            else if (a == "--delivery") {
                std::string d = val();
                if (d == "complete") o.delivery = mocap::FrameSync::Delivery::Complete;
                else if (d == "favor-timely") o.delivery = mocap::FrameSync::Delivery::FavorTimely;
                else if (d == "timely") o.delivery = mocap::FrameSync::Delivery::Timely;
                else throw std::runtime_error("--delivery " + d + ": not complete, favor-timely or timely");
            }
            else return usage();
        }
        if (o.seconds <= 0 || o.warmup < 0 || o.cameras < 1) return usage();
        for (char c : o.label)
            if (!isalnum(static_cast<unsigned char>(c)) && c != '-' && c != '_')
                throw std::runtime_error("--label: letters, digits, - and _ only");
    } catch (const std::exception& e) {
        printf("%s\n", e.what());
        return usage();
    }

    try {
        const mocap::MaskConfig cfg = mocap::loadMaskConfig(o.configDir);
        printf("settings (%s/mask.yaml): exposure %d us, gain %d, intensity %d, threshold %d, %d Hz\n",
               o.configDir.c_str(), cfg.camera.exposure, cfg.camera.gain, cfg.camera.intensity,
               cfg.camera.threshold, cfg.camera.frameRate);

        // ---- cameras ---------------------------------------------------
        printf("discovering ...\n");
        std::vector<Cam> cams;
        for (auto& info : mocap::discover()) {
            if (!info.initialized) { printf("  %d not initialized\n", info.serial); continue; }
            Cam c;
            c.serial = info.serial;
            c.cam = std::make_unique<mocap::Camera>(info.serial);
            cams.push_back(std::move(c));
        }
        std::sort(cams.begin(), cams.end(), [](const Cam& a, const Cam& b) { return a.serial < b.serial; });
        if (int(cams.size()) != o.cameras) {
            printf("REFUSED: %zu cameras opened, --cameras expects %d\n", cams.size(), o.cameras);
            return 2;
        }

        // Masks are read and checked before any camera is touched.
        bool masksOk = true;
        for (auto& c : cams) {
            c.maskPath = (fs::path(cfg.masksDir) / (std::to_string(c.serial) + ".json")).string();
            c.mask = mocap::loadMaskFile(c.maskPath);
            std::string why = c.mask.ok ? "" : c.mask.error;
            if (c.mask.ok && c.mask.serial != c.serial)
                why = "file is for serial " + std::to_string(c.mask.serial);
            // Rule 9: a mask is valid only at the settings it was learned under.
            if (why.empty()) {
                std::string diff = mocap::maskSettingsDiff(c.mask.settings, cfg.camera);
                if (!diff.empty()) why = "learned under other settings (" + diff + ")";
            }
            if (!why.empty()) { printf("  %d: mask REFUSED - %s\n", c.serial, why.c_str()); masksOk = false; }
        }
        if (!masksOk) { printf("REFUSED: every camera needs a usable mask (src/07_mask)\n"); return 2; }

        for (auto& c : cams) c.cam->start(cfg.camera);
        drain(cams, 1000);

        printf("\n  serial  mode        exposure gain intensity thresh  fps  object-mode-synchronous  mask\n");
        bool ready = true;
        for (auto& c : cams) {
            auto grid = c.cam->maskGrid();
            if (grid.cellPx != c.mask.cellPx || grid.cols != c.mask.cols || grid.rows != c.mask.rows) {
                printf("  %d: mask REFUSED - saved grid %dx%d/%dpx, camera %dx%d/%dpx\n", c.serial,
                       c.mask.cols, c.mask.rows, c.mask.cellPx, grid.cols, grid.rows, grid.cellPx);
                ready = false;
                continue;
            }
            c.cam->setMask(c.mask.cells);
            c.readback = c.cam->actual();
            c.objectSynchronous = c.cam->objectSynchronous();
            const char* mode = !c.sawFrame ? "NO FRAMES" : c.lastWasObject ? "object" : "WRONG MODE";
            printf("  %6d  %-10s  %8d %4d %9d %6d %4d  %-23s  %d cells, learned %s\n", c.serial, mode,
                   c.readback.exposure, c.readback.gain, c.readback.intensity, c.readback.threshold,
                   c.readback.frameRate, c.objectSynchronous ? "yes" : "NO", c.mask.cellsSet,
                   c.mask.learnedAt.c_str());
            std::string diff = mocap::maskSettingsDiff(cfg.camera, c.readback);
            if (!diff.empty()) printf("          READBACK DIFFERS from request: %s\n", diff.c_str());
            if (!c.sawFrame) ready = false;
        }
        if (!ready) { printf("REFUSED: a camera sent no frames or its mask does not fit\n"); return 2; }
        drain(cams, 1000);     // let the mask commits land; per-camera fetches stop here

        // ---- sync ------------------------------------------------------
        mocap::FrameSync sync;
        sync.setDelivery(o.delivery);
        sync.setAllowIncomplete(true);
        for (auto& c : cams) sync.add(*c.cam);
        printf("\ncModuleSync: %d cameras, delivery %s (asked %s), incomplete groups %s, timestamps %s\n",
               sync.cameraCount(), mocap::FrameSync::deliveryName(sync.delivery()),
               mocap::FrameSync::deliveryName(o.delivery),
               sync.allowIncomplete() ? "allowed" : "NOT allowed", sync.timeStampMode());

        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);

        mocap::SyncGroup g;
        long warmupGroups = 0;
        for (auto t0 = clk::now(); secs(t0) < o.warmup && !gStop;)
            if (sync.next(g, 20)) ++warmupGroups;
        printf("warm-up: %ld groups discarded in %.1f s\n", warmupGroups, o.warmup);
        if (warmupGroups == 0)
            printf("  NO GROUPS during warm-up; recording anyway so the absence is on record\n");

        std::vector<int> serials;
        for (auto& c : cams) serials.push_back(c.serial);

        const std::string created = stamp("%Y-%m-%dT%H:%M:%S%z");
        const fs::path dir = fs::path(o.outRoot) /
            ("sync_" + (o.label.empty() ? "" : o.label + "_") + stamp("%Y%m%d_%H%M%S"));
        fs::create_directories(dir);
        std::ofstream out(dir / "groups.jsonl");
        if (!out) throw std::runtime_error("cannot write " + (dir / "groups.jsonl").string());

        mocap::SyncStats stats(serials);
        std::vector<std::pair<std::string, long>> health;
        for (auto& h : sync.health()) health.emplace_back(h.first, 0);
        long healthPolls = 0;

        const int64_t monoAtStart = monoNs();
        const double unixAtStart = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        writeSession(dir, o, cfg, cams, sync, created, unixAtStart, monoAtStart, warmupGroups,
                     "running", health, healthPolls, nullptr);

        printf("recording %.0f s to %s\n", o.seconds, dir.c_str());
        auto t0 = clk::now();
        long index = 0;
        int shown = 0;
        double nextHealth = 1.0;
        for (double t; (t = secs(t0)) < o.seconds && !gStop;) {
            if (sync.next(g, 20)) {
                g.index = index++;
                out << mocap::groupJson(g, serials) << '\n';
                stats.add(g);
            }
            if (t >= nextHealth) {
                nextHealth += 1.0;
                ++healthPolls;
                auto h = sync.health();
                for (size_t i = 0; i < h.size() && i < health.size(); ++i)
                    if (h[i].second) ++health[i].second;
                out.flush();
            }
            if (int(t) / 5 > shown) {
                shown = int(t) / 5;
                auto s = stats.summary();
                printf("  %3d s  %ld groups, %ld complete\n", shown * 5, s.groups, s.complete);
                fflush(stdout);
            }
        }
        out.close();
        if (!out) throw std::runtime_error("write to groups.jsonl failed");
        const std::string ended = gStop ? "interrupted" : "duration";

        writeSession(dir, o, cfg, cams, sync, created, unixAtStart, monoAtStart, warmupGroups,
                     ended, health, healthPolls, &stats);

        // ---- summary ---------------------------------------------------
        printf("\n=== frame sync summary (%s; ended: %s) ===\n",
               o.label.empty() ? "no label" : o.label.c_str(), ended.c_str());
        printf("  IsVideoTypeSynchronous(ObjectMode):");
        for (auto& c : cams) printf(" %d:%s", c.serial, c.objectSynchronous ? "yes" : "NO");
        printf("\n%s", stats.text(cfg.camera.frameRate).c_str());
        printf("\n  SDK FrameDeliveryRate() at the end: %.3f\n", double(sync.deliveryRate()));
        printf("  SDK health indicators, polls raised of %ld (once a second):\n", healthPolls);
        for (auto& h : health) printf("    %-44s %ld\n", h.first.c_str(), h.second);
        printf("\n  These numbers describe how the SDK grouped and stamped frames. They do not\n"
               "  measure when any camera exposed.\n");
        printf("\nwrote %s/{groups.jsonl,session.json}\n", dir.c_str());
        return 0;
    } catch (const std::exception& e) {
        printf("error: %s\n", e.what());
        return 1;
    }
}
