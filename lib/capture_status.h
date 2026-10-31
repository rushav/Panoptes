// Live status of an auto capture, as one JSON file rewritten every frame.
//
// 04_capture --status-file PATH writes it so another process (the arm
// probe, python/arm_capture.py) can see what the camera sees without
// sharing the camera. Read it with python/capture_status.py. The schema is
// in docs/ARCHITECTURE.md §5 and is versioned by the "schema" field: change
// that string whenever a field changes meaning or is removed.
//
// Atomic: the JSON goes to PATH.tmp, then rename() replaces PATH. On one
// filesystem a reader sees the old file or the new one, never half of one.

#pragma once

#include "autocapture.h"
#include "camera.h"
#include "coverage.h"
#include "detect.h"

#include <string>

namespace mocap {

inline const char* kStatusSchema = "panoptes.capture_status/1";

struct StatusMeta {
    long   frame = 0;          // frames evaluated, 1-based
    double tMs = 0;            // monotonic, since the capture started
    std::string utc;           // wall clock, ISO 8601 UTC with ms
    int    serial = 0;
    bool   dryRun = true;
    std::string sessionPath;   // capture's session folder; empty in a dry run
};

// "2026-10-28T18:03:12.345Z", now.
std::string utcNowIso();

std::string captureStatusJson(const StatusMeta& meta, const Settings& actual,
                              const Detection& d, const BoardSpec& board,
                              const Decision& dec, const AutoCapture& ac,
                              const Coverage& coverage);

// Write text to path atomically (path + ".tmp", then rename). Throws
// std::runtime_error naming the path on failure.
void writeFileAtomic(const std::string& path, const std::string& text);

}  // namespace mocap
