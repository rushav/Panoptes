// Frame groups: the plain records, the session-file writer and the
// statistics. No SDK types here, so tests build without the SDK; the SDK
// side (cModuleSync) is mocap::FrameSync in camera.h.
//
// Everything in a record is what the SDK reported for that frame or group.
// Nothing here knows, or claims, when a camera actually exposed.

#pragma once

#include "camera.h"

#include <cstdint>
#include <string>
#include <vector>

namespace mocap {

// One camera's frame inside a group.
struct SyncFrame {
    int serial = 0;
    int frameId = -1;            // Frame::FrameID()
    bool object = false;         // FrameType() == ObjectMode; blobs are read only then
    bool invalid = false;        // Frame::IsInvalid()
    double timeStamp = 0;        // Frame::TimeStamp(), seconds, SDK clock
    bool hwStampValid = false;   // Frame::IsHardwareTimeStamp()
    double hwTimeStamp = 0;      // Frame::HardwareTimeStampValue(), seconds
    uint64_t hwTicks = 0;        // Frame::HardwareTimeStamp()
    unsigned hwFreq = 0;         // Frame::HardwareTimeFreq()
    bool hwMaster = false;       // Frame::MasterTimingDevice()
    bool syncVideo = false;      // Frame::IsFromSynchronousVideoCamera()
    bool hwTiming = false;       // Frame::IsFromHardwareTimingCamera()
    std::vector<Blob> blobs;
};

// FrameGroup::Modes, by value.
enum { kSyncModeNone = 0, kSyncModeSoftware = 1, kSyncModeHardware = 2 };
const char* syncModeName(int mode);

struct SyncGroup {
    long index = -1;             // 0-based, counted by the recorder
    int64_t hostMonoNs = 0;      // CLOCK_MONOTONIC when the group was taken from the SDK
    int frameId = -1;            // FrameGroup::FrameID()
    int mode = kSyncModeNone;    // FrameGroup::Mode()
    double timeStamp = 0;        // FrameGroup::TimeStamp()
    double timeSpread = 0;       // FrameGroup::TimeSpread()
    double earliest = 0;         // FrameGroup::EarliestTimeStamp()
    double latest = 0;           // FrameGroup::LatestTimeStamp()
    std::vector<int> sdkDropped; // FrameGroup::DroppedFrames(), serials
    std::vector<SyncFrame> frames;
};

// One line of groups.jsonl (no trailing newline). Every serial in
// `expected` gets an entry, in that order: a camera with no frame in the
// group is written {"serial":N,"present":false}. A frame from a serial not
// in `expected` is appended with "unexpected":true.
std::string groupJson(const SyncGroup& g, const std::vector<int>& expected);

// Running statistics over the groups of one recording.
class SyncStats {
public:
    explicit SyncStats(std::vector<int> expected);

    void add(const SyncGroup& g);

    struct Dist {
        long n = 0;
        double min = 0, median = 0, p95 = 0, max = 0;
    };
    static Dist dist(std::vector<double> v);

    struct PerCamera {
        int serial = 0;
        long present = 0, missing = 0;
        long wrongMode = 0;          // present, but not an object-mode frame
        long sdkDropped = 0;         // times listed in FrameGroup::DroppedFrames()
        long idGaps = 0;             // steps > 1 between this camera's consecutive frames
        long idSkipped = 0;          // frame IDs jumped over, summed
        long idNotIncreasing = 0;    // steps <= 0
        long framesWithBlobs = 0, blobs = 0;
        int firstId = -1, lastId = -1;
        bool anyOffset = false;      // frame ID minus group frame ID, range seen
        int offsetMin = 0, offsetMax = 0;
        long hwValid = 0, syncVideo = 0, hwTiming = 0, hwMaster = 0;

        // Hardware stamps. Each camera stamps in its own clock, and the
        // clocks start seconds apart, so stamps are compared as this
        // camera's stamp minus the reference camera's in the same group.
        Dist hwIntervalMs;           // between this camera's consecutive frame IDs
        long hwPairs = 0;            // groups where it and the reference both had a stamp
        double hwOffsetMeanS = 0;    // mean of (this - reference)
        double hwOffsetRangeUs = 0;  // max - min of that difference over the run
        double hwOffsetSlopePpm = 0; // its least-squares drift against reference time
        double hwOffsetResidUs = 0;  // RMS about that line
    };

    struct Summary {
        long groups = 0, complete = 0;
        std::vector<long> byPresent;         // [k] = groups with k expected cameras present
        long unexpectedFrames = 0;
        long modeNone = 0, modeSoftware = 0, modeHardware = 0;
        double hostSpanS = 0;                // first to last group, host clock
        double hostRateHz = 0;               // (groups - 1) / hostSpanS
        double groupTsSpanS = 0;             // first to last FrameGroup::TimeStamp()
        double groupTsRateHz = 0;
        Dist hostIntervalMs, groupTsIntervalMs;
        long groupIdGaps = 0, groupIdSkipped = 0, groupIdNotIncreasing = 0;
        int groupIdFirst = -1, groupIdLast = -1;
        long idSpreadGroups = 0;             // groups with 2+ frames to compare
        long idSpreadNonZero = 0;
        int idSpreadMax = 0;
        Dist tsSpreadUs;                     // max - min of Frame::TimeStamp() in a group
        Dist sdkSpreadUs;                    // FrameGroup::TimeSpread()
        int hwReference = 0;                 // serial the hardware stamps are compared to; 0 = none
        std::vector<PerCamera> cameras;
    };
    Summary summary() const;

    // The printed block and the same numbers as a JSON object.
    std::string text(double requestedHz) const;
    std::string json(double requestedHz) const;

private:
    std::vector<int> expected_;
    std::vector<PerCamera> cams_;
    std::vector<long> byPresent_;
    long groups_ = 0, complete_ = 0, unexpected_ = 0;
    long mode_[3] = {0, 0, 0};
    int64_t firstNs_ = 0, lastNs_ = 0;
    double firstTs_ = 0, lastTs_ = 0;
    int firstGid_ = -1, lastGid_ = -1;
    long gidGaps_ = 0, gidSkipped_ = 0, gidNotInc_ = 0;
    long idSpreadGroups_ = 0, idSpreadNonZero_ = 0;
    int idSpreadMax_ = 0;
    std::vector<double> hostMs_, tsMs_, tsSpreadUs_, sdkSpreadUs_;
    int hwRef_ = -1;                                   // index into cams_, -1 until chosen
    std::vector<std::vector<double>> hwX_, hwD_;       // per camera: reference stamp, difference
    std::vector<std::vector<double>> hwIv_;            // per camera: stamp interval, ms
    std::vector<double> hwPrev_;                       // per camera: previous stamp
    std::vector<int> hwPrevId_;                        // and the frame ID it belonged to
};

}  // namespace mocap
