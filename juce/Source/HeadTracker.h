#pragma once

#include <cstdint>
#include <functional>

// Plain quaternion in the producer's native frame (Hamilton, scalar first).
// HeadPoseProcessor converts to native frame.
struct Quat
{
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
    static Quat identity() { return {1.0f, 0.0f, 0.0f, 0.0f}; }
};

// Abstract backend for any head-tracker (Buds 2 Pro now; AirPods / Supperware
// later). Producer-side: a background thread connects to hardware and
// publishes raw poses. Audio thread polls latestPose() once per block.
class HeadTracker
{
public:
    enum class Status { Disconnected, Connecting, Streaming, Failed };

    virtual ~HeadTracker() = default;

    virtual void start() = 0;
    virtual void stop()  = 0;

    virtual Status status() const noexcept = 0;

    // Last raw pose seen on the producer thread. `frameId` lets the
    // audio thread detect stalled streams (no new frames since N ms ago).
    virtual Quat   latestPose(uint64_t* frameId = nullptr) const noexcept = 0;

    // Fired on the JUCE message thread when status changes. May be empty.
    std::function<void(Status)> onStatusChange;
};
