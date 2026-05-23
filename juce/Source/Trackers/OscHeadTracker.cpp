#include "OscHeadTracker.h"

#include <chrono>

namespace {
int64_t nowMs() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}
}

OscHeadTracker& OscHeadTracker::shared()
{
    // Meyers singleton — one OSC receiver + atomics per process, shared
    // across all SpatialAudioProcessor instances the host creates.
    static OscHeadTracker s(9000);
    return s;
}

OscHeadTracker::OscHeadTracker(int port) : port_(port)
{
    watchdog_.owner = this;
}

OscHeadTracker::~OscHeadTracker()
{
    watchdog_.stopTimer();
    receiver_.removeListener(this);
    receiver_.disconnect();
}

void OscHeadTracker::start()
{
    bool expected = false;
    if (! started_.compare_exchange_strong(expected, true)) return;  // already started
    notify(Status::Connecting);
    if (! receiver_.connect(port_))
    {
        notify(Status::Failed);
        return;
    }
    receiver_.addListener(this, "/headpose");
    watchdog_.startTimer(500);
}

void OscHeadTracker::stop()
{
    // Shared backend stays alive while the host has any AU instance loaded.
    // We don't tear down on a single instance stop; the toggle in the
    // processor controls whether the pose is applied, not whether OSC runs.
}

Quat OscHeadTracker::latestPose(uint64_t* frameId) const noexcept
{
    if (frameId) *frameId = frameId_.load(std::memory_order_acquire);
    return { qw_.load(std::memory_order_relaxed),
             qx_.load(std::memory_order_relaxed),
             qy_.load(std::memory_order_relaxed),
             qz_.load(std::memory_order_relaxed) };
}

void OscHeadTracker::oscMessageReceived(const juce::OSCMessage& msg)
{
    if (msg.size() < 4) return;
    if (! (msg[0].isFloat32() && msg[1].isFloat32() &&
           msg[2].isFloat32() && msg[3].isFloat32()))
        return;
    qw_.store(msg[0].getFloat32(), std::memory_order_relaxed);
    qx_.store(msg[1].getFloat32(), std::memory_order_relaxed);
    qy_.store(msg[2].getFloat32(), std::memory_order_relaxed);
    qz_.store(msg[3].getFloat32(), std::memory_order_relaxed);
    frameId_.fetch_add(1, std::memory_order_release);
    lastRxMs_.store(nowMs(), std::memory_order_release);
    if (status_.load() != Status::Streaming) notify(Status::Streaming);
}

void OscHeadTracker::watchdogTick()
{
    // If no /headpose for 1s, consider the daemon offline.
    const int64_t last = lastRxMs_.load(std::memory_order_acquire);
    const Status  s    = status_.load();
    if (last == 0)
    {
        if (s != Status::Connecting) notify(Status::Connecting);
        return;
    }
    // 3 s tolerance: AU hosts schedule the message thread irregularly and
    // can stall UDP delivery briefly. We only want to flip to Disconnected
    // for a genuine daemon outage.
    if (nowMs() - last > 3000)
    {
        if (s == Status::Streaming) notify(Status::Disconnected);
    }
}

void OscHeadTracker::notify(Status s)
{
    auto prev = status_.exchange(s);
    if (prev == s) return;
    if (onStatusChange) onStatusChange(s);
}
